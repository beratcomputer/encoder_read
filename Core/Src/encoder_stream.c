/**
  ******************************************************************************
  * @file    encoder_stream.c
  * @brief   Continuous streaming of the 6 quadrature encoders over LPUART1 DMA.
  *
  *          The counters are sampled every ENC_STREAM_PERIOD_MS and unwrapped
  *          into 32 bit positions, so a value survives the hardware counter
  *          rolling over. The frame is then handed to the LPUART1 TX DMA
  *          channel; nothing in this file waits on the UART.
  ******************************************************************************
  */

#include "encoder_stream.h"
#include "tim.h"
#include "usart.h"

/* Worst case ASCII frame: '<' + 3 digit seq + 6 x ",-2147483648" + "*XX\r\n". */
#if ENC_STREAM_BINARY
#define ENC_FRAME_MAX   (2u + 1u + (ENC_COUNT * 4u) + 1u)
#else
#define ENC_FRAME_MAX   96u
#endif

volatile int32_t  EncPos[ENC_COUNT];
volatile uint8_t  EncSeq;
volatile uint32_t EncDrops;

/* Frame layout, in motor order ENC1..ENC6, following how the encoders are
   physically wired. The timer and its counter width are stored together so
   the two can never drift apart if this order is changed again.
   TIM2 is the only 32 bit counter of the six; the others wrap at 16 bits. */
typedef struct
{
  TIM_HandleTypeDef *tim;
  uint8_t            is_32bit;
} EncChannel;

static const EncChannel enc_ch[ENC_COUNT] = {
  { &htim3,  0u },   /* ENC1 */
  { &htim4,  0u },   /* ENC2 */
  { &htim20, 0u },   /* ENC3 */
  { &htim1,  0u },   /* ENC4 */
  { &htim2,  1u },   /* ENC5 */
  { &htim8,  0u },   /* ENC6 */
};

static uint32_t enc_prev[ENC_COUNT];
static uint32_t enc_next_tick;

/* Double buffered: one half is filled while the DMA is reading the other. */
static uint8_t  enc_tx_buf[2][ENC_FRAME_MAX];
static uint8_t  enc_tx_idx;

/* -------------------------------------------------------------------------- */

static void EncStream_Sample(void)
{
  for (uint32_t i = 0u; i < ENC_COUNT; i++)
  {
    uint32_t now = __HAL_TIM_GET_COUNTER(enc_ch[i].tim);
    int32_t  delta;

    if (enc_ch[i].is_32bit != 0u)
    {
      delta = (int32_t)(now - enc_prev[i]);
    }
    else
    {
      delta = (int16_t)((uint16_t)now - (uint16_t)enc_prev[i]);
    }

    enc_prev[i] = now;
    EncPos[i]  += delta;
  }
}

static uint8_t EncStream_Xor(const uint8_t *data, uint32_t len)
{
  uint8_t x = 0u;

  for (uint32_t i = 0u; i < len; i++)
  {
    x = (uint8_t)(x ^ data[i]);
  }
  return x;
}

#if ENC_STREAM_BINARY

static uint32_t EncStream_BuildFrame(uint8_t *dst)
{
  uint32_t n = 0u;

  dst[n++] = 0xAAu;
  dst[n++] = 0x55u;

  uint32_t start = n;
  dst[n++] = EncSeq;

  for (uint32_t i = 0u; i < ENC_COUNT; i++)
  {
    uint32_t v = (uint32_t)EncPos[i];
    for (uint32_t b = 0u; b < 4u; b++)
    {
      dst[n++] = (uint8_t)(v >> (8u * b));
    }
  }

  dst[n] = EncStream_Xor(&dst[start], n - start);
  n++;

  return n;
}

#else /* ASCII */

static uint32_t EncStream_PutU32(uint8_t *dst, uint32_t v)
{
  uint8_t  tmp[10];
  uint32_t n = 0u;

  do
  {
    tmp[n++] = (uint8_t)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u);

  for (uint32_t i = 0u; i < n; i++)
  {
    dst[i] = tmp[n - 1u - i];
  }
  return n;
}

static uint32_t EncStream_PutI32(uint8_t *dst, int32_t v)
{
  uint32_t n = 0u;
  uint32_t mag;

  if (v < 0)
  {
    dst[n++] = (uint8_t)'-';
    mag = 0u - (uint32_t)v;   /* well defined for INT32_MIN as well */
  }
  else
  {
    mag = (uint32_t)v;
  }

  return n + EncStream_PutU32(&dst[n], mag);
}

static uint32_t EncStream_BuildFrame(uint8_t *dst)
{
  static const char hex[] = "0123456789ABCDEF";
  uint32_t n = 0u;

  dst[n++] = (uint8_t)'<';

  uint32_t start = n;
  n += EncStream_PutU32(&dst[n], (uint32_t)EncSeq);

  for (uint32_t i = 0u; i < ENC_COUNT; i++)
  {
    dst[n++] = (uint8_t)',';
    n += EncStream_PutI32(&dst[n], EncPos[i]);
  }

  uint8_t x = EncStream_Xor(&dst[start], n - start);

  dst[n++] = (uint8_t)'*';
  dst[n++] = (uint8_t)hex[(x >> 4) & 0x0Fu];
  dst[n++] = (uint8_t)hex[x & 0x0Fu];
  dst[n++] = (uint8_t)'\r';
  dst[n++] = (uint8_t)'\n';

  return n;
}

#endif /* ENC_STREAM_BINARY */

/* -------------------------------------------------------------------------- */

void EncStream_Init(void)
{
  for (uint32_t i = 0u; i < ENC_COUNT; i++)
  {
    if (HAL_TIM_Encoder_Start(enc_ch[i].tim, TIM_CHANNEL_ALL) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(enc_ch[i].tim, 0u);
    enc_prev[i] = 0u;
    EncPos[i]   = 0;
  }

  EncSeq        = 0u;
  EncDrops      = 0u;
  enc_tx_idx    = 0u;
  enc_next_tick = HAL_GetTick() + ENC_STREAM_PERIOD_MS;
}

void EncStream_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((int32_t)(now - enc_next_tick) < 0)
  {
    return;
  }

  enc_next_tick += ENC_STREAM_PERIOD_MS;
  if ((int32_t)(now - enc_next_tick) >= 0)
  {
    /* The loop stalled for more than one period, typically a debugger halt;
       resync instead of catching up with a burst of back to back frames. */
    enc_next_tick = now + ENC_STREAM_PERIOD_MS;
  }

  /* Sampling happens even when the previous frame is still in flight, so a
     dropped frame never costs encoder counts. */
  EncStream_Sample();

  if (hlpuart1.gState != HAL_UART_STATE_READY)
  {
    EncDrops++;
    return;
  }

  uint8_t *buf = enc_tx_buf[enc_tx_idx];
  uint32_t len = EncStream_BuildFrame(buf);

  if (HAL_UART_Transmit_DMA(&hlpuart1, buf, (uint16_t)len) == HAL_OK)
  {
    enc_tx_idx ^= 1u;
    EncSeq++;
  }
  else
  {
    EncDrops++;
  }
}
