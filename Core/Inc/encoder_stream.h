/**
  ******************************************************************************
  * @file    encoder_stream.h
  * @brief   Continuous streaming of the 6 quadrature encoders over LPUART1 DMA.
  ******************************************************************************
  */
#ifndef __ENCODER_STREAM_H__
#define __ENCODER_STREAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Number of encoders. Frame order is motor order ENC1..ENC6, which maps to
   the timers as: ENC1=TIM3, ENC2=TIM4, ENC3=TIM20, ENC4=TIM1, ENC5=TIM2,
   ENC6=TIM8. Index 0 of every array below is ENC1. */
#define ENC_COUNT               6u

/* Frame period in milliseconds. 2 ms -> 500 Hz.
   ASCII frames are at most 81 bytes; at 921600 baud (92160 byte/s) one frame
   takes ~0.88 ms, so 2 ms keeps the link below 50% and leaves headroom.
   Switch to ENC_STREAM_BINARY for 1 ms (or faster) frames. */
#define ENC_STREAM_PERIOD_MS    2u

/* 0 = ASCII, NMEA style:  <seq,e0,e1,e2,e3,e4,e5*XX\r\n
       '<'    frame start, the only place this byte may appear
       seq    frame counter, 0..255 rolling
       eN     wrap corrected position of ENC(N+1), signed decimal
       *XX    XOR of every byte between '<' and '*', two upper case hex digits
   1 = binary, fixed 28 bytes:
       0xAA 0x55 | seq u8 | 6 x int32 LE | XOR of the 25 bytes after the sync */
#define ENC_STREAM_BINARY       0

/* Live state, deliberately global so STM32CubeIDE Live Expressions can watch
   them from any context: type "EncPos", "EncSeq" or "EncDrops" in the view. */
extern volatile int32_t  EncPos[ENC_COUNT];  /* EncPos[0] is ENC1, [5] is ENC6*/
extern volatile uint8_t  EncSeq;             /* sequence of the next frame    */
extern volatile uint32_t EncDrops;           /* frames skipped, should stay 0 */

/* Start the encoder timers and the stream time base. Call once after the
   MX_*_Init() functions. */
void EncStream_Init(void);

/* Sample the encoders and kick off a DMA frame when it is due.
   Never blocks; call it as often as possible from the main loop. */
void EncStream_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_STREAM_H__ */
