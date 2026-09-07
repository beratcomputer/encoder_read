#!/usr/bin/env python3
"""
Read the 6 encoder stream produced by encoder_stream.c and print it live.

Frame format (ENC_STREAM_BINARY = 0), one per line:

    <seq,e0,e1,e2,e3,e4,e5*XX\r\n

    '<'    frame start, appears nowhere else, so it is a resync anchor
    seq    frame counter, 0..255 rolling
    eN     wrap corrected position of encoder N, signed decimal
    *XX    XOR of every byte between '<' and '*', two hex digits

Usage:
    python read_encoders.py                     # auto detect the ST-Link port
    python read_encoders.py -p COM5             # Windows
    python read_encoders.py -p /dev/ttyACM0     # Linux
    python read_encoders.py --raw               # dump lines as they arrive
    python read_encoders.py --record dump.bin   # save the raw stream too
    python read_encoders.py --replay dump.bin   # parse a saved stream offline

Needs pyserial:  pip install pyserial
On Linux you normally also need to be in the 'dialout' group:
    sudo usermod -aG dialout $USER      (log out and back in)
"""

import argparse
import sys
import time

BAUD = 921600
ENC_COUNT = 6
TIMER_NAMES = ["TIM1", "TIM2", "TIM3", "TIM4", "TIM8", "TIM20"]


class Frame:
    __slots__ = ("seq", "pos", "t")

    def __init__(self, seq, pos, t):
        self.seq = seq      # 0..255
        self.pos = pos      # list of 6 ints
        self.t = t          # host timestamp, seconds


class FrameParser:
    """Turns a byte stream into frames. Tolerates being fed arbitrary chunks:
    a frame split across two reads is reassembled, garbage is skipped."""

    MAX_BUF = 4096          # a frame is at most 81 bytes; beyond this we are lost

    def __init__(self):
        self._buf = bytearray()
        self._last_seq = None
        self.frames = 0         # frames accepted
        self.bad_crc = 0        # checksum mismatch
        self.bad_fmt = 0        # could not be parsed at all
        self.lost = 0           # frames missing according to the sequence counter

    def feed(self, data):
        """Feed raw bytes, get back a list of Frame objects."""
        out = []
        self._buf.extend(data)

        while True:
            nl = self._buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(self._buf[:nl])
            del self._buf[: nl + 1]
            frame = self._parse(line)
            if frame is not None:
                out.append(frame)

        # No terminator in sight: the line is junk, keep only a short tail.
        if len(self._buf) > self.MAX_BUF:
            del self._buf[:-128]

        return out

    def _parse(self, line):
        line = line.rstrip(b"\r")

        # Resync: if the reader started mid frame there may be leftovers in
        # front of the real start byte. The last '<' is the true beginning.
        start = line.rfind(b"<")
        if start < 0:
            self.bad_fmt += 1
            return None
        line = line[start:]

        # The checksum is always the last three bytes: '*' plus two hex digits.
        if len(line) < 5 or line[-3:-2] != b"*":
            self.bad_fmt += 1
            return None
        body = line[1:-3]

        want = 0
        for b in body:
            want ^= b
        try:
            got = int(line[-2:], 16)
        except ValueError:
            self.bad_fmt += 1
            return None
        if got != want:
            self.bad_crc += 1
            return None

        fields = body.split(b",")
        if len(fields) != ENC_COUNT + 1:
            self.bad_fmt += 1
            return None
        try:
            nums = [int(f) for f in fields]
        except ValueError:
            self.bad_fmt += 1
            return None

        seq, pos = nums[0], nums[1:]

        # The sequence counter is a rolling byte, so the gap has to be taken
        # modulo 256. A gap of 0 would mean 256 lost frames in a row, which we
        # cannot tell apart from no loss at all; treat it as no loss.
        if self._last_seq is not None:
            self.lost += (seq - self._last_seq - 1) & 0xFF
        self._last_seq = seq

        self.frames += 1
        return Frame(seq, pos, time.monotonic())


def enable_ansi():
    """Windows consoles need VT sequences switched on explicitly. No-op on
    Linux and macOS, where they are always available."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        k = ctypes.windll.kernel32
        k.SetConsoleMode(k.GetStdHandle(-11), 7)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass


def find_port():
    """Pick the ST-Link virtual COM port. VID 0x0483 is STMicroelectronics."""
    from serial.tools import list_ports

    candidates = list(list_ports.comports())
    for p in candidates:
        if p.vid == 0x0483:
            return p.device
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        sys.exit("No serial port found. Is the Nucleo plugged in?")
    listing = "\n".join("  %s  %s" % (p.device, p.description) for p in candidates)
    sys.exit("Could not tell which port is the Nucleo. Pick one with -p:\n" + listing)


class Display:
    """Redraws a fixed block of lines in place instead of scrolling.

    The firmware sends 500 frames per second. Printing every one of them would
    just flood the terminal, so every frame is parsed but the screen is only
    repainted REFRESH_HZ times per second with the newest values.
    """

    REFRESH_HZ = 20
    LINES = ENC_COUNT + 6

    def __init__(self, port, live=True):
        self.port = port
        self.live = live
        self.first = True
        self.next_draw = 0.0
        self.prev = None            # previous drawn frame, for the rate column
        self.start = time.monotonic()

    def maybe_draw(self, frame, parser):
        now = time.monotonic()
        if now < self.next_draw:
            return
        self.next_draw = now + 1.0 / self.REFRESH_HZ
        self.draw(frame, parser)

    def draw(self, frame, parser):
        rates = [0.0] * ENC_COUNT
        if self.prev is not None:
            dt = frame.t - self.prev.t
            if dt > 0:
                rates = [(c - p) / dt for c, p in zip(frame.pos, self.prev.pos)]
        self.prev = frame

        elapsed = time.monotonic() - self.start
        fps = parser.frames / elapsed if elapsed > 0 else 0.0

        rows = []
        if self.live:
            rows.append("Encoder stream  %s @ %d baud" % (self.port, BAUD))
            rows.append(
                "frames %-10d %7.1f f/s    lost %-6d bad crc %-6d bad fmt %-6d"
                % (parser.frames, fps, parser.lost, parser.bad_crc, parser.bad_fmt)
            )
        else:
            # Replaying a file has no real time base, so f/s and counts/s
            # would be meaningless here. Show only what is actually known.
            rows.append("Replay of %s" % self.port)
            rows.append(
                "frames %-10d %7s        lost %-6d bad crc %-6d bad fmt %-6d"
                % (parser.frames, "-", parser.lost, parser.bad_crc, parser.bad_fmt)
            )
        rows.append("")
        rows.append("  %-3s%-8s%14s%14s" % ("#", "timer", "position", "counts/s"))
        rows.append("  " + "-" * 39)
        for i in range(ENC_COUNT):
            rate = ("%14.1f" % rates[i]) if self.live else "%14s" % "-"
            rows.append("  %-3d%-8s%14d%s" % (i, TIMER_NAMES[i], frame.pos[i], rate))
        rows.append("")
        rows.append("Ctrl-C to stop")

        if self.first:
            self.first = False
        else:
            sys.stdout.write("\033[%dA" % self.LINES)
        # \033[K clears to end of line so shorter values leave no debris behind.
        sys.stdout.write("".join(r + "\033[K\n" for r in rows))
        sys.stdout.flush()


def run(source, port_label, args):
    """source is any object with .read(n) returning bytes (possibly empty)."""
    parser = FrameParser()
    display = Display(port_label, live=not args.replay)
    last = None

    while True:
        data = source.read(4096)
        if not data:
            if args.replay:
                break
            continue
        if args.record_fh:
            args.record_fh.write(data)

        for frame in parser.feed(data):
            last = frame
            if args.raw:
                print("seq %3d %s" % (frame.seq, " ".join("%11d" % v for v in frame.pos)))
            else:
                display.maybe_draw(frame, parser)

    if args.replay and last is not None and not args.raw:
        display.draw(last, parser)
    return parser


def main():
    ap = argparse.ArgumentParser(description="Live view of the 6 encoder stream.")
    ap.add_argument("-p", "--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=BAUD)
    ap.add_argument("--raw", action="store_true",
                    help="print every frame instead of a live table")
    ap.add_argument("--record", metavar="FILE", help="also write the raw stream to FILE")
    ap.add_argument("--replay", metavar="FILE",
                    help="parse FILE instead of reading a serial port")
    args = ap.parse_args()
    args.record_fh = None

    enable_ansi()

    if args.record:
        args.record_fh = open(args.record, "wb")

    try:
        if args.replay:
            with open(args.replay, "rb") as fh:
                parser = run(fh, args.replay, args)
        else:
            import serial

            port = args.port or find_port()
            # A timeout keeps read() from blocking forever, so Ctrl-C stays live.
            with serial.Serial(port, args.baud, timeout=0.05) as ser:
                ser.reset_input_buffer()
                parser = run(ser, port, args)
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    finally:
        if args.record_fh:
            args.record_fh.close()

    print("\n%d frames, %d lost, %d bad checksum, %d unparsable"
          % (parser.frames, parser.lost, parser.bad_crc, parser.bad_fmt))
    return 0


if __name__ == "__main__":
    sys.exit(main())
