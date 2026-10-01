#!/usr/bin/env python3
"""Pull one-shot IQ snapshots from an ESP32-S3 running the eSpDR snapshot
firmware over its USB Serial/JTAG port, and write them as int16 IQ for
blue-dragon (--format ci16).

eSpDR samples use the LO-minus-RF convention and 10-bit values, so each pair
is conjugated and scaled by 64 before writing. Snapshots are separated by
zeros so a burst detector resets at every seam.
"""
import argparse
import struct
import sys
import time
import zlib

import serial

REQ_MAGIC, RSP_MAGIC = 0xB4, 0xB5
CTL_INFO, CTL_STATUS = 1, 3
ESP_ARG_HIGH, ESP_SET_LO, ESP_SET_RATE, ESP_SET_WIDTH, ESP_SET_GAIN = 19, 20, 21, 22, 24
ESP_SET_DC = 27
ESP_SNAPSHOT = 40
ESP_STAT_RADIO, ESP_STAT_LO_HZ, ESP_STAT_RATE, ESP_STAT_PLL = 13, 15, 16, 28
SNAP_MAGIC = 0x50414E53
STATUS = {0: "OK", 1: "UNKNOWN_OP", 2: "BAD_ARGUMENT", 3: "BUSY",
          4: "NOT_READY", 5: "RUN_FAILED", 6: "FAILED"}


class Esp:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=2, write_timeout=2)
        self.seq = 0
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def _read(self, n):
        data = self.s.read(n)
        if len(data) != n:
            raise IOError(f"short read: {len(data)} of {n} bytes")
        return data

    def cmd(self, op, arg=0):
        self.seq = (self.seq + 1) & 0xFFFF
        head = struct.pack("<BBHH", REQ_MAGIC, op, arg & 0xFFFF, self.seq)
        self.s.write(head + struct.pack("<I", zlib.crc32(head)))
        rsp = self._read(16)
        magic, node, rop, status, seq = struct.unpack_from("<BBBBH", rsp)
        value, crc = struct.unpack_from("<II", rsp, 8)
        if magic != RSP_MAGIC or zlib.crc32(rsp[:12]) != crc or rop != op or seq != self.seq:
            raise IOError(f"bad reply to op {op}: {rsp.hex()}")
        if status:
            raise IOError(f"op {op} failed: {STATUS.get(status, status)}")
        return value

    def cmd32(self, op, arg):
        self.cmd(ESP_ARG_HIGH, arg >> 16)
        return self.cmd(op, arg & 0xFFFF)

    def snapshot(self):
        pairs = self.cmd(ESP_SNAPSHOT)
        magic, count, rate, lo = struct.unpack("<IIII", self._read(16))
        if magic != SNAP_MAGIC or count != pairs:
            raise IOError(f"bad snapshot header {magic:#x} {count}")
        raw = self._read(count * 4)
        (crc,) = struct.unpack("<I", self._read(4))
        if zlib.crc32(raw) != crc:
            raise IOError("snapshot CRC mismatch")
        return raw, rate, lo


def to_ci16(raw, scale=64):
    """Unpack 10-bit I/Q words, conjugate and scale to interleaved int16."""
    import numpy as np
    w = np.frombuffer(raw, dtype="<u4")
    i = ((w & 1023) ^ 512).astype(np.int32) - 512
    q = (((w >> 10) & 1023) ^ 512).astype(np.int32) - 512
    out = np.empty(2 * len(w), dtype="<i2")
    out[0::2] = np.clip(i * scale, -32768, 32767)
    out[1::2] = np.clip(-q * scale, -32768, 32767)  # conjugate: LO-RF -> RF-LO
    return out, i, q


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--lo-mhz", type=float, default=2426.0)
    ap.add_argument("--rate", type=int, choices=(16, 80), default=16)
    ap.add_argument("--gain", type=int, default=48)
    ap.add_argument("--count", type=int, default=20, help="snapshots to take")
    ap.add_argument("--gap", type=int, default=4000, help="zero pairs between snapshots")
    ap.add_argument("--out", default="/tmp/espdr_snaps.ci16")
    args = ap.parse_args()

    import numpy as np
    esp = Esp(args.port)
    fw = esp.cmd(CTL_INFO, 0)
    print(f"firmware id {fw:#010x}, radio status {esp.cmd(CTL_STATUS, ESP_STAT_RADIO)}")
    esp.cmd(ESP_SET_RATE, 1 if args.rate == 16 else 0)
    esp.cmd(ESP_SET_WIDTH, 20 if args.rate == 16 else 40)
    esp.cmd(ESP_SET_GAIN, args.gain)
    lo = esp.cmd32(ESP_SET_LO, int(round(args.lo_mhz * 1e6)))
    print(f"LO {lo} Hz, rate {args.rate} Msps, gain {args.gain}, "
          f"pll {esp.cmd(CTL_STATUS, ESP_STAT_PLL):#x}")

    gap = np.zeros(2 * args.gap, dtype="<i2")
    t0 = time.time()
    with open(args.out, "wb") as f:
        for n in range(args.count):
            raw, rate, lo_hz = esp.snapshot()
            iq, i, q = to_ci16(raw)
            f.write(iq.tobytes())
            f.write(gap.tobytes())
            mag = np.sqrt(i.astype(float) ** 2 + q.astype(float) ** 2)
            print(f"snap {n:3d}: {len(i)} pairs  rms {mag.mean():6.1f}  peak {mag.max():6.1f}  "
                  f"dc ({i.mean():+6.1f},{q.mean():+6.1f})  rail {(np.abs(i) >= 511).sum() + (np.abs(q) >= 511).sum()}")
    dt = time.time() - t0
    print(f"{args.count} snapshots in {dt:.1f}s -> {args.out}")
    print(f"blue-dragon: --file {args.out} --format ci16 --sample-rate {args.rate}000000 "
          f"-c {int(round(lo / 1e6))} -C {args.rate}")


if __name__ == "__main__":
    sys.exit(main())
