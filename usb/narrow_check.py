#!/usr/bin/env python3
"""Check the ESP's channel filter (esp32s3/src/narrow*) against a model.

ESP_NARROW_TEST takes a snapshot, runs the filter over its oldest 4096 pairs
for channel offset k, and sends those pairs and the 1024 outputs. The same
integer arithmetic is repeated here; the outputs must match exactly.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import snap  # noqa: E402

ESP_NARROW_TEST = 42
PAIRS = 4096

# Q15 cos(2 pi m / 16) and the Q14 low-pass, as in narrow.c.
UNIT = np.array([32767, 30274, 23170, 12540, 0, -12540, -23170, -30274,
                 -32767, -30274, -23170, -12540, 0, 12540, 23170, 30274], np.int64)
TAPS = np.array([-27, 62, 476, 1428, 2676, 3577, 3577, 2676, 1428, 476, 62, -27], np.int64)


def signed10(v):
    return ((v & 1023) ^ 512).astype(np.int64) - 512


def model(words, k):
    i, q = signed10(words), signed10(words >> 10)
    m = (k * np.arange(len(words))) & 15
    c, s = UNIT[m], UNIT[(m + 12) & 15]
    # Vector multiplies keep the top half of the 32-bit product (floor), and
    # the adds saturate to 16 bits.
    mul = lambda a, b: (a * 64 * b) >> 16
    sat = lambda v: np.clip(v, -32768, 32767)
    mi = np.concatenate([np.zeros(8, np.int64), sat(mul(i, c) + mul(q, s))])
    mq = np.concatenate([np.zeros(8, np.int64), sat(mul(q, c) - mul(i, s))])
    out = []
    for end in range(3 + 8, len(words) + 8, 4):  # twelve pairs ending at n + 3, n + 7, ...
        window = slice(end - 11, end + 1)
        for v in (mi, mq):
            acc = int(np.dot(v[window], TAPS[::-1]))
            out.append(max(-512, min(511, (acc + (1 << 18)) >> 19)))
    return np.array(out).reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--lo-mhz", type=float, default=2426.0)
    ap.add_argument("--gain", type=int, default=28)
    args = ap.parse_args()

    esp = snap.Esp(args.port)
    esp.cmd(snap.ESP_SET_RATE, 1)
    esp.cmd(snap.ESP_SET_GAIN, args.gain)
    esp.cmd32(snap.ESP_SET_LO, int(round(args.lo_mhz * 1e6)))
    failed = 0
    for k in range(-8, 8):
        cycles = esp.cmd(ESP_NARROW_TEST, k + 8)
        raw = esp._read(PAIRS * 4 + PAIRS)
        words = np.frombuffer(raw[: PAIRS * 4], "<u4")
        outs = np.frombuffer(raw[PAIRS * 4 :], "<u4")
        got = np.stack([signed10(outs), signed10(outs >> 10)], 1)
        bad = int(np.sum(model(words, k) != got))
        failed += bad != 0
        print(f"k {k:+d}: {cycles / PAIRS:.2f} cycles per pair, {bad} mismatches")
    print("all outputs match" if not failed else f"{failed} offsets mismatched")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
