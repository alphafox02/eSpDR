#!/usr/bin/env python3
"""Receive burst-gated IQ from an ESP32-S3 running the eSpDR USB stream mode.

Starts ESP_STREAM, reads records for --seconds, then stops the stream and
drains to the END record. Every record's sequence number and checksum are
checked. Bursts are written as interleaved int16 IQ for blue-dragon
(--format ci16): conjugated (eSpDR is LO-minus-RF), scaled by 64, with gaps
between bursts shortened to --gap pairs of noise at the reported floor.
With --channelize the ESP sends each burst as its own channel at 4 Msps;
those are restored to the 16 MHz window here, so the output is the same.
"""
import argparse
import struct
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import snap  # noqa: E402  (shares the control protocol)

ESP_STREAM = 41
MAGIC = 0x54535242
BURST, STATUS, END, NARROW = 1, 2, 3, 4
TRUNCATED = 1
CHANNELIZE = 2

# The ESP's channel filter (Q14, see esp32s3/src/narrow.c). Narrow output j
# is centred on pair start + 4j - 2.5 and was mixed down by k MHz.
NARROW_TAPS = np.array([-27, 62, 476, 1428, 2676, 3577, 3577, 2676, 1428, 476, 62, -27]) / 16384


def signed(words):
    i = ((words & 1023) ^ 512).astype(np.int32) - 512
    q = (((words >> 10) & 1023) ^ 512).astype(np.int32) - 512
    return i + 1j * q


def widen(words, start, k):
    """Narrow record -> pairs at 16 Msps from `start` on, LO-minus-RF like a burst."""
    y = signed(words)
    up = np.zeros(4 * len(y), complex)
    up[::4] = y
    # Interpolate with the same filter (gain 4 restores the level). Output i
    # of the full convolution is centred on pair start - 8 + i.
    z = np.convolve(up, 4 * NARROW_TAPS)[8 : 8 + len(up)]
    n = start + np.arange(len(z))
    return z * np.exp(2j * np.pi * ((k * n) % 16) / 16)


def complex_to_ci16(x):
    out = np.empty(2 * len(x), dtype="<i2")
    out[0::2] = np.clip(np.round(x.real) * 64, -32768, 32767)
    out[1::2] = np.clip(-np.round(x.imag) * 64, -32768, 32767)
    return out


def unpack_pairs(payload, pairs):
    b = np.frombuffer(payload, dtype=np.uint8).reshape(-1, 5).astype(np.uint32)
    lo = b[:, 0] | b[:, 1] << 8 | b[:, 2] << 16 | b[:, 3] << 24
    p0 = lo & 0xFFFFF
    p1 = (lo >> 20) | (b[:, 4] << 12)
    words = np.empty(2 * len(lo), dtype=np.uint32)
    words[0::2], words[1::2] = p0, p1
    return words[: 2 * len(lo)], words[:pairs]


def to_ci16(words):
    i = ((words & 1023) ^ 512).astype(np.int32) - 512
    q = (((words >> 10) & 1023) ^ 512).astype(np.int32) - 512
    out = np.empty(2 * len(words), dtype="<i2")
    out[0::2] = np.clip(i * 64, -32768, 32767)
    out[1::2] = np.clip(-q * 64, -32768, 32767)
    return out


class Reader:
    def __init__(self, port):
        self.port = port
        self.expect_seq = 0
        self.errors = 0

    def read(self, n):
        data = self.port.read(n)
        if len(data) != n:
            raise IOError(f"short read {len(data)}/{n}")
        return data

    def record(self):
        # Resynchronise on the magic if the stream ever slips.
        window = b""
        while True:
            window = (window + self.read(1))[-4:]
            if len(window) == 4 and struct.unpack("<I", window)[0] == MAGIC:
                break
        rest = self.read(20)
        tf, seq, start_lo, start_hi, length = struct.unpack("<IIIII", rest)
        rtype, flags = tf & 0xFFFF, tf >> 16
        start = start_lo | start_hi << 32
        if rtype in (BURST, NARROW):
            payload = self.read(((length + 1) // 2) * 5)
        elif rtype == STATUS:
            payload = self.read(32)
        else:
            payload = b""
        (check,) = struct.unpack("<I", self.read(4))
        if rtype in (BURST, NARROW):
            all_words, words = unpack_pairs(payload, length)
            ok = int(all_words.sum(dtype=np.uint64)) & 0xFFFFFFFF == check
        elif rtype == STATUS:
            ok = sum(struct.unpack("<8I", payload)) & 0xFFFFFFFF == check
            words = struct.unpack("<8I", payload)
        else:
            ok, words = True, None
        if seq != self.expect_seq or not ok:
            self.errors += 1
            print(f"  !! record seq {seq} (expected {self.expect_seq}) checksum {'ok' if ok else 'BAD'}")
        self.expect_seq = seq + 1
        return rtype, flags, start, length, words


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--lo-mhz", type=float, default=2426.0)
    ap.add_argument("--gain", type=int, default=28)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--reject-wideband", action="store_true")
    ap.add_argument("--channelize", action="store_true", help="have the ESP send each burst's channel at 4 Msps")
    ap.add_argument("--positions", default="", help="with --channelize, only these channel offsets from the LO "
                    "(MHz, LO-minus-RF, comma separated, -8..7), for sharing one LO between receivers")
    ap.add_argument("--gap", type=int, default=2048, help="noise pairs written between bursts")
    ap.add_argument("--out", default="/tmp/espdr_stream.ci16")
    ap.add_argument("--verbose", action="store_true", help="print every status record")
    args = ap.parse_args()

    esp = snap.Esp(args.port)
    esp.cmd(snap.ESP_SET_RATE, 1)
    esp.cmd(snap.ESP_SET_WIDTH, 20)
    esp.cmd(snap.ESP_SET_GAIN, args.gain)
    lo = esp.cmd32(snap.ESP_SET_LO, int(round(args.lo_mhz * 1e6)))
    print(f"LO {lo / 1e6:.6f} MHz, 16 Msps, gain {args.gain}, reject wideband {args.reject_wideband}")

    mask = 0
    for k in filter(None, args.positions.split(",")):
        mask |= 1 << (int(k) + 8)
    if mask:
        esp.cmd(snap.ESP_ARG_HIGH, mask)
    esp.cmd(ESP_STREAM, (1 if args.reject_wideband else 0) | (CHANNELIZE if args.channelize else 0))
    rd = Reader(esp.s)
    rng = np.random.default_rng(1)
    floor_rms = 4.0
    bursts = burst_pairs = truncated = 0
    lengths, channels = [], {}
    payload_bytes = 0
    last_status = None
    t0 = time.time()
    stopped = False
    with open(args.out, "wb") as f:
        while True:
            if not stopped and time.time() - t0 > args.seconds:
                esp.s.write(b"\x00")
                stopped = True
            rtype, flags, start, length, words = rd.record()
            if rtype in (BURST, NARROW):
                payload_bytes += ((length + 1) // 2) * 5
            if rtype == NARROW:
                k = ((flags >> 8) & 15) - 8
                channels[k] = channels.get(k, 0) + 1
                x = widen(words, start, k)
                bursts += 1
                burst_pairs += len(x)
                lengths.append(len(x))
                truncated += bool(flags & TRUNCATED)
                gap = rng.normal(0, floor_rms * 64, 2 * args.gap).astype("<i2")
                f.write(gap.tobytes())
                f.write(complex_to_ci16(x).tobytes())
            elif rtype == BURST:
                bursts += 1
                burst_pairs += length
                lengths.append(length)
                truncated += bool(flags & TRUNCATED)
                gap = rng.normal(0, floor_rms * 64, 2 * args.gap).astype("<i2")
                f.write(gap.tobytes())
                f.write(to_ci16(words).tobytes())
            elif rtype == STATUS:
                floor = struct.unpack("<f", struct.pack("<I", words[0]))[0]
                floor_rms = max(1.0, (floor / 2) ** 0.5)
                last_status = (start, words)
                if args.verbose:
                    print(f"  t={start / 16e6:6.2f}s floor {floor:7.1f} sent {words[1]} rejected {words[2]} "
                          f"dropped {words[3]} truncated {words[4]} overruns {words[5]} "
                          f"discontinuities {words[6]} queue {words[7]} B")
            elif rtype == END:
                elapsed_pairs = start
                break
    wall = time.time() - t0
    air = elapsed_pairs / 16e6
    print(f"stream: {wall:.1f}s wall, {air:.2f}s of samples processed, record errors {rd.errors}")
    if last_status:
        _, w = last_status
        floor = struct.unpack("<f", struct.pack("<I", w[0]))[0]
        print(f"esp counters: sent {w[1]} rejected {w[2]} dropped {w[3]} truncated {w[4]} "
              f"overruns {w[5]} discontinuities {w[6]} floor {floor:.1f}")
    if bursts:
        L = np.array(lengths)
        print(f"received {bursts} bursts, {burst_pairs / 16e3:.1f} ms of burst airtime "
              f"({100 * burst_pairs / max(elapsed_pairs, 1):.2f}% duty), truncated {truncated}; "
              f"length us median {np.median(L) / 16:.0f} max {L.max() / 16:.0f}; "
              f"{payload_bytes / 1e3:.0f} KB of samples over USB")
    if channels:
        # Offsets are LO-minus-RF: the channel is at LO - k MHz.
        print("channels (MHz: bursts): " + ", ".join(
            f"{lo / 1e6 - k:.0f}: {n}" for k, n in sorted(channels.items(), key=lambda kv: -kv[0])))
    print(f"-> {args.out}  (blue-dragon --file {args.out} --format ci16 --sample-rate 16000000 "
          f"-c {round(lo / 1e6)} -C 16)")


if __name__ == "__main__":
    sys.exit(main())
