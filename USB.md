# USB modes (no FPGA)

This fork adds two ways to get IQ out of the ESP32-S3 using only its own
USB Serial/JTAG port, without the Alchitry FPGA link. It builds on the
eSpDR radio firmware by h0m3us3r (https://github.com/h0m3us3r/eSpDR, 0BSD).

The USB port runs at Full Speed (about 1 MB/s; 0.6 MB/s in practice), far
below the 200 MB/s the 80 Msps stream needs, so neither mode is continuous
IQ. The firmware still loads into RAM only; a power cycle restores the board.

## Stream (`ESP_STREAM`, op 41)

The ESP watches a 16 MHz window (16 Msps) continuously and sends only the
bursts that rise above the noise floor, each timestamped in samples since the
stream began. It runs until the host sends any byte, then ends with an END
record.

* Core 1 keeps the dump engine running. The CPU cannot read a capture bank
  while the writer has it selected, so the writer rotates through banks 0, 1
  and 3. Each bank is prepared with sentinels just before the writer enters
  it, and the exact first and last pairs written are found after it leaves;
  a segment that is not contiguous with the previous one is reported as a
  discontinuity, never stitched over. Core 1 also feeds USB from its polling
  loop, since the endpoint's single 64-byte buffer must be refilled promptly.
* Core 0 measures power in 8 us blocks, tracks the noise floor, opens a burst
  6 dB above it (with 32 us of lead-in) and closes it after 32 us within
  3 dB. Bursts are capped at 512 us (longer ones are truncated and flagged).
  Optionally (`arg` bit 0) bursts whose envelope fluctuates like OFDM (Wi-Fi)
  are dropped, keeping constant-envelope ones such as GFSK (Bluetooth).
* Bursts are packed at 20 bits per pair into a 64 KB queue in bank 2, which
  is never given to the writer. Bank 3's top holds the ROM's working memory,
  so it is saved before streaming and restored afterwards.

Every record has a magic, type, sequence number, start (in pairs) and a
check value; `esp32s3/src/stream.h` documents the format. Status records
every 250 ms carry the noise floor and counters for bursts sent, rejected,
dropped for queue space and truncated, processing overruns, and stream
discontinuities.

Observed with an antenna on advertising channel 38 (`-g 28`): about 40
bursts a second forwarded, and blue-dragon decoded 84 of 89 BLE packets
with a valid CRC in 15 s. When the band is busy the USB link is the limit
and the queue drops bursts.

## Snapshot (`ESP_SNAPSHOT`, op 40)

Runs the dump engine into capture bank 0 until its 16384-pair ring has been
overwritten, stops it, and sends the ring oldest-first after the normal
reply: a 16-byte header (`SNAP`, pair count, rate selector, LO in Hz), the
raw 32-bit dump words, then a CRC-32. At 16 Msps that is about 1 ms of a
16 MHz window per request. It works at 80 Msps too, and is the simplest way
to check reception.

## Use

```sh
. $IDF_PATH/export.sh          # ESP-IDF v5.5.3 or later
make -C esp32s3
esptool --chip esp32s3 --port /dev/ttyACM0 --before default-reset \
        --after no-reset --no-stub load-ram esp32s3/build/iq-source.bin
python3 usb/stream.py --lo-mhz 2426 --gain 28 --seconds 10 --reject-wideband
python3 usb/snap.py --lo-mhz 2426 --rate 16 --gain 28 --count 100
```

Both tools write interleaved int16 IQ. eSpDR samples use the LO-minus-RF
convention and 10-bit values, so they conjugate each pair and scale by 64.
blue-dragon reads the ESP directly with `-i espdr0` (built with
`--features espdr`), streaming when the firmware supports it.

`-g`/`--gain` is the receiver's gain-table selector (0-127, not dB). With an
antenna attached, about 24-30 avoids clipping; the table is not linear.

ESP-IDF v5.5.3 or later is needed for `I2S_CLK_SRC_PLL_240M`.
