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
  6 dB above it by default (with 32 us of lead-in) and closes it after 32 us
  within 3 dB. Bursts are capped at 512 us (longer ones are truncated and flagged).
  Optionally (`arg` bit 0) bursts whose envelope fluctuates like OFDM (Wi-Fi)
  are dropped, keeping constant-envelope ones such as GFSK (Bluetooth).
* Bursts are packed at 20 bits per pair into a 64 KB queue in bank 2, which
  is never given to the writer. Bank 3's top holds the ROM's working memory,
  so it is saved before streaming and restored afterwards.

Every record has a magic, type, sequence number, start (in pairs) and a
check value; `esp32s3/src/stream.h` documents the format. Status records
carry the noise floor and counters for bursts sent, rejected, dropped for
queue space and truncated, processing overruns, and stream discontinuities.
They are emitted on common 256 ms USB start-of-frame boundaries: flag bit 15
marks this, bits 0-10 hold the host's USB frame number, and `start` is the
sample pair latched just after that frame arrived. Receivers below the same
USB host can therefore measure their sample-count offset and crystal drift
without hearing a common RF channel. Existing hosts ignore the new flags and
remain compatible.

Argument bits 3-7 optionally set the integer trigger-to-noise power ratio;
zero keeps the default ratio of 4. Argument bit 2 enables extended status
records for receiver tests. Their first
eight words are the normal status values. The remaining words cover the last
reporting interval: queue high-water, trigger count, maximum trigger power and
its noise floor (float bits), rejections by mask/envelope/in-channel power (the
envelope count covers both the whole-window and the in-channel envelope
tests), and maximum unread-pair backlog. Normal streams keep the original record format.

Observed with an antenna on advertising channel 38 (`-g 28`): about 40
bursts a second forwarded, and blue-dragon decoded 84 of 89 BLE packets
with a valid CRC in 15 s. When the band is busy the USB link is the limit
and the queue drops bursts.

### Channelized bursts (`arg` bit 1)

Bluetooth uses 1 or 2 MHz of the 16 MHz window, so most of a whole-window
burst is noise. With bit 1 set, the ESP measures each burst's frequency over
its first 32 us, rounds it to whole MHz, and sends only that channel:
mixed down, filtered to +-1.6 MHz and decimated to 4 Msps (STREAM_NARROW
records, a quarter of the data). The filter runs on the S3's vector
extension at about 8 cycles per pair (`esp32s3/src/narrow_run.S`); in C it
took 98, far beyond the 15 available at 16 Msps. Bursts may then run to
3 ms (a whole 3-DH5 packet) instead of 512 us. With bit 0 set as well,
bursts that keep less than half their power in their channel are dropped
as Wi-Fi, which catches OFDM the envelope test misses. Channels within
2 MHz of the LO skip the in-channel envelope test: the radio's DC offset
lies inside their filter and beats with a Bluetooth packet, while Wi-Fi
there covers the whole window and fails the power test anyway. During an
`l2ping` flood (LO 2408.5 MHz) the two channels beside the LO went from a
median of 59% to 105% of the packets on the farther channels. The host
interpolates each record back to 16 Msps (see `widen()` in `usb/stream.py`).

A burst is sent on one channel only, so two signals on different channels
at the same instant are not both kept; whole-window bursts remain the
default for that.

Measured on a busy band (antenna, `-g 28`, LO 2426 MHz, Wi-Fi nearby), 45 s
live through blue-dragon: 514 BLE packets with a valid CRC channelized
against 257 with whole-window bursts. With an `l2ping` flood between two
Classic devices (LO 2441 MHz, 38 s): 10,993 Classic framings and 12 EDR
packets with a valid CRC, against 2,973 and none.

### What 16 Msps hears

The 16 Msps samples are the 80 Msps capture decimated by five without a
digital filter, so the analog baseband filter (`ESP_SET_FILTER`, a 6-bit
capacitor code; larger is narrower) is the only anti-alias filter. At its
default of 0 (about 69 MHz) a receiver hears about 80 MHz around its LO
folded into its 16 MHz window: a signal 16 or 32 MHz away from a position in
the window arrives there at much the same strength (measured flat to about
25 MHz from the LO, 5 dB down at 39, gone by 55). At code 54 a signal 9 or
23 MHz outside the window no longer came through, while one 7 MHz inside
still did, about a dB down, and the noise floor fell. The bandwidth
calibration for these codes came from the
[ESP-SDR](https://github.com/ESPARGOS/esp-sdr) project. A burst's channel offset is
therefore only known modulo 16 MHz. A BLE packet passes its CRC only when
dewhitened for the channel it was sent on, which settles it for BLE.

Several receivers can share one LO and split the channel positions
between them: `ESP_STREAM`'s high half (sent first with `ESP_ARG_HIGH`)
is a mask of offsets k (-8..7, bit k + 8), and a channelized burst whose
offset is outside it is skipped. Tuned to 2441 MHz, the three BLE
advertising channels fold to k = +7, -1 and -7 (2402, 2426, 2480 MHz). With
five boards sharing 2441 MHz and blue-dragon placing each burst at every
frequency it could have come from, about 800 advertising packets with a
valid CRC were received in 30 s across all three channels.
`usb/stream.py --positions` sets the mask.

`ESP_NARROW_TEST` (op 42, `arg` = offset + 8) runs the filter over a
snapshot and returns its input and output; `usb/narrow_check.py` compares
them with the same arithmetic in numpy for every offset.

## Snapshot (`ESP_SNAPSHOT`, op 40)

Runs the dump engine into capture bank 0 until its 16384-pair ring has been
overwritten, stops it, and sends the ring oldest-first after the normal
reply: a 16-byte header (`SNAP`, pair count, rate selector, LO in Hz), the
raw 32-bit dump words, then a CRC-32. At 16 Msps that is about 1 ms of a
16 MHz window per request. It works at 80 Msps too, and is the simplest way
to check reception.

## Use

Without building anything: download `iq-source.bin` from this
repository's releases and load it into every attached ESP32-S3 (RAM only;
a power cycle restores the board):

```sh
pip install esptool pyserial numpy
python3 usb/load.py --image iq-source.bin
```

Or build it and load the result:

```sh
. $IDF_PATH/export.sh          # ESP-IDF v5.5.3 or later
make -C esp32s3
python3 usb/load.py            # every attached ESP32-S3; --port for one
esptool --chip esp32s3 --port /dev/ttyACM0 --before default-reset \
        --after no-reset --no-stub load-ram esp32s3/build/iq-source.bin  # the same, by hand
python3 usb/stream.py --lo-mhz 2426 --gain 28 --seconds 10 --reject-wideband --channelize
python3 usb/snap.py --lo-mhz 2426 --rate 16 --gain 28 --count 100
```

Both tools write interleaved int16 IQ. eSpDR samples use the LO-minus-RF
convention and 10-bit values, so they conjugate each pair and scale by 64.
blue-dragon reads the ESP directly with `-i espdr0` (built with
`--features espdr`), streaming channelized bursts when the firmware
supports it.

`-g`/`--gain` is the receiver's gain-table selector (0-127, not dB). With an
antenna attached, about 24-30 avoids clipping; the table is not linear.

ESP-IDF v5.5.3 or later is needed for `I2S_CLK_SRC_PLL_240M`.
