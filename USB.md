# USB mode (no FPGA)

This branch adds a way to get IQ out of the ESP32-S3 using only its own
USB Serial/JTAG port, without the Alchitry FPGA link. It builds on the
eSpDR radio firmware by h0m3us3r (https://github.com/h0m3us3r/eSpDR, 0BSD).

The USB port runs at Full Speed (about 1 MB/s), far below the 200 MB/s the
80 Msps stream needs, so this mode trades continuity for simplicity:

* `ESP_SNAPSHOT` (op 40) runs the dump engine into capture bank 0 until its
  16384-pair ring has been overwritten, stops it, and sends the ring
  oldest-first after the normal reply: a 16-byte header (`SNAP`, pair count,
  rate selector, LO in Hz), the raw 32-bit dump words, then a CRC-32.
* At 16 Msps a snapshot covers about 1 ms of a 16 MHz window. Bank 3, which
  overlaps the ROM's working memory, is never selected, and no link GPIOs or
  second-core code are used.

The firmware still loads into RAM only; a power cycle restores the board.

## Use

```sh
. $IDF_PATH/export.sh          # ESP-IDF v5.5.3 or later
make -C esp32s3
esptool --chip esp32s3 --port /dev/ttyACM0 --before default-reset \
        --after no-reset --no-stub load-ram esp32s3/build/iq-source.bin
python3 usb/snap.py --lo-mhz 2426 --rate 16 --gain 28 --count 100
```

`usb/snap.py` writes interleaved int16 IQ. eSpDR samples use the LO-minus-RF
convention and 10-bit values, so it conjugates each pair and scales by 64.
blue-dragon reads the ESP directly with `-i espdr0` (built with
`--features espdr`).

ESP-IDF v5.5.3 or later is needed for `I2S_CLK_SRC_PLL_240M`.
