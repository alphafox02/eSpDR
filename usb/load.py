#!/usr/bin/env python3
"""Load the eSpDR firmware into the RAM of every attached ESP32-S3.

Nothing is written to flash: a power cycle restores whatever the board had.
Each board is found by its USB Serial/JTAG port, loaded with esptool
(`pip install esptool`) and then asked to identify itself, so a board that
did not take the image is reported. Use --port to load only some boards.

The image is esp32s3/build/iq-source.bin after `make -C esp32s3`, or the
iq-source.bin attached to a release of this repository.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

from serial.tools import list_ports

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import snap  # noqa: E402  (shares the control protocol)

FIRMWARE_ID = 0x49515305
DEFAULT_IMAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "esp32s3", "build", "iq-source.bin")


def esp_ports():
    """USB Serial/JTAG ports of attached ESP32-S3 boards, stable when possible."""
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit*"))
    if ports:
        return ports
    # Some systems do not mount /dev/serial/by-id. Do not fall back to every
    # ttyACM device: resetting an unrelated modem or debug probe is surprising.
    # Espressif's native USB Serial/JTAG controller is VID:PID 303a:1001.
    devices = [p for p in list_ports.comports() if (p.vid, p.pid) == (0x303A, 0x1001)]
    devices.sort(key=lambda p: (p.serial_number or "", p.location or "", p.device))
    return [p.device for p in devices]


def esptool_command():
    for name in ("esptool", "esptool.py"):
        path = shutil.which(name)
        if path:
            return [path]
    return [sys.executable, "-m", "esptool"]


def load(port, image):
    command = esptool_command() + [
        "--chip", "esp32s3", "--port", port,
        "--before", "default-reset", "--after", "no-reset",
        "--no-stub", "load-ram", image,
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        lines = (result.stderr or result.stdout).strip().splitlines()
        return lines[-1] if lines else "esptool failed"
    return None


def answers(port):
    for _ in range(3):
        esp = None
        try:
            esp = snap.Esp(port)
            ident = esp.cmd(snap.CTL_INFO)
            return ident == FIRMWARE_ID
        except Exception:
            time.sleep(0.5)
        finally:
            if esp is not None:
                esp.s.close()
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", default=DEFAULT_IMAGE, help="firmware image (default: the local build)")
    ap.add_argument("--port", action="append", help="a board's serial port (repeatable; default: every ESP32-S3)")
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        sys.exit(f"no firmware image at {args.image}: build it (make -C esp32s3) or pass --image")
    ports = args.port or esp_ports()
    if not ports:
        sys.exit("no ESP32-S3 USB serial ports found")

    failed = 0
    for port in ports:
        name = os.path.basename(os.path.realpath(port))
        error = load(port, args.image)
        if error is None:
            time.sleep(1.0)  # the firmware starts its radio after loading
            error = None if answers(port) else "loaded, but the firmware does not answer"
        print(f"{name}: {'ok' if error is None else error}")
        failed += error is not None
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
