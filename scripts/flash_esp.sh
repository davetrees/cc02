#!/bin/bash
# Flash the CC-02 ESP32-S3 (Waveshare Touch-LCD-3.5B) from a Mac with ESP-IDF.
# Usage: ./flash_esp.sh [/dev/cu.usbmodemXXX]
#
# KNOWN QUIRK (proven 2026-08-16): after esptool touches the chip over
# USB-Serial-JTAG it can latch in ROM download mode — no software reset exits.
# If the app doesn't stream after flashing, PRESS THE PHYSICAL RESET (RST/EN)
# BUTTON or replug USB. Also: any serial attach must set dtr=True, rts=False
# BEFORE open(), or pyserial's defaults strap the chip back into download mode.
set -e
PORT=${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}
[ -z "$PORT" ] && { echo "no /dev/cu.usbmodem* port found"; exit 1; }
cd "$(dirname "$0")/../firmware"
source ~/esp/esp-idf/export.sh
idf.py -p "$PORT" flash
echo "Flashed. If no telemetry within 15 s, press the RESET button on the board."
echo "Verify: python3 ../scripts/verify_esp.py $PORT"
