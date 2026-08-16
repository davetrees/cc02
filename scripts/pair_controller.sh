#!/bin/bash
# Pair a game controller (Switch Pro Controller or SteelSeries Nimbus) to this Pi.
set -u
echo "=== CC-02 controller pairing ==="
echo "Put YOUR controller in pairing mode first:"
echo "  Switch Pro Controller: HOLD the small SYNC button (top edge, next to"
echo "    USB-C) until the player LEDs sweep back and forth."
echo "  SteelSeries Nimbus: power on unconnected (LEDs flash = pairing). If it"
echo "    keeps grabbing an old host, put it in pairing mode per its manual"
echo "    (Nimbus+: hold Menu+A until LEDs flash) and forget it on that host."
echo "  NOTE: the original Nimbus is an Apple MFi device - if Linux pairing"
echo "  fails, pair it to your phone/laptop instead and drive from the web"
echo "  panel (browser Gamepad API - already supported)."
echo
sudo systemctl start bluetooth 2>/dev/null || true
bluetoothctl power on >/dev/null
echo "Scanning up to 30s for 'Pro Controller' or 'Nimbus'..."
bluetoothctl --timeout 30 scan on >/dev/null 2>&1 &
SCAN_PID=$!
MAC=""
for i in $(seq 1 30); do
    MAC=$(bluetoothctl devices | grep -iE "Pro Controller|Nimbus" | awk '{print $2}' | head -1)
    [ -n "$MAC" ] && break
    sleep 1
done
kill $SCAN_PID 2>/dev/null
wait $SCAN_PID 2>/dev/null
if [ -z "$MAC" ]; then
    echo "FAILED: no controller seen. Was it in pairing mode? Run me again."
    exit 1
fi
NAME=$(bluetoothctl devices | grep "$MAC" | cut -d" " -f3-)
echo "Found '$NAME' at $MAC - pairing..."
bluetoothctl pair "$MAC"
bluetoothctl trust "$MAC"
bluetoothctl connect "$MAC"
echo
echo "Done. The brain picks it up within 3s (panel shows the controller badge)."
echo "It reconnects on its own later - press any button to wake it."
