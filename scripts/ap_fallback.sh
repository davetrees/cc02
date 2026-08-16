#!/bin/bash
# CC-02 AP fallback: if wlan0 not connected AND no eth carrier after 20s boot wait,
# bring up a hotspot. Never breaks eth0. Always exits 0.
sleep 20
if nmcli -t -f DEVICE,STATE d | grep -q '^wlan0:connected'; then
    echo "ap_fallback: wlan0 already connected, nothing to do"
    exit 0
fi
if [ "$(cat /sys/class/net/eth0/carrier 2>/dev/null)" = "1" ]; then
    echo "ap_fallback: eth0 has carrier, nothing to do"
    exit 0
fi
echo "ap_fallback: no wifi, no eth carrier -> starting hotspot CC02-1630"
nmcli device wifi hotspot ifname wlan0 ssid CC02-1630 password CHANGEME_AP_PASS || true
exit 0
