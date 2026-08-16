#!/bin/bash
# CC-02 Pi provisioning — reproducible record of the 2026-08-16 deploy.
# Run as pi on a fresh Raspberry Pi OS Trixie Lite 64-bit.
set -e

# --- 1. apt packages ---
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv python3-numpy \
    git rsync network-manager

# --- 2. groups ---
sudo usermod -aG dialout,video pi

# --- 3. venv (system site packages so apt opencv/numpy are visible) ---
mkdir -p /home/pi/cc02 /home/pi/cc02/logs /home/pi/cc02/brain /home/pi/cc02/scripts
python3 -m venv --system-site-packages /home/pi/cc02/venv

# --- 4. pip packages (CPU torch index) ---
/home/pi/cc02/venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu \
    --extra-index-url https://pypi.org/simple torch torchvision
/home/pi/cc02/venv/bin/pip install ultralytics aiohttp pyserial
# Pro Controller support (direct Bluetooth pairing; kernel hid-nintendo driver)
/home/pi/cc02/venv/bin/pip install evdev || sudo apt-get install -y python3-evdev
sudo systemctl enable --now bluetooth.service
# NOTE: vision runs on the Hailo-10H HAT (hailort + /usr/share/hailo-models/
# yolov8m_h10.hef from the hailo-all stack) with automatic CPU fallback.

# --- 5. predownload YOLO model ---
cd /home/pi/cc02 && /home/pi/cc02/venv/bin/python -c \
    "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# --- 6. deploy brain code ---
# (rsync/scp the brain/ scripts/ trees from the dev machine into /home/pi/cc02/)

# --- 7. udev rule for ESP32-S3 (Espressif VID 303a) ---
sudo tee /etc/udev/rules.d/99-cc02.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", SYMLINK+="cc02esp"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

# --- 8. systemd services ---
sudo cp /home/pi/cc02/system/cc02-brain.service /etc/systemd/system/
sudo cp /home/pi/cc02/system/cc02-ap.service /etc/systemd/system/
chmod +x /home/pi/cc02/scripts/ap_fallback.sh
sudo systemctl daemon-reload
sudo systemctl enable --now cc02-brain.service
sudo systemctl enable cc02-ap.service   # oneshot, runs at boot

# --- 9. SSH keys (dev Mac pubkeys appended; password auth left ON) ---
mkdir -p /home/pi/.ssh && chmod 700 /home/pi/.ssh
# cat dev-mac ~/.ssh/*.pub >> /home/pi/.ssh/authorized_keys
chmod 600 /home/pi/.ssh/authorized_keys 2>/dev/null || true

echo PROVISION_DONE
