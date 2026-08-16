# CC-02 RC Car — Runbook

## Power-on sequence
1. Power the car's ESC/servo battery OFF. Power the Pi 5 (USB-C).
2. Pi boots; `cc02-brain` starts automatically (~60-90s until YOLO is warm — torch import is slow).
3. Plug the ESP32-S3 (Waveshare 3.5B) into the Pi USB-A. The brain auto-connects within ~2s (it polls every 2s; runs fine without it).
4. Wheels-off-ground FIRST: **put the car on a stand before any throttle test.** ESC arms when its battery comes on; a stray throttle command means a runaway car.
5. Power the ESC battery on only when the panel shows serial "up".

## URLs
- LAN (ethernet/wifi): http://pi.local:8080 or http://192.168.1.195:8080
- No network found at boot → Pi opens hotspot **CC02-1630** / password **CHANGEME_AP_PASS**, then browse http://10.42.0.1:8080

## Modes
- **MANUAL** — direct passthrough of joystick/keys/gamepad (max-speed slider caps throttle).
- **ASSIST** — same, but forward throttle cut to neutral while a collision is detected (reverse always allowed).
- **AUTO** — creeps forward (~1560µs) steering toward the most-open image column; stops on collision.
- **RTH** — BEST-EFFORT breadcrumb replay via dead reckoning (integrated yaw rate + throttle speed model). It is NOT SLAM; expect drift. Stops on collision.
- **ESTOP** — giant red button (or ESP-side request): outputs locked to 1500/1500 and the estop flag is sent to the ESP. Toggle again to clear.

## Vision
YOLO runs on the **Hailo-10H HAT** (yolov8m_h10.hef, 640x640, NMS on-device) — primary backend, camera-capped fps. If the HAT is absent, busy, or errors repeatedly, the brain **automatically falls back to CPU** yolov8n (imgsz=320, ~3-8 fps, thread-capped). Active backend is logged at service start: `journalctl -u cc02-brain | grep backend=`.

## Controllers
- Web panel: virtual joysticks, WASD/arrows, or a phone/laptop-paired gamepad (browser Gamepad API).
- **Pro Controller direct to the Pi:** run `/home/pi/cc02/scripts/pair_controller.sh` and HOLD the controller's sync button when prompted (one-time; it auto-reconnects after). Left stick = steer, right stick = throttle (up = forward), **B = ESTOP toggle**, **PLUS = mode cycle**. Panel shows "Pro Controller: connected". If it stops sending for 150ms (sleep, out of range), inputs go neutral. Most recent input wins between panel and controller.

## Failsafe behavior
- The **ESP owns the servo/ESC** and its failsafe: if it stops receiving CMD frames for 200ms it centers steering and neutrals throttle on its own. Pi crash/reboot = car stops.
- Pi side: if no client input (web/gamepad) for 500ms, steer/throttle go neutral.
- Anti-tip (roll/pitch cut, default 45°/45°) runs on the ESP; thresholds set from the panel.

## Logs
- CSV telemetry (10Hz): `/home/pi/cc02/logs/log_YYYYmmdd_HHMMSS.csv` — newest downloadable at `/logs/latest.csv` from the panel.
- Breadcrumb map: `/home/pi/cc02/logs/map.jsonl`
- Service log: `journalctl -u cc02-brain -f`

## Restarting services
```
sudo systemctl restart cc02-brain     # the whole brain
sudo systemctl status cc02-brain
sudo systemctl start cc02-ap          # re-run AP fallback check
```
Config lives in `/home/pi/cc02/config.json` (saved automatically from the panel).
Full reprovision record: `/home/pi/cc02/scripts/provision.sh`.
