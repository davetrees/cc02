# CC-02 — Autonomous Tamiya CC-02 RC Car

A Tamiya CC-02 crawler turned into a camera-driven robot. A **Raspberry Pi 5 + AI HAT+ 2 (Hailo-10H)**
runs YOLO vision and the control/web stack; a **Waveshare ESP32-S3-Touch-LCD-3.5B** owns the servo/ESC
PWM and all fail-safe behaviour, driven over one USB-CDC cable. The stock receiver is removed; the only
wires onto the Tamiya are steering-servo signal, ESC signal, and common ground.

## Why two computers
Linux is too jittery/crash-prone to be the last device holding an ESC PWM signal, so the ESP owns the
servo + ESC and fails safe (neutral throttle, LCD "LINK LOST") the instant the Pi stops sending commands.
The Pi does vision, planning, the web panel, and controller input; it only sends setpoints.

## Layout
- `firmware/` — ESP-IDF 5.x firmware for the Waveshare 3.5B: USB-CDC binary link (COBS + CRC16),
  QMI8658 IMU + complementary filter, LEDC RC-PWM out, mount calibration, anti-roll counter-steer,
  live tip-cut, active-brake ESTOP, and a 320×480 LVGL instrument cluster.
- `brain/` — Python 3 on the Pi: `serial_link` (protocol), `vision` (Hailo YOLO + floor-model
  runway sensor, CPU fallback), `autopilot` (MANUAL/ASSIST/AUTO cruiser/RTH + slew + counter-steer),
  `web` (aiohttp panel + MJPEG + WebSocket on :8080), `gamepad` (Switch Pro / Nimbus over BLE),
  `mapping` (breadcrumb dead-reckoning), `logger`, `debugport` (see security note).
- `docs/` — `WIRING.md` (frozen pin table + power Y-split) and `RUNBOOK.md` (operation).
- `scripts/` — provisioning, systemd units, ESP flash + verify, controller pairing, AP fallback.

## Control pins (Waveshare J8 header)
| Signal | ESP GPIO | Header pin |
|---|---|---|
| Steering servo signal | GPIO41 | 13 |
| ESC / throttle signal | GPIO42 | 15 |
| Common ground | — | 29/30 |

Full wiring, power split, and fail-safe rules: [docs/WIRING.md](docs/WIRING.md).

## Serial protocol (v1, frozen)
COBS-framed, CRC16-CCITT. ESP→Pi TELEM @100 Hz (IMU, attitude, live PWM, battery, flags) +
MODEREQ on touch; Pi→ESP CMD @50 Hz (steer/throttle/mode/failsafe) + DISP @10 Hz. Header of record:
[firmware/main/protocol.h](firmware/main/protocol.h).

## ⚠️ Security note — `brain/debugport.py`
The debug port runs **arbitrary Python against the live process** for field debugging. In this build it
binds `0.0.0.0:8001` for convenience on a private hobby network — that is a **remote-code-execution
endpoint reachable by anyone on the LAN**. Do not expose it beyond a trusted network; bind it to
`127.0.0.1` (or remove it) for anything but bench/backyard use.

## Config
`config.pi-current.json` is a sample of the live tuning (speeds, YOLO, AUTO cruiser gains). The AP
fallback password in `scripts/ap_fallback.sh` and `docs/RUNBOOK.md` is a placeholder
(`CHANGEME_AP_PASS`) — set your own.

Built for a specific vehicle; not a generic robot framework.
