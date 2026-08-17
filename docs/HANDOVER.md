# CC-02 — Session Handover

Paste this into a fresh Claude Code session (working dir `/Volumes/home/ETR`) to continue.

---

You are continuing work on my **Tamiya CC-02 autonomous RC car**. Read this fully before acting; then confirm the live stack is healthy before changing anything.

## What it is
- **Raspberry Pi 5 + AI HAT+2 (Hailo-10H)** on the car runs vision (Hailo YOLOv8m ~33fps), planning, the web panel, and gamepad input. **Waveshare ESP32-S3-Touch-LCD-3.5B** owns the steering-servo + ESC PWM and ALL failsafe (Linux is too jittery to hold PWM), talking to the Pi over one USB-CDC cable (COBS + CRC16 binary protocol, see `firmware/main/protocol.h`).
- Only wires onto the Tamiya: steer signal → ESP **GPIO41 (header pin 13)**, ESC signal → **GPIO42 (pin 15)**, common GND → pin 29/30. Stock receiver removed.
- **Public repo: github.com/davetrees/cc02** — mirror at `/Volumes/home/ETR/cc02`, live on the Pi at `/home/pi/cc02`. Commit and push meaningful changes (author davetrees <dave@rees.com.au>).

## Access & procedures
- SSH: `ssh pi@pi.local` (key auth works; password fallback 12345678). **One ssh session at a time** (BlueZ/USB got flaky under parallel sessions).
- **Live debug port (your best tool): `curl -s http://pi.local:8001/exec -X POST -H 'Content-Type: application/json' -d '{"code":"<python>"}'`** — runs arbitrary Python in the running brain process; `st` is the live State. `/state`, `/config` also exist. (It's an RCE bound to 0.0.0.0 — fine on the backyard LAN; bind localhost before any untrusted network.)
- Web panel: **http://pi.local:8080** ; 1080p MJPEG at `/stream.mjpg` (has YOLO boxes + yellow collision-ROI box drawn).
- Brain is a systemd service `cc02-brain` (auto-starts on boot). After editing `brain/*.py`: `sudo systemctl restart cc02-brain`. **Patch method that works:** write a Python patch script locally, `scp` it to the Pi, run it (avoids heredoc quote-hell). Always `ast.parse` after and grep for tracebacks.
- **Flash the ESP from the Pi:** `scp` the built `.bin`, then `sudo systemctl stop cc02-brain; /home/pi/cc02/venv/bin/esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 460800 write_flash 0x20000 <bin>; sudo systemctl start cc02-brain`. **Then the human MUST press the physical RESET button** — the USB-Serial-JTAG download-mode latch means no software reset boots the new app. Build the firmware on the Mac: `cd ~/cc02/firmware && source ~/esp/esp-idf/export.sh && idf.py build`.
- **Pi shutdown:** never yank power (10Hz logging → SD corruption risk). `sudo systemctl stop cc02-brain; sync; sudo shutdown -h now`, wait for the green LED to go dark.

## Conventions you MUST know (hard-won)
- **Throttle/direction:** internal frame is standard `throttle_us > 1500 = FORWARD`. Input is negated once in `autopilot._step`; a SINGLE inversion in `main.cmd_tuple` (`invert_throttle=True`, `invert_steer=False`) maps to the ESC. Manual PWM: forward = esc 1400µs, reverse = esc 1600µs. **Do not re-add an output inversion** — an earlier bug had the flip duplicated (self-cancelling). Any new "is it going forward?" test must use `> 1500`.
- **ESC reverse lockout is HARDWARE:** from forward motion a reverse command only brakes to a stop; reverse engages only from a standstill. Not a code bug — do not try to "fix" it in software.
- **ESC deadband:** forward commands below ~110µs deviation produce NO motion. `auto_min_move_us=110` enforces this in AUTO; respect it anywhere you command throttle.
- **Config** is `config.json` on the Pi, live-editable via the debug port or the panel Settings. Key: max_speed, path_center_frac (collision ROI width), auto_* cruiser gains, cam_width/height.

## State: what works (commit a5c55cc)
Serial link 100Hz, Hailo YOLO ~33fps (1080p camera, downscaled internally), web panel with virtual sticks/keyboard/direct-left-stick/Pro-Controller, tunable rectangular collision ROI with panel sliders, anti-roll counter-steer (ARS_DIR=+1, firmware), active-brake ESTOP, tip-cut, mount-cal (tap horizon or ZERO button on level ground), gyro auto-zero, breadcrumb RTH, 1080p stream with a full-res link. A Haiku sub-agent drove it ~13min autonomously via the debug port.

## OPEN — work these (priority order)
1. **Verify the direction fix (a5c55cc) wheels-off:** stick fwd→fwd, reverse→reverse (should be unchanged); collision in ASSIST cuts FORWARD and allows reverse; AUTO/RTH creep FORWARD not backward (fixed by derivation — eyeball it).
2. **BLE Nimbus on the ESP32-S3** (my active request): firmware exists (`firmware/main/hid_host.c` + `esp_hid_gap.c`, BT enabled in sdkconfig, flashed). It builds/boots without crashing the display and relays HID reports up as protocol frame `CC02_T_GPAD` — BUT enabling BLE causes **intermittent I2C timeouts** on the IMU/touch bus (BLE scan starves core-0 I2C). **Fix: pin control_task to core 1 (away from Bluedroid on core 0) and/or widen the BLE scan interval.** Then confirm the Nimbus actually connects (it's MFi/BLE — may be picky) and map its HID report bytes to steer/throttle on the Pi side (observe raw reports via the relay). Note: S3 is BLE-only so it can NEVER host the Switch Pro Controller (BT-Classic → stays on the Pi). If the I2C fix or the Nimbus connection proves unworkable, the fallback is to bulletproof the Pi's BlueZ pairing instead. There is a pre-BLE firmware you can rebuild from git history if the ESP needs to be rock-solid meanwhile.
3. **Floor vision fails on non-grass** (dirt/concrete → all 9 columns read "blocked" → false collision + AUTO can't move). It's a lawn-tuned HSV colour model with an adapt-freeze chicken-and-egg (freezes learning while collision=True, so it never learns a surface it starts blocked on). Broaden to "drivable ground" or fix the freeze. (On grass it's fine; on the bench pointed across a room it correctly sees no floor.)
4. Bind the debug port to localhost before the car goes on any untrusted network.
5. Pro Controller (Pi/BlueZ) pairing is flaky — needed `ClassicBondedOnly=false` in /etc/bluetooth/input.conf; a pairing loop must NOT `bluetoothctl remove` existing records (self-defeating). `scripts/pair_controller.sh`.

## Working style I like
Read code before theorising; verify claims with the debug port / actual output, not assertions; keep reports short (lead with result); patch→restart→verify→sync-to-repo→push each change; flag hardware realities in one sentence and do the safe version. You've been excellent — keep going.
