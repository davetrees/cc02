# CC-02 Wiring — FROZEN 2026-08-16

Board: Waveshare **ESP32-S3-Touch-LCD-3.5B** (AXS15231B QSPI LCD, QMI8658 IMU, AXP2101 PMU).
Pins verified against the official 3.5B schematic (J8 = 2x16 2.54mm "PinOut" header).

## ESP PWM outputs (LEDC, 50 Hz, 1000–2000 µs, neutral 1500 µs)

Header pin numbers per the official Waveshare pinout diagram (2×16, odd column = left).

| Signal          | ESP GPIO | Header pin | Notes |
|-----------------|----------|------------|-------|
| STEER_PULSE     | GPIO41   | pin 13     | Camera-DVP GPIO — free because the ESP camera FPC is never used in this build. |
| THROTTLE_PULSE  | GPIO42   | pin 15     | Same. Adjacent to pin 13 in the left column. |
| SIGNAL GND      | —        | pin 29/30  | Bond to ESC/servo ground. Bottom of left column, before 3V3. |

(GPIO43/44 also exist on pins 27/25, silk-labeled TXD/RXD — kept free for console/debug.)
Do not touch: QSPI LCD (1–5,12), backlight (6), I2C bus (7/8), SD (9/10/11), strap (0).
If an ESP camera is ever fitted later, GPIO41/42 must be vacated — move to GPIO43/44.

## Servo / ESC (Tamiya CC-02, stock)

```
                    +--------------------------- car battery pack (XT60 Y-split)
                    |                       |
              Branch A (drivetrain)   Branch B (computers)
                    |                       |
                   ESC                 5V/5A+ UBEC
                госп|  \__ BEC 5-6V          |
        motor <----+      |            Pi 5 USB-C power
                          |                  |
   Servo red  <-----------+            Pi USB-A ----USB cable----> Waveshare USB-C
   Servo brown/black ---- GND (common)                             (5V power + CDC data)
   Servo orange (sig) <-- ESP GPIO43 (J8 pin 26)
   ESC signal        <-- ESP GPIO44 (J8 pin 28)
   ESC sig GND + servo GND --- J8 pin 29/30 (ESP GND)
```

- ESC BEC red powers the SERVO ONLY. **Do NOT connect ESC BEC red to the ESP 5V pin**
  (the Waveshare is already powered by the Pi's USB — avoid back-feed).
- **Never power the Pi 5 from the ESC BEC** — Pi 5 needs the dedicated 5 V ≥5 A UBEC.
- PWM is 3.3 V signal-level only. Do not power the servo from ESP 3.3 V.
- Motor leads stay on the ESC. Receiver removed.

## Pi side

- Camera: Logitech C930e on Pi USB (per current build — CSI Camera Module 3 not fitted).
- AI HAT+ 2 (Hailo-10H): fitted and USED — YOLO runs on the Hailo (yolov8m_h10.hef),
  with automatic CPU-YOLO fallback if the HAT is absent/fails. Its fan is also the Pi's cooling.
- Pi power note: bench brick negotiating 5V/3A caused undervoltage crashes AND a USB
  over-current port shutdown; `usb_max_current_enable=1` is set in config.txt as a stopgap.
  On the car, the 5 V/5 A UBEC is mandatory.
- Waveshare enumerates on the Pi as `/dev/serial/by-id/*Espressif*` (USB-Serial-JTAG,
  MAC 20:6e:f1:9a:16:30). Never hardcode ttyUSB0.

## Failsafe behavior (owned by the ESP, works with Pi dead)

- No CMD frame for `failsafe_timeout_ms` (default 200 ms) → throttle 1500 µs,
  steer center (configurable hold), LCD shows LINK LOST.
- Tip: |roll| or |pitch| > threshold (default 45°) or |gyro| > 400 dps → local throttle cut.
- Touch ESTOP on LCD → immediate local neutral, no Pi involved.
