# Novastar VSX400 LED Screen Remote

## Overview

A dedicated touchscreen remote for the Novastar VSX400 LED screen processor, built on the Waveshare ESP32-S3-Touch-LCD-3.5B. Connects to the VSX400 over WiFi via TCP port 5200 using the Novastar binary protocol.

## Architecture

**Hardware:** Waveshare ESP32-S3-Touch-LCD-3.5B
- Display: 3.5" IPS, 320x480, 262K colors (16-bit)
- Controller: AXS15231B (integrated LCD + touch), QSPI interface
- Processor: ESP32-S3R8 (dual-core 240MHz, 8MB PSRAM, 16MB Flash)
- Touch: Capacitive, integrated in AXS15231B via I2C
- Onboard: QMI8658 IMU, PCF85063 RTC, AXP2101 PMU, ES8311 audio codec (unused)

**Pin assignments (3.5B):**
- QSPI LCD: CS=GPIO45, CLK=GPIO47, D0=GPIO21, D1=GPIO48, D2=GPIO40, D3=GPIO39
- I2C (touch): SDA=GPIO8, SCL=GPIO9
- Backlight: GPIO2

**Key difference from the 4.3" variant:** The 3.5B uses AXS15231B with QSPI (not ST7796 with SPI), and touch is integrated in the display controller (not a separate FT6336). The `lcd_init` and `touch` components must be rewritten for this hardware. The `board.h` pin definitions must be updated. Resolution remains 320x480.

**Components (kept, unchanged):**
- `lvgl_port` — LVGL task, DMA buffers, flush callback

**Components (rewritten for 3.5B hardware):**
- `board` — updated pin definitions for 3.5B
- `lcd_init` — AXS15231B QSPI display init (replaces ST7796 SPI)
- `touch` — AXS15231B integrated touch via I2C (replaces FT6336)

**Components (modified):**
- `storage` — simplified to NVS-only for settings (no FFat, no config.json)
- `wifi_sta` — replaces `wifi_ap`, connects to existing network as station

**Components (new):**
- `novastar` — TCP client for VSX400 binary protocol

**Components (removed):**
- `http_api`, `rs232`, `cmd_dispatch`, `devices`, `webapp`

**ESP-IDF managed components (new):**
- `espressif/esp_lcd_axs15231b` — LCD driver for AXS15231B

**UI (rewritten):**
- `ui` — two screens: control screen + settings screen

## Novastar Protocol

Binary protocol over TCP port 5200. Fire-and-forget (no response parsing required, but query support added if the processor responds).

### Commands

**Normal Display (ON):**
```
55 aa 00 38 fe 00 00 00 00 00 01 00 04 00 00 13 01 00 03 a7 56 0d
```

**Screen Black (OFF):**
```
55 aa 00 37 fe 00 00 00 00 00 01 00 04 00 00 13 01 00 05 a8 56 0d
```

**Screen Freeze:**
```
55 aa 00 35 fe 00 00 00 00 00 01 00 04 00 00 13 01 00 04 a5 56 0d
```

**Brightness (0-100%):**
Template: `55 aa 00 00 fe ff 01 ff ff ff 01 00 01 00 00 02 01 00 <VAL> <CHK> 5a 0d`
- `VAL` = `round(percent * 255 / 100)` (0x00-0xFF)
- `CHK` = `(0x55 + VAL) & 0xFF`

**Presets (1-16):**
Template: `55 aa 00 d6 fe 00 00 00 00 00 01 00 00 01 51 13 01 00 <N> <CHK> 5a 0d`
- `N` = preset index (0x00-0x0F)
- `CHK` = `(0x3B + N) & 0xFF`

**Input Select:**
- HDMI 1: `55 AA 00 00 FE 00 00 00 00 00 01 00 12 00 02 13 03 00 00 00 00 7e 56 0d`
- DVI: `55 AA 00 00 FE 00 00 00 00 00 01 00 12 00 02 13 03 00 02 00 00 80 56 0d`

## UI Design

### Control Screen (main)

320x480 portrait layout matching the provided mockup:

```
+----------------------------------+
|          LED SCREEN              |  <- Title bar (10s long-press = settings)
|  (blue gradient header bar)      |
+----------------------------------+
|                                  |
|   +----------+  +----------+    |
|   |    ON    |  |   OFF    |    |  <- Green / Red buttons
|   +----------+  +----------+    |
|                                  |
|   +----------+  +----------+    |
|   |   DAY    |  |  NIGHT   |    |  <- Light blue / Dark blue buttons
|   |   (sun)  |  |  (moon)  |    |
|   +----------+  +----------+    |
|                                  |
|  [-] ===|=========== [sun] [+]  |  <- Brightness slider with +/- buttons
|         Brightness               |
+----------------------------------+
```

**Behavior:**
- ON button: sends Normal Display command, button highlights green
- OFF button: sends Screen Black command, button highlights red
- DAY button: sends brightness command at configured day%, updates slider position
- NIGHT button: sends brightness command at configured night%, updates slider position
- Slider: sends brightness command on release (or throttled during drag)
- +/- buttons: increment/decrement brightness by 5%, send command, update slider

**State tracking:**
- Current brightness % (local, updated by slider/buttons/day/night)
- ON/OFF state (visual indicator only, tracks last command sent)

### Settings Screen

Accessed via 10-second long-press on the "LED SCREEN" title bar.

```
+----------------------------------+
|          SETTINGS                |
+----------------------------------+
|  WiFi SSID:    [____________]   |
|  WiFi Pass:    [____________]   |
|  VSX400 IP:    [____________]   |
|  TCP Port:     [5200________]   |
|  Day Bright:   [100_________] % |
|  Night Bright: [30__________] % |
|                                  |
|   +----------+  +----------+    |
|   |   SAVE   |  |   BACK   |    |
|   +----------+  +----------+    |
+----------------------------------+
```

**Input:** LVGL keyboard (on-screen) for text fields, numeric keyboard for number fields.

**On SAVE:** Write all values to NVS, reconnect WiFi if credentials changed, return to control screen.

**On BACK:** Discard changes, return to control screen.

## Storage (NVS)

| Key | Type | Default |
|-----|------|---------|
| `wifi_ssid` | string | "" |
| `wifi_pass` | string | "" |
| `nova_ip` | string | "192.168.1.50" |
| `nova_port` | uint16 | 5200 |
| `day_bright` | uint8 | 100 |
| `night_bright` | uint8 | 30 |

## WiFi STA

- On boot, read SSID/password from NVS
- If empty, skip connection and show settings screen
- If set, attempt connection with 10s timeout
- Show connection status on the control screen (small indicator)
- Auto-reconnect on disconnect

## Novastar Component API

```c
// Initialize (nothing to do at init, stateless TCP)
void novastar_init(void);

// Set target (reads from NVS or passed in)
void novastar_set_target(const char *ip, uint16_t port);

// Commands (fire-and-forget, open socket, send, close)
esp_err_t novastar_screen_on(void);
esp_err_t novastar_screen_off(void);
esp_err_t novastar_set_brightness(uint8_t percent);  // 0-100
esp_err_t novastar_load_preset(uint8_t preset);      // 1-16
esp_err_t novastar_set_input_hdmi1(void);
esp_err_t novastar_set_input_dvi(void);
```

Each function opens a TCP socket, sends the command, closes the socket. Returns `ESP_OK` on successful send, `ESP_FAIL` on connection error.

## Boot Sequence

1. LCD init + backlight
2. LVGL init + touch init
3. Load settings from NVS
4. Start WiFi STA (non-blocking)
5. Show control screen (or settings if no WiFi configured)
6. Start LVGL timer task

## Non-Goals

- No web configuration interface
- No RS-232 support
- No multi-device support
- No activity/macro system
- No OTA updates (for now)
