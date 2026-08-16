# AV Controller v2 — Design Spec

## Overview

Enhancements to the ESP32-S3 Novastar VSX400 touchscreen controller. All features build on the existing codebase: LVGL 8 UI, esp_lv_port, NovaStar TCP protocol with 16-bit checksums, OTA via HTTP, web settings on port 8080, NVS persistence.

Board: Waveshare ESP32-S3-Touch-LCD-3.5B. Display: AXS15231B QSPI 480x320 landscape. Touch: AXS15231B I2C. PMIC: AXP2101 via XPowersLib. Audio: ES8311 codec + speaker.

---

## 1. Power Management (AXP2101)

### Components
- Copy `XPowersLib` and `bsp_axp2101.cpp/.h` from the demo (`/mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/`). Do not modify them.
- New component: `power_mgmt` — wraps AXP2101 reads and implements threshold logic.

### Behavior
- Read battery voltage and percentage every 5 seconds via a FreeRTOS task.
- Display battery icon in top-right corner of control screen (hideable, see section 7).
- Thresholds:
  - **> 20%**: Normal operation. Icon shows approximate level (4 bars: >75%, >50%, >25%, >0%).
  - **20%**: Auto-dim backlight to 50% of current active setting. Icon turns amber.
  - **10%**: Dim to 25%. Icon turns red.
  - **5%**: Save settings to NVS, display "Shutting down" for 1 second, power off via AXP2101.
- When USB power is connected (charging), show a charging indicator instead of battery level.
- Battery percentage exposed in `/api/settings` response as `battery_pct` and `battery_charging` fields.

### NVS Keys
None — thresholds are hardcoded. Battery state is runtime only.

---

## 2. WiFi AP Fallback

### Boot Sequence
1. If WiFi SSID is configured in NVS:
   - Try STA connection, 3 attempts, 5-second gap between retries.
   - If all 3 fail: start AP mode.
2. If WiFi SSID is empty (first boot): go straight to AP mode.

### AP Mode
- SSID: `NovaStar Remote` (no password, open network).
- IP: `192.168.4.1` (ESP-IDF default for AP).
- Web UI served on `192.168.4.1:8080` — same settings page as STA mode.
- Status bar on LCD shows `AP: NovaStar Remote` in amber.
- OTA endpoint also available in AP mode.

### Reconnection
- When user saves new WiFi credentials via web UI in AP mode, the device reboots and tries STA with the new creds.
- No background STA retry while in AP mode — clean separation, avoids instability.

### Changes to `wifi_sta` Component
- Add `app_wifi_start_ap(void)` function.
- Add `WIFI_STA_AP_MODE` to the `wifi_sta_status_t` enum.
- Move retry logic into `wifi_sta.c` with a configurable max retry count.
- `app_main` calls a new `app_wifi_auto_connect()` that handles the STA-then-AP sequence.

---

## 3. Configurable Buttons

### What's Configurable
Six controls: ON, OFF, DAY, NIGHT, +, -. Each has:
- **Label text**: string, max 12 chars. Defaults: "ON", "OFF", "DAY", "NIGHT", "+", "-".
- **Background color**: 24-bit hex. Defaults: current muted green/red/blue/navy/charcoal.
- **Text color**: 24-bit hex. Defaults: current values.
- **Brightness parameter**: uint8 0-100. ON default=100, OFF default=0, DAY default=100, NIGHT default=30. +/- use the slider value (no separate parameter).

### Command Mapping
- ON: sends `novastar_set_brightness(on_bright)` — configurable, default 100%.
- OFF: sends `novastar_set_brightness(0)` — always 0%, the "off brightness."
- DAY: sends `novastar_set_brightness(day_bright)` + sets LCD backlight to `day_backlight`.
- NIGHT: sends `novastar_set_brightness(night_bright)` + sets LCD backlight to `night_backlight`.
- +/-: adjusts slider by 5 and sends `novastar_set_brightness(slider_value)`.
- Slider: sends `novastar_set_brightness(slider_value)` on change.

Note: `novastar_screen_on()` and `novastar_screen_off()` are no longer called directly. ON/OFF are now brightness commands. The screen on/off register commands remain in `novastar.c` for potential future use but are not wired to UI buttons.

### Web UI
Settings page gets a "Buttons" section with a row per button: label input, color pickers, brightness input. Each row has a "Test" button that fires the configured command immediately (same as pressing the LCD button).

### NVS Keys
- `btn_on_lbl`, `btn_on_bg`, `btn_on_fg`, `btn_on_brt` (repeat for off, day, night, plus, minus).
- Total: 24 new NVS keys. All optional — defaults used if absent.

### Storage Changes
- `settings_t` struct grows with button config fields.
- Alternatively: button config stored as a separate NVS namespace `btn_config` to keep `settings_t` manageable. Separate load/save functions.

**Decision: separate namespace `btn_config`** with its own struct `btn_config_t` and `btn_config_load()`/`btn_config_save()` functions. Keeps the storage component clean.

---

## 4. UI Customization

### Configurable via Web UI
- **Background color**: 24-bit hex. Default: `#000000` (pure black).
- **Title text**: string, max 20 chars. Default: "LED SCREEN".
- **Title bar color**: 24-bit hex. Default: `#101018`.

### NVS Keys
- `ui_bg_color`, `ui_title_text`, `ui_title_color`. Stored in main `ledremote` namespace alongside WiFi/NovaStar settings.

### Implementation
- `storage.h` `settings_t` gains 3 fields: `bg_color` (uint32), `title_text[21]`, `title_color` (uint32).
- `build_control_screen()` reads these at build time.
- Web UI settings page gets a "Theme" section with the 3 fields.

---

## 5. Backlight Standby & Wake

### Behavior
- After `standby_timeout_s` seconds of no touch input (default 60), dim backlight to `standby_brightness` percent (default 5%, minimum 1%).
- On first touch after standby:
  - Restore backlight to previous active level.
  - **Consume the touch** — do not pass to LVGL. The touch only wakes the display.
  - Reset the idle timer.
- Subsequent touches work normally.

### Implementation
- New module or addition to `ui.c`: an LVGL timer that runs every 1 second, checks `lv_disp_get_inactive_time()`. If it exceeds the threshold, dim the backlight and set a `s_standby_active` flag.
- In `touchpad_read()` in `app_main.c`: if `s_standby_active` is true and touch is detected, restore backlight, set flag to false, and report `LV_INDEV_STATE_REL` (suppress the touch). Next poll returns normal state.
- Expose `ui_is_standby()` and `ui_wake()` for the touchpad_read callback to use.

### NVS Keys
- `standby_timeout` (uint16, seconds, default 60). 0 = disabled.
- `standby_bright` (uint8, percent, default 5).

### Web UI
Settings page gets a "Display" section: standby timeout input, standby brightness input.

---

## 6. Audio Feedback

### Hardware
- ES8311 codec on I2C, connected to I2S output and speaker. Use `bsp_es8311.c/.h` from the demo.

### Sounds
- **Click**: 1kHz sine, 50ms duration. Played on every successful button press.
- **Error**: 400Hz sine, 200ms duration. Played when TCP send fails.
- Generated programmatically — no audio files needed. Fill an I2S DMA buffer with the sine wave and send it.

### Implementation
- New component: `audio` with `audio_init()`, `audio_click()`, `audio_error()`.
- `audio_init()` configures I2S + ES8311 at boot.
- `audio_click()` and `audio_error()` are non-blocking — they write to I2S from a small pre-computed buffer. If a sound is already playing, skip (don't queue).
- Button handlers in `ui.c` call `audio_click()` on press and `audio_error()` if `novastar_*` returns `ESP_FAIL`.

### NVS Keys
- `audio_enabled` (uint8, 0 or 1, default 1).

### Web UI
Settings page: "Sound" toggle (on/off).

---

## 7. Hideable Indicators

### Settings
- `show_battery` (uint8, 0 or 1, default 1).
- `show_wifi` (uint8, 0 or 1, default 1).

### Implementation
- Battery icon (top-right) and WiFi status label (bottom-center) check their respective flags.
- When hidden: `lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN)`. Layout unchanged — buttons don't move.
- `ui_update_status()` skips updating hidden indicators.

### Web UI
Settings page: two toggles in a "Display" section alongside standby settings.

---

## 8. Visual Confirmation

### Button Press Feedback
- On `LV_EVENT_CLICKED`: button background flashes 30% lighter for 200ms, then returns to configured color. Implemented via `lv_obj_set_style_bg_color` with a one-shot LVGL timer to restore.
- Simultaneously, a toast label appears at the bottom of the screen (above the WiFi status) showing the command sent: e.g. "ON 100%", "NIGHT 30%", "Brightness 65%". Visible for 1.5 seconds, then hidden.
- On TCP failure: button flashes red (`#CF6060`) for 200ms. Toast shows "Error" in red. Error sound plays.

### Implementation
- Add `s_toast_label` to `ui.c`, positioned at bottom-center, hidden by default.
- `ui_toast(const char *msg, bool is_error)` — sets text, shows label, starts 1.5s hide timer.
- Each button handler calls `ui_toast()` after the novastar call, checking the return value for errors.

---

## 9. Slider Layout Fix

### Problem
Slider currently overlaps the +/- buttons.

### Fix
Reduce slider width. Current calculation: `sl_w = SCR_W - 2*m - 2*pm_sz - 24`. Add more gap: `sl_w = SCR_W - 2*m - 2*pm_sz - 48` (12px extra gap on each side). Slider starts 24px after the - button and ends 24px before the + button.

---

## NVS Summary

### Namespace `ledremote` (existing, extended)
| Key | Type | Default |
|-----|------|---------|
| wifi_ssid | string | "" |
| wifi_pass | string | "" |
| nova_ip | string | "192.168.1.2" |
| nova_port | u16 | 5200 |
| day_brt | u8 | 100 |
| night_brt | u8 | 30 |
| day_bl | u8 | 100 |
| night_bl | u8 | 40 |
| on_brt | u8 | 100 |
| ui_bg | u32 | 0x000000 |
| ui_title | string | "LED SCREEN" |
| ui_title_clr | u32 | 0x101018 |
| standby_time | u16 | 60 |
| standby_brt | u8 | 5 |
| audio_on | u8 | 1 |
| show_batt | u8 | 1 |
| show_wifi | u8 | 1 |

### Namespace `btn_config` (new)
| Key | Type | Default |
|-----|------|---------|
| on_lbl | string | "ON" |
| on_bg | u32 | 0x1a3a2a |
| on_fg | u32 | 0x4CAF6a |
| off_lbl | string | "OFF" |
| off_bg | u32 | 0x3a1a1a |
| off_fg | u32 | 0xCF6060 |
| day_lbl | string | "DAY" |
| day_bg | u32 | 0x0f1f30 |
| day_fg | u32 | 0xe0e0e0 |
| night_lbl | string | "NIGHT" |
| night_bg | u32 | 0x0a0a18 |
| night_fg | u32 | 0xe0e0e0 |
| plus_lbl | string | "+" |
| plus_bg | u32 | 0x111111 |
| plus_fg | u32 | 0xe0e0e0 |
| minus_lbl | string | "-" |
| minus_bg | u32 | 0x111111 |
| minus_fg | u32 | 0xe0e0e0 |

---

## Component Summary

| Component | Status | Notes |
|-----------|--------|-------|
| `esp_bsp` | Existing | Add back `bsp_axp2101.cpp/.h`, `bsp_es8311.c/.h` from demo. Do not modify demo code. |
| `XPowersLib` | Re-add | Copy from demo. Do not modify. |
| `storage` | Extend | Add new NVS keys to `settings_t`. Add `btn_config_t` with separate namespace. |
| `wifi_sta` | Extend | Add AP mode, retry logic, `WIFI_STA_AP_MODE` status. |
| `power_mgmt` | New | AXP2101 battery monitoring task, threshold logic. |
| `audio` | New | ES8311 init, click/error tone generation via I2S. |
| `novastar` | No change | Protocol is correct. ON/OFF buttons now send brightness instead of screen on/off register commands. |
| `ui` | Extend | Configurable colors/labels, battery icon, toast, standby wake, button flash, slider gap fix. |
| `ota_update` | Extend | Web UI gains buttons config section, theme section, display section, sound toggle, indicator toggles, test buttons. |

---

## Web UI Settings Page Layout

1. **Settings** — WiFi SSID, WiFi Pass, VSX400 IP, TCP Port
2. **Buttons** — Per-button: label, bg color, text color, brightness, [Test] button
3. **Brightness Presets** — Day LED %, Day LCD %, Night LED %, Night LCD %
4. **Theme** — Background color, Title text, Title bar color
5. **Display** — Standby timeout, Standby brightness, Show battery toggle, Show WiFi toggle
6. **Sound** — Audio enabled toggle
7. **Firmware Update** — File upload
8. **System** — IP, WiFi status, battery %, version, Reboot button
