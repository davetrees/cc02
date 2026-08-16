# AV Controller — Novastar LED Screen Remote

## Hardware
- Board: Waveshare ESP32-S3-Touch-LCD-3.5B
- Display: 320x480 SPI (AXS15231B), rotated 90° to 480x320 landscape
- Touch: FT6336
- PMU: AXP2101 (battery management)
- Audio: ES8311 codec + speaker
- Serial: /dev/ttyACM0
- Device WiFi IP: 192.168.1.138 (when connected)

## ABSOLUTE RULES (violating these = session terminated)

1. **DO NOT MODIFY** any file under `components/esp_bsp/` or `components/esp_lv_port/`. These are from the Waveshare demo and WORK. Touch them and the display breaks. ONE EXCEPTION: ALDO1 voltage in bsp_axp2101.cpp has been set to 3500mV (user approved).
2. **DO NOT MODIFY** `components/XPowersLib/`. This is a third-party library.
3. **ONE CHANGE AT A TIME.** Make one fix, build, flash, verify via serial or OTA. Do not batch changes.
4. **NEVER change sdkconfig.defaults or partitions.csv** without explicit user approval.
5. **Read the code before changing it.** Every time.
6. **After every flash**, read serial output or check device responds within 15 seconds.
7. **If 2 attempts fail**, STOP. Tell the user. Do not keep iterating.
8. **User observations are diagnostic data.** Act on them immediately. Do not question hardware.

## Build & Flash

```
cd /mnt/2tbstorage/Claude/last_chance_LED_controller
source ~/esp/esp-idf/export.sh
idf.py build
```

Serial flash (device on /dev/ttyACM0):
```
sudo chmod 666 /dev/ttyACM0
idf.py -p /dev/ttyACM0 flash monitor
```

OTA flash (device on WiFi at 192.168.1.138):
```
curl -X POST --data-binary @build/av-controller.bin http://192.168.1.138:8080/ota
```

## Architecture

- `main/app_main.c` — Entry point, init sequence (PMU → expander → display → touch → WiFi → brightness → LVGL → audio → OTA → UI)
- `components/ui/ui.c` — Main control screen (buttons, slider, status, standby, toast). Buttons fire on LV_EVENT_PRESSED for instant response. Gradients toggled via `#define ENABLE_GRADIENTS 0` at top of file.
- `components/ui/ui_settings.c` — On-device settings screen (currently unused, settings via web)
- `components/novastar/novastar.c` — TCP binary protocol to VSX400/VX600. Uses FreeRTOS queue + background task for non-blocking sends. Supports: screen on/off/freeze, brightness, preset load, layer source switch.
- `components/novastar/include/novastar.h` — Public API. Call novastar_init() before use.
- `components/storage/storage.c` — NVS settings persistence (WiFi, Nova IP/port, brightness presets, backlight, UI theme, standby, audio, battery/wifi display)
- `components/storage/btn_config.c` — Per-button NVS persistence: label, bg_color, fg_color, command_type, command_param, command_param2
- `components/ota_update/ota_update.c` — HTTP web UI on port 8080 with: settings GET/POST, button config GET/POST, test commands, touch test, OTA flash, reboot
- `components/wifi_sta/wifi_sta.c` — WiFi STA with auto-connect, AP fallback
- `components/power_mgmt/power_mgmt.c` — Battery monitoring every 5s, auto-dim at 20%/10%, shutdown at 5%
- `components/audio/audio.c` — Click/error sounds via ES8311

## DO NOT TOUCH (Waveshare demo files)
- `components/esp_bsp/*` — Display, touch, I2C, AXP2101 BSP
- `components/esp_lv_port/*` — LVGL port (task, mutex, display driver)
- `components/XPowersLib/*` — AXP2101 C++ driver

## Completed work (2026-03-30)

### Bug fixes applied
1. ✅ Slider now syncs when DAY/NIGHT buttons pressed (s_brightness + s_slider updated)
2. ⏸️ Boot backlight — parked, may be hardware limitation not software
3. ⏸️ s_active_backlight init — parked with #2
4. ⏸️ Web settings backlight apply — parked with #2
5. ⏸️ power_mgmt backlight override — parked with #2

### Features implemented
- ✅ Command configurability: each button has configurable command_type (CMD_BRIGHTNESS, CMD_SCREEN_NORMAL, CMD_SCREEN_BLACKOUT, CMD_SCREEN_FREEZE, CMD_PRESET_LOAD, CMD_LAYER_SOURCE) with params, stored in NVS, configurable via web UI dropdown
- ✅ Non-blocking TCP sends: FreeRTOS queue in novastar.c, background task does connect+send, LVGL thread never blocks
- ✅ Buttons fire on LV_EVENT_PRESSED (instant touch response)
- ✅ Gradients made toggleable via #define ENABLE_GRADIENTS (currently 0/off for brightness)
- ✅ ALDO1 bumped to 3.5V (marginal brightness gain, user approved)
- ✅ Novastar protocol: screen on/off/freeze, brightness, preset load, layer source switching all implemented

### Remaining work
- Backlight bugs #2-5 — parked pending further investigation, likely hardware limitation
- WiFi AP fallback — status unknown, needs testing
- Web UI could use polish (theme settings don't rebuild UI live, require reboot)
- Simulator is a separate project at /mnt/2tbstorage/Claude/novastar-simulator/

## Novastar VX Pro protocol reference
- Header: 55 AA (request) / AA 55 (response)
- Checksum: SUM = (sum of bytes 0x02..end_of_data) + 0x5555, little-endian [SUM_L, SUM_H]
- Optional 0x0D terminator after checksum
- Broadcast header (brightness): FE FF 01 FF FF FF 01 00
- Direct header (display/preset/layer): FE 00 00 00 00 00 01 00

### Register addresses (little-endian at offset 0x0C-0x0F)
- Brightness: 0x02000001 (01 00 00 02) — 1 byte data, 0x00-0xFF
- Display mode: 0x13000004 (04 00 00 13) — 2 bytes [mode, 0x00]. Normal=0x03, Freeze=0x04, Blackout=0x05, Test=0x06
- Preset load: 0x13510100 (00 01 51 13) — 1 byte, zero-indexed (Preset 1=0x00)
- Preset save: 0x13510102 (02 01 51 13) — 1 byte
- Layer source: 0x13020012 + (window × 0x30) — 3 bytes [card_slot, 0x00, 0x00]. HDMI=0, SDI1=1, SDI2=2, DVI1-4=3-6
