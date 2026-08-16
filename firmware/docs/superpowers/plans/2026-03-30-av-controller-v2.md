# AV Controller v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add power management, WiFi AP fallback, configurable buttons/theme, backlight standby, audio feedback, visual confirmation, and hideable indicators to the ESP32-S3 NovaStar touchscreen controller.

**Architecture:** Extend existing component structure. Copy demo libraries (XPowersLib, bsp_axp2101, bsp_es8311) without modification. New components `power_mgmt` and `audio` wrap the demo BSP. Extended `storage` holds all new NVS settings. Extended `ui` renders configurable buttons, battery icon, toast, and standby. Extended `wifi_sta` handles AP fallback. Extended `ota_update` (web UI) adds all new config sections.

**Tech Stack:** ESP-IDF 5.3, LVGL 8.4, C, XPowersLib (C++), ESP Codec Dev, FreeRTOS. Build: `idf.py build`. Flash: OTA via `curl -X POST --data-binary @build/av-controller.bin http://<ip>:8080/ota`.

**Demo library source:** `/mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/`

**Project root:** `/mnt/2tbstorage/Waveshare/4.3inlcdtouch/AV Controller/av-controller`

---

## File Map

### New files
| File | Purpose |
|------|---------|
| `components/XPowersLib/` | Copied from demo. Do not modify. |
| `components/esp_bsp/bsp_axp2101.cpp` | Copied from demo. Do not modify. |
| `components/esp_bsp/bsp_axp2101.h` | Copied from demo. Do not modify. |
| `components/esp_bsp/bsp_es8311.c` | Copied from demo. Do not modify. |
| `components/esp_bsp/bsp_es8311.h` | Copied from demo. Do not modify. |
| `components/power_mgmt/power_mgmt.c` | Battery monitoring task, threshold logic, shutdown. |
| `components/power_mgmt/include/power_mgmt.h` | Public API: init, get_pct, is_charging, is_low. |
| `components/power_mgmt/CMakeLists.txt` | Component build file. |
| `components/audio/audio.c` | Tone generation, I2S playback via ES8311. |
| `components/audio/include/audio.h` | Public API: init, click, error. |
| `components/audio/CMakeLists.txt` | Component build file. |
| `components/storage/btn_config.c` | Button config NVS load/save (namespace `btn_config`). |
| `components/storage/include/btn_config.h` | `btn_config_t` struct and API. |

### Modified files
| File | Changes |
|------|---------|
| `components/esp_bsp/CMakeLists.txt` | Add `bsp_axp2101.cpp`, `bsp_es8311.c`, add `XPowersLib` and `espressif__esp_codec_dev` to REQUIRES. |
| `components/storage/storage.c` | Add new NVS keys: `on_brt`, `ui_bg`, `ui_title`, `ui_title_clr`, `standby_time`, `standby_brt`, `audio_on`, `show_batt`, `show_wifi`. |
| `components/storage/include/storage.h` | Extend `settings_t` with new fields. |
| `components/wifi_sta/wifi_sta.c` | Add AP mode, `app_wifi_start_ap()`, `app_wifi_auto_connect()`, `WIFI_STA_AP_MODE` enum value. |
| `components/wifi_sta/include/wifi_sta.h` | Add new enum value and function declarations. |
| `components/ui/ui.c` | Configurable button colors/labels, battery icon, toast, standby timer, button flash, slider gap fix. |
| `components/ui/include/ui.h` | Add `ui_toast()`, `ui_is_standby()`, `ui_wake()`, `ui_show_touch_test()`. |
| `components/ota_update/ota_update.c` | Web UI: buttons config, theme, display, sound, indicator toggles, test buttons, battery info. Add `/api/btn_config` GET/POST, extend `/api/settings`. |
| `components/ota_update/CMakeLists.txt` | Add `power_mgmt`, `audio` to REQUIRES. |
| `main/app_main.c` | Init power_mgmt, audio, call `app_wifi_auto_connect()`, standby wake logic in `touchpad_read()`. |
| `main/idf_component.yml` | Add `espressif/esp_codec_dev: "^1.3.4"`. |

---

### Task 1: Copy Demo Libraries (XPowersLib, bsp_axp2101, bsp_es8311)

**Files:**
- Create: `components/XPowersLib/` (entire directory tree from demo)
- Create: `components/esp_bsp/bsp_axp2101.cpp`
- Create: `components/esp_bsp/bsp_axp2101.h`
- Create: `components/esp_bsp/bsp_es8311.c`
- Create: `components/esp_bsp/bsp_es8311.h`
- Modify: `components/esp_bsp/CMakeLists.txt`
- Modify: `main/idf_component.yml`

- [ ] **Step 1: Copy XPowersLib from demo**

```bash
cp -r /mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/XPowersLib \
  components/XPowersLib
```

- [ ] **Step 2: Copy bsp_axp2101 files from demo**

```bash
cp /mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/esp_bsp/bsp_axp2101.cpp \
  components/esp_bsp/bsp_axp2101.cpp
cp /mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/esp_bsp/bsp_axp2101.h \
  components/esp_bsp/bsp_axp2101.h
```

- [ ] **Step 3: Copy bsp_es8311 files from demo**

```bash
cp /mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/esp_bsp/bsp_es8311.c \
  components/esp_bsp/bsp_es8311.c
cp /mnt/2tbstorage/Waveshare/ESP32-S3-Touch-LCD-3.5B-Demo/ESP-IDF/05_lvgl_example/components/esp_bsp/bsp_es8311.h \
  components/esp_bsp/bsp_es8311.h
```

- [ ] **Step 4: Update esp_bsp CMakeLists.txt**

```cmake
idf_component_register(SRCS "bsp_i2c.c" "bsp_display.c" "bsp_touch.c" "bsp_axp2101.cpp" "bsp_es8311.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_lcd" "driver" "espressif__esp_lcd_axs15231b" "XPowersLib" "espressif__esp_codec_dev")
```

- [ ] **Step 5: Add esp_codec_dev to main/idf_component.yml**

```yaml
dependencies:
  idf: ">=5.1"
  lvgl/lvgl: "~8.4.0"
  espressif/esp_lcd_axs15231b: "^1.0.0"
  espressif/esp_io_expander_tca9554: "^2.0.0"
  espressif/esp_codec_dev: "^1.3.4"
```

- [ ] **Step 6: Build to verify demo libs compile**

```bash
source ~/esp/esp-idf/export.sh && cd "/mnt/2tbstorage/Waveshare/4.3inlcdtouch/AV Controller/av-controller"
rm -rf build managed_components && idf.py build
```

Expected: Clean build. The bsp_axp2101 and bsp_es8311 aren't called yet but they compile.

- [ ] **Step 7: Commit**

```bash
git add components/XPowersLib components/esp_bsp/bsp_axp2101.* components/esp_bsp/bsp_es8311.* \
  components/esp_bsp/CMakeLists.txt main/idf_component.yml
git commit -m "feat: add XPowersLib, bsp_axp2101, bsp_es8311 from demo"
```

---

### Task 2: Extend Storage — New Settings Fields

**Files:**
- Modify: `components/storage/include/storage.h`
- Modify: `components/storage/storage.c`

- [ ] **Step 1: Extend settings_t in storage.h**

Add these fields after the existing `night_backlight` field:

```c
typedef struct {
    char wifi_ssid[33];
    char wifi_pass[65];
    char nova_ip[16];
    uint16_t nova_port;
    uint8_t day_bright;
    uint8_t night_bright;
    uint8_t day_backlight;
    uint8_t night_backlight;
    /* v2 additions */
    uint8_t on_bright;          /* ON button brightness, default 100 */
    uint32_t ui_bg_color;       /* Background color, default 0x000000 */
    char ui_title[21];          /* Title text, default "LED SCREEN" */
    uint32_t ui_title_color;    /* Title bar color, default 0x101018 */
    uint16_t standby_timeout;   /* Standby timeout seconds, 0=disabled, default 60 */
    uint8_t standby_bright;     /* Standby backlight %, default 5 */
    uint8_t audio_enabled;      /* Audio on/off, default 1 */
    uint8_t show_battery;       /* Show battery indicator, default 1 */
    uint8_t show_wifi;          /* Show wifi indicator, default 1 */
} settings_t;
```

- [ ] **Step 2: Update storage_init() to load new keys**

Add after the existing `load_u8(h, "night_bl", ...)` call:

```c
load_u8(h,  "on_brt",      &s_settings.on_bright, 100);
load_u32(h, "ui_bg",       &s_settings.ui_bg_color, 0x000000);
load_str(h, "ui_title",    s_settings.ui_title, sizeof(s_settings.ui_title), "LED SCREEN");
load_u32(h, "ui_title_clr",&s_settings.ui_title_color, 0x101018);
load_u16(h, "stby_time",   &s_settings.standby_timeout, 60);
load_u8(h,  "stby_brt",    &s_settings.standby_bright, 5);
load_u8(h,  "audio_on",    &s_settings.audio_enabled, 1);
load_u8(h,  "show_batt",   &s_settings.show_battery, 1);
load_u8(h,  "show_wifi",   &s_settings.show_wifi, 1);
```

Add a `load_u32` helper (doesn't exist yet):

```c
static void load_u32(nvs_handle_t h, const char *key, uint32_t *val, uint32_t def)
{
    if (nvs_get_u32(h, key, val) != ESP_OK) *val = def;
}
```

Set the same defaults in the `else` (no NVS) branch:

```c
s_settings.on_bright       = 100;
s_settings.ui_bg_color     = 0x000000;
strncpy(s_settings.ui_title, "LED SCREEN", sizeof(s_settings.ui_title));
s_settings.ui_title_color  = 0x101018;
s_settings.standby_timeout = 60;
s_settings.standby_bright  = 5;
s_settings.audio_enabled   = 1;
s_settings.show_battery    = 1;
s_settings.show_wifi       = 1;
```

- [ ] **Step 3: Update storage_save_settings() to write new keys**

Add after the existing `nvs_set_u8(h, "night_bl", ...)`:

```c
nvs_set_u8(h,  "on_brt",      s->on_bright);
nvs_set_u32(h, "ui_bg",       s->ui_bg_color);
nvs_set_str(h, "ui_title",    s->ui_title);
nvs_set_u32(h, "ui_title_clr",s->ui_title_color);
nvs_set_u16(h, "stby_time",   s->standby_timeout);
nvs_set_u8(h,  "stby_brt",    s->standby_bright);
nvs_set_u8(h,  "audio_on",    s->audio_enabled);
nvs_set_u8(h,  "show_batt",   s->show_battery);
nvs_set_u8(h,  "show_wifi",   s->show_wifi);
```

- [ ] **Step 4: Build to verify**

```bash
idf.py build
```

Expected: Clean build.

- [ ] **Step 5: Commit**

```bash
git add components/storage/
git commit -m "feat: extend settings_t with v2 NVS fields"
```

---

### Task 3: Button Config Storage (btn_config)

**Files:**
- Create: `components/storage/include/btn_config.h`
- Create: `components/storage/btn_config.c`
- Modify: `components/storage/CMakeLists.txt`

- [ ] **Step 1: Create btn_config.h**

```c
#pragma once

#include <stdint.h>
#include "esp_err.h"

#define BTN_LABEL_MAX 13  /* 12 chars + null */
#define BTN_COUNT 6

typedef struct {
    char label[BTN_LABEL_MAX];
    uint32_t bg_color;
    uint32_t fg_color;
} btn_style_t;

typedef struct {
    btn_style_t on;
    btn_style_t off;
    btn_style_t day;
    btn_style_t night;
    btn_style_t plus;
    btn_style_t minus;
} btn_config_t;

esp_err_t btn_config_load(btn_config_t *cfg);
esp_err_t btn_config_save(const btn_config_t *cfg);
void btn_config_defaults(btn_config_t *cfg);
```

- [ ] **Step 2: Create btn_config.c**

```c
#include <string.h>
#include "nvs.h"
#include "btn_config.h"

#define NVS_NS "btn_config"

void btn_config_defaults(btn_config_t *cfg)
{
    /* ON */
    strncpy(cfg->on.label, "ON", BTN_LABEL_MAX);
    cfg->on.bg_color = 0x1a3a2a;
    cfg->on.fg_color = 0x4CAF6a;
    /* OFF */
    strncpy(cfg->off.label, "OFF", BTN_LABEL_MAX);
    cfg->off.bg_color = 0x3a1a1a;
    cfg->off.fg_color = 0xCF6060;
    /* DAY */
    strncpy(cfg->day.label, "DAY", BTN_LABEL_MAX);
    cfg->day.bg_color = 0x0f1f30;
    cfg->day.fg_color = 0xe0e0e0;
    /* NIGHT */
    strncpy(cfg->night.label, "NIGHT", BTN_LABEL_MAX);
    cfg->night.bg_color = 0x0a0a18;
    cfg->night.fg_color = 0xe0e0e0;
    /* PLUS */
    strncpy(cfg->plus.label, "+", BTN_LABEL_MAX);
    cfg->plus.bg_color = 0x111111;
    cfg->plus.fg_color = 0xe0e0e0;
    /* MINUS */
    strncpy(cfg->minus.label, "-", BTN_LABEL_MAX);
    cfg->minus.bg_color = 0x111111;
    cfg->minus.fg_color = 0xe0e0e0;
}

static void load_btn(nvs_handle_t h, const char *prefix, btn_style_t *btn, const btn_style_t *def)
{
    char key[16];
    size_t len = BTN_LABEL_MAX;

    snprintf(key, sizeof(key), "%s_lbl", prefix);
    if (nvs_get_str(h, key, btn->label, &len) != ESP_OK)
        strncpy(btn->label, def->label, BTN_LABEL_MAX);

    snprintf(key, sizeof(key), "%s_bg", prefix);
    if (nvs_get_u32(h, key, &btn->bg_color) != ESP_OK)
        btn->bg_color = def->bg_color;

    snprintf(key, sizeof(key), "%s_fg", prefix);
    if (nvs_get_u32(h, key, &btn->fg_color) != ESP_OK)
        btn->fg_color = def->fg_color;
}

static void save_btn(nvs_handle_t h, const char *prefix, const btn_style_t *btn)
{
    char key[16];
    snprintf(key, sizeof(key), "%s_lbl", prefix);
    nvs_set_str(h, key, btn->label);
    snprintf(key, sizeof(key), "%s_bg", prefix);
    nvs_set_u32(h, key, btn->bg_color);
    snprintf(key, sizeof(key), "%s_fg", prefix);
    nvs_set_u32(h, key, btn->fg_color);
}

esp_err_t btn_config_load(btn_config_t *cfg)
{
    btn_config_t defs;
    btn_config_defaults(&defs);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        *cfg = defs;
        return ESP_OK;
    }
    load_btn(h, "on",    &cfg->on,    &defs.on);
    load_btn(h, "off",   &cfg->off,   &defs.off);
    load_btn(h, "day",   &cfg->day,   &defs.day);
    load_btn(h, "night", &cfg->night, &defs.night);
    load_btn(h, "plus",  &cfg->plus,  &defs.plus);
    load_btn(h, "minus", &cfg->minus, &defs.minus);
    nvs_close(h);
    return ESP_OK;
}

esp_err_t btn_config_save(const btn_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    save_btn(h, "on",    &cfg->on);
    save_btn(h, "off",   &cfg->off);
    save_btn(h, "day",   &cfg->day);
    save_btn(h, "night", &cfg->night);
    save_btn(h, "plus",  &cfg->plus);
    save_btn(h, "minus", &cfg->minus);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}
```

- [ ] **Step 3: Update storage CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "storage.c" "btn_config.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash log
)
```

- [ ] **Step 4: Build to verify**

```bash
idf.py build
```

- [ ] **Step 5: Commit**

```bash
git add components/storage/
git commit -m "feat: add btn_config NVS storage for configurable buttons"
```

---

### Task 4: Power Management Component

**Files:**
- Create: `components/power_mgmt/include/power_mgmt.h`
- Create: `components/power_mgmt/power_mgmt.c`
- Create: `components/power_mgmt/CMakeLists.txt`

- [ ] **Step 1: Create power_mgmt.h**

```c
#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t power_mgmt_init(i2c_master_bus_handle_t bus_handle);
uint8_t power_mgmt_get_pct(void);
bool power_mgmt_is_charging(void);
bool power_mgmt_is_usb(void);
```

- [ ] **Step 2: Create power_mgmt.c**

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_axp2101.h"
#include "bsp_display.h"
#include "storage.h"
#include "power_mgmt.h"

static const char *TAG = "power";

extern XPowersPMU power;  /* defined in bsp_axp2101.cpp */

static uint8_t s_pct = 100;
static bool s_charging = false;
static bool s_usb = false;
static uint8_t s_active_backlight = 100;

static void power_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        s_usb = power.isVbusIn();
        s_charging = power.isCharging();

        uint16_t mv = power.getBattVoltage();
        /* Simple linear approximation: 3.0V=0%, 4.1V=100% */
        if (mv <= 3000) s_pct = 0;
        else if (mv >= 4100) s_pct = 100;
        else s_pct = (uint8_t)((mv - 3000) * 100 / 1100);

        /* Threshold actions (only when on battery) */
        if (!s_usb) {
            const settings_t *cfg = storage_get_settings();

            if (s_pct <= 5) {
                ESP_LOGW(TAG, "Battery critical (%u%%) — shutting down", s_pct);
                storage_save_settings(cfg);
                vTaskDelay(pdMS_TO_TICKS(1000));
                power.shutdown();
            } else if (s_pct <= 10) {
                bsp_display_set_brightness(s_active_backlight / 4);
            } else if (s_pct <= 20) {
                bsp_display_set_brightness(s_active_backlight / 2);
            }
        }
    }
}

esp_err_t power_mgmt_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t err = bsp_axp2101_init(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 not found — running without battery management");
        return err;
    }

    s_active_backlight = 100;
    xTaskCreate(power_task, "power", 3072, NULL, 2, NULL);
    ESP_LOGI(TAG, "Power management started");
    return ESP_OK;
}

uint8_t power_mgmt_get_pct(void) { return s_pct; }
bool power_mgmt_is_charging(void) { return s_charging; }
bool power_mgmt_is_usb(void) { return s_usb; }
```

Note: This file is C but calls `bsp_axp2101_init()` which is C-linkage (`extern "C"`). The `extern XPowersPMU power` won't work from C — we need to add C wrapper functions to access battery data. Alternative: put the task in the `.cpp` file or add getters.

Actually, `bsp_axp2101.cpp` defines `XPowersPMU power` as a global C++ object. We can't access it from C directly. Instead, add thin C wrapper functions to `bsp_axp2101.cpp`:

- [ ] **Step 2b: Add C wrapper functions — create `components/esp_bsp/bsp_axp2101_ext.cpp`**

We must NOT modify the demo's `bsp_axp2101.cpp`. Instead create a new file that accesses the global `power` object:

```cpp
#include "bsp_axp2101.h"

extern XPowersPMU power;

extern "C" {

uint16_t bsp_axp2101_get_batt_voltage(void) { return power.getBattVoltage(); }
bool bsp_axp2101_is_charging(void) { return power.isCharging(); }
bool bsp_axp2101_is_vbus_in(void) { return power.isVbusIn(); }
void bsp_axp2101_shutdown(void) { power.shutdown(); }

}
```

Add a header `components/esp_bsp/bsp_axp2101_ext.h`:

```c
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t bsp_axp2101_get_batt_voltage(void);
bool bsp_axp2101_is_charging(void);
bool bsp_axp2101_is_vbus_in(void);
void bsp_axp2101_shutdown(void);

#ifdef __cplusplus
}
#endif
```

Add `bsp_axp2101_ext.cpp` to `esp_bsp/CMakeLists.txt` SRCS list.

- [ ] **Step 3: Update power_mgmt.c to use C wrappers**

Replace the `extern XPowersPMU` line and PMU calls:

```c
#include "bsp_axp2101_ext.h"
/* ... */
s_usb = bsp_axp2101_is_vbus_in();
s_charging = bsp_axp2101_is_charging();
uint16_t mv = bsp_axp2101_get_batt_voltage();
/* ... */
bsp_axp2101_shutdown();
```

- [ ] **Step 4: Create power_mgmt CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "power_mgmt.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_bsp storage log freertos
)
```

- [ ] **Step 5: Build to verify**

```bash
idf.py build
```

- [ ] **Step 6: Commit**

```bash
git add components/power_mgmt/ components/esp_bsp/bsp_axp2101_ext.*
git commit -m "feat: add power_mgmt component with battery monitoring"
```

---

### Task 5: Audio Component

**Files:**
- Create: `components/audio/include/audio.h`
- Create: `components/audio/audio.c`
- Create: `components/audio/CMakeLists.txt`

- [ ] **Step 1: Create audio.h**

```c
#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t audio_init(i2c_master_bus_handle_t bus_handle);
void audio_click(void);
void audio_error(void);
void audio_set_enabled(bool enabled);
```

- [ ] **Step 2: Create audio.c**

Generates sine wave tones programmatically and plays via ES8311/I2S:

```c
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_es8311.h"
#include "bsp_i2c.h"
#include "audio.h"

static const char *TAG = "audio";
static bool s_enabled = true;
static bool s_initialized = false;

/* Pre-computed tone buffers (48kHz, 16-bit mono) */
#define SAMPLE_RATE 48000
#define CLICK_FREQ 1000
#define CLICK_MS 50
#define ERROR_FREQ 400
#define ERROR_MS 200

#define CLICK_SAMPLES (SAMPLE_RATE * CLICK_MS / 1000)  /* 2400 */
#define ERROR_SAMPLES (SAMPLE_RATE * ERROR_MS / 1000)  /* 9600 */

static int16_t s_click_buf[CLICK_SAMPLES];
static int16_t s_error_buf[ERROR_SAMPLES];

static void generate_tone(int16_t *buf, int samples, int freq)
{
    for (int i = 0; i < samples; i++) {
        float t = (float)i / SAMPLE_RATE;
        float envelope = 1.0f;
        /* Fade out last 20% */
        if (i > samples * 4 / 5)
            envelope = (float)(samples - i) / (samples / 5);
        buf[i] = (int16_t)(sinf(2.0f * M_PI * freq * t) * 16000 * envelope);
    }
}

esp_err_t audio_init(i2c_master_bus_handle_t bus_handle)
{
    bsp_es8311_init(bus_handle);
    generate_tone(s_click_buf, CLICK_SAMPLES, CLICK_FREQ);
    generate_tone(s_error_buf, ERROR_SAMPLES, ERROR_FREQ);
    s_initialized = true;
    ESP_LOGI(TAG, "Audio initialized");
    return ESP_OK;
}

void audio_set_enabled(bool enabled) { s_enabled = enabled; }

void audio_click(void)
{
    if (!s_enabled || !s_initialized) return;
    bsp_es8311_playing((uint8_t *)s_click_buf, sizeof(s_click_buf));
}

void audio_error(void)
{
    if (!s_enabled || !s_initialized) return;
    bsp_es8311_playing((uint8_t *)s_error_buf, sizeof(s_error_buf));
}
```

- [ ] **Step 3: Create audio CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "audio.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_bsp log freertos
)
```

- [ ] **Step 4: Build to verify**

```bash
idf.py build
```

- [ ] **Step 5: Commit**

```bash
git add components/audio/
git commit -m "feat: add audio component with click and error tones"
```

---

### Task 6: WiFi AP Fallback

**Files:**
- Modify: `components/wifi_sta/include/wifi_sta.h`
- Modify: `components/wifi_sta/wifi_sta.c`

- [ ] **Step 1: Update wifi_sta.h**

Add `WIFI_STA_AP_MODE` to enum and new function declarations:

```c
typedef enum {
    WIFI_STA_DISCONNECTED,
    WIFI_STA_CONNECTING,
    WIFI_STA_CONNECTED,
    WIFI_STA_FAILED,
    WIFI_STA_AP_MODE,
} wifi_sta_status_t;

esp_err_t app_wifi_init(void);
esp_err_t app_wifi_connect(const char *ssid, const char *password);
void app_wifi_disconnect(void);
esp_err_t app_wifi_start_ap(void);
esp_err_t app_wifi_auto_connect(const char *ssid, const char *password);
wifi_sta_status_t app_wifi_get_status(void);
const char *app_wifi_get_ip(void);
```

- [ ] **Step 2: Add AP mode and auto_connect to wifi_sta.c**

Add `app_wifi_start_ap()`:

```c
esp_err_t app_wifi_start_ap(void)
{
    if (!s_initialized) {
        app_wifi_init();
    }

    wifi_config_t ap_config = {0};
    strncpy((char *)ap_config.ap.ssid, "NovaStar Remote", sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen("NovaStar Remote");
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_status = WIFI_STA_AP_MODE;
    strcpy(s_ip, "192.168.4.1");
    ESP_LOGI(TAG, "AP mode started: NovaStar Remote");
    return ESP_OK;
}
```

Add `app_wifi_auto_connect()`:

```c
esp_err_t app_wifi_auto_connect(const char *ssid, const char *password)
{
    if (ssid[0] == '\0') {
        ESP_LOGI(TAG, "No WiFi configured — starting AP");
        return app_wifi_start_ap();
    }

    app_wifi_init();

    /* Try STA 3 times */
    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "STA attempt %d/3 for \"%s\"", attempt, ssid);
        s_retry_count = 0;
        s_status = WIFI_STA_CONNECTING;

        wifi_config_t wifi_config = {0};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
        wifi_config.sta.threshold.authmode = password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_start();

        /* Wait up to 5 seconds for connection */
        for (int i = 0; i < 50; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (s_status == WIFI_STA_CONNECTED) {
                ESP_LOGI(TAG, "Connected on attempt %d", attempt);
                return ESP_OK;
            }
        }

        esp_wifi_stop();
    }

    ESP_LOGW(TAG, "STA failed after 3 attempts — starting AP");
    return app_wifi_start_ap();
}
```

Also update the event handler to handle AP mode (add `WIFI_EVENT_AP_STACONNECTED` etc. if needed — keep simple for now).

- [ ] **Step 3: Build to verify**

```bash
idf.py build
```

- [ ] **Step 4: Commit**

```bash
git add components/wifi_sta/
git commit -m "feat: add WiFi AP fallback mode"
```

---

### Task 7: UI — Configurable Buttons, Toast, Battery Icon, Standby, Slider Fix

**Files:**
- Modify: `components/ui/ui.c`
- Modify: `components/ui/include/ui.h`
- Modify: `components/ui/CMakeLists.txt`

This is the largest task. It modifies `build_control_screen()` to use configurable colors/labels from `btn_config_t` and `settings_t`, adds a toast label, adds a battery icon, adds standby timer, fixes slider gap, and adds button flash animation.

- [ ] **Step 1: Update ui.h**

```c
#pragma once

void ui_init(void);
void ui_show_settings(void);
void ui_show_control(void);
void ui_show_touch_test(void);
void ui_update_status(void);
void ui_toast(const char *msg, bool is_error);
bool ui_is_standby(void);
void ui_wake(void);
void ui_set_active_backlight(uint8_t pct);
```

- [ ] **Step 2: Update ui CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "ui.c" "ui_settings.c"
    INCLUDE_DIRS "include"
    REQUIRES novastar storage wifi_sta lvgl__lvgl freertos log esp_bsp audio power_mgmt
)
```

- [ ] **Step 3: Rewrite build_control_screen() to use btn_config and settings**

Key changes:
- Load `btn_config_t` via `btn_config_load()` and `settings_t` via `storage_get_settings()`.
- Use `cfg->ui_bg_color` for background, `cfg->ui_title` for title text, `cfg->ui_title_color` for title bar.
- Each button uses `btn_cfg.on.label`, `btn_cfg.on.bg_color`, `btn_cfg.on.fg_color` etc.
- ON handler sends `novastar_set_brightness(settings->on_bright)` instead of `novastar_screen_on()`.
- OFF handler sends `novastar_set_brightness(0)`.
- Slider gap: change `sl_w = SCR_W - 2*m - 2*pm_sz - 48` (was -24).
- Remove hardcoded `#define CLR_*` for button colors, read from config.
- Keep accent line with `lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE)` and `pad_all=0`.

(Full implementation code for this step should be written by the implementing agent based on the existing `ui.c` structure. The key principle: replace every hardcoded color/label with a config read.)

- [ ] **Step 4: Add toast label**

Add to `build_control_screen()` after the WiFi status label:

```c
s_toast_label = lv_label_create(scr);
lv_obj_align(s_toast_label, LV_ALIGN_BOTTOM_MID, 0, -22);
lv_obj_set_style_text_font(s_toast_label, &lv_font_montserrat_14, 0);
lv_obj_set_style_text_color(s_toast_label, lv_color_hex(0x888888), 0);
lv_obj_add_flag(s_toast_label, LV_OBJ_FLAG_HIDDEN);
```

Implement `ui_toast()`:

```c
static lv_obj_t *s_toast_label = NULL;
static lv_timer_t *s_toast_timer = NULL;

static void toast_hide_cb(lv_timer_t *t) {
    lv_obj_add_flag(s_toast_label, LV_OBJ_FLAG_HIDDEN);
    lv_timer_del(t);
    s_toast_timer = NULL;
}

void ui_toast(const char *msg, bool is_error)
{
    if (!s_toast_label) return;
    lv_label_set_text(s_toast_label, msg);
    lv_obj_set_style_text_color(s_toast_label,
        is_error ? lv_color_hex(0xCF6060) : lv_color_hex(0x888888), 0);
    lv_obj_clear_flag(s_toast_label, LV_OBJ_FLAG_HIDDEN);
    if (s_toast_timer) lv_timer_del(s_toast_timer);
    s_toast_timer = lv_timer_create(toast_hide_cb, 1500, NULL);
    lv_timer_set_repeat_count(s_toast_timer, 1);
}
```

- [ ] **Step 5: Add button flash on click**

In each button handler, after sending the command:

```c
static void flash_btn(lv_obj_t *btn, uint32_t original_color, bool is_error)
{
    lv_obj_set_style_bg_color(btn, is_error ? lv_color_hex(0xCF6060) : lv_color_hex(0x3a3a3a), 0);
    /* Timer to restore — store original color in timer user_data is tricky,
       simpler: use a static array or just set back directly after delay.
       Use an LVGL timer with user_data pointing to a struct. */
}
```

Simpler approach: each handler sets the button color lighter, starts a 200ms one-shot timer that restores it.

- [ ] **Step 6: Add battery icon (hideable)**

Add a label in top-right of control screen showing battery level:

```c
static lv_obj_t *s_batt_label = NULL;

/* In build_control_screen(): */
s_batt_label = lv_label_create(scr);
lv_obj_align(s_batt_label, LV_ALIGN_TOP_RIGHT, -8, 18);
lv_obj_set_style_text_font(s_batt_label, &lv_font_montserrat_12, 0);
lv_obj_set_style_text_color(s_batt_label, lv_color_hex(0x555555), 0);
```

Update in `ui_update_status()`:

```c
if (s_batt_label) {
    const settings_t *cfg = storage_get_settings();
    if (cfg->show_battery) {
        uint8_t pct = power_mgmt_get_pct();
        bool chg = power_mgmt_is_charging();
        if (chg)
            lv_label_set_text_fmt(s_batt_label, LV_SYMBOL_CHARGE " %u%%", pct);
        else
            lv_label_set_text_fmt(s_batt_label, LV_SYMBOL_BATTERY_FULL " %u%%", pct);
        /* Color by level */
        lv_color_t c = pct > 20 ? lv_color_hex(0x555555) :
                       pct > 10 ? lv_color_hex(0xE0A030) : lv_color_hex(0xCF6060);
        lv_obj_set_style_text_color(s_batt_label, c, 0);
        lv_obj_clear_flag(s_batt_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_batt_label, LV_OBJ_FLAG_HIDDEN);
    }
}
```

- [ ] **Step 7: Add standby logic**

Add standby timer and wake functions:

```c
static bool s_standby_active = false;
static uint8_t s_active_backlight = 100;

static void standby_check_cb(lv_timer_t *t)
{
    const settings_t *cfg = storage_get_settings();
    if (cfg->standby_timeout == 0) return;
    uint32_t idle = lv_disp_get_inactive_time(NULL);
    if (!s_standby_active && idle > (uint32_t)cfg->standby_timeout * 1000) {
        s_standby_active = true;
        bsp_display_set_brightness(cfg->standby_bright);
    }
}

bool ui_is_standby(void) { return s_standby_active; }

void ui_wake(void)
{
    if (s_standby_active) {
        s_standby_active = false;
        bsp_display_set_brightness(s_active_backlight);
    }
}

void ui_set_active_backlight(uint8_t pct) { s_active_backlight = pct; }
```

Add to `ui_init()`:

```c
lv_timer_create(standby_check_cb, 1000, NULL);
```

- [ ] **Step 8: Build to verify**

```bash
idf.py build
```

- [ ] **Step 9: Commit**

```bash
git add components/ui/
git commit -m "feat: configurable buttons, toast, battery, standby, slider fix"
```

---

### Task 8: Update app_main — Wire Everything Together

**Files:**
- Modify: `main/app_main.c`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Update main CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_bsp esp_lv_port ui storage wifi_sta novastar ota_update
             power_mgmt audio
             freertos esp_driver_i2c espressif__esp_io_expander_tca9554
)
```

- [ ] **Step 2: Update app_main.c**

Key changes:
- Add `#include "power_mgmt.h"` and `#include "audio.h"`.
- Call `power_mgmt_init(i2c_bus_handle)` after I2C init.
- Call `audio_init(i2c_bus_handle)` after power init.
- Call `audio_set_enabled(cfg->audio_enabled)`.
- Replace `app_wifi_init()` + `app_wifi_connect()` with `app_wifi_auto_connect(cfg->wifi_ssid, cfg->wifi_pass)`.
- In `touchpad_read()`, add standby wake logic:

```c
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
    touch_data_t touch_data;

    bsp_touch_read();
    if (bsp_touch_get_coordinates(&touch_data)) {
        last_x = touch_data.coords[0].x;
        last_y = touch_data.coords[0].y;

        /* If waking from standby, consume this touch */
        if (ui_is_standby()) {
            ui_wake();
            data->state = LV_INDEV_STATE_REL;  /* suppress */
        } else {
            data->state = LV_INDEV_STATE_PR;
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}
```

- [ ] **Step 3: Build to verify**

```bash
idf.py build
```

- [ ] **Step 4: Commit**

```bash
git add main/
git commit -m "feat: wire power_mgmt, audio, AP fallback, standby into app_main"
```

---

### Task 9: Web UI — All New Settings Sections

**Files:**
- Modify: `components/ota_update/ota_update.c`
- Modify: `components/ota_update/CMakeLists.txt`

- [ ] **Step 1: Update ota_update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "ota_update.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server app_update log freertos storage novastar wifi_sta json
             esp_bsp esp_lv_port ui power_mgmt audio
)
```

- [ ] **Step 2: Add `/api/btn_config` GET and POST handlers**

GET returns the current `btn_config_t` as JSON. POST accepts JSON and calls `btn_config_save()`. Include `btn_config.h`.

- [ ] **Step 3: Extend `/api/settings` GET response**

Add fields: `on_bright`, `ui_bg_color`, `ui_title`, `ui_title_color`, `standby_timeout`, `standby_bright`, `audio_enabled`, `show_battery`, `show_wifi`, `battery_pct`, `battery_charging`.

- [ ] **Step 4: Extend `/api/settings` POST handler**

Parse and save all new fields from JSON.

- [ ] **Step 5: Add test command endpoint `/api/test_cmd`**

POST with `{"cmd":"on"}` or `{"cmd":"off"}` or `{"cmd":"day"}` or `{"cmd":"night"}` or `{"cmd":"brightness","value":50}`. Calls the corresponding `novastar_set_brightness()`.

- [ ] **Step 6: Update HTML — add Buttons, Theme, Display, Sound sections**

Add to the `WEB_HTML` string:
- **Buttons** section: 6 rows, each with label/bg/fg/brightness inputs and a Test button.
- **Theme** section: bg color, title text, title bar color.
- **Display** section: standby timeout, standby brightness, show battery toggle, show wifi toggle.
- **Sound** section: audio enabled toggle.
- **System** section: add battery percentage.

Each Test button calls `fetch('/api/test_cmd', {method:'POST', body: JSON.stringify({cmd:'on'})})` etc.

- [ ] **Step 7: Build to verify**

```bash
idf.py build
```

- [ ] **Step 8: OTA flash and test**

```bash
curl -X POST --data-binary @build/av-controller.bin http://192.168.1.138:8080/ota
```

Verify: open web UI, change a button label, save, verify LCD updates. Press Test buttons, verify simulator receives commands.

- [ ] **Step 9: Commit**

```bash
git add components/ota_update/
git commit -m "feat: web UI with buttons config, theme, display, sound settings"
```

---

### Task 10: Integration Test and Cleanup

- [ ] **Step 1: Full build from clean**

```bash
rm -rf build managed_components && idf.py build
```

- [ ] **Step 2: OTA flash**

```bash
curl -X POST --data-binary @build/av-controller.bin http://192.168.1.138:8080/ota
```

- [ ] **Step 3: Test all features**

1. Battery icon visible (if AXP2101 present).
2. WiFi connects, IP shown on status bar.
3. All 6 buttons respond to touch.
4. Slider doesn't overlap +/- buttons.
5. Button press plays click sound.
6. Toast shows "ON 100%" etc. on press.
7. Failed TCP shows "Error" toast + error sound.
8. Web UI: change button labels, colors, save, verify LCD updates.
9. Web UI: change title text, background color, save, reboot, verify.
10. Web UI: toggle show_battery, show_wifi, verify indicators hide/show.
11. Web UI: set standby timeout to 10s, wait, verify backlight dims, tap to wake.
12. Web UI: toggle audio off, verify no click sound.
13. Disconnect WiFi (change SSID to garbage), reboot, verify AP mode starts as "NovaStar Remote".
14. Connect to AP, open 192.168.4.1:8080, reconfigure WiFi, save, verify reboot and STA connect.

- [ ] **Step 4: Remove touch debug code**

Remove the debug ring buffer and `/api/touch` endpoint from `bsp_touch.c` and `ota_update.c`. Remove the touch test screen from `ui.c` and the `/api/touchtest` endpoint. These were diagnostic tools.

- [ ] **Step 5: Final commit**

```bash
git add -A
git commit -m "feat: AV Controller v2 — all features integrated and tested"
```
