# Novastar VSX400 LED Screen Remote — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a dedicated touchscreen remote for the Novastar VSX400 LED processor on the Waveshare ESP32-S3-Touch-LCD-3.5B.

**Architecture:** Strip the existing AV controller to its LCD/touch/LVGL core, replace WiFi AP with STA, add a Novastar TCP protocol component, and build a purpose-built UI with ON/OFF, DAY/NIGHT presets, brightness slider, and a hidden settings screen.

**Tech Stack:** ESP-IDF 5.x, LVGL 8.4, AXS15231B QSPI LCD + integrated touch, ESP32-S3 with PSRAM

**Spec:** `docs/superpowers/specs/2026-03-29-novastar-led-remote-design.md`

---

## File Structure

### New/Rewritten Files
| File | Purpose |
|------|---------|
| `components/board/include/board.h` | Pin definitions for ESP32-S3-Touch-LCD-3.5B |
| `components/lcd_init/lcd_init.c` | AXS15231B QSPI display init |
| `components/lcd_init/include/lcd_init.h` | LCD init API (same interface, different internals) |
| `components/lcd_init/CMakeLists.txt` | Updated deps for QSPI |
| `components/touch/touch.c` | AXS15231B integrated touch via esp_lcd_touch API |
| `components/touch/include/touch.h` | Touch init API |
| `components/touch/CMakeLists.txt` | Updated deps |
| `components/esp_lcd_axs15231b/*` | LCD driver (copied from reference, local component) |
| `components/esp_lcd_touch_axs15231b/*` | Touch driver (copied from reference, local component) |
| `components/storage/storage.c` | NVS-only settings (no FFat) |
| `components/storage/include/storage.h` | Settings API |
| `components/storage/CMakeLists.txt` | Simplified deps |
| `components/wifi_sta/wifi_sta.c` | WiFi station mode |
| `components/wifi_sta/include/wifi_sta.h` | WiFi STA API |
| `components/wifi_sta/CMakeLists.txt` | Component deps |
| `components/novastar/novastar.c` | TCP client for VSX400 binary protocol |
| `components/novastar/include/novastar.h` | Novastar API |
| `components/novastar/CMakeLists.txt` | Component deps |
| `components/ui/ui.c` | Control screen (main UI) |
| `components/ui/ui_settings.c` | Settings screen |
| `components/ui/include/ui.h` | UI API |
| `components/ui/CMakeLists.txt` | Updated deps |
| `main/app_main.c` | Simplified boot sequence |
| `main/CMakeLists.txt` | Updated component requires |
| `main/idf_component.yml` | Managed component deps (esp_lcd_touch, lvgl) |
| `sdkconfig.defaults` | Updated for 3.5B hardware |
| `partitions.csv` | Simplified (no FFat needed) |
| `CMakeLists.txt` | Root project file (minimal change) |

### Deleted Files/Components
| Path | Reason |
|------|--------|
| `components/http_api/` | No web config needed |
| `components/rs232/` | No serial control |
| `components/cmd_dispatch/` | Direct TCP, no queue needed |
| `components/devices/` | No generic device config |
| `components/wifi_ap/` | Replaced by wifi_sta |
| `components/ui/ui_home.c` | Replaced by new ui.c |
| `components/ui/ui_device.c` | Not needed |
| `components/ui/ui_activity.c` | Not needed |
| `components/ui/ui_admin.c` | Replaced by ui_settings.c |
| `webapp/` | No embedded web UI |

---

## Task 1: Clean Up — Remove Unused Components

**Files:**
- Delete: `components/http_api/` (entire directory)
- Delete: `components/rs232/` (entire directory)
- Delete: `components/cmd_dispatch/` (entire directory)
- Delete: `components/devices/` (entire directory)
- Delete: `components/wifi_ap/` (entire directory)
- Delete: `components/ui/ui_home.c`
- Delete: `components/ui/ui_device.c`
- Delete: `components/ui/ui_activity.c`
- Delete: `components/ui/ui_admin.c`
- Delete: `webapp/` (entire directory)

- [ ] **Step 1: Delete unused component directories**

```bash
cd "/mnt/2tbstorage/Waveshare/4.3inlcdtouch/AV Controller/av-controller"
rm -rf components/http_api components/rs232 components/cmd_dispatch components/devices components/wifi_ap webapp
rm -f components/ui/ui_home.c components/ui/ui_device.c components/ui/ui_activity.c components/ui/ui_admin.c
```

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "chore: remove unused AV controller components (http_api, rs232, cmd_dispatch, devices, wifi_ap, webapp)"
```

---

## Task 2: Add AXS15231B LCD and Touch Driver Components

Copy the LCD and touch driver source from the reference project into local components.

**Files:**
- Create: `components/esp_lcd_axs15231b/esp_lcd_axs15231b.c`
- Create: `components/esp_lcd_axs15231b/include/esp_lcd_axs15231b.h`
- Create: `components/esp_lcd_axs15231b/CMakeLists.txt`
- Create: `components/esp_lcd_touch_axs15231b/esp_lcd_touch_axs15231b.c`
- Create: `components/esp_lcd_touch_axs15231b/include/esp_lcd_touch_axs15231b.h`
- Create: `components/esp_lcd_touch_axs15231b/CMakeLists.txt`

- [ ] **Step 1: Create AXS15231B LCD driver component**

`components/esp_lcd_axs15231b/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "esp_lcd_axs15231b.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver log
)
```

`components/esp_lcd_axs15231b/include/esp_lcd_axs15231b.h`:
Copy verbatim from `/tmp/S3-IDF_AXS15231B-QSPI-I2C_LVGL/components/esp_lcd_axs15231b/include/esp_lcd_axs15231b.h`

`components/esp_lcd_axs15231b/esp_lcd_axs15231b.c`:
Copy verbatim from `/tmp/S3-IDF_AXS15231B-QSPI-I2C_LVGL/components/esp_lcd_axs15231b/esp_lcd_axs15231b.c`

- [ ] **Step 2: Create AXS15231B touch driver component**

`components/esp_lcd_touch_axs15231b/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "esp_lcd_touch_axs15231b.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd_touch esp_lcd driver log
)
```

`components/esp_lcd_touch_axs15231b/include/esp_lcd_touch_axs15231b.h`:
Copy verbatim from `/tmp/S3-IDF_AXS15231B-QSPI-I2C_LVGL/components/esp_lcd_touch_axs15231b/include/esp_lcd_touch_axs15231b.h`

`components/esp_lcd_touch_axs15231b/esp_lcd_touch_axs15231b.c`:
Copy verbatim from `/tmp/S3-IDF_AXS15231B-QSPI-I2C_LVGL/components/esp_lcd_touch_axs15231b/esp_lcd_touch_axs15231b.c`

- [ ] **Step 3: Create idf_component.yml for managed dependencies**

`main/idf_component.yml`:
```yaml
dependencies:
  espressif/esp_lcd_touch: "^1.1.1"
  lvgl/lvgl: "^8.3.11"
  idf:
    version: ">=5.0.0"
```

- [ ] **Step 4: Commit**

```bash
git add components/esp_lcd_axs15231b components/esp_lcd_touch_axs15231b main/idf_component.yml
git commit -m "feat: add AXS15231B LCD and touch driver components for 3.5B board"
```

---

## Task 3: Update Board Definitions for ESP32-S3-Touch-LCD-3.5B

**Files:**
- Modify: `components/board/include/board.h`

- [ ] **Step 1: Rewrite board.h**

```c
#pragma once

/* ── Waveshare ESP32-S3-Touch-LCD-3.5B pin definitions ───────────────────── */

/* QSPI LCD (AXS15231B) */
#define PIN_LCD_CS      45
#define PIN_LCD_CLK     47
#define PIN_LCD_D0      21
#define PIN_LCD_D1      48
#define PIN_LCD_D2      40
#define PIN_LCD_D3      39
#define PIN_LCD_RST     (-1)    /* software reset */
#define PIN_BL          1

/* I2C bus (touch) */
#define PIN_I2C_SDA     4
#define PIN_I2C_SCL     8

/* Touch interrupt */
#define PIN_TOUCH_INT   3

/* Display geometry — portrait orientation */
#define LCD_H_RES       320
#define LCD_V_RES       480
#define LCD_SPI_CLK_HZ  (40 * 1000 * 1000)

/* LVGL draw buffer: full framebuffer for AXS15231B (full refresh required) */
#define LV_BUF_LINES    LCD_V_RES
```

- [ ] **Step 2: Commit**

```bash
git add components/board/include/board.h
git commit -m "feat: update board.h pin definitions for ESP32-S3-Touch-LCD-3.5B"
```

---

## Task 4: Rewrite LCD Init for AXS15231B QSPI

**Files:**
- Modify: `components/lcd_init/lcd_init.c`
- Modify: `components/lcd_init/include/lcd_init.h`
- Modify: `components/lcd_init/CMakeLists.txt`

- [ ] **Step 1: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "lcd_init.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_lcd_axs15231b esp_lcd esp_driver_spi esp_driver_i2c esp_driver_ledc freertos log
)
```

- [ ] **Step 2: Rewrite lcd_init.h**

```c
#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"

/**
 * Initialise the AXS15231B QSPI LCD in portrait mode (320x480).
 *
 * @param on_color_trans_done  Callback when DMA colour transfer completes (for LVGL flush_ready).
 * @param user_data            Passed to on_color_trans_done callback.
 * @param out_io               Receives the panel IO handle.
 * @param out_panel            Receives the panel handle.
 * @param out_i2c_bus          Receives the I2C bus handle (for touch).
 */
esp_err_t lcd_init(esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done,
                   void *user_data,
                   esp_lcd_panel_io_handle_t *out_io,
                   esp_lcd_panel_handle_t *out_panel,
                   i2c_master_bus_handle_t *out_i2c_bus);

/**
 * Set backlight brightness via PWM (0-100%).
 */
void lcd_backlight_set(int percent);
```

- [ ] **Step 3: Rewrite lcd_init.c**

```c
/*
 * lcd_init — AXS15231B QSPI display initialisation for Waveshare ESP32-S3-Touch-LCD-3.5B
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#include "board.h"
#include "lcd_init.h"
#include "esp_lcd_axs15231b.h"

static const char *TAG = "lcd_init";

/* ── Backlight PWM ───────────────────────────────────────────────────────── */

static bool s_bl_inited = false;

void lcd_backlight_set(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    if (!s_bl_inited) {
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .timer_num       = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .freq_hz         = 5000,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timer);

        ledc_channel_config_t ch = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LEDC_CHANNEL_0,
            .timer_sel  = LEDC_TIMER_0,
            .gpio_num   = PIN_BL,
            .duty       = 0,
            .hpoint     = 0,
        };
        ledc_channel_config(&ch);
        s_bl_inited = true;
    }

    uint32_t duty = (uint32_t)(percent * 1023 / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ── Main init ───────────────────────────────────────────────────────────── */

esp_err_t lcd_init(esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done,
                   void *user_data,
                   esp_lcd_panel_io_handle_t *out_io,
                   esp_lcd_panel_handle_t *out_panel,
                   i2c_master_bus_handle_t *out_i2c_bus)
{
    /* Backlight off during init */
    lcd_backlight_set(0);

    /* I2C bus — shared with touch */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus), TAG, "I2C bus");

    /* QSPI bus */
    const spi_bus_config_t spi_bus = AXS15231B_PANEL_BUS_QSPI_CONFIG(
        PIN_LCD_CLK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
        LCD_H_RES * LCD_V_RES * 2
    );
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &spi_bus, SPI_DMA_CH_AUTO), TAG, "SPI bus");

    /* Panel IO */
    const esp_lcd_panel_io_spi_config_t io_cfg = AXS15231B_PANEL_IO_QSPI_CONFIG(
        PIN_LCD_CS, on_color_trans_done, user_data
    );
    esp_lcd_panel_io_handle_t io;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io),
        TAG, "panel IO");

    /* AXS15231B panel driver */
    axs15231b_vendor_config_t vendor_config = {
        .flags = { .use_qspi_interface = 1 },
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = &vendor_config,
    };
    esp_lcd_panel_handle_t panel;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_axs15231b(io, &panel_cfg, &panel), TAG, "panel");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "disp on");

    *out_io      = io;
    *out_panel   = panel;
    *out_i2c_bus = i2c_bus;

    ESP_LOGI(TAG, "AXS15231B QSPI LCD init OK — %dx%d portrait", LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}
```

- [ ] **Step 4: Commit**

```bash
git add components/lcd_init/
git commit -m "feat: rewrite lcd_init for AXS15231B QSPI display"
```

---

## Task 5: Rewrite Touch for AXS15231B Integrated Controller

**Files:**
- Modify: `components/touch/touch.c`
- Modify: `components/touch/include/touch.h`
- Modify: `components/touch/CMakeLists.txt`

- [ ] **Step 1: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "touch.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_lcd_touch_axs15231b esp_lcd_touch lvgl__lvgl esp_lcd esp_driver_i2c freertos log
)
```

- [ ] **Step 2: Update touch.h**

```c
#pragma once

#include "driver/i2c_master.h"

/**
 * Initialise AXS15231B integrated touch and register as LVGL input device.
 * Must be called after lvgl_port_init() and while holding the LVGL mutex.
 *
 * Uses legacy I2C driver internally (required by esp_lcd_touch API).
 * The i2c_bus parameter from lcd_init is NOT used — touch uses its own
 * I2C instance via the legacy API since esp_lcd_panel_io_i2c requires it.
 */
void touch_init(void);
```

- [ ] **Step 3: Rewrite touch.c**

```c
/*
 * touch — AXS15231B integrated touch driver for LVGL
 *
 * The AXS15231B has an integrated touch controller accessed via I2C (addr 0x3B).
 * Uses the esp_lcd_touch framework with interrupt-driven reads.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

#include "board.h"
#include "touch.h"
#include "esp_lcd_touch_axs15231b.h"

static const char *TAG = "touch";

static esp_lcd_touch_handle_t s_tp = NULL;
static SemaphoreHandle_t s_touch_mux = NULL;

static void touch_isr_cb(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_touch_mux, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;

    if (xSemaphoreTake(s_touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(tp);
    }

    uint16_t x, y;
    uint8_t cnt = 0;
    bool pressed = esp_lcd_touch_get_coordinates(tp, &x, &y, NULL, &cnt, 1);

    if (pressed && cnt > 0) {
        data->point.x = x;
        data->point.y = y;
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void touch_init(void)
{
    /* Legacy I2C driver init (required by esp_lcd_panel_io_i2c) */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    /* Touch panel IO over I2C */
    esp_lcd_panel_io_handle_t tp_io = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_cfg, &tp_io));

    s_touch_mux = xSemaphoreCreateBinary();

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = PIN_TOUCH_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
        .interrupt_callback = touch_isr_cb,
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_axs15231b(tp_io, &tp_cfg, &s_tp));

    /* Register with LVGL */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    indev_drv.user_data = s_tp;
    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "AXS15231B touch init OK (I2C addr 0x%02X, INT=GPIO%d)",
             ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS, PIN_TOUCH_INT);
}
```

- [ ] **Step 4: Commit**

```bash
git add components/touch/
git commit -m "feat: rewrite touch driver for AXS15231B integrated controller"
```

---

## Task 6: Update LVGL Port for Full-Refresh Mode

The AXS15231B requires full-screen refresh. Update lvgl_port to use a full-framebuffer draw buffer allocated in PSRAM.

**Files:**
- Modify: `components/lvgl_port/lvgl_port.c`

- [ ] **Step 1: Update lvgl_port.c**

Changes from existing:
1. Allocate a single full-framebuffer in PSRAM (not two partial DMA buffers)
2. Set `full_refresh = 1` on the display driver
3. Pass `panel_handle` as `user_data` on the display driver (flush_cb needs it)

```c
/*
 * lvgl_port — LVGL display driver + timer task for ESP-IDF
 *
 * AXS15231B variant: full-refresh mode with PSRAM framebuffer.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#include "board.h"
#include "lvgl_port.h"

static const char *TAG = "lvgl_port";

static lv_disp_drv_t          s_disp_drv;
static SemaphoreHandle_t      s_lvgl_mutex;

/* ── Flush callback ──────────────────────────────────────────────────────── */

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    int x1 = area->x1, y1 = area->y1;
    int x2 = area->x2 + 1;
    int y2 = area->y2 + 1;
    esp_lcd_panel_draw_bitmap(panel, x1, y1, x2, y2, color_map);
}

/* ── DMA-complete ISR callback ───────────────────────────────────────────── */

static bool on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx)
{
    lv_disp_drv_t *drv = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(drv);
    return false;
}

/* ── LVGL tick ───────────────────────────────────────────────────────────── */

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(1);
}

/* ── LVGL timer handler task ─────────────────────────────────────────────── */

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started on core %d", xPortGetCoreID());
    for (;;) {
        if (xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(50))) {
            uint32_t next_ms = lv_timer_handler();
            xSemaphoreGive(s_lvgl_mutex);
            if (next_ms < 5) next_ms = 5;
            vTaskDelay(pdMS_TO_TICKS(next_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void lvgl_port_init(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel)
{
    (void)io; /* not needed — DMA callback wired via lcd_init */
    s_lvgl_mutex = xSemaphoreCreateMutex();
    assert(s_lvgl_mutex);

    lv_init();

    /* 1ms periodic timer for lv_tick_inc */
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name = "lv_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000));

    /* Full framebuffer in PSRAM for AXS15231B full-refresh mode */
    size_t buf_sz = LCD_H_RES * LCD_V_RES * sizeof(lv_color_t);
    void *buf1 = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    assert(buf1);
    ESP_LOGI(TAG, "Draw buffer: %u bytes in PSRAM (full framebuffer)", (unsigned)buf_sz);

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_H_RES * LCD_V_RES);

    /* Display driver — full refresh mode */
    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res       = LCD_H_RES;
    s_disp_drv.ver_res       = LCD_V_RES;
    s_disp_drv.flush_cb      = flush_cb;
    s_disp_drv.draw_buf      = &draw_buf;
    s_disp_drv.user_data     = panel;
    s_disp_drv.full_refresh  = 1;
    lv_disp_drv_register(&s_disp_drv);
}

esp_lcd_panel_io_color_trans_done_cb_t lvgl_port_get_flush_done_cb(void)
{
    return on_color_trans_done;
}

void *lvgl_port_get_flush_done_user_data(void)
{
    return &s_disp_drv;
}

void lvgl_port_start(void)
{
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8192, NULL, 5, NULL, 1);
}

bool lvgl_port_lock(int timeout_ms)
{
    return xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void lvgl_port_unlock(void)
{
    xSemaphoreGive(s_lvgl_mutex);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/lvgl_port/
git commit -m "feat: update lvgl_port for AXS15231B full-refresh mode with PSRAM buffer"
```

---

## Task 7: Rewrite Storage — NVS Only

**Files:**
- Modify: `components/storage/storage.c`
- Modify: `components/storage/include/storage.h`
- Modify: `components/storage/CMakeLists.txt`

- [ ] **Step 1: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "storage.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash log
)
```

- [ ] **Step 2: Rewrite storage.h**

```c
#pragma once

#include <stdint.h>
#include "esp_err.h"

/** Settings structure — all fields stored in NVS */
typedef struct {
    char wifi_ssid[33];
    char wifi_pass[65];
    char nova_ip[16];
    uint16_t nova_port;
    uint8_t day_bright;
    uint8_t night_bright;
} settings_t;

/** Init NVS and load settings. */
esp_err_t storage_init(void);

/** Get pointer to current settings (read-only). */
const settings_t *storage_get_settings(void);

/** Save all settings to NVS. */
esp_err_t storage_save_settings(const settings_t *s);

/** Returns true if WiFi credentials are configured. */
bool storage_has_wifi(void);
```

- [ ] **Step 3: Rewrite storage.c**

```c
/*
 * storage — NVS-only settings for Novastar LED remote
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "storage.h"

static const char *TAG = "storage";
#define NVS_NS "ledremote"

static settings_t s_settings;

static void load_str(nvs_handle_t h, const char *key, char *buf, size_t buf_len, const char *def)
{
    size_t len = buf_len;
    if (nvs_get_str(h, key, buf, &len) != ESP_OK) {
        strncpy(buf, def, buf_len - 1);
        buf[buf_len - 1] = '\0';
    }
}

static void load_u16(nvs_handle_t h, const char *key, uint16_t *val, uint16_t def)
{
    if (nvs_get_u16(h, key, val) != ESP_OK) *val = def;
}

static void load_u8(nvs_handle_t h, const char *key, uint8_t *val, uint8_t def)
{
    if (nvs_get_u8(h, key, val) != ESP_OK) *val = def;
}

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init");

    nvs_handle_t h;
    err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_OK) {
        load_str(h, "wifi_ssid", s_settings.wifi_ssid, sizeof(s_settings.wifi_ssid), "");
        load_str(h, "wifi_pass", s_settings.wifi_pass, sizeof(s_settings.wifi_pass), "");
        load_str(h, "nova_ip",   s_settings.nova_ip,   sizeof(s_settings.nova_ip),   "192.168.1.50");
        load_u16(h, "nova_port", &s_settings.nova_port, 5200);
        load_u8(h,  "day_brt",   &s_settings.day_bright, 100);
        load_u8(h,  "night_brt", &s_settings.night_bright, 30);
        nvs_close(h);
    } else {
        /* First boot — use defaults */
        strncpy(s_settings.nova_ip, "192.168.1.50", sizeof(s_settings.nova_ip));
        s_settings.nova_port    = 5200;
        s_settings.day_bright   = 100;
        s_settings.night_bright = 30;
    }

    ESP_LOGI(TAG, "Settings loaded (SSID: \"%s\", VSX400: %s:%u, day=%u%%, night=%u%%)",
             s_settings.wifi_ssid, s_settings.nova_ip, s_settings.nova_port,
             s_settings.day_bright, s_settings.night_bright);
    return ESP_OK;
}

const settings_t *storage_get_settings(void)
{
    return &s_settings;
}

esp_err_t storage_save_settings(const settings_t *s)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &h), TAG, "NVS open");

    nvs_set_str(h, "wifi_ssid", s->wifi_ssid);
    nvs_set_str(h, "wifi_pass", s->wifi_pass);
    nvs_set_str(h, "nova_ip",   s->nova_ip);
    nvs_set_u16(h, "nova_port", s->nova_port);
    nvs_set_u8(h,  "day_brt",   s->day_bright);
    nvs_set_u8(h,  "night_brt", s->night_bright);

    esp_err_t err = nvs_commit(h);
    nvs_close(h);

    memcpy(&s_settings, s, sizeof(s_settings));
    ESP_LOGI(TAG, "Settings saved");
    return err;
}

bool storage_has_wifi(void)
{
    return s_settings.wifi_ssid[0] != '\0';
}
```

- [ ] **Step 4: Commit**

```bash
git add components/storage/
git commit -m "feat: rewrite storage as NVS-only settings for LED remote"
```

---

## Task 8: Create WiFi STA Component

**Files:**
- Create: `components/wifi_sta/wifi_sta.c`
- Create: `components/wifi_sta/include/wifi_sta.h`
- Create: `components/wifi_sta/CMakeLists.txt`

- [ ] **Step 1: Create CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "wifi_sta.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event log
)
```

- [ ] **Step 2: Create wifi_sta.h**

```c
#pragma once

#include "esp_err.h"

/** WiFi connection status */
typedef enum {
    WIFI_STA_DISCONNECTED,
    WIFI_STA_CONNECTING,
    WIFI_STA_CONNECTED,
    WIFI_STA_FAILED,
} wifi_sta_status_t;

/**
 * Init WiFi subsystem. Call once at boot.
 */
esp_err_t wifi_sta_init(void);

/**
 * Connect to an AP. Non-blocking — poll status with wifi_sta_get_status().
 */
esp_err_t wifi_sta_connect(const char *ssid, const char *password);

/**
 * Disconnect from current AP.
 */
void wifi_sta_disconnect(void);

/**
 * Get current connection status.
 */
wifi_sta_status_t wifi_sta_get_status(void);

/**
 * Get assigned IP address string. Returns "0.0.0.0" if not connected.
 */
const char *wifi_sta_get_ip(void);
```

- [ ] **Step 3: Create wifi_sta.c**

```c
/*
 * wifi_sta — WiFi station mode for connecting to existing network
 */

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "wifi_sta.h"

static const char *TAG = "wifi_sta";

static wifi_sta_status_t s_status = WIFI_STA_DISCONNECTED;
static char s_ip[16] = "0.0.0.0";
static bool s_initialized = false;
static int s_retry_count = 0;
#define MAX_RETRIES 5

static void event_handler(void *arg, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            s_status = WIFI_STA_CONNECTING;
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (s_retry_count < MAX_RETRIES) {
                s_retry_count++;
                s_status = WIFI_STA_CONNECTING;
                ESP_LOGI(TAG, "Reconnecting (attempt %d/%d)...", s_retry_count, MAX_RETRIES);
                esp_wifi_connect();
            } else {
                s_status = WIFI_STA_FAILED;
                ESP_LOGW(TAG, "Connection failed after %d attempts", MAX_RETRIES);
            }
            strcpy(s_ip, "0.0.0.0");
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
        s_status = WIFI_STA_CONNECTED;
        s_retry_count = 0;
        ESP_LOGI(TAG, "Connected — IP: %s", s_ip);
    }
}

esp_err_t wifi_sta_init(void)
{
    if (s_initialized) return ESP_OK;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    s_initialized = true;
    ESP_LOGI(TAG, "WiFi STA initialized");
    return ESP_OK;
}

esp_err_t wifi_sta_connect(const char *ssid, const char *password)
{
    s_retry_count = 0;
    s_status = WIFI_STA_CONNECTING;

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to \"%s\"...", ssid);
    return ESP_OK;
}

void wifi_sta_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    s_status = WIFI_STA_DISCONNECTED;
    strcpy(s_ip, "0.0.0.0");
}

wifi_sta_status_t wifi_sta_get_status(void)
{
    return s_status;
}

const char *wifi_sta_get_ip(void)
{
    return s_ip;
}
```

- [ ] **Step 4: Commit**

```bash
git add components/wifi_sta/
git commit -m "feat: add wifi_sta component for station mode connectivity"
```

---

## Task 9: Create Novastar Component

**Files:**
- Create: `components/novastar/novastar.c`
- Create: `components/novastar/include/novastar.h`
- Create: `components/novastar/CMakeLists.txt`

- [ ] **Step 1: Create CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "novastar.c"
    INCLUDE_DIRS "include"
    REQUIRES lwip log
)
```

- [ ] **Step 2: Create novastar.h**

```c
#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * Set the target VSX400 address. Must be called before sending commands.
 */
void novastar_set_target(const char *ip, uint16_t port);

/**
 * Screen on (Normal Display).
 */
esp_err_t novastar_screen_on(void);

/**
 * Screen off (Screen Black / Freeze).
 */
esp_err_t novastar_screen_off(void);

/**
 * Set brightness (0-100%).
 */
esp_err_t novastar_set_brightness(uint8_t percent);

/**
 * Load preset (1-16).
 */
esp_err_t novastar_load_preset(uint8_t preset);
```

- [ ] **Step 3: Create novastar.c**

```c
/*
 * novastar — TCP client for Novastar VSX400 binary protocol
 *
 * Fire-and-forget: opens socket, sends command, closes socket.
 * Protocol: binary packets on TCP port 5200.
 */

#include <string.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "novastar.h"

static const char *TAG = "novastar";

static char s_ip[16] = "192.168.1.50";
static uint16_t s_port = 5200;

/* ── Pre-computed commands ───────────────────────────────────────────────── */

static const uint8_t CMD_SCREEN_ON[] = {
    0x55, 0xaa, 0x00, 0x38, 0xfe, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x13,
    0x01, 0x00, 0x03, 0xa7, 0x56, 0x0d
};

static const uint8_t CMD_SCREEN_OFF[] = {
    0x55, 0xaa, 0x00, 0x37, 0xfe, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x13,
    0x01, 0x00, 0x05, 0xa8, 0x56, 0x0d
};

/* Brightness template: bytes 18=value, 19=checksum, rest fixed */
static const uint8_t CMD_BRIGHTNESS_TEMPLATE[] = {
    0x55, 0xaa, 0x00, 0x00, 0xfe, 0xff, 0x01, 0xff,
    0xff, 0xff, 0x01, 0x00, 0x01, 0x00, 0x00, 0x02,
    0x01, 0x00, 0x00, 0x55, 0x5a, 0x0d
};

/* Preset template: byte 18=preset_index(0-15), 19=checksum */
static const uint8_t CMD_PRESET_TEMPLATE[] = {
    0x55, 0xaa, 0x00, 0xd6, 0xfe, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x51, 0x13,
    0x01, 0x00, 0x00, 0x3b, 0x5a, 0x0d
};

/* ── TCP send helper ─────────────────────────────────────────────────────── */

static esp_err_t tcp_send(const uint8_t *data, size_t len)
{
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(s_port),
    };
    if (inet_pton(AF_INET, s_ip, &dest.sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid IP: %s", s_ip);
        return ESP_FAIL;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        return ESP_FAIL;
    }

    /* 2 second timeout */
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    esp_err_t ret = ESP_OK;
    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGE(TAG, "Connect to %s:%u failed", s_ip, s_port);
        ret = ESP_FAIL;
    } else {
        int sent = send(sock, data, len, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "Send failed");
            ret = ESP_FAIL;
        } else {
            ESP_LOGD(TAG, "Sent %d bytes to %s:%u", sent, s_ip, s_port);
        }
    }

    close(sock);
    return ret;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void novastar_set_target(const char *ip, uint16_t port)
{
    strncpy(s_ip, ip, sizeof(s_ip) - 1);
    s_ip[sizeof(s_ip) - 1] = '\0';
    s_port = port;
    ESP_LOGI(TAG, "Target: %s:%u", s_ip, s_port);
}

esp_err_t novastar_screen_on(void)
{
    ESP_LOGI(TAG, "Screen ON (Normal Display)");
    return tcp_send(CMD_SCREEN_ON, sizeof(CMD_SCREEN_ON));
}

esp_err_t novastar_screen_off(void)
{
    ESP_LOGI(TAG, "Screen OFF (Screen Black)");
    return tcp_send(CMD_SCREEN_OFF, sizeof(CMD_SCREEN_OFF));
}

esp_err_t novastar_set_brightness(uint8_t percent)
{
    if (percent > 100) percent = 100;

    uint8_t cmd[sizeof(CMD_BRIGHTNESS_TEMPLATE)];
    memcpy(cmd, CMD_BRIGHTNESS_TEMPLATE, sizeof(cmd));

    /* Value byte: 0-255 mapped from 0-100% */
    uint8_t val = (uint8_t)((percent * 255 + 50) / 100);
    cmd[18] = val;
    cmd[19] = (0x55 + val) & 0xFF;

    ESP_LOGI(TAG, "Brightness %u%% (val=0x%02X)", percent, val);
    return tcp_send(cmd, sizeof(cmd));
}

esp_err_t novastar_load_preset(uint8_t preset)
{
    if (preset < 1 || preset > 16) return ESP_ERR_INVALID_ARG;

    uint8_t cmd[sizeof(CMD_PRESET_TEMPLATE)];
    memcpy(cmd, CMD_PRESET_TEMPLATE, sizeof(cmd));

    uint8_t idx = preset - 1;
    cmd[18] = idx;
    cmd[19] = (0x3B + idx) & 0xFF;

    ESP_LOGI(TAG, "Preset %u", preset);
    return tcp_send(cmd, sizeof(cmd));
}
```

- [ ] **Step 4: Commit**

```bash
git add components/novastar/
git commit -m "feat: add novastar component — TCP client for VSX400 binary protocol"
```

---

## Task 10: Rewrite UI — Control Screen

The main screen matching the mockup: title bar, ON/OFF, DAY/NIGHT, brightness slider with +/- buttons.

**Files:**
- Modify: `components/ui/ui.c`
- Modify: `components/ui/include/ui.h`
- Modify: `components/ui/CMakeLists.txt`

- [ ] **Step 1: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "ui.c" "ui_settings.c"
    INCLUDE_DIRS "include"
    REQUIRES novastar storage wifi_sta board lvgl__lvgl freertos log
)
```

- [ ] **Step 2: Write ui.h**

```c
#pragma once

/**
 * Initialise the UI — creates the control screen.
 * Must be called while holding the LVGL mutex.
 */
void ui_init(void);

/**
 * Switch to settings screen.
 */
void ui_show_settings(void);

/**
 * Switch back to control screen.
 */
void ui_show_control(void);

/**
 * Update WiFi status indicator on control screen.
 * Call periodically from the LVGL task.
 */
void ui_update_status(void);
```

- [ ] **Step 3: Write ui.c (control screen)**

```c
/*
 * ui.c — LED Screen control UI
 *
 * Layout (320x480 portrait):
 *   - Title bar: "LED SCREEN" with 10s long-press for settings
 *   - ON / OFF buttons (green / red)
 *   - DAY / NIGHT preset buttons
 *   - Brightness slider with +/- buttons
 *   - WiFi status indicator
 */

#include "lvgl.h"
#include "esp_log.h"

#include "board.h"
#include "novastar.h"
#include "storage.h"
#include "wifi_sta.h"
#include "ui.h"

static const char *TAG = "ui";

/* ── Screens ─────────────────────────────────────────────────────────────── */
static lv_obj_t *s_scr_control = NULL;
static lv_obj_t *s_scr_settings = NULL;

/* ── Control screen widgets ──────────────────────────────────────────────── */
static lv_obj_t *s_lbl_title = NULL;
static lv_obj_t *s_btn_on = NULL;
static lv_obj_t *s_btn_off = NULL;
static lv_obj_t *s_btn_day = NULL;
static lv_obj_t *s_btn_night = NULL;
static lv_obj_t *s_slider = NULL;
static lv_obj_t *s_lbl_brightness = NULL;
static lv_obj_t *s_lbl_status = NULL;

static uint8_t s_brightness = 100;
static bool s_screen_on = true;

/* ── Long-press timer for title bar ──────────────────────────────────────── */
static lv_timer_t *s_longpress_timer = NULL;
static uint32_t s_press_start = 0;
#define SETTINGS_HOLD_MS 10000

/* ── Colors ──────────────────────────────────────────────────────────────── */
#define CLR_BG          lv_color_hex(0x1a1a2e)
#define CLR_TITLE_BG    lv_color_hex(0x0a3d91)
#define CLR_GREEN       lv_color_hex(0x2ecc40)
#define CLR_GREEN_DARK  lv_color_hex(0x1a7a28)
#define CLR_RED         lv_color_hex(0xe74c3c)
#define CLR_RED_DARK    lv_color_hex(0x8b2020)
#define CLR_DAY_BG      lv_color_hex(0x3498db)
#define CLR_NIGHT_BG    lv_color_hex(0x1a237e)
#define CLR_BTN_PLUS    lv_color_hex(0x2c3e50)
#define CLR_SLIDER_BG   lv_color_hex(0x2c3e50)
#define CLR_SLIDER_IND  lv_color_hex(0x3498db)
#define CLR_SLIDER_KNOB lv_color_hex(0x5dade2)
#define CLR_WHITE       lv_color_hex(0xffffff)
#define CLR_GRAY        lv_color_hex(0x888888)

/* ── Forward declarations ────────────────────────────────────────────────── */
extern void ui_settings_create(lv_obj_t *scr);

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void update_slider_label(void)
{
    lv_label_set_text_fmt(s_lbl_brightness, "Brightness  %u%%", s_brightness);
}

static void send_brightness(uint8_t pct)
{
    s_brightness = pct;
    lv_slider_set_value(s_slider, pct, LV_ANIM_ON);
    update_slider_label();
    novastar_set_brightness(pct);
}

/* ── Event callbacks ─────────────────────────────────────────────────────── */

static void on_btn_on(lv_event_t *e)
{
    (void)e;
    s_screen_on = true;
    novastar_screen_on();
    ESP_LOGI(TAG, "ON pressed");
}

static void on_btn_off(lv_event_t *e)
{
    (void)e;
    s_screen_on = false;
    novastar_screen_off();
    ESP_LOGI(TAG, "OFF pressed");
}

static void on_btn_day(lv_event_t *e)
{
    (void)e;
    const settings_t *s = storage_get_settings();
    send_brightness(s->day_bright);
    ESP_LOGI(TAG, "DAY preset (%u%%)", s->day_bright);
}

static void on_btn_night(lv_event_t *e)
{
    (void)e;
    const settings_t *s = storage_get_settings();
    send_brightness(s->night_bright);
    ESP_LOGI(TAG, "NIGHT preset (%u%%)", s->night_bright);
}

static void on_slider_changed(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    s_brightness = (uint8_t)lv_slider_get_value(slider);
    update_slider_label();
    novastar_set_brightness(s_brightness);
}

static void on_btn_minus(lv_event_t *e)
{
    (void)e;
    uint8_t pct = s_brightness >= 5 ? s_brightness - 5 : 0;
    send_brightness(pct);
}

static void on_btn_plus(lv_event_t *e)
{
    (void)e;
    uint8_t pct = s_brightness <= 95 ? s_brightness + 5 : 100;
    send_brightness(pct);
}

/* Title long-press: 10 second hold detection */
static void on_title_press(lv_event_t *e)
{
    (void)e;
    s_press_start = lv_tick_get();
}

static void longpress_check_cb(lv_timer_t *timer)
{
    (void)timer;
    if (s_press_start == 0) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) indev = lv_indev_get_next(NULL);
    if (!indev) { s_press_start = 0; return; }

    lv_indev_data_t data;
    lv_indev_read(indev, &data);

    if (data.state == LV_INDEV_STATE_RELEASED) {
        s_press_start = 0;
        return;
    }

    if (lv_tick_elaps(s_press_start) >= SETTINGS_HOLD_MS) {
        s_press_start = 0;
        ESP_LOGI(TAG, "10s hold — opening settings");
        ui_show_settings();
    }
}

static void on_title_release(lv_event_t *e)
{
    (void)e;
    s_press_start = 0;
}

/* ── Build control screen ────────────────────────────────────────────────── */

static void build_control_screen(lv_obj_t *scr)
{
    lv_obj_set_style_bg_color(scr, CLR_BG, 0);

    /* ── Title bar ───────────────────────────────────────────────────────── */
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, 300, 44);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(title_bar, CLR_TITLE_BG, 0);
    lv_obj_set_style_radius(title_bar, 6, 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_lbl_title = lv_label_create(title_bar);
    lv_label_set_text(s_lbl_title, "LED SCREEN");
    lv_obj_set_style_text_font(s_lbl_title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_lbl_title, CLR_WHITE, 0);
    lv_obj_center(s_lbl_title);

    /* Long-press events on title bar */
    lv_obj_add_flag(title_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(title_bar, on_title_press, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(title_bar, on_title_release, LV_EVENT_RELEASED, NULL);

    /* ── ON / OFF buttons ────────────────────────────────────────────────── */
    s_btn_on = lv_btn_create(scr);
    lv_obj_set_size(s_btn_on, 138, 60);
    lv_obj_align(s_btn_on, LV_ALIGN_TOP_LEFT, 12, 70);
    lv_obj_set_style_bg_color(s_btn_on, CLR_GREEN, 0);
    lv_obj_set_style_bg_color(s_btn_on, CLR_GREEN_DARK, LV_STATE_PRESSED);
    lv_obj_set_style_radius(s_btn_on, 8, 0);
    lv_obj_add_event_cb(s_btn_on, on_btn_on, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_on = lv_label_create(s_btn_on);
    lv_label_set_text(lbl_on, "ON");
    lv_obj_set_style_text_font(lbl_on, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl_on, CLR_WHITE, 0);
    lv_obj_center(lbl_on);

    s_btn_off = lv_btn_create(scr);
    lv_obj_set_size(s_btn_off, 138, 60);
    lv_obj_align(s_btn_off, LV_ALIGN_TOP_RIGHT, -12, 70);
    lv_obj_set_style_bg_color(s_btn_off, CLR_RED, 0);
    lv_obj_set_style_bg_color(s_btn_off, CLR_RED_DARK, LV_STATE_PRESSED);
    lv_obj_set_style_radius(s_btn_off, 8, 0);
    lv_obj_add_event_cb(s_btn_off, on_btn_off, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_off = lv_label_create(s_btn_off);
    lv_label_set_text(lbl_off, "OFF");
    lv_obj_set_style_text_font(lbl_off, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl_off, CLR_WHITE, 0);
    lv_obj_center(lbl_off);

    /* ── DAY / NIGHT buttons ─────────────────────────────────────────────── */
    s_btn_day = lv_btn_create(scr);
    lv_obj_set_size(s_btn_day, 138, 80);
    lv_obj_align(s_btn_day, LV_ALIGN_TOP_LEFT, 12, 150);
    lv_obj_set_style_bg_color(s_btn_day, CLR_DAY_BG, 0);
    lv_obj_set_style_radius(s_btn_day, 10, 0);
    lv_obj_add_event_cb(s_btn_day, on_btn_day, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_day = lv_label_create(s_btn_day);
    lv_label_set_text(lbl_day, LV_SYMBOL_IMAGE "\nDAY");
    lv_obj_set_style_text_font(lbl_day, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_day, CLR_WHITE, 0);
    lv_obj_set_style_text_align(lbl_day, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_day);

    s_btn_night = lv_btn_create(scr);
    lv_obj_set_size(s_btn_night, 138, 80);
    lv_obj_align(s_btn_night, LV_ALIGN_TOP_RIGHT, -12, 150);
    lv_obj_set_style_bg_color(s_btn_night, CLR_NIGHT_BG, 0);
    lv_obj_set_style_radius(s_btn_night, 10, 0);
    lv_obj_add_event_cb(s_btn_night, on_btn_night, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_night = lv_label_create(s_btn_night);
    lv_label_set_text(lbl_night, LV_SYMBOL_IMAGE "\nNIGHT");
    lv_obj_set_style_text_font(lbl_night, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_night, CLR_WHITE, 0);
    lv_obj_set_style_text_align(lbl_night, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_night);

    /* ── Brightness controls ─────────────────────────────────────────────── */

    /* Minus button */
    lv_obj_t *btn_minus = lv_btn_create(scr);
    lv_obj_set_size(btn_minus, 48, 48);
    lv_obj_align(btn_minus, LV_ALIGN_TOP_LEFT, 12, 260);
    lv_obj_set_style_bg_color(btn_minus, CLR_BTN_PLUS, 0);
    lv_obj_set_style_radius(btn_minus, 8, 0);
    lv_obj_add_event_cb(btn_minus, on_btn_minus, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_minus = lv_label_create(btn_minus);
    lv_label_set_text(lbl_minus, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(lbl_minus, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_minus, CLR_WHITE, 0);
    lv_obj_center(lbl_minus);

    /* Slider */
    s_slider = lv_slider_create(scr);
    lv_obj_set_size(s_slider, 180, 20);
    lv_obj_align(s_slider, LV_ALIGN_TOP_MID, 0, 274);
    lv_slider_set_range(s_slider, 0, 100);
    lv_slider_set_value(s_slider, s_brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_slider, CLR_SLIDER_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_slider, CLR_SLIDER_IND, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_slider, CLR_SLIDER_KNOB, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_slider, 6, LV_PART_KNOB);
    lv_obj_add_event_cb(s_slider, on_slider_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* Plus button */
    lv_obj_t *btn_plus = lv_btn_create(scr);
    lv_obj_set_size(btn_plus, 48, 48);
    lv_obj_align(btn_plus, LV_ALIGN_TOP_RIGHT, -12, 260);
    lv_obj_set_style_bg_color(btn_plus, CLR_BTN_PLUS, 0);
    lv_obj_set_style_radius(btn_plus, 8, 0);
    lv_obj_add_event_cb(btn_plus, on_btn_plus, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_plus = lv_label_create(btn_plus);
    lv_label_set_text(lbl_plus, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(lbl_plus, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_plus, CLR_WHITE, 0);
    lv_obj_center(lbl_plus);

    /* Brightness label */
    s_lbl_brightness = lv_label_create(scr);
    lv_obj_align(s_lbl_brightness, LV_ALIGN_TOP_MID, 0, 310);
    lv_obj_set_style_text_font(s_lbl_brightness, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lbl_brightness, CLR_GRAY, 0);
    update_slider_label();

    /* ── WiFi status ─────────────────────────────────────────────────────── */
    s_lbl_status = lv_label_create(scr);
    lv_obj_align(s_lbl_status, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(s_lbl_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_status, CLR_GRAY, 0);
    lv_label_set_text(s_lbl_status, "");
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ui_init(void)
{
    s_scr_control = lv_obj_create(NULL);
    build_control_screen(s_scr_control);

    s_scr_settings = lv_obj_create(NULL);
    ui_settings_create(s_scr_settings);

    lv_scr_load(s_scr_control);

    /* Timer to check long-press on title (runs every 500ms) */
    s_longpress_timer = lv_timer_create(longpress_check_cb, 500, NULL);

    ESP_LOGI(TAG, "UI init OK");
}

void ui_show_settings(void)
{
    lv_scr_load(s_scr_settings);
}

void ui_show_control(void)
{
    lv_scr_load(s_scr_control);
}

void ui_update_status(void)
{
    if (!s_lbl_status) return;

    wifi_sta_status_t st = wifi_sta_get_status();
    switch (st) {
    case WIFI_STA_CONNECTED:
        lv_label_set_text_fmt(s_lbl_status, LV_SYMBOL_WIFI "  %s", wifi_sta_get_ip());
        lv_obj_set_style_text_color(s_lbl_status, CLR_GREEN, 0);
        break;
    case WIFI_STA_CONNECTING:
        lv_label_set_text(s_lbl_status, "Connecting...");
        lv_obj_set_style_text_color(s_lbl_status, lv_color_hex(0xf39c12), 0);
        break;
    case WIFI_STA_FAILED:
        lv_label_set_text(s_lbl_status, "WiFi Failed");
        lv_obj_set_style_text_color(s_lbl_status, CLR_RED, 0);
        break;
    default:
        lv_label_set_text(s_lbl_status, "No WiFi");
        lv_obj_set_style_text_color(s_lbl_status, CLR_GRAY, 0);
        break;
    }
}
```

- [ ] **Step 4: Commit**

```bash
git add components/ui/
git commit -m "feat: implement LED SCREEN control UI with ON/OFF, DAY/NIGHT, brightness"
```

---

## Task 11: Create Settings Screen

**Files:**
- Create: `components/ui/ui_settings.c`

- [ ] **Step 1: Write ui_settings.c**

```c
/*
 * ui_settings.c — Settings screen (accessed via 10s title hold)
 *
 * Fields: WiFi SSID, WiFi Password, VSX400 IP, TCP Port, Day %, Night %
 * Buttons: SAVE, BACK
 */

#include "lvgl.h"
#include "esp_log.h"

#include "storage.h"
#include "wifi_sta.h"
#include "novastar.h"
#include "ui.h"

static const char *TAG = "ui_settings";

#define CLR_BG          lv_color_hex(0x1a1a2e)
#define CLR_FIELD_BG    lv_color_hex(0x2c3e50)
#define CLR_WHITE       lv_color_hex(0xffffff)
#define CLR_GREEN       lv_color_hex(0x2ecc40)
#define CLR_GRAY        lv_color_hex(0x888888)
#define CLR_TITLE_BG    lv_color_hex(0x0a3d91)

static lv_obj_t *s_ta_ssid = NULL;
static lv_obj_t *s_ta_pass = NULL;
static lv_obj_t *s_ta_ip = NULL;
static lv_obj_t *s_ta_port = NULL;
static lv_obj_t *s_ta_day = NULL;
static lv_obj_t *s_ta_night = NULL;
static lv_obj_t *s_kb = NULL;

static lv_obj_t *s_focused_ta = NULL;

/* ── Keyboard management ─────────────────────────────────────────────────── */

static void show_keyboard(lv_obj_t *ta, bool numeric)
{
    s_focused_ta = ta;
    if (numeric) {
        lv_keyboard_set_mode(s_kb, LV_KEYBOARD_MODE_NUMBER);
    } else {
        lv_keyboard_set_mode(s_kb, LV_KEYBOARD_MODE_TEXT_LOWER);
    }
    lv_keyboard_set_textarea(s_kb, ta);
    lv_obj_clear_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
}

static void hide_keyboard(void)
{
    lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(s_kb, NULL);
    s_focused_ta = NULL;
}

static void on_ta_focus(lv_event_t *e)
{
    lv_obj_t *ta = lv_event_get_target(e);
    bool numeric = (ta == s_ta_port || ta == s_ta_day || ta == s_ta_night);
    show_keyboard(ta, numeric);
}

static void on_kb_ready(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        hide_keyboard();
    }
}

/* ── Save / Back ─────────────────────────────────────────────────────────── */

static void on_save(lv_event_t *e)
{
    (void)e;
    hide_keyboard();

    const settings_t *old = storage_get_settings();
    settings_t s;

    strncpy(s.wifi_ssid, lv_textarea_get_text(s_ta_ssid), sizeof(s.wifi_ssid) - 1);
    s.wifi_ssid[sizeof(s.wifi_ssid) - 1] = '\0';
    strncpy(s.wifi_pass, lv_textarea_get_text(s_ta_pass), sizeof(s.wifi_pass) - 1);
    s.wifi_pass[sizeof(s.wifi_pass) - 1] = '\0';
    strncpy(s.nova_ip, lv_textarea_get_text(s_ta_ip), sizeof(s.nova_ip) - 1);
    s.nova_ip[sizeof(s.nova_ip) - 1] = '\0';

    const char *port_str = lv_textarea_get_text(s_ta_port);
    s.nova_port = (uint16_t)atoi(port_str);
    if (s.nova_port == 0) s.nova_port = 5200;

    const char *day_str = lv_textarea_get_text(s_ta_day);
    int day_val = atoi(day_str);
    s.day_bright = (uint8_t)(day_val > 100 ? 100 : (day_val < 0 ? 0 : day_val));

    const char *night_str = lv_textarea_get_text(s_ta_night);
    int night_val = atoi(night_str);
    s.night_bright = (uint8_t)(night_val > 100 ? 100 : (night_val < 0 ? 0 : night_val));

    storage_save_settings(&s);
    novastar_set_target(s.nova_ip, s.nova_port);

    /* Reconnect WiFi if credentials changed */
    bool wifi_changed = strcmp(old->wifi_ssid, s.wifi_ssid) != 0 ||
                        strcmp(old->wifi_pass, s.wifi_pass) != 0;
    if (wifi_changed && s.wifi_ssid[0] != '\0') {
        wifi_sta_disconnect();
        wifi_sta_connect(s.wifi_ssid, s.wifi_pass);
    }

    ESP_LOGI(TAG, "Settings saved");
    ui_show_control();
}

static void on_back(lv_event_t *e)
{
    (void)e;
    hide_keyboard();
    ui_show_control();
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static lv_obj_t *add_field(lv_obj_t *parent, const char *label_text, int y,
                           int max_len, bool password)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, label_text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, CLR_GRAY, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 16, y);

    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_size(ta, 186, 36);
    lv_obj_align(ta, LV_ALIGN_TOP_RIGHT, -16, y - 4);
    lv_textarea_set_max_length(ta, max_len);
    lv_textarea_set_one_line(ta, true);
    lv_obj_set_style_bg_color(ta, CLR_FIELD_BG, 0);
    lv_obj_set_style_text_color(ta, CLR_WHITE, 0);
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_14, 0);
    lv_obj_set_style_border_color(ta, CLR_GRAY, 0);
    lv_obj_set_style_border_width(ta, 1, 0);
    lv_obj_set_style_radius(ta, 4, 0);
    if (password) {
        lv_textarea_set_password_mode(ta, true);
    }
    lv_obj_add_event_cb(ta, on_ta_focus, LV_EVENT_FOCUSED, NULL);

    return ta;
}

/* ── Build settings screen ───────────────────────────────────────────────── */

void ui_settings_create(lv_obj_t *scr)
{
    lv_obj_set_style_bg_color(scr, CLR_BG, 0);

    /* Title */
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, 300, 40);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_bg_color(title_bar, CLR_TITLE_BG, 0);
    lv_obj_set_style_radius(title_bar, 6, 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(title_bar);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(title, CLR_WHITE, 0);
    lv_obj_center(title);

    /* Fields */
    const settings_t *cfg = storage_get_settings();
    int y = 60;
    int spacing = 44;

    s_ta_ssid = add_field(scr, "WiFi SSID", y, 32, false);
    lv_textarea_set_text(s_ta_ssid, cfg->wifi_ssid);

    s_ta_pass = add_field(scr, "WiFi Pass", y + spacing, 64, true);
    lv_textarea_set_text(s_ta_pass, cfg->wifi_pass);

    s_ta_ip = add_field(scr, "VSX400 IP", y + spacing * 2, 15, false);
    lv_textarea_set_text(s_ta_ip, cfg->nova_ip);

    s_ta_port = add_field(scr, "TCP Port", y + spacing * 3, 5, false);
    char port_buf[8];
    snprintf(port_buf, sizeof(port_buf), "%u", cfg->nova_port);
    lv_textarea_set_text(s_ta_port, port_buf);

    s_ta_day = add_field(scr, "Day %", y + spacing * 4, 3, false);
    char day_buf[8];
    snprintf(day_buf, sizeof(day_buf), "%u", cfg->day_bright);
    lv_textarea_set_text(s_ta_day, day_buf);

    s_ta_night = add_field(scr, "Night %", y + spacing * 5, 3, false);
    char night_buf[8];
    snprintf(night_buf, sizeof(night_buf), "%u", cfg->night_bright);
    lv_textarea_set_text(s_ta_night, night_buf);

    /* SAVE / BACK buttons */
    int btn_y = y + spacing * 6 + 10;

    lv_obj_t *btn_save = lv_btn_create(scr);
    lv_obj_set_size(btn_save, 130, 44);
    lv_obj_align(btn_save, LV_ALIGN_TOP_LEFT, 16, btn_y);
    lv_obj_set_style_bg_color(btn_save, CLR_GREEN, 0);
    lv_obj_set_style_radius(btn_save, 8, 0);
    lv_obj_add_event_cb(btn_save, on_save, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "SAVE");
    lv_obj_set_style_text_font(lbl_save, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_save, CLR_WHITE, 0);
    lv_obj_center(lbl_save);

    lv_obj_t *btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 130, 44);
    lv_obj_align(btn_back, LV_ALIGN_TOP_RIGHT, -16, btn_y);
    lv_obj_set_style_bg_color(btn_back, CLR_FIELD_BG, 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_add_event_cb(btn_back, on_back, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl_back, CLR_WHITE, 0);
    lv_obj_center(lbl_back);

    /* Keyboard (hidden by default) */
    s_kb = lv_keyboard_create(scr);
    lv_obj_set_size(s_kb, 320, 180);
    lv_obj_align(s_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_kb, on_kb_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(s_kb, on_kb_ready, LV_EVENT_CANCEL, NULL);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/ui/ui_settings.c
git commit -m "feat: add settings screen with WiFi, IP, brightness presets config"
```

---

## Task 12: Rewrite app_main and Build Config

**Files:**
- Modify: `main/app_main.c`
- Modify: `main/CMakeLists.txt`
- Modify: `sdkconfig.defaults`
- Modify: `partitions.csv`

- [ ] **Step 1: Update main/CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES lcd_init lvgl_port touch board storage novastar wifi_sta ui freertos log esp_lcd esp_driver_i2c
)
```

- [ ] **Step 2: Rewrite app_main.c**

```c
/*
 * app_main.c — Novastar VSX400 LED Screen Remote
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lvgl.h"

#include "board.h"
#include "lcd_init.h"
#include "lvgl_port.h"
#include "touch.h"
#include "storage.h"
#include "novastar.h"
#include "wifi_sta.h"
#include "ui.h"

static const char *TAG = "app_main";

/* Periodic task to update WiFi status on UI */
static void status_task(void *arg)
{
    for (;;) {
        if (lvgl_port_lock(100)) {
            ui_update_status();
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Novastar VSX400 LED Remote — init");

    /* Load settings from NVS */
    ESP_ERROR_CHECK(storage_init());
    const settings_t *cfg = storage_get_settings();

    /* Set Novastar target */
    novastar_set_target(cfg->nova_ip, cfg->nova_port);

    /* WiFi STA init + connect if configured */
    ESP_ERROR_CHECK(wifi_sta_init());
    if (storage_has_wifi()) {
        wifi_sta_connect(cfg->wifi_ssid, cfg->wifi_pass);
    }

    /* LCD init */
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    i2c_master_bus_handle_t i2c_bus;

    ESP_ERROR_CHECK(lcd_init(lvgl_port_get_flush_done_cb(),
                             lvgl_port_get_flush_done_user_data(),
                             &io, &panel, &i2c_bus));

    /* LVGL init */
    lvgl_port_init(io, panel);
    lcd_backlight_set(80);

    /* Touch + UI init */
    lvgl_port_lock(1000);
    touch_init();
    ui_init();

    /* If no WiFi configured, open settings immediately */
    if (!storage_has_wifi()) {
        ui_show_settings();
    }

    lvgl_port_unlock();

    /* Start LVGL task */
    lvgl_port_start();

    /* Start WiFi status update task */
    xTaskCreate(status_task, "status", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "Init complete — running");
}
```

- [ ] **Step 3: Update sdkconfig.defaults**

```
# sdkconfig.defaults — Waveshare ESP32-S3-Touch-LCD-3.5B
# Novastar VSX400 LED Screen Remote

# ── Target ────────────────────────────────────────────────────────────────────
CONFIG_IDF_TARGET="esp32s3"

# ── CPU speed ─────────────────────────────────────────────────────────────────
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ_240=y

# ── PSRAM (Octal SPI, 80 MHz) ────────────────────────────────────────────────
CONFIG_ESP32S3_SPIRAM_SUPPORT=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_BOOT_INIT=y
CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y
CONFIG_SPIRAM_USE_CAPS_ALLOC=y

# ── Flash (16 MB) ────────────────────────────────────────────────────────────
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y

# ── Partition table ──────────────────────────────────────────────────────────
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"

# ── Logging ──────────────────────────────────────────────────────────────────
CONFIG_LOG_DEFAULT_LEVEL_INFO=y

# ── FreeRTOS ─────────────────────────────────────────────────────────────────
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_UNICORE=n

# ── Stack sizes ──────────────────────────────────────────────────────────────
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192

# ── WiFi ─────────────────────────────────────────────────────────────────────
CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10
CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32

# ── LVGL (via Kconfig) ──────────────────────────────────────────────────────
CONFIG_LV_COLOR_DEPTH_16=y
CONFIG_LV_COLOR_16_SWAP=y
CONFIG_LV_MEM_CUSTOM=y
CONFIG_LV_MEM_CUSTOM_INCLUDE="stdlib.h"
CONFIG_LV_DISP_DEF_REFR_PERIOD=16
CONFIG_LV_INDEV_DEF_READ_PERIOD=20
CONFIG_LV_USE_LOG=y
CONFIG_LV_LOG_LEVEL_WARN=y
CONFIG_LV_LOG_PRINTF=y

# Fonts
CONFIG_LV_FONT_MONTSERRAT_12=y
CONFIG_LV_FONT_MONTSERRAT_14=y
CONFIG_LV_FONT_MONTSERRAT_18=y
CONFIG_LV_FONT_MONTSERRAT_24=y
CONFIG_LV_FONT_DEFAULT_MONTSERRAT_14=y

# Theme
CONFIG_LV_USE_THEME_DEFAULT=y
CONFIG_LV_THEME_DEFAULT_DARK=y
```

- [ ] **Step 4: Update partitions.csv**

```csv
# Name,   Type, SubType, Offset,   Size,   Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  0x300000,
```

- [ ] **Step 5: Clean old build artifacts and config**

```bash
cd "/mnt/2tbstorage/Waveshare/4.3inlcdtouch/AV Controller/av-controller"
rm -rf build sdkconfig
```

- [ ] **Step 6: Commit**

```bash
git add main/app_main.c main/CMakeLists.txt sdkconfig.defaults partitions.csv
git commit -m "feat: complete app_main, build config, and partitions for LED remote"
```

---

## Task 13: Build and Fix Compilation Errors

- [ ] **Step 1: Set IDF target and build**

```bash
cd "/mnt/2tbstorage/Waveshare/4.3inlcdtouch/AV Controller/av-controller"
source $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build
```

- [ ] **Step 2: Fix any compilation errors that arise**

Common issues to watch for:
- Missing `#include <stdlib.h>` in ui_settings.c for `atoi()`
- Legacy vs new I2C API conflicts (touch uses legacy, lcd_init uses new — may need separate I2C bus instances or use legacy for both)
- LVGL symbol macros (LV_SYMBOL_IMAGE etc.) may differ between versions
- Component dependency resolution issues in CMake

- [ ] **Step 3: Commit fixes**

```bash
git add -A
git commit -m "fix: resolve compilation errors"
```

---

## Task 14: Flash and Test

- [ ] **Step 1: Flash to device**

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

- [ ] **Step 2: Verify boot sequence**
Expected log output:
```
I (xxx) app_main: Novastar VSX400 LED Remote — init
I (xxx) storage: Settings loaded (SSID: "", VSX400: 192.168.1.50:5200, ...)
I (xxx) wifi_sta: WiFi STA initialized
I (xxx) lcd_init: AXS15231B QSPI LCD init OK — 320x480 portrait
I (xxx) touch: AXS15231B touch init OK
I (xxx) ui: UI init OK
```

- [ ] **Step 3: Test settings screen (should auto-open on first boot — no WiFi configured)**

- [ ] **Step 4: Configure WiFi and VSX400 IP, save, verify connection**

- [ ] **Step 5: Test ON/OFF, DAY/NIGHT, brightness slider, +/- buttons**

- [ ] **Step 6: Commit any fixes from testing**
