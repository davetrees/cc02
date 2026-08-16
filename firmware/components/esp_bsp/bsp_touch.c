#include <stdio.h>
#include <string.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "bsp_touch.h"
#include "bsp_i2c.h"

static uint16_t g_rotation = 0;
static uint16_t g_width = 0;
static uint16_t g_height = 0;

static i2c_master_dev_handle_t dev_handle;

touch_data_t g_touch_data;

/* Debug ring buffer */
#define TOUCH_DBG_LINES 30
#define TOUCH_DBG_LINE_LEN 120
static char g_dbg_buf[TOUCH_DBG_LINES * TOUCH_DBG_LINE_LEN];
static int g_dbg_pos = 0;

static void dbg_log(const char *fmt, ...)
{
    char *line = &g_dbg_buf[(g_dbg_pos % TOUCH_DBG_LINES) * TOUCH_DBG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, TOUCH_DBG_LINE_LEN, fmt, ap);
    va_end(ap);
    g_dbg_pos++;
}

const char *bsp_touch_get_debug_log(void)
{
    static char out[TOUCH_DBG_LINES * TOUCH_DBG_LINE_LEN + 100];
    out[0] = '\0';
    int start = g_dbg_pos >= TOUCH_DBG_LINES ? g_dbg_pos - TOUCH_DBG_LINES : 0;
    int end = g_dbg_pos;
    for (int i = start; i < end; i++) {
        strcat(out, &g_dbg_buf[(i % TOUCH_DBG_LINES) * TOUCH_DBG_LINE_LEN]);
        strcat(out, "\n");
    }
    return out;
}

void bsp_touch_read(void)
{
    uint8_t data[14] = {0};
    uint8_t read_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00};
    esp_err_t err = ESP_OK;
    if (bsp_i2c_lock(0))
    {
        err = i2c_master_transmit_receive(dev_handle, read_cmd, 11, data, 14, pdMS_TO_TICKS(1000));
        bsp_i2c_unlock();
        if (err != ESP_OK)
        {
            return;
        }

        if (data[1] == 0 || data[2] == 0 || data[3] < 2 || data[5] < 2 ) {
            return ;
        }

        if (data[0] == 0xff || data[1] > 2)
        {
            g_touch_data.touch_num = 0;
            return;
        }

        g_touch_data.touch_num = data[1];
        for (int i = 0; i < g_touch_data.touch_num; i++)
        {
            g_touch_data.coords[i].x = ((data[6 * i + 2] & 0x0F) << 8) | data[6 * i + 3];
            g_touch_data.coords[i].y = ((data[6 * i + 4] & 0x0F) << 8) | data[6 * i + 5];
        }
    }
}

bool bsp_touch_get_coordinates(touch_data_t *touch_data)
{
    if ((touch_data == NULL) || (g_touch_data.touch_num == 0))
        return false;

    for (int i = 0; i < g_touch_data.touch_num; i++)
    {
        switch (g_rotation)
        {
        case 1:
            touch_data->coords[i].y = g_height - 1 - g_touch_data.coords[i].x;
            touch_data->coords[i].x = g_touch_data.coords[i].y;
            break;
        case 2:
            touch_data->coords[i].x = g_width - 1 - g_touch_data.coords[i].x;
            touch_data->coords[i].y = g_height - 1 - g_touch_data.coords[i].y;
            break;
        case 3:
            touch_data->coords[i].y = g_touch_data.coords[i].x;
            touch_data->coords[i].x = g_width - 1 - g_touch_data.coords[i].y;
            break;
        default:
            touch_data->coords[i].x = g_touch_data.coords[i].x;
            touch_data->coords[i].y = g_touch_data.coords[i].y;
            break;
        }
    }
    return true;
}

void bsp_touch_init(i2c_master_bus_handle_t bus_handle, uint16_t width, uint16_t height, uint16_t rotation)
{
    g_rotation = rotation;
    g_width = width;
    g_height = height;
    memset(g_dbg_buf, 0, sizeof(g_dbg_buf));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_AXS15231B_ADDRESS,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}
