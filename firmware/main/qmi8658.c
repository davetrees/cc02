/* QMI8658 IMU on the shared I2C bus (SDA=8 SCL=7). Register recipe proven on
 * the LCD-2 sibling board (same chip): reset, auto-inc, accel ±4g 250Hz,
 * gyro ±512dps 250Hz. WHO_AM_I (0x00) = 0x05. */
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "qmi8658.h"

static const char *TAG = "qmi8658";
static i2c_master_dev_handle_t s_dev;
static bool s_ok = false;

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, 100);
}

static esp_err_t rd(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, n, 100);
}

bool qmi8658_ok(void) { return s_ok; }

esp_err_t qmi8658_init(i2c_master_bus_handle_t bus)
{
    const uint8_t addrs[2] = { 0x6B, 0x6A };
    for (int a = 0; a < 2; a++) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[a],
            .scl_speed_hz = 400000,
        };
        if (i2c_master_bus_add_device(bus, &cfg, &s_dev) != ESP_OK) continue;
        uint8_t who = 0;
        if (rd(0x00, &who, 1) == ESP_OK && who == 0x05) {
            ESP_LOGI(TAG, "found at 0x%02X", addrs[a]);
            wr(0x60, 0xB0);                 /* soft reset */
            vTaskDelay(pdMS_TO_TICKS(15));
            wr(0x02, 0x40);                 /* CTRL1: auto-increment */
            wr(0x03, 0x95);                 /* CTRL2: accel ±4g 250Hz */
            wr(0x04, 0xD5);                 /* CTRL3: gyro ±512dps 250Hz */
            wr(0x08, 0x03);                 /* CTRL7: enable accel+gyro */
            vTaskDelay(pdMS_TO_TICKS(10));
            s_ok = true;
            return ESP_OK;
        }
        i2c_master_bus_rm_device(s_dev);
    }
    ESP_LOGE(TAG, "not found (graceful: continuing without IMU)");
    return ESP_FAIL;
}

esp_err_t qmi8658_read(qmi_sample_t *out)
{
    if (!s_ok) return ESP_FAIL;
    uint8_t raw[12];
    esp_err_t err = rd(0x35, raw, 12);
    if (err != ESP_OK) return err;
    int16_t v[6];
    for (int i = 0; i < 6; i++) v[i] = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
    const float g = 9.80665f;
    out->ax = v[0] / 8192.0f * g;           /* ±4g -> 8192 LSB/g */
    out->ay = v[1] / 8192.0f * g;
    out->az = v[2] / 8192.0f * g;
    const float d2r = (float)M_PI / 180.0f;
    out->gx = v[3] / 64.0f * d2r;           /* ±512dps -> 64 LSB/dps */
    out->gy = v[4] / 64.0f * d2r;
    out->gz = v[5] / 64.0f * d2r;
    return ESP_OK;
}
