/* USB CDC (USB-Serial-JTAG) binary link: COBS framing + CRC16-CCITT. */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/usb_serial_jtag.h"
#include "link.h"

#define FRAME_MAX 128

static SemaphoreHandle_t s_tx_mutex;
static link_rx_cb_t s_cb;

static uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
    size_t ri = 0, wi = 1, code_i = 0;
    uint8_t code = 1;
    while (ri < len) {
        if (in[ri] == 0) {
            out[code_i] = code; code = 1; code_i = wi++; ri++;
        } else {
            out[wi++] = in[ri++]; code++;
            if (code == 0xFF) { out[code_i] = code; code = 1; code_i = wi++; }
        }
    }
    out[code_i] = code;
    return wi;
}

static size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out)
{
    size_t ri = 0, wi = 0;
    while (ri < len) {
        uint8_t code = in[ri++];
        if (code == 0 || ri + code - 1 > len) return 0;
        for (uint8_t i = 1; i < code; i++) out[wi++] = in[ri++];
        if (code != 0xFF && ri < len) out[wi++] = 0;
    }
    return wi;
}

void link_send(uint8_t type, const void *payload, size_t len)
{
    if (len + 3 > FRAME_MAX) return;
    uint8_t raw[FRAME_MAX], enc[FRAME_MAX + FRAME_MAX / 254 + 3];
    raw[0] = type;
    memcpy(&raw[1], payload, len);
    uint16_t crc = crc16_ccitt(raw, len + 1);
    raw[len + 1] = crc & 0xFF;
    raw[len + 2] = crc >> 8;
    size_t n = cobs_encode(raw, len + 3, enc);
    enc[n++] = 0x00;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    /* 0 timeout: if no host is draining the buffer, drop — never block control */
    usb_serial_jtag_write_bytes(enc, n, 0);
    xSemaphoreGive(s_tx_mutex);
}

static void rx_task(void *arg)
{
    static uint8_t acc[FRAME_MAX * 2];
    static uint8_t dec[FRAME_MAX];
    size_t acc_len = 0;
    uint8_t buf[64];
    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            uint8_t c = buf[i];
            if (c == 0x00) {
                if (acc_len >= 4) {
                    size_t dl = cobs_decode(acc, acc_len, dec);
                    if (dl >= 3) {
                        uint16_t rx_crc = dec[dl - 2] | (dec[dl - 1] << 8);
                        if (crc16_ccitt(dec, dl - 2) == rx_crc && s_cb)
                            s_cb(dec[0], &dec[1], dl - 3);
                    }
                }
                acc_len = 0;
            } else if (acc_len < sizeof(acc)) {
                acc[acc_len++] = c;
            } else {
                acc_len = 0; /* overrun: resync at next delimiter */
            }
        }
    }
}

void link_init(link_rx_cb_t cb)
{
    s_cb = cb;
    s_tx_mutex = xSemaphoreCreateMutex();
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    xTaskCreatePinnedToCore(rx_task, "link_rx", 4096, NULL, 10, NULL, 0);
}
