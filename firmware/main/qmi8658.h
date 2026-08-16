#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct { float ax, ay, az, gx, gy, gz; } qmi_sample_t;

esp_err_t qmi8658_init(i2c_master_bus_handle_t bus);
esp_err_t qmi8658_read(qmi_sample_t *out);
bool qmi8658_ok(void);
