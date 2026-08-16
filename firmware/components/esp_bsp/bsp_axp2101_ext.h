#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** C-callable wrapper for bsp_axp2101_init (which has C++ linkage) */
esp_err_t bsp_axp2101_init_c(i2c_master_bus_handle_t bus_handle);

uint16_t bsp_axp2101_get_batt_voltage(void);
bool bsp_axp2101_is_charging(void);
bool bsp_axp2101_is_vbus_in(void);
void bsp_axp2101_shutdown(void);

#ifdef __cplusplus
}
#endif
