#include "bsp_axp2101.h"
#include "bsp_axp2101_ext.h"

extern XPowersPMU power;

extern "C" {

esp_err_t bsp_axp2101_init_c(i2c_master_bus_handle_t bus_handle) { return bsp_axp2101_init(bus_handle); }
uint16_t bsp_axp2101_get_batt_voltage(void) { return power.getBattVoltage(); }
bool bsp_axp2101_is_charging(void) { return power.isCharging(); }
bool bsp_axp2101_is_vbus_in(void) { return power.isVbusIn(); }
void bsp_axp2101_shutdown(void) { power.shutdown(); }

}
