idf_component_register(
    SRCS "app_main.c" "link.c" "qmi8658.c" "cluster_ui.c"
    INCLUDE_DIRS "."
    REQUIRES esp_bsp esp_lv_port freertos esp_timer driver nvs_flash
             esp_driver_usb_serial_jtag esp_driver_i2c
             espressif__esp_io_expander_tca9554
)
