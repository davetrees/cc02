idf_component_register(SRCS "bsp_i2c.c" "bsp_display.c" "bsp_touch.c" "bsp_axp2101.cpp" "bsp_axp2101_ext.cpp" "bsp_es8311.c" "bsp_wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_lcd" "driver" "espressif__esp_lcd_axs15231b" "XPowersLib" "espressif__esp_codec_dev" "esp_wifi" "nvs_flash" "lwip")
