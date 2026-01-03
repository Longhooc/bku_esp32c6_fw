idf_component_register(SRCS "battery.c" "sensor.c" "espnow_example_main.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi bmi160 driver esp_timer esp_adc esp-dsp
                    INCLUDE_DIRS ".")
