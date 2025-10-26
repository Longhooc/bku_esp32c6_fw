idf_component_register(SRCS "sensor.c" "espnow_example_main.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi bmi160 driver
                    INCLUDE_DIRS ".")
