idf_component_register(SRCS mainBLEmpu6050.c mpu6050.c
                       PRIV_REQUIRES esp_driver_rmt esp_driver_i2c esp_wifi esp_event esp_netif nvs_flash esp_timer lwip bt driver nvs_flash esp_timer
                       INCLUDE_DIRS ".")