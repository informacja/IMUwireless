idf_component_register(SRCS "led_strip_example_main.c" "led_strip_encoder.c"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_i2c
                       INCLUDE_DIRS ".")
