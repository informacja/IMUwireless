#pragma once
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    float ax, ay, az;   // przyspieszenie [g]
    float gx, gy, gz;   // predkosc katowa [deg/s]
} mpu6050_data_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read(mpu6050_data_t *out);
