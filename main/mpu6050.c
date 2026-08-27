#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// Podlaczenie MPU6050 do ESP32-C3 (zmien pod swoja platytke, jesli inne piny):
//   VCC -> 3V3, GND -> GND, SCL -> GPIO8, SDA -> GPIO9
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    9
#define I2C_SCL_GPIO    8
#define I2C_FREQ_HZ     100000

#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_ACCEL_XOUT_H    0x3B

static const char *TAG = "mpu6050";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 100);
}

static void log_i2c_probe(void)
{
    ESP_LOGI(TAG, "Skanowanie I2C: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    for (uint8_t address = 0x68; address <= 0x69; address++) {
        esp_err_t err = i2c_master_probe(bus_handle, address, 100);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Urzadzenie odpowiada pod adresem 0x%02X", address);
        } else {
            ESP_LOGI(TAG, "Brak odpowiedzi pod adresem 0x%02X (%s)",
                     address, esp_err_to_name(err));
        }
    }
}

esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Blad tworzenia magistrali I2C: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(bus_handle, &device_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Blad dodawania MPU6050 do I2C: %s", esp_err_to_name(err));
        return err;
    }

    log_i2c_probe();

    uint8_t who_am_i = 0;
    err = read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 nie wykryty (WHO_AM_I=0x%02X, err=%s)",
                 who_am_i, esp_err_to_name(err));
        for (int attempt = 1; attempt <= 3; attempt++) {
            uint8_t retry_value = 0;
            esp_err_t retry_err = read_regs(REG_WHO_AM_I, &retry_value, 1);
            ESP_LOGE(TAG, "Kontrolny odczyt WHO_AM_I #%d: 0x%02X (%s)",
                     attempt, retry_value, esp_err_to_name(retry_err));
        }
        return ESP_FAIL;
    }

    // Wybudzenie z trybu sleep (domyslnie po wlaczeniu zasilania jest uspiony)
    err = write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Blad wybudzania MPU6050: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "MPU6050 zainicjalizowany poprawnie.");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *out)
{
    uint8_t raw[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax_r = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_r = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_r = (int16_t)((raw[4] << 8) | raw[5]);
    // raw[6],raw[7] = temperatura (pomijamy)
    int16_t gx_r = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_r = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_r = (int16_t)((raw[12] << 8) | raw[13]);

    // Domyslny zakres czujnika: +-2g oraz +-250 deg/s
    out->ax = ax_r / 16384.0f;
    out->ay = ay_r / 16384.0f;
    out->az = az_r / 16384.0f;
    out->gx = gx_r / 131.0f;
    out->gy = gy_r / 131.0f;
    out->gz = gz_r / 131.0f;

    return ESP_OK;
}
