/* ============================================================
 * ESP32-C3 - serwer BLE (GATT) streamujacy dane z MPU6050
 * ============================================================
 * ESP32-C3 nie obsluguje Bluetooth Classic - tylko BLE, dlatego
 * komunikacja z MATLABEM odbywa sie przez powiadomienia (notify)
 * z jednej charakterystyki GATT.
 *
 * Podlaczenie MPU6050: patrz komentarz w mpu6050.c (domyslnie
 * SDA=GPIO8, SCL=GPIO9).
 *
 * Budowanie (VS Code, rozszerzenie Espressif IDF):
 *   ESP-IDF: Set Espressif target -> esp32c3
 *   ESP-IDF: Build, Flash and Start a Monitor
 *
 * Kod bazuje na standardowym wzorcu z oficjalnego przykladu
 * ESP-IDF "bluetooth/bluedroid/ble/gatt_server". Jesli po zmianie
 * wersji ESP-IDF pojawia sie bledy kompilacji API Bluedroid,
 * warto porownac z aktualnym przykladem w komponencie IDF.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h" 
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#include "mpu6050.h"

static const char *TAG = "ble_mpu6050";
#define DEVICE_NAME "ESP32_MPU6050"

#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4
#define PROFILE_APP_ID       0

static uint16_t service_handle;
static uint16_t char_handle;
static uint16_t char_cccd_handle;
static esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_CHAR_UUID}};

static uint16_t conn_id_g = 0;
static esp_gatt_if_t gatts_if_g = ESP_GATT_IF_NONE;
static volatile bool notify_enabled = false;
static volatile bool is_connected = false;

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "Rozglaszanie BLE rozpoczete jako \"%s\"", DEVICE_NAME);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                         esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        gatts_if_g = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);

        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;

        esp_gatt_srvc_id_t service_id = {
            .id.uuid.len = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID,
            .id.inst_id = 0,
            .is_primary = true,
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);

        esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_ble_gatts_add_char(service_handle, &char_uuid, ESP_GATT_PERM_READ, char_property, NULL, NULL);
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        char_handle = param->add_char.attr_handle;
        esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
        };
        esp_ble_gatts_add_char_descr(service_handle, &descr_uuid,
                                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        char_cccd_handle = param->add_char_descr.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        conn_id_g = param->connect.conn_id;
        is_connected = true;
        ESP_LOGI(TAG, "Klient (MATLAB) polaczony przez BLE.");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        notify_enabled = false;
        ESP_LOGW(TAG, "Klient rozlaczony - wznawiam rozglaszanie.");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep && param->write.handle == char_cccd_handle && param->write.len == 2) {
            uint16_t cccd_val = param->write.value[0] | (param->write.value[1] << 8);
            notify_enabled = (cccd_val == 0x0001);
            ESP_LOGI(TAG, "Powiadomienia %s przez klienta.", notify_enabled ? "wlaczone" : "wylaczone");
        }
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT && param->reg.app_id != PROFILE_APP_ID) {
        return;
    }
    gatts_profile_event_handler(event, gatts_if, param);
}

static void sensor_task(void *arg)
{
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "Blad inicjalizacji MPU6050 - zadanie zakonczone.");
        vTaskDelete(NULL);
        return;
    }

    mpu6050_data_t d;
    while (1) {
        if (is_connected && notify_enabled && mpu6050_read(&d) == ESP_OK) {
            // Skalowanie do int16, zeby zmiescic 6 wartosci w 12 bajtach
            int16_t payload[6] = {
                (int16_t)(d.ax * 1000.0f), (int16_t)(d.ay * 1000.0f), (int16_t)(d.az * 1000.0f),
                (int16_t)(d.gx * 10.0f),   (int16_t)(d.gy * 10.0f),   (int16_t)(d.gz * 10.0f),
            };
            esp_ble_gatts_send_indicate(gatts_if_g, conn_id_g, char_handle,
                                         sizeof(payload), (uint8_t *)payload, false);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // ~20 Hz
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
