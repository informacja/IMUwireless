/* ============================================================
 * ESP32-C3 (natywny ESP-IDF, bez Arduino)
 * Serwer TCP strumieniujacy dane pomiarowe do MATLABA (LAN)
 * ============================================================
 *
 * Budowanie w VS Code (rozszerzenie "Espressif IDF"):
 *   1. ESP-IDF: Set Espressif target -> esp32c3
 *   2. ESP-IDF: Build your project
 *   3. ESP-IDF: Flash your project
 *   4. ESP-IDF: Monitor your device (sprawdz przydzielony adres IP)
 *
 * Lub z terminala:
 *   idf.py set-target esp32c3
 *   idf.py build
 *   idf.py -p <PORT> flash monitor
 */

#include <string.h>
#include <math.h>
#include <sys/param.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ------------------- KONFIGURACJA ------------------- */
#include "wifi_config.h"  /* Zawiera WIFI_SSID i WIFI_PASS */
#define WIFI_MAX_RETRY  10

#define TCP_PORT        3333
#define SEND_INTERVAL_MS 100   /* 10 Hz */

static const char *TAG = "esp32c3_tcp_server";

/* Event group sygnalizujacy uzyskanie polaczenia WiFi */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

/* ------------------- OBSLUGA ZDARZEN WIFI ------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Ponawiam polaczenie z WiFi (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Polaczono. Adres IP ESP32: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Laczenie z WiFi \"%s\"...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Polaczenie WiFi nawiazane.");
    } else {
        ESP_LOGE(TAG, "Nie udalo sie polaczyc z WiFi.");
    }
}

/* ------------------- ZADANIE SERWERA TCP ------------------- */
static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Blad tworzenia gniazda: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Blad bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Blad listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Serwer TCP nasluchuje na porcie %d", TCP_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);

        ESP_LOGI(TAG, "Oczekiwanie na polaczenie klienta (MATLAB)...");
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Blad accept: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Klient MATLAB polaczony.");

        /* Tryb nieblokujacy, zeby jednoczesnie wysylac dane i sprawdzac komendy */
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        int64_t t0_us = esp_timer_get_time();

        while (1) {
            /* Odbior ewentualnej komendy z MATLABa (nieblokujaco) */
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len > 0) {
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "Otrzymano komende: %s", rx_buffer);
                /* TODO: obsluga komend, np. sterowanie GPIO */
            } else if (len == 0) {
                ESP_LOGW(TAG, "Klient rozlaczyl sie.");
                break;
            } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
                ESP_LOGE(TAG, "Blad recv: errno %d", errno);
                break;
            }

            /* Wysylanie danych pomiarowych */
            int64_t t_ms = (esp_timer_get_time() - t0_us) / 1000;

            /* TODO: podmien na odczyt z prawdziwego czujnika (ADC, I2C, SPI itd.) */
            float wartosc1 = 1.65f + 0.1f * sinf(t_ms / 500.0f);
            float wartosc2 = sinf(t_ms / 1000.0f);

            char tx_buffer[64];
            int msg_len = snprintf(tx_buffer, sizeof(tx_buffer), "%lld,%.4f,%.4f\n",
                                    (long long)t_ms, wartosc1, wartosc2);

            int to_write = msg_len;
            int written = 0;
            while (to_write > 0) {
                int w = send(sock, tx_buffer + written, to_write, 0);
                if (w < 0) {
                    if (errno == EWOULDBLOCK || errno == EAGAIN) {
                        vTaskDelay(1);
                        continue;
                    }
                    ESP_LOGE(TAG, "Blad send: errno %d", errno);
                    to_write = -1;
                    break;
                }
                written += w;
                to_write -= w;
            }
            if (to_write < 0) break; /* blad wysylania -> zamknij polaczenie */

            vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        }

        shutdown(sock, 0);
        close(sock);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

/* ------------------- PUNKT WEJSCIA ------------------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);
}
