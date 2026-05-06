#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "debug.h"

#define WIFI_SSID "TODO"
#define WIFI_PASS "TODO"
#define WIFI_MAX_RETRY 5

#define TCP_SERVER_IP "192.168.0.69"
#define TCP_SERVER_PORT 5000

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "uart_tcp";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t debug_network_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi SSID: %s", WIFI_SSID);

        int rssi;
        esp_wifi_sta_get_rssi(&rssi);
        ESP_LOGI(TAG, "RSSI: %d", rssi);

        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to Wi-Fi SSID: %s", WIFI_SSID);
    return ESP_FAIL;
}

int debug_tcp_connect(void) {
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(TCP_SERVER_IP),
        .sin_family = AF_INET,
        .sin_port = htons(TCP_SERVER_PORT),
    };

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Socket connect failed: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Connected to %s:%d", TCP_SERVER_IP, TCP_SERVER_PORT);
    return sock;
}

int debug_tcp_send(int sock, const void *data, size_t len) {
    if (data == NULL || len == 0) {
        return -1;
    }

    ESP_LOGI(TAG, "%s", (const char *)data);

    return sock ? send(sock, data, len, 0) : 0;
}

int debug_tcp_printf(int sock, const char *format, ...) {
    if (format == NULL) {
        return -1;
    }

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return -1;
    }

    return debug_tcp_send(sock, buffer, len);
}

void debug_tcp_hexdump(int sock, const void *data, size_t len) {
    if (sock < 0 || data == NULL || len == 0) {
        return;
    }

    const uint8_t *byte_data = (const uint8_t *)data;
    char line[17]; // 16 bytes + null terminator

    for (size_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            if (i > 0) {
                debug_tcp_printf(sock, "  %s\n", line);
            }
            debug_tcp_printf(sock, "%08x: ", (unsigned int)i);
        }

        debug_tcp_printf(sock, "%02x ", byte_data[i]);
        line[i % 16] = (byte_data[i] >= 32 && byte_data[i] <= 126) ? byte_data[i] : '.';
        line[(i % 16) + 1] = '\0';
    }

    // Print remaining bytes in the last line
    if (len % 16 != 0) {
        for (size_t j = len % 16; j < 16; j++) {
            debug_tcp_printf(sock, "   ");
        }
        debug_tcp_printf(sock, "  %s\n", line);
    }
}

void debug_tcp_close(int sock) {
    if (sock >= 0) {
        close(sock);
    }
}
