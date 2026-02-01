/*
 * ESP32-C3 Receiver/Dongle - UART <-> ESP-NOW bridge
 * Routes binary protocol frames with sender IDs.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "ESP_RECEIVER";

// UART Config - UART1 for GPIO pins to Pico
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 21  // TX to Pico RX (GP1)
#define UART_RX_PIN 20  // RX from Pico TX (GP0)
#define UART_BUF_SIZE 256

// ---- Protocol (binary frame) ----
#define PROTO_SYNC0 0xA5
#define PROTO_SYNC1 0x5A
#define PROTO_VER   1
#define PROTO_MAX_PAYLOAD 32

#define PROTO_MIN_FRAME 8  // sync0 sync1 ver type sender seq len crc

// Sender registry (extendable)
#define MAX_PEERS 8

typedef struct {
    uint8_t sender_id;
    uint8_t mac[6];
} sender_peer_t;

static sender_peer_t peers[MAX_PEERS] = {
    {1, {0x20, 0x6e, 0xf1, 0x6a, 0xa3, 0xb8}},
};

static size_t peer_count = 1;

// Queue for received ESP-NOW data
static QueueHandle_t espnow_queue;

typedef struct {
    uint8_t data[250];
    int len;
} espnow_msg_t;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Callback when ESP-NOW data received
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    (void)recv_info;
    if (len <= 0 || len > 250) {
        return;
    }
    espnow_msg_t msg;
    msg.len = len;
    memcpy(msg.data, data, len);
    xQueueSend(espnow_queue, &msg, 0);
}

// Callback when data sent
static void on_data_sent(const esp_now_send_info_t *send_info, esp_now_send_status_t status) {
    (void)send_info;
    (void)status;
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Own MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    for (size_t i = 0; i < peer_count; i++) {
        esp_now_peer_info_t peer = {
            .channel = 0,
            .ifidx = ESP_IF_WIFI_STA,
            .encrypt = false,
        };
        memcpy(peer.peer_addr, peers[i].mac, 6);
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    }

    ESP_LOGI(TAG, "ESP-NOW initialized with %d peer(s)", (int)peer_count);
}

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));

    ESP_LOGI(TAG, "UART1 initialized (TX=GPIO%d, RX=GPIO%d)", UART_TX_PIN, UART_RX_PIN);
}

static const sender_peer_t *find_peer(uint8_t sender_id) {
    for (size_t i = 0; i < peer_count; i++) {
        if (peers[i].sender_id == sender_id) {
            return &peers[i];
        }
    }
    return NULL;
}

// Simple frame parser for UART -> ESP-NOW routing
static void bridge_task(void *arg) {
    (void)arg;
    uint8_t uart_buf[UART_BUF_SIZE];
    espnow_msg_t espnow_msg;

    uint8_t frame[7 + PROTO_MAX_PAYLOAD + 1];
    uint8_t idx = 0;
    uint8_t payload_len = 0;
    enum { WAIT_SYNC0, WAIT_SYNC1, READ_HEADER, READ_PAYLOAD, READ_CRC } state = WAIT_SYNC0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, uart_buf, UART_BUF_SIZE, 1);
        for (int i = 0; i < len; i++) {
            uint8_t b = uart_buf[i];
            switch (state) {
                case WAIT_SYNC0:
                    if (b == PROTO_SYNC0) {
                        frame[0] = b;
                        state = WAIT_SYNC1;
                    }
                    break;
                case WAIT_SYNC1:
                    if (b == PROTO_SYNC1) {
                        frame[1] = b;
                        idx = 2;
                        state = READ_HEADER;
                    } else {
                        state = WAIT_SYNC0;
                    }
                    break;
                case READ_HEADER:
                    frame[idx++] = b;
                    if (idx == 7) {
                        payload_len = frame[6];
                        if (payload_len > PROTO_MAX_PAYLOAD) {
                            state = WAIT_SYNC0;
                        } else if (payload_len == 0) {
                            state = READ_CRC;
                        } else {
                            state = READ_PAYLOAD;
                        }
                    }
                    break;
                case READ_PAYLOAD:
                    frame[idx++] = b;
                    if (idx == (uint8_t)(7 + payload_len)) {
                        state = READ_CRC;
                    }
                    break;
                case READ_CRC:
                    frame[idx++] = b;
                    if (idx == (uint8_t)(7 + payload_len + 1)) {
                        uint8_t crc = 0;
                        for (int k = 2; k < 7 + payload_len; k++) {
                            crc = crc8_update(crc, frame[k]);
                        }
                        if (crc == frame[7 + payload_len] && frame[2] == PROTO_VER) {
                            uint8_t sender_id = frame[4];
                            if (sender_id == 0xFF) {
                                for (size_t p = 0; p < peer_count; p++) {
                                    esp_now_send(peers[p].mac, frame, idx);
                                }
                            } else {
                                const sender_peer_t *peer = find_peer(sender_id);
                                if (peer) {
                                    esp_now_send(peer->mac, frame, idx);
                                }
                            }
                        }
                    }
                    state = WAIT_SYNC0;
                    break;
            }
        }

        while (xQueueReceive(espnow_queue, &espnow_msg, 0) == pdTRUE) {
            uart_write_bytes(UART_NUM, (const char *)espnow_msg.data, espnow_msg.len);
        }

        taskYIELD();
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== ESP-NOW RECEIVER/DONGLE ===");

    espnow_queue = xQueueCreate(10, sizeof(espnow_msg_t));

    uart_init();
    wifi_init();
    espnow_init();

    xTaskCreate(bridge_task, "bridge_task", 4096, NULL, 5, NULL);
}
