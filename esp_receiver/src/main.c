/*
 * ESP32-C3 Empfänger/Dongle - UART <-> ESP-NOW Bridge
 * Empfängt UART vom Pico, sendet via ESP-NOW, Antwort zurück via UART
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

// UART Config - UART1 für GPIO Pins zum Pico
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 21  // TX zum Pico RX (GP1)
#define UART_RX_PIN 20  // RX vom Pico TX (GP0)
#define UART_BUF_SIZE 256

// MAC-Adresse des Senders (COM11)
static uint8_t sender_mac[6] = {0x20, 0x6e, 0xf1, 0x6a, 0xa3, 0xb8};

// Queue für empfangene ESP-NOW Daten
static QueueHandle_t espnow_queue;

typedef struct {
    uint8_t data[250];
    int len;
} espnow_msg_t;

// Callback wenn ESP-NOW Daten empfangen
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "ESP-NOW empfangen (%d bytes)", len);
    
    espnow_msg_t msg;
    msg.len = len > 250 ? 250 : len;
    memcpy(msg.data, data, msg.len);
    
    xQueueSend(espnow_queue, &msg, 0);
}

// Callback wenn Daten gesendet (ESP-IDF 5.5+ API)
static void on_data_sent(const esp_now_send_info_t *send_info, esp_now_send_status_t status) {
    ESP_LOGD(TAG, "Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
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
    ESP_LOGI(TAG, "Eigene MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, sender_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    
    ESP_LOGI(TAG, "ESP-NOW initialisiert");
}

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 921600,  // Hohe Baudrate für minimale Latenz
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // TX=GPIO21, RX=GPIO20 für Verbindung zum Pico
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART1 initialisiert (TX=GPIO%d, RX=GPIO%d)", UART_TX_PIN, UART_RX_PIN);
}

void bridge_task(void *arg) {
    uint8_t uart_buf[UART_BUF_SIZE];
    espnow_msg_t espnow_msg;
    
    while (1) {
        // UART vom Pico lesen - minimaler Timeout für Latenz
        int len = uart_read_bytes(UART_NUM, uart_buf, UART_BUF_SIZE - 1, 1);  // 1 Tick = ~1ms max
        if (len > 0) {
            uart_buf[len] = '\0';
            ESP_LOGD(TAG, "UART empfangen (%d bytes): %s", len, uart_buf);  // Debug statt Info
            
            // Via ESP-NOW an Sender schicken - sofort, kein Logging
            esp_now_send(sender_mac, uart_buf, len);
        }
        
        // ESP-NOW Antworten an UART weiterleiten - sofort
        while (xQueueReceive(espnow_queue, &espnow_msg, 0) == pdTRUE) {
            ESP_LOGI(TAG, "UART TX (%d bytes): %.*s", espnow_msg.len, espnow_msg.len, espnow_msg.data);
            uart_write_bytes(UART_NUM, espnow_msg.data, espnow_msg.len);
        }
        
        // Kein vTaskDelay - yield nur wenn nichts zu tun
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
