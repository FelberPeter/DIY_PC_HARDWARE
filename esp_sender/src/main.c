/*
 * ESP32-C3 Sender/Device - Latenz-optimierte Tastatur
 * GPIO-Interrupt für minimale Latenz, sendet KEY:x:1/0 Format
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
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "ESP_SENDER";

// Taster GPIO Pins - erweitert für mehr Tasten (GPIO 0-7)
#define BUTTON_0_GPIO   0
#define BUTTON_1_GPIO   1
#define BUTTON_2_GPIO   2
#define BUTTON_3_GPIO   3
#define BUTTON_4_GPIO   4
#define BUTTON_5_GPIO   5
#define BUTTON_6_GPIO   6
#define BUTTON_7_GPIO   7
#define DEBOUNCE_US     20000  // 20ms Debounce in Mikrosekunden

// Integrierte LED (ESP32-C3 hat LED auf GPIO8)
#define LED_GPIO        8

// MAC-Adresse des Empfängers (COM13)
static uint8_t receiver_mac[6] = {0x20, 0x6e, 0xf1, 0x6a, 0xc4, 0xb0};

// Event Queue für GPIO Interrupts
static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    uint8_t gpio;
    bool pressed;
    int64_t timestamp;
} gpio_event_t;

// GPIO zu Taste Mapping (GPIO-Nummer = Taste)
static char gpio_to_key(uint8_t gpio) {
    switch(gpio) {
        case BUTTON_0_GPIO: return '0';
        case BUTTON_1_GPIO: return '1';
        case BUTTON_2_GPIO: return '2';
        case BUTTON_3_GPIO: return '3';
        case BUTTON_4_GPIO: return '4';
        case BUTTON_5_GPIO: return '5';
        case BUTTON_6_GPIO: return '6';
        case BUTTON_7_GPIO: return '7';
        default: return '?';
    }
}

// ISR Handler - schnellstmöglich, nur Queue befüllen
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint8_t gpio = (uint8_t)(uintptr_t)arg;
    gpio_event_t evt = {
        .gpio = gpio,
        .pressed = (gpio_get_level(gpio) == 0),  // Aktiv LOW
        .timestamp = esp_timer_get_time()
    };
    xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
}

// Callback wenn Daten empfangen - LED Steuerung
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "RX: %.*s", len, (char*)data);
    
    // LED Befehle verarbeiten: LED:1 oder LED:0
    if (len >= 5 && strncmp((char*)data, "LED:", 4) == 0) {
        bool led_state = (data[4] == '1');
        gpio_set_level(LED_GPIO, led_state);
        ESP_LOGI(TAG, "LED -> %s", led_state ? "AN" : "AUS");
    }
}

// Callback wenn Daten gesendet
static void on_data_sent(const esp_now_send_info_t *send_info, esp_now_send_status_t status) {
    // Kein Logging für maximale Performance
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Eigene MAC ausgeben
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Eigene MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    
    // Peer hinzufügen
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, receiver_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    
    ESP_LOGI(TAG, "ESP-NOW initialisiert, warte auf Pakete...");
}

// GPIO für Taster mit Interrupt initialisieren
static void button_init(void) {
    // Queue für GPIO Events
    gpio_evt_queue = xQueueCreate(20, sizeof(gpio_event_t));
    
    // Alle Taster-GPIOs konfigurieren (GPIO 0-7)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_0_GPIO) | (1ULL << BUTTON_1_GPIO) |
                        (1ULL << BUTTON_2_GPIO) | (1ULL << BUTTON_3_GPIO) |
                        (1ULL << BUTTON_4_GPIO) | (1ULL << BUTTON_5_GPIO) | 
                        (1ULL << BUTTON_6_GPIO) | (1ULL << BUTTON_7_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  // Beide Flanken für Press und Release
    };
    gpio_config(&io_conf);
    
    // ISR Service installieren
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_0_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_0_GPIO);
    gpio_isr_handler_add(BUTTON_1_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_1_GPIO);
    gpio_isr_handler_add(BUTTON_2_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_2_GPIO);
    gpio_isr_handler_add(BUTTON_3_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_3_GPIO);
    gpio_isr_handler_add(BUTTON_4_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_4_GPIO);
    gpio_isr_handler_add(BUTTON_5_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_5_GPIO);
    gpio_isr_handler_add(BUTTON_6_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_6_GPIO);
    gpio_isr_handler_add(BUTTON_7_GPIO, gpio_isr_handler, (void*)(uintptr_t)BUTTON_7_GPIO);
    
    ESP_LOGI(TAG, "Taster GPIO0-7 mit Interrupt initialisiert");
}

// Debounce Tracking pro GPIO
static int64_t last_event_time[8] = {0};

// Tracking welche Tasten gedrückt sind
static bool key_pressed[8] = {false};

// Key Event Task - verarbeitet GPIO Events und sendet KEY:x:1/0
static void key_event_task(void *arg) {
    gpio_event_t evt;
    char msg[16];
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, pdMS_TO_TICKS(10))) {
            // Debounce Check
            if ((evt.timestamp - last_event_time[evt.gpio]) < DEBOUNCE_US) {
                continue;  // Ignoriere Bounce
            }
            last_event_time[evt.gpio] = evt.timestamp;
            
            // Status tracken
            key_pressed[evt.gpio] = evt.pressed;
            
            // KEY:x:1 für Press, KEY:x:0 für Release
            char key = gpio_to_key(evt.gpio);
            snprintf(msg, sizeof(msg), "KEY:%c:%d\n", key, evt.pressed ? 1 : 0);
            
            // Sofort senden - ESP-NOW ist schnell
            esp_now_send(receiver_mac, (uint8_t*)msg, strlen(msg));
            
            ESP_LOGI(TAG, "TX: %s", msg);
        }
    }
}

// Key Repeat Task - sendet alle 50ms ein Signal solange Taste gehalten
static void key_repeat_task(void *arg) {
    char msg[16];
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Alle 50ms
        
        // Prüfe welche GPIOs noch gedrückt sind (hardware check)
        for (int i = 0; i < 8; i++) {
            bool currently_pressed = (gpio_get_level(i) == 0);  // Aktiv LOW
            
            if (currently_pressed && key_pressed[i]) {
                // Taste wird gehalten - sende Repeat
                char key = gpio_to_key(i);
                snprintf(msg, sizeof(msg), "KEY:%c:1\n", key);
                esp_now_send(receiver_mac, (uint8_t*)msg, strlen(msg));
            } else if (!currently_pressed && key_pressed[i]) {
                // Taste wurde losgelassen aber Event verpasst - sende Release
                key_pressed[i] = false;
                char key = gpio_to_key(i);
                snprintf(msg, sizeof(msg), "KEY:%c:0\n", key);
                esp_now_send(receiver_mac, (uint8_t*)msg, strlen(msg));
                ESP_LOGI(TAG, "TX (missed release): %s", msg);
            }
        }
    }
}

void app_main(void) {
    // NVS initialisieren
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "=== ESP-NOW SENDER - BIDIREKTIONAL ===");
    
    // LED GPIO initialisieren
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_GPIO, 0);  // LED aus
    
    wifi_init();
    espnow_init();
    button_init();
    
    // High-Priority Task für Key Events
    xTaskCreate(key_event_task, "key_evt", 2048, NULL, 10, NULL);  // Priorität 10 = hoch
    
    // Key Repeat Task für gehaltene Tasten
    xTaskCreate(key_repeat_task, "key_repeat", 2048, NULL, 5, NULL);  // Priorität 5
    
    ESP_LOGI(TAG, "Bereit! GPIO0='0', GPIO1='1', GPIO2='2', GPIO3='3'");
    ESP_LOGI(TAG, "LED auf GPIO%d steuerbar via LED:1/LED:0", LED_GPIO);
}
