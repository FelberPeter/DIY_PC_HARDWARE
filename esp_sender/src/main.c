/*
 * ESP32-C3 Sender/Device - Low-latency HID event source
 * Sends compact binary frames (versioned) over ESP-NOW.
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

// GPIOs for buttons (expandable)
#define NUM_KEYS        8
static const uint8_t BUTTON_GPIOS[NUM_KEYS] = {0, 1, 2, 3, 4, 5, 6, 7};
#define DEBOUNCE_US     20000  // 20ms debounce

// Built-in LED (ESP32-C3 has LED on GPIO8)
#define LED_GPIO        8

// Receiver MAC address (dongle)
static uint8_t receiver_mac[6] = {0x20, 0x6e, 0xf1, 0x6a, 0xc4, 0xb0};

// Sender ID (1..255). Keep stable per device.
#define SENDER_ID 1

// Event Queue for GPIO interrupts
static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    uint8_t gpio;
    bool pressed;
    int64_t timestamp;
} gpio_event_t;

// GPIO to key mapping (example: '0'..'7')
static char gpio_to_key(uint8_t gpio) {
    for (int i = 0; i < NUM_KEYS; i++) {
        if (BUTTON_GPIOS[i] == gpio) {
            return (char)('0' + i);
        }
    }
    return '?';
}

// ---- Protocol (binary frame) ----
#define PROTO_SYNC0 0xA5
#define PROTO_SYNC1 0x5A
#define PROTO_VER   1
#define PROTO_MAX_PAYLOAD 32

typedef enum {
    MSG_KEY_EVENT = 0x01,   // payload: keycode(1), action(1)
    MSG_KEY_STATE = 0x02,   // payload: bitmask_lo, bitmask_hi
    MSG_CONTROL   = 0x10,   // payload: cmd(1), value(1)
} msg_type_t;

typedef enum {
    KEY_PRESS   = 1,
    KEY_RELEASE = 0,
} key_action_t;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t frame_seq = 0;

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len) {
    if (len > PROTO_MAX_PAYLOAD) {
        return;
    }

    uint8_t frame[7 + PROTO_MAX_PAYLOAD + 1];
    uint8_t idx = 0;
    frame[idx++] = PROTO_SYNC0;
    frame[idx++] = PROTO_SYNC1;
    frame[idx++] = PROTO_VER;
    frame[idx++] = type;
    frame[idx++] = SENDER_ID;
    frame[idx++] = frame_seq++;
    frame[idx++] = len;

    uint8_t crc = 0;
    for (int i = 2; i < idx; i++) {
        crc = crc8_update(crc, frame[i]);
    }
    for (int i = 0; i < len; i++) {
        frame[idx++] = payload[i];
        crc = crc8_update(crc, payload[i]);
    }
    frame[idx++] = crc;

    esp_now_send(receiver_mac, frame, idx);
}

// ISR handler - keep it fast
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint8_t gpio = (uint8_t)(uintptr_t)arg;
    gpio_event_t evt = {
        .gpio = gpio,
        .pressed = (gpio_get_level(gpio) == 0),  // Active LOW
        .timestamp = esp_timer_get_time()
    };
    xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
}

// Callback when data received (control)
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    (void)recv_info;
    if (len < 8) {
        return;
    }
    if (data[0] != PROTO_SYNC0 || data[1] != PROTO_SYNC1) {
        return;
    }
    uint8_t ver = data[2];
    uint8_t type = data[3];
    uint8_t payload_len = data[6];
    if (ver != PROTO_VER || (7 + payload_len + 1) != len) {
        return;
    }

    uint8_t crc = 0;
    for (int i = 2; i < 7 + payload_len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    if (crc != data[7 + payload_len]) {
        return;
    }

    // Control: LED command (cmd=1)
    if (type == MSG_CONTROL && payload_len >= 2) {
        uint8_t cmd = data[7];
        uint8_t value = data[8];
        if (cmd == 1) {
            gpio_set_level(LED_GPIO, value ? 1 : 0);
        }
    }
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

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, receiver_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW initialized");
}

// GPIO initialization for buttons
static void button_init(void) {
    gpio_evt_queue = xQueueCreate(20, sizeof(gpio_event_t));

    uint64_t mask = 0;
    for (int i = 0; i < NUM_KEYS; i++) {
        mask |= (1ULL << BUTTON_GPIOS[i]);
    }
    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    for (int i = 0; i < NUM_KEYS; i++) {
        gpio_isr_handler_add(BUTTON_GPIOS[i], gpio_isr_handler, (void*)(uintptr_t)BUTTON_GPIOS[i]);
    }

    ESP_LOGI(TAG, "Buttons initialized on GPIO0-7");
}

// Debounce tracking per GPIO (indexed by GPIO number 0..7)
static int64_t last_event_time[NUM_KEYS] = {0};

// Track pressed state by index
static bool key_pressed[NUM_KEYS] = {false};

// Key Event Task
static void key_event_task(void *arg) {
    (void)arg;
    gpio_event_t evt;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, pdMS_TO_TICKS(10))) {
            if (evt.gpio >= NUM_KEYS) {
                continue;
            }
            if ((evt.timestamp - last_event_time[evt.gpio]) < DEBOUNCE_US) {
                continue;
            }
            last_event_time[evt.gpio] = evt.timestamp;
            key_pressed[evt.gpio] = evt.pressed;

            uint8_t payload[2];
            payload[0] = (uint8_t)gpio_to_key(evt.gpio);
            payload[1] = evt.pressed ? KEY_PRESS : KEY_RELEASE;
            send_frame(MSG_KEY_EVENT, payload, sizeof(payload));
        }
    }
}

// Periodic key state sync (bitmask) for safety
static void key_state_task(void *arg) {
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        uint16_t mask = 0;
        for (int i = 0; i < NUM_KEYS; i++) {
            bool currently_pressed = (gpio_get_level(BUTTON_GPIOS[i]) == 0);
            if (currently_pressed) {
                mask |= (1U << i);
            }
        }

        uint8_t payload[2];
        payload[0] = (uint8_t)(mask & 0xFF);
        payload[1] = (uint8_t)((mask >> 8) & 0xFF);
        send_frame(MSG_KEY_STATE, payload, sizeof(payload));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== ESP-NOW SENDER ===");

    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_GPIO, 0);

    wifi_init();
    espnow_init();
    button_init();

    xTaskCreate(key_event_task, "key_evt", 2048, NULL, 10, NULL);
    xTaskCreate(key_state_task, "key_state", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "Ready. Sender ID=%d, LED GPIO=%d", SENDER_ID, LED_GPIO);
}
