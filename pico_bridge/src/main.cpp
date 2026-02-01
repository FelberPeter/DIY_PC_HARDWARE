/*
 * Raspberry Pi Pico - USB HID Bridge
 * Parses binary frames from ESP and emits USB HID events.
 * Accepts simple text commands over USB Serial and forwards as control frames.
 */

#include <Arduino.h>
#include <Keyboard.h>

// UART to ESP32 (GP0=TX, GP1=RX)
#define ESP_SERIAL Serial1

// ---- Protocol (binary frame) ----
#define PROTO_SYNC0 0xA5
#define PROTO_SYNC1 0x5A
#define PROTO_VER   1
#define PROTO_MAX_PAYLOAD 32

#define MSG_KEY_EVENT 0x01
#define MSG_KEY_STATE 0x02
#define MSG_CONTROL   0x10

#define KEY_PRESS   1
#define KEY_RELEASE 0

#define BROADCAST_ID 0xFF

#define DEBUG_LOG 0

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void send_frame(uint8_t type, uint8_t sender_id, const uint8_t *payload, uint8_t len) {
    if (len > PROTO_MAX_PAYLOAD) {
        return;
    }
    static uint8_t seq = 0;
    uint8_t frame[7 + PROTO_MAX_PAYLOAD + 1];
    uint8_t idx = 0;
    frame[idx++] = PROTO_SYNC0;
    frame[idx++] = PROTO_SYNC1;
    frame[idx++] = PROTO_VER;
    frame[idx++] = type;
    frame[idx++] = sender_id;
    frame[idx++] = seq++;
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

    ESP_SERIAL.write(frame, idx);
}

static char index_to_key(uint8_t index) {
    return (char)('0' + index);
}

static void apply_key_state(uint16_t mask) {
    for (uint8_t i = 0; i < 8; i++) {
        char key = index_to_key(i);
        if (mask & (1U << i)) {
            Keyboard.press(key);
        } else {
            Keyboard.release(key);
        }
    }
}

static void handle_key_event(uint8_t keycode, uint8_t action) {
    char key = (char)keycode;
    char lower = (key >= 'A' && key <= 'Z') ? (char)(key + 32) : key;
    if (action == KEY_PRESS) {
        Keyboard.press(lower);
    } else {
        Keyboard.release(lower);
    }
}

static void parse_control_line(const char *line) {
    // Supported:
    // LED:1 / LED:0  -> broadcast
    // ID:<n> LED:1   -> targeted
    uint8_t target_id = BROADCAST_ID;
    const char *cmd = line;

    if (strncmp(line, "ID:", 3) == 0) {
        target_id = (uint8_t)atoi(line + 3);
        const char *space = strchr(line, ' ');
        if (!space) {
            return;
        }
        cmd = space + 1;
    }

    if (strncmp(cmd, "LED:", 4) == 0 && (cmd[4] == '0' || cmd[4] == '1')) {
        uint8_t payload[2];
        payload[0] = 1; // cmd: LED
        payload[1] = (uint8_t)(cmd[4] == '1');
        send_frame(MSG_CONTROL, target_id, payload, sizeof(payload));
    }
}

void setup() {
    Keyboard.begin();
    Serial.begin(115200);
    delay(1000);

    ESP_SERIAL.setRX(1);
    ESP_SERIAL.setTX(0);
    ESP_SERIAL.begin(921600);

#if DEBUG_LOG
    Serial.println("=== PICO USB BRIDGE ===");
#endif
}

void loop() {
    static uint8_t frame[7 + PROTO_MAX_PAYLOAD + 1];
    static uint8_t idx = 0;
    static uint8_t payload_len = 0;
    static enum { WAIT_SYNC0, WAIT_SYNC1, READ_HEADER, READ_PAYLOAD, READ_CRC } state = WAIT_SYNC0;

    static unsigned long lastDataTime = 0;
    static bool anyKeys = false;

    // Read frames from ESP
    while (ESP_SERIAL.available()) {
        uint8_t b = (uint8_t)ESP_SERIAL.read();
        lastDataTime = millis();

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
                    for (int i = 2; i < 7 + payload_len; i++) {
                        crc = crc8_update(crc, frame[i]);
                    }
                    if (crc == frame[7 + payload_len] && frame[2] == PROTO_VER) {
                        uint8_t type = frame[3];
                        if (type == MSG_KEY_EVENT && payload_len >= 2) {
                            handle_key_event(frame[7], frame[8]);
                            anyKeys = true;
                        } else if (type == MSG_KEY_STATE && payload_len >= 2) {
                            uint16_t mask = (uint16_t)frame[7] | ((uint16_t)frame[8] << 8);
                            apply_key_state(mask);
                            anyKeys = (mask != 0);
                        }
                    }
                    state = WAIT_SYNC0;
                }
                break;
        }
    }

    // Safety timeout: release all keys after 200ms of silence
    if (anyKeys && (millis() - lastDataTime > 200)) {
        Keyboard.releaseAll();
        anyKeys = false;
#if DEBUG_LOG
        Serial.println("[TIMEOUT] releaseAll");
#endif
    }

    // USB Serial commands -> control frames
    static char usb_buffer[64];
    static int usb_idx = 0;
    static unsigned long lastUsbCharTime = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        lastUsbCharTime = millis();

        if (c == '\n' || c == '\r' || usb_idx >= 63) {
            usb_buffer[usb_idx] = '\0';
            if (usb_idx > 0) {
                parse_control_line(usb_buffer);
            }
            usb_idx = 0;
        } else {
            usb_buffer[usb_idx++] = c;
        }
    }

    if (usb_idx > 0 && (millis() - lastUsbCharTime > 50)) {
        usb_buffer[usb_idx] = '\0';
        parse_control_line(usb_buffer);
        usb_idx = 0;
    }
}
