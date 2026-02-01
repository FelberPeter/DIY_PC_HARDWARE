/*
 * Raspberry Pi Pico - USB HID Keyboard + Bidirektionale Bridge
 * Empfängt Tasten via UART vom ESP und sendet als USB Keyboard
 * Empfängt Befehle via USB Serial und leitet an ESP weiter (z.B. LED:1)
 */

#include <Arduino.h>
#include <Keyboard.h>

// UART zum ESP32 (GP0=TX, GP1=RX)
#define ESP_SERIAL Serial1

void setup() {
    // USB Keyboard initialisieren
    Keyboard.begin();
    
    // Debug Serial (USB CDC) - auch für Befehle vom PC
    Serial.begin(115200);
    delay(2000);  // Warte auf USB
    
    // UART zum ESP32 - Hohe Baudrate für minimale Latenz
    ESP_SERIAL.setRX(1);  // GP1
    ESP_SERIAL.setTX(0);  // GP0  
    ESP_SERIAL.begin(921600);
    
    Serial.println("=== PICO USB KEYBOARD - BIDIREKTIONAL ===");
    Serial.println("Tasten vom ESP -> USB HID Keyboard");
    Serial.println("Befehle an ESP: LED:1 / LED:0");
}

void parseKeyMessage(char* msg) {
    // Format: "KEY:a:1" oder "KEY:a:0"
    if (strncmp(msg, "KEY:", 4) != 0) return;
    if (strlen(msg) < 7) return;  // Mindestlaenge pruefen
    
    char key = msg[4];
    bool pressed = (msg[6] == '1');
    
    Serial.print("Taste: ");
    Serial.print(key);
    Serial.println(pressed ? " PRESS" : " RELEASE");
    
    // Keyboard.press/release erwarten lowercase
    char lowerKey = (key >= 'A' && key <= 'Z') ? (key + 32) : key;
    
    if (pressed) {
        Keyboard.press(lowerKey);
    } else {
        Keyboard.release(lowerKey);
    }
}

void loop() {
    static char uart_buffer[64];
    static int uart_idx = 0;
    static unsigned long lastDataTime = 0;
    static bool keysPressed = false;
    
    // UART vom ESP lesen
    while (ESP_SERIAL.available()) {
        char c = ESP_SERIAL.read();
        lastDataTime = millis();
        
        if (c == '\n' || c == '\r' || uart_idx >= 63) {
            uart_buffer[uart_idx] = '\0';
            
            if (uart_idx > 0) {
                Serial.print("[ESP->USB] ");
                Serial.println(uart_buffer);
                
                parseKeyMessage(uart_buffer);
                keysPressed = true;
            }
            uart_idx = 0;
        } else {
            uart_buffer[uart_idx++] = c;
        }
    }
    
    // Sicherheits-Timeout: Alle Tasten loslassen nach 150ms ohne Daten
    if (keysPressed && (millis() - lastDataTime > 150)) {
        Keyboard.releaseAll();
        keysPressed = false;
        Serial.println("[TIMEOUT] Alle Tasten losgelassen");
    }
    
    // USB Serial Befehle vom PC empfangen und an ESP weiterleiten
    static char usb_buffer[64];
    static int usb_idx = 0;
    static unsigned long lastUsbCharTime = 0;
    
    while (Serial.available()) {
        char c = Serial.read();
        lastUsbCharTime = millis();
        
        if (c == '\n' || c == '\r' || usb_idx >= 63) {
            usb_buffer[usb_idx] = '\0';
            
            if (usb_idx > 0) {
                Serial.print("[PC->ESP] ");
                Serial.println(usb_buffer);
                
                // An ESP weiterleiten
                ESP_SERIAL.print(usb_buffer);
                ESP_SERIAL.print('\n');
            }
            usb_idx = 0;
        } else {
            usb_buffer[usb_idx++] = c;
        }
    }
    
    // Timeout: Wenn 50ms keine neuen Zeichen, Buffer senden (für Hercules ohne Newline)
    if (usb_idx > 0 && (millis() - lastUsbCharTime > 50)) {
        usb_buffer[usb_idx] = '\0';
        Serial.print("[PC->ESP] ");
        Serial.println(usb_buffer);
        
        ESP_SERIAL.print(usb_buffer);
        ESP_SERIAL.print('\n');
        usb_idx = 0;
    }
}
