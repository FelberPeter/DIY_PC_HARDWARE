# ESP-NOW Wireless HID Framework

## Übersicht

Dieses Framework ermöglicht die Entwicklung von kabellosen USB-HID-Geräten (Tastatur, Maus, Controller, etc.) basierend auf ESP32-C3 und Raspberry Pi Pico.

```
┌─────────────────┐     ESP-NOW      ┌─────────────────┐      UART       ┌─────────────────┐      USB HID
│   ESP-Sender    │ ──────────────► │  ESP-Receiver   │ ──────────────► │   Pico Bridge   │ ──────────────► PC
│  (Eingabegerät) │ ◄────────────── │    (Dongle)     │ ◄────────────── │  (USB Adapter)  │ ◄──────────────
│                 │     ESP-NOW      │                 │      UART       │                 │    USB Serial
│  GPIO 0-7       │                  │  GPIO 20/21     │                 │  GP0/GP1        │
│  LED GPIO 8     │                  │                 │                 │                 │
└─────────────────┘                  └─────────────────┘                 └─────────────────┘
     COM11                                COM13                              COM15
```

## Hardware

### Benötigte Komponenten

| Komponente | Anzahl | Beschreibung |
|------------|--------|--------------|
| ESP32-C3 Super Mini | 2 | Sender + Receiver |
| Raspberry Pi Pico | 1 | USB HID Bridge |
| Taster | 1-8 | Für Tasteneingabe |
| Verbindungskabel | 3 | UART-Verbindung |

### Pin-Belegung

#### ESP-Sender (Eingabegerät)
| GPIO | Funktion | Beschreibung |
|------|----------|--------------|
| 0-7 | Taster | Aktiv LOW, interner Pull-Up |
| 8 | LED | Blaue Status-LED |

#### ESP-Receiver (Dongle)
| GPIO | Funktion | Beschreibung |
|------|----------|--------------|
| 20 | UART RX | Empfang vom Pico |
| 21 | UART TX | Senden zum Pico |

#### Raspberry Pi Pico
| GPIO | Funktion | Beschreibung |
|------|----------|--------------|
| GP0 | UART TX | Senden zum ESP |
| GP1 | UART RX | Empfang vom ESP |
| USB | HID + CDC | Tastatur + Serial |

### Verkabelung ESP-Receiver ↔ Pico

```
ESP-Receiver          Pico
    GPIO 21 (TX) ──────► GP1 (RX)
    GPIO 20 (RX) ◄────── GP0 (TX)
    GND          ◄─────► GND
```

## Software-Architektur

### Kommunikationsprotokolle

#### Tastatur-Nachrichten (Sender → PC)
```
Format: KEY:<taste>:<status>\n
Beispiele:
  KEY:0:1   → Taste '0' gedrückt
  KEY:0:0   → Taste '0' losgelassen
  KEY:a:1   → Taste 'a' gedrückt
```

#### LED-Befehle (PC → Sender)
```
Format: LED:<status>
Beispiele:
  LED:1     → LED einschalten
  LED:0     → LED ausschalten
```

### Datenfluss

#### Tastendruck → PC
1. **ESP-Sender**: GPIO-Interrupt erkennt Tastendruck
2. **ESP-Sender**: Sendet `KEY:x:1` via ESP-NOW
3. **ESP-Receiver**: Empfängt ESP-NOW, leitet an UART
4. **Pico**: Empfängt UART, ruft `Keyboard.press(key)` auf
5. **PC**: Erhält HID Keyboard Event

#### LED-Steuerung vom PC
1. **PC**: Sendet `LED:1` via USB Serial
2. **Pico**: Empfängt Serial, leitet an UART
3. **ESP-Receiver**: Empfängt UART, sendet via ESP-NOW
4. **ESP-Sender**: Empfängt ESP-NOW, schaltet GPIO8

## Projekt-Struktur

```
python_delet_me/
├── README.md                 # Diese Dokumentation
├── esp_sender/
│   ├── platformio.ini        # PlatformIO Konfiguration
│   └── src/
│       └── main.c            # ESP-IDF Firmware
├── esp_receiver/
│   ├── platformio.ini        # PlatformIO Konfiguration
│   └── src/
│       └── main.c            # ESP-IDF Firmware
└── pico_bridge/
    ├── platformio.ini        # PlatformIO Konfiguration
    └── src/
        └── main.cpp          # Arduino-Pico Firmware
```

## Build & Flash

### Voraussetzungen
- PlatformIO Core oder VS Code mit PlatformIO Extension
- USB-Treiber für ESP32-C3 und Pico

### ESP-Sender flashen
```powershell
cd esp_sender
pio run -t upload --upload-port COM11
```

### ESP-Receiver flashen
```powershell
cd esp_receiver
pio run -t upload --upload-port COM13
```

### Pico flashen
1. BOOTSEL-Taste gedrückt halten
2. USB-Kabel einstecken
3. Firmware kopieren:
```powershell
cd pico_bridge
pio run
Copy-Item ".pio\build\pico\firmware.uf2" -Destination "D:\"
```

## Konfiguration

### MAC-Adressen anpassen

Die MAC-Adressen müssen in beiden ESP-Firmwares eingetragen werden:

**esp_sender/src/main.c:**
```c
// MAC-Adresse des Empfängers
static uint8_t receiver_mac[6] = {0x20, 0x6e, 0xf1, 0x6a, 0xc4, 0xb0};
```

**esp_receiver/src/main.c:**
```c
// MAC-Adresse des Senders
static uint8_t sender_mac[6] = {0x20, 0x6e, 0xf1, 0x6a, 0xa3, 0xb8};
```

### MAC-Adresse herausfinden
Die MAC-Adresse wird beim Start im Serial Monitor ausgegeben:
```
Eigene MAC: 20:6e:f1:6a:a3:b8
```

### Tasten-Mapping anpassen

**esp_sender/src/main.c:**
```c
static char gpio_to_key(uint8_t gpio) {
    switch(gpio) {
        case 0: return '0';  // oder 'w' für WASD
        case 1: return '1';  // oder 'a'
        case 2: return '2';  // oder 's'
        case 3: return '3';  // oder 'd'
        // ... weitere Tasten
    }
}
```

### Sondertasten (Keyboard.h)

Für Sondertasten die Arduino Keyboard-Konstanten verwenden:

```cpp
// pico_bridge/src/main.cpp
#include <Keyboard.h>

// Beispiele:
Keyboard.press(KEY_LEFT_CTRL);
Keyboard.press(KEY_LEFT_SHIFT);
Keyboard.press(KEY_LEFT_ALT);
Keyboard.press(KEY_UP_ARROW);
Keyboard.press(KEY_RETURN);
Keyboard.press(KEY_ESC);
Keyboard.press(KEY_BACKSPACE);
Keyboard.press(KEY_TAB);
Keyboard.press(KEY_F1);  // F1-F12
```

## Features

### Implementiert ✅

- [x] **8 Tasten** (GPIO 0-7) mit Interrupt-basierter Erkennung
- [x] **Taste halten** - Repeat-Signal alle 50ms für Gaming
- [x] **Auto-Release** - Timeout nach 150ms falls Signal verloren
- [x] **Debouncing** - 20ms Software-Debounce
- [x] **Bidirektionale Kommunikation** - PC kann LED steuern
- [x] **Hohe Baudrate** - 921600 baud für minimale Latenz
- [x] **USB HID Keyboard** - Funktioniert mit allen Programmen

### Geplante Erweiterungen

- [ ] **USB HID Maus** - Mouse.h Library
- [ ] **USB HID Gamepad** - Joystick.h Library
- [ ] **WS2812 RGB LEDs** - Neopixel-Unterstützung
- [ ] **Akku-Betrieb** - Deep Sleep Modus
- [ ] **Mehrere Sender** - Multi-Device Support
- [ ] **Verschlüsselung** - ESP-NOW Encryption
- [ ] **OTA Updates** - Firmware Over-the-Air

## Erweiterung: USB Maus

### Pico-Code erweitern

```cpp
#include <Mouse.h>

void setup() {
    Mouse.begin();
    // ...
}

// Neue Nachrichtentypen:
// MOUSE:X:Y      → Relative Bewegung
// MOUSE:C:1      → Linksklick gedrückt
// MOUSE:C:0      → Linksklick losgelassen
// MOUSE:R:1      → Rechtsklick gedrückt
// MOUSE:W:3      → Scroll 3 Einheiten

void parseMouseMessage(char* msg) {
    if (strncmp(msg, "MOUSE:", 6) != 0) return;
    
    char type = msg[6];
    int value = atoi(&msg[8]);
    
    switch(type) {
        case 'X': Mouse.move(value, 0); break;
        case 'Y': Mouse.move(0, value); break;
        case 'C': value ? Mouse.press(MOUSE_LEFT) : Mouse.release(MOUSE_LEFT); break;
        case 'R': value ? Mouse.press(MOUSE_RIGHT) : Mouse.release(MOUSE_RIGHT); break;
        case 'W': Mouse.move(0, 0, value); break;
    }
}
```

### ESP-Sender: Joystick/Analog-Eingabe

```c
#include "driver/adc.h"

// ADC für Joystick
adc1_config_width(ADC_WIDTH_BIT_12);
adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);  // GPIO0

int x = adc1_get_raw(ADC1_CHANNEL_0);  // 0-4095
int mapped_x = (x - 2048) / 20;  // -100 bis +100

char msg[32];
snprintf(msg, sizeof(msg), "MOUSE:X:%d\n", mapped_x);
esp_now_send(receiver_mac, (uint8_t*)msg, strlen(msg));
```

## Erweiterung: USB Gamepad

### PlatformIO.ini anpassen

```ini
[env:pico]
lib_deps = 
    Keyboard
    Joystick  ; oder arduino-libraries/Joystick
```

### Gamepad-Code

```cpp
#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
    JOYSTICK_TYPE_GAMEPAD,
    8,    // Button Count
    0,    // Hat Switch Count
    true, // X Axis
    true, // Y Axis
    false, false, false, false, false, false, false, false, false);

void setup() {
    Joystick.begin();
}

// Nachrichtenformat:
// PAD:B:0:1    → Button 0 gedrückt
// PAD:X:512   → X-Achse auf 512 (0-1023)
// PAD:Y:256   → Y-Achse auf 256

void parsePadMessage(char* msg) {
    if (strncmp(msg, "PAD:", 4) != 0) return;
    
    char type = msg[4];
    int id = msg[6] - '0';
    int value = atoi(&msg[8]);
    
    switch(type) {
        case 'B': 
            Joystick.setButton(id, value); 
            break;
        case 'X': 
            Joystick.setXAxis(value); 
            break;
        case 'Y': 
            Joystick.setYAxis(value); 
            break;
    }
}
```

## Erweiterung: RGB LEDs (WS2812)

### ESP-Sender mit Neopixel

```c
#include "led_strip.h"

#define LED_STRIP_GPIO 8
#define LED_COUNT 10

led_strip_handle_t led_strip;

void rgb_init(void) {
    led_strip_config_t config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_new_rmt_device(&config, &rmt_config, &led_strip);
}

// Befehlsformat: RGB:0:255:128:64  → LED 0 auf R=255, G=128, B=64
void on_data_recv(...) {
    if (strncmp(data, "RGB:", 4) == 0) {
        int led, r, g, b;
        sscanf(data, "RGB:%d:%d:%d:%d", &led, &r, &g, &b);
        led_strip_set_pixel(led_strip, led, r, g, b);
        led_strip_refresh(led_strip);
    }
}
```

## Troubleshooting

### Problem: Keine Tasteneingabe
1. Prüfe UART-Verbindung (TX↔RX, GND)
2. Prüfe Baudrate (921600)
3. Prüfe MAC-Adressen in beiden ESP-Firmwares
4. Serial Monitor auf ESP-Sender/Receiver öffnen

### Problem: Tasten bleiben hängen
- Timeout ist auf 150ms eingestellt
- Repeat-Signal wird alle 50ms gesendet
- Falls weiterhin Problem: Timeout erhöhen

### Problem: LED reagiert nicht
1. Prüfe GPIO 8 auf ESP-Sender
2. Teste mit Serial Monitor: `LED:1` senden
3. Prüfe ob ESP-Receiver Befehl empfängt

### Problem: Hercules sendet nicht korrekt
- Timeout-Fix ist implementiert (50ms)
- Einfach `LED:1` eingeben und Send klicken
- Kein `\n` am Ende nötig

## Latenz-Optimierungen

Das Framework ist für minimale Latenz optimiert:

| Komponente | Optimierung |
|------------|-------------|
| GPIO | Interrupt-basiert, kein Polling |
| Debounce | 20ms, in ISR-Kontext |
| ESP-NOW | Direktverbindung, kein WiFi-AP |
| UART | 921600 Baud |
| Pico | Kein delay() im Loop |
| USB HID | Native USB, kein CDC-Only |

Geschätzte Gesamtlatenz: **< 5ms**

## Lizenz

MIT License - Frei verwendbar für private und kommerzielle Projekte.

## Autor

Erstellt mit GitHub Copilot, Februar 2026.
