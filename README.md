# ESP-NOW Wireless HID Framework

## Overview
This project builds a low-latency, wireless USB HID bridge using:
- ESP32-C3 (sender input device)
- ESP32-C3 (receiver/dongle)
- Raspberry Pi Pico (USB HID bridge)

Data path:
ESP Sender -> ESP-NOW -> ESP Receiver -> UART -> Pico -> USB HID

## Hardware
- 2x ESP32-C3 Super Mini (sender + receiver)
- 1x Raspberry Pi Pico (USB HID bridge)
- Buttons on GPIO0..GPIO7 (active LOW)
- LED on GPIO8 (sender)

UART wiring (ESP receiver <-> Pico):
- ESP GPIO21 (TX) -> Pico GP1 (RX)
- ESP GPIO20 (RX) -> Pico GP0 (TX)
- GND -> GND

## Protocol v1 (Binary, Low-Latency)
All ESP-NOW + UART traffic uses a compact binary frame:

```
SYNC0 SYNC1 VER TYPE SENDER SEQ LEN PAYLOAD... CRC8
 0xA5  0x5A  1   T   ID     N   L   L bytes  CRC
```

Types:
- 0x01 KEY_EVENT: payload = [keycode, action] (action: 1=press, 0=release)
- 0x02 KEY_STATE: payload = [bitmask_lo, bitmask_hi]
- 0x10 CONTROL: payload = [cmd, value] (cmd 1 = LED)

Sender ID is per device. Broadcast uses ID 0xFF.

## Build & Flash
PlatformIO is required. If `pio` is not in PATH, use:

```powershell
python -m platformio run
```

Note (Windows): PlatformIO fails with spaces in project paths. Use a SUBST drive:
```powershell
cmd /c 'subst S: "C:\Onedrive\OneDrive - FH JOANNEUM\ChatGPT_CLI\diy_pc_hardware"'
```
Then run builds from `S:\`.

Flash ESP sender:
```powershell
cd esp_sender
python -m platformio run -t upload --upload-port COM11
```

Flash ESP receiver:
```powershell
cd esp_receiver
python -m platformio run -t upload --upload-port COM13
```

Pico UF2:
```powershell
cd pico_bridge
python -m platformio run
Copy-Item ".pio\build\pico\firmware.uf2" -Destination "D:\"
```

## Configuration
- Sender MAC + SENDER_ID in `esp_sender/src/main.c`
- Receiver peer list in `esp_receiver/src/main.c`
- Key mapping in `esp_sender/src/main.c` (`gpio_to_key`)

## Dev GUI
Run:
```powershell
python dev_gui.py
```

Features:
- Flash sender/receiver
- Build Pico and copy UF2
- Pico serial control (LED:1 / LED:0)

## Dev CLI
Examples:
```powershell
python dev_tool.py ports
python dev_tool.py flash-sender --port COM11
python dev_tool.py flash-receiver --port COM13
python dev_tool.py build-pico --copy
python dev_tool.py send --port COM15 "LED:1"
python dev_tool.py monitor --port COM15
```

Config file (shared by GUI + CLI): `dev_config.json`

## Tests (Manual)
- LED: send `LED:1` and `LED:0` via Pico USB serial
- Keyboard: press GPIO0..GPIO7 and verify USB HID events

## Roadmap
- Mouse HID
- Gamepad HID
- Multi-sender scaling
- Encryption/pairing
