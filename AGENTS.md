# Repository Guidelines

## Project Structure & Module Organization
- `esp_sender/` contains the ESP32-C3 sender firmware (ESP-IDF, `src/main.c`) and its `platformio.ini`.
- `esp_receiver/` contains the ESP32-C3 receiver/dongle firmware (ESP-IDF, `src/main.c`) and its `platformio.ini`.
- `pico_bridge/` contains the Raspberry Pi Pico USB HID bridge firmware (Arduino core, `src/main.cpp`).
- `test_*.py` scripts are host-side serial tests (roundtrip/port checks).
- `*.bat` and `*.txt` files are helper notes or quick actions for flashing/board info.

## Build, Test, and Development Commands
- `pio run -t upload` in each firmware folder builds and flashes that target (set `upload_port` in `platformio.ini`).
- `pio run` in `pico_bridge/` builds a UF2; copy `.pio\build\pico\firmware.uf2` to the Pico drive.
- `pio device monitor -p COMxx -b 115200` opens a serial monitor for ESP/Pico.
- `python test_pico_serial.py` validates Pico serial connectivity on the configured COM port.
- `python test_roundtrip.py` performs the end-to-end latency test (PC ? Pico ? ESP ? back).

## Coding Style & Naming Conventions
- ESP-IDF sources are C (`main.c`), Pico bridge is C++ (`main.cpp`). Keep file names lowercase with underscores.
- Indentation is 4 spaces for C/C++ and Python. Keep constants uppercase (e.g., `BAUD_RATE`).
- Configuration values live in `platformio.ini` (ports, build flags, USB IDs). Avoid hard-coding these in code unless required.

## Testing Guidelines
- Tests are simple Python scripts using `pyserial` (no test framework). Name new tests `test_<topic>.py`.
- Keep tests hardware-aware and document required COM ports at the top of the file.
- Run tests from the repo root so relative paths and ports are consistent.

## Commit & Pull Request Guidelines
- No Git history is present in this folder. If you add Git, prefer clear, imperative commit messages (e.g., `Add pico HID debounce`).
- PRs should include: a short summary, hardware used, COM ports, and proof of flashing/testing (serial logs or photos).

## Configuration & Safety Notes
- MAC addresses are defined in `esp_sender/src/main.c` and `esp_receiver/src/main.c`; keep them in sync.
- Verify `upload_port` and `monitor_port` in each `platformio.ini` before flashing to avoid the wrong device.
