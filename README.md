# 25HS_VenusProbe
Firmware workspace for the ETH Zürich Space Systems Engineering core course (Venus Probe Project).

> **Educational Use Only** — This repository is course material belonging to the ETH Zürich Space Systems Engineering program. Redistribution outside the class, commercial reuse, or publication without written consent from the teaching team is prohibited.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Hardware Platform](#hardware-platform)
3. [Repository Layout](#repository-layout)
4. [Software Stack & Libraries](#software-stack--libraries)
5. [Configuration (`lib/include/config.h`)](#configuration-libincludeconfigh)
6. [Build, Flash & Verify](#build-flash--verify)
7. [Subsystem Notes](#subsystem-notes)
8. [Data Handling & Logging](#data-handling--logging)
9. [Testing & Troubleshooting](#testing--troubleshooting)
10. [Course Policies & Attribution](#course-policies--attribution)

## Project Overview
- **Objective:** Provide a reusable firmware baseline for Venus Probe payload avionics used in the Venus Probe project of Space Systems Engineering Core Course 1.
- **Architecture:** Single MCU (Teensy 4.1) orchestrating instrument drivers (GNSS, IMU, pressure, microphone, LoRa radio, SD logging).
- **Framework:** PlatformIO + Arduino core for Teensy. Code is modularized into `lib/<subsystem>` directories with explicit namespaces (e.g., `gnss::read`).
- **Design Goals:** Deterministic sensor polling, robust data logging, and clear separation between hardware abstraction and mission logic.

## Hardware Platform
| Component | Role | Connection Notes |
|-----------|------|------------------|
| Teensy 4.1 | Main MCU (600 MHz ARM Cortex-M7) | Programmed over USB; provides SDIO, SPI, I²C, UART. |
| RFM95W (LoRa) | Telemetry link (868 MHz) | SPI bus; CS=`RFM95_CS`, RST=`RFM95_RST`, DIO0=`RFM95_INT`. |
| u-blox GNSS (UART) | Position & timing | Uses `Serial2` at `GNSS_BAUD_RATE`. |
| Adafruit ICM‑20948 | IMU (Accel/Gyro/Mag) | I²C (`Wire`) with address `IMU_I2C_ADDRESS`. |
| BMP/BME pressure sensor | Altitude reference | I²C (see `lib/bmp`). |
| ICS-43434 / custom microphone | Acoustic sensing | Captured via Teensy Audio library. |
| microSD (Teensy SDIO) | Primary storage | Accessed through `BUILTIN_SDCARD`. |

## Repository Layout
| Path | Description |
|------|-------------|
| `platformio.ini` | PlatformIO environment targeting Teensy 4.1. Adjust build flags or upload port here. |
| `src/main.cpp` | High-level mission loop (initializes subsystems, orchestrates actions). |
| `lib/include/config.h` | Shared configuration constants (pins, radio parameters, baud rates). |
| `lib/<subsystem>` | Self-contained drivers (`bmp`, `gps`, `imu`, `lora`, `mic`, `sd`). Each exposes a namespace-level API. |
| `include/` | (Optional) public headers for `src` usage. |
| `test/` | Placeholder for PlatformIO unit tests. |

## Software Stack & Libraries
| Layer | Library | Purpose |
|-------|---------|---------|
| Core | `framework-arduinoteensy` | Teensyduino runtime (Serial, Wire, SPI, Audio). |
| GNSS | `TinyGPS++` | NMEA parsing and fix extraction. |
| IMU | `Adafruit_ICM20X` / `Adafruit_ICM20948` | Sensor fusion driver for ICM‑20948. |
| LoRa | `sandeepmistry/LoRa` | SPI-based radio driver for Semtech SX1276 (RFM95) modules. |
| Audio/Mic | `Teensy Audio` | Streams microphone blocks into a circular buffer. |
| Storage | `SD.h` | SDIO filesystem for logging. |

Keep `platformio.ini` in sync with library versions if you make changes. Run `pio pkg update` when switching machines.

## Configuration (`lib/include/config.h`)
- **GNSS:** `GNSS_BAUD_RATE` (default 9600). Serial port is fixed to `Serial2` on Teensy.
- **IMU:** `IMU_I2C_ADDRESS` plus any scaling factors you add.
- **LoRa:** `RFM95_CS`, `RFM95_INT`, `RFM95_RST`, `LORA_FREQ`, `LORA_TX_POWER`, `LORA_SPREADING_FACTOR`, `LORA_CODING_RATE`, `LORA_BANDWIDTH`. Adjust to comply with local spectrum regulations.
- **Optional definitions:** You may add `SD_CS_PIN`, buffer sizes, or logging paths. Keep all shared constants here so subsystems remain decoupled.

## Build, Flash & Verify
### Prerequisites
- VS Code + PlatformIO extension.
- Teensy 4.1 drivers / Teensy Loader (installed automatically via PlatformIO).

### Typical Workflow
```sh
# 1. Configure platformio.ini if needed
# 2. Build
pio run

# 3. Upload (auto-detects Teensy in bootloader mode)
pio run -t upload

# 4. Monitor serial output (adjust port)
pio device monitor --baud 115200
```
- Use `pio run -t clean` after large refactors to ensure a fresh build.
- For multiple environments (e.g., RadioHead vs raw LoRa), define profiles in `platformio.ini` and build with `pio run -e <env>`.

## Subsystem Notes
### LoRa (`lib/lora`)
- `lora::setup()` configures pins, frequency, bandwidth, spreading factor, coding rate, and TX power through the Arduino LoRa driver.
- Packets are transmitted without RadioHead software headers; the full payload budget (≤255 bytes) is available for mission data.
- `lora::send` currently performs immediate retries via `yield()` (non-blocking). Higher layers may implement their own retry/backoff strategy if acknowledgements are required.
- Because no built-in addressing exists, include destination/source bytes inside your payload if you need routing logic.

### GNSS (`lib/gps`)
- Uses `TinyGPS++` to parse Serial2 stream.
- `gnss::Location` struct wraps lat/lon/alt/speed/course plus validity flag.
- Call `gnss::setup()` once, then poll `gnss::read()` regularly; it drains the UART each loop.

### IMU (`lib/imu`)
- Based on Adafruit ICM‑20948 driver via I²C.
- `imu::setup()` initializes sensor, enables necessary banks, and configures sample rates.
- Provide a `cleanup_imu()` if you need to release power or reinitialize.

### Microphone (`lib/mic`)
- Custom `AudioStream` subclass (`MicStream`) pushes blocks into a circular buffer protected by interrupt-safe cursors.
- `mic::setup(bufSize)` allocates the buffer, configures Audio library memory, and arms DMA.
- Use `mic::readBuffer()` to copy samples, then call `mic::clearBufferFlag()` when processed.

### SD Logging (`lib/sd`)
- `sd::setup()` wraps `SD.begin(BUILTIN_SDCARD)`.
- `sd::mkdirs()` creates nested directories without `String` allocations (512-byte path cap).
- `sd::open(path, append)` truncates by removing existing files when `append=false`.
- Remember to call `sd::flush()` or `sd::close()` to guarantee data hits the card.

### BMP / Pressure (`lib/bmp`)
- Wraps pressure/temperature sensor for altitude estimation. Calibrate offsets before flights.

## Data Handling & Logging
- **Telemetry:** Use the LoRa module (Arduino LoRa driver). Keep payloads ≤255 bytes and embed any addressing/checksum bytes you require at the application layer.
- **Onboard Storage:** Stream CSV/JSON lines via `sd::writeLine`. Suggested directory structure: `/logs/<date>/sensor_x.txt`.
- **Timestamps:** Teensy `millis()` is used by default. If GPS lock is available, propagate GPS time into log headers.

## Testing & Troubleshooting
- **Unit tests:** Place doctest/Unity tests in `test/` and run `pio test`.
- **Serial diagnostics:** Each subsystem prints status via `Serial`. Ensure `Serial.begin()` and adequate delays exist before first log.
- **Common issues:**
	- *LoRa not responding:* verify SPI wiring and `LoRa.setPins`. Ensure antenna connected before TX.
	- *GNSS silence:* confirm `Serial2` pins (Teensy pins 7/8) wiring and 3.3 V power.
	- *SD init failure:* card must be FAT32; avoid exFAT. Remove and reseat card, check `sd::setup()` return codes.
	- *Audio buffer overruns:* increase `MicStream` buffer size or reduce processing latency.

## Course Policies & Attribution
- Code, schematics, and documentation are for **Space Systems Engineering Core Course 1** participants only.
- When extending or publishing derived work, cite the course and obtain approval from the teaching assistants.
- Include this README (or an adapted version) with any fork distributed within the class to maintain provenance and compliance.

For questions, contact the Space Systems teaching team via the course forum or official ETH email channels.
