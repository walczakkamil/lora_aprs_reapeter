# LoRa APRS Repeater (Dual SX1276)

## 📡 Project Overview

This project implements a **LoRa APRS repeater** based on an **STM32 Blue Pill (STM32F103C8T6)** microcontroller and **two SX1276 / RFM96W LoRa radio modules**.

The device operates as a **simple digital repeater**:
- **Radio 1 (RX)** receives APRS LoRa packets
- **Radio 2 (TX-FWD)** forwards the received packet unchanged on another frequency

The project is designed for **low power consumption**, **high reliability**, and **simple firmware architecture** without an RTOS.

---

## 🧱 Hardware Used

### Microcontroller Board
- **STM32 Blue Pill**
- MCU: **STM32F103C8T6**
- Core: ARM Cortex-M3 @ 72 MHz
- Flash: 64 KB
- RAM: 20 KB
- Operating voltage: 3.3 V

### Radio Modules
- **2 × SX1276 / RFM96W (LoRa)**
- Frequency band: **433 MHz**
- Interface: **SPI**
- Modulation: **LoRa**
- Used for **APRS over LoRa**

---

## 📻 Radio Configuration

Both radios use identical LoRa parameters, except for frequency:

| Parameter            | Value              |
|----------------------|--------------------|
| Modulation           | LoRa               |
| Bandwidth            | 125 kHz            |
| Spreading Factor     | SF9                |
| Coding Rate          | 4/7                |
| Sync Word            | 0x12 (private)     |
| CRC                  | Enabled            |
| Preamble Length      | 8 symbols          |
| Mode                 | Continuous RX / TX |

### Frequencies
- **Radio 1 (RX):** 434.855 MHz  
- **Radio 2 (TX-FWD):** 434.955 MHz  

---

## 🔁 Operating Principle

1. Radio 1 stays in **continuous receive mode**
2. When a valid LoRa packet is received:
   - Payload is read from FIFO
   - Packet metadata (RSSI, SNR) is logged via UART
3. The same payload is immediately transmitted by Radio 2
4. After transmission:
   - Radio 2 returns to sleep
   - Radio 1 continues listening

> Radio 2 does **not** receive packets – this reduces power consumption and prevents feedback loops.

---

## 🔌 Pin Connections

### SPI (shared by both radios)

| STM32 Pin | Function |
|----------|----------|
| PA5      | SPI1_SCK |
| PA6      | SPI1_MISO|
| PA7      | SPI1_MOSI|

---

### Radio 1 (RX)

| SX1276 Pin | STM32 Pin | Description |
|-----------|-----------|-------------|
| NSS       | PA4       | SPI CS      |
| RESET     | PB0       | Reset       |
| DIO0      | PB1       | RX Done IRQ |
| VCC       | 3.3V      | Power       |
| GND       | GND       | Ground      |

---

### Radio 2 (TX-FWD)

| SX1276 Pin | STM32 Pin | Description |
|-----------|-----------|-------------|
| NSS       | PA3       | SPI CS      |
| RESET     | PB10      | Reset       |
| DIO0      | *unused*  | Not needed  |
| VCC       | 3.3V      | Power       |
| GND       | GND       | Ground      |

> `DIO0` is intentionally **not connected** for Radio 2, as only transmission is required.

---

## 🧠 Firmware Architecture

- Bare-metal firmware using **STM32 HAL**
- No RTOS
- Single main loop
- Interrupt-driven RX (DIO0 on Radio 1)
- Blocking TX on Radio 2
- UART used for debugging and packet dump

---

## 🧾 UART Debug Output

- Packet length
- RSSI (dBm)
- SNR (dB)
- Payload dump:
  - HEX
  - ASCII (printable characters)

Example:
```
R1 RX DONE: len=56 RSSI=-28 dBm SNR=11.5 dB
HEX: 3C FF 01 02 ...
ASCII: <..APRS....>
```

---

## 🔒 Reliability & Safety

- Hardware reset of both radios at startup
- Watchdog-friendly structure
- Optional **panic reset** using `NVIC_SystemReset()` in case of unrecoverable errors
- No dynamic memory allocation

---

## ⚡ Power Considerations

- Radio 2 stays in **sleep mode** when idle
- No reception on TX radio
- Suitable for battery or solar-powered installations

---

## 🚀 Future Improvements

- APRS payload parsing (position, telemetry)
- Smart digipeater logic
- Packet filtering
- EEPROM / Flash configuration storage
- CAD-based reception
- FreeRTOS support (optional)

---

## 📜 License

This project is provided for **educational and amateur radio use**.  
Use it responsibly and according to your local radio regulations.

---

## 👤 Author 

SP7FM @ Kamil

LoRa APRS Repeater  
Built with ❤️ for amateur radio experimentation
