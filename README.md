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
- Debug mode
  - A9 RX - TX UART
  - A10 TX - RX UART
  - GNG - GND UART
  - B12 - GND - debug enabler

Example:
```
R1 RX DONE: len=56 RSSI=-28 dBm SNR=11.5 dB
R1 HEX: 3C FF 01 02 ...
R1 ASCII: <..APRS....>
R2 TX: forwarding 40 bytes
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

# 📡 LoRa APRS Repeater – Radio State Flow

This document describes the **runtime flow and radio state management** used in the LoRa APRS Repeater project based on **STM32 (BluePill) + SX127x**.

The design uses **two LoRa radios**:

- 📥 **RX1 (RID_RX1)** – continuous receiver (LoRa APRS RX)
- 📤 **TX / RX2 (RID_RX2)** – transmitter only (forwarding + telemetry)

The main goals are:
- 🔄 continuous reception on RX1,
- 🔋 very low power consumption on TX/RX2,
- 📊 periodic APRS telemetry transmission (VDD) every 1 hour,
- 🚀 immediate forwarding of received APRS frames.

---

## ⚙️ Radio Modes Overview

| Mode | Description | Typical Current | Notes |
|---|---|---|---|
| 💤 `MODE_SLEEP` | Deep sleep | ~µA | Oscillator off, registers lost |
| ⏸️ `MODE_STDBY` | Standby / ready | ~mA | Fast TX/RX start, registers kept |
| 📡 `MODE_RX_CONTINUOUS` | Continuous receive | ~10–12 mA | RX active |
| 📶 `MODE_TX` | Transmit | up to 120 mA | Depends on power level |

---

## 🔁 Runtime Flow – State Table

### 🗂️ Legend
- **RX1** = RID_RX1 (receiver radio)
- **TX/RX2** = RID_RX2 (transmit-only radio)

| Step | Trigger | RX1 (RID_RX1) | TX / RX2 (RID_RX2) | Code Activity | Purpose |
|---|---|---|---|---|---|
| 0️⃣ | MCU reset | Not configured | Not configured | HAL init, UART, SPI, ADC init, SX127x reset, REG_VERSION check | 🟢 System startup |
| 1️⃣ | LoRa configuration | Configured → 📡 RX_CONTINUOUS | Configured → 💤 SLEEP | `RADIO_RX_LoRaInit()` + `RADIO_RX_StartContinuous()` | RX1 starts listening, TX sleeps |
| 2️⃣ | Startup telemetry | 📡 RX_CONTINUOUS | 💤 → ⏸️ → 📶 → 💤 | `TELEMETRY_SendVddOnce()` | 📊 Immediate VDD telemetry after boot |
| 3️⃣ | Normal operation | 📡 RX_CONTINUOUS | 💤 SLEEP | Main loop polling RX1, checking telemetry timer | 🔋 Idle / low power |
| 4️⃣ | Packet received | 📡 RX IRQ | 💤 SLEEP | RX1 FIFO read, packet buffered, `g_tx_pending_2 = 1` | 📥 Prepare forwarding |
| 5️⃣ | Packet forward | TX pending flag | 📡 RX_CONTINUOUS | 💤 → ⏸️ → 📶 → 💤 | `RADIO_TX_Send()` forwards APRS frame | 🚀 Forward packet |
| 6️⃣ | Periodic telemetry | ⏱️ 1h timer | 📡 RX_CONTINUOUS | 💤 → ⏸️ → 📶 → 💤 | APRS telemetry `T#...` frame | 📈 VDD history |
| 7️⃣ | Fault recovery | Error threshold | n/a | n/a | Panic reset (`NVIC_SystemReset`) | 🔁 Self-recovery |

---

## ✅ Why `MODE_SLEEP` After TX Is Correct Here

Using 💤 `MODE_SLEEP` at the end of `RADIO_TX_Send()` is **intentional and correct** because:

- 📤 TX/RX2 **does not perform reception**
- ⏳ The next TX happens **minutes or hours later**
- 🔋 Standby current would unnecessarily drain a solar-powered node
- 🔄 Radio configuration is fully re-initialized before each TX

This makes 💤 `MODE_SLEEP` the **most power-efficient choice** for this architecture.

---

## ⚠️ Important Rule

> After exiting 💤 `MODE_SLEEP`, **LoRa configuration must be fully re-applied**.

This project already follows this rule.

---

## 🧾 Summary

- 📥 RX1 stays in `MODE_RX_CONTINUOUS`
- 💤 TX/RX2 sleeps almost all the time
- 📶 TX wakes up only to forward packets or send telemetry
- 📊 Telemetry is sent:
  - immediately after boot
  - then every **1 hour**
- 🔋 Power consumption is minimized without losing functionality

---

This flow is optimized for **solar-powered LoRa APRS infrastructure nodes** such as digipeaters or repeaters.

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


## 👤 Author 

SP7FM @ Kamil

LoRa APRS Repeater  
Built with ❤️ for amateur radio experimentation
