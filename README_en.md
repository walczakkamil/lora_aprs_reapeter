# STM32 LoRa APRS Repeater (Bluepill + 2x RFM96W)

A simple, energy-efficient **LoRa APRS Repeater** based on the STM32F103C8T6 microcontroller (Bluepill) and two RFM96W radio modules (SX1278). The device operates in "Cross-Band" mode (receives on one frequency, transmits on another) and features telemetry and watchdog functions.

## 🚀 Features and Capabilities

* **Dual Radio:** Independent modules for RX (receiving) and TX (transmitting).
* **Continuous Listening (RX Continuous):** You won't miss any frames.
* **Buffering (Queue):** FIFO queue for 5 packets – prevents data loss when multiple frames arrive simultaneously.
* **Transparency:** Forwards raw LoRa frames (including `3C FF 01` headers), making it compatible with most trackers and iGates.
* **Telemetry:** Automatic status transmission every 1h (MCU supply voltage + station coordinates).
* **Energy Saving:** Processor enters `SLEEP` mode (WFI) when idle (wakes up on radio interrupt).
* **Watchdog (IWDG):** Automatic reset in case of system hang.
* **Debug Mode:** Live view of received and transmitted frames via UART when the service jumper is shorted.

## ⚙️ Radio Parameters (LoRa)

Settings are identical for RX and TX (except for frequency):

| Parameter | Value |
| :--- | :--- |
| **RX Frequency** | `434.855 MHz` |
| **TX Frequency** | `434.955 MHz` |
| **Spreading Factor (SF)** | `9` |
| **Bandwidth (BW)** | `125 kHz` |
| **Coding Rate (CR)** | `4/7` |
| **Tx Power** | `Max (0xFF)` |

## 🔌 Wiring Diagram (Pinout)

The device uses the **SPI1** bus shared by both radio modules.

### Module 1: Receiver (RX - 434.855 MHz)
| RFM96W Pin | STM32 Pin (Bluepill) | Notes |
| :--- | :--- | :--- |
| MISO | **PA6** | Shared SPI |
| MOSI | **PA7** | Shared SPI |
| SCK | **PA5** | Shared SPI |
| NSS (CS) | **PA4** | Chip Select |
| RST | **PB0** | Reset |
| DIO0 | **PB1** | Interrupt (EXTI) |
| 3.3V | 3.3V | |
| GND | GND | |

### Module 2: Transmitter (TX - 434.955 MHz)
| RFM96W Pin | STM32 Pin (Bluepill) | Notes |
| :--- | :--- | :--- |
| MISO | **PA6** | Shared SPI |
| MOSI | **PA7** | Shared SPI |
| SCK | **PA5** | Shared SPI |
| NSS (CS) | **PA3** | Chip Select |
| RST | **PB10** | Reset |
| DIO0 | *(NC)* | Not Connected (TX Blocking) |
| 3.3V | 3.3V | |
| GND | GND | |

### Other
| Function | STM32 Pin | Description |
| :--- | :--- | :--- |
| **DEBUG UART TX** | **PA9** | Logs (Baud: 115200) |
| **DEBUG SWITCH** | **PB12** | Short to GND enables logs |
| **LED STATUS** | **PC13** | Blinks on transmission (Built-in) |

## 📡 Telemetry

The repeater identifies itself with the callsign: `SP7FM-1`.
Telemetry frame format (sent every 1 hour):
```text
!5144.22N/01934.44E#SP7FM-1 BAT:x.xxV
```

* **Coordinates:** 51.737N, 19.574E (encoded in NMEA format).

* **Voltage:** Internal reference voltage (VREFINT) reading converted to supply voltage (VDDA).

## 🛠️ Debugging

To view device operation:

* Connect a USB-UART converter to pins PA9 (RX converter) and GND.

* Short pin PB12 to ground (GND).

* Open a terminal (e.g., PuTTY, RealTerm) with a baud rate of 115200 bps.

Example logs:
```text
SYS: Booting...
SYS: RX Init OK
SYS: TX Init OK
RX: Recv 89 bytes
RX CONTENT (TXT): <▒SP7FM-10>APLRG1...
QUEUE: Added packet (89 B)
TX: Preparing to send...
APRS CONTENT: <▒SP7FM-10>APLRG1...
TX: Done.
```
If pin PB12 is open (High state - PullUp), the repeater operates "silently" on UART, saving processor time.

## ⚠️ Important Notes

* **Antennas:** Never power up the TX module without an antenna connected! This risks damaging the RFM96 chip.
* **Power Supply:** Ensure the 3.3V source has sufficient current capability (LoRa transmission can draw >100mA).
* **Separation:** Due to the close frequencies (100kHz spacing), physical separation of RX and TX antennas is recommended to prevent the transmitter from desensitizing the receiver, or use bandpass filters/duplexer.


## 📝 Compilation
Project prepared for STM32CubeIDE / STM32CubeMX.

* **MCU:** STM32F103C8Tx
* **Libraries:** HAL Driver
* **Language:** C (C99/GNU11)

---
Project created for the LoRa APRS network by SP7FM.
