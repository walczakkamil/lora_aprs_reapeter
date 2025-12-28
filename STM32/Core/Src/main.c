/* --- APRS TELEMETRY DEFINITIONS --- */
/* Channel 1: VDD */
#define APRS_TLM_PARM  "PARM.VDD,,,,,"
#define APRS_TLM_UNIT  "UNIT.V,,,,,"
#define APRS_TLM_EQNS  "EQNS.0,0.02,0,0,0,0,0,0,0,0,0,0,0,0,0"
/* T#sss,aaa,bbb,ccc,ddd,eee,xxxxxxxx */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* =============================================================================
 * Logging / debug options
 * =============================================================================
 * You can control UART verbosity without touching the main radio logic.
 *
 * Compile-time:
 *   - Define DEBUG (e.g. -DDEBUG) to get a more verbose default.
 *   - Or override APP_LOG_LEVEL_DEFAULT explicitly.
 *
 * Levels:
 *   0 = OFF   (no UART logs; PANIC still prints)
 *   1 = ERROR
 *   2 = INFO
 *   3 = DEBUG
 *
 * Optional features:
 *   - APP_LOG_AUTO_MUTE_MS:
 *       If > 0, automatically disables UART logs after N ms from boot.
 *   - APP_LOG_SERVICE_PIN_ENABLE:
 *       If 1, a "service pin" (button/jumper) forces logs ON at boot.
 *
 * Runtime:
 *   - UART_LogEnable(1/0) toggles the log gate.
 *   - LOG_SetLevel(...) changes verbosity.
 * =============================================================================
 */
#define APP_LOG_LEVEL_DEFAULT 0u

#ifndef APP_LOG_LEVEL_DEFAULT
  #ifdef DEBUG
    #define APP_LOG_LEVEL_DEFAULT 3u
  #else
    #define APP_LOG_LEVEL_DEFAULT 2u
  #endif
#endif

#ifndef APP_LOG_AUTO_MUTE_MS
  #define APP_LOG_AUTO_MUTE_MS 0u
#endif

#ifndef APP_LOG_SERVICE_PIN_ENABLE
  #define APP_LOG_SERVICE_PIN_ENABLE 0u
#endif

#if (APP_LOG_SERVICE_PIN_ENABLE)
  #ifndef APP_LOG_SERVICE_GPIO_Port
    #define APP_LOG_SERVICE_GPIO_Port GPIOB
  #endif
  #ifndef APP_LOG_SERVICE_Pin
    #define APP_LOG_SERVICE_Pin GPIO_PIN_12
  #endif
  #ifndef APP_LOG_SERVICE_PIN_ACTIVE
    #define APP_LOG_SERVICE_PIN_ACTIVE GPIO_PIN_RESET
  #endif
#endif


/**
 * @file main.c
 * @brief Dual SX127x (RFM9x) LoRa receive + forward firmware (STM32F103).
 *
 * Features:
 *  - Two SX127x radios on a shared SPI bus (separate NSS/RESET/DIO0).
 *  - RX1 runs in continuous RX and captures LoRa frames.
 *  - RX2 is used as a low-duty TX forwarder (stays in STDBY most of the time to save power).
 *  - UART logging is gated by a runtime flag (g_uart_log_enabled) to reduce power usage.
 *  - A panic path (system_panic_reset) always emits a short message and resets the MCU.
 *
 * Notes:
 *  - This file is intentionally self-contained (minimal init helpers included).
 *  - Pin mapping must match your CubeMX project / board wiring.
 */

/* ===== Global buffers to reduce stack usage (prevents hardfault-like hangs) ===== */
/// @brief Shared UART formatting buffer (global to reduce stack usage).
static char g_uart_buf[256];
/// @brief Shared APRS dump buffer (HEX/ASCII output formatting).
static char g_aprs_out[240];
/// @brief RX1 raw payload buffer (max 128 bytes in this demo).
static uint8_t g_rx_buf_1[128];
/// @brief RX2 raw payload buffer (not used when RX2 is disabled; kept for symmetry).
static uint8_t g_rx_buf_2[128];

/* TX queue for Radio2 forwarding */
/// @brief Queued payload to forward on RX2 (TX buffer).
static uint8_t g_tx_buf_2[128];
/// @brief Length of queued TX payload for RX2.
static volatile uint8_t g_tx_len_2 = 0;
/// @brief Set to 1 when there is a payload pending TX on RX2.
static volatile uint8_t g_tx_pending_2 = 0;

/* Telemetry TX payload (separate from forward queue) */
static uint8_t g_telem_buf[128];

/// @brief Simple watchdog counter for repeated CRC errors.
static uint8_t crc_storm = 0;

/* =========================================================
   TWO SX127x MODULES ON A SINGLE SPI1 BUS (Variant A)

   Shared SPI1:
     SCK  PA5
     MISO PA6
     MOSI PA7

   Radio RX1:
     NSS/CS  PA4
     RST     PB0
     DIO0    PB1  (EXTI1)

   Radio RX2:
     NSS/CS  PA3
     RST     PB10
     DIO0    PB11 (EXTI15_10)

   UART1:
     TX PA9
     RX PA10

   LED (CubeMX) e.g. PC13 (depends on your board/project).
   ========================================================= */


/* ===================== Handles ===================== */
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;

/* ===================== Radio IDs ===================== */
#define RID_RX1 1
#define RID_RX2 2

/* ===================== Telemetry / power monitor =====================
 * - Measure VDD (3.3V rail) via internal VrefInt using ADC1
 * - Send an APRS status/telemetry line once per hour from SP7FM-1
 * - First transmission happens immediately after boot
 * - Keep the TX radio (RID_RX2) in SLEEP between transmissions/forwards
 *   to reduce power consumption.
 */
#ifndef TELEM_CALLSIGN
#define TELEM_CALLSIGN "SP7FM-1"
#endif

#ifndef TELEM_DEST
#define TELEM_DEST     "APRS"
#endif

#ifndef TELEM_INTERVAL_MS
#define TELEM_INTERVAL_MS (3600000u) /* 1h */
#endif
/* Fixed position beacon (required so aprs.fi can place the station on the map).
 * Format uses uncompressed APRS position: !DDMM.mmN/DDDMM.mmE<sym><comment>
 *
 * IMPORTANT: Set APRS_LAT/APRS_LON to your actual coordinates.
 * Example:
 *   #define APRS_LAT "4916.45N"
 *   #define APRS_LON "01902.12E"
 */
#ifndef APRS_LAT
#define APRS_LAT "51.737N"
#endif
#ifndef APRS_LON
#define APRS_LON "19.574E"
#endif
#ifndef APRS_SYMBOL_TABLE
#define APRS_SYMBOL_TABLE '/'
#endif
#ifndef APRS_SYMBOL_CODE
#define APRS_SYMBOL_CODE '#'
#endif
#ifndef APRS_BEACON_COMMENT
#define APRS_BEACON_COMMENT "LoRa APRS DIGI (solar)"
#endif



/* VrefInt nominal voltage (mV). You can improve accuracy by calibrating this
 * once against a known-good VDD and adjusting the constant.
 */
#ifndef VREFINT_MV
#define VREFINT_MV 1200u
#endif

/* ===================== GPIO mapping (DOPASUJ JEŚLI INNE) ===================== */
#define RX1_NSS_GPIO_Port GPIOA
#define RX1_NSS_Pin       GPIO_PIN_4
#define RX2_NSS_GPIO_Port GPIOA
#define RX2_NSS_Pin       GPIO_PIN_3

#define RX1_RST_GPIO_Port GPIOB
#define RX1_RST_Pin       GPIO_PIN_0
#define RX2_RST_GPIO_Port GPIOB
#define RX2_RST_Pin       GPIO_PIN_10

#define RX1_DIO0_GPIO_Port GPIOB
#define RX1_DIO0_Pin       GPIO_PIN_1
#define RX2_DIO0_GPIO_Port GPIOB
#define RX2_DIO0_Pin       GPIO_PIN_11

/* LED from CubeMX (jeśli nie masz LED_GPIO_Port/LED_Pin w main.h, usuń miganie) */
#ifndef LED_GPIO_Port
#define LED_GPIO_Port GPIOC
#endif
#ifndef LED_Pin
#define LED_Pin GPIO_PIN_13

/* =============================================================================
 * LED blink helper (non-blocking, low-power friendly)
 * =============================================================================
 * Motivation:
 *  - Do NOT use HAL_GPIO_TogglePin() on RX/TX events, because it can leave the LED
 *    ON for a long time (wasting power) depending on packet timing.
 *
 * Behavior:
 *  - RX event  -> 1 short blink
 *  - TX event  -> 2 short blinks
 *
 * Implementation:
 *  - Non-blocking state machine driven by HAL_GetTick().
 *  - Call LED_Task() regularly from the main loop.
 * =============================================================================
 */

typedef struct
{
  uint8_t transitions_left;   /**< Remaining ON/OFF transitions (2 per blink). */
  uint8_t phase;              /**< 0 = LED OFF, 1 = LED ON. */
  uint32_t next_tick;         /**< Next tick to toggle LED phase. */
} led_blink_t;

static led_blink_t g_led = {0};

#ifndef LED_ON_TIME_MS
#define LED_ON_TIME_MS  40u
#endif
#ifndef LED_OFF_TIME_MS
#define LED_OFF_TIME_MS 60u
#endif

#define LED_ON()   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)


/**
 * @brief Schedule N LED blinks (non-blocking).
 * @param count Number of blinks to perform (1=RX, 2=TX).
 */
static void LED_Blink(uint8_t count)
{
  if (count == 0) return;

  /* Each blink consists of ON + OFF => 2 transitions. */
  g_led.transitions_left = (uint8_t)(count * 2u);
  g_led.phase = 0;
  g_led.next_tick = HAL_GetTick();
}

/**
 * @brief LED state machine task (call from the main loop).
 */
static void LED_Task(void)
{
  if (g_led.transitions_left == 0) return;

  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - g_led.next_tick) < 0) return;

  if (g_led.phase == 0)
  {
	LED_ON();
    g_led.phase = 1;
    g_led.next_tick = now + LED_ON_TIME_MS;
  }
  else
  {
	LED_OFF();
    g_led.phase = 0;
    g_led.transitions_left--;
    g_led.next_tick = now + LED_OFF_TIME_MS;
  }
}

#endif

/* =========================
   SX127x registers (LoRa mode)
   ========================= */
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT         0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PAYLOAD_LENGTH          0x22
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_MODEM_CONFIG_3          0x26
#define REG_RSSI_WIDEBAND           0x2C
#define REG_INVERT_IQ               0x33
#define REG_INVERT_IQ2              0x3B
#define REG_SYNC_WORD               0x39
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42

/* =========================
   SX127x modes / flags
   ========================= */
#define LONG_RANGE_MODE             0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RX_CONTINUOUS          0x05

/* IRQ flags */
#define IRQ_RX_TIMEOUT          0x80
#define IRQ_RX_DONE             0x40
#define IRQ_VALID_HEADER        0x10
#define IRQ_PAYLOAD_CRC_ERROR   0x20
#define IRQ_TX_DONE             0x08

/* =========================
   Konfiguracja LoRa (jak u Ciebie)
   ========================= */
#define RX1_LORA_FREQ_HZ     434855000UL
#define RX2_LORA_FREQ_HZ     434955000UL
#define RX_LORA_SYNCWORD     0x12   // private
#define RX_LORA_INVERT_IQ    0      // 0=normal, 1=inverted IQ

// BW=125kHz + CR=4/7 + Explicit header
#define RX_LORA_BW_CR_EXPL   0x74
// SF9 + CRC ON
#define RX_LORA_SF_CRC       0x94
// AGC auto
#define RX_LORA_MODEM_CFG3   0x04

/* =========================
   Prototypy
   ========================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void Error_Handler(void);


/* =========================
 * UART / logging helpers
 * ========================= */

/**
 * @brief Log levels (higher = more verbose).
 */
typedef enum
{
  LOG_LEVEL_OFF   = 0,
  LOG_LEVEL_ERROR = 1,
  LOG_LEVEL_INFO  = 2,
  LOG_LEVEL_DEBUG = 3
} log_level_t;

/**
 * @brief Global log gate. When 0, logs are silent (except PANIC).
 */
static volatile uint8_t g_uart_log_enabled =
#if (APP_LOG_LEVEL_DEFAULT == 0u)
  0u;
#else
  1u;
#endif

/**
 * @brief Global log level (verbosity threshold).
 */
static volatile log_level_t g_log_level = (log_level_t)APP_LOG_LEVEL_DEFAULT;

/**
 * @brief Enable/disable UART logs at runtime (fast kill-switch).
 */
static inline void UART_LogEnable(uint8_t enable)
{
  g_uart_log_enabled = (enable ? 1u : 0u);
}

/**
 * @brief Set log level at runtime.
 */
static inline void LOG_SetLevel(log_level_t level)
{
  g_log_level = level;
}

/**
 * @brief Returns 1 if a message at @p level should be printed.
 */
static inline uint8_t LOG_IsEnabled(log_level_t level)
{
  if (!g_uart_log_enabled) return 0u;
  if ((uint8_t)g_log_level < (uint8_t)level) return 0u;
  return 1u;
}

/**
 * @brief Ungated UART transmit (used for PANIC and other must-print paths).
 */
static void uart_raw_tx(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0) return;
  (void)HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 200);
}

/**
 * @brief Gated UART transmit (normal logs should use this).
 */
static void uart_tx(const uint8_t *data, uint16_t len)
{
  if (!g_uart_log_enabled) return;
  uart_raw_tx(data, len);
}

/**
 * @brief Print formatted log line at given level.
 */
static void LOG_Printf(log_level_t level, const char *fmt, ...)
{
  if (!LOG_IsEnabled(level)) return;

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_uart_buf, sizeof(g_uart_buf), fmt, ap);
  va_end(ap);

  uart_raw_tx((const uint8_t*)g_uart_buf, (uint16_t)strlen(g_uart_buf));
}

/* Convenience macros */
#define LOGE(...) LOG_Printf(LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOGI(...) LOG_Printf(LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOGD(...) LOG_Printf(LOG_LEVEL_DEBUG, __VA_ARGS__)

/* Backward-compatible wrappers (treat as INFO) */
static void uart_puts(const char *s)
{
  if (s == NULL) return;
  if (!LOG_IsEnabled(LOG_LEVEL_INFO)) return;
  uart_raw_tx((const uint8_t*)s, (uint16_t)strlen(s));
}

static void uart_printf(const char *fmt, ...)
{
  if (!LOG_IsEnabled(LOG_LEVEL_INFO)) return;

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_uart_buf, sizeof(g_uart_buf), fmt, ap);
  va_end(ap);

  uart_raw_tx((const uint8_t*)g_uart_buf, (uint16_t)strlen(g_uart_buf));
}

/**
 * @brief Initialize logging policy (defaults + optional service pin override).
 *
 * Must be called after MX_GPIO_Init() (to read service pin) and after
 * MX_USART1_UART_Init() (to be able to print).
 */
static void LOG_InitFromConfig(void)
{
#if (APP_LOG_LEVEL_DEFAULT == 0u)
  g_uart_log_enabled = 0u;
#else
  g_uart_log_enabled = 1u;
#endif
  g_log_level = (log_level_t)APP_LOG_LEVEL_DEFAULT;

#if (APP_LOG_SERVICE_PIN_ENABLE)
  if (HAL_GPIO_ReadPin(APP_LOG_SERVICE_GPIO_Port, APP_LOG_SERVICE_Pin) == APP_LOG_SERVICE_PIN_ACTIVE)
  {
    g_uart_log_enabled = 1u;
    if ((uint8_t)g_log_level < (uint8_t)LOG_LEVEL_DEBUG)
      g_log_level = LOG_LEVEL_DEBUG;
  }
#endif
}

static void system_panic_reset(const char *reason)
{
  /* Ostatni log – krótki i bezpieczny */
  uart_printf("PANIC: %s -> RESET\r\n", reason);

  /* Daj UARTowi chwilę na wysłanie */
  HAL_Delay(50);

  /* Twardy reset MCU */
  __disable_irq();
  NVIC_SystemReset();

  /* safety – nigdy tu nie wrócimy */
  while (1) {}
}

/**
 * @section spi_lock SPI bus lock
 * The radios share one SPI peripheral. We use a minimal spin-lock to prevent
 * concurrent transfers (e.g. from interrupt context vs. main loop).
 */
static volatile uint8_t spi_busy = 0;
static volatile uint32_t spi_err_count = 0;

static void SPI_Lock(void)   { while (spi_busy) {} spi_busy = 1; }
static void SPI_Unlock(void) { spi_busy = 0; }

/** @brief Assert/deassert NSS (chip select) for a selected radio. */
static inline void RADIO_NSS_LOW(uint8_t rid)
{
  if (rid == RID_RX1) HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_RESET);
  else               HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_RESET);
}
static inline void RADIO_NSS_HIGH(uint8_t rid)
{
  if (rid == RID_RX1) HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_SET);
  else               HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_SET);
}

/* =========================
   Reset SX127x
   ========================= */
static void RADIO_Reset(uint8_t rid)
{
  if (rid == RID_RX1)
  {
    HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_SET);
  }
  HAL_Delay(10);
}

/* =========================
   SPI read/write SX reg
   ========================= */
static uint8_t RADIO_ReadReg(uint8_t rid, uint8_t reg)
{
  uint8_t addr = reg & 0x7F;
  uint8_t val = 0;

  SPI_Lock();
  RADIO_NSS_LOW(rid);

  HAL_StatusTypeDef st1 = HAL_SPI_Transmit(&hspi1, &addr, 1, 50);
  HAL_StatusTypeDef st2 = (st1 == HAL_OK) ? HAL_SPI_Receive(&hspi1, &val, 1, 50) : st1;

  RADIO_NSS_HIGH(rid);
  SPI_Unlock();

  if (st2 != HAL_OK) spi_err_count++;
  return val;
}

static void RADIO_WriteReg(uint8_t rid, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { (uint8_t)(reg | 0x80), val };

  SPI_Lock();
  RADIO_NSS_LOW(rid);

  HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, buf, 2, 50);

  RADIO_NSS_HIGH(rid);
  SPI_Unlock();

  if (st != HAL_OK) spi_err_count++;
}

static void RADIO_ReadFifo(uint8_t rid, uint8_t *buf, uint8_t len)
{
  uint8_t addr = (uint8_t)(REG_FIFO & 0x7F);

  SPI_Lock();
  RADIO_NSS_LOW(rid);

  HAL_StatusTypeDef st1 = HAL_SPI_Transmit(&hspi1, &addr, 1, 50);
  HAL_StatusTypeDef st2 = (st1 == HAL_OK) ? HAL_SPI_Receive(&hspi1, buf, len, 200) : st1;

  RADIO_NSS_HIGH(rid);
  SPI_Unlock();

  if (st2 != HAL_OK) spi_err_count++;
}

static void RADIO_WriteFifo(uint8_t rid, const uint8_t *buf, uint8_t len)
{
  uint8_t addr = (uint8_t)(REG_FIFO | 0x80);

  SPI_Lock();
  RADIO_NSS_LOW(rid);

  HAL_StatusTypeDef st1 = HAL_SPI_Transmit(&hspi1, &addr, 1, 50);
  HAL_StatusTypeDef st2 = (st1 == HAL_OK) ? HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, 200) : st1;

  RADIO_NSS_HIGH(rid);
  SPI_Unlock();

  if (st2 != HAL_OK) spi_err_count++;
}

/* =========================
   Helpers: Mode + Freq + IQ
   ========================= */
static void RADIO_SetMode(uint8_t rid, uint8_t mode)
{
  RADIO_WriteReg(rid, REG_OP_MODE, (uint8_t)(LONG_RANGE_MODE | mode));
}

static void RADIO_SetFrequency_Hz(uint8_t rid, uint32_t hz)
{
  // FRF = (hz << 19) / 32e6
  uint64_t frf = ((uint64_t)hz << 19) / 32000000ULL;
  RADIO_WriteReg(rid, REG_FRF_MSB, (uint8_t)(frf >> 16));
  RADIO_WriteReg(rid, REG_FRF_MID, (uint8_t)(frf >> 8));
  RADIO_WriteReg(rid, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

static void RADIO_SetInvertIQ(uint8_t rid, uint8_t invert)
{
  if (!invert)
  {
    RADIO_WriteReg(rid, REG_INVERT_IQ,  0x27);
    RADIO_WriteReg(rid, REG_INVERT_IQ2, 0x1D);
  }
  else
  {
    RADIO_WriteReg(rid, REG_INVERT_IQ,  0x66);
    RADIO_WriteReg(rid, REG_INVERT_IQ2, 0x19);
  }
}

/* =========================
   Packet SNR/RSSI (debug)
   ========================= */
static int8_t RADIO_ReadPacketSNR_q4(uint8_t rid)
{
  return (int8_t)RADIO_ReadReg(rid, REG_PKT_SNR_VALUE);
}

static int16_t RADIO_ReadPacketRSSI_dBm(uint8_t rid)
{
  // approximate for 433MHz band
  uint8_t raw = RADIO_ReadReg(rid, REG_PKT_RSSI_VALUE);
  return (int16_t)raw - 157;
}

/* =========================
   LoRa init + RX start
   ========================= */
static void RADIO_RX_LoRaInit(uint8_t rid)
{
  RADIO_SetMode(rid, MODE_SLEEP);
  HAL_Delay(5);

  RADIO_WriteReg(rid, REG_FIFO_TX_BASE_ADDR, 0x00);
  RADIO_WriteReg(rid, REG_FIFO_RX_BASE_ADDR, 0x00);

  // LNA boost
  RADIO_WriteReg(rid, REG_LNA, (uint8_t)(RADIO_ReadReg(rid, REG_LNA) | 0x03));

  // SyncWord + modem config
  RADIO_WriteReg(rid, REG_SYNC_WORD, RX_LORA_SYNCWORD);

  // DIO mapping: DIO0=RxDone (LoRa)
  RADIO_WriteReg(rid, REG_DIO_MAPPING_1, 0x00);
  RADIO_WriteReg(rid, REG_DIO_MAPPING_2, 0x00);

  RADIO_WriteReg(rid, REG_MODEM_CONFIG_1, RX_LORA_BW_CR_EXPL);
  RADIO_WriteReg(rid, REG_MODEM_CONFIG_2, RX_LORA_SF_CRC);
  RADIO_WriteReg(rid, REG_MODEM_CONFIG_3, RX_LORA_MODEM_CFG3);

  RADIO_SetInvertIQ(rid, RX_LORA_INVERT_IQ);

  // Preamble = 8
  RADIO_WriteReg(rid, REG_PREAMBLE_MSB, 0x00);
  RADIO_WriteReg(rid, REG_PREAMBLE_LSB, 0x08);

  // DIO0 = RxDone
  uint8_t dmap1 = RADIO_ReadReg(rid, REG_DIO_MAPPING_1);
  dmap1 &= 0x3F;
  RADIO_WriteReg(rid, REG_DIO_MAPPING_1, dmap1);

  // clear IRQ
  RADIO_WriteReg(rid, REG_IRQ_FLAGS, 0xFF);

  RADIO_SetMode(rid, MODE_STDBY);
  HAL_Delay(5);
}

static void RADIO_RX_StartContinuous(uint8_t rid)
{
  RADIO_WriteReg(rid, REG_IRQ_FLAGS, 0xFF);
  RADIO_SetMode(rid, MODE_RX_CONTINUOUS);
}

/* =========================
   TX send on Radio2
   ========================= */
static void RADIO_TX_Send(uint8_t rid, const uint8_t *payload, uint8_t len)
{
  if (len == 0) return;

  RADIO_SetMode(rid, MODE_STDBY);
  HAL_Delay(1);

  // DIO0 = TxDone (LoRa): bits [7:6] = 01
  uint8_t dmap1 = RADIO_ReadReg(rid, REG_DIO_MAPPING_1);
  dmap1 = (uint8_t)((dmap1 & 0x3F) | 0x40);
  RADIO_WriteReg(rid, REG_DIO_MAPPING_1, dmap1);

  RADIO_WriteReg(rid, REG_IRQ_FLAGS, 0xFF);

  RADIO_WriteReg(rid, REG_FIFO_TX_BASE_ADDR, 0x00);
  RADIO_WriteReg(rid, REG_FIFO_ADDR_PTR, 0x00);

  RADIO_WriteFifo(rid, payload, len);
  RADIO_WriteReg(rid, REG_PAYLOAD_LENGTH, len);

  RADIO_SetMode(rid, MODE_TX);

  uint32_t t0 = HAL_GetTick();
  uint8_t tx_done = 0;

  while (HAL_GetTick() - t0 < 2000)
  {
    uint8_t irq = RADIO_ReadReg(rid, REG_IRQ_FLAGS);
    if (irq & IRQ_TX_DONE)
    {
      RADIO_WriteReg(rid, REG_IRQ_FLAGS, irq);
      tx_done = 1;
      break;
    }
  }

  /* TX się nie zakończył → reset całości */
  if (!tx_done)
  {
    system_panic_reset("TX stuck");
  }

  /* TX success indication: two short blinks */
  LED_Blink(2);

  /* R2 wraca do SLEEP (oszczędność energii) */
  RADIO_SetMode(rid, MODE_SLEEP);
}

/* =========================
   ADC: VDD via VrefInt
   ========================= */
static uint16_t ADC_ReadVrefintRaw(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = ADC_CHANNEL_VREFINT;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    system_panic_reset("ADC cfg");
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    system_panic_reset("ADC start");
  }

  if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    system_panic_reset("ADC poll");
  }

  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return raw;
}

static float ReadVdd_V(void)
{
  /* VDD = VrefInt * 4095 / ADC(VrefInt) */
  uint16_t raw = ADC_ReadVrefintRaw();
  if (raw == 0) return 0.0f;
  float vdd = ((float)VREFINT_MV * 4095.0f) / ((float)raw * 1000.0f);
  return vdd;
}

/* =========================
   APRS telemetry/status TX
   ========================= */
static void TELEMETRY_SendDefsOnce(void)
{
  /* Send PARM/UNIT/EQNS once per boot so aprs.fi can label/scale telemetry plots. */
  uint8_t buf[200];
  int n;

  n = snprintf((char*)buf, sizeof(buf), "%s>%s:%s", TELEM_CALLSIGN, TELEM_DEST, APRS_TLM_PARM);
  if (n > 0) RADIO_TX_Send(RID_RX2, buf, (uint8_t)((n > (int)sizeof(buf)) ? (int)sizeof(buf) : n));
  HAL_Delay(800);

  n = snprintf((char*)buf, sizeof(buf), "%s>%s:%s", TELEM_CALLSIGN, TELEM_DEST, APRS_TLM_UNIT);
  if (n > 0) RADIO_TX_Send(RID_RX2, buf, (uint8_t)((n > (int)sizeof(buf)) ? (int)sizeof(buf) : n));
  HAL_Delay(800);

  n = snprintf((char*)buf, sizeof(buf), "%s>%s:%s", TELEM_CALLSIGN, TELEM_DEST, APRS_TLM_EQNS);
  if (n > 0) RADIO_TX_Send(RID_RX2, buf, (uint8_t)((n > (int)sizeof(buf)) ? (int)sizeof(buf) : n));
  HAL_Delay(800);
}

static void APRS_SendPositionOnce(void)
{
  /* Fixed position (map placement). */
  uint8_t buf[180];
  int n = snprintf((char*)buf, sizeof(buf),
                   "%s>%s:!%s%c%s%c%s",
                   TELEM_CALLSIGN, TELEM_DEST,
                   APRS_LAT, (char)APRS_SYMBOL_TABLE,
                   APRS_LON, (char)APRS_SYMBOL_CODE,
                   APRS_BEACON_COMMENT);
  if (n <= 0) return;
  if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
  RADIO_TX_Send(RID_RX2, buf, (uint8_t)n);
}

static void TELEMETRY_SendVddOnce(void)
{
  static uint16_t seq = 0; /* 000..999 */
  float vdd = ReadVdd_V();

  /* Convert VDD [V] into A1 [0..255] with scale: V = A1 * 0.02  (A1 = V*50) */
  int a1 = (int)(vdd * 50.0f + 0.5f);
  if (a1 < 0) a1 = 0;
  if (a1 > 255) a1 = 255;

  /* Debug print after boot and each telemetry send */
  LOGI("[TLM] VDD=%.3f V -> A1=%d\r\n", (double)vdd, a1);

  /* APRS telemetry frame: T#sss,aaa,bbb,ccc,ddd,eee,xxxxxxxx */
  int n = snprintf((char*)g_telem_buf, sizeof(g_telem_buf),
                   "%s>%s:T#%03u,%03u,000,000,000,000,00000000",
                   TELEM_CALLSIGN, TELEM_DEST,
                   (unsigned)(seq % 1000u),
                   (unsigned)a1);
  if (n <= 0) return;
  if (n > (int)sizeof(g_telem_buf)) n = (int)sizeof(g_telem_buf);

  /* Wake TX radio, send, then it goes back to SLEEP in RADIO_TX_Send(). */
  RADIO_TX_Send(RID_RX2, g_telem_buf, (uint8_t)n);

  seq = (uint16_t)((seq + 1u) % 1000u);
}

/* =========================
   APRS print: HEX + ASCII (sanitized) with optional 3-byte header skip
   ========================= */
static void print_aprs_payload(uint8_t rid, const uint8_t *buf, uint8_t len)
{
#if (APP_LOG_LEVEL_DEFAULT < LOG_LEVEL_DEBUG)
  (void)rid; (void)buf; (void)len;
  return;
#endif

  uint8_t start = 0;

  /* Optional iGate header: 3C FF 01 */
  if (len > 3 && buf[0] == 0x3C && buf[1] == 0xFF && buf[2] == 0x01)
    start = 3;

  char *out = g_aprs_out;
  int pos = 0;

  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos,
                  "R%u HEX:", (unsigned)rid);
  for (uint8_t i = 0; i < len && pos < (int)sizeof(g_aprs_out) - 4; i++)
    pos += snprintf(out + pos, sizeof(g_aprs_out) - pos,
                    " %02X", buf[i]);
  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos, "\r\n");

  LOGD("%s", out);

  pos = 0;
  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos,
                  "R%u ASCII: ", (unsigned)rid);

  for (uint8_t i = start; i < len && pos < (int)sizeof(g_aprs_out) - 3; i++)
  {
    uint8_t c = buf[i];
    out[pos++] = (c >= 32 && c <= 126) ? (char)c : '.';
  }

  out[pos++] = '\r';
  out[pos++] = '\n';

  LOGD("%s", out);
}

/* =========================
   Odbiór pakietu dla danego radia (RX_DONE)
   ========================= */
static void RADIO_RX_ProcessIfAny(uint8_t rid)
{
  uint8_t irq = RADIO_ReadReg(rid, REG_IRQ_FLAGS);

  // tylko ValidHeader/Timeout -> czyścimy i wracamy
  if ((irq & (IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR)) == 0)
  {
    if (irq & (IRQ_VALID_HEADER | IRQ_RX_TIMEOUT))
      RADIO_WriteReg(rid, REG_IRQ_FLAGS, (uint8_t)(IRQ_VALID_HEADER | IRQ_RX_TIMEOUT));
    return;
  }

  // Czyścimy IRQ dopiero po RxDone/CRC error
  RADIO_WriteReg(rid, REG_IRQ_FLAGS, 0xFF);

  if (irq & IRQ_PAYLOAD_CRC_ERROR)
  {
    if (++crc_storm > 10)
      system_panic_reset("CRC storm");
  }
  else
  {
    crc_storm = 0;
  }

  // RX DONE
  uint8_t cur = RADIO_ReadReg(rid, REG_FIFO_RX_CURRENT);
  RADIO_WriteReg(rid, REG_FIFO_ADDR_PTR, cur);

  uint8_t len = RADIO_ReadReg(rid, REG_RX_NB_BYTES);
  if (len > 128) len = 128;

  uint8_t *buf = (rid == RID_RX1) ? g_rx_buf_1 : g_rx_buf_2;
  memset(buf, 0, 128);
  RADIO_ReadFifo(rid, buf, len);

  int16_t rssi = RADIO_ReadPacketRSSI_dBm(rid);
  int8_t snr_q4 = RADIO_ReadPacketSNR_q4(rid);

  int8_t snr_int = (int8_t)(snr_q4 / 4);
  uint8_t snr_frac = (uint8_t)(snr_q4 & 0x03);
  const char *snr_frac_txt = (snr_frac == 0) ? "00" : (snr_frac == 1) ? "25" : (snr_frac == 2) ? "50" : "75";

  uart_printf("R%u RX DONE: len=%u RSSI=%ddBm SNR=%d.%s dB\r\n",
              (unsigned)rid, (unsigned)len, (int)rssi, (int)snr_int, snr_frac_txt);

  print_aprs_payload(rid, buf, len);

  /* Forwarding: Radio1 RX -> queue TX on Radio2 (same bytes) */
  if (rid == RID_RX1)
  {
    if (!g_tx_pending_2)
    {
      g_tx_len_2 = len;
      memcpy((void*)g_tx_buf_2, buf, len);
      g_tx_pending_2 = 1;
    }
    else
    {
      uart_puts("R1 RX: TX queue busy -> drop\r\n");
    }
  }

  LED_Blink(1);
}

/** @brief EXTI flags set in HAL_GPIO_EXTI_Callback() and consumed in the main loop. */
static volatile uint8_t rx1_dio0_flag = 0;
static volatile uint8_t rx2_dio0_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == RX1_DIO0_Pin) rx1_dio0_flag = 1;
}

/* =========================================================
   main
   ========================================================= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  LOG_InitFromConfig();

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
  {
    UART_LogEnable(1);
    LOG_SetLevel(LOG_LEVEL_DEBUG);
    uart_raw_tx((uint8_t*)"DEBUG PIN ACTIVE\r\n", 18);
  }

  uart_puts("\r\n=== DUAL SX127x (RX1 RX, RX2 TX-FWD) START ===\r\n");

  // oba CS w high
  RADIO_NSS_HIGH(RID_RX1);
  RADIO_NSS_HIGH(RID_RX2);
  HAL_Delay(20);

  uart_puts("Reset RX1...\r\n");
  RADIO_Reset(RID_RX1);
  uart_puts("Reset RX2...\r\n");
  RADIO_Reset(RID_RX2);

  uint8_t v1 = RADIO_ReadReg(RID_RX1, REG_VERSION);
  uint8_t v2 = RADIO_ReadReg(RID_RX2, REG_VERSION);
  uart_printf("RX1 RegVersion=0x%02X\r\n", v1);
  uart_printf("RX2 RegVersion=0x%02X\r\n", v2);

  uart_printf("R1 freq=%luHz, R2 freq=%luHz BW125 SF9 CR4/7 Sync=0x%02X\r\n",
              (unsigned long)RX1_LORA_FREQ_HZ, (unsigned long)RX2_LORA_FREQ_HZ, (unsigned)RX_LORA_SYNCWORD);

  // init + start RX na obu
  RADIO_RX_LoRaInit(RID_RX1);
  RADIO_SetFrequency_Hz(RID_RX1, RX1_LORA_FREQ_HZ);
  RADIO_RX_StartContinuous(RID_RX1);

  RADIO_RX_LoRaInit(RID_RX2);
  RADIO_SetFrequency_Hz(RID_RX2, RX2_LORA_FREQ_HZ);
  /* R2 stays in SLEEP until TX (lower power) */
  RADIO_SetMode(RID_RX2, MODE_SLEEP);

  LED_OFF();

  if (v1 == 0x00 || v1 == 0xFF)
  {
    system_panic_reset("SX127x RX1 no response");
  }
  if (v2 == 0x00 || v2 == 0xFF)
  {
    system_panic_reset("SX127x RX2 no response");
  }

  uart_puts("Start RX continuous on RX1 and RX2...\r\n");

  /* --- Telemetry setup ---
   - Send APRS telemetry definitions (PARM/UNIT/EQNS) once per boot
   - Send a fixed position beacon once per boot (required for map placement)
   - Send first VDD telemetry immediately after boot
   - Then send VDD telemetry once per hour
 */
  TELEMETRY_SendDefsOnce();
  APRS_SendPositionOnce();
  TELEMETRY_SendVddOnce();
  uint32_t next_telem_tick = HAL_GetTick() + TELEM_INTERVAL_MS;

  uint32_t last_poll = HAL_GetTick();
  uint32_t log_boot_t0 = HAL_GetTick();
  uint8_t  log_force_on = 0;
#if (APP_LOG_SERVICE_PIN_ENABLE)
  if (HAL_GPIO_ReadPin(APP_LOG_SERVICE_GPIO_Port, APP_LOG_SERVICE_Pin) == APP_LOG_SERVICE_PIN_ACTIVE) log_force_on = 1;
#endif

  while (1)
  {
    /* Handle scheduled LED blinks (RX=1, TX=2) */
    LED_Task();

    static uint32_t last_led = 0;
    if (HAL_GetTick() - last_led > 500)
    {
      last_led = HAL_GetTick();
      // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // opcjonalnie
    }


    /* Optional: auto-mute logs after boot (saves power / reduces UART spam) */
#if (APP_LOG_AUTO_MUTE_MS > 0u)
    if (!log_force_on && g_uart_log_enabled && (HAL_GetTick() - log_boot_t0 >= APP_LOG_AUTO_MUTE_MS))
    {
      UART_LogEnable(0);
    }
#endif

    // EXTI-driven
    if (rx1_dio0_flag) { rx1_dio0_flag = 0; RADIO_RX_ProcessIfAny(RID_RX1); }

    /* TX forwarding: if R1 queued a packet, transmit it on Radio2 */
    if (g_tx_pending_2)
    {
      uint8_t len = g_tx_len_2;
      g_tx_pending_2 = 0;

      uart_printf("R2 TX: forwarding %u bytes\r\n", (unsigned)len);
      RADIO_TX_Send(RID_RX2, g_tx_buf_2, len);
    }

    /* Hourly VDD telemetry (skip if a forward TX is pending) */
    if (!g_tx_pending_2 && (int32_t)(HAL_GetTick() - next_telem_tick) >= 0)
    {
      TELEMETRY_SendVddOnce();
      next_telem_tick += TELEM_INTERVAL_MS;
    }

    // polling co ~50ms (działa nawet bez EXTI w it.c)
    if (HAL_GetTick() - last_poll >= 50)
    {
      last_poll = HAL_GetTick();
      RADIO_RX_ProcessIfAny(RID_RX1);
    }

    // restarts on spi errors
    if (spi_err_count > 50)
    {
      system_panic_reset("SPI error storm");
    }
  }
}

/* =========================================================
   Minimalne inicjalizacje (jeśli masz CubeMX, możesz zostawić swoje)
   ========================================================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 64MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // NSS outputs
  HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = RX1_NSS_Pin;
  HAL_GPIO_Init(RX1_NSS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RX2_NSS_Pin;
  HAL_GPIO_Init(RX2_NSS_GPIO_Port, &GPIO_InitStruct);

  // RESET outputs
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = RX1_RST_Pin;
  HAL_GPIO_Init(RX1_RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RX2_RST_Pin;
  HAL_GPIO_Init(RX2_RST_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_SET);


#if (APP_LOG_SERVICE_PIN_ENABLE)
  /* Service pin input (used to force logging ON at boot). Adjust Pull as needed. */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* common: button/jumper to GND */
  GPIO_InitStruct.Pin = APP_LOG_SERVICE_Pin;
  HAL_GPIO_Init(APP_LOG_SERVICE_GPIO_Port, &GPIO_InitStruct);
#endif

// DIO0 inputs with EXTI rising
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = RX1_DIO0_Pin;
  HAL_GPIO_Init(RX1_DIO0_GPIO_Port, &GPIO_InitStruct);

  // LED output
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = LED_Pin;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
  /* Default LED state: OFF (saves power) */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  // NVIC for EXTI lines (RX1 on PB1 -> EXTI1, RX2 on PB11 -> EXTI15_10)
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  GPIO_InitStruct.Pin  = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  /* Enable ADC + run calibration once at boot for better stability */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while(1) {}
}
