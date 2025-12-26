#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ===== Global buffers to reduce stack usage (prevents hardfault-like hangs) ===== */
static char g_uart_buf[256];
static char g_aprs_out[240];
static uint8_t g_rx_buf_1[128];
static uint8_t g_rx_buf_2[128];

/* TX queue for Radio2 forwarding */
static uint8_t g_tx_buf_2[128];
static volatile uint8_t g_tx_len_2 = 0;
static volatile uint8_t g_tx_pending_2 = 0;

static uint8_t crc_storm = 0;

/* =========================================================
   DWA MODUŁY SX127x NA JEDNYM SPI1 (wariant A)

   Wspólne SPI1:
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

   LED (z CubeMX) np. PC13 albo inny - zależy od projektu.
   ========================================================= */


/* ===================== Handles ===================== */
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* ===================== Radio IDs ===================== */
#define RID_RX1 1
#define RID_RX2 2

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
void Error_Handler(void);

/* =========================
   UART helpers (log gating)
   ========================= */

/*
 * g_uart_log_enabled:
 *  - 1 => logi na UART/COM włączone
 *  - 0 => logi wyłączone (cisza na UART), oszczędność energii
 */
#ifdef DEBUG
static volatile uint8_t g_uart_log_enabled = 1;
#else
static volatile uint8_t g_uart_log_enabled = 0;
#endif

static inline void UART_LogEnable(uint8_t enable)
{
  g_uart_log_enabled = (enable ? 1u : 0u);
}

/* Niegatowany TX – używamy tylko w PANIC/reset itp. */
static void uart_raw_tx(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0) return;
  (void)HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 200);
}

/* Gatowany TX – normalne logi przechodzą tylko gdy flaga=1 */
static void uart_tx(const uint8_t *data, uint16_t len)
{
  if (!g_uart_log_enabled) return;
  uart_raw_tx(data, len);
}

static void uart_puts(const char *s)
{
  if (s == NULL) return;
  uart_tx((const uint8_t*)s, (uint16_t)strlen(s));
}

static void uart_printf(const char *fmt, ...)
{
  if (!g_uart_log_enabled) return;

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_uart_buf, sizeof(g_uart_buf), fmt, ap);
  va_end(ap);

  uart_raw_tx((const uint8_t*)g_uart_buf, (uint16_t)strlen(g_uart_buf));
}

static void system_panic_reset(const char *reason)
{
  /* PANIC zawsze wysyłamy (nawet gdy logi są wyłączone) */
  int n = snprintf(g_uart_buf, sizeof(g_uart_buf), "PANIC: %s -> RESET\r\n", (reason ? reason : "unknown"));
  if (n > 0) uart_raw_tx((const uint8_t*)g_uart_buf, (uint16_t)n);

  /* Daj UARTowi chwilę na wysłanie */
  HAL_Delay(50);

  /* Twardy reset MCU */
  __disable_irq();
  NVIC_SystemReset();

  /* safety – nigdy tu nie wrócimy */
  while (1) {}
}

/* =========================
   SPI lock (prosty)
   ========================= */
static volatile uint8_t spi_busy = 0;
static volatile uint32_t spi_err_count = 0;

static void SPI_Lock(void)   { while (spi_busy) {} spi_busy = 1; }
static void SPI_Unlock(void) { spi_busy = 0; }

/* =========================
   NSS helpers
   ========================= */
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

  /* R2 wraca do STDBY (oszczędność energii) */
  RADIO_SetMode(rid, MODE_STDBY);
}

/* =========================
   APRS print: HEX + ASCII (sanitized) with optional 3-byte header skip
   ========================= */
static void print_aprs_payload(uint8_t rid, const uint8_t *buf, uint8_t len)
{
  uint8_t start = 0;

  // iGate header: 3C FF 01
  if (len > 3 && buf[0] == 0x3C && buf[1] == 0xFF && buf[2] == 0x01)
    start = 3;

  char *out = g_aprs_out;
  int pos = 0;

  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos, "R%u HEX:", (unsigned)rid);
  for (uint8_t i = 0; i < len && pos < (int)sizeof(g_aprs_out) - 4; i++)
    pos += snprintf(out + pos, sizeof(g_aprs_out) - pos, " %02X", buf[i]);
  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos, "\r\n");

  uart_tx((const uint8_t*)out, (uint16_t)pos);

  pos = 0;
  pos += snprintf(out + pos, sizeof(g_aprs_out) - pos, "R%u ASCII: ", (unsigned)rid);

  for (uint8_t i = start; i < len && pos < (int)sizeof(g_aprs_out) - 3; i++)
  {
    uint8_t c = buf[i];
    if (c >= 32 && c <= 126) out[pos++] = (char)c;
    else out[pos++] = '.';
  }

  out[pos++] = '\r';
  out[pos++] = '\n';
  uart_tx((const uint8_t*)out, (uint16_t)pos);
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

  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* =========================================================
   EXTI flags (ustawiane w HAL_GPIO_EXTI_Callback)
   ========================================================= */
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
  /* R2 stays in STDBY until TX */
  RADIO_SetMode(RID_RX2, MODE_STDBY);


  if (v1 == 0x00 || v1 == 0xFF)
  {
    system_panic_reset("SX127x RX1 no response");
  }
  if (v2 == 0x00 || v2 == 0xFF)
  {
    system_panic_reset("SX127x RX2 no response");
  }

  uart_puts("Start RX continuous on RX1 and RX2...\r\n");

  uint32_t last_poll = HAL_GetTick();

  while (1)
  {
    static uint32_t last_led = 0;
    if (HAL_GetTick() - last_led > 500)
    {
      last_led = HAL_GetTick();
      // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // opcjonalnie
    }

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

  // NVIC for EXTI lines (RX1 on PB1 -> EXTI1, RX2 on PB11 -> EXTI15_10)
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

void Error_Handler(void)
{
  __disable_irq();
  while(1) {}
}
