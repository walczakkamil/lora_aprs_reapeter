#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

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
     DIO0    PB11 (EXTI11 => EXTI15_10)

   UART1 (log):
     TX PA9
     RX PA10

   LED:
     PC13
   ========================================================= */

/* ========= Piny LED ========= */
#define LED_GPIO_Port       GPIOC
#define LED_Pin             GPIO_PIN_13

/* ========= Radio IDs ========= */
#define RID_RX1  1
#define RID_RX2  2

/* ========= Piny RX1 ========= */
#define RX1_NSS_GPIO_Port    GPIOA
#define RX1_NSS_Pin          GPIO_PIN_4
#define RX1_RST_GPIO_Port    GPIOB
#define RX1_RST_Pin          GPIO_PIN_0
#define RX1_DIO0_GPIO_Port   GPIOB
#define RX1_DIO0_Pin         GPIO_PIN_1

/* ========= Piny RX2 ========= */
#define RX2_NSS_GPIO_Port    GPIOA
#define RX2_NSS_Pin          GPIO_PIN_3
#define RX2_RST_GPIO_Port    GPIOB
#define RX2_RST_Pin          GPIO_PIN_10
#define RX2_DIO0_GPIO_Port   GPIOB
#define RX2_DIO0_Pin         GPIO_PIN_11

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* Flagi z DIO0 (jeśli EXTI działa) */
static volatile uint8_t rx1_dio0_flag = 0;
static volatile uint8_t rx2_dio0_flag = 0;

/* =========================
   Konfiguracja LoRa RX (APRS over LoRa)
   Zgodnie z Twoim ustawieniem:
   - 434.855 MHz
   - BW 125 kHz
   - SF9
   - CR 4/7
   - CRC ON
   - SyncWord 0x12 (private)
   ========================= */
#define RX_LORA_FREQ_HZ      434855000UL
#define RX_LORA_SYNCWORD     0x12
#define RX_LORA_BW_CR_EXPL   0x74   // BW=125kHz(0x70) + CR=4/7(0x04) + Explicit(0)
#define RX_LORA_SF_CRC       0x94   // SF9(0x90) + CRC ON(0x04)
#define RX_LORA_MODEM_CFG3   0x04   // AGC AUTO ON

/* =========================
   UART log
   ========================= */
static void uart_puts(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 500);
}

static void uart_printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uart_puts(buf);
}

/* =========================
   CS/NSS helpers (dla 2 radii)
   ========================= */
static inline void RADIO_NSS_HIGH(uint8_t rid)
{
  if (rid == RID_RX1) HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_SET);
  else               HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_SET);
}

static inline void RADIO_NSS_LOW(uint8_t rid)
{
  if (rid == RID_RX1) HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_RESET);
  else               HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_RESET);
}

/* =========================
   Reset dla konkretnego radia
   ========================= */
static void RADIO_Reset(uint8_t rid)
{
  if (rid == RID_RX1)
  {
    HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
  }
  else
  {
    HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
  }
}

/* =========================
   SPI REG Read/Write (parametryzowane radiem)
   ========================= */
static uint8_t RADIO_ReadReg(uint8_t rid, uint8_t addr)
{
  uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
  uint8_t rx[2] = { 0, 0 };

  RADIO_NSS_LOW(rid);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 200);
  RADIO_NSS_HIGH(rid);

  return rx[1];
}

static void RADIO_WriteReg(uint8_t rid, uint8_t addr, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };

  RADIO_NSS_LOW(rid);
  HAL_SPI_Transmit(&hspi1, tx, 2, 200);
  RADIO_NSS_HIGH(rid);
}

static void RADIO_ReadFifo(uint8_t rid, uint8_t *buf, uint8_t len)
{
  uint8_t addr = 0x00 & 0x7F; // FIFO read
  RADIO_NSS_LOW(rid);
  HAL_SPI_Transmit(&hspi1, &addr, 1, 200);
  HAL_SPI_Receive(&hspi1, buf, len, 200);
  RADIO_NSS_HIGH(rid);
}

/* =========================
   SX127x reg map (LoRa)
   ========================= */
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_LNA                 0x0C
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_SNR_VALUE       0x19
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PREAMBLE_MSB        0x20
#define REG_PREAMBLE_LSB        0x21
#define REG_MODEM_CONFIG_3      0x26
#define REG_SYNC_WORD           0x39
#define REG_DIO_MAPPING_1       0x40
#define REG_VERSION             0x42

#define LONG_RANGE_MODE         0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_RX_CONTINUOUS      0x05

#define IRQ_RX_DONE             0x40
#define IRQ_PAYLOAD_CRC_ERROR   0x20

/* =========================
   LoRa helpers
   ========================= */
static void RADIO_SetMode(uint8_t rid, uint8_t mode)
{
  RADIO_WriteReg(rid, REG_OP_MODE, LONG_RANGE_MODE | mode);
}

static void RADIO_SetFrequency_Hz(uint8_t rid, uint32_t hz)
{
  // FRF = (freq * 2^19) / 32e6
  uint64_t frf = ((uint64_t)hz << 19) / 32000000ULL;
  RADIO_WriteReg(rid, REG_FRF_MSB, (uint8_t)(frf >> 16));
  RADIO_WriteReg(rid, REG_FRF_MID, (uint8_t)(frf >> 8));
  RADIO_WriteReg(rid, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

static void RADIO_RX_LoRaInit(uint8_t rid)
{
  RADIO_SetMode(rid, MODE_SLEEP);
  HAL_Delay(5);

  RADIO_WriteReg(rid, REG_FIFO_TX_BASE_ADDR, 0x00);
  RADIO_WriteReg(rid, REG_FIFO_RX_BASE_ADDR, 0x00);

  // LNA boost
  RADIO_WriteReg(rid, REG_LNA, RADIO_ReadReg(rid, REG_LNA) | 0x03);

  // SyncWord + modem config
  RADIO_WriteReg(rid, REG_SYNC_WORD, RX_LORA_SYNCWORD);
  RADIO_WriteReg(rid, REG_MODEM_CONFIG_1, RX_LORA_BW_CR_EXPL);
  RADIO_WriteReg(rid, REG_MODEM_CONFIG_2, RX_LORA_SF_CRC);
  RADIO_WriteReg(rid, REG_MODEM_CONFIG_3, RX_LORA_MODEM_CFG3);

  // Preamble = 8
  RADIO_WriteReg(rid, REG_PREAMBLE_MSB, 0x00);
  RADIO_WriteReg(rid, REG_PREAMBLE_LSB, 0x08);

  // DIO0 = RxDone (DIO0 mapping bits = 00)
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

static int16_t RADIO_ReadPacketRSSI_dBm(uint8_t rid)
{
  uint8_t v = RADIO_ReadReg(rid, REG_PKT_RSSI_VALUE);
  return (int16_t)(-157 + (int16_t)v);
}

/* SNR jako q4 (ćwiartki dB), bez floatów */
static int8_t RADIO_ReadPacketSNR_q4(uint8_t rid)
{
  return (int8_t)RADIO_ReadReg(rid, REG_PKT_SNR_VALUE);
}

/* =========================
   APRS pretty print:
   - jeśli payload zaczyna się od 3C FF 01 -> pomijamy 3 bajty
   - resztę drukujemy ASCII (niedrukowalne -> '.')
   ========================= */
static void print_aprs_payload(uint8_t rid, const uint8_t *buf, uint8_t len)
{
  uint8_t start = 0;

  if (len > 3 && buf[0] == 0x3C && buf[1] == 0xFF && buf[2] == 0x01)
    start = 3;

  uart_printf("R%u APRS: ", (unsigned)rid);

  for (uint8_t i = start; i < len; i++)
  {
    uint8_t c = buf[i];
    if (c == '\r' || c == '\n' || c == '\t' || (c >= 0x20 && c <= 0x7E))
      HAL_UART_Transmit(&huart1, &c, 1, 100);
    else
    {
      uint8_t dot = '.';
      HAL_UART_Transmit(&huart1, &dot, 1, 100);
    }
  }
  uart_puts("\r\n");
}

/* =========================
   Odbiór pakietu dla danego radia (RX_DONE)
   ========================= */
static void RADIO_RX_ProcessIfAny(uint8_t rid)
{
  uint8_t irq = RADIO_ReadReg(rid, REG_IRQ_FLAGS);
  if ((irq & (IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR)) == 0)
    return; // nic nowego

  RADIO_WriteReg(rid, REG_IRQ_FLAGS, 0xFF); // clear

  if (irq & IRQ_PAYLOAD_CRC_ERROR)
  {
    uart_printf("R%u RX: CRC ERROR\r\n", (unsigned)rid);
    return;
  }

  if (irq & IRQ_RX_DONE)
  {
    // gdzie payload
    uint8_t cur = RADIO_ReadReg(rid, REG_FIFO_RX_CURRENT);
    RADIO_WriteReg(rid, REG_FIFO_ADDR_PTR, cur);

    uint8_t len = RADIO_ReadReg(rid, REG_RX_NB_BYTES);
    if (len > 128) len = 128;

    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));
    RADIO_ReadFifo(rid, buf, len);

    int16_t rssi = RADIO_ReadPacketRSSI_dBm(rid);
    int8_t snr_q4 = RADIO_ReadPacketSNR_q4(rid);

    // druk SNR jako X.YY bez float
    int8_t snr_int = (int8_t)(snr_q4 / 4);
    uint8_t snr_frac = (uint8_t)(snr_q4 & 0x03);
    const char *snr_frac_txt = (snr_frac == 0) ? "00" : (snr_frac == 1) ? "25" : (snr_frac == 2) ? "50" : "75";

    uart_printf("R%u RX DONE: len=%u RSSI=%ddBm SNR=%d.%s dB\r\n",
                (unsigned)rid, (unsigned)len, (int)rssi, (int)snr_int, snr_frac_txt);

    print_aprs_payload(rid, buf, len);

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

/* =========================================================
   EXTI callback (działa jeśli w stm32f1xx_it.c masz IRQ handler
   który woła HAL_GPIO_EXTI_IRQHandler() dla PB1 i PB11)
   ========================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == RX1_DIO0_Pin) rx1_dio0_flag = 1;
  if (GPIO_Pin == RX2_DIO0_Pin) rx2_dio0_flag = 1;
}

/* =========================
   Init: Clock / GPIO / SPI / UART (samowystarczalne)
   ========================= */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // 4 MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;              // 36 MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) while(1){}

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) while(1){}
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // LED PC13
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  // CS1 PA4
  GPIO_InitStruct.Pin = RX1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RX1_NSS_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RX1_NSS_GPIO_Port, RX1_NSS_Pin, GPIO_PIN_SET);

  // CS2 PA3
  GPIO_InitStruct.Pin = RX2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RX2_NSS_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RX2_NSS_GPIO_Port, RX2_NSS_Pin, GPIO_PIN_SET);

  // RST1 PB0
  GPIO_InitStruct.Pin = RX1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RX1_RST_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RX1_RST_GPIO_Port, RX1_RST_Pin, GPIO_PIN_SET);

  // RST2 PB10
  GPIO_InitStruct.Pin = RX2_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RX2_RST_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RX2_RST_GPIO_Port, RX2_RST_Pin, GPIO_PIN_SET);

  // DIO0_1 PB1 EXTI rising
  GPIO_InitStruct.Pin = RX1_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RX1_DIO0_GPIO_Port, &GPIO_InitStruct);

  // DIO0_2 PB11 EXTI rising
  GPIO_InitStruct.Pin = RX2_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RX2_DIO0_GPIO_Port, &GPIO_InitStruct);

  // NVIC (nie szkodzi nawet jeśli IRQ handler jeszcze nie ogarnia PB11 – polling i tak działa)
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // SPI1: PA5/PA7 AF_PP, PA6 input
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // UART1: PA9 TX AF_PP, PA10 RX input
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   // Mode 0
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;       // Mode 0
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // stabilnie
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) while(1){}
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

  if (HAL_UART_Init(&huart1) != HAL_OK) while(1){}
}

/* =========================
   main
   ========================= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  uart_puts("\r\n=== DUAL SX127x (RX1+RX2) START ===\r\n");

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

  uart_printf("RX cfg: freq=%luHz BW125 SF9 CR4/7 Sync=0x%02X\r\n",
              (unsigned long)RX_LORA_FREQ_HZ, (unsigned)RX_LORA_SYNCWORD);

  // init + start RX na obu (na razie oba identycznie)
  RADIO_RX_LoRaInit(RID_RX1);
  RADIO_SetFrequency_Hz(RID_RX1, RX_LORA_FREQ_HZ);
  RADIO_RX_StartContinuous(RID_RX1);

  RADIO_RX_LoRaInit(RID_RX2);
  RADIO_SetFrequency_Hz(RID_RX2, RX_LORA_FREQ_HZ);
  RADIO_RX_StartContinuous(RID_RX2);

  uart_puts("Start RX continuous on RX1 and RX2...\r\n");

  uint32_t last_poll = HAL_GetTick();

  while (1)
  {
    // jeśli działają EXTI: reaguj szybko
    if (rx1_dio0_flag) { rx1_dio0_flag = 0; RADIO_RX_ProcessIfAny(RID_RX1); }
    if (rx2_dio0_flag) { rx2_dio0_flag = 0; RADIO_RX_ProcessIfAny(RID_RX2); }

    // polling co ~50ms (działa nawet bez EXTI w it.c)
    if (HAL_GetTick() - last_poll >= 50)
    {
      last_poll = HAL_GetTick();
      RADIO_RX_ProcessIfAny(RID_RX1);
      RADIO_RX_ProcessIfAny(RID_RX2);
    }

    HAL_Delay(1);
  }
}
