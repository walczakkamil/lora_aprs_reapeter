/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- KONFIGURACJA SPRZĘTOWA (Hardcoded dla pewności) ---
// RX Module (RFM96) - podłączony do SPI1
#define RX_CS_PORT    GPIOA
#define RX_CS_PIN     GPIO_PIN_4
#define RX_RST_PORT   GPIOB
#define RX_RST_PIN    GPIO_PIN_0

// TX Module (RFM96) - podłączony do SPI1
#define TX_CS_PORT    GPIOA
#define TX_CS_PIN     GPIO_PIN_3
#define TX_RST_PORT   GPIOB
#define TX_RST_PIN    GPIO_PIN_10

// Debug & LED
#define DEBUG_PIN_PORT GPIOB
#define DEBUG_PIN      GPIO_PIN_12 // Zmiana na PB12 zgodnie z MX_GPIO_Init
#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_13

// --- PARAMETRY LORA ---
#define LORA_RX_FREQ    434855000
#define LORA_TX_FREQ    434955000
#define LORA_SF         9
#define LORA_BW_IDX     7         		// 125 kHz
#define LORA_CR         3         		// 4/7
#define TELEMETRY_INTERVAL 3600000 		// 1h
#define APRS_CALLSIGN   "SP7FM-1"
#define APRS_COORDS     "!5144.22N/01934.44E"

// --- SYSTEM ---
#define QUEUE_SIZE 5
#define MAX_PKT_LEN 256

typedef struct {
    uint8_t data[MAX_PKT_LEN];
    uint8_t len;
} LoRaPacket;

uint16_t tx_reset_counter = 0;
LoRaPacket txQueue[QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueTail = 0;

uint32_t lastTelemetryTime = 0;
volatile uint8_t packetReceivedFlag = 0;

// Struktura pomocnicza modułu
typedef struct {
    GPIO_TypeDef* CS_Port;
    uint16_t      CS_Pin;
    GPIO_TypeDef* RST_Port;
    uint16_t      RST_Pin;
    uint32_t      Frequency;
} LoRa_Module;

LoRa_Module loraRX = { RX_CS_PORT, RX_CS_PIN, RX_RST_PORT, RX_RST_PIN, LORA_RX_FREQ };
LoRa_Module loraTX = { TX_CS_PORT, TX_CS_PIN, TX_RST_PORT, TX_RST_PIN, LORA_TX_FREQ };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void LoRa_Init(LoRa_Module* mod);
void LoRa_SetMode(LoRa_Module* mod, uint8_t mode);
uint8_t LoRa_Send(LoRa_Module* mod, uint8_t* data, uint8_t len);
uint8_t LoRa_Receive(LoRa_Module* mod, uint8_t* buffer);
void Queue_Push(uint8_t* data, uint8_t len);
int Queue_Pop(uint8_t* buffer);
void SendTelemetry(void);
void DebugPrint(const char *format, ...);
float GetInternalVoltage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Rejestry SX1278
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_PA_CONFIG 0x09
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_DIO_MAPPING_1 0x40
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05

uint8_t LoRa_ReadReg(LoRa_Module* mod, uint8_t addr) {
    uint8_t txByte = addr & 0x7F;
    uint8_t rxByte = 0;
    HAL_GPIO_WritePin(mod->CS_Port, mod->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &txByte, 1, 100);
    HAL_SPI_Receive(&hspi1, &rxByte, 1, 100);
    HAL_GPIO_WritePin(mod->CS_Port, mod->CS_Pin, GPIO_PIN_SET);
    return rxByte;
}

void LoRa_WriteReg(LoRa_Module* mod, uint8_t addr, uint8_t val) {
    uint8_t data[2] = { addr | 0x80, val };
    HAL_GPIO_WritePin(mod->CS_Port, mod->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, 100);
    HAL_GPIO_WritePin(mod->CS_Port, mod->CS_Pin, GPIO_PIN_SET);
}

void LoRa_Init(LoRa_Module* mod) {
    // Reset
    HAL_GPIO_WritePin(mod->RST_Port, mod->RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(mod->RST_Port, mod->RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    LoRa_SetMode(mod, MODE_SLEEP);
    LoRa_WriteReg(mod, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

    // Częstotliwość
    uint64_t frf = ((uint64_t)mod->Frequency << 19) / 32000000;
    LoRa_WriteReg(mod, REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(mod, REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
    LoRa_WriteReg(mod, REG_FRF_MSB + 2, (uint8_t)(frf >> 0));

    // Config: BW=125kHz (7), CR=4/7 (3) -> Reg 0x76
    uint8_t config1 = (7 << 4) | (LORA_CR << 1);
    LoRa_WriteReg(mod, REG_MODEM_CONFIG_1, config1);

    // SF=9, CRC ON
    LoRa_WriteReg(mod, REG_MODEM_CONFIG_2, (LORA_SF << 4) | 0x04);

    // Max Power
    LoRa_WriteReg(mod, REG_PA_CONFIG, 0xFF);

    LoRa_WriteReg(mod, REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_WriteReg(mod, REG_FIFO_RX_BASE_ADDR, 0);

    LoRa_SetMode(mod, MODE_STDBY);
}

void LoRa_SetMode(LoRa_Module* mod, uint8_t mode) {
    LoRa_WriteReg(mod, REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

uint8_t LoRa_Send(LoRa_Module* mod, uint8_t* data, uint8_t len) {
    // 1. Wybudzamy radio (przejście ze SLEEP do STDBY)
    LoRa_SetMode(mod, MODE_STDBY);

    // !!! WAŻNE: Dajemy czas na ustabilizowanie się oscylatora (Warm-up)
    // Bez tego zapis do FIFO może się nie udać zaraz po wybudzeniu.
    HAL_Delay(2);

    // 2. Wypełniamy bufor
    LoRa_WriteReg(mod, REG_FIFO_ADDR_PTR, 0);
    for(int i=0; i<len; i++) {
        LoRa_WriteReg(mod, REG_FIFO, data[i]);
    }

    // 3. Rozpoczynamy nadawanie
    LoRa_WriteReg(mod, 0x22, len);
    LoRa_SetMode(mod, MODE_TX);

    // 4. Czekamy na koniec TX
    uint32_t start = HAL_GetTick();
    uint8_t tx_success = 0;
    while(HAL_GetTick() - start < 2000) { // Timeout 2 sekundy
    	if(LoRa_ReadReg(mod, REG_IRQ_FLAGS) & 0x08) {
    		tx_success = 1; // Znaleziono flagę TxDone!
    		break;
		}
	}

    // 5. Czyścimy flagi
    LoRa_WriteReg(mod, REG_IRQ_FLAGS, 0xFF);

    // 6. Idziemy spać zamiast czuwać (zmiana STDBY -> SLEEP)
    LoRa_SetMode(mod, MODE_SLEEP);

    return tx_success; // Zwracamy wynik
}

uint8_t LoRa_Receive(LoRa_Module* mod, uint8_t* buffer) {
    uint8_t irq = LoRa_ReadReg(mod, REG_IRQ_FLAGS);
    if(irq & 0x40) { // RxDone
        if(!(irq & 0x20)) { // CRC OK
            uint8_t len = LoRa_ReadReg(mod, REG_RX_NB_BYTES);
            uint8_t currentAddr = LoRa_ReadReg(mod, REG_FIFO_RX_CURRENT_ADDR);
            LoRa_WriteReg(mod, REG_FIFO_ADDR_PTR, currentAddr);
            for(int i=0; i<len; i++) {
                buffer[i] = LoRa_ReadReg(mod, REG_FIFO);
            }
            LoRa_WriteReg(mod, REG_IRQ_FLAGS, 0xFF);
            return len;
        }
    }
    LoRa_WriteReg(mod, REG_IRQ_FLAGS, 0xFF);
    return 0;
}

void DebugPrint(const char *format, ...) {
    // Debug aktywny gdy PB12 zwarty do masy
    if(HAL_GPIO_ReadPin(DEBUG_PIN_PORT, DEBUG_PIN) == GPIO_PIN_RESET) {
        char buffer[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
    }
}

void Queue_Push(uint8_t* data, uint8_t len) {
    uint8_t nextHead = (queueHead + 1) % QUEUE_SIZE;
    if(nextHead != queueTail) {
        memcpy(txQueue[queueHead].data, data, len);
        txQueue[queueHead].len = len;
        queueHead = nextHead;
        DebugPrint("QUEUE: Dodano pakiet (%d B)\r\n", len);
    } else {
        DebugPrint("QUEUE: FULL!\r\n");
    }
}

int Queue_Pop(uint8_t* buffer) {
    if(queueHead == queueTail) return 0;

    int len = txQueue[queueTail].len;
    memcpy(buffer, txQueue[queueTail].data, len);
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    return len;
}

float GetInternalVoltage(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adcVal = HAL_ADC_GetValue(&hadc1);
        if(adcVal == 0) return 0.0;
        // VREFINT typowo 1.20V
        return (1.20f * 4095.0f) / (float)adcVal;
    }
    return 0.0;
}

void SendTelemetry(void) {
    char packet[200]; // 128-> 200 zwiększam bufor dla bezpieczeństwa
    float voltage = GetInternalVoltage();

    // Format pełnej ramki LoRa APRS:
    // 1. Nagłówek LoRa: < (0x3C), 0xFF, 0x01
    // 2. Nagłówek AX.25: Źródło>Cel,Ścieżka:
    // 3. Dane APRS: !Lat/Lon#Komentarz
    // 4. TX reset counter
    sprintf(packet, "\x3c\xff\x01%s>APRS,WIDE1-1:%s#%s BAT:%.2fV R_CNT:%u", APRS_CALLSIGN, APRS_COORDS, APRS_CALLSIGN, voltage, tx_reset_counter);

    DebugPrint("TELEMETRY: %s\r\n", packet + 3); // +3 żeby nie wyświetlać krzaków w logu
    Queue_Push((uint8_t*)packet, strlen(packet)); // Wysyłamy całość (z krzakami)
}

// Przerwanie EXTI (dla RX DIO0 - PB1)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_1) {
        packetReceivedFlag = 1;
    }
}

// Sprawdzenie czy radio żyje
uint8_t LoRa_IsAlive(LoRa_Module* mod) {
    // Odczytujemy rejestr wersji (zawsze powinien zwracać 0x12 dla SX127x)
    uint8_t version = LoRa_ReadReg(mod, 0x42);

    // Dodatkowo sprawdzamy, czy rejestr trybu nie zawiera niemożliwych wartości
    uint8_t mode = LoRa_ReadReg(mod, 0x01);

    if (version == 0x12 && mode != 0xFF) {
        return 1; // Radio żyje
    }
    return 0; // Radio nie odpowiada lub magistrala SPI "wisi"
}

// Funkcja do odczytu i inkrementacji licznika
void Update_Reset_Counter(void) {
    // 1. Włącz zasilanie modułu PWR i dostęp do rejestrów BKP
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    // 2. Odczytaj wartość z rejestru 1 (możesz użyć DR1 do DR10)
    // Rejestry BKP przechowują wartości 16-bitowe
    tx_reset_counter = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

    // 3. Jeśli to był reset od Watchdoga (IWDG), zwiększ licznik
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        tx_reset_counter++;
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, tx_reset_counter);
    }

    // 4. Wyczyść flagi resetu, aby przy następnym uruchomieniu wiedzieć, co go wywołało
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  DebugPrint("SYS: Booting...\r\n");

  // Inicjalizacja RX (434.855 MHz)
  LoRa_Init(&loraRX);
  // Mapowanie DIO0 na RxDone (00)
  LoRa_WriteReg(&loraRX, REG_DIO_MAPPING_1, 0x00);
  LoRa_SetMode(&loraRX, MODE_RX_CONTINUOUS);
  DebugPrint("SYS: RX Init OK\r\n");

  // Inicjalizacja TX (434.955 MHz)
  LoRa_Init(&loraTX);
  LoRa_SetMode(&loraTX, MODE_SLEEP);
  DebugPrint("SYS: TX Init OK (Sleeping)\r\n");

  // Led na start
  // LED ON (PC13 Low), delay, LED OFF (PC13 High)
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  // Telemetria na start
  SendTelemetry();
  lastTelemetryTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. Watchdog
    HAL_IWDG_Refresh(&hiwdg);

    // 2. Obsługa odbioru
    if(packetReceivedFlag) {
		packetReceivedFlag = 0;
		uint8_t rxBuffer[MAX_PKT_LEN];
		uint8_t len = LoRa_Receive(&loraRX, rxBuffer);

		if(len > 0) {
			DebugPrint("RX: Recv %d bytes\r\n", len);

			// --- DEBUG: Podgląd odebranych danych ---
			// Sprawdzamy czy Debug Switch jest włączony (zwarty do masy)
			if(HAL_GPIO_ReadPin(DEBUG_PIN_PORT, DEBUG_PIN) == GPIO_PIN_RESET) {

				// 1. Wyświetlanie jako tekst (dla APRS)
				char debugMsg[MAX_PKT_LEN + 1];
				int safeLen = (len < MAX_PKT_LEN) ? len : MAX_PKT_LEN;
				memcpy(debugMsg, rxBuffer, safeLen);
				debugMsg[safeLen] = '\0'; // Bezpiecznik stringa
				DebugPrint("RX CONTENT (TXT): %s\r\n", debugMsg);

				// 2. Wyświetlanie jako HEX (gdyby tekst był krzakami)
				char hexBuf[10];
				DebugPrint("RX HEX: ");
				for(int i=0; i<safeLen; i++) {
					sprintf(hexBuf, "%02X ", rxBuffer[i]);
					HAL_UART_Transmit(&huart1, (uint8_t*)hexBuf, strlen(hexBuf), 10);
				}
				DebugPrint("\r\n");
			}
			// ----------------------------------------

			Queue_Push(rxBuffer, len);
		}

		// Restart RX Continuous
		LoRa_SetMode(&loraRX, MODE_RX_CONTINUOUS);
	}

    // Test radia TX - czy żyje
    if (!LoRa_IsAlive(&loraTX)) {
    	tx_reset_counter++;
        DebugPrint("SYS: TX Radio failure detected! Re-initializing...\r\n");

        // Twardy reset pinem RST tylko dla modułu TX
        HAL_GPIO_WritePin(GPIOB, TX_RST_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOB, TX_RST_Pin, GPIO_PIN_SET);
        HAL_Delay(10);

        // Ponowna inicjalizacja rejestrów
        LoRa_Init(&loraTX);
        LoRa_SetMode(&loraTX, MODE_SLEEP);
    }

    // 3. Obsługa nadawania
    if(queueHead != queueTail) {
		uint8_t txBuffer[MAX_PKT_LEN];
		int len = Queue_Pop(txBuffer);

		if(len > 0) {
			DebugPrint("TX: Preparing to send...\r\n");

			// --- NOWE: Podgląd treści ramki APRS w Debugu ---
			// Sprawdzamy pin ręcznie, aby nie tracić czasu procesora na memcpy, gdy debug jest wyłączony
			if(HAL_GPIO_ReadPin(DEBUG_PIN_PORT, DEBUG_PIN) == GPIO_PIN_RESET) {
				char debugMsg[MAX_PKT_LEN + 1]; // +1 na znak końca stringa

				// Zabezpieczenie przed przepełnieniem
				int safeLen = (len < MAX_PKT_LEN) ? len : MAX_PKT_LEN;

				memcpy(debugMsg, txBuffer, safeLen);
				debugMsg[safeLen] = '\0'; // Dodajemy terminator null

				DebugPrint("APRS CONTENT: %s\r\n", debugMsg);
			}
			// -----------------------------------------------

			// LED ON (PC13 Low)
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

			//LoRa_Send(&loraTX, txBuffer, len);
			if (LoRa_Send(&loraTX, txBuffer, len)) {
			    DebugPrint("TX: Success\r\n");
			} else {
			    DebugPrint("TX: FAILED! Re-queuing...\r\n");
			    tx_reset_counter++;
			    Queue_Push(txBuffer, len);
			}

			// LED OFF (PC13 High)
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
			DebugPrint("TX: Done.\r\n");
		}
	}

    // 4. Telemetria
    if(HAL_GetTick() - lastTelemetryTime > TELEMETRY_INTERVAL) {
        SendTelemetry();
        lastTelemetryTime = HAL_GetTick();
    }

    // 5. Sleep (tylko jeśli nic nie robimy)
    if(queueHead == queueTail && !packetReceivedFlag) {
        // Zatrzymanie Tick, aby nie budzić się co 1ms, ale EXTI LoRa obudzi MCU
        // Uwaga: IWDG działa niezależnie, więc MCU i tak zresetuje się jeśli za długo pośpi.
        // IWDG Reload jest ustawiony na ~4 sekundy, więc musimy się budzić częściej.
        // Bezpieczniej nie używać SuspendTick przy IWDG bez dokładnego wyliczania.
        // Używamy zwykłego WFI - budzi nas SysTick (1ms) lub LoRa EXTI.
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TX_CS_Pin|LORA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_RST_Pin|TX_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_CS_Pin LORA_CS_Pin */
  GPIO_InitStruct.Pin = TX_CS_Pin|LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin TX_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin|TX_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO0_Pin TX_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin|TX_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
