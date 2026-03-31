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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CC1200.h"
#include "cmsis_os2.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  Length_Byte,
  Data_Bytes
} Pkt_Phase;

typedef enum {
  No_CRC_Failure,
  CRC_Failure,
  CRC_Failure_Handled
} CRC_Failure_State;

typedef enum {
  Camera_Data,
  Sensor_Data
} Buffer_Type;

#define BUFFER_SIZE 1024

typedef struct {
  uint8_t buffer[BUFFER_SIZE]; // Data buffer
} buffer_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_BUFFER_COUNT 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiReceiveTask */
osThreadId_t spiReceiveTaskHandle;
const osThreadAttr_t spiReceiveTask_attributes = {
  .name = "spiReceiveTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for crcTask */
osThreadId_t crcTaskHandle;
const osThreadAttr_t crcTask_attributes = {
  .name = "crcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for spiTransmitTask */
osThreadId_t spiTransmitTaskHandle;
const osThreadAttr_t spiTransmitTask_attributes = {
  .name = "spiTransmitTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for SPI_Receive_Queue */
osMessageQueueId_t SPI_Receive_QueueHandle;
const osMessageQueueAttr_t SPI_Receive_Queue_attributes = {
  .name = "SPI_Receive_Queue"
};
/* Definitions for CRC_Queue */
osMessageQueueId_t CRC_QueueHandle;
const osMessageQueueAttr_t CRC_Queue_attributes = {
  .name = "CRC_Queue"
};
/* Definitions for SPI_Transmit_Queue */
osMessageQueueId_t SPI_Transmit_QueueHandle;
const osMessageQueueAttr_t SPI_Transmit_Queue_attributes = {
  .name = "SPI_Transmit_Queue"
};
/* USER CODE BEGIN PV */
Pkt_Phase Phase = Length_Byte; // Start with expecting the length byte

volatile uint32_t count, count2, count3, count4, count5, count6 = 0;
volatile uint32_t getSPI1, putSPI1, getCRC, putCRC, getSPI2, putSPI2 = 0;

static buffer_t *currentReceiveBuffer = NULL;
static buffer_t *prevReceiveBuffer = NULL;
static buffer_t *currentCRCBuffer = NULL;
static buffer_t *currentTransmitBuffer = NULL;

static CRC_Failure_State crcFailureState = No_CRC_Failure; // Flag to indicate the state of the last CRC check

static buffer_t dataBuffers[DATA_BUFFER_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);
void SPIReceiveTask(void *argument);
void CRCTask(void *argument);
void SPITransmitTask(void *argument);

/* USER CODE BEGIN PFP */
uint32_t Calculate_CRC(uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SPI_Receive_Queue */
  SPI_Receive_QueueHandle = osMessageQueueNew (30, sizeof(buffer_t*), &SPI_Receive_Queue_attributes);

  /* creation of CRC_Queue */
  CRC_QueueHandle = osMessageQueueNew (30, sizeof(buffer_t*), &CRC_Queue_attributes);

  /* creation of SPI_Transmit_Queue */
  SPI_Transmit_QueueHandle = osMessageQueueNew (30, sizeof(buffer_t*), &SPI_Transmit_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  for (int i = 0; i < DATA_BUFFER_COUNT; i++) {
    buffer_t *ptr = &dataBuffers[i];
    osMessageQueuePut(SPI_Receive_QueueHandle, &ptr, 0, 0);
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of spiReceiveTask */
  spiReceiveTaskHandle = osThreadNew(SPIReceiveTask, NULL, &spiReceiveTask_attributes);

  /* creation of crcTask */
  crcTaskHandle = osThreadNew(CRCTask, NULL, &crcTask_attributes);

  /* creation of spiTransmitTask */
  spiTransmitTaskHandle = osThreadNew(SPITransmitTask, NULL, &spiTransmitTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|SPI2_INT_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_MISO_U_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPI1_MISO_U_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin SPI2_INT_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|SPI2_INT_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CRC_OK_Pin */
  GPIO_InitStruct.Pin = CRC_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CRC_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if (hspi->Instance == SPI1) {
    osThreadFlagsSet(spiReceiveTaskHandle, 0x00000010U);
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
  if (hspi->Instance == SPI2) {
    osThreadFlagsSet(spiTransmitTaskHandle, 0x00000001U);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if (GPIO_Pin == CRC_OK_Pin) {
    // Send flag to spiTransmitTask indicating that the CRC check of the packet in the CC1200 RXFIFO is OK  (Flag 0x00000008U)
    osThreadFlagsSet(spiReceiveTaskHandle, 0x00000008U);
  }
}

uint32_t Calculate_CRC(uint8_t *data, uint16_t length)
{
    uint32_t crc;
    uint32_t word;
    uint16_t i;

    __HAL_CRC_DR_RESET(&hcrc);

    for (i = 0; i < length; i += 4)
    {
        word = 0;

        word |= data[i] << 24;

        if (i + 1 < length) word |= data[i + 1] << 16;
        if (i + 2 < length) word |= data[i + 2] << 8;
        if (i + 3 < length) word |= data[i + 3];

        crc = HAL_CRC_Accumulate(&hcrc, &word, 1);
    }

    return crc;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    count = osMessageQueueGetCount(SPI_Receive_QueueHandle);
    count2 = osMessageQueueGetCount(CRC_QueueHandle);
    count3 = osMessageQueueGetCount(SPI_Transmit_QueueHandle);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SPIReceiveTask */
/**
* @brief Function implementing the spiReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPIReceiveTask */
void SPIReceiveTask(void *argument)
{
  /* USER CODE BEGIN SPIReceiveTask */
  
  CC1200_SetSPIHandle(&hspi1, SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
  CC1200_SetUserMISOPins(SPI1_MISO_U_GPIO_Port, SPI1_MISO_U_Pin);

  uint8_t headerBuffer[3];
  
  CC1200_Init();

Reset_Receive:

  // Put CC1200 in RX Mode
  CC1200_CommandStrobe(CC1200_SIDLE);
  CC1200_CommandStrobe(CC1200_SFRX);
  CC1200_CommandStrobe(CC1200_SRX);

  osThreadFlagsClear(0x00000008U);

  uint8_t currentCamera1DoubleBuffer = 0, currentCamera2DoubleBuffer = 0;
  uint8_t prevCamera1DoubleBuffer = 0, prevCamera2DoubleBuffer = 0;

  // Wait for a buffer to be available from the SPI transmit queue
  osMessageQueueGet(SPI_Receive_QueueHandle, &currentReceiveBuffer, NULL, osWaitForever);
  getSPI1 = getSPI1 + 1;
  count = osMessageQueueGetCount(SPI_Receive_QueueHandle);
  uint16_t currentStartOffset = 0, prevStartOffset = 0;

  /* Infinite loop */
  for(;;){

    prevCamera1DoubleBuffer = currentCamera1DoubleBuffer;
    prevCamera2DoubleBuffer = currentCamera2DoubleBuffer; 
    currentCamera1DoubleBuffer = 0;
    currentCamera2DoubleBuffer = 0;
    
    CC1200_ReceiveHeader(&headerBuffer[0]);
    
    CC1200_CommandStrobe(CC1200_SRX);

    if (headerBuffer[0] == 0) {
      osMessageQueuePut(SPI_Receive_QueueHandle, &currentReceiveBuffer, 0, 0);
      goto Reset_Receive;
    }

    if ((headerBuffer[1] + currentStartOffset + 2 > BUFFER_SIZE-1) & (prevCamera1DoubleBuffer == 0) & (prevCamera2DoubleBuffer == 0)) {
      currentReceiveBuffer->buffer[currentStartOffset] = 0;
      osMessageQueuePut(CRC_QueueHandle, &currentReceiveBuffer, 0, 0);
      putSPI1 = putSPI1 + 1;
      osMessageQueueGet(SPI_Receive_QueueHandle, &currentReceiveBuffer, NULL, osWaitForever);
      getSPI1 = getSPI1 + 1;
      currentStartOffset = 0;
      prevStartOffset = 0;
    }

    if (((headerBuffer[2] & 0x80) != 0x80) & (prevCamera1DoubleBuffer | prevCamera2DoubleBuffer)) { // New camera stream packet started without finishing the previous one
      currentStartOffset = prevStartOffset;
      CC1200_CommandStrobe(CC1200_SIDLE);
      CC1200_CommandStrobe(CC1200_SFRX);
      CC1200_CommandStrobe(CC1200_SRX);
    }
    else if ((headerBuffer[2] & 0x80) == 0x80) { // Second part of camera stream packet
      if ((((headerBuffer[2] == 0x81) & prevCamera1DoubleBuffer) | ((headerBuffer[2] == 0x82) & prevCamera2DoubleBuffer)) 
      & (headerBuffer[1] == currentReceiveBuffer->buffer[currentStartOffset-CC1200_TX_FIFO_SIZE])) {
        if(CC1200_ReceivePayload(currentReceiveBuffer->buffer + currentStartOffset, headerBuffer[0]) == 1){
          currentStartOffset = prevStartOffset; // Discard the first part of the camera stream packet
        } else {
        prevStartOffset = currentStartOffset;
        currentStartOffset += headerBuffer[0]; // Rest of payload + 2byte CRC RSSI
        }
      } else {
          CC1200_CommandStrobe(CC1200_SIDLE);
          CC1200_CommandStrobe(CC1200_SFRX);
          CC1200_CommandStrobe(CC1200_SRX);
          currentStartOffset = prevStartOffset; // Discard the first part of the camera stream packet
      }
    } else if (headerBuffer[1] > CC1200_TX_FIFO_SIZE) {

      if (headerBuffer[2] == 0x01) {
        currentCamera1DoubleBuffer = 1;
      } else if (headerBuffer[2] == 0x02) {
        currentCamera2DoubleBuffer = 1;
      }
      
      currentReceiveBuffer->buffer[currentStartOffset] = headerBuffer[1];   // Packet Length
      currentReceiveBuffer->buffer[currentStartOffset+1] = headerBuffer[2]; // ID
      if(CC1200_ReceivePayload(currentReceiveBuffer->buffer + currentStartOffset + 2, headerBuffer[0]) == 0){
        prevStartOffset = currentStartOffset;
        currentStartOffset += headerBuffer[0]; // Payload (Packet Length + ID + DATA)
      }
      
    } else { //Sensor Data or Single Camera Data Buffer
      currentReceiveBuffer->buffer[currentStartOffset] = headerBuffer[1];   // Packet Length
      currentReceiveBuffer->buffer[currentStartOffset+1] = headerBuffer[2]; // ID
      if(CC1200_ReceivePayload(currentReceiveBuffer->buffer + currentStartOffset + 2, headerBuffer[0]) == 0){
      prevStartOffset = currentStartOffset;
      currentStartOffset += headerBuffer[0] + 2; // Payload (Packet Length + ID + DATA) + 2byte CRC RSSI
      }
    }

    count4 += 1;

  }
  /* USER CODE END SPIReceiveTask */
}

/* USER CODE BEGIN Header_CRCTask */
/**
* @brief Function implementing the crcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CRCTask */
void CRCTask(void *argument)
{
  /* USER CODE BEGIN CRCTask */
  /* Infinite loop */
  for(;;)
  {
    // Wait for a buffer to be available from the CRC queue
    if (osMessageQueueGet(CRC_QueueHandle, &currentCRCBuffer, NULL, osWaitForever) == osOK) {
      getCRC = getCRC + 1;
      // Calculate CRC for the buffer
      count2 = osMessageQueueGetCount(CRC_QueueHandle);
      uint16_t currentStartOffset = 0;
      uint8_t lastPacketFlg = 0;
      for(;;){
        uint8_t subPacketLength = currentCRCBuffer->buffer[currentStartOffset];

        lastPacketFlg = currentCRCBuffer->buffer[currentStartOffset + currentCRCBuffer->buffer[currentStartOffset] + 2] == 0 ? 1 : 0;

        if (subPacketLength%4 != 0){
          goto skip_crc_calculation;
        }

        uint32_t calculated_crc = Calculate_CRC(currentCRCBuffer->buffer + currentStartOffset, subPacketLength - 4); // Exclude the last 4 bytes which are the received CRC

        uint32_t received_crc = 
            (currentCRCBuffer->buffer[currentStartOffset + subPacketLength - 4] << 24) |
            (currentCRCBuffer->buffer[currentStartOffset + subPacketLength - 3] << 16) |
            (currentCRCBuffer->buffer[currentStartOffset + subPacketLength - 2] << 8)  |
            (currentCRCBuffer->buffer[currentStartOffset + subPacketLength - 1]);

        count6 = count6 + 1;

        if ((calculated_crc == received_crc) && lastPacketFlg == 1) {
          // CRC is correct, put the buffer into the SPI transmit queue
          osMessageQueuePut(SPI_Transmit_QueueHandle, &currentCRCBuffer, 0, 0);
          putCRC = putCRC + 1;
          break;
        } else if (calculated_crc != received_crc) {
skip_crc_calculation:
          // CRC is incorrect, set crcFailureState to CRC_Failure to indicate that a CRC failure has occurred
          osMessageQueuePut(SPI_Receive_QueueHandle, &currentCRCBuffer, 0, 0);
          // crcFailureState = CRC_Failure;
          // // Wait until failure is handled in spiReceiveTask before processing the next buffer in the CRC queue
          // osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
          count5 = count5 + 1;
          break;
        }

        currentStartOffset += subPacketLength + 2; // Skip the 2 byte CRC and RSSI from the CC1200
      }
      currentCRCBuffer = NULL;
    }
  }
  /* USER CODE END CRCTask */
}

/* USER CODE BEGIN Header_SPITransmitTask */
/**
* @brief Function implementing the spiTransmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPITransmitTask */
void SPITransmitTask(void *argument)
{
  /* USER CODE BEGIN SPITransmitTask */

  /* Infinite loop */
  for(;;)
  {
    // Wait for a buffer to be available from the SPI transmit queue
    if (osMessageQueueGet(SPI_Transmit_QueueHandle, &currentTransmitBuffer, NULL, osWaitForever) == osOK) {
      getSPI2 = getSPI2 + 1;
      count3 = osMessageQueueGetCount(SPI_Transmit_QueueHandle);

      // Transmit the buffer over SPI2 using DMA
      HAL_SPI_Transmit_DMA(&hspi2, currentTransmitBuffer->buffer, 1024);

      // Pulse the SPI2_INT_Pin to indicate to the receiving device that a new buffer is being transmitted
      HAL_GPIO_WritePin(SPI2_INT_GPIO_Port, SPI2_INT_Pin, GPIO_PIN_SET);

      // Wait for the transmission to complete by waiting for the flag set in the HAL_SPI_TxCpltCallback callback function
      osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
      HAL_GPIO_WritePin(SPI2_INT_GPIO_Port, SPI2_INT_Pin, GPIO_PIN_RESET);

      // We can now put it back into the the SPI or CAN receive queue depending on its type
      osMessageQueuePut(SPI_Receive_QueueHandle, &currentTransmitBuffer, 0, 0);
      putSPI2 = putSPI2 + 1;
    
      currentTransmitBuffer = NULL;
    }
  }
  /* USER CODE END SPITransmitTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
