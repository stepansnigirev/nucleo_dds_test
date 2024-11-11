/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9910.h"
#include <string.h>
#include "server.h"
#include <stdio.h>

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
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

#define DEBUG_LOG 0

void Serial_print(const char * str){
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
}
void Serial_println(const char * str){
	Serial_print(str);
	Serial_print("\r\n");
}
void SPI_Transmit(uint8_t *strBuffer, int nums, int pause)
{
#if DEBUG_LOG
	Serial_print("SPI transmit: ");
	char s[3] = "";
	for (int i=0;i<nums; i++)
	{
		sprintf(s, "%02x", strBuffer[i]);
		Serial_print(s);
	}
	Serial_println(" ");
#endif
	GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, strBuffer, nums, pause);
	GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
}

void GPIO_WritePin(GPIO_TypeDef *port, int pin, int mode)
{
#if DEBUG_LOG
	char s[200] = "";
	sprintf(s, "Pin %d set to %d", pin, mode);
	Serial_println(s);
#endif
	HAL_GPIO_WritePin(port, pin, mode);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  char tx_buff[] = "Hello from Nucleo-DDS\r\n";
  while (1)
  {
//	HAL_Delay(500);
//	Serial_print("*");
//
//	GPIO_WritePin(DDS_DRCTL_GPIO_PORT, DDS_DRCTL_PIN, GPIO_PIN_SET);
//	HAL_Delay(10);
//	GPIO_WritePin(DDS_DRCTL_GPIO_PORT, DDS_DRCTL_PIN, GPIO_PIN_RESET);
//
//	//	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, strlen(tx_buff), 1000);
//		GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
//		GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
//		GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
//		HAL_Delay(500);
//		GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
//		GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
//		GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);

//	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, strlen(tx_buff), 1000);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE7 PE8
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF4 PF7 PF8
                           PF9 PF10 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB2 PB10 PB11
                           LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD0 PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define CMDLEN 256
static char cmd[CMDLEN] = "";

int exec_profile(const char * cmd){
	unsigned int profile = 0;
	uint32_t freq = 0;
	uint32_t amp = 0;
	int count = sscanf(cmd, "%u %lu %lu", &profile, &freq, &amp);
	char arr[255];
	sprintf(arr, "%d PROFILE %d <- %lu %lu", count, profile, freq, amp);
	Serial_print(arr);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(freq > 500){
		Serial_println("Invalid frequency. Should be between 1 and 500");
		return -2;
	}
	if(profile > 7){
		Serial_println("Invalid profile");
		return -3;
	}
	SingleProfileFreqOut(freq*1000000, 0, profile);
	return 0;
}

int exec_setprofile(const char * cmd){
	unsigned int profile = 0;
	int count = sscanf(cmd, "%u", &profile);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(profile > 7){
		Serial_println("Invalid profile");
		return -3;
	}
	GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, profile&1);
	GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, (profile>>1)&1);
	GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, (profile>>2)&1);
	return 0;
}

int exec_sweep(const char * cmd){
	uint32_t start;
	uint32_t stop;
	uint32_t period_ms;
	int autoclear = 0;
	int count = sscanf(cmd, "%lu %lu %lu %d", &start, &stop, &period_ms, &autoclear);
	if(count < 3){
		Serial_println("Error parsing command");
		return -1;
	}
	if(count < 4){
		autoclear = 1;
	}
	Sweep(start*1000000, stop*1000000, period_ms, (uint8_t)autoclear);
	return 0;
}

int exec_drhold(const char * cmd){
	unsigned int value = 0;
	int count = sscanf(cmd, "%u", &value);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(value > 1){
		Serial_println("Invalid value");
		return -3;
	}
	GPIO_WritePin(DDS_DRHOLD_GPIO_PORT, DDS_DRHOLD_PIN, value&1);
	return 0;
}

int exec_drctrl(const char * cmd){
	unsigned int value = 0;
	int count = sscanf(cmd, "%u", &value);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(value > 1){
		Serial_println("Invalid value");
		return -3;
	}
	GPIO_WritePin(DDS_DRCTL_GPIO_PORT, DDS_DRCTL_PIN, value&1);
	return 0;
}

int exec_pwr(const char * cmd){
	unsigned int value = 0;
	int count = sscanf(cmd, "%u", &value);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(value > 1){
		Serial_println("Invalid value");
		return -3;
	}
	GPIO_WritePin(DDS_PWR_DWN_GPIO_PORT, DDS_PWR_DWN_PIN, !(value&1));
	return 0;
}

int exec_txen(const char * cmd){
	unsigned int value = 0;
	int count = sscanf(cmd, "%u", &value);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(value > 1){
		Serial_println("Invalid value");
		return -3;
	}
	GPIO_WritePin(DDS_TxENABLE_GPIO_PORT, DDS_TxENABLE_PIN, value&1);
	return 0;
}

int exec_amplitude(const char * cmd){
	unsigned int value = 0;
	int count = sscanf(cmd, "%u", &value);
	if(count < 1){
		Serial_println("Error parsing command");
		return -1;
	}
	if(value >= (1<<14)){
		Serial_println("Invalid value");
		return -3;
	}
	GPIO_WritePin(DDS_D2_GPIO_PORT, DDS_D2_PIN, value&1);
	GPIO_WritePin(DDS_D3_GPIO_PORT, DDS_D3_PIN, (value>>1)&1);
	GPIO_WritePin(DDS_D4_GPIO_PORT, DDS_D4_PIN, (value>>2)&1);
	GPIO_WritePin(DDS_D5_GPIO_PORT, DDS_D5_PIN, (value>>3)&1);
	GPIO_WritePin(DDS_D6_GPIO_PORT, DDS_D6_PIN, (value>>4)&1);
	GPIO_WritePin(DDS_D7_GPIO_PORT, DDS_D7_PIN, (value>>5)&1);
	GPIO_WritePin(DDS_D8_GPIO_PORT, DDS_D8_PIN, (value>>6)&1);
	GPIO_WritePin(DDS_D9_GPIO_PORT, DDS_D9_PIN, (value>>7)&1);
	GPIO_WritePin(DDS_D10_GPIO_PORT, DDS_D10_PIN, (value>>8)&1);
	GPIO_WritePin(DDS_D11_GPIO_PORT, DDS_D11_PIN, (value>>9)&1);
	GPIO_WritePin(DDS_D12_GPIO_PORT, DDS_D12_PIN, (value>>10)&1);
	GPIO_WritePin(DDS_D13_GPIO_PORT, DDS_D13_PIN, (value>>11)&1);
	GPIO_WritePin(DDS_D14_GPIO_PORT, DDS_D14_PIN, (value>>12)&1);
	GPIO_WritePin(DDS_D15_GPIO_PORT, DDS_D15_PIN, (value>>13)&1);
	GPIO_WritePin(DDS_TxENABLE_GPIO_PORT, DDS_TxENABLE_PIN, 1);
//	HAL_Delay(100);
//	GPIO_WritePin(DDS_TxENABLE_GPIO_PORT, DDS_TxENABLE_PIN, 0);
	return 0;
}

void process_data(void * payload, uint16_t len){
	if(len+strlen(cmd) < CMDLEN){
		memcpy(cmd+strlen(cmd), payload, len);
	}
	Serial_print(cmd);
	char * p = strchr(cmd, '\r');
	if(p!=NULL){
		p[0] = '\n';
	}
	p = strchr(cmd, '\n');
	if(p!=NULL){
		Serial_println("new line!");
		p[0] = 0;
		if(memcmp(cmd, "profile ", strlen("profile "))==0){
			exec_profile(cmd+strlen("profile "));
		}else if(memcmp(cmd, "setprofile ", strlen("setprofile "))==0){
			exec_setprofile(cmd+strlen("setprofile "));
		}else if(memcmp(cmd, "sweep ", strlen("sweep "))==0){
			exec_sweep(cmd+strlen("sweep "));
		}else if(memcmp(cmd, "drhold ", strlen("drhold "))==0){
			exec_drhold(cmd+strlen("drhold "));
		}else if(memcmp(cmd, "drctrl ", strlen("drctrl "))==0){
			exec_drctrl(cmd+strlen("drctrl "));
		}else if(memcmp(cmd, "pwr ", strlen("pwr "))==0){
			exec_pwr(cmd+strlen("pwr "));
		}else if(memcmp(cmd, "txen ", strlen("txen "))==0){
			exec_txen(cmd+strlen("txen "));
		}else if(memcmp(cmd, "amplitude ", strlen("amplitude "))==0){
			exec_amplitude(cmd+strlen("amplitude "));
		}else{
			Serial_println("Invalid command");
		}
		memset(cmd, 0, CMDLEN);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

	Serial_println("\r\nHello world!");
	extern struct netif gnetif;
	while (ip4_addr_isany_val(*netif_ip4_addr(&gnetif))) {
	  osDelay(200);
	}
	Serial_println(ip4addr_ntoa(netif_ip4_addr(&gnetif)));
	// switch setup
	if(INIT_SWITCH_VALUE){
		  GPIO_WritePin(DDS_V2_GPIO_PORT, DDS_V2_PIN, GPIO_PIN_RESET);
		  GPIO_WritePin(DDS_V1_GPIO_PORT, DDS_V1_PIN, GPIO_PIN_SET);
		  // refclk LED and oscillator power setup
		  GPIO_WritePin(DDS_REF_LED_GPIO_PORT, DDS_REF_LED_PIN, GPIO_PIN_RESET);
	}else{
		  GPIO_WritePin(DDS_V1_GPIO_PORT, DDS_V1_PIN, GPIO_PIN_RESET);
		  GPIO_WritePin(DDS_V2_GPIO_PORT, DDS_V2_PIN, GPIO_PIN_SET);
		  // refclk LED and oscillator power setup
		  GPIO_WritePin(DDS_REF_LED_GPIO_PORT, DDS_REF_LED_PIN, GPIO_PIN_SET);
	}
	osDelay(10);
	// dds setup
	// DDS_Init(INIT_PLL, INIT_DIV, INIT_REFCLK);
	DDS_Init(true, false, 50000000);

	// set amplitude
	GPIO_WritePin(DDS_D0_GPIO_PORT, DDS_D0_PIN, 1);
	GPIO_WritePin(DDS_D1_GPIO_PORT, DDS_D1_PIN, 1);
	GPIO_WritePin(DDS_D2_GPIO_PORT, DDS_D2_PIN, 1);
	GPIO_WritePin(DDS_D3_GPIO_PORT, DDS_D3_PIN, 1);
	GPIO_WritePin(DDS_D4_GPIO_PORT, DDS_D4_PIN, 1);
	GPIO_WritePin(DDS_D5_GPIO_PORT, DDS_D5_PIN, 1);
	GPIO_WritePin(DDS_D6_GPIO_PORT, DDS_D6_PIN, 1);
	GPIO_WritePin(DDS_D7_GPIO_PORT, DDS_D7_PIN, 1);
	GPIO_WritePin(DDS_D8_GPIO_PORT, DDS_D8_PIN, 1);
	GPIO_WritePin(DDS_D9_GPIO_PORT, DDS_D9_PIN, 1);
	GPIO_WritePin(DDS_D10_GPIO_PORT, DDS_D10_PIN, 1);
	GPIO_WritePin(DDS_D11_GPIO_PORT, DDS_D11_PIN, 1);
	GPIO_WritePin(DDS_D12_GPIO_PORT, DDS_D12_PIN, 1);
	GPIO_WritePin(DDS_D13_GPIO_PORT, DDS_D13_PIN, 1);
	GPIO_WritePin(DDS_D14_GPIO_PORT, DDS_D14_PIN, 1);
	GPIO_WritePin(DDS_D15_GPIO_PORT, DDS_D15_PIN, 1);


	// single-tone
	SingleProfileFreqOut(1000000L, INIT_A * -1, 0);
	echo_init();



  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
	Serial_print("*");

//	GPIO_WritePin(DDS_DRCTL_GPIO_PORT, DDS_DRCTL_PIN, GPIO_PIN_SET);
//	osDelay(10);
//	GPIO_WritePin(DDS_DRCTL_GPIO_PORT, DDS_DRCTL_PIN, GPIO_PIN_RESET);

	//	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, strlen(tx_buff), 1000);
		GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
		GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		osDelay(500);
		GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
		GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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
