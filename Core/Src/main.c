/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdint.h>
#include <SEGGER_RTT.h>

#include "st7735.h"
#include "fonts.h"

#include "hdc1080.h"
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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskHDC */
osThreadId_t TaskHDCHandle;
const osThreadAttr_t TaskHDC_attributes = {
  .name = "TaskHDC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

//HDC1080
volatile float temp;
volatile uint8_t humi;

struct TempHum tempHum;

volatile uint8_t read_data[2];
volatile uint16_t reg=0;

//RTT
static uint8_t logBuffer[32];

//RTC
RTC_TimeTypeDef sTime = {0};
RTC_TimeTypeDef photoTime = {0};
RTC_TimeTypeDef diffTime = {0};
RTC_DateTypeDef DateToUpdate = {0};

//
uint8_t photoCounter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
void StartDefaultTask(void *argument);
void StartDisplayTask(void *argument);
void StartTaskHDC(void *argument);

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_RTT_Init();

    /* add channel 1: log */
  SEGGER_RTT_ConfigUpBuffer(1, "Log", logBuffer, sizeof(logBuffer), SEGGER_RTT_MODE_NO_BLOCK_TRIM);
  SEGGER_RTT_printf(0, "Hello, RTT!");
  hdc1080_init(&hi2c1,Temperature_Resolution_11_bit,Humidity_Resolution_8_bit);
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
  ST7735_Init();
  //uint8_t Buffer[25] = {0};
  for(int i =1; i<128; i++)
      {
	  uint8_t  ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
          if (ret != HAL_OK) /* No ACK Received At That Address */
          {
        	  sprintf(logBuffer, "I2C OK on address - %d \n\r", i);
        	  SEGGER_RTT_printf(0, logBuffer);
          }
          else if(ret == HAL_OK)
          {
        	  sprintf(logBuffer, "I2C not OK on address - %d \n\r", i);
        	  SEGGER_RTT_printf(0, logBuffer);
          }
      }
  // Check border
  	      ST7735_FillScreen(ST7735_BLACK);

  	      for(int x = 0; x < ST7735_WIDTH; x++) {
  	          ST7735_DrawPixel(x, 0, ST7735_RED);
  	          ST7735_DrawPixel(x, ST7735_HEIGHT-1, ST7735_RED);
  	      }

  	      for(int y = 0; y < ST7735_HEIGHT; y++) {
  	          ST7735_DrawPixel(0, y, ST7735_RED);
  	          ST7735_DrawPixel(ST7735_WIDTH-1, y, ST7735_RED);
  	      }

  	      HAL_Delay(100);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of TaskHDC */
  TaskHDCHandle = osThreadNew(StartTaskHDC, NULL, &TaskHDC_attributes);

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
  //HAL_TIM_Base_Start_IT(&htim16);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x4;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LCD_BL_Pin|LCD_BL1_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin|LED_ORANGE_Pin|LCD_DC_Pin
                          |LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_BL_Pin LCD_BL1_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_BL1_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin LED_ORANGE_Pin LCD_DC_Pin
                           LCD_RST_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin|LED_ORANGE_Pin|LCD_DC_Pin
                          |LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PHOTO_Pin */
  GPIO_InitStruct.Pin = PHOTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PHOTO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == PHOTO_Pin)
	{
		//HAL_TIM_Base_Start_IT(&htim16);
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

		HAL_RTC_GetTime(&hrtc, &photoTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);

	photoTime.Hours = sTime.Hours;
	photoTime.Minutes = sTime.Minutes;
	photoTime.Seconds = sTime.Seconds;

	ST7735_FillScreen(ST7735_BLACK);

	hdc1080_ask_temp_hum(&hi2c1);
	osDelay(15);
	tempHum =hdc1080_receive_temp_hum(&hi2c1);
	uint8_t firstInit = 1;
	//tempHum = hdc1080_get_temp_hum(&hi2c1);
  for(;;)
  {
	  char buffer[30];
	  memset(buffer, 0, sizeof(buffer));
	  sprintf(buffer, "Humi:%d%% \n\r", tempHum.Humidity);
	  SEGGER_RTT_printf(0, buffer);
	  sprintf(buffer, "Humi:%d%%", tempHum.Humidity);
	  ST7735_WriteString(5, 8, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
	  memset(buffer, 0, sizeof(buffer));

	  sprintf(buffer, "Temp:%3.2fC\n\r", tempHum.Temperature);
	  SEGGER_RTT_printf(0, buffer);
	  sprintf(buffer, "Temp:%3.2f C", tempHum.Temperature);
	  ST7735_WriteString(5, 35, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  snprintf(buffer, 63, "Time:%02d:%02d:%02d ", sTime.Hours, sTime.Minutes, sTime.Seconds);
	  SEGGER_RTT_printf(0, buffer);
	  ST7735_WriteString(5, 90, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	  snprintf(buffer, 63, "Date %d-%d-20%d\n", DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
	  SEGGER_RTT_printf(0, buffer);

	  //
	  //Calculate difference
	  //

	  //Hours
	  if(firstInit)
	  {
		  diffTime.Hours = 0;
		  diffTime.Minutes = 0;
		  diffTime.Seconds = sTime.Seconds;
		  firstInit = 0;
	  }
	  else
	  {
	  	  if((photoTime.Hours > sTime.Hours) && photoTime.Hours != 0)
	  	  { diffTime.Hours = 60 - photoTime.Hours; }
	  	  else
	  	  { diffTime.Hours = sTime.Hours - photoTime.Hours; }

	  	  //Minutes
	  	  if((photoTime.Minutes > sTime.Minutes) && photoTime.Minutes != 0)
	  	  { diffTime.Minutes = 60 - photoTime.Minutes; }
	  	  else
	  	  { diffTime.Minutes = sTime.Minutes - photoTime.Minutes; }

	  	  //Seconds
	  	  if((photoTime.Seconds > sTime.Seconds) && photoTime.Seconds != 0)
	  	  { diffTime.Seconds = 60 - photoTime.Seconds; }
	  	  else
	  	  { diffTime.Seconds = sTime.Seconds - photoTime.Seconds; }
	  }

	  snprintf(buffer, 63, "Diff:%02d:%02d:%02d ", diffTime.Hours, diffTime.Minutes, diffTime.Seconds);
	  SEGGER_RTT_printf(0, buffer);
	  ST7735_WriteString(5, 63, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);

	  osDelay(1000);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartTaskHDC */
/**
* @brief Function implementing the TaskHDC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHDC */
void StartTaskHDC(void *argument)
{
  /* USER CODE BEGIN StartTaskHDC */
  /* Infinite loop */
  HAL_StatusTypeDef err_c;
  for(;;)
  {
	  err_c = hdc1080_ask_temp_hum(&hi2c1);
	  if(err_c == HAL_OK)
	  {
		  osDelay(15);
		  taskENTER_CRITICAL();
		  {
			  tempHum =hdc1080_receive_temp_hum(&hi2c1);
		  }
		  taskEXIT_CRITICAL();
		  //tempHum =hdc1080_receive_temp_hum(&hi2c1);
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_ORANGE_Pin);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		  SEGGER_RTT_printf(0, (char*)err_c);
	  }
	  osDelay(1000);
	  //tempHum = hdc1080_get_temp_hum(&hi2c1);
	  //hdc1080_start_measurement(&hi2c1,(float*)&temp,(uint8_t*)&humi);
  }
  /* USER CODE END StartTaskHDC */
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
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == htim16.Instance)
  {

  }
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
