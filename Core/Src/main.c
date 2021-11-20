/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "nokia_5110.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define FIRST_BIT 0x01
#define SECOND_BIT 0x02
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
   PING = 0,
   PONG = 1,
}cmd_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

osThreadId LCD_TaskHandle;
osThreadId LED_TASK_1Handle;
osThreadId LCD_Task_2Handle;
osThreadId PING_TASKHandle;
osThreadId PING_TASK_2Handle;
osTimerId oneShotTimerHandle;
osTimerId periodicTimerHandle;
osMutexId lcdMutexHandle;
SemaphoreHandle_t isrSemaphoreHandle;
/* USER CODE BEGIN PV */
QueueHandle_t pingQueue;
int lcd_count;
osThreadId EVT_GROUP_1Handle;
osThreadId EVT_GROUP_2Handle;
osThreadId EVT_GROUP_3Handle;
/* USER CODE END PV */
EventGroupHandle_t evtGroup;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void LCDTask1(void const * argument);
void LCDTask2(void const * argument);
void ledTask1(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void Callback01(void const * argument);
void Callback02(void const * argument);
void evtGroupSetter1(void const * argument);
void evtGroupSetter2(void const * argument);
void evtGroupWaiter(void const * argument);
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
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Nokia_Reset();
  Nokia_Config(hspi1.Instance);
  Clear_Display(hspi1.Instance);
  Nokia5110_SetCursor(hspi1.Instance, 0, 0);
  Nokia5110_OutString(hspi1.Instance, "Hello");
  evtGroup = xEventGroupCreate();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of lcdMutex */
  osMutexDef(lcdMutex);
  lcdMutexHandle = osMutexCreate(osMutex(lcdMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of isrSemaphore */
  osSemaphoreDef(isrSemaphore);
  isrSemaphoreHandle = osSemaphoreCreate(osSemaphore(isrSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of oneShotTimer */
  osTimerDef(oneShotTimer, Callback01);
  oneShotTimerHandle = osTimerCreate(osTimer(oneShotTimer), osTimerOnce, NULL);

  /* definition and creation of periodicTimer */
  osTimerDef(periodicTimer, Callback02);
  periodicTimerHandle = osTimerCreate(osTimer(periodicTimer), osTimerPeriodic, NULL);
  osTimerStart(periodicTimerHandle, 1000);
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  pingQueue = xQueueCreate(3, sizeof(cmd_type));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LCD_Task */
  osThreadDef(LCD_Task, LCDTask1, osPriorityAboveNormal, 0, 128);
  LCD_TaskHandle = osThreadCreate(osThread(LCD_Task), NULL);

  /* definition and creation of LED_TASK_1 */
  osThreadDef(LED_TASK_1, ledTask1, osPriorityNormal, 0, 128);
  LED_TASK_1Handle = osThreadCreate(osThread(LED_TASK_1), NULL);

  /* definition and creation of LCD_Task_2 */
  osThreadDef(LCD_Task_2, LCDTask2, osPriorityAboveNormal, 0, 128);
  LCD_Task_2Handle = osThreadCreate(osThread(LCD_Task_2), NULL);

  /* definition and creation of PING_TASK */
  osThreadDef(PING_TASK, StartTask04, osPriorityNormal, 0, 128);
  PING_TASKHandle = osThreadCreate(osThread(PING_TASK), NULL);

  /* definition and creation of PING_TASK_2 */
  osThreadDef(PING_TASK_2, StartTask05, osPriorityAboveNormal, 0, 128);
  PING_TASK_2Handle = osThreadCreate(osThread(PING_TASK_2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(EVT_GROUP_TASK_1, evtGroupSetter1, osPriorityBelowNormal, 0, 128);
  EVT_GROUP_1Handle = osThreadCreate(osThread(EVT_GROUP_TASK_1), NULL);

  osThreadDef(EVT_GROUP_TASK_2, evtGroupSetter2, osPriorityBelowNormal, 0, 128);
  EVT_GROUP_2Handle = osThreadCreate(osThread(EVT_GROUP_TASK_2), NULL);

  osThreadDef(EVT_GROUP_TASK_3, evtGroupWaiter, osPriorityBelowNormal, 0, 128);
  EVT_GROUP_3Handle = osThreadCreate(osThread(EVT_GROUP_TASK_3), NULL);

  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  __HAL_UART_ENABLE(&huart4);
  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ISR_LED_Pin|QUEUE_LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, QUEUE_LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |QUEUE_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ISR_LED_Pin QUEUE_LED2_Pin */
  GPIO_InitStruct.Pin = ISR_LED_Pin|QUEUE_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB14
                           QUEUE_LED1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |QUEUE_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void evtGroupSetter1(void const * argument)
{
  char buf[50];
  for(;;)
  {
    snprintf(buf, sizeof(buf), "Event Task Setting Bit 0: \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t  *)buf, strlen(buf), 1000);
	xEventGroupSetBits(evtGroup, 0x01);
	osDelay(1000);
  }
}
void evtGroupSetter2(void const * argument)
{
  char buf[50];
  for(;;)
  {
    snprintf(buf, sizeof(buf), "Event Task Setting Bit 1: \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t  *)buf, strlen(buf), 1000);
	xEventGroupSetBits(evtGroup, 0x02);
	osDelay(2000);
  }
}
void evtGroupWaiter(void const * argument)
{
  char buf[50];
  for(;;)
  {
    xEventGroupWaitBits(evtGroup,
      0x03,
      pdTRUE,
      pdTRUE,
	  portMAX_DELAY );
    snprintf(buf, sizeof(buf), "Event Waiting Task Unblocked \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t  *)buf, strlen(buf), 1000);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

void LCDTask2(void const * argument)
{
  for(;;)
  {
	osDelay(1000);
    xSemaphoreTake(lcdMutexHandle, portMAX_DELAY);
    Nokia5110_SetCursor(hspi1.Instance, 0, lcd_count);
    Nokia5110_OutString(hspi1.Instance, "Task 2:Hi ");
	++lcd_count;
	if(lcd_count == 7)
	{
	  Clear_Display(hspi1.Instance);
      lcd_count = 0;
	}
	xSemaphoreGive(lcdMutexHandle);
  }
}

/* USER CODE END Header_StartDefaultTask */
void LCDTask1(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(lcdMutexHandle, portMAX_DELAY);
    Nokia5110_SetCursor(hspi1.Instance, 0, lcd_count);
    Nokia5110_OutString(hspi1.Instance, "Task 1:Hello");
    ++lcd_count;
    if(lcd_count == 7)
    {
      Clear_Display(hspi1.Instance);
   	  lcd_count = 0;
    }
    xSemaphoreGive(lcdMutexHandle);
	osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ledTask1 */
/**
* @brief Function implementing the LED_TASK_1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledTask1 */
void ledTask1(void const * argument)
{
  /* USER CODE BEGIN ledTask1 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14|GPIO_PIN_1);
    osDelay(500);
  }
  /* USER CODE END ledTask1 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the LCD_Task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  cmd_type cmdToQueue = PING;
  cmd_type recvCmd;
  for(;;)
  {
    /* Should not delay becasue queue will never be full */
	xQueueSend(pingQueue, &cmdToQueue, 0);
	xQueueReceive(pingQueue, &recvCmd, portMAX_DELAY);
	if(recvCmd == PONG)
	{
      HAL_GPIO_TogglePin(GPIOA, QUEUE_LED2_Pin);
      osDelay(500);
	}
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the PING_TASK_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  cmd_type cmdToQueue = PONG;
  cmd_type recvCmd;
  /* Infinite loop */
  for(;;)
  {
	xQueueReceive(pingQueue, &recvCmd, portMAX_DELAY);
	if(recvCmd == PING)
	{
	  HAL_GPIO_TogglePin(GPIOB, QUEUE_LED1_Pin);
	  xQueueSend(pingQueue, &cmdToQueue, 0);
	  osDelay(500);
	}
	else
	{
	  xQueueSend(pingQueue, &cmdToQueue, 0);
      osDelay(500);
	}
  }
  /* USER CODE END StartTask05 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  HAL_GPIO_WritePin(GPIOA, ISR_LED_Pin, GPIO_PIN_RESET);
  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  /* USER CODE END Callback02 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
