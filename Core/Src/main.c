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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "eeprom.h"
#include "retarget.h"
#include "writeToFlash.h"
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
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId receiveTaskHandle;
osThreadId statusTaskHandle;
osThreadId updateTaskHandle;
/* USER CODE BEGIN PV */

uint8_t version = 1;
static int statusDelay = 10000;
uint8_t dataLengthOrig = 0;
uint8_t dataLengthNew = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
/*uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR] = { 'M', 'a', 't', 'e', 'u', 's', 'z', ' ', 'S',
		'a', 'l', 'a', 'm', 'o', 'n', ' ', 'm', 's', 'a', 'l', 'a', 'm', 'o',
		'n', '.', 'p', 'l' };
uint8_t VarDataTabRead[NB_OF_VAR];
uint16_t VarIndex, VarDataTmp = 0;*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartReceiveTask(void const * argument);
void StartStatusTask(void const * argument);
void StartUpdateTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define LOCATE_FUNC  __attribute__((__section__(".mysection")))

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  printf("--------F401: Starting up.....\r\n");

  // Write initial data to EEPROM
  char data[50] = "This is the original data";
  dataLengthOrig = strlen(data);

	// Fill EEPROM variables addresses
	uint16_t VirtAddVarTab[dataLengthOrig];
	uint8_t VarDataTabRead[50];
	uint8_t dataLength = strlen(data);

	HAL_Delay(2000);

		// Unlock the Flash Program Erase controller
		HAL_FLASH_Unlock();

		/* EEPROM Init */
		if (EE_Init() != EE_OK) {
			Error_Handler();
		}

		// Fill EEPROM variables addresses
		for (uint16_t i = 1; i <= dataLength; i++) {
			VirtAddVarTab[i - 1] = i;
		}

		// Store Values in EEPROM emulation
		for (uint16_t i = 0; i < dataLength; i++) {
			/* Sequence 1 */
			if ((EE_WriteVariable(VirtAddVarTab[i], data[i]))
					!= HAL_OK) {
				Error_Handler();
			}
		}

		// Read values
		for (uint16_t i = 0; i < dataLength; i++) {
			if ((EE_ReadVariable(VirtAddVarTab[i],
					&VarDataTabRead[i])) != HAL_OK) {
				Error_Handler();
			}
		}

	printf("	Startup: Successfully saved original data to EEPROM \r\n");
  //writeToFlash(huart2, data);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of receiveTask */
  osThreadDef(receiveTask, StartReceiveTask, osPriorityNormal, 0, 128);
  receiveTaskHandle = osThreadCreate(osThread(receiveTask), NULL);

  /* definition and creation of statusTask */
  osThreadDef(statusTask, StartStatusTask, osPriorityNormal, 0, 128);
  statusTaskHandle = osThreadCreate(osThread(statusTask), NULL);

  /* definition and creation of updateTask */
  osThreadDef(updateTask, StartUpdateTask, osPriorityNormal, 0, 128);
  updateTaskHandle = osThreadCreate(osThread(updateTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //vTaskSuspend(receiveTaskHandle);
  //vTaskSuspend(updateTaskHandle);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Interrupt handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t ulStatusRegister;
  // Execute when blue push button pressed
  if(GPIO_Pin == GPIO_PIN_13) {
	  printf("-- Button pressed, starting receive task \r\n");

	  //ulStatusRegister = ulReadPeripheralInterruptStatus();
	  //vClearPeripheralInterruptStatus( ulStatusRegister );
	  xTaskNotifyFromISR( receiveTaskHandle, 0x01, eSetBits, NULL );
	  //vTaskResume(receiveTaskHandle);
  } else {
      __NOP();
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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  //printf("F401: Normal operation \r\n");
	  osDelay(200);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartReceiveTask */
	char input[100];
	/* Infinite loop */
	for (;;) {
		// Wait until button is pressed
		xTaskNotifyWait( 0x00, 0xffffffff, NULL, pdMS_TO_TICKS(100000));

		// For debugging purposes disable watchdog
		//vTaskSuspend(statusTaskHandle);

		// Read the user input
		printf("\r\n 	Provide update code: \r\n");
		if (fgets(input, 10, stdin)) {
			printf("	Code received:  %s \r\n", input);
			uint16_t len = strlen(input);

			//---- CRC ----
			uint32_t data32bit[len];
			for(uint16_t i = 0; i < len; i++){
				data32bit[i] = (uint32_t) input[i];
			}

			uint32_t crcValue = HAL_CRC_Calculate(&hcrc, data32bit, len);
			printf("		crcValue: %lu \r\n", crcValue);

			// Split CRC for storage in EEPROM
			uint8_t a,b,c,d;
			a=(crcValue >> 24) & 0xFF;
			b=(crcValue >> 16) & 0xFF;
			c=(crcValue >> 8) & 0xFF;
			d=(crcValue) & 0xFF;

			input[len] = a;
			input[len+1] = b;
			input[len+2] = c;
			input[len+3] = d;

			//---- Write to EEPROM ----
			dataLengthNew = strlen(input);
			uint16_t VirtAddVarTab[dataLengthNew];

			// Unlock the Flash Program Erase controller
			HAL_FLASH_Unlock();

			// EEPROM Init
			if (EE_Init() != EE_OK) {
				Error_Handler();
			}

			// Fill EEPROM variables addresses with offset from original data
			for (uint16_t i = 1; i <= dataLengthNew; i++) {
				VirtAddVarTab[i - 1] = 100 + i;
			}

			// Store values in EEPROM emulation
			for (uint16_t i = 0; i < dataLengthNew; i++) {
				// Sequence 1
				if ((EE_WriteVariable(VirtAddVarTab[i], input[i])) != HAL_OK) {
					printf("! Error in saving update data to EEPROM \r\n");
					Error_Handler();
				}
			}
			//printf("	Update data and CRC saved on EEPROM \r\n");

			// Read values for debugging:
			uint8_t VarDataTabRead[dataLengthNew+10];
			for (uint16_t i = 0; i < dataLengthNew; i++) {
				if ((EE_ReadVariable(VirtAddVarTab[i], &VarDataTabRead[i])) != HAL_OK) {
					printf("! Error in reading update data from EEPROM \r\n");
					Error_Handler();
				}
			}
			printf("	Successfully read update data: %s \r\n", VarDataTabRead);
			uint32_t storedCrc = (((uint32_t)VarDataTabRead[dataLengthNew-4]) << 24) | (((uint32_t)VarDataTabRead[dataLengthNew-3]) << 16) | (((uint32_t)VarDataTabRead[dataLengthNew-2]) << 8) | ((uint32_t)VarDataTabRead[dataLengthNew-1]);
			printf("	crc value: %lu \r\n", storedCrc);

			//vTaskSuspend(receiveTaskHandle);
			xTaskNotifyGive(updateTaskHandle);

		} else {
			printf("! Invalid input \r\n");
		}
		//printf("-- receiveTask finished \r\n");
		osDelay(1);
	}
  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartStatusTask */
/**
* @brief Function implementing the statusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatusTask */
void StartStatusTask(void const * argument)
{
  /* USER CODE BEGIN StartStatusTask */
  int prevTime = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	  int currentTime = HAL_GetTick();
	  if(currentTime < prevTime + statusDelay) {
		  osDelay(currentTime + statusDelay - currentTime);
	  } else {
		  printf("	F401: Starting status transmission \r\n");
		  uint8_t checkSum = 2+version;
		  uint8_t tx_buff[]={1,0,1,0,version,checkSum};
		  HAL_UART_Transmit(&huart1, tx_buff, 6, 1000);
		  osDelay(10000);
	  }
  }
  /* USER CODE END StartStatusTask */
}

/* USER CODE BEGIN Header_StartUpdateTask */
/**
* @brief Function implementing the updateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateTask */
void StartUpdateTask(void const * argument)
{
  /* USER CODE BEGIN StartUpdateTask */
  /* Infinite loop */
  for(;;)
  {
	  //printf("Task Handle: %s", xTaskGetCurrentTaskHandle());
	  ulTaskNotifyTake( 0x00, pdMS_TO_TICKS(100000) );
	  osDelay(500);

	  printf("-- updateTask started \r\n");

	  HAL_FLASH_Unlock();

		/* EEPROM Init */
		if (EE_Init() != EE_OK) {
			Error_Handler();
		}

	  // Read from EEPROM
	  // Fill EEPROM variables addresses on the update data
		uint16_t VirtAddNew[dataLengthNew];
		for (uint16_t i = 1; i <= dataLengthNew; i++) {
			VirtAddNew[i - 1] = 100 + i;
		}
	  //char data[100] = "Hello, this is updated code";
	  uint8_t data[50];
	  for (uint16_t i = 0; i < dataLengthNew; i++) {
	  		if ((EE_ReadVariable(VirtAddNew[i], &data[i])) != HAL_OK) {
	  			printf("! Error reading update data \r\n");
	  			Error_Handler();
	  		}
	  }

	  printf("	Update data read from EEPROM: %s \r\n", data);

	  // Fault introduction
	  /*int faultMask = 11;
	  printf("%i \r\n", data[0]);
	  data[0] ^= faultMask;
	  printf("%i \r\n", data[0]);*/

	  //CRC

	  uint32_t origCRC = (((uint32_t)data[dataLengthNew-4]) << 24) | (((uint32_t)data[dataLengthNew-3]) << 16) | (((uint32_t)data[dataLengthNew-2]) << 8) | ((uint32_t)data[dataLengthNew-1]);
	  printf("		Orig CRC: %lu \r\n", origCRC);

	  uint32_t data32bit[dataLengthNew-4];
	  for(uint16_t i = 0; i < dataLengthNew - 4; i++){
		  data32bit[i] = (uint32_t) data[i];
	  }

	  uint32_t crcValue = HAL_CRC_Calculate(&hcrc, data32bit, dataLengthNew-4);
	  printf("		CRC: %lu \r\n", crcValue);


	  if(crcValue == origCRC){
		  printf("		CRC matches memory value \r\n");
	  } else {
		  printf("! CRC doesn't match memory, update should be cancelled \r\n");
	  }

	  // Write to EEPROM
	  // Unlock the Flash Program Erase controller
		uint16_t VirtAddOrig[dataLengthOrig];
			for (uint16_t i = 1; i <= dataLengthOrig; i++) {
				VirtAddOrig[i - 1] = i;
			}

		// Read values for debugging:
		uint8_t VarDataTabReadA[dataLengthOrig];
		for (uint16_t i = 0; i < dataLengthOrig; i++) {
			if ((EE_ReadVariable(VirtAddOrig[i], &VarDataTabReadA[i])) != HAL_OK) {
				printf("! Error in reading update data after update \r\n");
				Error_Handler();
			}
		}
		printf("	Successfully read data prior to update: %s \r\n", VarDataTabReadA);

		//---- UPDATE ----
		printf("	Start writing update data \r\n");
		// Store values in EEPROM emulation except CRC
		for (uint16_t i = 0; i < dataLengthNew - 4; i++) {
			/* Sequence 1 */
			if ((EE_WriteVariable(VirtAddOrig[i], data[i])) != HAL_OK) {
				printf("! Error in writing update data \r\n");
				Error_Handler();
			}
		}
		printf("	Update data written on EEPROM \r\n");

		// Read values for debugging:
		uint8_t VarDataTabRead[dataLengthOrig];
		for (uint16_t i = 0; i < dataLengthOrig; i++) {
			if ((EE_ReadVariable(VirtAddOrig[i], &VarDataTabRead[i])) != HAL_OK) {
				printf("! Error in reading update data after update \r\n");
				Error_Handler();
			}
		}

		printf("\r\n");
		printf("	Successfully read data after update: %s \r\n", VarDataTabRead);
		printf("\r\n");

	  version++;
	  printf("Code update successful \r\n");


	  // Resume watchdog:
	  //vTaskResume(statusTaskHandle);

	  osDelay(1);
  }
  /* USER CODE END StartUpdateTask */
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
  osDelay(1000);
  printf("HAL ERROR \r\n");
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
