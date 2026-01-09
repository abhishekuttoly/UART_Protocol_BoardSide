/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "UART_Packet_Builder.h"
#include "UART_Packet_Parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __sPollingParameters {
	uint8_t ucParamId;			// Parameter id
	uint16_t unPollingInterval;		// Parameter polling interval
	uint32_t ulLastPollingTime;		// Last polling time
} _sPollingParameters;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_SIZE 10
#define MAX_LENGTH 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct __sQueue {
	uint8_t *pBuffer[QUEUE_SIZE][MAX_LENGTH];
	uint8_t ucLength[QUEUE_SIZE];
	uint8_t ucFront;
	uint8_t ucBack;
	uint8_t ucCount;
} _sQueue;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for CreatePacket */
osThreadId_t CreatePacketHandle;
const osThreadAttr_t CreatePacket_attributes = {
  .name = "CreatePacket",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
_sQueue pBufferQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void PollingTask(void *argument);
void CommunicationTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CreatePacket */
  CreatePacketHandle = osThreadNew(PollingTask, NULL, &CreatePacket_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(CommunicationTask, NULL, &UARTTask_attributes);

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
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 9600;
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
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void InitQueue(_sQueue *pQueue) {
	pQueue->ucFront = 0;
	pQueue->ucBack = 0;
}

bool IsEmpty(_sQueue *pQueue)
{
	if (pQueue->ucCount == 0) {
		return true;
	}
	return false;
}

bool IsFull(_sQueue *pQueue)
{
	if (pQueue->ucCount == QUEUE_SIZE) {
		return true;
	}
	return false;
}

bool PushBufferToQueue(_sQueue *pQueue, uint8_t ucPushBuffer[], uint8_t ucLength)
{
	if (IsFull(pQueue)) {
		return false;
	}
	uint8_t ucIndex;
	for (ucIndex = 0; ucIndex < ucLength; ucIndex++) {
		pQueue->pBuffer[pQueue->ucBack][ucIndex] = ucPushBuffer[ucIndex];
	}

	pQueue->ucLength[pQueue->ucBack] = ucLength;
	pQueue->ucBack = (pQueue->ucBack + 1) % QUEUE_SIZE;
	pQueue->ucCount++;

	return true;
}

bool PopFromQueue(_sQueue *pQueue, uint8_t ucPopBuffer[], uint8_t *ucLength)
{
	*ucLength = pQueue->ucLength[pQueue->ucFront];
	uint8_t ucIndex;

	for (ucIndex = 0; ucIndex < *ucLength; ucIndex++) {
		ucPopBuffer[ucIndex] = pQueue->pBuffer[pQueue->ucFront][ucIndex];
	}

	pQueue->ucFront = (pQueue->ucFront + 1) % QUEUE_SIZE;
	pQueue->ucCount--;

	return true;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_PollingTask */
/**
 * @brief  Function implementing the CreatePacket thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_PollingTask */
void PollingTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	_sPollingParameters	sVersionParams		= {1, 60, 0};
	_sPollingParameters	sDeviceIdParams		= { 2, 300, 0};
	_sPollingParameters	sDeviceNameParams	= { 3, 180, 0};

	TickType_t xCurrentTime;

	InitQueue(&pBufferQueue);
	bool bSet = true;	//variable used to set param testing
	bool bSetPC = true;

	/* Infinite loop */
	for (;;) {
		xCurrentTime = xTaskGetTickCount();
		uint32_t unTimeinSeconds = xCurrentTime / configTICK_RATE_HZ;

//		//set command testing code
		if(bSet && unTimeinSeconds >= 330){
		uint8_t ucParamId = 3;
		uint8_t ucParameterBuffer[] = {"SET"};
		uint8_t ucParamLength = strlen(ucParameterBuffer);
		uint8_t ucTLVSetPacketBuffer[64] = {0};
		CreateSetPacket(ucParamId, ucParameterBuffer,ucParamLength, ucTLVSetPacketBuffer);
		PushBufferToQueue(&pBufferQueue
				, ucTLVSetPacketBuffer, ucTLVSetPacketBuffer[1]+2);
		bSet = false;
		}

		if (bSetPC && unTimeinSeconds >= 600) {
			uint8_t ucParamId = 3;
			uint8_t ucParameterBuffer[] = { "PC" };
			uint8_t ucParamLength = strlen(ucParameterBuffer);
			uint8_t ucTLVSetPacketBuffer[64] = { 0 };
			CreateSetPacket(ucParamId, ucParameterBuffer, ucParamLength,
					ucTLVSetPacketBuffer);
			PushBufferToQueue(&pBufferQueue, ucTLVSetPacketBuffer,
					ucTLVSetPacketBuffer[1] + 2);
			bSetPC = false;
		}


		if (sVersionParams.ulLastPollingTime == 0
				&& unTimeinSeconds > sVersionParams.unPollingInterval) {
			uint8_t ucVersionGetPacketBuffer[64];
			CreateGetPacket(sVersionParams.ucParamId, ucVersionGetPacketBuffer);
			PushBufferToQueue(&pBufferQueue, ucVersionGetPacketBuffer, 5);
			sVersionParams.ulLastPollingTime = unTimeinSeconds;
		}

		if ((unTimeinSeconds - sDeviceIdParams.ulLastPollingTime)
				>= sDeviceIdParams.unPollingInterval) {
			uint8_t ucDeviceIdGetPacketBuffer[64];
			CreateGetPacket(sDeviceIdParams.ucParamId, ucDeviceIdGetPacketBuffer);
			PushBufferToQueue(&pBufferQueue, ucDeviceIdGetPacketBuffer, 5);
			sDeviceIdParams.ulLastPollingTime = unTimeinSeconds;
		}

		if ((unTimeinSeconds - sDeviceNameParams.ulLastPollingTime)
				>= sDeviceNameParams.unPollingInterval) {
			uint8_t ucDeviceNameGetPacketBuffer[64];
			CreateGetPacket(sDeviceNameParams.ucParamId,
					ucDeviceNameGetPacketBuffer);
			PushBufferToQueue(&pBufferQueue, ucDeviceNameGetPacketBuffer, 5);
			sDeviceNameParams.ulLastPollingTime = unTimeinSeconds;
			}
		osDelay(5000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CommunicationTask */
/**
 * @brief Function implementing the UARTTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CommunicationTask */
void CommunicationTask(void *argument)
{
  /* USER CODE BEGIN CommunicationTask */
	uint8_t ucRxBuffer[MAX_LENGTH] = { 0 };
	/* Infinite loop */
	for (;;) {
		if (!IsEmpty(&pBufferQueue)) {
			uint8_t ucTxBuffer[MAX_LENGTH];
			uint8_t ucLength = 0;

			if (!PopFromQueue(&pBufferQueue, &ucTxBuffer, &ucLength)) {
				printf("ERROR: %d Failed to pop buffer from queue\n", __LINE__);
				fflush(stdout);
			}
			HAL_StatusTypeDef eTransmitStatus = HAL_UART_Transmit(&huart3,
					ucTxBuffer, ucLength, portMAX_DELAY);

			if (eTransmitStatus != HAL_OK) {
				printf("ERROR: %d UART transmit failed\n", __LINE__);
				fflush(stdout);
			}

			HAL_UART_AbortReceive(&huart3);
			HAL_Delay(10);
			memset(ucRxBuffer,0xAA, sizeof(ucRxBuffer));
			HAL_Delay(2);

			HAL_StatusTypeDef eReceiveStatus = HAL_UART_Receive(&huart3,
					ucRxBuffer, sizeof(ucRxBuffer), portMAX_DELAY);

			if (eReceiveStatus == HAL_OK) {
				_sPacketData sDataPacket;
				printf("Received Data: ");
				for (int i = 0; i < 12; i++) {
					printf("%02x ", ucRxBuffer[i]);
				}
				printf("\n");
				fflush(stdout);

				if (ParsePacket(ucRxBuffer, &sDataPacket))
				{
					if (sDataPacket.ucRequestType == 1)
					{
						if (sDataPacket.ucNumberOfTLVs == 0)
						{
							printf("ERROR: %d Invalid packet\n", __LINE__);
							fflush(stdout);
						}
						else
						{
							uint8_t ucIndex;
							uint8_t ucTotalTLVs = sDataPacket.ucNumberOfTLVs;

							for (ucIndex = 0; ucIndex < sDataPacket.ucNumberOfTLVs; ucIndex += 2)
							{
								uint8_t ucParameter = sDataPacket.psTlv[ucIndex+ 1].psTlvParam.ucType;

								if (ucParameter == TYPE_CHAR)
								{
									uint8_t ucCharValue[64];
									memcpy(ucCharValue,
											sDataPacket.psTlv[ucIndex + 1].psTlvParam.ucValueBuffer,
											sDataPacket.psTlv[ucIndex + 1].psTlvParam.ucLength);

									printf("\nResponse value: ");
									for (int i = 0; i < sDataPacket.psTlv[ucIndex + 1].psTlvParam.ucLength; i++)
									{
										printf("%c", ucCharValue[i]);
										fflush(stdout);
									}
									printf("\n");
								}
								else
								{
									uint16_t unIntValue;
									memcpy(&unIntValue,
											sDataPacket.psTlv[ucIndex + 1].psTlvParam.ucValueBuffer,
											sDataPacket.psTlv[ucIndex + 1].psTlvParam.ucLength);
									printf("Response value = %d\n", unIntValue);
									fflush(stdout);
								}
							}
						}
					}

					if (sDataPacket.ucRequestType == 2) {
						printf("Set success\n");
						fflush(stdout);
					}
				}
				else
				{
					printf("parse failed\n");
					fflush(stdout);
				}
			}
			else
			{
				printf("ERROR: %d Receive timeout\n");
				fflush(stdout);
			}
		}
		osDelay(1000);
	}
  /* USER CODE END CommunicationTask */
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
	while (1) {
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
