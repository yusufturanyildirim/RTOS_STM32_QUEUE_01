/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* *** *** *** *** *** *** *** TASK HANDLE *** *** *** *** *** *** *** */
TaskHandle_t Task1_Handle;
TaskHandle_t Task2_Handle;
TaskHandle_t ReceiverTaskHandle;

/* *** *** *** *** *** *** *** QUEUE HANDLE *** *** *** *** *** ***    */
QueueHandle_t simpleQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

void TASK1_CONTROL(void *params);
void TASK2_CONTROL(void *params);
void RECEIVER_TASK(void *params);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  simpleQueue = xQueueCreate(5,sizeof(unsigned int));

  if(simpleQueue != NULL)
  {
	  char data[] = "Queue olusumu basarili";
	  HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 500);

  }
  else
  {
	  char data[] = "Queue olusumu basarisiz";
	  HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 500);
  }

  xTaskCreate(TASK1_CONTROL, "Task1", configMINIMAL_STACK_SIZE, NULL, 3, &Task1_Handle);
  xTaskCreate(TASK2_CONTROL, "Task2", configMINIMAL_STACK_SIZE, (void *)123, 2, &Task2_Handle);
  xTaskCreate(RECEIVER_TASK, "Receiver Task", configMINIMAL_STACK_SIZE, NULL, 1, &ReceiverTaskHandle);

  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void TASK1_CONTROL(void *params)
{
	int i = 567;
	uint32_t tickDelay = pdMS_TO_TICKS(2000);
	for(;;)
	{
		char *str = "Entered TASK1\nAbout to send a number to the queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
		if(xQueueSend(simpleQueue, &i, portMAX_DELAY) == pdPASS)
		{
			char *str2 = "Successfully sent the number to the queue\nLeaving from TASK1\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen(str2), HAL_MAX_DELAY);
		}
		vTaskDelay(tickDelay);
	}
}

void TASK2_CONTROL(void *params)
{
	int toSend;
	uint32_t tickDelay = pdMS_TO_TICKS(1000);
	for(;;)
	{
		toSend = (int)params;
		char *str = "Entered TASK2\nAbout to send a number to the queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);

		if(xQueueSend(simpleQueue, &toSend, portMAX_DELAY) == pdPASS)
		{
			char *str2 = "Successfully sent the number to the queue\nLeaving from TASK2\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t*)str2, strlen(str2), HAL_MAX_DELAY);
		}


		vTaskDelay(tickDelay);
	}
}

void RECEIVER_TASK(void *params)
{
	int received = 0;
	uint32_t tickDelay = pdMS_TO_TICKS(5000);
	for(;;)
	{
		char *str = "Entered RECEIVER TASK\nAbout to receive a number from the queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		if(xQueueReceive(simpleQueue, &received, portMAX_DELAY) == pdTRUE)
		{
			char str2[100];
			sprintf(str2,"Successfully RECEIVED the number %d to the queue\nLeaving RECEIVER Task\n\n",received);
			HAL_UART_Transmit(&huart2, (uint8_t*)str2, strlen(str2), HAL_MAX_DELAY);
		}
		else
		{
			char *str2 = "Error in Receiving from Queue\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen(str2), HAL_MAX_DELAY);
		}

		vTaskDelay(tickDelay);
	}
}

/*
 * If we want to send data from UART ISR
 */
//
//
//void HAL_UART_RxCpltCallback(,,,,,,,,,)
//{
//	HAL_UART_Receive_IT(&huart2, %rx_Data, 1);
//	int toSend = 123123;
//	if(rx_data == 'e')
//	{
//		/*
//		 * The HigherPriorityTaskWoken parameter must be initialized to pdFALSE as
//		 * it will get set to pdTURE inside the interrupt safe API function if a
//		 * context switch is required .
//		 */
//		if(xQueueSendToFrontFromISR(simpleQueue,&toSend,xHighPriorityTaskWoken) == pdPASS)
//		{
//			HAL_UART_Transmit(....);
//		}
//
//		/*
//		 * Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
//		 * xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
//		 * then calling portEND_SWITCHIN_ISR() will request a context switch. If
//		 * xHigherPriorityTaskWoken is still pdFALSE then calling
//		 * portEND_SWITCHING_ISR() will have no effect
//		 */
//		 portEND_SWITCHING_IST(xHigherPriorityTaskWoken);
//	}
//}
//


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
