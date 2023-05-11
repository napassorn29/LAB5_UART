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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"

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
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

int t_frequancy = 5;
float millisec;

uint8_t RxBuffer[100];
uint8_t TxBuffer[500];
uint32_t timestamp = 0;

int statePin;
int textState;
uint8_t state = 'x';
int condition = 1;
int openclose = 1;
int button;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void DummyTask();
void UARTDMAConfig();
void statePress();
void StateOpenLED(button);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t text[] = "if Press 0 : LED Control\r\nif Press 1 : Button Status   \r\n";
  HAL_UART_Transmit(&huart2, text, 58, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  statePin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if (openclose == 1)
	  {
		  DummyTask();
	  }
	  else if (openclose == 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  huart2.Init.BaudRate = 460800;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

}

/* USER CODE BEGIN 4 */

void UARTDMAConfig()
{
	//start UART in DMA Mode
	HAL_UART_Receive_DMA(&huart2, RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		RxBuffer[1] = '\0';
		if(RxBuffer[0]=='x')button = 'x';
		else if(RxBuffer[0]=='0')button = '0';
		else if(RxBuffer[0]=='1')button = '1';
		else if(RxBuffer[0]=='a')button = 'a';
		else if(RxBuffer[0]=='s')button = 's';
		else if(RxBuffer[0]=='d')
		{
			button = 'd';
			if (openclose == 1)
			{
				openclose = 0;
			}
			else if (openclose == 0)
			{
				openclose = 1;
			}
		}
		else button = 'z';

		StateOpenLED(button);
	}
}

void StateOpenLED(state)
{
	switch(state)
	{
		case 'x':
		{
			if (condition == 2 || condition == 1 || condition == 3)
			{
			sprintf((char*)TxBuffer, "if Press 0 : LED Control\r\nif Press 1 : Button Status \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 1;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case '0':
		{
			if (condition == 1)
			{
			sprintf((char*)TxBuffer, "  ===== LED Control =====\r\nPress a : speed up +1 Hz \r\nPress s : speed down -1 Hz \r\nPress d : On/off \r\nPress x : back \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case '1':
		{
			if(condition == 1 || condition == 3 && statePin == 1)
			{
			sprintf((char*)TxBuffer, "  ===== Button Status =====\r\nstate of blue button : unpress \r\nif Press x :back \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 3;
			}
			else if(condition == 1 || condition == 3 && statePin == 0)
			{
			sprintf((char*)TxBuffer, "state of blue button : press \r\nif Press x :back \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 3;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case 'z':
		{
			if (condition == 1 ||condition == 0 ||condition == 2 ||condition == 3 )
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case 'a':
		{
			if(condition == 2)
			{
			t_frequancy = t_frequancy + 1;
			sprintf((char*)TxBuffer, "frequancy : %d Hz \r\n-----------------------\r\n", t_frequancy);
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case 's':
		{
			if(condition == 2 && t_frequancy >= 1)
			{
			t_frequancy = t_frequancy - 1;
			sprintf((char*)TxBuffer, "frequancy : %d Hz \r\n-----------------------\r\n", t_frequancy);
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else if(condition == 2 && t_frequancy == 0)
			{
			t_frequancy = 0;
			sprintf((char*)TxBuffer, "frequancy less than 0 Hz\r\nplease press a for speed up +1 Hz\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;

		case 'd':
		{
			if(condition == 2 && openclose == 1)
			{
			sprintf((char*)TxBuffer, "open LED \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else if(condition == 2 && openclose == 0)
			{
			sprintf((char*)TxBuffer, "close LED \r\nyou can press d for open LED \r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
			else
			{
			sprintf((char*)TxBuffer, "Press something again please\r\nplease press x for back\r\n-----------------------\r\n");
			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			condition = 2;
			}
		}
		break;
	}
}

void DummyTask()
{
	static uint32_t timestamp = 0;
	if(HAL_GetTick()>=timestamp)
	{
		millisec = (1.0/t_frequancy)*1000;
		timestamp = HAL_GetTick()+ millisec;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

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
