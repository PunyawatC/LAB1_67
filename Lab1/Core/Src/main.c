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
typedef struct _PortPin
{
	GPIO_TypeDef* PORT;
	uint16_t PIN;
}PortPin;

PortPin R[4] = { { GPIOA, GPIO_PIN_10 }, { GPIOB, GPIO_PIN_3 },
		              { GPIOB,GPIO_PIN_5 }, { GPIOB, GPIO_PIN_4 } };

PortPin L[4] = { { GPIOA, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_7 },
		               { GPIOB,GPIO_PIN_6 }, { GPIOA, GPIO_PIN_7 } };

uint16_t static_Num = 0;
uint16_t ButtonMatrix=0;
uint16_t ID[] = {6,4,3,4,0,5,0,0,0,6,7};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadMatrixButton_1Row();
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t timestamp=0;
	  if(HAL_GetTick()>=timestamp)
	  {
		  timestamp = HAL_GetTick()+10;
		  ReadMatrixButton_1Row();
		switch(static_Num)
		  {
		case 0://6
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32)
			{
				static_Num = 1;
				while(ButtonMatrix == 32){ReadMatrixButton_1Row();};
			}
			break;
		case 1://4
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 8192)
			{
				static_Num = 2;
				while(ButtonMatrix == 8192){ReadMatrixButton_1Row();};
			}
			break;
		case 2://3
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 64)
			{
				static_Num = 3;
				while(ButtonMatrix == 64){ReadMatrixButton_1Row();};
			}
			break;
		case 3://4
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num = 0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 8192)
			{
				static_Num = 4;
				while(ButtonMatrix == 8192){ReadMatrixButton_1Row();};
			}
			break;
		case 4://0
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32768)
			{
				static_Num = 5;
				while(ButtonMatrix == 32768){ReadMatrixButton_1Row();};
			}
			break;
		case 5://5
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 512)
			{
				static_Num = 6;
				while(ButtonMatrix == 512){ReadMatrixButton_1Row();};
			}
			break;
		case 6://0
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32768)
			{
				static_Num = 7;
				while(ButtonMatrix == 32768){ReadMatrixButton_1Row();};
			}
			break;
		case 7://0
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32768)
			{
				static_Num = 8;
				while(ButtonMatrix == 32768){ReadMatrixButton_1Row();};
			}
			break;
		case 8://0
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32768)
			{
				static_Num = 9;
				while(ButtonMatrix == 32768){ReadMatrixButton_1Row();};
			}
			break;
		case 9://6
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num =  0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 32)
			{
				static_Num = 10;
				while(ButtonMatrix == 32){ReadMatrixButton_1Row();};
			}
			break;
		case 10://7
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num = 0;}
			if (ButtonMatrix == 2 && static_Num > 0 ){static_Num = static_Num - 1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 4096)
			{
				static_Num = 11;
				while(ButtonMatrix == 4096){ReadMatrixButton_1Row();};
			}
			break;
		case 11:
			if (ButtonMatrix == 2 && static_Num > 0 ){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); static_Num = static_Num -1;
			while(ButtonMatrix == 2){ReadMatrixButton_1Row();}}
			if (ButtonMatrix == 1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);static_Num = 1;}
			if (ButtonMatrix == 8)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			}
			break;
		  }
	  }

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ReadMatrixButton_1Row() {
    static uint8_t X = 0;
    register int i;
    for (i = 0; i < 4; i++) {
        if (HAL_GPIO_ReadPin(L[i].PORT, L[i].PIN)) {
            ButtonMatrix &= ~(1 << (X * 4 + i));
        } else {
            ButtonMatrix |= 1 << (X * 4 + i);
        }
    }
    HAL_GPIO_WritePin(R[X].PORT, R[X].PIN, 1);
    HAL_GPIO_WritePin(R[(X + 1) % 4].PORT, R[(X + 1) % 4].PIN, 0);
    X++;
    X %= 4;
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
