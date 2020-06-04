/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;	//PWM
UART_HandleTypeDef huart1;	//HC-05	
UART_HandleTypeDef huart2;	//HOST-BOARD CONNECTION
DMA_HandleTypeDef hdma_usart1_rx;	//UART BUFFER OF BT

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

char txData[30];
uint8_t rxData[30];
// bool led_state = false;
int speed = 0;

void setSpeed(uint8_t left, uint8_t right) {
	htim1.Instance->CCR1 = left;
	htim1.Instance->CCR2 = right;
}

void Forward(int pwm) {

	//M1 Directions
	HAL_GPIO_WritePin(M1_dir2_GPIO_Port, M1_dir2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, M1_dir1_Pin, GPIO_PIN_SET);

	//M2 Directions
	HAL_GPIO_WritePin(GPIOB, M2_dir2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, M2_dir1_Pin, GPIO_PIN_SET);

	setSpeed(pwm, pwm);
}

void Backward(int pwm) {
	//M1 Directions
	HAL_GPIO_WritePin(M1_dir2_GPIO_Port, M1_dir2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M1_dir1_Pin, GPIO_PIN_RESET);

	//M2 Directions
	HAL_GPIO_WritePin(GPIOB, M2_dir2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M2_dir1_Pin, GPIO_PIN_RESET);

	setSpeed(pwm, pwm);
}

void Right(int pwm) {
	//M1 Directions
	HAL_GPIO_WritePin(M1_dir2_GPIO_Port, M1_dir2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M1_dir1_Pin, GPIO_PIN_RESET);

	//M2 Directions
	HAL_GPIO_WritePin(GPIOB, M2_dir2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, M2_dir1_Pin, GPIO_PIN_SET);

	setSpeed(pwm, pwm);
	HAL_Delay(300);
	setSpeed(0, 0);
}

void Left(int pwm) {
	//M1 Directions
	HAL_GPIO_WritePin(M1_dir2_GPIO_Port, M1_dir2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, M1_dir1_Pin, GPIO_PIN_SET);

	//M2 Directions
	HAL_GPIO_WritePin(GPIOB, M2_dir2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M2_dir1_Pin, GPIO_PIN_RESET);

	setSpeed(pwm, pwm);
	HAL_Delay(300);
	setSpeed(0, 0);	//STOP
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	speed = 255;	//FULL SPEED
	//Initialize all PWM channels
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	printf("BT Test! \r\n");
	while (1) {
		if (HAL_UART_Receive(&huart1, (uint8_t*) rxData, 30, 500)) {

			if (rxData[0] == 'f' && rxData[1] == 'o' && rxData[2] == 'r') {

				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Forward\n"), 500);

				Forward(speed);
				printf("Forward \n\r");

			} else if (rxData[0] == 'b' && rxData[1] == 'a' && rxData[2] == 'c'
					&& rxData[3] == 'k') {

				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Backward\n"), 500);

				Backward(speed);
				printf("Backward\n\r");

			} else if (rxData[0] == 'l' && rxData[1] == 'e' && rxData[2] == 'f'
					&& rxData[3] == 't') {

				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Left\n"), 500);

				Left(speed);
				for (int i = 0; i < 30; i++)	//clear the  buffer
					rxData[i] = '\0';
				printf("Left\n\r");

			} else if (rxData[0] == 'r' && rxData[1] == 'i' && rxData[2] == 'g'
					&& rxData[3] == 'h' && rxData[4] == 't') {

				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Right\n"), 500);

				Right(speed);
				printf("Right\n\r");
				for (int i = 0; i < 30; i++)
					rxData[i] = '\0';

			} else if (rxData[0] == 's' && rxData[1] == 't' && rxData[2] == 'o'
					&& rxData[3] == 'p') {
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Stop\n"), 500);

				setSpeed(0, 0);
				printf("Stop\n\r");
				for (int i = 0; i < 30; i++)
					rxData[i] = '\0';

			} else if (atoi((uint8_t*) rxData)) {
				speed = atoi((uint8_t*) rxData);

				if (speed >= 255) //higher threshold
					speed = 255;
				else if (speed <= 215)	//lower threshold
					speed = 215;

				setSpeed(speed, speed);
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Change Speed: %i \n\r", speed), 500);
				printf("Change Speed: %i \n\r", speed);
				for (int i = 0; i < 30; i++)
					rxData[i] = '\0';
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 255;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, M2_dir1_Pin | M1_dir1_Pin | M2_dir2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(M1_dir2_GPIO_Port, M1_dir2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : M2_dir1_Pin M1_dir1_Pin M2_dir2_Pin */
	GPIO_InitStruct.Pin = M2_dir1_Pin | M1_dir1_Pin | M2_dir2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : M1_dir2_Pin */
	GPIO_InitStruct.Pin = M1_dir2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(M1_dir2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
