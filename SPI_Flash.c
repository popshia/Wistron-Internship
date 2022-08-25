/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t READ = 0b00000011;
const uint8_t WRITE = 0b00000010;
const uint8_t WRITE_DISABLE = 0b00000100;
const uint8_t WRITE_ENABLE = 0b00000110;
const uint8_t READ_STATUS_REGISTER = 0b00000101;
const uint8_t WRITE_STATUS_REGISTER = 0b00000001;
const uint8_t ERASE = 0b00100000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SPI_Write_Enable();
void SPI_Check();
void SPI_Erase(uint8_t address[]);
void SPI_Write(uint8_t address[], char input[]);
void SPI_Read(uint8_t address[], char output[]);
void SPI_Wait();

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xffff);
	return ch;
} // overwrite putchar prototype for printf

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint8_t address[3] = { 0x00, 0x00, 0x00 };
	char input[8] = "Wistron!";
	char output[8] = { 0 };
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	SPI_Write_Enable();
	SPI_Check();
	SPI_Erase(address);
	SPI_Wait();
	SPI_Read(address, output);
	SPI_Wait();
	SPI_Write_Enable();
	SPI_Check();
	SPI_Write(address, input);
	SPI_Wait();
	SPI_Read(address, output);

	/* USER CODE END 2 */

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SPI_Write_Enable() {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET); // pull the CS pin low
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WRITE_ENABLE, 1, 100); // send mode
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET); // pull the CS pin high
} // enable spi write

void SPI_Check() {
	char statusRegister[1];

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET); // pull the CS pin low
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &READ_STATUS_REGISTER, 1, 100); // send mode
	HAL_SPI_Receive(&hspi1, (uint8_t*) statusRegister, 1, 100); // recieve mode
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET); // pull the CS pin high

	printf("Current status: 0x%x\r\n", (unsigned int) statusRegister[0]);
} // check spi status register

void SPI_Erase(uint8_t address[]) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET); // pull the CS pin low
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &ERASE, 1, 100); // write data to register
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &address, 3, 100); // write data to register
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET); // pull the CS pin high
} // erase block

void SPI_Write(uint8_t address[], char input[8]) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET); // pull the CS pin low
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WRITE, 1, 100); // write data to register
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &address, 3, 100); // write data to register
	HAL_SPI_Transmit(&hspi1, (uint8_t*) input, 8, 100); // write data to register
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET); // pull the CS pin high
} // write data to spi

void SPI_Read(uint8_t address[], char output[8]) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);  // pull the pin low
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &READ, 1, 100); // write data to register
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &address, 3, 100);  // send address
	HAL_SPI_Receive(&hspi1, (uint8_t*) output, 8, 100); // receive 6 bytes data
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);  // pull the pin high
} // read data from spi

void SPI_Wait() {
	char statusRegister[1];
	uint8_t wip = 1;

	while (wip) {
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET); // pull the CS pin low
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &READ_STATUS_REGISTER, 1, 100); // send mode
		HAL_SPI_Receive(&hspi1, (uint8_t*) statusRegister, 1, 100); // recieve mode
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET); // pull the CS pin high

		wip = statusRegister[0] & 0b00000001;
	} // loop wip
} // wait for wip bits
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
