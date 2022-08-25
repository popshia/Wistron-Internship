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
#include "stdint.h"
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
 TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xffff);
	return ch;
} // overwrite putchar prototype for printf

void Delay(uint16_t);
void Set_SDA_In(void);
void Set_SDA_Out(void);
void SCL_1(void);
void SCL_0(void);
void SDA_1(void);
void SDA_0(void);
uint8_t SDA_Read(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Send_Byte(uint8_t data);
uint8_t I2C_Read_Byte();
uint8_t I2C_Wait_Ack(void);
void I2C_Send_Ack(void);
void I2C_Send_NAck(void);
void I2C_Write_Data(uint8_t address[3], uint8_t data[], int datasize);
void I2C_Read_Data(uint8_t address[3], uint8_t *outputData, int datasize);
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
	uint8_t address[3] = { 0x57, 0x00, 0x00 };
	// array to store slave address and memory address
	uint8_t inputData[8] = "Wistron.";
	// input data array
	uint8_t outputData[8] = { 0 };
	// output data array
	int datasize = sizeof(inputData) / sizeof(inputData[0]);
	// how many bit to read and write
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	I2C_Write_Data(address, inputData, datasize);
	HAL_Delay(100);
	I2C_Read_Data(address, outputData, datasize);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin|SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Delay(uint16_t delayTime) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delayTime) {
	}
} // delay in microseconds

void Set_SDA_In() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/*Configure GPIO pins : SDA_Pin */
	GPIO_InitStruct.Pin = SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
} // set SDA pin as input

void Set_SDA_Out() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/*Configure GPIO pins : SDA_Pin */
	GPIO_InitStruct.Pin = SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
} // set SDA pin as output

void SCL_1() {
	HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, SET);
} // scl high

void SCL_0() {
	HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, RESET);
} // scl low

void SDA_1() {
	HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, SET);
} // sda high

void SDA_0() {
	HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, RESET);
} // sda low

uint8_t SDA_Read() {
	return HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
} // read one byte of data from sda

void I2C_Start() {
	Set_SDA_Out();
	SDA_1();
	SCL_1();
	Delay(4);
	SDA_0();
	Delay(4);
	SCL_0();
} // start i2c transmission

void I2C_Stop() {
	Set_SDA_Out();
	SCL_0();
	SDA_0();
	Delay(4);
	SCL_1();
	SDA_1();
	Delay(4);
} // stop i2c transmission

uint8_t I2C_Wait_Ack() {
	uint8_t errorTime = 0;
	Set_SDA_In();
	SDA_1();
	Delay(1);
	SCL_1();
	Delay(1);

	while (SDA_Read()) {
		errorTime++;

		if (errorTime > 250) {
			I2C_Stop();
			return 1;
		} // if success tranfer data
	}

	SCL_0();
	return 0;
} // wait for ack after reading data from sda

void I2C_Send_Ack() {
	SCL_0();
	Set_SDA_Out();
	SDA_0();
	Delay(4);
	SCL_1();
	Delay(2);
	SCL_0();
} // send ack

void I2C_Send_NAck() {
	SCL_0();
	Set_SDA_Out();
	SDA_1();
	Delay(2);
	SCL_1();
	Delay(2);
	SCL_0();
} // send nack

void I2C_Send_Byte(uint8_t data) {
	Set_SDA_Out();
	SCL_0(); // pull scl down

	for (int i = 0; i < 8; i++) {
		if (data & 0x80) {
			SDA_1();
		} // if 1
		else {
			SDA_0();
		} // else 0

		data <<= 1;
		Delay(2);
		SCL_1(); // keep the clock ticking
		Delay(2);
		SCL_0(); // keep the clock ticking
		Delay(2);
	} // loop 8 bits
} // i2c send one byte of data

uint8_t I2C_Read_Byte() {
	int i = 0;
	uint8_t readbyte = 0;
	Set_SDA_In();

	for (i = 0; i < 8; i++) {
		SCL_0();
		Delay(2);
		SCL_1();
		readbyte <<= 1;

		if (SDA_Read()) {
			readbyte++;
		} // if read data

		Delay(1);
	} // read 8 bits

	if (i == 8) {
		I2C_Send_Ack();
	} // successful read data
	else {
		I2C_Send_NAck();
	} // unsuccessful

	return readbyte;
} // read byte data

void I2C_Write_Data(uint8_t address[3], uint8_t data[], int dataSize) {
	I2C_Start();

	for (int i = 0; i < 3; i++) {
		if (i == 0) {
			I2C_Send_Byte(address[i] << 1);
		} // slave address
		else {
			I2C_Send_Byte(address[i]);
		} // memory addree

		I2C_Wait_Ack();
	} // loop address

	for (int i = 0; i < dataSize; i++) {
		I2C_Send_Byte(data[i]);
		I2C_Wait_Ack();
	} // loop data

	I2C_Stop();
	Delay(10);
} // i2c write data

void I2C_Read_Data(uint8_t address[3], uint8_t *outputData, int datasize) {
	I2C_Start();

	for (int i = 0; i < 3; i++) {
		if (i == 0) {
			I2C_Send_Byte(address[i] << 1);
		} // slave address
		else {
			I2C_Send_Byte(address[i]);
		} // memory addree

		I2C_Wait_Ack();
	} // loop address

	I2C_Start();
	I2C_Send_Byte((address[0] << 1) + 1);
	I2C_Wait_Ack();

	for (int i = 0; i < datasize; i++) {
		outputData[i] = I2C_Read_Byte();
	} // read data from eeprom

	I2C_Stop();
} // i2c read data

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
