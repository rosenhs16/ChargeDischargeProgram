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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "i2c-lcd.h"
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_CHANNEL_COUNT				3

uint16_t uhADCxConvertedValue[ADC_CHANNEL_COUNT];
uint16_t VoltageCell[ADC_CHANNEL_COUNT];

#define MovingAverageCount				3
#define MovingAverageBufferCount		32
#define MovingAverageBufferCountBinary	5

//moving average filter
uint32_t sumValue[MovingAverageCount];
uint16_t bufferIndex[MovingAverageCount];
uint16_t bufferFilterAdc[MovingAverageCount][MovingAverageBufferCount];

uint16_t MovingAverageResult[ADC_CHANNEL_COUNT];
/*
 *  PA5 -> 0 ->B3
 * 	PC2 -> 1 ->B1
 * 	PC3 -> 2 ->B2
 */

void InitMovingAverage(){
	for(uint16_t i=0;i<MovingAverageCount;i++){
		sumValue[i]=0;
		bufferIndex[i]=0;
		for(uint16_t j=0;j<MovingAverageBufferCount;j++){
			bufferFilterAdc[i][j]=0;
		}
	}
}
void CalculateMovingAverage(){
	for(uint16_t i=0;i<MovingAverageCount;i++){
		sumValue[i] -= bufferFilterAdc[i][bufferIndex[i]];
		bufferFilterAdc[i][bufferIndex[i]] = VoltageCell[i];
		sumValue[i] += bufferFilterAdc[i][bufferIndex[i]];
		bufferIndex[i]++;
		if(bufferIndex[i]==MovingAverageBufferCount){
			bufferIndex[i]=0;
		}
		MovingAverageResult[i]=sumValue[i]>>MovingAverageBufferCountBinary;
	}

//	sumValue1 -= bufferFilterAdc1[(bufferIndex1+1)%MovingAverageBufferCount];
//	bufferFilterAdc1[bufferIndex1] = VoltageCell[0];
//	sumValue1 += bufferFilterAdc1[bufferIndex1];
//	bufferIndex1++;
//	bufferIndex1%=MovingAverageBufferCount;
//	MovingAverageResult[0]=sumValue1>>MovingAverageBufferCountBinary;
//
//	sumValue1 -= bufferFilterAdc1[(bufferIndex1+1)%MovingAverageBufferCount];
//	bufferFilterAdc1[bufferIndex1] = VoltageCell[0];
//	sumValue1 += bufferFilterAdc1[bufferIndex1];
//	bufferIndex1++;
//	bufferIndex1%=MovingAverageBufferCount;
//	MovingAverageResult[0]=sumValue1>>MovingAverageBufferCountBinary;
//
//	sumValue1 -= bufferFilterAdc1[(bufferIndex1+1)%MovingAverageBufferCount];
//	bufferFilterAdc1[bufferIndex1] = VoltageCell[0];
//	sumValue1 += bufferFilterAdc1[bufferIndex1];
//	bufferIndex1++;
//	bufferIndex1%=MovingAverageBufferCount;
//	MovingAverageResult[0]=sumValue1>>MovingAverageBufferCountBinary;

}
uint16_t GetVoltageCell3(){				//B3
	//5.000V, 3111
	return uhADCxConvertedValue[0]*4210/2555;
}
uint16_t GetVoltageCell2(){				//B2
	//5.000V, 3131
	return uhADCxConvertedValue[2]*4210/2580;
}
uint16_t GetVoltageCell1(){				//B1
	//5.000V, 3113
	return uhADCxConvertedValue[1]*4210/2595;
}
void MeasureAllVoltage(){
	VoltageCell[0]=GetVoltageCell1();
	VoltageCell[1]=GetVoltageCell2();
	VoltageCell[2]=GetVoltageCell3();
}


void CheckCellVoltage(){
	if (MovingAverageResult[0]==4200 && MovingAverageResult[1]==4200 && MovingAverageResult[2]==4200)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}
void LcdCheckStatus(){

	if(MovingAverageResult[0]>4160 && MovingAverageResult[1]>4160 && MovingAverageResult[2]>4160){

		lcd_put_cur(0,2);
		lcd_send_string("Battery Full");
		lcd_put_cur(1,1);
		lcd_send_string("Trickle Charge");
	}
	else  {

		lcd_put_cur(0,5);
		lcd_send_string("Battery");
		lcd_put_cur(1,4);
		lcd_send_string("Charging");
	}
//
//	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)==GPIO_PIN_SET){
//
//		lcd_put_cur(0,5);
//		lcd_send_string("Battery");
//		lcd_put_cur(1,6);
//		lcd_send_string("Full");
//	}
//	else {
//
//		lcd_put_cur(0,5);
//		lcd_send_string("Battery");
//		lcd_put_cur(1,4);
//		lcd_send_string("Charging");
//	}
}
uint8_t displayState = 0;
void LcdDisplayVoltage(){
	char bufferTemp[17];
	char bufferTemp2[17];
	float decimalValue1 = (float)MovingAverageResult[0]/1000;
	float decimalValue2 = (float)MovingAverageResult[1]/1000;
	float decimalValue3 = (float)MovingAverageResult[2]/1000;
	float decimalValue4 = (float)decimalValue1+decimalValue2+decimalValue3;
	sprintf(bufferTemp,"C1=%.2f C2=%.2f",decimalValue1,decimalValue2);
	sprintf(bufferTemp2,"C3=%.2f V=%.2f",decimalValue3,decimalValue4);
	lcd_put_cur(0,0);
	lcd_send_string(bufferTemp);
	lcd_put_cur(1,0);
	lcd_send_string(bufferTemp2);
}
void CheckButton(){
	uint8_t buttonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	if(buttonState==GPIO_PIN_RESET){
		lcd_clear();
		HAL_Delay(100);
		displayState= displayState + 1;

		if(displayState>2){
			displayState = 0;
		}
	}
}
void DisplayManagement(){
	switch(displayState){
		case 0:
			LcdDisplayVoltage();
			break;
		case 1:
			LcdCheckStatus();
			break;
	}
}
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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc, uhADCxConvertedValue, ADC_CHANNEL_COUNT);
  InitMovingAverage();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  lcd_init();
  lcd_clear();

//  lcd_send_string("hello world");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MeasureAllVoltage();
	  CalculateMovingAverage();
	  CheckCellVoltage();

	  CheckButton();
	  DisplayManagement();

	  if(MovingAverageResult[0]>4200){		//B1 = CCR3 BAL_C3@R9
		  if(TIM1->CCR3 < TIM1->ARR)
		  TIM1->CCR3++;
	  }
	  if(MovingAverageResult[0]>4210){		//B1 <- if alot of overvoltage increase of PWM higher
		  if(TIM1->CCR3 < TIM1->ARR)
			  if(TIM1->CCR3 <= 1550)
			 TIM1->CCR3 = TIM1->CCR3+50;
	  }
	  if(MovingAverageResult[0]<=4190){
		  if(TIM1->CCR3>0)
		  TIM1->CCR3--;
	  }

	  if(MovingAverageResult[1]>4200){		//B2 = CCR1 BAL_C2@R19
		  if(TIM1->CCR1 < TIM1->ARR)
		    TIM1->CCR1++;
	  }
	  if(MovingAverageResult[1]>4210){		//B2 <- if alot of overvoltage increase of PWM higher
	  	  if(TIM1->CCR1 < TIM1->ARR)
	  		TIM1->CCR1 = TIM1->CCR1+50;
	  	  }
	  if(MovingAverageResult[1]<=4190){
		  if(TIM1->CCR1>0)
		    TIM1->CCR1--;
	  }

	  if(MovingAverageResult[2]>4200){		//B3 = CCR2 BAL_C1@R27
		  if(TIM1->CCR2 < TIM1->ARR)
		  TIM1->CCR2++;

	  }
	  if(MovingAverageResult[2]>4210){		//B3 <- if alot of overvoltage increase of PWM higher
	 		  if(TIM1->CCR2 < TIM1->ARR)
	 		  TIM1->CCR2 = TIM1->CCR2+50;

	 	  }
	  if(MovingAverageResult[2]<=4190){
		  if(TIM1->CCR2>0)
		  TIM1->CCR2--;
	  }
//	  if (MovingAverageResult[0]>4190 && MovingAverageResult[1]>4190 && MovingAverageResult[2]>4190)
//	  	{
//		  if(TIM1->CCR2>0)
//		  		  TIM1->CCR2--;
//		  if(TIM1->CCR1>0)
//		  		  TIM1->CCR1--;
//		  if(TIM1->CCR3>0)
//		  		  TIM1->CCR3--;
//	  	}
	  HAL_Delay(0);

//	  if(MovingAverageResult[0]>4100){		//B1 = CCR3 BAL_C3@R9
//		  TIM1->CCR3 = 18;
//	  }
//	  else{
//		  TIM1->CCR3 = 0;
//	  }
//	  if(MovingAverageResult[1]>4100){		//B2 = CCR1 BAL_C2@R19
//		  TIM1->CCR1 = 18;
//	  }
//	  else{
//		  TIM1->CCR1 = 0;
//	  }
//	  if(MovingAverageResult[2]>4100){		//B3 = CCR2 BAL_C1@R27
//		  TIM1->CCR2 = 18;
//	  }
//	  else{
//		  TIM1->CCR2 = 0;
//	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
