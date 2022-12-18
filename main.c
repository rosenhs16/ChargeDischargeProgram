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
#include "main.h"	//declare main function

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>		//include standard I/O library
#include "i2c-lcd.h"	//include i2c lcd program
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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM15_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_CHANNEL_COUNT				3   //defining the constant ADC_CHANNEL_COUNT

uint16_t uhADCxConvertedValue[ADC_CHANNEL_COUNT];
uint16_t VoltageCell[ADC_CHANNEL_COUNT];

/*Setting up Moving Average Filter*/

#define MovingAverageCount				3
#define MovingAverageBufferCount		64  	// sample count
#define MovingAverageBufferCountBinary	6  		// divider in bits
uint8_t  mode;
uint8_t  buttonState;

//moving average filter
uint32_t sumValue[MovingAverageCount];
uint16_t bufferIndex[MovingAverageCount];
uint16_t bufferFilterAdc[MovingAverageCount][MovingAverageBufferCount];

uint16_t MovingAverageResult[ADC_CHANNEL_COUNT];

uint8_t batteryDetectedFlag; 					//0 if battery not detected, 1 if battery detected.
uint16_t batteryDetectedTimeCount;
/* PA0 = A0 = IN0 = sense voltage
 * PA1 = A1 = IN1 = charging current sense
 * PA6 = D12 = IN5 = discharging current sense
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
}
uint16_t GetVoltage1(){							//PA0, voltage sense

	return uhADCxConvertedValue[0]*4270/892;	//4.27V= ADC value 892
}
uint16_t GetVoltage2(){							//PA1, charging current sense
	//
	return uhADCxConvertedValue[1]*3030/3580;	//3.03A = ADC value 3580
}
uint16_t GetVoltage3(){							//PA6, discharging current sense
	//
	return uhADCxConvertedValue[2]*1000/1900;	//1A = ADC value 1970
}
void MeasureAllVoltage(){
	VoltageCell[0]=GetVoltage1();
	VoltageCell[1]=GetVoltage2();
	VoltageCell[2]=GetVoltage3();

}
void LoadFunction(){
	if(MovingAverageResult[2]<1000){
		if(TIM3->CCR1 < TIM3->ARR)
			TIM3->CCR1++;
	if(MovingAverageResult[2]>1000)
		TIM3->CCR1--;
}
	LcdDisplayDischargeCurrent();
}
void ChargeFunction() {
		  if(MovingAverageResult[0]<1000){				//if voltage < 1V, turn off pwm
			  if(MovingAverageResult[1]<200){			//if current < 200mA, turn off pwm
			  	  	 TIM15->CCR1=0;
			  	  	 }
		  }
	if (batteryDetectedFlag) {
		//* Li-Ion Charging Algorithm *//
		/*CC Mode*/
		if (MovingAverageResult[0] < 9000) {
			//if voltage detected<10V, li-ion charge
			if (TIM15->CCR1 < TIM15->ARR - 1400)
				//set D to 41%
				if (MovingAverageResult[0] < 4000){
					if (MovingAverageResult[1] < 1000)
						TIM15->CCR1++;
			//if current sense<2A, increase D
					if (MovingAverageResult[1] > 1000)
						TIM15->CCR1--;
			//if current sense>2A, decrease D
					if (MovingAverageResult[1] == 1000)
						TIM15->CCR1 + 0;
						}
			//if current sense=2A, let it stay there}
			/*CV Mode*/
			if (MovingAverageResult[0] >= 4880) {
				//if v>4V,
				if (MovingAverageResult[0] < 5000)
					//but less than 5V
					//				 if(MovingAverageResult[0]<=4310)
					TIM15->CCR1 = 440;
			      }
			LcdDisplayVoltageLiion();
			}
			//* SLA Charging Algorithm *//
		if (MovingAverageResult[0] > 9000) {
				//V>9V
				if (MovingAverageResult[0] < 14778)
					//V<15V
		//			if (TIM15->CCR1 < TIM15->ARR - 936)
						//set D to 60%
						TIM15->CCR1 = 1450;
				//charge with 15V}
				LcdDisplayVoltageSLA();

			}

	}

	if (MovingAverageResult[1] == 0 &&
			MovingAverageResult[0] < 2000) {
		TIM15->CCR1 = 0; //set output to 0
		batteryDetectedTimeCount = 500; //give 500 ms wait time
		batteryDetectedFlag = 0; //to get out from charge function
			} else {
		if (batteryDetectedTimeCount) {
			batteryDetectedTimeCount--;
		} else {
			if (MovingAverageResult[0] > 2000
					&& MovingAverageResult[1] < 100) {
				//if V sense >3000
				//charge
				batteryDetectedFlag = 1; //start charge
			}
		}
	}
}
uint8_t displayState = 0;
void LcdDisplayVoltageSLA(){
	char bufferTemp[17];
	char bufferTemp2[17];
	float decimalValue1 = (float)MovingAverageResult[0]/1000;
	float decimalValue2 = (float)MovingAverageResult[1]/1000;
	sprintf(bufferTemp,"SLA Battery"); //put c1, c2 values in buffertemp, 2f = 2float = 2decimal points
	sprintf(bufferTemp2,"V=%.2fV A=%.2fA",decimalValue1,decimalValue2); //put c3, v calues in buffertemp2
	lcd_put_cur(0,3);
	lcd_send_string(bufferTemp);
	lcd_put_cur(1,0);
	lcd_send_string(bufferTemp2); //buffertemp2 is at (1,0)
}
void LcdDisplayVoltageLiion(){
	char bufferTemp[17];
	char bufferTemp2[17];
	float decimalValue1 = (float)MovingAverageResult[0]/1000;
	float decimalValue2 = (float)MovingAverageResult[1]/1000;
	sprintf(bufferTemp,"LI-ION Battery"); //put c1, c2 values in buffertemp, 2f = 2float = 2decimal points
	sprintf(bufferTemp2,"V=%.2fV A=%.2fA",decimalValue1,decimalValue2); //put c3, v calues in buffertemp2
	lcd_put_cur(0,1);
	lcd_send_string(bufferTemp);
	lcd_put_cur(1,0);
	lcd_send_string(bufferTemp2); //buffertemp2 is at (1,0)
}
void LcdDisplayDischargeCurrent(){
	char bufferTemp[17];
	char bufferTemp2[17];
	float decimalValue1 = (float)MovingAverageResult[0]/1000;
	float decimalValue2 = (float)MovingAverageResult[1]/1000;
	float decimalValue3 = (float)MovingAverageResult[2]/1000;
	sprintf(bufferTemp,"DischargeCurrent"); //put c1, c2 values in buffertemp, 2f = 2float = 2decimal points
	sprintf(bufferTemp2,"V=%.2fV A=%.2fA",decimalValue1,decimalValue3); //put c3, v calues in buffertemp2
	lcd_put_cur(0,0);
	lcd_send_string(bufferTemp);
	lcd_put_cur(1,0);
	lcd_send_string(bufferTemp2); //buffertemp2 is at (1,0)
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
  MX_TIM15_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); 			//pb14 highside
  	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);		//pb15 lowside
 // 	TIM15->CCR1=1450;
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);			//pa8
 //  	TIM3->CCR1=1500;
  	HAL_ADC_Start_DMA(&hadc, uhADCxConvertedValue, ADC_CHANNEL_COUNT);
    InitMovingAverage();
    batteryDetectedTimeCount=0;

    lcd_init();
    lcd_clear();
    lcd_put_cur(0,1);
    lcd_send_string("Rose - Charger");
    lcd_put_cur(1,2);
    lcd_send_string("& Load Tester");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MeasureAllVoltage();
	  CalculateMovingAverage();
	  //HAL_Delay(1);

	  switch(mode){
	  	  case 0: break;
	  	  case 1: ChargeFunction();break;
	  	  case 2: break;
	  	  case 3: LoadFunction();break;
	  }


		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET){
			buttonState = 1;
			lcd_clear();
		}
		else{
			if(buttonState == 1){
				buttonState=0;
				mode++;
				if(mode==4)
					mode=0;
				switch(mode){
				case 0:
					TIM3->CCR1=0;
					lcd_clear();
					lcd_put_cur(0,5);
					lcd_send_string("Charge");
					lcd_put_cur(1,6);
					lcd_send_string("Mode");
					break;

				case 2://shutdown charger
					TIM15->CCR1 = 0;
					lcd_clear();
					lcd_put_cur(0,4);
					lcd_send_string("Discharge");
					lcd_put_cur(1,6);
					lcd_send_string("Mode");
					break;
				}
			}
		}
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
