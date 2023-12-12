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
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM	TIM5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float speedOfSound = 0.0343/2;
float distance1;
float distance2;
float distance3;
float distance4;
uint8_t uartBuf[100];
uint8_t uartBuf2[100];
static uint8_t GAIN;
int32_t hx711_value;
char uartData[50];
//char uartBuf[100];
int weight;
int status = 0;
int loop = 0;
int rev = 0;

void Lid_Open(void){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 50);
	status = 1;
	HAL_Delay(100);
}

void Lid_Close(void){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 100);
	status = 0;
	HAL_Delay(100);
}

uint32_t ReadUltrasonic1(void){
	uint32_t numTicks1 = 0;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(3);

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);

	numTicks1 = 0;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
	{
		numTicks1++;
		usDelay(2);
	}
	uint32_t dis = (numTicks1 + 0.0f)*2.8*speedOfSound;
	return dis;

}

uint32_t ReadUltrasonic2(void){
	uint32_t numTicks2 = 0;
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);
	usDelay(3);

	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_RESET);

	numTicks2 = 0;
	while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_SET)
	{
		numTicks2++;
		usDelay(2);
	}

	//4. Estimate distance in cm
	uint32_t dis2 = (numTicks2 + 0.0f)*2.8*speedOfSound;
	return dis2;

}

uint32_t ReadUltrasonic3(void){
	uint32_t numTicks3 = 0;
	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);
	usDelay(3);

	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_RESET);

	numTicks3 = 0;
	while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_SET)
	{
		numTicks3++;
		usDelay(2);
	}
	uint32_t dis3 = (numTicks3 + 0.0f)*2.8*speedOfSound;
	return dis3;

}

uint32_t ReadUltrasonic4(void){
	uint32_t numTicks4 = 0;
	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_RESET);
	usDelay(3);

	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_RESET);

	numTicks4 = 0;
	while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_SET)
	{
		numTicks4++;
		usDelay(2);
	}
	uint32_t dis4 = (numTicks4 + 0.0f)*2.8*speedOfSound;
	return dis4;

}

void hx711_powerUp(void) //Power up function
{
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET); //writing to the pin and setting it to 0.
}
void hx711_setGain(uint8_t gain)  //the values should be 32, 64 or 128
{
	if(gain < 64) GAIN = 2; //32, channel B
	else if(gain < 128) GAIN = 3; //64, channel A
	else GAIN = 1; //128, channel A
}


void hx711_init(void) //initializes the hx711 module by calling 2 functions.
{
  hx711_setGain(128); //setting gain to 128, as this was our best result after trying with other gains.
	hx711_powerUp(); //power up the hx711 module.
}

int32_t hx711_get_value(void) //getting the weight from the module.
{
	uint32_t data = 0; //the data (weight) is firstly set to 0.
	uint8_t dout; // this is to show whether at this bit, if theres a number that should be recorded.
	int32_t filler; //to fill the rest of the 32 bits.
	int32_t ret_value; //final value to return after adding the filling and the data together.

	for (uint8_t i = 0; i < 24; i++) //read 24 bit data + set gain and start next conversion
	{

		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET); //set the clock pin to 1.
		delay_us(1); //delay
			dout = HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin); //read from the dout pin in variable dout.
			data = data << 1; //shift the data by 1 to make sure we are in correct position depending on the counter.
			if (dout) //if this bit has an output (value of 1) .
			{
				data++; //it sets the data value at this position as 1 as well.
			}
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET); //set clock pin to 0.
		delay_us(1); //delay
	}

	for( int i = 0; i < GAIN; i ++ ) //this for loop is for the gain, to add more clock cycles based on the gain.
	{
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET); //set clock pin to 1.
		delay_us(1); //delay
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET); //set clock pin to 0.
		delay_us(1); //delay, here we are making a clock cycle.
	}

	if( data & 0x800000 ) //here we are checking if theres values in the 24 bits by anding.
		filler = 0xFF000000; //if there are values we add 1's to the last 8 bits which are needed as this is a 32-bit adc.
	else
		filler = 0x00000000; //however, if nothing is in the data we just add 0's.

	ret_value = filler + data; //the return value is the addition of the data with the filler to have the 32-bits.
	return ret_value; //returning the value to be printed.
}

uint8_t hx711_is_ready(void) //making sure that the HX711 module is ready
{
	return HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin) == GPIO_PIN_RESET; //reading a value from it, the reseting it.
}

void delay_us (uint16_t us) //delay function
{
__HAL_TIM_SET_COUNTER(&htim2,0);  // setting the delay counter to 0.
while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // while loop till the counter reaches the delay given (us).
}

void Motor_Init(void) {
    // Initialize GPIO pins for motor control as outputs
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void Motor_Forward(void){
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void Motor_Reverse(void){
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}

void Motor_Stop(void){
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void ReadSensors(void)
{
	distance1 = ReadUltrasonic1();
	HAL_Delay(10);
	distance2 = ReadUltrasonic2();
	HAL_Delay(10);
	distance3 = ReadUltrasonic3();
	HAL_Delay(10);
	distance4 = ReadUltrasonic4();
	HAL_Delay(10);

	hx711_value = hx711_get_value();
    weight = abs(((hx711_value+146000)/1000))-9;

    sprintf(uartData, "%d %d %d %d %d %d\r\n", (int)distance1, (int)distance2, (int)distance3, (int)distance4, (int)weight, (int)status);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartData, strlen(uartData), HAL_MAX_DELAY);
	HAL_Delay(50);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  hx711_init();

  HAL_Delay(1000);
//
  Motor_Init();
//  Motor_Forward();
//  HAL_Delay(1000);
//  Motor_Stop();
//  HAL_Delay(1000);
//  Motor_Reverse();
//  HAL_Delay(1000);
//  Motor_Stop();
//  HAL_Delay(1000);
//
  Lid_Close();
  HAL_Delay(1000);

//  hx711_value = hx711_get_value();
//  weight = abs(((hx711_value+146000)/1000));
//  char str[25];
//  sprintf(str, "[ Weight = %d ]  ",  weight);
//  HAL_UART_Transmit(&huart2, &str, strlen(str), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ReadSensors();
	  HAL_Delay(200);
	  if ((distance1 <= 30) && (status == 0) && (distance1 >= 10)){
		  HAL_Delay(500);
		  Lid_Open();
		  HAL_Delay(3000);
	  }
	  if ((status == 1) && (distance1 > 30)){
		  HAL_Delay(500);
		  Lid_Close();
		  HAL_Delay(300);
	  }
	  char t[8];
	  if (HAL_UART_Receive(&huart2, t, 1, 1000)==HAL_OK){
		  if (t[0] == '1'){
			  ReadSensors();
			  if ((status == 1) && (distance1 > 30)){
			  		  HAL_Delay(500);
			  		  Lid_Close();
			  		  HAL_Delay(300);
			  }

			  hx711_value = hx711_get_value();
			  weight = abs(((hx711_value+146000)/1000))-9;
			  while (weight <= 300){
				  if (loop > 180){
					  break;
				  }
				  Motor_Reverse();
				  HAL_Delay(50);
				  hx711_value = hx711_get_value();
				  weight = abs(((hx711_value+146000)/1000))-9;
				  HAL_Delay(50);
				  loop += 1;
			  }
			  Motor_Stop();
			  ReadSensors();
			  HAL_Delay(2000);
			  for (rev = 0; rev < loop; rev++){
				  Motor_Forward();
				  HAL_Delay(100 * 0.977);
			  }
			  loop = 0;
			  Motor_Stop();
			  HAL_Delay(1000);
		  }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1680;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3360;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|TRIG3_Pin|TRIG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCK_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENA_Pin|TRIG4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin TRIG3_Pin TRIG2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|TRIG3_Pin|TRIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SCK_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = SCK_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DT_Pin ECHO_Pin */
  GPIO_InitStruct.Pin = DT_Pin|ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_Pin TRIG4_Pin */
  GPIO_InitStruct.Pin = ENA_Pin|TRIG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO4_Pin */
  GPIO_InitStruct.Pin = ECHO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO3_Pin ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO3_Pin|ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
