/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RC_NUM_CHANNELS 8

// Set up our receiver channels - these are the channels from the receiver
#define RC_CH1  0 // Right Stick LR
#define RC_CH2  1 // Right Stick UD
#define RC_CH3  2 // Left  Stick UD
#define RC_CH4  3 // Left  Stick LR
#define RC_CH5  4 // Left  Stick LR
#define RC_CH6  5 // Left  Stick LR
#define RC_CH7  6 // Left  Stick LR
#define RC_CH8  7 // Left  Stick LR

// Set up our channel pins - these are the pins that we connect to the receiver
#define RC_CH1_INPUT  GPIO_PIN_9 // receiver pin 1
#define RC_CH2_INPUT  GPIO_PIN_8 // receiver pin 2
#define RC_CH3_INPUT  GPIO_PIN_7 // receiver pin 3
#define RC_CH4_INPUT  GPIO_PIN_6 // receiver pin 4
#define RC_CH5_INPUT  GPIO_PIN_5 // receiver pin 1
#define RC_CH6_INPUT  GPIO_PIN_4 // receiver pin 2
#define RC_CH7_INPUT  GPIO_PIN_3 // receiver pin 3
#define RC_CH8_INPUT  GPIO_PIN_15 // receiver pin 4

#define RC_CH1_PORT  GPIOB // receiver pin 1
#define RC_CH2_PORT  GPIOB // receiver pin 2
#define RC_CH3_PORT  GPIOA // receiver pin 3
#define RC_CH4_PORT  GPIOA // receiver pin 4
#define RC_CH5_PORT  GPIOB // receiver pin 1
#define RC_CH6_PORT  GPIOB // receiver pin 2
#define RC_CH7_PORT  GPIOB // receiver pin 3
#define RC_CH8_PORT  GPIOA // receiver pin 4

#define RC_ON 1
#define RC_OFF 0

uint8_t RC_State = RC_OFF;



// Set up some arrays to store our pulse starts and widths
uint16_t RC_VALUES[RC_NUM_CHANNELS];
uint32_t RC_START[RC_NUM_CHANNELS];
uint16_t RC_SHARED[RC_NUM_CHANNELS];

uint32_t micros = 0;
#define MAX_MOTOR_SPEED 15000
int z = 0, cnt = -1, x=0, y=0;
uint8_t bt, tmp[100];
uint8_t Rx_flag = 0;
int pot;
#define RX2_Size 8
uint8_t Rx2_Buff[RX2_Size];
float Pitch=0, Roll=0, Heading=0;
float Pitch_correction = 0, Roll_correction = 0, Heading_correction = 0;
float Pitch_set = 0, Roll_set = 0, Heading_set = 0;
float Roll_K_p = 30, Pitch_K_p = 30, Heading_K_p = 70;
uint8_t GY_A5[] = {0xA5}, GY_54[] = {0x54}, GY_51[] = {0x51}, GY_55[] = {0x55}, GY_Init_Command[]    = {0xA5, 0x54, 0xA5, 0x51}, GY_Request_Command[] = {0xA5, 0x51}, GY_Set_Command[] = {0xA5, 0x55};
void initGY(){
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart2, GY_54, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart2, GY_51, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart2, GY_55, 1, 100);
	HAL_Delay(500);
}
void readGY(){
	HAL_UART_Transmit(&huart2, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart2, GY_51, 1, 100);
	HAL_UART_Receive(&huart2, Rx2_Buff, RX2_Size, 100);
	for(int i=0; i<RX2_Size; i++){
		if(Rx2_Buff[i] == 0xAA){
			Heading = (int16_t)(Rx2_Buff[(i+1)%8]<<8 | Rx2_Buff[(i+2)%8])/100.00 + Heading_set;
			Pitch = (int16_t)(Rx2_Buff[(i+3)%8]<<8 | Rx2_Buff[(i+4)%8])/100.00 + Pitch_set + y;
			Roll = (int16_t)(Rx2_Buff[(i+5)%8]<<8 | Rx2_Buff[(i+6)%8])/100.00 + Roll_set + x;

			if(Heading > 180) Heading -= 360;
			if(Heading <-180) Heading += 360;
			if(Pitch > 180) Pitch -= 360;
			if(Pitch <-180) Pitch += 360;
			if(Roll > 180) Roll -= 360;
			if(Roll <-180) Roll += 360;
		}
	}
}
void delay(long int _time){
	for(int i=0; i<_time*1000;i++);
}
void motor(int M1, int M2, int M3, int M4){
	M1 -= 2500;
	M3 -= 500;
//	if(M3 > 20) M3 += 128;
//	if(M4 > 20) M4 += 192;
	if(M1 > MAX_MOTOR_SPEED) M1 = MAX_MOTOR_SPEED;
	if(M1 < 0)   						 M1 = 0;
	if(M2 > MAX_MOTOR_SPEED) M2 = MAX_MOTOR_SPEED;
	if(M2 < 0)  						 M2 = 0;
	if(M3 > MAX_MOTOR_SPEED) M3 = MAX_MOTOR_SPEED;
	if(M3 < 0) 							 M3 = 0;
	if(M4 > MAX_MOTOR_SPEED) M4 = MAX_MOTOR_SPEED;
	if(M4 < 0) 							 M4 = 0;

	TIM1->CCR1 = 16000 + M3; //M3
	TIM1->CCR2 = 16000 + M2; //M2
	TIM1->CCR3 = 16000 + M1; //M1
	TIM1->CCR4 = 16000 + M4; //M4
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	cnt++;
	tmp[cnt] = bt;
	if(cnt >= 100){
		for(int i=0; i<100; i++) tmp[i] = 0;
		cnt = -1;
	}
	Rx_flag = 1;
	HAL_UART_Receive_IT(&huart1, &bt, 1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RC_CH1_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH1_PORT, RC_CH1_INPUT)){
			RC_START[RC_CH1] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH1]);
			RC_SHARED[RC_CH1] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH2_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH2_PORT, RC_CH2_INPUT)){
			RC_START[RC_CH2] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH2]);
			RC_SHARED[RC_CH2] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH3_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH3_PORT, RC_CH3_INPUT)){
			RC_START[RC_CH3] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH3]);
			RC_SHARED[RC_CH3] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH4_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH4_PORT, RC_CH4_INPUT)){
			RC_START[RC_CH4] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH4]);
			RC_SHARED[RC_CH4] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH5_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH5_PORT, RC_CH5_INPUT)){
			RC_START[RC_CH5] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH5]);
			RC_SHARED[RC_CH5] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH6_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH6_PORT, RC_CH6_INPUT)){
			RC_START[RC_CH6] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH6]);
			RC_SHARED[RC_CH6] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH7_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH7_PORT, RC_CH7_INPUT)){
			RC_START[RC_CH7] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH7]);
			RC_SHARED[RC_CH7] = rc_compare;
		}
	}
	if(GPIO_Pin == RC_CH8_INPUT) {
		if(HAL_GPIO_ReadPin(RC_CH8_PORT, RC_CH8_INPUT)){
			RC_START[RC_CH8] = micros;
		}else{
			uint16_t rc_compare = (uint16_t)(micros - RC_START[RC_CH8]);
			RC_SHARED[RC_CH8] = rc_compare;
		}
	}
	if(RC_SHARED[RC_CH7] > 70) RC_State = RC_OFF;
	else RC_State = RC_ON;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    micros ++;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	motor(0,0,0,0);
//	initGY();
//	HAL_UART_Receive_IT(&huart1, &bt, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  z = (110 - RC_SHARED[RC_CH2]) * 600;
	  x = -(110 - RC_SHARED[RC_CH1] - 25) * 100;
	  y = -(110 - RC_SHARED[RC_CH3] - 27) * 100;
	  if(x >= -400 && x <= 400) x = 0;
	  if(y >= -400 && y <= 400) y = 0;
	  if(z < 0) z = 0;
	  if(z == 0) {x = 0; y = 0;}

	  if(RC_State == RC_ON){
//		  motor(z,z,z,z);
		  motor(
			  z - y - x,
			  z + y - x,
			  z + y + x,
			  z - y + x
		  );
	  }
	  else{
		  motor(0,0,0,0);
	  }
//	  if(Rx_flag == 1){
//		if(tmp[cnt] == '\n'){
//			for(int i = cnt-1; i >= cnt - 5 && tmp[i] != '\n'; i--){
//				if(tmp[i] == 'J'){
//					z = 0;
//					for(int j=i+1; j<cnt; j++){
//						z = z*10 + (tmp[j]-'0');
//					}
//					z = z*32;
//					i = -1;
//				}
//				if(tmp[i] == 'X')	Roll_set  += 0.5;
//				if(tmp[i] == 'Y')	Roll_set  -= 0.5;
//				if(tmp[i] == 'N')	Pitch_set += 0.5;
//				if(tmp[i] == 'M')	Pitch_set -= 0.5;
//				if(tmp[i] == 'F')	y = -15;
//				if(tmp[i] == 'G')	y = 15;
//				if(tmp[i] == 'R')	x = -15;
//				if(tmp[i] == 'L')	x = 15;
//				if(tmp[i] == 'S')	{
//					x = 0;
//					y = 0;
//				}
//			}
//		}
//		Rx_flag = 0;
//	}
//
//
//
//
//	if(z == 0){
//		Roll_correction = 0;
//		Pitch_correction = 0;
//		Heading_correction = 0;
//	}
//	else{
//		Roll_correction = Roll * Roll_K_p;
//		Pitch_correction = Pitch * Pitch_K_p;
//		Heading_correction = Heading * Heading_K_p;
//	}
//
//	if(Roll > 40 || Roll < -40 || Pitch > 40 || Pitch < -40){
//		motor(0,0,0,0);
//	}
//	else {
//		motor(z, z, z, z);
////			motor(
////				z - Roll_correction + Pitch_correction + Heading_correction,
////				z - Roll_correction - Pitch_correction - Heading_correction,
////				z + Roll_correction - Pitch_correction + Heading_correction,
////				z + Roll_correction + Pitch_correction - Heading_correction
////			);
//	}
	//motor(0, 0, 0, 0);
	//readGY();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 128;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA6 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
