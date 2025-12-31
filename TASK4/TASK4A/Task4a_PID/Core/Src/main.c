/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include<stdio.h>
#include<string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 255

// Left Motor (L298N)
#define LM_IN1 TIM_CHANNEL_1   // PA0
#define LM_IN2 TIM_CHANNEL_2   // PA1

// Right Motor (L298N)
#define RM_IN1 TIM_CHANNEL_3   // PA2
#define RM_IN2 TIM_CHANNEL_4   // PA3
#define TURN_SPEED     110
#define STOP_TIME_MS   1000



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t adcBuffer[5];
char uartBuf[100];       // UART print buffer
/* >>> ADD: Calibration variables */
uint16_t irMin[5];
uint16_t irMax[5];
uint16_t irThreshold[5];
/* >>> ADD: Line sensing variables */
uint16_t sensorNorm[5];     // 0–1000 normalized
uint8_t  sensorBinary[5];   // 1 = white, 0 = black
int8_t   sensorWeight[5] = { -2, -1, 0, 1, 2 };
//int8_t   bitWeight[5] = {1,2,4,8,16};
typedef enum {
    DIR_STRAIGHT = 0,
    DIR_LEFT,
    DIR_RIGHT
} Direction_t;
//
//Direction_t direction = DIR_STRAIGHT;
typedef enum {
    STATE_PID,
    STATE_PEEK_FORWARD,
    STATE_STOP_BEFORE_TURN,
    STATE_TURNING,
	ALL_BLACK_FORWARD,
	ALL_BLACK_CHECK,
	INVERSE_LINE
} RobotState_t;

RobotState_t robotState = STATE_PID;
Direction_t storedDirection = DIR_STRAIGHT;

uint32_t stateStartTime = 0;

typedef enum {
    LINE_WHITE_ON_BLACK = 0,   // white line, black surface
    LINE_BLACK_ON_WHITE        // black line, white surface
} LineType_t;

LineType_t currentLineType = LINE_WHITE_ON_BLACK;
uint8_t onLine = 0;
float lineError = 0;
float Kp = 0.15f;
float Ki = 0.002f;
float Kd = 0.15f;
float pid_P = 0, pid_I = 0, pid_D = 0;
float previousError = 0;
float pidOutput = 0;

/* Base speed settings */
uint8_t currentspeed= 50;
uint8_t baseSpeed = 150;     // cruising speed
uint8_t maxSpeed  = 200;     // safety limit
uint8_t Bval=0;
uint8_t inSharpTurn = 0;
uint8_t bit_sensor;










/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Stop(void);
void Motor_Forward(uint8_t left_speed, uint8_t right_speed);
void Motor_Reverse(uint8_t left_speed, uint8_t right_speed);
void Motor_Turn_Left(uint8_t left_speed, uint8_t right_speed);
void Motor_Turn_Right(uint8_t left_speed, uint8_t right_speed);
void Calibrate_IR_Sensors(void);
void Read_Line_Sensors(void);
float Compute_Line_Error(void);
void Line_PID_Control(void);
void Print_Sensor_Binary(void);
void side_calibration(void);




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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer,5);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_Delay(500);          // wait before calibration
  Calibrate_IR_Sensors();






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    Read_Line_Sensors();
	    lineError = Compute_Line_Error();
//	    Print_Sensor_Binary();
	    if(currentspeed<baseSpeed){
	    	currentspeed++;
	    }
//	    /* -------- SHARP TURN DETECTION -------- */
	    if (robotState == STATE_PID)
	    {
	        if (bit_sensor == 0b11100 || bit_sensor == 0b11000 ||
	            bit_sensor == 0b10000 )
	        {
	            storedDirection = DIR_LEFT;
	            robotState = STATE_PEEK_FORWARD;
	            stateStartTime = HAL_GetTick();
	            continue;
	        }

	        if (bit_sensor == 0b00111 || bit_sensor == 0b00011 ||
	            bit_sensor == 0b00001 )
	        {
	            storedDirection = DIR_RIGHT;
	            robotState = STATE_PEEK_FORWARD;
	            stateStartTime = HAL_GetTick();
	            continue;
	        }
//	        if(bit_sensor==0b00000){
//	        	robotState = ALL_BLACK_FORWARD;
//	        	stateStartTime = HAL_GetTick();
//	        	continue;
//	        }
	        if(bit_sensor==0b11011|| bit_sensor==0b10001 || bit_sensor==0b11101|| bit_sensor==0b10111){
	        	//inverse line detected
	        	robotState=INVERSE_LINE;
	        	continue;
	        }
	    }

	    if(robotState==INVERSE_LINE){
	    	//Black line with White background
	    	Motor_Stop();
	    	continue;
	    }
	    if (robotState == STATE_PEEK_FORWARD)
	    {
	        Motor_Forward(100, 100);

	        if (HAL_GetTick() - stateStartTime > 150)   // move forward ~300ms
	        {
//	            robotState = STATE_STOP_BEFORE_TURN;
	        	robotState = STATE_TURNING;
	            stateStartTime = HAL_GetTick();
	        }
	        continue;
	    }
//	    if (robotState == ALL_BLACK_FORWARD)
//	    {
//	    	 Motor_Forward(70, 70);
//
//	    	 if (HAL_GetTick() - stateStartTime >= 50)
//	    	 {
//	    		 Motor_Stop();
//	    	     robotState = ALL_BLACK_CHECK;   // 👈 decision state
//	    	 }
//	    	 continue;
//	    }
//	    if (robotState == ALL_BLACK_CHECK)
//	    {
//	        Read_Line_Sensors();
//	        Compute_Line_Error();   // updates bit_sensor
//
//	        if (bit_sensor == 0b00000)
//	        {
//	            //STILL ALL BLACK → FULL STOP
//	            Motor_Stop();
//	            // stay here forever OR until reset
//	        }
//	        else
//	        {
//	            //LINE FOUND → RESUME PID
//	            pid_I = 0;
//	            previousError = 0;
//	            robotState = STATE_PID;
//	        }
//	        continue;
//	    }
//	    if (robotState == STATE_STOP_BEFORE_TURN)
//	    {
//	        Motor_Stop();
//
//	        if (HAL_GetTick() - stateStartTime > 150)
//	        {
//	            robotState = STATE_TURNING;
//	        }
//	        continue;
//	    }
	    if (robotState == STATE_TURNING)
	    {
	        if (storedDirection == DIR_LEFT)
	            Motor_Turn_Left(baseSpeed,baseSpeed);
	        else
	            Motor_Turn_Right(baseSpeed, baseSpeed);

	        if (sensorBinary[2])   // center sensor found line
	        {
	            Motor_Stop();
	            pid_I = 0;
	            previousError = 0;
	            storedDirection =DIR_STRAIGHT;
	            robotState = STATE_PID;

	        }
	        continue;
	    }
	    if (robotState == STATE_PID)
	    {
	        Line_PID_Control();
	    }
//	    if(onLine == 1 && inSharpTurn == 0){
//	    	Line_PID_Control();
//	    }
//	    else{
//
//	    if((sensorBinary[0] && sensorBinary[1] && sensorBinary[2] && !sensorBinary[3] && !sensorBinary[4])||
//	    	   (sensorBinary[0] && sensorBinary[1] &&! sensorBinary[2] && !sensorBinary[3] && !sensorBinary[4])||
//			   (sensorBinary[0] && !sensorBinary[1] && !sensorBinary[2] && !sensorBinary[3] && !sensorBinary[4])||
//			   (sensorBinary[0] && sensorBinary[1] && sensorBinary[2] && sensorBinary[3] && !sensorBinary[4])){
//	    		inSharpTurn = 1;
//
//	    		    // 1️⃣ STOP
//	    		    Motor_Stop();
//	    		    HAL_Delay(STOP_TIME_MS);
//
//	    		    // 2️⃣ TURN LEFT
//	    		    Motor_Turn_Left(100, 200);
//	    		    HAL_Delay(500);
//
//	    		    // 3️⃣ STOP again briefly
//	    		    Motor_Stop();
//	    		    HAL_Delay(50);
//
//	    		    // Reset PID so it doesn't jerk
//	    		    pid_I = 0;
//	    		    previousError = 0;
//
//	    		    inSharpTurn = 0;
//	    	}
//	    	else if((!sensorBinary[0] && sensorBinary[1] && sensorBinary[2] && sensorBinary[3] && sensorBinary[4])||
//	    			(!sensorBinary[0] && !sensorBinary[1] && sensorBinary[2] && sensorBinary[3] && sensorBinary[4])||
//					(!sensorBinary[0] && !sensorBinary[1] && !sensorBinary[2] && sensorBinary[3] && sensorBinary[4])||
//					(!sensorBinary[0] && !sensorBinary[1] && !sensorBinary[2] && !sensorBinary[3] && sensorBinary[4])){
//	    		inSharpTurn = 1;
//
//	    		    // 1️⃣ STOP
//	    		    Motor_Stop();
//	    		    HAL_Delay(STOP_TIME_MS);
//
//	    		    // 2️⃣ TURN RIGHT
//	    		    Motor_Turn_Right(200, 100);
//	    		    HAL_Delay(500);
//
//	    		    // 3️⃣ STOP again briefly
//	    		    Motor_Stop();
//	    		    HAL_Delay(50);
//
//	    		    pid_I = 0;
//	    		    previousError = 0;
//
//	    		    inSharpTurn = 0;
//	    	}
//	    }
//	    HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PUSH_BUTTON_Pin */
  GPIO_InitStruct.Pin = PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Electromagnet_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Electromagnet_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin GREEN_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Box_detect_Pin */
  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}

void Motor_Forward(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);

    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, right_speed);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}

void Motor_Reverse(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, left_speed);

    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, right_speed);
}

void Motor_Turn_Left(uint8_t left_speed, uint8_t right_speed)
{
	 	 // Left motor backward
	    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, left_speed);

	    // Right motor forward
	    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, right_speed);
	    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}

void Motor_Turn_Right(uint8_t left_speed, uint8_t right_speed)
{
		// Left motor forward
	    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, left_speed);
	    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);

	    // Right motor backward
	    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, right_speed);
}

/* IR sensor calibration function */
void Calibrate_IR_Sensors(void)
{
    // Initialize min & max with first ADC readings
    for (int i = 0; i < 5; i++)
    {
        irMin[i] = adcBuffer[i];
        irMax[i] = adcBuffer[i];
    }

    uint32_t startTime = HAL_GetTick();

    // Rotate robot for calibration (~4 seconds)
    while (HAL_GetTick() - startTime < 4000)
    {
        // Rotate in place
        Motor_Turn_Left(100, 100);

        for (int i = 0; i < 5; i++)
        {
            uint16_t val = adcBuffer[i];

            if (val < irMin[i]) irMin[i] = val;
            if (val > irMax[i]) irMax[i] = val;
        }

        HAL_Delay(2);  // small delay for ADC stability
    }




    // Calculate threshold for each sensor
    for (int i = 0; i < 5; i++)
    {
        irThreshold[i] = (irMin[i] + irMax[i]) / 2;
    }
    while(1){
    	 Motor_Stop();
    	 Bval=HAL_GPIO_ReadPin(PUSH_BUTTON_GPIO_Port, PUSH_BUTTON_Pin);
    	 if(Bval==0){
    		 break;
    	 }

    }

//  	Print calibration results
//    int len = snprintf(uartBuf, sizeof(uartBuf),
//        "CAL DONE | T: %d %d %d %d %d\r\n",
//        irThreshold[0], irThreshold[1], irThreshold[2],
//        irThreshold[3], irThreshold[4]);
//
//    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, HAL_MAX_DELAY);
}

/*: Read & classify IR sensors (WHITE line) */
void Read_Line_Sensors(void)
{
    onLine = 0;

    for (int i = 0; i < 5; i++)
    {
        uint16_t raw = adcBuffer[i];

        // Normalize to 0–1000 (WHITE = 1000)
        if (raw <= irMin[i])
            sensorNorm[i] = 0;
        else if (raw >= irMax[i])
            sensorNorm[i] = 1000;
        else
            sensorNorm[i] = (uint16_t)(
                ( (raw - irMin[i]) * 1000UL ) / (irMax[i] - irMin[i])
            );

        // Binary decision: WHITE or BLACK
        sensorBinary[i] = (raw > irThreshold[i]) ? 1 : 0;

        if (sensorBinary[i])
            onLine = 1;
    }
}

/*  Compute weighted line error */
float Compute_Line_Error(void)
{
    int activeSensors = 0;
    float errorSum = 0;
    bit_sensor=0;

    for (int i = 0; i < 5; i++)
    {
        if (sensorBinary[i])
        {
            errorSum += sensorWeight[i] * sensorNorm[i];
            bit_sensor |= (sensorBinary[i] << (4 - i));
            activeSensors++;
        }
    }


    if (activeSensors == 0)
    {
        return previousError;   // preserve turn direction
    }


    return errorSum / activeSensors;
}

/* PID based line following */
void Line_PID_Control(void)
{
//    /* ---------------- SHARP TURN DETECTION ---------------- */
//
//    direction = DIR_STRAIGHT;
//
//    switch (bit_sensor)
//    {
//        /* LEFT sharp turn */
//        case 0b11100:
//        case 0b11000:
//        case 0b10000:
//        case 0b11110:
//            direction = DIR_LEFT;
//            break;
//
//        /* RIGHT sharp turn */
//        case 0b00111:
//        case 0b00011:
//        case 0b00001:
//        case 0b01111:
//            direction = DIR_RIGHT;
//            break;
//    }
//
//    /* ---------------- TURN EXECUTION ---------------- */
//
//    if (bit_sensor == 0 && direction != DIR_STRAIGHT)
//    {
//        inSharpTurn = 1;
//
//        Motor_Forward(baseSpeed,baseSpeed);
//        HAL_Delay(500);
//
//        if (direction == DIR_LEFT){
//        	while(!sensorBinary[2]){
//        	Motor_Stop();
//        	HAL_Delay(500);
//            Motor_Turn_Left(TURN_SPEED, TURN_SPEED);
//            Read_Line_Sensors();
//            lineError = Compute_Line_Error();
//            direction = DIR_STRAIGHT;
//
//        	}
//        }
//        else{
//        	while(!sensorBinary[2]){
//        	Motor_Stop();
//        	HAL_Delay(500);
//            Motor_Turn_Right(TURN_SPEED, TURN_SPEED);
//            Read_Line_Sensors();
//            lineError = Compute_Line_Error();
//            direction = DIR_STRAIGHT;
//        	}
//        }
//        HAL_Delay(350);
//
//        Motor_Stop();
//        HAL_Delay(20);
//
//        pid_I = 0;
//        previousError = 0;
//        inSharpTurn = 0;
//        lineError = 0;
//
//
//        return;   // ⛔ EXIT PID
//    }
	/* ---------- SHARP TURN DETECTION (ONLY IN PID STATE) ---------- */

    /* ---------------- NORMAL PID ---------------- */

    pid_P = lineError;
    pid_I += lineError;

    if (pid_I > 300)  pid_I = 300;
    if (pid_I < -300) pid_I = -300;

    pid_D = lineError - previousError;

    pidOutput = (Kp * pid_P) + (Ki * pid_I) + (Kd * pid_D);
    previousError = lineError;

    int leftSpeed  = baseSpeed + pidOutput;
    int rightSpeed = baseSpeed - pidOutput;

    if (leftSpeed > maxSpeed)  leftSpeed = maxSpeed;
    if (leftSpeed < 0)         leftSpeed = 0;
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
    if (rightSpeed < 0)        rightSpeed = 0;

    Motor_Forward((uint8_t)leftSpeed, (uint8_t)rightSpeed);
}


void Print_Sensor_Binary(void){
	int16_t err_i = (int16_t)(lineError * 100);

	    int len = snprintf(uartBuf, sizeof(uartBuf),
	        "BIN:%d%d%d%d%d | on:%d | err:%d\r\n",
	        sensorBinary[0],
	        sensorBinary[1],
	        sensorBinary[2],
	        sensorBinary[3],
	        sensorBinary[4],
	        onLine,
	        err_i
	    );

	    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, HAL_MAX_DELAY);
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
#ifdef USE_FULL_ASSERT
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
