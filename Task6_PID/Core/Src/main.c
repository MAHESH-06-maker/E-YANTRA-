/*
 * Team Id: 2731
 * Author List: Mahesh, Vallari, Tushar, Saras
 * Filename: CB_TASK6_#2731
 * Theme: eYRC 2025-26: CropDrop Bot (CB)
 * Functions: main, SystemClock_Config, MX_GPIO_Init, MX_DMA_Init, MX_ADC1_Init,
 * MX_TIM2_Init, MX_TIM3_Init, MX_USART2_UART_Init, Motor_Forward, Motor_Reverse,
 * Motor_Stop, Motor_Turn_Left, Motor_Turn_Right, Motor_180_Turn,
 * Calibrate_IR_Sensors, Read_Line_Sensors, Compute_Line_Error,
 * Line_PID_Control, Print_Sensor_Binary, Set_LED_Color,
 * Handle_Object_Detection, Handle_Drop_Point, RGB_Off, RGB_Yellow, RGB_Process,
 * Error_Handler
 * Global Variables: adcBuffer, irMin, irMax, irThreshold, sensorNorm, sensorBinary,
 * robotState, currentLineType, lineError, pid variables, junction_count,
 * junction_debounced, total_delivered, hasPackage, colorRed, colorGreen, colorBlue
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CropDrop Bot - Line Following with Pick and Place
  ******************************************************************************
  * @attention
  *
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
#include "Color.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 255              // Maximum PWM value for motors

// Left Motor PWM channels (L298N)
#define LM_IN1 TIM_CHANNEL_1     // Left motor forward pin
#define LM_IN2 TIM_CHANNEL_2     // Left motor reverse pin

// Right Motor PWM channels (L298N)
#define RM_IN1 TIM_CHANNEL_3     // Right motor forward pin
#define RM_IN2 TIM_CHANNEL_4     // Right motor reverse pin

#define TURN_SPEED     110       // Speed used while turning
#define STOP_TIME_MS   1000      // Delay time for stopping

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
volatile uint16_t adcBuffer[5];   // Raw ADC values from 5 IR sensors
char uartBuf[100];                // UART debug buffer

uint16_t irMin[5];                // Minimum IR calibration values
uint16_t irMax[5];                // Maximum IR calibration values
uint16_t irThreshold[5];          // Mid threshold for line detection

uint16_t sensorNorm[5];           // Normalized IR value (0–1000)
uint8_t  sensorBinary[5];         // Binary line detection (1=line)
int8_t   sensorWeight[5] = { -2, -1, 0, 1, 2 }; // Weights for PID error

uint8_t ledBlinkRequest = 0;      // LED blink flag
uint32_t ledBlinkStart = 0;       // LED blink start time

uint8_t hasPackage = 0;           // 1 if robot is carrying box
uint32_t colorRed, colorGreen, colorBlue; // Color sensor values

uint8_t junction_count = 0;       // Counts junctions passed
uint8_t junction_debounced = 0;   // Prevents double count
uint8_t total_delivered = 0;      // Total boxes delivered

typedef enum {
    DIR_STRAIGHT = 0,   // Go straight
    DIR_LEFT,          // Turn left
    DIR_RIGHT          // Turn right
} Direction_t;

typedef enum {
    STATE_PID,             // Normal line following
    STATE_PEEK_FORWARD,    // Small forward movement before turn
    STATE_STOP_BEFORE_TURN,// Not used (reserved)
    STATE_TURNING,         // Turning state
    ALL_BLACK_FORWARD,     // Forward on black
    ALL_BLACK_CHECK,       // Check dead end
    INVERSE_LINE,          // Black on white line
    STATE_OBJECT_DETECTED, // Box detected
    STATE_PICKING,         // Picking box
    STATE_FINAL_DROP       // Dropping box
} RobotState_t;

typedef enum {
    LINE_WHITE_ON_BLACK = 0, // White line on black floor
    LINE_BLACK_ON_WHITE      // Black line on white floor
} LineType_t;
LineType_t currentLineType = LINE_WHITE_ON_BLACK;
uint8_t onLine = 0;
//pid values
float lineError = 0;
float Kp = 0.25f;
float Ki = 0.002f;
float Kd = 0.2f;
float pid_P = 0, pid_I = 0, pid_D = 0;
float previousError = 0;
float pidOutput = 0;

/* Base speed settings */
uint8_t currentspeed = 50;
uint8_t baseSpeed = 130;     // cruising speed
uint8_t maxSpeed  = 200;     // safety limit
uint8_t Bval = 0;
uint8_t inSharpTurn = 0;
uint8_t bit_sensor;

/* ⭐ ADDED: Pick and Place variables */
uint8_t hasPackage = 0;
uint32_t colorRed = 0, colorGreen = 0, colorBlue = 0;
uint32_t lastPrintTick = 0;

/* ⭐ ADDED: Junction tracking variables */
uint8_t junction_count = 0;
uint8_t junction_debounced = 0;
uint8_t total_delivered = 0;    // ⭐ Delivery Sequence Tracker
uint8_t peekpattern=0;

/* ⭐ ADDED: Printf support */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
void Motor_180_Turn(void);
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
void Handle_Object_Detection(void);
void Handle_Drop_Point(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/*
 * Function Name: main
 * Input: None
 * Output: int
 * Logic:
 *  - Initializes peripherals
 *  - Calibrates IR sensors
 *  - Runs finite state machine:
 *      PID → Junction → Pick → Drop → Return
 * Example Call: Called by startup code
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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 5);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // ⭐ ADDED: Initialize Color Sensor
  Color_Init(&htim3);

  HAL_Delay(500);          // wait before calibration
  Calibrate_IR_Sensors();

  printf("=== CropDrop Bot Ready ===\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Read_Line_Sensors();
//    Print_Sensor_Binary();
    lineError = Compute_Line_Error();

    if (HAL_GetTick() - lastPrintTick > 100)
    {
        Print_Sensor_Binary();
        lastPrintTick = HAL_GetTick();
    }

    if(currentspeed < baseSpeed){
        currentspeed++;
    }

    /* ========== STATE:  PID LINE FOLLOWING ========== */
    if (robotState == STATE_PID)
    {
        // ⭐ 1. Check for object detection at pickup point
        uint8_t ir_state = HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin);

        // ONLY pick up if we haven't finished all 3 deliveries
        if (ir_state == 0 && !hasPackage && currentLineType == LINE_WHITE_ON_BLACK && total_delivered < 3)
        {
            robotState = STATE_OBJECT_DETECTED;
            Motor_Stop();
            HAL_Delay(200);
            continue;
        }

        // ⭐ 2. Inverse line transition (Toggle line type)
        if(bit_sensor == 0b11011 || bit_sensor == 0b10001 ||
        		bit_sensor == 0b11101 || bit_sensor == 0b10111 ||
				bit_sensor == 0b11001 || bit_sensor == 0b10011)
        {
           if (currentLineType == LINE_WHITE_ON_BLACK)
        	   currentLineType = LINE_BLACK_ON_WHITE;
           else
             currentLineType = LINE_WHITE_ON_BLACK;
           	   printf("Line type switched!\r\n");

           	   // ⭐ FIX 1: Shorter jump to prevent drifting off the line
//                    Motor_Forward(baseSpeed, baseSpeed);
//                    HAL_Delay(60);
           	   robotState = STATE_PID;
           	   pid_I = 0;
           	   previousError = 0;
           	   junction_count = 0;
           	   junction_debounced = 1;
                    // ⭐ FIX 2: Force an immediate fresh sensor read so PID calculates correctly
           	   Read_Line_Sensors();
           	   lineError = Compute_Line_Error();
           	   continue;
        }

        // ⭐ 3. INVERSE LINE LOGIC (Black line on White floor)
        if (currentLineType == LINE_BLACK_ON_WHITE)
        {
            // Reset debounce if safely on a straight line
            if (bit_sensor == 0b00100 || bit_sensor == 0b01110 || bit_sensor == 0b00000) {
                junction_debounced = 0;
            }

            // Drop Box Detection
            // Check for drop box ONLY if we have passed the required junction
            if (hasPackage) {
                uint8_t valid_drop = 0;
                if (total_delivered == 0 && junction_count >= 2) valid_drop = 1; // Box 1 -> Point C
                if (total_delivered == 1 && junction_count >= 3) valid_drop = 1; // Box 2 -> Point D
                if (total_delivered == 2 && junction_count >= 1) valid_drop = 1; // Box 3 -> Point B

                if (valid_drop && (bit_sensor == 0b01100)) {
                    printf("Black Drop Box Detected!\r\n");
                    Motor_Stop();
                    robotState = STATE_FINAL_DROP;
                    stateStartTime = HAL_GetTick();
                    continue;
                }
            }

            // Junction Detection (Left, Right, or T-junction)
            if (!junction_debounced && (
                bit_sensor == 0b11100 || bit_sensor == 0b11110 || // Left
                bit_sensor == 0b00111 || bit_sensor == 0b01111 || // Right
                bit_sensor == 0b11111))                           // Full Cross / T
            {
                // If heading to drop box, blind the bot to junctions after making the final turn
                uint8_t blind = 0;
                if (hasPackage) {
                    if (total_delivered == 0 && junction_count >= 2) blind = 1;
                    if (total_delivered == 1 && junction_count >= 3) blind = 1;
                    if (total_delivered == 2 && junction_count >= 1) blind = 1;
                }
                if (blind) {
                    Line_PID_Control();
                    continue;
                }

                junction_count++;
                junction_debounced = 1;
                printf("Junction %d detected!\r\n", junction_count);

                if (hasPackage) {
                    // ⭐ BOX 1 ROUTING (Point C)
                    if (total_delivered == 0) {
                        if (junction_count == 2) {
                            printf("2nd Junction! Taking Left to Point C.\r\n");
                            Motor_Stop(); HAL_Delay(50);
                            storedDirection = DIR_LEFT;
                            stateStartTime = HAL_GetTick();
                            robotState = STATE_PEEK_FORWARD;
                            continue;
                        } else {
                            printf("Junction 1: Going straight\r\n");
                            Motor_Forward(baseSpeed, baseSpeed); HAL_Delay(150);
                            continue;
                        }
                    }
                    // ⭐ BOX 2 ROUTING (Point D)
                    else if (total_delivered == 1) {
                        if (junction_count == 3) {
                            printf("3rd Junction! Taking Right to Point D.\r\n");
                            Motor_Stop(); HAL_Delay(50);
                            storedDirection = DIR_RIGHT;
                            stateStartTime = HAL_GetTick();
                            robotState = STATE_PEEK_FORWARD;
                            continue;
                        } else {
                            printf("Junction %d: Going straight\r\n", junction_count);
                            Motor_Forward(baseSpeed, baseSpeed); HAL_Delay(150);
                            continue;
                        }
                    }
                    // ⭐ BOX 3 ROUTING (Point B)
                    else if (total_delivered == 2) {
                        if (junction_count == 1) {
                            printf("1st Junction! Taking Right to Point B.\r\n");
                            Motor_Stop(); HAL_Delay(50);
                            storedDirection = DIR_RIGHT;
                            stateStartTime = HAL_GetTick();
                            robotState = STATE_PEEK_FORWARD;
                            continue;
                        }
                    }
                } else {
                    // ⭐ RETURNING EMPTY
                    if (junction_count == 1) {
                        if (total_delivered == 1) {
                            printf("Returning from Point C! Taking Right.\r\n");
                            Motor_Stop(); HAL_Delay(50); storedDirection = DIR_RIGHT; stateStartTime = HAL_GetTick(); robotState = STATE_PEEK_FORWARD; continue;
                        }
                        else if (total_delivered == 2) {
                            printf("Returning from Point D! Taking Left.\r\n");
                            Motor_Stop(); HAL_Delay(50); storedDirection = DIR_LEFT; stateStartTime = HAL_GetTick(); robotState = STATE_PEEK_FORWARD; continue;
                        }
                        else if (total_delivered == 3) {
                            printf("Returning from Point B! Taking Left.\r\n");
                            Motor_Stop(); HAL_Delay(50); storedDirection = DIR_LEFT; stateStartTime = HAL_GetTick(); robotState = STATE_PEEK_FORWARD; continue;
                        }
                    } else {
                        printf("Return Trip: Ignoring junction, going straight\r\n");
                        Motor_Forward(baseSpeed, baseSpeed); HAL_Delay(150);
                        continue;
                    }
                }
            }

            // Normal PID for Inverse Line
            Line_PID_Control();
            continue;
        }

        // ⭐ 4. NORMAL LINE LOGIC (White line on Black floor)
        if (currentLineType == LINE_WHITE_ON_BLACK)
        {

            // ⭐ FINAL STOP & NODE A CROSSING
            if (bit_sensor == 0b01110) {
                if (total_delivered == 3 && !hasPackage) {
                    printf("MISSION ACCOMPLISHED! Returned to Point A.\r\n");
                    while(1) {
                        Motor_Stop();
                        // Flash LEDs rapidly to celebrate
                        HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, GPIO_PIN_SET);
                        HAL_Delay(200);
                        HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, GPIO_PIN_RESET);
                        HAL_Delay(200);
                    }
                } else {
                    // We are driving past Node A to get the next box or deliver one.
                    // Jump over it safely so it isn't mistaken for a sharp corner!
                    Motor_Forward(baseSpeed, baseSpeed);
                    HAL_Delay(150);
                    continue;
                }
            }

            // Sharp left turn detection
            if (bit_sensor == 0b11100 || bit_sensor == 0b11000 || bit_sensor == 0b10000 || bit_sensor == 0b11110)
            {
                storedDirection = DIR_LEFT;
                robotState = STATE_PEEK_FORWARD;
                stateStartTime = HAL_GetTick();
                continue;
            }
            // Sharp right turn detection
            if (bit_sensor == 0b00111 || bit_sensor == 0b00011 || bit_sensor == 0b00001 || bit_sensor == 0b01111)
            {
                storedDirection = DIR_RIGHT;
                robotState = STATE_PEEK_FORWARD;
                stateStartTime = HAL_GetTick();
                continue;
            }
        	// Detect "No Line" (All sensors read 0 / Black)
        	if (bit_sensor == 0b00000) {
        		robotState = ALL_BLACK_CHECK;
        		stateStartTime = HAL_GetTick();
        		continue;
        	}

        }

        // Normal PID control for White on Black
        Line_PID_Control();
    } // <-- (THIS WAS THE MISSING CLOSING BRACE!)

    else if (robotState == ALL_BLACK_CHECK)
    {
            // 1. "Peek" forward
    	 	Motor_Forward(80, 80);
    	 	HAL_Delay(100);
    		Motor_Stop();
            HAL_Delay(100);



            // 2. Dash Line Condition: If we find the line again quickly, it was just a gap!
            if (bit_sensor != 0b00000) {
                robotState = STATE_PID;
                pid_I = 0;
                previousError = 0;
                continue;
            }

            // 3. Dead End Condition: If X milliseconds pass and STILL no line, turn around.
            // (You may need to tune '250' depending on your bot's speed and the gap size)
            if (HAL_GetTick() - stateStartTime > 250) {
                Motor_Stop();
                HAL_Delay(100); // Brief pause to stabilize
                printf("Dead End Detected! Turning around...\r\n");

                // Execute your existing 180 degree turn function
                Motor_180_Turn();

                // Reset and go back to normal line following
                robotState = STATE_PID;
                pid_I = 0;
                previousError = 0;
            }
            continue;
    }
    /* ⭐ ADDED: STATE: OBJECT DETECTED */
    else if (robotState == STATE_OBJECT_DETECTED)
    {
        Handle_Object_Detection();
        continue;
    }

    /* ⭐ ADDED: STATE: PICKING */
    else if (robotState == STATE_PICKING)
    {
        if (HAL_GetTick() - stateStartTime > 1000)  // Wait 1 sec for pickup
        {
            printf("Package picked!\r\n");
            hasPackage = 1;
            robotState = STATE_PID;
            pid_I = 0;
            previousError = 0;
        }
        continue;
    }

    /* ⭐ FINAL STATE: FINAL DROP (ALL BLACK ZONE) */
    else if (robotState == STATE_FINAL_DROP)
    {
        Motor_Stop();

        // safety delay to stabilize bot
        if (HAL_GetTick() - stateStartTime < 300)
            continue;

        printf("Dropping package at destination...\r\n");
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_RESET);   // MAGNET OFF

        // ⭐ Stop for 3 seconds
        HAL_Delay(3000);

        total_delivered++; // Count the delivery!

        printf("Package %d Dropped. Turning 180...\r\n", total_delivered);

        // Turn 180 and return
        Motor_180_Turn();

        hasPackage = 0;
        junction_count = 0;
        junction_debounced = 0;

        // Resume PID line following (it will track back to the white-on-black zone)
        robotState = STATE_PID;
        pid_I = 0;
        previousError = 0;
        continue;
    }

    /* ========== STATE: INVERSE LINE (STOP) ========== */
    else if(robotState == INVERSE_LINE)
    {
        Motor_Stop();
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_RESET);
        printf("End of track!\r\n");
        continue;
    }

    /* ========== STATE:  PEEK FORWARD ========== */
    /* ========== STATE:  PEEK FORWARD ========== */
        else if (robotState == STATE_PEEK_FORWARD)
        {
            // ⭐ FIX 3: Intercept false turns! If we hit the transition at an angle,
            // it triggers a turn. This catches it, switches modes, and aborts the turn.
            if(bit_sensor == 0b11011 || bit_sensor == 0b10001 ||
               bit_sensor == 0b11101 || bit_sensor == 0b10111 ||
               bit_sensor == 0b11001 || bit_sensor == 0b10011)
            {
                if (currentLineType == LINE_WHITE_ON_BLACK) currentLineType = LINE_BLACK_ON_WHITE;
                else currentLineType = LINE_WHITE_ON_BLACK;

                printf("Transition hit at angle! Aborting turn.\r\n");

//                Motor_Forward(baseSpeed, baseSpeed);
//                HAL_Delay(60);

                robotState = STATE_PID;
                pid_I = 0; previousError = 0;
                junction_count = 0; junction_debounced = 1;

                Read_Line_Sensors();
                lineError = Compute_Line_Error();
                continue;
            }

            Motor_Forward(80, 80);

            if (HAL_GetTick() - stateStartTime > 100)
            {
                robotState = STATE_TURNING;
                stateStartTime = HAL_GetTick();
            }
            continue;
        }

    /* ========== STATE: TURNING ========== */
    else if (robotState == STATE_TURNING && currentLineType == LINE_WHITE_ON_BLACK)
    {
        if (storedDirection == DIR_LEFT)
            Motor_Turn_Left(50, 180);
        else
            Motor_Turn_Right(180, 50);

        if (HAL_GetTick() - stateStartTime > 150) {
            if (sensorBinary[2])   // center sensor found line
            {
                Motor_Stop();
                pid_I = 0;
                previousError = 0;
                storedDirection = DIR_STRAIGHT;
                robotState = STATE_PID;
            }
        }
        continue;
    }
    else if (robotState == STATE_TURNING)
    {
        if (storedDirection == DIR_LEFT)
            Motor_Turn_Left(baseSpeed, baseSpeed);
        else
            Motor_Turn_Right(baseSpeed, baseSpeed);

        if (HAL_GetTick() - stateStartTime > 200) {
            if (sensorBinary[2])   // center sensor found line
            {
                Motor_Stop();
                pid_I = 0;
                previousError = 0;
                storedDirection = DIR_STRAIGHT;
                robotState = STATE_PID;
            }
        }
        continue;
    }

  } // <-- End of while(1) loop
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

  /*Configure GPIO pin : PUSH_BUTTON_PIN_Pin */
  GPIO_InitStruct.Pin = PUSH_BUTTON_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSH_BUTTON_PIN_GPIO_Port, &GPIO_InitStruct);

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
/*
 * Function Name: Motor_180_Turn
 * Input: None
 * Output: None
 * Logic:
 *  - Rotates robot until center sensor detects line again
 * Example Call: Motor_180_Turn();
 */
void Motor_180_Turn(void) {
    Motor_Turn_Right(TURN_SPEED, TURN_SPEED);
    HAL_Delay(1350);

    while (1) {
        Motor_Turn_Right(TURN_SPEED, TURN_SPEED);
        Read_Line_Sensors();

        if (sensorBinary[2] == 1) {
            Motor_Stop();
            HAL_Delay(200);
            break;
        }
    }
}

/*
 * Function Name: Motor_Stop
 * Input: None
 * Output: None
 * Logic: Stops both motors by setting PWM to zero
 * Example Call: Motor_Stop();
 */
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}
/*
 * Function Name: Motor_Forward
 * Input: left_speed, right_speed (0–255)
 * Output: None
 * Logic: Runs both motors forward using PWM
 * Example Call: Motor_Forward(120,120);
 */
void Motor_Forward(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, right_speed);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}
/*
 * Function Name: Motor_Reverse
 * Input: left_speed, right_speed (0–255)
 * Output: None
 * Logic: Runs both motors in reverse direction using PWM
 * Example Call: Motor_Reverse(100,100);
 */
void Motor_Reverse(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, right_speed);
}
/*
 * Function Name: Motor_Turn_Left
 * Input: left_speed, right_speed (0–255)
 * Output: None
 * Logic: Turns robot left by reversing left motor and forwarding right motor
 * Example Call: Motor_Turn_Left(80,150);
 */
void Motor_Turn_Left(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, right_speed);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, 0);
}
/*
 * Function Name: Motor_Turn_Right
 * Input: left_speed, right_speed (0–255)
 * Output: None
 * Logic: Turns robot right by forwarding left motor and reversing right motor
 * Example Call: Motor_Turn_Right(150,80);
 */
void Motor_Turn_Right(uint8_t left_speed, uint8_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, LM_IN2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, RM_IN2, right_speed);
}
/*
 * Function Name: Calibrate_IR_Sensors
 * Input: None
 * Output: None
 * Logic:
 *  - Rotates robot
 *  - Stores min and max ADC values
 *  - Computes threshold for each sensor
 * Example Call: Calibrate_IR_Sensors();
 */
void Calibrate_IR_Sensors(void)
{
    for (int i = 0; i < 5; i++)
    {
        irMin[i] = adcBuffer[i];
        irMax[i] = adcBuffer[i];
    }

    uint32_t startTime = HAL_GetTick();

    while (HAL_GetTick() - startTime < 4000)
    {
        Motor_Turn_Left(100, 100);

        for (int i = 0; i < 5; i++)
        {
            uint16_t val = adcBuffer[i];
            if (val < irMin[i]) irMin[i] = val;
            if (val > irMax[i]) irMax[i] = val;
        }

        HAL_Delay(2);
    }

    Motor_Stop();

    for (int i = 0; i < 5; i++)
    {
        irThreshold[i] = (irMin[i] + irMax[i]) / 2;
    }

    while(1){
    	Bval = HAL_GPIO_ReadPin(PUSH_BUTTON_PIN_GPIO_Port, PUSH_BUTTON_PIN_Pin);
    	 if(Bval == 0){
    		 HAL_Delay(50);  // Debounce
    		 break;
    	 }
    	 HAL_Delay(10);
    }
}
/*
 * Function Name: Read_Line_Sensors
 * Input: None
 * Output: None
 * Logic:
 *  - Reads ADC buffer
 *  - Normalizes values
 *  - Generates binary line pattern
 * Example Call: Read_Line_Sensors();
 */
void Read_Line_Sensors(void)
{
    onLine = 0;

    for (int i = 0; i < 5; i++)
    {
        uint16_t raw = adcBuffer[i];

        if (raw <= irMin[i])
            sensorNorm[i] = 0;
        else if (raw >= irMax[i])
            sensorNorm[i] = 1000;
        else
            sensorNorm[i] = (uint16_t)(
                ((raw - irMin[i]) * 1000UL) / (irMax[i] - irMin[i])
            );

        if (currentLineType == LINE_WHITE_ON_BLACK)
        {
            sensorBinary[i] = (raw > irThreshold[i]) ? 1 : 0;
        }
        else
        {
            sensorBinary[i] = (raw > irThreshold[i]) ? 0 : 1;
            sensorNorm[i] = 1000 - sensorNorm[i];
        }

        if (sensorBinary[i])
            onLine = 1;
    }
}
/*
 * Function Name: Compute_Line_Error
 * Input: None
 * Output: float (error)
 * Logic:
 *  - Uses weighted average method
 *  - Converts binary sensor pattern to error
 * Example Call: err = Compute_Line_Error();
 */
float Compute_Line_Error(void)
{
    int activeSensors = 0;
    float errorSum = 0;
    bit_sensor = 0;

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
        return previousError;
    }

    return errorSum / activeSensors;
}
/*
 * Function Name: Line_PID_Control
 * Input: None
 * Output: None
 * Logic:
 *  - Computes P, I, D terms
 *  - Adjusts motor speed
 * Example Call: Line_PID_Control();
 */
void Line_PID_Control(void)
{
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
/*
 * Function Name: Print_Sensor_Binary
 * Input: None
 * Output: None
 * Logic: Prints IR sensor binary pattern and PID error on UART
 * Example Call: Print_Sensor_Binary();
 */
void Print_Sensor_Binary(void)
{
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
/*
 * Function Name: Print_Sensor_Binary
 * Input: None
 * Output: None
 * Logic: Prints IR sensor binary pattern and PID error on UART
 * Example Call: Print_Sensor_Binary();
 */
void Set_LED_Color(GPIO_PinState red, GPIO_PinState green, GPIO_PinState blue) {
    HAL_GPIO_WritePin(GPIOC, RED_Pin, red);
    HAL_GPIO_WritePin(GPIOC, GREEN_Pin, green);
    HAL_GPIO_WritePin(GPIOC, BLUE_Pin, blue);
}
/*
 * Function Name: Handle_Object_Detection
 * Input: None
 * Output: None
 * Logic:
 *  - Reads color sensor
 *  - Identifies box color
 *  - Turns ON electromagnet
 *  - Updates delivery logic
 * Example Call: Handle_Object_Detection();
 */
void Handle_Object_Detection(void)
{

	colorRed   = Color_ReadRed();
	colorGreen = Color_ReadGreen();
	colorBlue  = Color_ReadBlue();

	printf("Color:  R=%lu G=%lu B=%lu\r\n", colorRed, colorGreen, colorBlue);
	Set_LED_Color(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET);

	if (colorRed < 600 && colorGreen < 600 && colorBlue < 600)

	{
		HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_RESET);
		printf("CLEAR object - skipping\r\n");
		robotState = STATE_PID;
		return;
	}

	HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_SET);

	const char* color_name;
	if (colorRed > colorBlue && colorRed > (colorGreen * 0.4)) {
		color_name = "RED";
		Set_LED_Color(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET);
	}
	else if (colorGreen > colorRed && colorGreen > colorBlue) {
		color_name = "GREEN";
		Set_LED_Color(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
	}
	else if (colorBlue > colorGreen && colorBlue > colorRed) {
		color_name = "BLUE";
		Set_LED_Color(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);
	}

	printf("Picking %s box\r\n", color_name);

	HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);

	hasPackage = 1;
	junction_count = 0;
	junction_debounced = 0;

	if (total_delivered < 2) {
		    Motor_180_Turn();   // Box 1 and Box 2
		} else {
		    printf("3rd box picked - NO 180 turn\r\n");
		}

		robotState = STATE_PID;
		pid_I = 0;
		previousError = 0;
}
/*
 * Function Name: Handle_Drop_Point
 * Input: None
 * Output: None
 * Logic:
 *  - Turns OFF electromagnet
 *  - Releases box
 *  - Updates delivery count
 * Example Call: Handle_Drop_Point();
 */
void Handle_Drop_Point(void)
{
	printf("Dropping package...\r\n");

	HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

	printf("Package dropped!\r\n");

	hasPackage = 0;

	robotState = STATE_PID;
	pid_I = 0;
	previousError = 0;
}
/*
 * Function Name: RGB_Off
 * Input: None
 * Output: None
 * Logic: Turns OFF all RGB LEDs
 * Example Call: RGB_Off();
 */
void RGB_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);
}
/*
 * Function Name: RGB_Yellow
 * Input: None
 * Output: None
 * Logic: Turns ON red and green LEDs to show yellow color
 * Example Call: RGB_Yellow();
 */
void RGB_Yellow(void)
{
    HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
}
/*
 * Function Name: RGB_Process
 * Input: None
 * Output: None
 * Logic: Handles LED blinking based on ledBlinkRequest flag
 * Example Call: RGB_Process();
 */
void RGB_Process(void)
{
    if (ledBlinkRequest)
    {
        if (HAL_GetTick() - ledBlinkStart < 80)
        {
            RGB_Yellow();
        }
        else
        {
            RGB_Off();
            ledBlinkRequest = 0;
        }
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
