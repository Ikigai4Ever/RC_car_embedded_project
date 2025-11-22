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
#include <math.h>
#include <string.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
#define ADC_MAX 4095
#define ADC_MID 2048
#define DEADZONE 80
#define TIM_ARR 3999

#define SPEED_RAMP_STEP 2.0f  // How fast target speed changes (encoder counts per 0.01s)

// PID Constants - tune these values
#define KP 6.5f     // Proportional gain
#define KI 3.0f     // Integral gain
#define KD 0.04f    // Derivative gain

// ENCODER FILTER: Low-pass filter coefficient (0.0 to 1.0)
#define ENCODER_FILTER_ALPHA 0.3f  // 30% new reading, 70% old reading

// Encoder speed limits (counts per 0.01s = counts per 100Hz update)
#define ENCODER_MAX_SPEED 100
#define ENCODER_MIN_SPEED -100

// Bluetooth UART buffers
#define BT_RX_BUFFER_SIZE 128
uint8_t bt_rx_buffer[BT_RX_BUFFER_SIZE];
uint8_t bt_line_buffer[64];

// Received Bluetooth values (replacing local ADC)
volatile uint32_t bt_joy_x = ADC_MID;  // Default to center
volatile uint32_t bt_joy_y = ADC_MID;  // Default to center
volatile uint32_t bt_speed = 0;        // Default to stopped
volatile uint32_t last_bt_rx_time = 0; // Timeout safety

volatile int32_t joy_Y = 0;
volatile int32_t joy_X = 0;
volatile uint32_t speed = 0;
volatile uint32_t speed_base = 0;
volatile uint32_t raw_joy_Y = 0;
volatile uint32_t raw_joy_X = 0;

typedef struct
{
    volatile uint32_t ch1_pwm;
    volatile uint32_t ch2_pwm;
    volatile uint32_t target_ch1;
    volatile uint32_t target_ch2;

    // Encoder variables
    volatile int32_t encoder_count;
    volatile int32_t encoder_speed;
    volatile int32_t encoder_speed_raw;
    volatile float encoder_speed_filtered;
    volatile int32_t last_encoder_count;

    // PID variables
    volatile float target_speed;
    volatile float desired_speed;
    volatile float pid_error;
    volatile float pid_integral;
    volatile float pid_last_error;
    volatile float pid_output;

    // Diagnostic variables
    volatile float p_term;
    volatile float i_term;
    volatile float d_term;

} Motor;

Motor motor1;
Motor motor2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float ramp_speed_to_target(float current, float target)
{
    if (current < target)
    {
        current += SPEED_RAMP_STEP;
        if (current > target)
            current = target;
    }
    else if (current > target)
    {
        current -= SPEED_RAMP_STEP;
        if (current < target)
            current = target;
    }
    return current;
}

void update_encoder_speed(Motor *motor, int32_t current_encoder_count)
{
    motor->encoder_count = current_encoder_count;

    // Calculate the difference (could be large if counter wraps around)
    int32_t count_diff = current_encoder_count - motor->last_encoder_count;

    // Handle timer counter overflow/underflow (65535 wrap-around)
    if (count_diff > 32768)
    {
        count_diff -= 65536;
    }
    else if (count_diff < -32768)
    {
        count_diff += 65536;
    }

    // Negate if encoders are wired backwards
    motor->encoder_speed_raw = count_diff;

    // Apply low-pass filter to smooth out noise
    motor->encoder_speed_filtered = ENCODER_FILTER_ALPHA * (float)motor->encoder_speed_raw +
                                    (1.0f - ENCODER_FILTER_ALPHA) * motor->encoder_speed_filtered;

    motor->encoder_speed = (int32_t)motor->encoder_speed_filtered;
    motor->last_encoder_count = current_encoder_count;
}

// PID controller function
void update_pid(Motor *motor)
{
    if (motor->target_speed == 0.0f)w
    {
        motor->pid_error = 0.0f;
        motor->pid_integral = 0.0f;
        motor->pid_last_error = 0.0f;
        motor->pid_output = 0.0f;
        motor->p_term = 0.0f;
        motor->i_term = 0.0f;
        motor->d_term = 0.0f;
        return;
    }

    float target_encoder_speed = fabs(motor->target_speed);
    float actual_encoder_speed = fabs((float)motor->encoder_speed);

    motor->pid_error = target_encoder_speed - actual_encoder_speed;

    motor->pid_integral += motor->pid_error * 0.01f;

    float max_integral = 200.0f;
    if (motor->pid_integral > max_integral)
        motor->pid_integral = max_integral;
    else if (motor->pid_integral < -max_integral)
        motor->pid_integral = -max_integral;

    float derivative = (motor->pid_error - motor->pid_last_error) / 0.01f;

    motor->p_term = KP * motor->pid_error;
    motor->i_term = KI * motor->pid_integral;
    motor->d_term = KD * derivative;

    float pid_speed_output = motor->p_term + motor->i_term + motor->d_term;

    motor->pid_output = (pid_speed_output / (float)ENCODER_MAX_SPEED) * (float)TIM_ARR;

    if (motor->pid_output > (float)TIM_ARR)
        motor->pid_output = (float)TIM_ARR;
    else if (motor->pid_output < 0.0f)
        motor->pid_output = 0.0f;

    motor->pid_last_error = motor->pid_error;
}

// Parse Bluetooth data and update motor control
void ParseBluetoothData(char *line)
{
    uint32_t x, y, pot;
    if (sscanf(line, "%lu,%lu,%lu", &x, &y, &pot) == 3)
    {
        // Update received values
        bt_joy_x = x;
        bt_joy_y = y;
        bt_speed = pot;

        // Process just like ADC callback
        raw_joy_X = bt_joy_x;
        raw_joy_Y = bt_joy_y;
        speed = bt_speed;

        // Calculate base speed from pot
        speed_base = (speed * TIM_ARR) / ADC_MAX;
        float speed_ratio = (float)speed_base / (float)TIM_ARR;

        joy_Y = raw_joy_Y - (int32_t)ADC_MID;
        joy_X = raw_joy_X - (int32_t)ADC_MID;

        if (speed_base == 0 || (joy_Y > -(int32_t)DEADZONE && joy_Y < (int32_t)DEADZONE))
        {
            motor1.desired_speed = 0.0f;
            motor2.desired_speed = 0.0f;
        }
        else if (joy_Y > (int32_t)DEADZONE)

        {



        	if ((joy_X > -(int32_t)DEADZONE && joy_X < (int32_t)DEADZONE)) //no turning
        	{

        		motor1.desired_speed = speed_ratio * (float)ENCODER_MAX_SPEED;
        		motor2.desired_speed = speed_ratio * (float)ENCODER_MAX_SPEED;

        	}

        	else if (joy_X > (int32_t)DEADZONE) //turning right
        	{
        		motor1.desired_speed = speed_ratio * (float)ENCODER_MAX_SPEED;
        		motor2.desired_speed = speed_ratio * (float)ENCODER_MIN_SPEED;

        	}

        	else if (joy_X < -(int32_t)DEADZONE) //turning left
        	{
        	   motor1.desired_speed = speed_ratio * (float)ENCODER_MIN_SPEED;
        	   motor2.desired_speed = speed_ratio * (float)ENCODER_MAX_SPEED;

        	}



        }
        else if (joy_Y < -(int32_t)DEADZONE) //would backwards turning be the same as forwards turning?
        {
            motor1.desired_speed = speed_ratio * (float)ENCODER_MIN_SPEED;
            motor2.desired_speed = speed_ratio * (float)ENCODER_MIN_SPEED;
        }
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
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  // Initialize motor structs
  motor1.ch1_pwm = 0;
  motor1.ch2_pwm = 0;
  motor1.target_ch1 = 0;
  motor1.target_ch2 = 0;
  motor1.encoder_count = 0;
  motor1.encoder_speed = 0;
  motor1.encoder_speed_raw = 0;
  motor1.encoder_speed_filtered = 0.0f;
  motor1.last_encoder_count = 0;
  motor1.target_speed = 0.0f;
  motor1.desired_speed = 0.0f;
  motor1.pid_error = 0.0f;
  motor1.pid_integral = 0.0f;
  motor1.pid_last_error = 0.0f;
  motor1.pid_output = 0.0f;
  motor1.p_term = 0.0f;
  motor1.i_term = 0.0f;
  motor1.d_term = 0.0f;

  motor2.ch1_pwm = 0;
  motor2.ch2_pwm = 0;
  motor2.target_ch1 = 0;
  motor2.target_ch2 = 0;
  motor2.encoder_count = 0;
  motor2.encoder_speed = 0;
  motor2.encoder_speed_raw = 0;
  motor2.encoder_speed_filtered = 0.0f;
  motor2.last_encoder_count = 0;
  motor2.target_speed = 0.0f;
  motor2.desired_speed = 0.0f;
  motor2.pid_error = 0.0f;
  motor2.pid_integral = 0.0f;
  motor2.pid_last_error = 0.0f;
  motor2.pid_output = 0.0f;
  motor2.p_term = 0.0f;
  motor2.i_term = 0.0f;
  motor2.d_term = 0.0f;

  // Start encoder timers (TIM1 for motor1, TIM2 for motor2)
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim6); //100Hz timer for speed calculation and control

  //start TIM3 (motor 1 PWM)
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
      while(1) { __NOP(); }
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
      while(1) { __NOP(); }
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
      while(1) { __NOP(); }
  }

  //start TIM4 (motor 2 PWM)
  if (HAL_TIM_Base_Start(&htim4) != HAL_OK)
  {
      while(1) { __NOP(); }
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
      while(1) { __NOP(); }
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
      while(1) { __NOP(); }
  }

  HAL_GPIO_WritePin(GPIOA, Right_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, Left_EN_Pin, GPIO_PIN_SET);

  // BLUETOOTH UART SETUP (replacing ADC)
  // Enable UART idle line detection interrupt
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

  // Start DMA reception with idle line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, bt_rx_buffer, BT_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT); // Disable half-transfer interrupt

  // Initialize Bluetooth timeout
  last_bt_rx_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Left_EN_Pin|Right_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Left_EN_Pin Right_EN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Left_EN_Pin|Right_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// UART Idle Line Detection Callback - Receives Bluetooth data continuously
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5) // Change to your UART instance
    {
        // Update timestamp for timeout safety
        last_bt_rx_time = HAL_GetTick();

        // Copy received data to line buffer
        memcpy(bt_line_buffer, bt_rx_buffer, Size);
        bt_line_buffer[Size] = '\0'; // Null terminate

        // Parse the received line (updates motor speeds)
        ParseBluetoothData((char*)bt_line_buffer);

        // CRITICAL: Restart DMA reception for continuous flow
        HAL_UARTEx_ReceiveToIdle_DMA(huart, bt_rx_buffer, BT_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

// Timer callback - 100Hz motor control loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) // 100Hz update
    {
        // SAFETY: If no Bluetooth data for 500ms, stop motors
        if ((HAL_GetTick() - last_bt_rx_time) > 500)
        {
            motor1.desired_speed = 0.0f;
            motor2.desired_speed = 0.0f;
        }

        // Read encoder counts
        int32_t enc1_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
        int32_t enc2_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

        // Update encoder speeds
        update_encoder_speed(&motor1, enc1_count);
        update_encoder_speed(&motor2, enc2_count);



        // Ramp target_speed smoothly towards desired_speed
        motor1.target_speed = ramp_speed_to_target(motor1.target_speed, motor1.desired_speed);
        motor2.target_speed = ramp_speed_to_target(motor2.target_speed, motor2.desired_speed);

        // Update PID controllers
        update_pid(&motor1);
        update_pid(&motor2);

        // Set targets based on PID output - Motor 1
        if (motor1.target_speed > 0.0f)
        {
            motor1.target_ch1 = (uint32_t)motor1.pid_output;
            motor1.target_ch2 = 0;
        }
        else if (motor1.target_speed < 0.0f)
        {
            motor1.target_ch1 = 0;
            motor1.target_ch2 = (uint32_t)motor1.pid_output;
        }
        else
        {
            motor1.target_ch1 = 0;
            motor1.target_ch2 = 0;
        }

        // Set targets based on PID output - Motor 2
        if (motor2.target_speed > 0.0f)
        {
            motor2.target_ch1 = (uint32_t)motor2.pid_output;
            motor2.target_ch2 = 0;
        }
        else if (motor2.target_speed < 0.0f)
        {
            motor2.target_ch1 = 0;
            motor2.target_ch2 = (uint32_t)motor2.pid_output;
        }
        else
        {
            motor2.target_ch1 = 0;
            motor2.target_ch2 = 0;
        }

        // SAFETY: Block opposite direction until current direction stops
        if (motor1.ch1_pwm > 10 && motor1.target_ch2 > 0)
        {
            motor1.target_ch2 = 0;
        }
        if (motor1.ch2_pwm > 10 && motor1.target_ch1 > 0)
        {
            motor1.target_ch1 = 0;
        }

        if (motor2.ch1_pwm > 10 && motor2.target_ch2 > 0)
        {
            motor2.target_ch2 = 0;
        }
        if (motor2.ch2_pwm > 10 && motor2.target_ch1 > 0)
        {
            motor2.target_ch1 = 0;
        }

        // PID directly controls PWM
        motor1.ch1_pwm = motor1.target_ch1;
        motor1.ch2_pwm = motor1.target_ch2;
        motor2.ch1_pwm = motor2.target_ch1;
        motor2.ch2_pwm = motor2.target_ch2;

        // Write to PWM registers
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor1.ch1_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, motor1.ch2_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor2.ch1_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor2.ch2_pwm);
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
