/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Motor.h"

/* -------------------------------------------------------------------------- */
/* Pin definitions                                                            */
/* -------------------------------------------------------------------------- */
#define SENSOR1_Pin        GPIO_PIN_1
#define SENSOR1_GPIO_Port  GPIOB

#define SENSOR2_Pin        GPIO_PIN_3
#define SENSOR2_GPIO_Port  GPIOB

#define SENSOR3_Pin        GPIO_PIN_4
#define SENSOR3_GPIO_Port  GPIOB

#define SENSOR4_Pin        GPIO_PIN_5
#define SENSOR4_GPIO_Port  GPIOB

#define SENSOR5_Pin        GPIO_PIN_6
#define SENSOR5_GPIO_Port  GPIOB

#define LED_Pin            GPIO_PIN_10
#define LED_GPIO_Port      GPIOB

#define TRIG_Pin           GPIO_PIN_8
#define TRIG_GPIO_Port     GPIOB
#define ECHO_Pin           GPIO_PIN_9
#define ECHO_GPIO_Port     GPIOB

#define START_BUTTON_Pin   GPIO_PIN_7
#define START_BUTTON_Port  GPIOB


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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Motor_t MotorA = {&htim1, TIM_CHANNEL_1, GPIOA, GPIO_PIN_4,
                  GPIOA, GPIO_PIN_5};
Motor_t MotorB = {&htim1, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1,
                  GPIOA, GPIO_PIN_15};
int sensor_read = 0;
int position = 0;
int actives = 0;

/* PID: start with P only to diagnose jerking; tune Kd later if needed. */
float Kp = 1;
float Ki = 0;
float Kd = 0;
float Kr = 0;

int P = 0, I = 0, D = 0, R = 0;
int lastError = 0;
int errors[10] = {0};

/* last_end: retained from the original code; verify direction on the robot. */
int last_end = 0;       /* Original convention: 0 -> Left, 1 -> Right */
int last_idle = 0;

const int maxspeedr = 900;
const int maxspeedl = 900;
const int basespeedr = 700;
const int basespeedl = 700;

/* Intersection and route state. */
int cross_count = 0;
int is_on_cross = 0;
int cross_exit_count = 0;
int system_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

static void motor_control(int16_t pwm_first, int16_t pwm_second);
static void sharp_turn(void);
static void forward_brake(int speed_right, int speed_left);
static int readsensor(void);
//static uint16_t get_distance(void);
static void ne_vat_can(void);
static void past_errors(int error);
static int errors_sum(int count, int take_abs);
static void PID_control(void);
/*void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}*/

/* USER CODE END PFP */
static void motor_control(int16_t pwm_first, int16_t pwm_second)
{
    /* Keep the original mapping: first value -> MotorA, second -> MotorB. */
    Motor_SetPWM(&MotorA, pwm_first);
    Motor_SetPWM(&MotorB, pwm_second);
}

static void sharp_turn(void)
{
    if (last_idle < 10)
    {
        if (last_end == 1)
            motor_control(600, 100);
        else
            motor_control(100, 600);
    }
    else
    {
        if (last_end == 1)
            motor_control(650, -300);
        else
            motor_control(-300, 650);
    }
}

static void forward_brake(int speed_right, int speed_left)
{
    if (actives == 0)
        sharp_turn();
    else
        /* Preserve the original right/left argument order. */
        motor_control((int16_t)speed_right, (int16_t)speed_left);
}

/* -------------------------------------------------------------------------- */
/* Read the five line sensors                                                 */
/* Active-low sensors: GPIO_PIN_RESET means black line detected.              */
/* -------------------------------------------------------------------------- */
static int readsensor(void)
{
    sensor_read = 0;
    int pos_sum = 0;
    int active = 0;

    if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET)
    {

        pos_sum += -400;
        active++;
        last_end = 1;
    }

    if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET)
    {

        pos_sum += -200;
        active++;
    }

    if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET)
    {

        pos_sum += 0;
        active++;
    }

    if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET)
    {

        pos_sum += 200;
        active++;
    }

    if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
    {

        pos_sum += 400;
        active++;
        last_end = 0;
    }

    actives = active;

    /* Keep the last valid position if all sensors lose the line. */
    if (active > 0)
        position = pos_sum / active;

    if (active == 0)
        last_idle++;
    else
        last_idle = 0;

    return position;
}

/* -------------------------------------------------------------------------- */
/* Ultrasonic distance                                                        */
/* NOTE: local_time-based conversion is an approximate original implementation.
 * It should be calibrated or replaced with timer input-capture for accuracy.
 */
/* -------------------------------------------------------------------------- */
/*static uint16_t get_distance(void)
{
    uint32_t local_time = 0;

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 720; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    uint32_t timeout = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - timeout) > 1U)
            return 999;
    }

    timeout = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
    {
        local_time++;
        for (volatile int i = 0; i < 5; i++)
            __NOP();

        if ((HAL_GetTick() - timeout) > 2U)
            break;
    }

    return (uint16_t)(local_time / 120U);
}*/

/* -------------------------------------------------------------------------- */
/* Basic obstacle-avoidance sequence                                          */
/* -------------------------------------------------------------------------- */
static void ne_vat_can(void)
{
    // 1. Dừng xe giảm quán tính
	for (int i = 0; i < 100; i++)
	    {
	        motor_control(0, 0);
	        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
	    }//1
// cần tăng thêm thời gian cho rẽ và tăng chênh lệch đoạn giữa
    // 2. Đánh lái lách sang phải
    for (int i = 0; i < 333; i++)
    {
        motor_control(800, 100);
        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
    }//1

    for (int i = 0; i < 100; i++)
    	    {
    	        motor_control(0, 0);
    	        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
    	    }
    for (int i = 0; i < 300; i++)
        {
            motor_control(500, 800);
            HAL_Delay(2);
            if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET ||
                                       HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET ||
                                       HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET ||
                                       HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET ||
                                       HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
                                   {
                                       break;
                                   }
        }//2
    for (int i = 0; i < 100; i++)
        	    {
        	        motor_control(0, 0);
        	        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
        	    }
    for (int i = 0; i < 334; i++)
                {
                    motor_control(700, 700);
                    HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
                    if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET ||
                                HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET ||
                                HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET ||
                                HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET ||
                                HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
                            {
                                break;
                            }}
    for (int i = 0; i < 100; i++)
        	    {
        	        motor_control(0, 0);
        	        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
        	    }
    for (int i = 0; i < 250; i++)
            {
                motor_control(100, 800);
                HAL_Delay(2);
                if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET ||
                                           HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET ||
                                           HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET ||
                                           HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET ||
                                           HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
                                       {
                                           break;
                                       }
            }//2

    /*for (int i = 0; i < 200; i++)
    {
        motor_control(100, 700);
        HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
    }//3
    for (int i = 0; i < 667; i++)
        {
            motor_control(700, 700);
            HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
        }//4

    for (int i = 0; i < 200; i++)
        {
            motor_control(100, 800);
            HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
        }//5
    for (int i = 0; i < 350; i++)
            {
                motor_control(750, 750);
                HAL_Delay(3);
            }//6
    for (int i = 0; i < 250; i++)
        {
            motor_control(800, 100);
            HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
        }//7*/
    for (int i = 0; i < 334; i++)
            {
                motor_control(700, 700);
                HAL_Delay(3); // Tổng thời gian: 150 * 5 = 750ms
                if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET ||
                            HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET ||
                            HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET ||
                            HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET ||
                            HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
                        {
                            break;
                        }
            }//8





    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < 3000U)
    {
        if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin) == GPIO_PIN_RESET)
        {
            break;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* PID history and control                                                    */
/* -------------------------------------------------------------------------- */
static void past_errors(int error)
{
    for (int i = 9; i > 0; i--)
        errors[i] = errors[i - 1];

    errors[0] = error;
}

static int errors_sum(int count, int take_abs)
{
    int sum = 0;

    if (count > 10)
        count = 10;

    for (int i = 0; i < count; i++)
    {
        if ((take_abs != 0) && (errors[i] < 0))
            sum += -errors[i];
        else
            sum += errors[i];
    }

    return sum;
}

static void PID_control(void)
{
    /* position and actives were already updated once at the start of this loop. */
    int error = position;
    past_errors(error);

    P = error;
    I = errors_sum(5, 0);
    D = error - lastError;
    R = errors_sum(5, 1);
    lastError = error;

    int motor_correction = (int)(P * Kp + I * Ki + D * Kd);

    int motorspeedl = basespeedl + motor_correction - (int)(R * Kr);
    int motorspeedr = basespeedr - motor_correction - (int)(R * Kr);

    if (motorspeedl > maxspeedl)  motorspeedl = maxspeedl;
    if (motorspeedr > maxspeedr)  motorspeedr = maxspeedr;
    if (motorspeedl < -maxspeedl) motorspeedl = -maxspeedl;
    if (motorspeedr < -maxspeedr) motorspeedr = -maxspeedr;

    forward_brake(motorspeedr, motorspeedl);
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t IC_Val1 = 0, IC_Val2 = 0;
volatile uint8_t  is_first_captured = 0;
volatile uint8_t  new_measurement = 0;
volatile uint16_t final_distance = 999;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    uint32_t time_us = 0;
      uint32_t distance = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_TIM_Base_Start(&htim4);
      // 2. Đứng chờ 1.5 giây để mạch xả hết nhiễu điện và tụ bù nạp đầy
        HAL_Delay(1500);

        // 3. Xóa sạch mọi tín hiệu nhiễu lúc chờ
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC4);
      HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

      uint32_t last_trig_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      /* USER CODE BEGIN WHILE */
      while (1)
              {
                  // 1. CHẠY BÁM LINE LIÊN TỤC KHÔNG NGỪNG NGHỈ
                  readsensor();
                  PID_control();

                  // 2. CỨ 100MS BẮN XUNG TRIG MỘT LẦN, RỒI BỎ ĐI CHẠY PID TIẾP
                  if (HAL_GetTick() - last_trig_time >= 100)
                  {
                      HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
                      for (volatile int i = 0; i < 720; i++) __NOP();
                      HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

                      last_trig_time = HAL_GetTick(); // Cập nhật mốc thời gian
                  }

                  // 3. KIỂM TRA BIẾN KHOẢNG CÁCH (Do ngắt tự cập nhật ngầm bên dưới)
                  if ((final_distance > 0) && (final_distance < 20))
                  {
                      ne_vat_can();

                      // Sau khi né xong, reset lại các thông số để xe không bị loạn
                      is_on_cross = 0;
                      cross_exit_count = 0;
                      lastError = 0;
                      final_distance = 999;
                      last_trig_time = HAL_GetTick();
                  }
              }

        /* USER CODE END WHILE */
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MTBT_Pin|MTAT_Pin|MTAL_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |MTBL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LED_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : MTBT_Pin MTAT_Pin MTAL_Pin PA6
                           PA7 PA10 PA11 PA12
                           MTBL_Pin */
  GPIO_InitStruct.Pin = MTBT_Pin|MTAT_Pin|MTAL_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |MTBL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LED_Pin PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LED_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB3 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // Kiểm tra đúng là ngắt của TIM4 Kênh 4 (PB9)
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        if (is_first_captured == 0) // Cạnh lên (Sóng phát đi)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            is_first_captured = 1;
            // Đổi cực sang chờ cạnh xuống
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (is_first_captured == 1) // Cạnh xuống (Sóng dội về)
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

            // Tính thời gian (đã quy đổi sẵn ra micro-giây nhờ Prescaler = 71)
            if (IC_Val2 > IC_Val1) {
                final_distance = (IC_Val2 - IC_Val1) / 59;
            } else { // Chống tràn bộ đếm
                final_distance = ((0xffff - IC_Val1) + IC_Val2) / 59;
            }

            is_first_captured = 0;
            // Đổi cực về lại cạnh lên cho lần đo sau
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
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
