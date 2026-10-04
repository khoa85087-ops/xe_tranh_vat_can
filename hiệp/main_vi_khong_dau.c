
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @mo_ta          : Than chuong trinh chinh cho robot bam line
  ******************************************************************************
  * @luu_y
  *
  * Ban quyen (c) 2025 STMicroelectronics.
  * Bao luu moi quyen.
  *
  * Phan mem nay duoc cap phep theo cac dieu khoan trong file LICENSE
  * nam trong thu muc goc cua thanh phan phan mem nay.
  * Neu thanh phan phan mem nay khong co file LICENSE, phan mem duoc cung cap nguyen trang.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Cac thu vien rieng ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Kieu du lieu rieng -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Dinh nghia rieng ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_COUNT 5
#define MAX_SPEED 100    // Toc do toi da
/* USER CODE END PD */

/* Macro rieng -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Cac bien rieng ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int weights[5] = {-4, -2, 0, 2, 4}; // Dinh nghia trong so (mm) cho 5 cam bien: tu trai sang phai
float Kp = 2.5;                     // He so ti le
float Ki = 0.001;                     // He so tich phan
float Kd = 0.5;                    // He so vi phan
float dt = 0.005;                   // Khoang thoi gian (giay) - dieu chinh theo he thong
float d = 10.0;                     // Khoang cach giua cac lan doc vi tri (mm)
float previous_error = 0;           // Sai so truoc do cho dieu khien vi phan
float integral = 0;                 // Sai so tich luy cho dieu khien tich phan
float previous_position = 0;        // Vi tri truoc do de tinh goc huong
int intersection_count=0;
int grab_count=0;
int has_grabbed = 0; // Co toan cuc theo doi viec gap da xay ra
int current_case = 0; // 0 = Case 0, 1 = Case 1, 2 = Case 2, ...
int sensor_count = 0; // Dem so lan ca 5 cam bien deu phat hien trong Case 1

uint8_t sensor_values[SENSOR_COUNT]; // Mang luu gia tri cam bien

float x =0;//vi tri ngang
float u =0; //tin hieu dieu khien
int pwmL =0;
int pwmR=0;
int basespeed=0;

int j=0; // Mat line

// Cac bien lien quan den UART
uint8_t rx_data;         // Du lieu UART nhan duoc
uint8_t last_command = 0;// Lenh nhan duoc gan nhat

uint8_t uto = 'U';       // Dieu khien che do: 'U' tu dong, 'u' thu cong
char grip = 0;           // Trang thai kep: 'W' dong, 'w' mo
int speed_level = 7;     // Muc toc do (0-9) cho che do thu cong

static int case1_delay_count = 0;
static int case2_stopped = 0;
static int case2_ignore_count = 0;
static int case2_post_stop_state = 0;
static int case2_post_action_count = 0;
static int case3_has_grabbed = 0;
static int turn_state = 0;
static int turn_count = 0;
static int case3_stopped = 0;
static int case3_ignore_count = 0;
static int case3_post_stop_state = 0;
static int case3_post_action_count = 0;

int caseF_active = 0;     // Co theo doi CaseF dang hoat dong
int caseF_end_condition = 0; // Co theo doi khi nao CaseF ket thuc

// Bien toan cuc moi cho thao tac day toi Case0
int case0_initial_push_count = 0; // Bo dem cho thao tac day toi ban dau trong Case0
int case0_initial_push_done = 0;  // Co theo doi thao tac day toi ban dau da hoan tat trong Case0

// Bien toan cuc moi cho thao tac day toi trong Case1
int case1_push_after_grab_done = 0; // Co theo doi thao tac day toi sau khi gap da hoan tat

// Bien toan cuc moi cho thao tac day toi trong Case3
int case3_push_after_grab_count = 0; // Bo dem cho thao tac day toi sau khi gap trong Case3

int case2_reset_active = 0; // Co theo doi Case2Dat lai dang hoat dong


/* USER CODE END PV */

/* Khai bao nguyen mau ham rieng -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void set_left_motor(int speed);
void set_right_motor(int speed);
void read_sensors();
float compute_position();
float compute_pid(float error);
float compute_heading(float x_now);
void Set_Servo_Angle(uint8_t angle);
void motor_ctr(int speedL, int speedR);
void followLine(void);
void followLine_Case0(void);
void followLine_Case1(void);
void followLine_Case2(void);
void followLine_Case3(void);
/* USER CODE END PFP */

/* Code nguoi dung rieng ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Servo_Angle(uint8_t angle)
{
  // Anh xa 0-180 do sang 500-2500us (do rong xung tinh theo bo dem timer)
  uint16_t pulse_width = 500 + ((angle * 2000) / 180);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width);  // Dat do rong xung PWM
}

/* USER CODE END 0 */

/**
  * @brief  Diem bat dau cua chuong trinh.
  * @ket_qua_tra_ve int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Dat lai tat ca ngoai vi, khoi tao giao dien Flash va Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Cau hinh xung nhip he thong */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Khoi tao tat ca configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Khoi dong PWM cho servo

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Bat ngat UART
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
  /* USER CODE END 2 */

  /* Vong lap vo han */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Cau hinh xung nhip he thong
  * @ket_qua_tra_ve Khong co
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Khoi tao cac bo dao dong RCC theo cac tham so da chi dinh
  * trong cau truc RCC_OscInitTypeDef.
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

  /** Khoi tao xung nhip CPU, AHB va APB
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
  * @brief Ham khoi tao TIM2
  * @param Khong co
  * @ket_qua_tra_ve Khong co
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
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
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
  * @brief Ham khoi tao TIM3
  * @param Khong co
  * @ket_qua_tra_ve Khong co
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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  * @brief Ham khoi tao TIM4
  * @param Khong co
  * @ket_qua_tra_ve Khong co
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief Ham khoi tao USART1
  * @param Khong co
  * @ket_qua_tra_ve Khong co
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
  * @brief Ham khoi tao GPIO
  * @param Khong co
  * @ket_qua_tra_ve Khong co
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* Bat xung nhip cho cac cong GPIO */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Cau hinh muc dau ra GPIO */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Cau hinh muc dau ra GPIO */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Cau hinh chan GPIO: PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Cau hinh cac chan GPIO: PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Cau hinh cac chan GPIO: PB1 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Dat toc do va chieu quay cho dong co trai
  * @param toc_do: Gia tri toc do (-95 den 95). Duong la chay toi, am la chay lui.
  * @ket_qua_tra_ve Khong co
  */
void set_left_motor(int speed)
{
  speed = (speed > MAX_SPEED) ? MAX_SPEED : (speed < -MAX_SPEED/2) ? -MAX_SPEED/2 : speed;
  TIM4->CCR2 = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (speed > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (speed < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief Dat toc do va chieu quay cho dong co phai
  * @param toc_do: Gia tri toc do (-95 den 95). Duong la chay toi, am la chay lui.
  * @ket_qua_tra_ve Khong co
  */
void set_right_motor(int speed)
{
  speed = (speed > MAX_SPEED) ? MAX_SPEED : (speed < -MAX_SPEED/2) ? -MAX_SPEED/2 : speed;
  TIM4->CCR1 = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (speed > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (speed < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_ctr(int speedL, int speedR)
{
  set_right_motor(speedR);
  set_left_motor(speedL);
}

/**
  * @brief Doc trang thai cam bien
  * @ket_qua_tra_ve Khong co
  */
void read_sensors(void)
{
  sensor_values[0] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  sensor_values[1] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
  sensor_values[2] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
  sensor_values[3] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  sensor_values[4] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

  if (sensor_values[0] == 1) {
    j = 1;
  }
  else if (sensor_values[4] == 1) {
    j = 2;
  }

  if (j == 1) {
    sensor_values[0] = 1;
  }
  else if (j == 2) {
    sensor_values[4] = 1;
  }
  if (sensor_values[1] == 1 || sensor_values[2] == 1 || sensor_values[3] == 1) {
    j = 0;
  }
}

/**
  * @brief Tinh vi tri ngang
  * @ket_qua_tra_ve float: Vi tri theo mm
  */
float compute_position()
{
  int sum_s = 0;
  int weighted_sum = 0;

  for (int i = 0; i < 5; i++) {
    sum_s += sensor_values[i];
    weighted_sum += sensor_values[i] * weights[i];
  }

  if (sum_s == 0)
    return 0.0; // Khong phat hien line, gia dinh o giua hoac kich hoat che do an toan

  return (float)weighted_sum / sum_s;
}

/**
  * @brief Tinh tin hieu dieu khien PID
  * @param sai_so: Sai so hien tai
  * @ket_qua_tra_ve float: Tin hieu dieu khien PID
  */
float compute_pid(float error)
{
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  return output;
}

/**
  * @brief Tinh goc huong
  * @param vi_tri_hien_tai: Vi tri hien tai
  * @ket_qua_tra_ve float: Goc huong theo radian
  */
float compute_heading(float x_now)
{
  float Q = (x_now - previous_position) / d;
  previous_position = x_now;
  return Q; // theo radian (xap xi)
}

void followLine(void)
{
  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 55;
  }
  else if (fabs(x) <= 2) {
    basespeed = 40;
  }
  else if (fabs(x) <= 4) {
    basespeed = 20;
  }
  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

void followLine_Case0(void)
{


	Set_Servo_Angle(40);

	// Buoc 2: Day toi ban dau 200ms (40 chu ky, moi chu ky 5ms)
	    if (!case0_initial_push_done) {
	        pwmL = 50; // Toc do co dinh cho thao tac day toi
	        pwmR = 50;
	        motor_ctr(pwmL, pwmR);
	        case0_initial_push_count++;
	        if (case0_initial_push_count >= 40) { // 200ms / 5ms = 40 chu ky
	            case0_initial_push_done = 1; // Danh dau thao tac day toi da hoan tat
	            case0_initial_push_count = 0; // Dat lai bo dem de co the su dung lai
	        }
	        return; // Thoat ham cho den khi thao tac day toi hoan tat
	    }

  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 55;
  } else if (fabs(x) <= 2) {
    basespeed = 37;
  } else if (fabs(x) <= 4) {
    basespeed = 25
    		;
  }

  int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
  if (!has_grabbed) {
    if (sensor_values[0] == 1 && sensor_values[1] == 1 && sensor_values[2] == 1 && sensor_values[3] == 1) {
      Set_Servo_Angle(0); // Gap ve 0 do
      has_grabbed = 1;    // Ngan khong cho gui them lenh servo
      current_case = 1;
    } else if (sensor_values[4] == 1 && sensor_values[3] == 1 && sensor_values[2] == 1 && sensor_values[1] == 1) {
      Set_Servo_Angle(0); // Gap ve 0 do
      has_grabbed = 1;    // Ngan khong cho gui them lenh servo
      current_case = 1;
    } else if (sensor_sum == 5 && sensor_values[2] == 1) { // Dam bao cam bien trung tam cung tat
      Set_Servo_Angle(0); // Gap ve 0 do
      has_grabbed = 1;    // Ngan khong cho gui them lenh servo
      current_case = 1;
    }
  }

  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

void followLine_Case1(void)
{

  // Dat gia tri PID moi cho Case 1
  Kp = 15;
  Ki = 0.0;
  Kd = 0.0;
  // Dat lai trang thai PID
  integral = 0;
  previous_error = 0;

  // Buoc 1: Day toi 200ms (40 chu ky, moi chu ky 5ms) sau khi gap
      if (!case1_push_after_grab_done) {
          if (case1_delay_count < 110) {
              pwmL = 30; // Toc do co dinh cho thao tac day toi
              pwmR = 30;
              motor_ctr(pwmL, pwmR);
              case1_delay_count++;
              return; // Thoat cho den khi thao tac day toi hoan tat
          } else {
              case1_push_after_grab_done = 1; // Danh dau thao tac day toi da hoan tat
              case1_delay_count = 0; // Dat lai bo dem cho giai doan tiep theo
          }
      }

  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 40;
  } else if (fabs(x) <= 2) {
    basespeed = 38;
  } else if (fabs(x) <= 4) {
    basespeed = 40;
  }

  int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

  // Bo qua sensor_sum == 5 trong 500ms (100 chu ky * 5ms)
  if (case1_delay_count < 100) {
	  case1_delay_count++;
  } else if (sensor_sum == 4) {
    current_case = 2;  // Chuyen sang Case 2 khi ca 5 cam bien deu phat hien line
  }

  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

// Ham followLine_Case2 da chinh sua
void followLine_Case2(void)
{
  if (!case2_stopped) {
    Kp = 2.5;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
      basespeed = 40;
    } else if (fabs(x) <= 2) {
      basespeed = 40;
    } else if (fabs(x) <= 4) {
      basespeed = 40;
    }

    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

    if (case2_ignore_count < 40) {
      case2_ignore_count++;
      pwmL = basespeed + u;
      pwmR = basespeed - u;
      motor_ctr(pwmL, pwmR);
    } else {
      if (sensor_sum == 5 && !case2_stopped) {
        case2_stopped = 1;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
        case2_post_stop_state = 1;
        case2_post_action_count = 0;
      } else if (!case2_stopped) {
        pwmL = basespeed + u;
        pwmR = basespeed - u;
        motor_ctr(pwmL, pwmR);
      }
    }
  } else {
    if (case2_post_stop_state == 1) {
      pwmL = 25;
      pwmR = 25;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 70) {
        case2_post_stop_state = 2;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 2) {
      Set_Servo_Angle(40);
      case2_post_action_count++;
      if (case2_post_action_count >= 20) {
        case2_post_stop_state = 3;
        case2_post_action_count = 0;
      }
    } else if (case2_post_stop_state == 3) {
      pwmL = -35;
      pwmR = -35;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 145) {
        case2_post_stop_state = 4;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 4) {
      pwmL = 70;
      pwmR = 30;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 100) {
        case2_post_stop_state = 5;
        current_case = 3;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    }
  }
}

void followLine_Case2Reset(void)
{
//	Dat_Servo_Angle(0);
  if (!case2_stopped) {
    Kp = 2.5;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
      basespeed = 40;
    } else if (fabs(x) <= 2) {
      basespeed = 40;
    } else if (fabs(x) <= 4) {
      basespeed = 40;
    }

    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

    if (case2_ignore_count < 50) {
      case2_ignore_count++;
      pwmL = basespeed + u;
      pwmR = basespeed - u;
      motor_ctr(pwmL, pwmR);
    } else {
      if (sensor_sum == 5 && !case2_stopped) {
        case2_stopped = 1;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
        case2_post_stop_state = 1;
        case2_post_action_count = 0;
      } else if (!case2_stopped) {
        pwmL = basespeed + u;
        pwmR = basespeed - u;
        motor_ctr(pwmL, pwmR);
      }
    }
  } else {
    if (case2_post_stop_state == 1) {
      pwmL = 25;
      pwmR = 25;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 70) {
        case2_post_stop_state = 2;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 2) {
      Set_Servo_Angle(40);
      case2_post_action_count++;
      if (case2_post_action_count >= 20) {
        case2_post_stop_state = 3;
        case2_post_action_count = 0;
      }
    } else if (case2_post_stop_state == 3) {
      pwmL = -30;
      pwmR = -30;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 145) {
        case2_post_stop_state = 4;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 4) {
      pwmL = 20;
      pwmR = -35;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 110) {
        case2_post_stop_state = 5;

        // Chuyen sang Case 3 va dat lai co Case2Dat lai
        current_case = 3;
        case2_reset_active = 0; // Xoa co Case2Dat lai

        // Dat lai cac bien Case3 de khoi dong moi
        case3_has_grabbed = 0;
        case3_push_after_grab_count = 0;

        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    }
  }
}
void followLine_Case3(void)
{
    if (!case3_stopped) {
        // Dat gia tri PID cho bam line thong thuong
        Kp = 15;
        Ki = 0.0;
        Kd = 0.0;
        integral = 0;
        previous_error = 0;

        x = compute_position();
        u = compute_pid(x);
        if (fabs(x) < 1) {
            basespeed = 35;
        } else if (fabs(x) <= 2) {
            basespeed = 38;
        } else if (fabs(x) <= 4) {
            basespeed = 35;
        }

        int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

        // Bo qua gia tri cam bien ban dau trong 250ms (50 chu ky * 5ms)
        if (case3_ignore_count < 50) {
            case3_ignore_count++;
            pwmL = basespeed + u;
            pwmR = basespeed - u;
            motor_ctr(pwmL, pwmR);
        } else {
            // Kiem tra 4 cam bien phat hien sau thoi gian bo qua
            if (sensor_sum == 4 && !case3_stopped) {
                case3_stopped = 1;
                pwmL = 0;
                pwmR = 0;
                motor_ctr(pwmL, pwmR);
                case3_post_stop_state = 1;
                case3_post_action_count = 0;
            } else if (!case3_stopped) {
                // Tiep tuc bam line binh thuong
                pwmL = basespeed + u;
                pwmR = basespeed - u;
                motor_ctr(pwmL, pwmR);
            }
        }
    } else {
        // Xu ly lan luot cac thao tac sau khi dung
        if (case3_post_stop_state == 1) {
            // Trang thai 1: Dung ngan 100ms (20 chu ky * 5ms)
            pwmL = 0;
            pwmR = 0;
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 20) {
                case3_post_stop_state = 2;
                case3_post_action_count = 0;
            }
        } else if (case3_post_stop_state == 2) {
            // Trang thai 2: Re trai trong 125ms (25 chu ky * 5ms)
            pwmL = -35; // Dong co trai chay lui
            pwmR = 20;  // Dong co phai chay toi
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 100) {
                case3_post_stop_state = 3;
                case3_post_action_count = 0;
            }
        } else if (case3_post_stop_state == 3) {
            // Trang thai 3: Dung cuoi va chuyen sang Case 4
            pwmL = 0;
            pwmR = 0;
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 10) { // Dung cuoi ngan
                // Chuyen sang Case 4
                current_case = 4;

                // Dat lai cac bien Case3 de co the su dung lai sau nay
                case3_stopped = 0;
                case3_ignore_count = 0;
                case3_post_stop_state = 0;
                case3_post_action_count = 0;

                // Dung dong co
                pwmL = 0;
                pwmR = 0;
                motor_ctr(pwmL, pwmR);
            }
        }
    }
}
void followLine_Case4(void)
{
    // Dat gia tri PID moi cho Case4
    Kp = 10;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
        basespeed = 35; // Toc do co ban cham hon
    } else if (fabs(x) <= 2) {
        basespeed = 35;
    } else if (fabs(x) <= 4) {
        basespeed = 35;
    }

    static int grab_state = 0;
    static int forward_count = 0;

    if (grab_state == 0) {
        int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
        if (sensor_sum == 5) {
            Set_Servo_Angle(0); // Gap
            grab_state = 1;
            forward_count = 0;
            pwmL = 20; // Bat dau chay toi cham
            pwmR = 20;
        } else {
            pwmL = basespeed + u;
            pwmR = basespeed - u;
        }
    } else if (grab_state == 1) {
        pwmL = 30; // Chay toi cham
        pwmR = 30;
        motor_ctr(pwmL, pwmR);
        forward_count++;
        if (forward_count >= 500) { // 2 giay (400 chu ky * 5ms)
            grab_state = 2;
            pwmL = 0;
            pwmR = 0;
        }
    } else {
        pwmL = 0;
        pwmR = 0;
    }

    motor_ctr(pwmL, pwmR);
}

/* Hoan thien ham followLine_CaseF */
void followLine_CaseF(void){
    // Dat gia tri PID cho CaseF
    Kp = 18;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    // Doc cam bien va tinh vi tri
    x = compute_position();
    u = compute_pid(x);

    // Dat toc do dua tren sai so vi tri
    if (fabs(x) < 1) {
        basespeed = 40;
    } else if (fabs(x) <= 2) {
        basespeed = 38;
    } else if (fabs(x) <= 4) {
        basespeed = 40;
    }

    // Tinh toc do dong co
    pwmL = basespeed + u;
    pwmR = basespeed - u;
    motor_ctr(pwmL, pwmR);

    // Vi du dieu kien ket thuc: khi tat ca cam bien deu phat hien line (co the sua lai)
    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
    if (sensor_sum == 5) {
        caseF_end_condition = 1;  // Dat co de ket thuc CaseF
    }

    // Hoac them cac dieu kien ket thuc khac neu can
    // Vi du, neu khong phat hien line trong mot khoang thoi gian:
    // if (sensor_sum == 0) {
    //     caseF_ket thuc_dieu kien = 1;
    // }
}

// Ham execute_lenh da chinh sua - PHIEN BAN DA SUA LOI
void execute_command(void)
{
  if (last_command != 0) {
    // Tinh basespeed dung cach - dam bao gia tri luon hop ly
    basespeed = (98 * speed_level) / 9;
    if (basespeed < 50) basespeed = 50; // Muc toi thieu thap hon de dieu khien tot hon

    switch (last_command) {
      case 'F': motor_ctr(basespeed, basespeed); break;
      case 'B': motor_ctr(-basespeed, -basespeed); break;
      case 'L': motor_ctr(-basespeed, basespeed); break;
      case 'R': motor_ctr(basespeed, -basespeed); break;
      case 'G': motor_ctr(basespeed / 2, basespeed); break;
      case 'I': motor_ctr(basespeed, basespeed / 2); break;
      case 'H': motor_ctr(-basespeed / 2, -basespeed); break;
      case 'J': motor_ctr(-basespeed, -basespeed / 2); break;
      case 'S':
        motor_ctr(0, 0);
        last_command = 0; // Xoa lenh sau khi dung
        break;
      // Bo cac lenh toc do khoi day - chung duoc xu ly trong UART callback
      default:
        // Khong xoa last_lenh voi lenh khong xac dinh - tiep tuc chay
        break;
    }
    // Khong tu dong xoa last_lenh - giu lai de robot tiep tuc chuyen dong lien tuc
  } else {
    motor_ctr(0, 0);  // Dung neu khong co lenh
  }
}

/* UART callback da chinh sua - them doan nay vao HAL_UART_RxCpltCallback hien tai */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Kich hoat lai ngat UART

        if (rx_data == 'W') {
            grip = 'W';          // Dong kep
            last_command = 0;
        }
        else if (rx_data == 'w') {
            grip = 'w';          // Mo kep
            last_command = 0;
        }
        else if (rx_data == 'X') {
            // Bat dau Case2Dat lai khi nhan 'X'
            case2_reset_active = 1;
            current_case = 2; // Dat ve Case 2 nhung se su dung Case2Dat lai
            uto = 'u'; // Dat che do tu dong
            last_command = 0;

            // Dat lai tat ca bien Case2 de khoi dong moi
            case2_stopped = 0;
            case2_ignore_count = 0;
            case2_post_stop_state = 0;
            case2_post_action_count = 0;

            // Dat lai PID
            integral = 0;
            previous_error = 0;
            j = 0;
        }
        else if (rx_data == 'V') {
            caseF_active = 1;
            caseF_end_condition = 0;
            uto = 'F';
            last_command = 0;
            integral = 0;
            previous_error = 0;
            j = 0;
            // Dat lai cac bien Case0 va Case1
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'v') {
            caseF_active = 0;
            caseF_end_condition = 0;
            uto = 'U';
            last_command = 0;
            motor_ctr(0, 0);
            // Dat lai cac bien Case0 va Case1
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'u') {
            uto = 'u';
            caseF_active = 0;
            case2_reset_active = 0; // Xoa co Case2Reset
            last_command = 0;
            current_case = 0;
            has_grabbed = 0;
            sensor_count = 0;
            integral = 0;
            previous_error = 0;
            Kp = 2.5;
            Ki = 0.01;
            Kd = 5.0;
            j = 0;
            speed_level = 7;
            Set_Servo_Angle(40);
            case1_delay_count = 0;
            case2_stopped = 0;
            case2_ignore_count = 0;
            case2_post_stop_state = 0;
            case2_post_action_count = 0;
            case3_has_grabbed = 0;
            // Dat lai cac bien Case0 va Case1
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'U') {
            uto = 'U';
            caseF_active = 0;
            case2_reset_active = 0; // Xoa co Case2Reset
            last_command = 0;
            // Dat lai cac bien Case0 va Case1
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (uto == 'U') {
            if (rx_data >= '0' && rx_data <= '9') {
                speed_level = rx_data - '0';
                if (speed_level > 9) speed_level = 9;
            }
            else if (rx_data == 'q') {
                speed_level--;
                if (speed_level < 0) speed_level = 0;
            }
            else {
                last_command = rx_data;
            }
        }
        else if (rx_data >= '0' && rx_data <= '9') {
            speed_level = rx_data - '0';
            if (speed_level > 9) speed_level = 9;
        }
        else if (rx_data == 'q') {
            speed_level--;
            if (speed_level < 0) speed_level = 0;
        }
    }
}

/* Da chinh sua callback Timer de xu ly Case2Dat lai */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (uto == 'u') {
            // Che do tu dong - thuc hien cac Case bam line
            read_sensors();  // Cap nhat gia tri cam bien
            if (current_case == 0) {
                followLine_Case0();
            } else if (current_case == 1) {
                followLine_Case1();
            } else if (current_case == 2) {
                // Kiem tra co su dung Case2Dat lai hay Case2 thong thuong
                if (case2_reset_active) {
                    followLine_Case2Reset();
                } else {
                    followLine_Case2();
                }
            } else if (current_case == 3) {
                followLine_Case3();
            }
            else if (current_case == 4){
            	followLine_Case4();

            }
        }
        else if (uto == 'F') {
            // Che do CaseF - thuc hien bam line CaseF
            read_sensors();      // Cap nhat gia tri cam bien
            followLine_CaseF();  // Thuc thi CaseF

            // Kiem tra CaseF co nen ket thuc va quay ve che do thu cong hay khong
            if (caseF_end_condition) {
                caseF_active = 0;
                caseF_end_condition = 0;
                uto = 'U';       // Quay ve che do thu cong
                last_command = 0;
                motor_ctr(0, 0); // Dung dong co
            }
        }
        else if (uto == 'U') {
            // Che do thu cong
            execute_command();
        }

        // Xu ly lenh kep (hoat dong trong moi che do)
        if (grip == 'W') {
            Set_Servo_Angle(0);  // Dong kep
            grip = 0;            // Xoa sau khi thuc thi
        }
        else if (grip == 'w') {
            Set_Servo_Angle(40); // Mo kep
            grip = 0;            // Xoa sau khi thuc thi
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Ham nay duoc thuc thi khi xay ra loi
  * @ket_qua_tra_ve Khong co
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Nguoi dung co the them xu ly rieng de bao cao trang thai tra ve loi HAL */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Bao cao ten file nguon va so dong nguon
  *         noi xay ra loi assert_param.
  * @param  file: con tro toi ten file nguon
  * @param  line: so dong nguon xay ra loi assert_param
  * @ket_qua_tra_ve Khong co
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
