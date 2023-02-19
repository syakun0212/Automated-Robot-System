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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "oled.h"
#include "ICM_20948_C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
		float kp;
		float ki;
		float kd;
} PIDCoeff_t;

typedef struct
{
		int32_t sum_err;
		int32_t prev_err;
} PIDStorage_t;

typedef struct
{
		float sum_err;
		float prev_err;
} PIDStorageF_t;

typedef struct
{
		float x;
		float y;
		float z;
} IMUAxisF_t;

typedef struct
{
		IMUAxisF_t acc;
		IMUAxisF_t gyr;
		IMUAxisF_t mag;
} IMUDataF_t;

typedef struct
{
		int16_t x;
		int16_t y;
		int16_t z;
} IMUAxisRaw_t;

typedef struct
{
		IMUAxisRaw_t acc;
		IMUAxisRaw_t gyr;
		IMUAxisRaw_t mag;
} IMUDataRaw_t;

typedef struct
{
	float x;
	float y;
	float x_v;
	float y_v;
	float x_a;
	float y_a;
	float yaw;
	float w;
} Pose_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Stability
#define EPSILON ((float)0.001)

// Robot Constants (meters)
#define ROBOT_T ((float)0.1600f)
#define ROBOT_L ((float)0.1450f)
#define WHEEL_R ((float)0.0325f)

// Motor Max Speed (Correspond to count)
#define MOTOR_MAX_CMD ((uint32_t) 8000)
#define MOTOR_MIN_CMD ((uint32_t) 0)

#define MOTOR_MAX_LIMIT ((uint32_t) 6000)

// Encoder
#define ENC_LPF_SIZE 5

typedef struct
{
		int32_t buffer[ENC_LPF_SIZE]; // Data buffer
		int buffer_ind; // Point to current entry to be replaced
		int32_t sum_val; // Current Val
} EncLPF_t;

#define MOTORA_PWM_TO_CMD ((float) 1560.0f)
#define MOTORB_PWM_TO_CMD ((float) 1560.0f)


// Steering
#define STEERING_M ((float) -93.02f)
#define STEERING_C ((float) 146.248f)

// 25 Deg limit
// #define STEERING_ANGLE_LIMIT_MAX ((float) 0.4363)
// #define STEERING_ANGLE_LIMIT_MIN ((float) -0.4363f)
// 30 Deg limit -> 0.25 turning rad
#define STEERING_ANGLE_LIMIT_MAX ((float) 0.5104f)
#define STEERING_ANGLE_LIMIT_MIN ((float) -0.5104f)

// UART
// +: meaning sign (+/-)

// UART RX Message '<+aaaaaa>,<+vvvvvv>\n' + \0 terminate
// <aaaaaa>: aa.aaaa angle in radians
// <vvvvvv>: vv.vvvv velociy in m/s
#define UART_RX_BUFFER_SIZE 16

// UART TX Message '<+aaaaaa>,<+vvvvvv>,<+llllll>,<+rrrrrr>,<+xxxxxx>,<+yyyyyy>,<+wwwww>,<+hhhhhh>,<+ssssss>\n'
// <aaaaaa>: aa.aaaa angle in radians
// <vvvvvv>: vv.vvvv velociy in m/s
// <llllll>: ll.llll velocity in m/s of left wheelw
// <rrrrrr>: rr.rrrr velocity in m/s of right wheel
// <xxxxxx>: xx.xxxx acceleration in foward dir
// <yyyvvy>: yy.yyyy acceleration in left dir
// <wwwwww>: www.www rotation about up axis
// <hhhhhh>: hh.hhhh yaw in radians
// <ssssss>: ss.ssss steering angle measured in radians
#define UART_TX_SIZE 74

// IMU
#define IMU_I2C_ADDR (ICM_20948_I2C_ADDR_AD0 << 1)
#define IMU_I2C_TIMEOUT ((uint32_t) 1000)

#define ACC_LPF_SIZE 20
typedef struct
{
		int32_t buffer[ACC_LPF_SIZE]; // Data buffer
		int buffer_ind; // Point to current entry to be replaced
		int32_t sum_val; // Current Val
} AccLPF_t;

#define GYRO_LPF_SIZE 2
typedef struct
{
		int32_t buffer[GYRO_LPF_SIZE]; // Data buffer
		int buffer_ind; // Point to current entry to be replaced
		int32_t sum_val; // Current Val
} GyroLPF_t;


// Aux Function
#define pdTICKS_TO_S( xTimeInTick ) ( (float) ((float) (xTimeInTick) / (float) configTICK_RATE_HZ) )


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorACmdTask */
osThreadId_t motorACmdTaskHandle;
const osThreadAttr_t motorACmdTask_attributes = {
  .name = "motorACmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for oledDisplayTask */
osThreadId_t oledDisplayTaskHandle;
const osThreadAttr_t oledDisplayTask_attributes = {
  .name = "oledDisplayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for motorCtrlTask */
osThreadId_t motorCtrlTaskHandle;
const osThreadAttr_t motorCtrlTask_attributes = {
  .name = "motorCtrlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for steeringCtrlTas */
osThreadId_t steeringCtrlTasHandle;
const osThreadAttr_t steeringCtrlTas_attributes = {
  .name = "steeringCtrlTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for encReadTask */
osThreadId_t encReadTaskHandle;
const osThreadAttr_t encReadTask_attributes = {
  .name = "encReadTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for uartIntTask */
osThreadId_t uartIntTaskHandle;
const osThreadAttr_t uartIntTask_attributes = {
  .name = "uartIntTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for IMUReadTask */
osThreadId_t IMUReadTaskHandle;
const osThreadAttr_t IMUReadTask_attributes = {
  .name = "IMUReadTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* USER CODE BEGIN PV */

volatile float vel_cmd = 0.0f;
volatile float angle_cmd = 0.0f; //21.5f*M_PI/180.0f;

volatile int32_t motora_enc_vel = 0;
volatile int32_t motorb_enc_vel = 0;

volatile int uart_tx_busy = 0;
volatile int uart_rx_busy = 0;
unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE] = "0000000,0000000\n";

volatile float imu_acc_x = 0.0f;
volatile float imu_acc_y = 0.0f;
volatile float imu_gyro_z = 0.0f;

volatile float steering_yaw = 0.0f;
volatile float steering_angle_measured = 0.0f;

volatile Pose_t estimated_pose;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void heartbeatLED(void *argument);
void motorACmd(void *argument);
void oledDisplay(void *argument);
void motorControl(void *argument);
void steeringControl(void *argument);
void encRead(void *argument);
void uartInterface(void *argument);
void IMURead(void *argument);

/* USER CODE BEGIN PFP */
void setPose(Pose_t *pose, float x, float y, float x_v, float y_v, float x_a, float y_a, float yaw, float w);

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(heartbeatLED, NULL, &heartbeatTask_attributes);

  /* creation of motorACmdTask */
  motorACmdTaskHandle = osThreadNew(motorACmd, NULL, &motorACmdTask_attributes);

  /* creation of oledDisplayTask */
  oledDisplayTaskHandle = osThreadNew(oledDisplay, NULL, &oledDisplayTask_attributes);

  /* creation of motorCtrlTask */
  motorCtrlTaskHandle = osThreadNew(motorControl, NULL, &motorCtrlTask_attributes);

  /* creation of steeringCtrlTas */
  steeringCtrlTasHandle = osThreadNew(steeringControl, NULL, &steeringCtrlTas_attributes);

  /* creation of encReadTask */
  encReadTaskHandle = osThreadNew(encRead, NULL, &encReadTask_attributes);

  /* creation of uartIntTask */
  uartIntTaskHandle = osThreadNew(uartInterface, NULL, &uartIntTask_attributes);

  /* creation of IMUReadTask */
  IMUReadTaskHandle = osThreadNew(IMURead, NULL, &IMUReadTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin
                          |LED_OUTn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTORA_IN2_Pin|MOTORA_IN1_Pin|MOTORB_IN1_Pin|MOTORB_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RES_Pin OLED_DC_Pin
                           LED_OUTn_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin
                          |LED_OUTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTORA_IN2_Pin MOTORA_IN1_Pin MOTORB_IN1_Pin MOTORB_IN2_Pin */
  GPIO_InitStruct.Pin = MOTORA_IN2_Pin|MOTORA_IN1_Pin|MOTORB_IN1_Pin|MOTORB_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void motorCCW(GPIO_TypeDef* port_in_1, uint16_t pin_in_1, GPIO_TypeDef* port_in_2, uint16_t pin_in_2)
{
	HAL_GPIO_WritePin(port_in_1, pin_in_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(port_in_2, pin_in_2, GPIO_PIN_SET);
}

void motorCW(GPIO_TypeDef* port_in_1, uint16_t pin_in_1, GPIO_TypeDef* port_in_2, uint16_t pin_in_2)
{
	HAL_GPIO_WritePin(port_in_1, pin_in_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(port_in_2, pin_in_2, GPIO_PIN_RESET);
}

int32_t processEncoderCount(int ccw, uint32_t count1, uint32_t count2, float dt, uint32_t enc_max)
{
	int32_t diff = 0;

	if(count1 == count2)
	{
		return 0;
	}

	if(ccw)
	{
		if(count1 > count2)
		{
			diff = count1 - count2;
		}
		else
		{
			diff = (enc_max - count2) + count1;
		}
		diff *= -1;
	}
	else
	{
		if(count2 > count1)
		{
			diff = count2 - count1;
		}
		else
		{
			diff = (enc_max - count1) + count2;
		}
	}

	return (int32_t)((float)diff / dt);
}

void initEncLPF(EncLPF_t *lpf, int32_t init_val)
{
	for(int i = 0; i < ENC_LPF_SIZE; ++i)
	{
		lpf->buffer[i] = init_val;
	}

	lpf->buffer_ind = ENC_LPF_SIZE-1;
	lpf->sum_val = init_val * ENC_LPF_SIZE;
}

int32_t processEncLPF(EncLPF_t *lpf, int32_t cur_val)
{
	int ind = lpf->buffer_ind;
	lpf->sum_val = (lpf->sum_val - lpf->buffer[ind]) + cur_val;
	lpf->buffer[ind] = cur_val;
	if(ind == 0)
	{
		lpf->buffer_ind = ENC_LPF_SIZE-1;
	}
	else
	{
		--lpf->buffer_ind;
	}
	return lpf->sum_val/ENC_LPF_SIZE;
}

void initAccLPF(AccLPF_t *lpf, int32_t init_val)
{
	for(int i = 0; i < ACC_LPF_SIZE; ++i)
	{
		lpf->buffer[i] = init_val;
	}

	lpf->buffer_ind = ACC_LPF_SIZE-1;
	lpf->sum_val = init_val * ACC_LPF_SIZE;
}

int32_t processAccLPF(AccLPF_t *lpf, int32_t cur_val)
{
	int ind = lpf->buffer_ind;
	lpf->sum_val = (lpf->sum_val - lpf->buffer[ind]) + cur_val;
	lpf->buffer[ind] = cur_val;
	if(ind == 0)
	{
		lpf->buffer_ind = ACC_LPF_SIZE-1;
	}
	else
	{
		--lpf->buffer_ind;
	}
	return lpf->sum_val/ACC_LPF_SIZE;
}

void initGyroLPF(GyroLPF_t *lpf, int32_t init_val)
{
	for(int i = 0; i < GYRO_LPF_SIZE; ++i)
	{
		lpf->buffer[i] = init_val;
	}

	lpf->buffer_ind = GYRO_LPF_SIZE-1;
	lpf->sum_val = init_val * GYRO_LPF_SIZE;
}

int32_t processGyroLPF(GyroLPF_t *lpf, int32_t cur_val)
{
	int ind = lpf->buffer_ind;
	lpf->sum_val = (lpf->sum_val - lpf->buffer[ind]) + cur_val;
	lpf->buffer[ind] = cur_val;
	if(ind == 0)
	{
		lpf->buffer_ind = GYRO_LPF_SIZE-1;
	}
	else
	{
		--lpf->buffer_ind;
	}
	return lpf->sum_val/GYRO_LPF_SIZE;
}


void forwardKinematic(float vel_cmd, float angle_cmd, float *left_cmd, float *right_cmd)
{

	//float sin_angle2 = sinf(angle_cmd/2.0f);

	float tan_angle = tan(angle_cmd);

	// r = L / (tan(angle))
	// va = v * ( 1 - T/(2*r) )
	// vb = v * ( 1 + T/(2*r) )

	static const float c1 = ROBOT_T / (2 * ROBOT_L);

	*left_cmd = -vel_cmd * (1 - tan_angle*c1) / (2*WHEEL_R*M_PI);
	*right_cmd = vel_cmd * (1 + tan_angle*c1) / (2*WHEEL_R*M_PI);

}

void inverseKinematic(float left_act, float right_act, float *vel_act)
{
	*vel_act = WHEEL_R*(left_act + right_act);
}

int32_t applyPID(const PIDCoeff_t *pid_coef, PIDStorage_t *pid_storage, int32_t cur_err)
{

	int32_t sum_err = cur_err + pid_storage->sum_err;
	int32_t d_err = cur_err - pid_storage->prev_err;

	int32_t cmd = (int32_t)(pid_coef->kp * ((float)cur_err) + pid_coef->ki * ((float)sum_err) + pid_coef->kd * ((float)d_err));
	pid_storage->sum_err = sum_err;
	pid_storage->prev_err = cur_err;

	return cmd;
}

float applyPIDF(const PIDCoeff_t *pid_coef, PIDStorageF_t *pid_storage, float cur_err)
{

	float sum_err = cur_err + pid_storage->sum_err;
	float d_err = cur_err - pid_storage->prev_err;

	float cmd = pid_coef->kp * cur_err + pid_coef->ki * sum_err + pid_coef->kd * d_err;
	pid_storage->sum_err = sum_err;
	pid_storage->prev_err = cur_err;

	return cmd;
}

void sendMotorCommand(GPIO_TypeDef* port_in_1, uint16_t pin_in_1, GPIO_TypeDef* port_in_2, uint16_t pin_in_2, __IO uint32_t *tim_ccr, int32_t cmd)
{
	if(cmd < 0)
	{
		motorCW(port_in_1, pin_in_1, port_in_2, pin_in_2);
	}
	else
	{
		motorCCW(port_in_1, pin_in_1, port_in_2, pin_in_2);
	}

	uint32_t a_cmd = (uint32_t) labs(cmd);

	// Safety
	if(a_cmd > MOTOR_MAX_LIMIT)
	{
		a_cmd = MOTOR_MAX_LIMIT;
	}

	*tim_ccr = a_cmd;
}

int32_t rateLimit(int32_t cur_cmd, int32_t new_cmd, int32_t max_rate)
{
	int32_t increment = new_cmd - cur_cmd;
	if(abs(increment) > max_rate)
	{
		return cur_cmd + ((increment > 0) ? max_rate : -max_rate);
	}
	return new_cmd;

}

float rateLimitF(float cur_cmd, float new_cmd, float max_rate)
{
	float increment = new_cmd - cur_cmd;
	if(abs(increment) > max_rate)
	{
		return cur_cmd + ((increment > 0) ? max_rate : -max_rate);
	}
	return new_cmd;

}

int32_t rateLimitSum(int32_t cur_cmd, int32_t increment, int32_t max_rate)
{
	if(abs(increment) > max_rate)
	{
		return cur_cmd + ((increment > 0) ? max_rate : -max_rate);
	}
	return cur_cmd + increment;

}

float rateLimitSumF(float cur_cmd, float increment, float max_rate)
{
	if(abs(increment) > max_rate)
	{
		return cur_cmd + ((increment > 0) ? max_rate : -max_rate);
	}
	return cur_cmd + increment;

}

uint32_t mapAnglePWM(float angle)
{

	if(angle > STEERING_ANGLE_LIMIT_MAX)
	{
		angle = STEERING_ANGLE_LIMIT_MAX;
	}
	else if(angle < STEERING_ANGLE_LIMIT_MIN)
	{
		angle = STEERING_ANGLE_LIMIT_MIN;
	}

	uint32_t output = (uint32_t) (angle*STEERING_M + STEERING_C);
	return output;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_busy = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int angle;
	int velocity;
	sscanf((char *)uart_rx_buffer, "%7d,%7d\n", &angle, &velocity);

	OLED_ShowString(10, 10, uart_rx_buffer);

	angle_cmd = ((float) angle) / 10000.0f;
	vel_cmd = ((float) velocity) / 10000.0f;
	HAL_UART_Receive_IT(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
}

ICM_20948_Status_e STMWriteI2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, reg, 1, data, len, IMU_I2C_TIMEOUT);
	return ICM_20948_Stat_Ok;

}

ICM_20948_Status_e STMReadI2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, reg, 1, buff, len, IMU_I2C_TIMEOUT);
	return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e IMUChangeBank(uint8_t bank)
{
	if(bank > 3u)
	{
		return ICM_20948_Stat_ParamErr;
	}

	uint8_t b;
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, REG_BANK_SEL, 1, &b, 1, IMU_I2C_TIMEOUT);
	b &= ~0x30;
	b |= (bank << 4);
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, REG_BANK_SEL, 1, &b, 1, IMU_I2C_TIMEOUT);
	return ICM_20948_Stat_Ok;

}

ICM_20948_Status_e setAccelerometer(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{



	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, reg, len, buff, len, IMU_I2C_TIMEOUT);
	return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e readAccelerometerRaw(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	uint8_t temp[6] = {0,0,0,0,0,0};
	stat |= IMUChangeBank(0);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_XOUT_H, 1, &(temp[0]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_XOUT_L, 1, &(temp[1]), 1, IMU_I2C_TIMEOUT);
	*acc_x = ((int16_t)((temp[0] << 8) | (temp[1]))) - offset_x;

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_YOUT_H, 1, &(temp[2]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_YOUT_L, 1, &(temp[3]), 1, IMU_I2C_TIMEOUT);
	*acc_y = ((int16_t)((temp[2] << 8) | (temp[3]))) - offset_y;

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_ZOUT_H, 1, &(temp[4]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_ZOUT_L, 1, &(temp[5]), 1, IMU_I2C_TIMEOUT);
	*acc_z = ((int16_t)((temp[4] << 8) | (temp[5]))) - offset_z;


	return stat;
}

void convertAccelerometer(uint8_t fs_setting, float *acc_x, float *acc_y, float *acc_z, int16_t raw_acc_x, int16_t raw_acc_y, int16_t raw_acc_z)
{
	float fs_scale = 1.0f;
	switch(fs_setting)
	{
		case gpm2:
			fs_scale = 2.0f;
			break;
		case gpm4:
			fs_scale = 4.0f;
			break;
		case gpm8:
			fs_scale = 8.0f;
			break;
		case gpm16:
			fs_scale = 16.0f;
			break;
	}
	*acc_x = fs_scale * ((float) raw_acc_x / 32767.0f);
	*acc_y = fs_scale * ((float) raw_acc_y / 32767.0f);
	*acc_z = fs_scale * ((float) raw_acc_z / 32767.0f);
}

ICM_20948_Status_e readAccelerometer(uint8_t fs_setting, float *acc_x, float *acc_y, float *acc_z, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	int16_t raw_acc_x;
	int16_t raw_acc_y;
	int16_t raw_acc_z;

	stat |= readAccelerometerRaw(&raw_acc_x, &raw_acc_y, &raw_acc_z, offset_x, offset_y, offset_z);
	convertAccelerometer(fs_setting, acc_x, acc_y, acc_z, raw_acc_x, raw_acc_y, raw_acc_z);

	// Old Code
	/*
	uint8_t temp[6] = {0,0,0,0,0,0};
	stat |= IMUChangeBank(2);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_XOUT_H, 1, &(temp[0]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_XOUT_L, 1, &(temp[1]), 1, IMU_I2C_TIMEOUT);
	*acc_x = fs_scale * ((float)((int16_t)((temp[0] << 8) | (temp[1])))) / (32767.0f);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_YOUT_H, 1, &(temp[2]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_YOUT_L, 1, &(temp[3]), 1, IMU_I2C_TIMEOUT);
	*acc_y = fs_scale * ((float)((int16_t)((temp[2] << 8) | (temp[3])))) / (32767.0f);


	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_ZOUT_H, 1, &(temp[4]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_ACCEL_ZOUT_L, 1, &(temp[5]), 1, IMU_I2C_TIMEOUT);
	*acc_z = fs_scale * ((float)((int16_t)((temp[4] << 8) | (temp[5])))) / (32767.0f);

	stat |= IMUChangeBank(0);
	*/

	return stat;
}

ICM_20948_Status_e getAccelerometerOffset(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	int32_t accum_x = 0;
	int32_t accum_y = 0;
	int32_t accum_z = 0;

	int16_t temp_x = 0;
	int16_t temp_y = 0;
	int16_t temp_z = 0;
	for(int i = 0; i < 32; ++i)
	{

		temp_x = 0;
		temp_y = 0;
		temp_z = 0;

		stat |= readAccelerometerRaw(&temp_x, &temp_y, &temp_z, 0, 0, 0);

		accum_x += temp_x;
		accum_y += temp_y;
		accum_z += temp_z;
		osDelay(pdMS_TO_TICKS(15));
	}

	*offset_x = (int16_t) (accum_x >> 5);
	*offset_y = (int16_t) (accum_y >> 5);
	*offset_z = (int16_t) (accum_z >> 5);

	return stat;
}

ICM_20948_Status_e readGyroRaw(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	uint8_t temp[6] = {0,0,0,0,0,0};
	stat |= IMUChangeBank(0);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_XOUT_H, 1, &(temp[0]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_XOUT_L, 1, &(temp[1]), 1, IMU_I2C_TIMEOUT);
	*gyro_x = ((int16_t)((temp[0] << 8) | (temp[1]))) - offset_x;

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_YOUT_H, 1, &(temp[2]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_YOUT_L, 1, &(temp[3]), 1, IMU_I2C_TIMEOUT);
	*gyro_y = ((int16_t)((temp[2] << 8) | (temp[3]))) - offset_y;

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_ZOUT_H, 1, &(temp[4]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_ZOUT_L, 1, &(temp[5]), 1, IMU_I2C_TIMEOUT);
	*gyro_z = ((int16_t)((temp[4] << 8) | (temp[5]))) - offset_z;

	return stat;
}

void convertGyro(uint8_t fs_setting, float *gyro_x, float *gyro_y, float *gyro_z, int16_t raw_gyro_x, int16_t raw_gyro_y, int16_t raw_gyro_z)
{
	float fs_scale = 1;
	switch(fs_setting)
	{
		case dps250:
			fs_scale = 250.0f;
			break;
		case dps500:
			fs_scale = 500.0f;
			break;
		case dps1000:
			fs_scale = 1000.0f;
			break;
		case dps2000:
			fs_scale = 2000.0f;
			break;
	}

	*gyro_x = fs_scale * ((float) raw_gyro_x / 32767.0f);
	*gyro_y = fs_scale * ((float) raw_gyro_y / 32767.0f);
	*gyro_z = fs_scale * ((float) raw_gyro_z / 32767.0f);

}

ICM_20948_Status_e readGyro(uint8_t fs_setting, float *gyro_x, float *gyro_y, float *gyro_z, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;

	stat |= readGyroRaw(&raw_gyro_x, &raw_gyro_y, &raw_gyro_z, offset_x, offset_y, offset_z);
	convertGyro(fs_setting, gyro_x, gyro_y, gyro_z, raw_gyro_x, raw_gyro_y, raw_gyro_z);

	// Old Code
	/*
	uint8_t temp[6] = {0,0,0,0,0,0};
	stat |= IMUChangeBank(2);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_XOUT_H, 1, &(temp[0]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_XOUT_L, 1, &(temp[1]), 1, IMU_I2C_TIMEOUT);
	*gyro_x = fs_scale * ((float)((int16_t)((temp[0] << 8) | (temp[1])))) / (32767.0f);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_YOUT_H, 1, &(temp[2]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_YOUT_L, 1, &(temp[3]), 1, IMU_I2C_TIMEOUT);
	*gyro_y = fs_scale * ((float)((int16_t)((temp[2] << 8) | (temp[3])))) / (32767.0f);

	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_ZOUT_H, 1, &(temp[4]), 1, IMU_I2C_TIMEOUT);
	stat |= HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_GYRO_ZOUT_L, 1, &(temp[5]), 1, IMU_I2C_TIMEOUT);
	*gyro_z = fs_scale * ((float)((int16_t)((temp[4] << 8) | (temp[5])))) / (32767.0f);

	stat |= IMUChangeBank(0);
	*/

	return stat;
}

ICM_20948_Status_e getGyroOffset(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z)
{
	ICM_20948_Status_e stat = ICM_20948_Stat_Ok;

	int32_t accum_x = 0;
	int32_t accum_y = 0;
	int32_t accum_z = 0;

	int16_t temp_x = 0;
	int16_t temp_y = 0;
	int16_t temp_z = 0;
	for(int i = 0; i < 32; ++i)
	{

		temp_x = 0;
		temp_y = 0;
		temp_z = 0;

		stat |= readGyroRaw(&temp_x, &temp_y, &temp_z, 0, 0, 0);

		accum_x += temp_x;
		accum_y += temp_y;
		accum_z += temp_z;
		osDelay(pdMS_TO_TICKS(15));
	}

	*offset_x = (int16_t) (accum_x >> 5);
	*offset_y = (int16_t) (accum_y >> 5);
	*offset_z = (int16_t) (accum_z >> 5);

	return stat;
}

void setPose(Pose_t *pose, float x, float y, float x_v, float y_v, float x_a, float y_a, float yaw, float w)
{
	pose->x = x;
	pose->y = y;
	pose->x_v = x_v;
	pose->y_v = y_v;
	pose->x_a = x_a;
	pose->y_a = y_a;
	pose->yaw = yaw;
	pose->w = w;
};

/* USER CODE END 4 */

/* USER CODE BEGIN Header_heartbeatLED */
/**
  * @brief  Function implementing the heartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_heartbeatLED */
void heartbeatLED(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
  	HAL_GPIO_TogglePin(LED_OUTn_GPIO_Port, LED_OUTn_Pin);
    osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motorACmd */
/**
* @brief Function implementing the motorACmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorACmd */
void motorACmd(void *argument)
{
  /* USER CODE BEGIN motorACmd */

  // Init Motor A


  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  uint32_t cmd_val = 0;
  UNUSED(cmd_val);

  for(;;)
  {
  	/*
  	motorCW(MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin, MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin);

  	while(cmd_val < MOTOR_MAX_CMD)
  	{
  		htim8.Instance->CCR1 = cmd_val;
  		++cmd_val;
  		osDelay(1);
  	}

  	while(cmd_val > MOTOR_MIN_CMD)
		{
			htim8.Instance->CCR1 = cmd_val;
			--cmd_val;
			osDelay(1);
		}

  	motorCCW(MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin, MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin);
  	while(cmd_val < MOTOR_MAX_CMD)
		{
			htim8.Instance->CCR1 = cmd_val;
			++cmd_val;
			osDelay(1);
		}
  	while(cmd_val > MOTOR_MIN_CMD)
  	{
  		htim8.Instance->CCR1 = cmd_val;
  		--cmd_val;
  		osDelay(1);
  	}
	 */
  	//motorCW(MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin, MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin);
  	//htim8.Instance->CCR1 = 2000;

  	osDelay(500);
  }
  /* USER CODE END motorACmd */
}

/* USER CODE BEGIN Header_oledDisplay */
/**
* @brief Function implementing the oledDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oledDisplay */
void oledDisplay(void *argument)
{
  /* USER CODE BEGIN oledDisplay */

	uint8_t buffer[20] = "Hello World!\n";


  for(;;)
  {
  	//OLED_ShowString(10, 10, buffer);
  	OLED_Refresh_Gram();
    osDelay(pdMS_TO_TICKS(250));
  }
  /* USER CODE END oledDisplay */
}

/* USER CODE BEGIN Header_motorControl */
/**
* @brief Function implementing the motorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorControl */
void motorControl(void *argument)
{
  /* USER CODE BEGIN motorControl */

	float target_vel = 0.0;
	float target_angle = 0.0;

	const uint32_t ERROR_THRESHOLD = 2;

	// Init motor A
	const float MOTORA_MAX_ACC = 1.5f * MOTORA_PWM_TO_CMD;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	float motora_targetf = 0.0f;
	int32_t motora_target_pwm = 0;
	int32_t motora_err = 0;

	PIDCoeff_t motora_pid_coef;
	// PID1
	/*
	motora_pid_coef.kp = 1.75f;
	motora_pid_coef.ki = 0.135f;
	motora_pid_coef.kd = 6.518f;
	*/
	// PID2

	motora_pid_coef.kp = 0.0396;
	motora_pid_coef.ki = 0.0294f;
	motora_pid_coef.kd = 0.00f;


	PIDStorage_t motora_pid_storage;
	motora_pid_storage.sum_err = 0;
	motora_pid_storage.prev_err = 0;

	int32_t motora_cur_cmd = 0;


	// Init motor B
	const float MOTORB_MAX_ACC = 1.5f * MOTORB_PWM_TO_CMD;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	float motorb_targetf = 0.0f;
	int32_t motorb_target_pwm = 0;
	int32_t motorb_err = 0;

	PIDCoeff_t motorb_pid_coef;
	// PID1
	/*
	motorb_pid_coef.kp = 1.75f;
	motorb_pid_coef.ki = 0.123f;
	motorb_pid_coef.kd = 6.747f;
	*/
	// PID2 Oscillate low load
	/*
	motorb_pid_coef.kp = 0.160f;
	motorb_pid_coef.ki = 0.0335f;
	motorb_pid_coef.kd = -3.232f;
	*/


	motorb_pid_coef.kp = 0.04f;
	motorb_pid_coef.ki = 0.0294f;
	motorb_pid_coef.kd = 0.00f;



	PIDStorage_t motorb_pid_storage;
	motorb_pid_storage.sum_err = 0;
	motorb_pid_storage.prev_err = 0;

	int32_t motorb_cur_cmd = 0;

	uint8_t displayBuffer[20] = "Starting..\0";

	uint32_t dt = 0;
	float dts = 0.0f;
	uint32_t tick = osKernelGetTickCount();

  for(;;)
  {

  	// Read Command
  	target_vel = vel_cmd;
  	target_angle = angle_cmd;

  	// Calculate Target Velocity for each wheels
  	forwardKinematic(target_vel, target_angle, &motora_targetf, &motorb_targetf);

  	motora_target_pwm = (int32_t) (motora_targetf * MOTORA_PWM_TO_CMD); // PWM in rev/sec
		motorb_target_pwm = (int32_t) (motorb_targetf * MOTORB_PWM_TO_CMD);


		dt = osKernelGetTickCount() - tick;
		dts = pdTICKS_TO_S(dt);

  	// Calculate Cmd Motor A
		motora_err = motora_target_pwm - motora_enc_vel;
		if(((uint32_t) abs(motora_err)) < ERROR_THRESHOLD)
		{
			motora_err = 0;
		}

		//motora_cur_cmd = rateLimitSum(motora_cur_cmd, applyPID(&motora_pid_coef, &motora_pid_storage, motora_err), (int32_t) (dts*MOTORA_MAX_ACC));
		motora_cur_cmd = rateLimit(motora_cur_cmd, applyPID(&motora_pid_coef, &motora_pid_storage, motora_err), (int32_t) (dts*MOTORA_MAX_ACC));
		//motora_cur_cmd = motora_target_pwm;
		//motora_cur_cmd = applyPID(&motora_pid_coef, &motora_pid_storage, motora_err);


		// Calculate Cmd Motor B
		motorb_err = motorb_target_pwm - motorb_enc_vel;
		if(((uint32_t) abs(motorb_err)) < ERROR_THRESHOLD)
		{
			motorb_err = 0;
		}

		//motorb_cur_cmd = rateLimitSum(motorb_cur_cmd, applyPID(&motorb_pid_coef, &motorb_pid_storage, motorb_err), (int32_t) (dts*MOTORB_MAX_ACC));
		motorb_cur_cmd = rateLimit(motorb_cur_cmd, applyPID(&motorb_pid_coef, &motorb_pid_storage, motorb_err), (int32_t) (dts*MOTORB_MAX_ACC));
		//motorb_cur_cmd = motorb_target_pwm;
		//motorb_cur_cmd = applyPID(&motorb_pid_coef, &motorb_pid_storage, motorb_err);

		// Drive Motor A
		sendMotorCommand(MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin, MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin, &(htim8.Instance->CCR1), motora_cur_cmd);

		// Drive Motor B
		sendMotorCommand(MOTORB_IN1_GPIO_Port, MOTORB_IN1_Pin, MOTORB_IN2_GPIO_Port, MOTORB_IN2_Pin, &(htim8.Instance->CCR2), -1*motorb_cur_cmd); // -1 for reverse factor

		tick = osKernelGetTickCount();

		// Motor A Display
		/*
		sprintf((char*)displayBuffer, "%10ld", motora_target_pwm);
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_enc_vel);
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_err);
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_cur_cmd);
		OLED_ShowString(10, 50, displayBuffer);
		*/

		// Motor B Display
		/*
		sprintf((char*)displayBuffer, "%10ld", motorb_target_pwm);
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_enc_vel);
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_err);
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_cur_cmd);
		OLED_ShowString(10, 50, displayBuffer);
		*/

		// Motor AB Display (CMD - VEL)
		/*
		sprintf((char*)displayBuffer, "%10ld", motora_cur_cmd);
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_enc_vel);
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_cur_cmd);
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_enc_vel);
		OLED_ShowString(10, 50, displayBuffer);
		*/



		// Motor AB Display (Target - Actual 2 decimal place)
		/*
		sprintf((char*)displayBuffer, "%10ld", (int32_t)(10000.0f * motora_targetf));
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", (int32_t)(10000.0f * motora_enc_vel / MOTORA_PWM_TO_CMD));
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", (int32_t)(10000.0f * motorb_targetf));
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", (int32_t)(10000.0f * motorb_enc_vel / MOTORB_PWM_TO_CMD));
		OLED_ShowString(10, 50, displayBuffer);
		*/


		/*
		if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit(&huart1, displayBuffer, 20, 100000);

		}
		*/


    osDelay(pdMS_TO_TICKS(15));
  }
  /* USER CODE END motorControl */
}

/* USER CODE BEGIN Header_steeringControl */
/**
* @brief Function implementing the steeringCtrlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_steeringControl */
void steeringControl(void *argument)
{
  /* USER CODE BEGIN steeringControl */

	// Init Steering
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	float cur_yaw = 0.0f;
	float cur_w = 0.0f;

	float expected_yaw = 0.0f;

	float target_angle = angle_cmd;

	float vl = 0.0f;
	float vr = 0.0f;
	float vel = 0.0f;

	float d_angle = 0.0f;

	float error = 0.0f;
	float angle_ctrl = 0.0f;

	// Set 1
	/*
	PIDCoeff_t servo_pid_coef;
	servo_pid_coef.kp = 0.3f;
	servo_pid_coef.ki = 0.00f;
	servo_pid_coef.kd = 0.00f;


	PIDStorageF_t servo_pid_storage;
	servo_pid_storage.sum_err = 0.0f;
	servo_pid_storage.prev_err = 0.0f;
	*/


	// Set 2
	/*
	PIDCoeff_t servo_pid_coef;
	servo_pid_coef.kp = 0.2f; // 0.5
	servo_pid_coef.ki = 0.3f; // 0.1
	servo_pid_coef.kd = 0.1f;


	PIDStorageF_t servo_pid_storage;
	servo_pid_storage.sum_err = 0.0f;
	servo_pid_storage.prev_err = 0.0f;
	*/

	PIDCoeff_t servo_pid_coef;
	servo_pid_coef.kp = 0.05f; // 0.5
	servo_pid_coef.ki = 0.012f; // 0.1
	servo_pid_coef.kd = 0.0f;


	PIDStorageF_t servo_pid_storage;
	servo_pid_storage.sum_err = 0.0f;
	servo_pid_storage.prev_err = 0.0f;

	const float RATE_LIMIT = 0.0175f;
	float l = 0.0f;

	float d = 0.0f;
	float act_r = 0.0f;
	float exp_r = 0.0f;

	float last_exp_yaw = 0.0f;
	float last_act_yaw = 0.0f;

	const float correction_d = 0.005;

	float target_w = 0.0f;

	uint32_t last_tick = osKernelGetTickCount();
	uint32_t dt = 0;
	float dtS = 0.0;

	uint8_t displayBuffer[20] = "Starting..\0";

	const int UPDATE_COUNT = 6;
	int cur_iteration = 0;

	const int PERIOD = 5; //5ms

	/* Infinite loop */
	for(;;)
	{
		dt = osKernelGetTickCount() - last_tick;
		vl = ((float) motora_enc_vel) * 2*M_PI*WHEEL_R/ MOTORA_PWM_TO_CMD;
		vr = ((float) motorb_enc_vel) * 2*M_PI*WHEEL_R/ MOTORB_PWM_TO_CMD;

		vel =  (-vl+vr) / 2.0f;
		d += vel * dt;

		//cur_w = (vr - (-vl)) / ROBOT_T;
		cur_w = imu_gyro_z * M_PI/180;

		dtS = pdTICKS_TO_S(dt);

		cur_yaw += cur_w*dtS;
		steering_yaw = cur_yaw;

		target_w = tan(angle_cmd) * vel / ROBOT_L;
		expected_yaw += dtS * target_w;


		last_tick = osKernelGetTickCount();

		if(cur_iteration > UPDATE_COUNT)
		{

			/*
			l = vel * 0.05;
			error = (expected_yaw - cur_yaw)*ROBOT_L;
			if(fabs(l) > 0.002)
			{
				error = atanf(error/l);
			}
			else
			{
				error = 0.0f;
			}


			//angle_ctrl = angle_cmd + kp*error;
			//angle_ctrl = rateLimitSumF(angle_ctrl, applyPIDF(&servo_pid_coef, &servo_pid_storage, error), RATE_LIMIT);

			if(target_angle != angle_cmd)
			{
				angle_ctrl = angle_cmd;
				//target_angle = angle_cmd;
			}
			else
			{
				angle_ctrl = rateLimitSumF(angle_cmd, applyPIDF(&servo_pid_coef, &servo_pid_storage, error), RATE_LIMIT);
			}

			htim1.Instance->CCR4 = mapAnglePWM(angle_ctrl);
			*/


			//exp_r = (expected_yaw - last_exp_yaw) / d;
			//act_r = (cur_yaw - last_act_yaw) / d;
			//error = (expected_yaw - cur_yaw) * ROBOT_L / d;
			//error = (expected_yaw - last_exp_yaw) - (cur_yaw - last_act_yaw);
			error = expected_yaw - cur_yaw;
			if(target_angle != angle_cmd)
			{
				target_angle = angle_cmd;
				angle_ctrl = angle_cmd;
				servo_pid_storage.prev_err = 0.0f;
				servo_pid_storage.sum_err = 0.0f;
			}
			else
			{
				if(fabs(vel) > 0.01)
				{
					angle_ctrl = rateLimitF(angle_ctrl, atanf(ROBOT_L * (target_w + fabs(vel) * applyPIDF(&servo_pid_coef, &servo_pid_storage, error) / correction_d)/vel), RATE_LIMIT);
					steering_angle_measured = (cur_yaw - last_act_yaw);
				}
			}
			htim1.Instance->CCR4 = mapAnglePWM(angle_ctrl);

			last_act_yaw = cur_yaw;
			last_exp_yaw = expected_yaw;
			d = 0.0f;

			sprintf((char*)displayBuffer, "%10d", (int) (1000*expected_yaw*180.0f/M_PI));
			OLED_ShowString(10, 20, displayBuffer);
			sprintf((char*)displayBuffer, "%10d", (int) (1000*cur_yaw*180.0f/M_PI));
			OLED_ShowString(10, 30, displayBuffer);
			sprintf((char*)displayBuffer, "%10d", (int) (1000*error*180.0f/M_PI));
			OLED_ShowString(10, 40, displayBuffer);
			sprintf((char*)displayBuffer, "%10d", (int) (1000*angle_ctrl*180.0f/M_PI));
			OLED_ShowString(10, 50, displayBuffer);

			//expected_yaw = 0.0f;
			//cur_yaw = 0.0f;

			cur_iteration = 0;
		}
		else
		{
			cur_iteration += 1;
		}

		osDelay(pdMS_TO_TICKS(PERIOD));
	}
  /* USER CODE END steeringControl */
}

/* USER CODE BEGIN Header_encRead */
/**
* @brief Function implementing the encReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encRead */
void encRead(void *argument)
{
  /* USER CODE BEGIN encRead */

	const uint32_t SPEED_FILTER = 10000;

	// Motor A
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	uint32_t motora_count1 = htim2.Instance->CNT;
	uint32_t motora_count2 = htim2.Instance->CNT;
	int32_t motora_diff = 0; // Allowable due to conversion
	uint32_t motora_tick = osKernelGetTickCount();
	uint32_t motora_dt = 0;
	int motora_ccw = 0;

	EncLPF_t motora_lpf;
	initEncLPF(&motora_lpf, 0);

	// Motor B
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	uint32_t motorb_count1 = htim3.Instance->CNT;
	uint32_t motorb_count2 = htim3.Instance->CNT;
	int32_t motorb_diff = 0; // Allowable due to conversion
	uint32_t motorb_tick = osKernelGetTickCount();
	uint32_t motorb_dt = 0;
	int motorb_ccw = 0;

	EncLPF_t motorb_lpf;
	initEncLPF(&motorb_lpf, 0);

	int first_run = ENC_LPF_SIZE;

	uint8_t displayBuffer[20] = "Starting..\0";

  for(;;)
  {
  	// Encoder
		taskENTER_CRITICAL();
		// Get Motor A Current Data
		motora_dt = osKernelGetTickCount() - motora_tick;
		motora_count2 = htim2.Instance->CNT;
		motora_ccw = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

		// Get Motor B Current Data
		motorb_dt = osKernelGetTickCount() - motorb_tick;
		motorb_count2 = htim3.Instance->CNT;
		motorb_ccw = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
		taskEXIT_CRITICAL();

		// Calculate  Motor A Encoder
		motora_diff = processEncoderCount(motora_ccw, motora_count1, motora_count2, pdTICKS_TO_S(motora_dt), UINT32_MAX); // -1 for reverse factor
		motora_count1 = htim2.Instance->CNT;
		motora_tick = osKernelGetTickCount();

		// Calculate Motor B Encoder
		motorb_diff = processEncoderCount(motorb_ccw, motorb_count1, motorb_count2, pdTICKS_TO_S(motorb_dt), (uint32_t) UINT16_MAX);
		motorb_count1 = htim3.Instance->CNT;
		motorb_tick = osKernelGetTickCount();

		if(((uint32_t) abs(motora_diff)) > SPEED_FILTER)
		{
			if(motora_diff > 0)
			{
				motora_diff = SPEED_FILTER;
			}
			else
			{
				motora_diff = -SPEED_FILTER;
			}
		}

		if(((uint32_t) abs(motorb_diff)) > SPEED_FILTER)
		{
			if(motorb_diff > 0)
			{
				motorb_diff = SPEED_FILTER;
			}
			else
			{
				motorb_diff = -SPEED_FILTER;
			}
		}

		if(first_run > 0)
		{
			--first_run;

			processEncLPF(&motora_lpf, motora_diff);
			processEncLPF(&motorb_lpf, motorb_diff);

			osDelay(pdMS_TO_TICKS(30));
			continue;
		}

		motora_enc_vel = processEncLPF(&motora_lpf, motora_diff);
		motorb_enc_vel = processEncLPF(&motorb_lpf, motorb_diff);


		// Show AB Position
		/*
		sprintf((char*)displayBuffer, "%10lu", motora_count1);
		OLED_ShowString(10, 20, displayBuffer);

		sprintf((char*)displayBuffer, "%10lu", motorb_count1);
		OLED_ShowString(10, 30, displayBuffer);
		 */

		// Show AB Speed
		/*
		sprintf((char*)displayBuffer, "%10ld", motora_diff);
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_enc_vel);
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_diff);
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_enc_vel);
		OLED_ShowString(10, 50, displayBuffer);
		*/


		// Show AB Count
		/*
		sprintf((char*)displayBuffer, "%10ld", motora_count1);
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motora_count2);
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_count1);
		OLED_ShowString(10, 40, displayBuffer);
		sprintf((char*)displayBuffer, "%10ld", motorb_count2);
		OLED_ShowString(10, 50, displayBuffer);
		*/


    osDelay(pdMS_TO_TICKS(30));
  }
  /* USER CODE END encRead */
}

/* USER CODE BEGIN Header_uartInterface */
/**
* @brief Function implementing the uartIntTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartInterface */
void uartInterface(void *argument)
{
  /* USER CODE BEGIN uartInterface */
  /* Infinite loop */

	uint32_t tx_tick = osKernelGetTickCount();

  for(;;)
  {
  	if(osKernelGetTickCount() - tx_tick > 15)
  	{
  		if(!uart_tx_busy)
			{

				unsigned char send_data[UART_TX_SIZE];
				sprintf((char *)send_data, "%+07d,%+07d,%+07d,%+07d,%+07d,%+07d,%+07d,%+07d,%+7d\n",
						(int)(10000.0f * angle_cmd),
						(int)(10000.0f * vel_cmd),
						(int)(10000.0f * ((float) motora_enc_vel) * 2*M_PI*WHEEL_R/ MOTORA_PWM_TO_CMD),
						(int)(10000.0f * ((float) motorb_enc_vel) * 2*M_PI*WHEEL_R/ MOTORB_PWM_TO_CMD),
						(int)(10000.0f * imu_acc_x),
						(int)(10000.0f * imu_acc_y),
						(int)(1000.0f * imu_gyro_z),
						(int)(10000.0f * steering_yaw),
						(int)(10000.0f * steering_angle_measured));

				taskENTER_CRITICAL();
				HAL_UART_Transmit_IT(&huart3, send_data, UART_TX_SIZE-1);
				uart_tx_busy = 1;
				tx_tick = osKernelGetTickCount();
				taskEXIT_CRITICAL();
			}
  	}

    osDelay(2);
  }
  /* USER CODE END uartInterface */
}

/* USER CODE BEGIN Header_IMURead */
/**
* @brief Function implementing the IMUReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMURead */
void IMURead(void *argument)
{
  /* USER CODE BEGIN IMURead */

	uint8_t displayBuffer[20] = "IMU..\0";

	const uint32_t WAIT_DELAY = 250u;

	ICM_20948_Device_t imu;

	ICM_20948_Serif_t serif;
	serif.read = STMReadI2C;
	serif.write = STMWriteI2C;
	serif.user = NULL;

	ICM_20948_init_struct(&imu);
	ICM_20948_link_serif(&imu, &serif);

	ICM_20948_Status_e stat = ICM_20948_Stat_Err;
	uint8_t whoami = 0x00;

	// Wait for connection to establish
	while ((stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI))
	{
		whoami = 0x00;
		stat = IMUChangeBank(0u);
		HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_WHO_AM_I, 1, &whoami, 1, IMU_I2C_TIMEOUT);

		sprintf((char*)displayBuffer, "whoami %02x", whoami);
		OLED_ShowString(10, 10, displayBuffer);
		osDelay(pdMS_TO_TICKS(50));
	}

	// Setup according Wheeltec

	ICM_20948_smplrt_t sample_rate;
	sample_rate.a = (uint16_t) 0x0007;
	sample_rate.g = (uint8_t) 0x04;


	ICM_20948_dlpcfg_t lpf_config;
	lpf_config.a = 0x06;
	lpf_config.g = 0x03;

	ICM_20948_fss_t full_scale_setting;
	full_scale_setting.a = gpm2;
	full_scale_setting.g = dps500;

	// Reset
	uint8_t reg_val;
	IMUChangeBank(0u);
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val |= 0x80u;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	do
	{
		osDelay(pdMS_TO_TICKS(WAIT_DELAY));
		HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	} while(reg_val != 0x41);

	// Wake Up
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val |= 0x05u;
	reg_val &= ~0x60u;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);


	// Set Sample Mode
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB0_REG_LP_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val &= ~0x70u;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB0_REG_LP_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);


	IMUChangeBank(2u);
	// Set Sample Rate Accelerator
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_SMPLRT_DIV_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val &= ~0x0Fu;
	reg_val |= (uint8_t) ((sample_rate.a & 0x0F00) >> 8);
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_SMPLRT_DIV_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val = (uint8_t) (sample_rate.a & 0x00FF);
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_SMPLRT_DIV_2, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	// Set Accelerometer
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val |= (0x01u | (full_scale_setting.a << 1) | (lpf_config.a << 3));
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	// Set Sample Rate Gyro
	reg_val = sample_rate.g;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_GYRO_SMPLRT_DIV, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	// Set Gyro
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val |= (0x01u | (full_scale_setting.g << 1) | (lpf_config.g << 3));
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	// Set ODR
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ODR_ALIGN_EN, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	reg_val |= 0x01u;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ODR_ALIGN_EN, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, REG_BANK_SEL, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	IMUChangeBank(0u);
	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, REG_BANK_SEL, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_LP_CONFIG, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	IMUChangeBank(2u);
	//HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, 1, &reg_val, 1, IMU_I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, AGB2_REG_ACCEL_SMPLRT_DIV_2, 1, &reg_val, 1, IMU_I2C_TIMEOUT);

	sprintf((char*)displayBuffer, "K: %u     ", reg_val);
	OLED_ShowString(10, 10, displayBuffer);

	osDelay(pdMS_TO_TICKS(50));

	if(stat != ICM_20948_Stat_Ok)
	{
		sprintf((char*)displayBuffer, "Error");
		OLED_ShowString(10, 10, displayBuffer);
	}
	else
	{
		sprintf((char*)displayBuffer, "Success");
		//sprintf((char*)displayBuffer, "R: %u %u %u ", set_status, accel_config, gyro_config);
		OLED_ShowString(10, 10, displayBuffer);
	}



	osDelay(pdMS_TO_TICKS(1000));

	IMUDataRaw_t imu_data_raw = {{0,0,0},{0,0,0},{0,0,0}};
	IMUDataF_t imu_data = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
	IMUDataRaw_t imu_offset = {{0,0,0},{0,0,0},{0,0,0}};

	getAccelerometerOffset(&imu_offset.acc.x, &imu_offset.acc.y, &imu_offset.acc.z);
	getGyroOffset(&imu_offset.gyr.x, &imu_offset.gyr.y, &imu_offset.gyr.z);

	AccLPF_t lpf_acc_x;
	AccLPF_t lpf_acc_y;

	GyroLPF_t lpf_gyro_z;

	initAccLPF(&lpf_acc_x, 0);
	initAccLPF(&lpf_acc_y, 0);

	initGyroLPF(&lpf_gyro_z, 0);

	int count = 0;

  for(;;)
  {

  	//readAccelerometer(full_scale_setting.a, &imu_data.acc.x, &imu_data.acc.y, &imu_data.acc.z, imu_offset.acc.x, imu_offset.acc.y, imu_offset.acc.z);
		//readGyro(full_scale_setting.g, &imu_data.gyr.x, &imu_data.gyr.y, &imu_data.gyr.z, imu_offset.gyr.x, imu_offset.gyr.y, imu_offset.gyr.z);

  	readAccelerometerRaw(&imu_data_raw.acc.x, &imu_data_raw.acc.y, &imu_data_raw.acc.z, imu_offset.acc.x, imu_offset.acc.y, imu_offset.acc.z);
  	readGyroRaw(&imu_data_raw.gyr.x, &imu_data_raw.gyr.y, &imu_data_raw.gyr.z, imu_offset.gyr.x, imu_offset.gyr.y, imu_offset.gyr.z);

  	imu_data_raw.acc.x = processAccLPF(&lpf_acc_x, imu_data_raw.acc.x);
  	imu_data_raw.acc.y = processAccLPF(&lpf_acc_y, imu_data_raw.acc.y);
  	imu_data_raw.gyr.z = processGyroLPF(&lpf_gyro_z, imu_data_raw.gyr.z);

  	convertAccelerometer(full_scale_setting.a, &imu_data.acc.x, &imu_data.acc.y, &imu_data.acc.z, imu_data_raw.acc.x, imu_data_raw.acc.y, imu_data_raw.acc.z);
  	convertGyro(full_scale_setting.g, &imu_data.gyr.x, &imu_data.gyr.y, &imu_data.gyr.z, imu_data_raw.gyr.x, imu_data_raw.gyr.y, imu_data_raw.gyr.z);

  	imu_acc_x = imu_data.acc.x;
		imu_acc_y = imu_data.acc.y;
		imu_gyro_z = imu_data.gyr.z;

		/*
  	readAccelerometer(full_scale_setting.a, &imu_data.acc.x, &imu_data.acc.y, &imu_data.acc.z, imu_offset.acc.x, imu_offset.acc.y, imu_offset.acc.z);
  	readGyro(full_scale_setting.g, &imu_data.gyr.x, &imu_data.gyr.y, &imu_data.gyr.z, imu_offset.gyr.x, imu_offset.gyr.y, imu_offset.gyr.z);

  	imu_acc_x = imu_data.acc.x;
  	imu_acc_y = imu_data.acc.y;
  	imu_gyro_z = imu_data.gyr.z;
  	*/

  	// IMU Acc + Gyro
  	/*
		sprintf((char*)displayBuffer, "%6d %6d", (int16_t)(imu_data.acc.x * 1000.0f), (int16_t)(imu_data.gyr.x * 10.0f));
		OLED_ShowString(10, 20, displayBuffer);
		sprintf((char*)displayBuffer, "%6d %6d", (int16_t)(imu_data.acc.y * 1000.0f), (int16_t)(imu_data.gyr.x * 10.0f));
		OLED_ShowString(10, 30, displayBuffer);
		sprintf((char*)displayBuffer, "%6d %6d", (int16_t)(imu_data.acc.z * 1000.0f), (int16_t)(imu_data.gyr.z * 10.0f));
		OLED_ShowString(10, 40, displayBuffer);
		*/

    osDelay(pdMS_TO_TICKS(5));
  }
  /* USER CODE END IMURead */
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
  	//HAL_GPIO_TogglePin(LED_OUTn_GPIO_Port, LED_OUTn_Pin);
  	//HAL_Delay(5000);
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

