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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>// <-- ADDED: For serial messages
#include <stdio.h>
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "Ekf.h"
#include "lqr.h"



//#include "Robot.h"



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
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId VelocityTaskHandle;
osThreadId BalanceTaskHandle;
/* USER CODE BEGIN PV */

// Macros
//#define TIMER_FREQ_HZ   80000   // was 40000
//#define MAX_SPEED_HZ    28000
#define MAX_SPEED_HZ    28000   // was 28000
#define TIMER_FREQ_HZ 70021

#define BALANCE_TASK_LOOP_MS 5
#define VELOCITY_TASK_LOOP_MS 20 // 50 Hz
#define USTEP_8              8
#define USTEP_16             16

// Constants for controlling the stepper motors
#define STEPS_PER_REV     3200        // 200 steps × 16 microsteps — match your DM556T DIP switches
#define STEPS_PER_RAD     (STEPS_PER_REV / (2.0f * 3.14159f))  // ~509.3 steps/rad
#define TORQUE_TO_OMEGA   1.5f    /* (rad/s) / Nm — tune this */
#define ENA_DEADBAND_HZ  150   // tune this — below this speed, disable motor
//#define TORQUE_TO_OMEGA   0.5f    /* (rad/s) / Nm — tune this */
//#define TORQUE_TO_OMEGA   1.0f
//#define TORQUE_TO_OMEGA   1.5f
//#define TORQUE_TO_OMEGA   4.0f
//#define TORQUE_TO_OMEGA   2.5f
//#define TORQUE_TO_OMEGA   3.0f
//#define TORQUE_TO_OMEGA   4.0f
//#define TORQUE_TO_OMEGA   5.0f

#define ENCODER_CPR  4000.0f


// ═══════════════════════════════════════════════════════════
// PHYSICAL GEOMETRY PARAMETERS
// ═══════════════════════════════════════════════════════════
#define BALL_DIAMETER_MM    350.0f
#define WHEEL_DIAMETER_MM   125.0f
#define RK                  (BALL_DIAMETER_MM  / 2000.0f)  // = 0.175  m
#define RW                  (WHEEL_DIAMETER_MM / 2000.0f)  // = 0.0625 m
#define RW_OVER_RK          (RW / RK)                      // = 0.35714

#define ZENITH_ANGLE_DEG    45.0f
#define ALPHA_RAD           (ZENITH_ANGLE_DEG * 3.14159265f / 180.0f)  // = 0.7854 rad

#define GAMMA1_DEG          120.0f
#define GAMMA2_DEG          240.0f
#define GAMMA3_DEG          0.0f
#define GAMMA1_RAD          (GAMMA1_DEG * 3.14159265f / 180.0f)
#define GAMMA2_RAD          (GAMMA2_DEG * 3.14159265f / 180.0f)
#define GAMMA3_RAD          (GAMMA3_DEG * 3.14159265f / 180.0f)

// ═══════════════════════════════════════════════════════════
// KINEMATIC MIXING COEFFICIENTS
// Forward kinematics: ball velocity from wheel angular velocity
//
// vx = (Rw/Rk) × [−sin(γ1)·ω1 − sin(γ2)·ω2 − sin(γ3)·ω3]
// vy = (Rw/Rk) × [+cos(γ1)·ω1 + cos(γ2)·ω2 + cos(γ3)·ω3]
//
// Substituting γ1=120°, γ2=240°, γ3=0°, Rw/Rk=0.35714:
//   sin120°=+0.8660, sin240°=−0.8660, sin0°= 0.0000
//   cos120°=−0.5000, cos240°=−0.5000, cos0°=+1.0000
// ═══════════════════════════════════════════════════════════
#define KIN_VX_W1   (-(RW_OVER_RK) * sinf(GAMMA1_RAD))  // = −0.3093
#define KIN_VX_W2   (-(RW_OVER_RK) * sinf(GAMMA2_RAD))  // = +0.3093
#define KIN_VX_W3   (-(RW_OVER_RK) * sinf(GAMMA3_RAD))  // =  0.0000

#define KIN_VY_W1   (+(RW_OVER_RK) * cosf(GAMMA1_RAD))  // = −0.1786
#define KIN_VY_W2   (+(RW_OVER_RK) * cosf(GAMMA2_RAD))  // = −0.1786
#define KIN_VY_W3   (+(RW_OVER_RK) * cosf(GAMMA3_RAD))  // = +0.3571

#define TILT_LIMIT_RAD    0.1f


#define STEPS_PER_RAD_8     254.648f
#define STEPS_PER_RAD_16    509.296f
#define USTEP_THRESH_HIGH   1.5f
#define USTEP_THRESH_LOW    0.8f
#define USTEP_CONFIRM_TICKS 5

#define J_WHEEL             0.00320//0.0006f       // kg·m² — tune to your wheel's inertia if robot vibrates could use this  0.00320

#define TILT_HARD_STOP_RAD 0.35f   // ~20° recovery ceiling
#define PHI_OFFSET    -0.022f
#define THETA_OFFSET  -0.185f


// USART
#define UART_RX_BUF_SIZE  64
#define CMD_PACKET_SIZE   15
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t uart_tx_buf[192];
volatile uint8_t robot_mode = 0; // 0 = Standby/Disabled, 1 = Active Balancing
uint8_t prev_robot_mode = 0;

extern float integral_error_x;
extern float integral_error_y;



volatile int32_t m1_speed_hz = 0;
volatile int32_t m2_speed_hz = 0;
volatile int32_t m3_speed_hz = 0;

// Accumulators for the Bresenham stepping algorithm
volatile uint32_t m1_accumulator = 0;
volatile uint32_t m2_accumulator = 0;
volatile uint32_t m3_accumulator = 0;

// Encoder Tracking Variables
volatile int32_t enc1_velocity = 0; // Ticks per loop for Wheel 1
volatile int32_t enc2_velocity = 0; // Ticks per loop for Wheel 2
volatile int32_t enc3_velocity = 0; // Ticks per loop for Wheel 3

// Previous timer counts
uint32_t enc1_prev = 0;
uint16_t enc2_prev = 0;
uint16_t enc3_prev = 0;

//static float m1_omega = 0.0f;
//static float m2_omega = 0.0f;
//static float m3_omega = 0.0f;

MPU6050_t MPU6050;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_IWDG_Init(void);
void Velocit_Task(void const * argument);
void Balance_Task(void const * argument);

/* USER CODE BEGIN PFP */

void Update_Encoder_Velocities(void);
void Set_Motor_Speeds(int32_t speed1, int32_t speed2, int32_t speed3);
void convert_torque_to_speed(float t1, float t2, float t3);
void USART2_IRQHandler_Extension(void);




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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  MX_USB_DEVICE_Init();
  HAL_UART_Receive_DMA(&huart2, uart_rx_buf, UART_RX_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);



  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

   // 1. Wait a moment for the MPU6050 to power up
 HAL_Delay(100);
 HAL_StatusTypeDef mpu_status = HAL_I2C_IsDeviceReady(&hi2c1, (0x68 << 1), 3, 100);
// if (MPU6050_Init(&hi2c1) != 0 && mpu_status != HAL_OK) {
////     // Initialization failed. Blink fast and stop.
////     sprintf(usb_buf, sizeof(usb_buf), "MPU6050 INIT FAILED! Check wiring.\r\n");
////     CDC_Transmit_FS((uint8_t*)usb_buf, strlen(usb_buf));
//     while (1) {
//         HAL_GPIO_TogglePin(Black_Pill_LED_GPIO_Port, Black_Pill_LED_Pin);
//         HAL_Delay(50);
//     }
// }

  /* USER CODE END 2 */

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
  /* definition and creation of VelocityTask */
  osThreadDef(VelocityTask, Velocit_Task, osPriorityNormal, 0, 512);
  VelocityTaskHandle = osThreadCreate(osThread(VelocityTask), NULL);

  /* definition and creation of BalanceTask */
  osThreadDef(BalanceTask, Balance_Task, osPriorityHigh, 0, 2048);
  BalanceTaskHandle = osThreadCreate(osThread(BalanceTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1370;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Black_Pill_LED_Pin|ENAPLE_M3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENAPLE_M2_Pin|M1_STEP_Pin|M1_DIR_Pin|M2_STEP_Pin
                          |M2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M3_STEP_Pin|M3_DIR_Pin|ENABLE_PIN_Pin|M3_DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Black_Pill_LED_Pin ENAPLE_M3_Pin */
  GPIO_InitStruct.Pin = Black_Pill_LED_Pin|ENAPLE_M3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENAPLE_M2_Pin M1_STEP_Pin M1_DIR_Pin M2_STEP_Pin
                           M2_DIR_Pin */
  GPIO_InitStruct.Pin = ENAPLE_M2_Pin|M1_STEP_Pin|M1_DIR_Pin|M2_STEP_Pin
                          |M2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_STEP_Pin M3_DIR_Pin ENABLE_PIN_Pin M3_DIR1_Pin */
  GPIO_InitStruct.Pin = M3_STEP_Pin|M3_DIR_Pin|ENABLE_PIN_Pin|M3_DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* ── TIM2 encoder — Motor 1 — PA0 = CH1 (A+), PA1 = CH2 (B+) ──
     Both PA0 and PA1 are 5V tolerant. J103 right side. AM26LS32 output → here. */
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── TIM3 encoder — Motor 2 — PA6 = CH1 (A+), PA7 = CH2 (B+) ──
     Both PA6 and PA7 are 5V tolerant. J103 right side. */
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── TIM4 encoder — Motor 3 — PB6 = CH1 (A+), PB7 = CH2 (B+) ──
     Both PB6 and PB7 are 5V tolerant. J102 left side — no alternative on F411. */
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ═══════════════════════════════════════════════════════════════
// CDC Receive — parses velocity commands from laptop
// Protocol: [0xAA][0x55][vx f32LE][vy f32LE][yaw_rate f32LE][XOR checksum]
// Total: 15 bytes
// Send from Python:
//   payload = struct.pack('<3f', vx, vy, yaw_rate)
//   checksum = 0
//   for b in payload: checksum ^= b
//   ser.write(bytes([0xAA, 0x55]) + payload + bytes([checksum]))
// ═══════════════════════════════════════════════════════════════
void CDC_ReceiveCallback(uint8_t* buf, uint32_t len)
{
    if (len < 15) return;

    // Find sync header
    for (uint32_t i = 0; i <= len - 15; i++) {
        if (buf[i] == 0xAA && buf[i+1] == 0x55) {
            // Verify checksum (XOR of bytes 2–13)
            uint8_t chk = 0;
            for (int k = 2; k < 14; k++) {
                chk ^= buf[i+k];
            }
            if (chk != buf[i+14]) continue;  // bad checksum, skip

            // Parse three floats (little-endian)
            float vx, vy, yaw_rate;
            memcpy(&vx,       &buf[i+2],  4);
            memcpy(&vy,       &buf[i+6],  4);
            memcpy(&yaw_rate, &buf[i+10], 4);

            // Update LQR references
            updateReferencesFromROS(vx, vy, yaw_rate, HAL_GetTick());
            return;
        }
    }
}

void USART2_IRQHandler_Extension(void)
{
    /* Only handle IDLE line interrupt */
    if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    uint16_t bytes = UART_RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if (bytes < 16) goto reset_dma;

    for (uint16_t i = 0; i + 16 <= bytes; i++) {
        if (uart_rx_buf[i] != 0xAA || uart_rx_buf[i+1] != 0x55) continue;

        uint8_t chk = 0;
        for (int k = 2; k < 15; k++) chk ^= uart_rx_buf[i+k];
        if (chk != uart_rx_buf[i+15]) continue;

        uint8_t rx_mode = uart_rx_buf[i+2];
        float vx, vy, yaw;
        memcpy(&vx,  &uart_rx_buf[i+3],  4);
        memcpy(&vy,  &uart_rx_buf[i+7],  4);
        memcpy(&yaw, &uart_rx_buf[i+11], 4);

        robot_mode = rx_mode;
        updateReferencesFromROS(vx, vy, yaw, HAL_GetTick());
        break;
    }

reset_dma:
    HAL_UART_DMAStop(&huart2);
    HAL_UART_Receive_DMA(&huart2, uart_rx_buf, UART_RX_BUF_SIZE);
}


void Set_Motor_Speeds(int32_t speed1, int32_t speed2, int32_t speed3) {
    m1_speed_hz = speed1;
    m2_speed_hz = speed2;
    m3_speed_hz = speed3;
}

void Update_Encoder_Velocities(void) {
    // ----------------------------------------------------
    // MOTOR 1 (TIM2 is a 32-bit timer on the F411)
    // ----------------------------------------------------
    uint32_t enc1_current = htim2.Instance->CNT;
    enc1_velocity = (int32_t)(enc1_current - enc1_prev);
    enc1_prev = enc1_current;

    // ----------------------------------------------------
    // MOTOR 2 (TIM3 is a 16-bit timer)
    // ----------------------------------------------------
    uint16_t enc2_current = htim3.Instance->CNT;
    int16_t enc2_diff  = (int16_t)(enc2_current - enc2_prev);
    enc2_velocity = (int32_t)enc2_diff;
    enc2_prev = enc2_current;

    // ----------------------------------------------------
    // MOTOR 3 (TIM4 is a 16-bit timer)
    // ----------------------------------------------------
    uint16_t enc3_current = htim4.Instance->CNT;
    int16_t enc3_diff  = (int16_t)(enc3_current - enc3_prev);
    enc3_velocity = (int32_t)enc3_diff;
    enc3_prev = enc3_current;
}

/* ═══════════════════════════════════════════════════════════════
 * convert_torque_to_speed — DIRECT mapping, no integration
 *
 * The LQR torque is treated as a velocity command proportion.
 * TORQUE_TO_OMEGA maps Nm to rad/s. Tune this constant on hardware:
 *   - Too large  → oscillation, aggressive response
 *   - Too small  → sluggish, falls over slowly
 * Start at 16.0, halve it if the robot oscillates.
 * ═══════════════════════════════════════════════════════════════ */
// Add these static variables inside convert_torque_to_speed:
void convert_torque_to_speed(float t1, float t2, float t3)
{
    // Persistence for the ramping logic (prevents wheel slip)
    static int32_t prev_spd1 = 0, prev_spd2 = 0, prev_spd3 = 0;

    // --- TUNING PARAMETERS ---

    // TORQUE_TO_OMEGA: Master sensitivity. Start at 6.0 for 17-22kg [History]

    // STEPS_PER_RAD:
    // For 3200: (3200 / 2*PI) = 509.296f
    // For 6400: (6400 / 2*PI) = 1018.592f

    // MIN_STEPS: Deadband to stop "stepper hum" and IMU pollution.
    // Recommended: 1200 Hz for 3200x; 2400 Hz for 6400x [1, 2]

    // MAX_RAMP: Acceleration limit per loop (dt=0.002s).
    // Tune this to prevent wheel slip on the ball.
    // For 3200: 320 steps/loop (160,000 steps/s^2)
    // For 6400: 640 steps/loop (320,000 steps/s^2)
    const int32_t MAX_RAMP   = 80;

    // MAX_SPEED_HZ: Hardware limit based on 60V supply corner speed.
    // For 3200: 14,000 Hz (~2.2 m/s robot speed)
    // For 6400: 28,000 Hz (~2.2 m/s robot speed) [2, 3]

    // 1. Map Torque directly to Radian Velocity (rad/s)
    float w1 = t1 * TORQUE_TO_OMEGA;
    float w2 = t2 * TORQUE_TO_OMEGA;
    float w3 = t3 * TORQUE_TO_OMEGA;

    // 2. Convert Rad/s to Target Step Frequency (Hz)
    int32_t target1 = (int32_t)(w1 * STEPS_PER_RAD);
    int32_t target2 = (int32_t)(w2 * STEPS_PER_RAD);
    int32_t target3 = (int32_t)(w3 * STEPS_PER_RAD);

    // 3. Deadband with hysteresis — prevents 1199↔1201 toggling
    //    DEAD_EXIT  = stop threshold  — if below this, motor stops
    //    DEAD_ENTER = start threshold — need this speed to START moving
    //    Effect: motor stops at 1200, but needs 1400 to restart
    //    This eliminates the clicking at small angles
//    const int32_t DEAD_EXIT  = 0;
//    const int32_t DEAD_ENTER = 0;
//
//    if (abs(target1) < DEAD_EXIT)  { target1 = 0; prev_spd1 = 0; }
//    else if (abs(target1) < DEAD_ENTER && prev_spd1 == 0) { target1 = 0; }
//
//    if (abs(target2) < DEAD_EXIT)  { target2 = 0; prev_spd2 = 0; }
//    else if (abs(target2) < DEAD_ENTER && prev_spd2 == 0) { target2 = 0; }
//
//    if (abs(target3) < DEAD_EXIT)  { target3 = 0; prev_spd3 = 0; }
//    else if (abs(target3) < DEAD_ENTER && prev_spd3 == 0) { target3 = 0; }


    // 4. Rate Limiting (Ramp): Protects the 17kg mass from sudden jerk/slip
    int32_t diff1 = target1 - prev_spd1;
    int32_t diff2 = target2 - prev_spd2;
    int32_t diff3 = target3 - prev_spd3;

    if (diff1 >  MAX_RAMP) diff1 =  MAX_RAMP;
    if (diff1 < -MAX_RAMP) diff1 = -MAX_RAMP;
    if (diff2 >  MAX_RAMP) diff2 =  MAX_RAMP;
    if (diff2 < -MAX_RAMP) diff2 = -MAX_RAMP;
    if (diff3 >  MAX_RAMP) diff3 =  MAX_RAMP;
    if (diff3 < -MAX_RAMP) diff3 = -MAX_RAMP;

    // 5. Update Persistent Speeds
    prev_spd1 += diff1;
    prev_spd2 += diff2;
    prev_spd3 += diff3;

    // 6. Final Hardware Output (Clamped to Electrical Corner Speed)
    Set_Motor_Speeds(
        (int32_t)fmaxf(-MAX_SPEED_HZ, fminf(MAX_SPEED_HZ, (float)prev_spd1)),
        (int32_t)fmaxf(-MAX_SPEED_HZ, fminf(MAX_SPEED_HZ, (float)prev_spd2)),
        (int32_t)fmaxf(-MAX_SPEED_HZ, fminf(MAX_SPEED_HZ, (float)prev_spd3))
    );
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Velocit_Task */
/**
  * @brief  Function implementing the VelocityTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Velocit_Task */
void Velocit_Task(void const * argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&xLastWakeTime, 10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Balance_Task */
/**
* @brief Function implementing the BalanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Balance_Task */
void Balance_Task(void const * argument)
{
  /* USER CODE BEGIN Balance_Task */
    const float      DT_S       = 0.005f;
    const TickType_t EKF_PERIOD = pdMS_TO_TICKS(5);
    static char      usb_buf[256];
    uint32_t         tick_count = 0;
    const float      D2R        = 3.14159265f / 180.0f;
    static float gyroz_filter = 0.0f;
    const float gyroz_LPF = 0.05f;


   // static const float TICKS_TO_RADS = 0.785398f;
   //static const float TICKS_TO_RADS = 0.392699f; // (x16)    2pi/(microstepping * delta t)
   //static const float TICKS_TO_RADS = 3.1415926f; // (x2)    2pi/(microstepping * delta t)
//   static const float TICKS_TO_RADS = (2.0f * 3.14159265f) / (STEPS_PER_REV * DT_S);
    static const float TICKS_TO_RADS = (2.0f * 3.14159265f) / (ENCODER_CPR * DT_S);


    vTaskDelay(pdMS_TO_TICKS(200));
    MPU6050_Init(&hi2c1);
    vTaskDelay(pdMS_TO_TICKS(50));
    BallbotEKF_Init();

    // ── Gyro bias calibration ───────────────────────────────────
    // FIX: disable motors so vibration does not corrupt bias
    HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2000));

    float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
    const int CAL_SAMPLES = 400;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        MPU6050_Read_All(&hi2c1, &MPU6050);
        gyro_bias_x += MPU6050.Gx;
        gyro_bias_y += MPU6050.Gy;
        gyro_bias_z += MPU6050.Gz;
        vTaskDelay(pdMS_TO_TICKS(5));   // FIX: was HAL_Delay — yields FreeRTOS
    }
    gyro_bias_x /= CAL_SAMPLES;
    gyro_bias_y /= CAL_SAMPLES;
    gyro_bias_z /= CAL_SAMPLES;

    // Re-enable motors after calibration
    HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);

    BallbotStates ekf = {0};
    TickType_t xLastWake = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
      tick_count++;
            if (robot_mode == 0) {
                Set_Motor_Speeds(0, 0, 0);
                HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_SET); // Keep drivers offline

                BallbotEKF_Init();
                ekf = (BallbotStates){0};
                integral_error_x = 0.0f;
                integral_error_y = 0.0f;

                if (prev_robot_mode != robot_mode) {
                    CDC_Transmit_FS((uint8_t*)"MOTORS_DISABLED\r\n", 17);
                    if (huart2.gState == HAL_UART_STATE_READY)
                        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"MOTORS_DISABLED\r\n", 17);
                    prev_robot_mode = robot_mode;
                }

                vTaskDelayUntil(&xLastWake, EKF_PERIOD);
                continue;
            }

            if (prev_robot_mode != robot_mode) {
                HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET); // Energize drivers
                xLastWake = xTaskGetTickCount();
                CDC_Transmit_FS((uint8_t*)"MOTORS_ENABLED\r\n", 16);
                if (huart2.gState == HAL_UART_STATE_READY)
                    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"MOTORS_ENABLED\r\n", 16);
                prev_robot_mode = robot_mode;
            }

       Update_Encoder_Velocities();
       MPU6050_Read_All(&hi2c1, &MPU6050);

       float gyro_x  = (MPU6050.Gx - gyro_bias_x) * D2R;
       float gyro_y  = (MPU6050.Gy - gyro_bias_y) * D2R;
       float gyro_z  = (MPU6050.Gz - gyro_bias_z) * D2R;
       float accel_x = MPU6050.Ax * 9.81f;
       float accel_y = MPU6050.Ay * 9.81f;
       float accel_z = MPU6050.Az * 9.81f;

       // ── Accel spike rejection ────────────────────────────────
       // FIX: use squared comparison — eliminates sqrtf() every loop
       // At rest: ax²+ay²+az² ≈ 9.81² = 96.2
       // Reject if outside [0.2g, 15g] → [2², 147²] = [4, 21609]
       float accel_mag2 = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z;
       float safe_ax, safe_ay;
       if (accel_mag2 > (147.0f*147.0f) || accel_mag2 < (2.0f*2.0f)) {
           safe_ax =  9.81f * sinf(ekf.theta);
           safe_ay = -9.81f * sinf(ekf.phi) * cosf(ekf.theta);
       } else {
           safe_ax = accel_x;
           safe_ay = accel_y;
       }



       // CORRECT — replace the entire commented block with this
       float w1 = (float)enc1_velocity * TICKS_TO_RADS;
       float w2 = (float)enc2_velocity * TICKS_TO_RADS;
       float w3 = (float)enc3_velocity * TICKS_TO_RADS;

//       float enc_vx = (-0.3093f*w1) + (0.3093f*w2) + (0.0000f*w3);
//       float enc_vy = (-0.1786f*w1) + (-0.1786f*w2) + (0.3571f*w3);
//
       //BallbotEKF_Update(gyro_x, gyro_y, safe_ax, safe_ay,
                //               0, 0, DT_S, &ekf);



       // Forward kinematics: wheel angular velocity (rad/s) → ball linear velocity (m/s)
       float enc_vx = KIN_VX_W1*w1 + KIN_VX_W2*w2 + KIN_VX_W3*w3;
       float enc_vy = KIN_VY_W1*w1 + KIN_VY_W2*w2 + KIN_VY_W3*w3;
       BallbotEKF_Update(gyro_x, gyro_y, safe_ax, safe_ay,
                                 enc_vx, enc_vy, DT_S, &ekf);


//        if (!isfinite(ekf.phi) || !isfinite(ekf.theta) ||
//            fabsf(ekf.phi   - PHI_OFFSET)   > TILT_HARD_STOP_RAD ||
//            fabsf(ekf.theta - THETA_OFFSET) > TILT_HARD_STOP_RAD) {
//            Set_Motor_Speeds(0, 0, 0);   // stop motors FIRST
//            BallbotEKF_Init();
//            ekf = (BallbotStates){0};    // reset struct
//            vTaskDelayUntil(&xLastWake, EKF_PERIOD);
//            continue;
//        }
//        if (fabsf(ekf.phi - PHI_OFFSET) > TILT_LIMIT_RAD || fabsf(current_theta - THETA_OFFSET) > THETA_OFFSET)
//            {
//                HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_SET);
//                Set_Motor_Speeds(0, 0, 0);
//                if (tick_count % 20 == 0) {
//                    int len = snprintf(usb_buf, sizeof(usb_buf),
//                        "phi:%+5.3f the:%+5.3f | ENC:%ld %ld %ld | Steps:%ld %ld %ld\r\n",
//                        current_state[0], current_state[1],
//                        enc1_velocity, enc2_velocity, enc3_velocity,
//                        m1_speed_hz, m2_speed_hz, m3_speed_hz);
//                    if (len > 0 && len < (int)sizeof(usb_buf))
//                        CDC_Transmit_FS((uint8_t*)usb_buf, (uint16_t)len);
//                }
//                vTaskDelayUntil(&xLastWake, EKF_PERIOD);
//                continue;
//            }
//            else
//            {
//                HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);
//            }

       gyroz_filter = gyroz_LPF * gyro_z + (1.0f - gyroz_LPF) * gyroz_filter;

       current_state[0] = ekf.phi   - PHI_OFFSET;
       current_state[1] = ekf.theta - THETA_OFFSET;
       current_state[2] = 0.00f;         // psi — no yaw control yet
       current_state[3] = enc_vx;
       current_state[4] = enc_vy;
       current_state[5] = ekf.dphi;
       current_state[6] = ekf.dtheta;
       current_state[7] = gyroz_filter;

       // ── Control ─────────────────────────────────────────────
       float torques[3];
       controlLoop(DT_S, HAL_GetTick(), torques);
       convert_torque_to_speed(torques[0], torques[1], torques[2]);

       // ── Telemetry — print at 10Hz (every 20 loops × 5ms) ────
       // FIX: use snprintf return value directly, not strlen pass
//       if (tick_count % 20 == 0) {
//           int len = snprintf(usb_buf, sizeof(usb_buf),
//               "phi:%+5.3f the:%+5.3f yaw:%+5.3f |ENC_Vx:%+5.3f  ENC_Vy:%+5.3f  | ENC:%ld %ld %ld | Steps:%ld %ld %ld\r\n",
//               current_state[0], current_state[1], current_state[7],
//				enc_vx,enc_vx,
//               enc1_velocity, enc2_velocity, enc3_velocity,
//               m1_speed_hz, m2_speed_hz, m3_speed_hz);
//           if (len > 0 && len < (int)sizeof(usb_buf))
//               CDC_Transmit_FS((uint8_t*)usb_buf, (uint16_t)len);
//       }
       if (tick_count % 20 == 0) {
           int len = snprintf((char*)uart_tx_buf, sizeof(uart_tx_buf),
               "phi:%+.3f the:%+.3f yaw:%+.3f |Vx:%+.3f Vy:%+.3f | ENC:%ld %ld %ld | S:%ld %ld %ld\r\n",
               current_state[0], current_state[1], current_state[7],
			   enc_vx, enc_vy,
               enc1_velocity, enc2_velocity, enc3_velocity,
               m1_speed_hz, m2_speed_hz, m3_speed_hz);
           if (len > 0 && len < (int)sizeof(uart_tx_buf)) {
               CDC_Transmit_FS(uart_tx_buf, (uint16_t)len);
               if (huart2.gState == HAL_UART_STATE_READY)
                   HAL_UART_Transmit_DMA(&huart2, uart_tx_buf, (uint16_t)len);
           }
       }

       vTaskDelayUntil(&xLastWake, EKF_PERIOD);
  }
  /* USER CODE END Balance_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  // THE 20kHz STEPPER ENGINE
  if (htim->Instance == TIM11)
    {
        uint8_t step1 = 0, step2 = 0, step3 = 0;

        // ----------------------------------------------------
        // 1. Calculate Math & Set Directions
        // ----------------------------------------------------

        // Motor 1
            HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);
            if (m1_speed_hz != 0) {
                HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin,
                                 (m1_speed_hz > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                m1_accumulator += abs(m1_speed_hz);
                if (m1_accumulator >= TIMER_FREQ_HZ) {
                    m1_accumulator -= TIMER_FREQ_HZ;
                    step1 = 1;
                }
            } else {
                m1_accumulator = 0;
            }

            // Motor 2
            HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);
            if (m2_speed_hz != 0) {
                HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin,
                                 (m2_speed_hz > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                m2_accumulator += abs(m2_speed_hz);
                if (m2_accumulator >= TIMER_FREQ_HZ) {
                    m2_accumulator -= TIMER_FREQ_HZ;
                    step2 = 1;
                }
            } else {
                m2_accumulator = 0;
            }

            // Motor 3
            HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);
            if (m3_speed_hz != 0) {
                HAL_GPIO_WritePin(ENAPLE_M3_GPIO_Port, ENAPLE_M3_Pin,
                                 (m3_speed_hz > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                m3_accumulator += abs(m3_speed_hz);
                if (m3_accumulator >= TIMER_FREQ_HZ) {
                    m3_accumulator -= TIMER_FREQ_HZ;
                    step3 = 1;
                }
            } else {
                m3_accumulator = 0;
            }

        // ----------------------------------------------------
        // 2. Turn HIGH all necessary STEP pins SIMULTANEOUSLY
        // ----------------------------------------------------
        if (step1) HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_SET);
        if (step2) HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_SET);
        if (step3) HAL_GPIO_WritePin(M3_STEP_GPIO_Port, M3_STEP_Pin, GPIO_PIN_SET);

        // ----------------------------------------------------
        // 3. The SHARED Delay (Only executed if at least 1 motor stepped)
        // ----------------------------------------------------
        if (step1 || step2 || step3) {
            uint32_t t0 = DWT->CYCCNT;

            // DM860H requires ~2.5 microseconds.
            // On a 100 MHz STM32F411, 1 microsecond = 100 clock cycles.
            // 2.5us * 100 = 250 cycles.
            // We use 260U just to be perfectly safe for the optocouplers.
            while ((DWT->CYCCNT - t0) < 260U);
        }

        // ----------------------------------------------------
        // 4. Turn LOW all necessary STEP pins
        // ----------------------------------------------------
        if (step1) HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);
        if (step2) HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_RESET);
        if (step3) HAL_GPIO_WritePin(M3_STEP_GPIO_Port, M3_STEP_Pin, GPIO_PIN_RESET);
    }
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
