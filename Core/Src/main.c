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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "usb_communication.h"
#include "Sensors/hall_sensor.h"
#include "Sensors/sensor.h"
#include "Motor/motor.h"
#include "Drivers/openloop_foc.h"
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

/* USER CODE BEGIN PV */
// Motor (configuração e estado)
Motor_t motor;

// Sensor Hall
HallSensor_t hall_sensor;
Sensor_t sensor_interface;

// Driver FOC
OpenLoopFOC_t foc_driver;

// Telemetria
Telemetry_t telemetry;

// Controle de timing
uint32_t last_telemetry_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USB_Device_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // Delay para estabilização USB
  HAL_Delay (500);

  // Inicializar comunicação USB
  USB_Comm_Init ();

  // Configurar motor
  MotorConfig_t motor_config = {
      .pole_pairs = MOTOR_POLE_PAIRS,
      .kv_rating = 190.0f,
      .phase_resistance = 0.045f,
      .phase_inductance = 0.000020f,
      .voltage_limit = 12.0f,
      .current_limit = 30.0f,
      .velocity_limit_rpm = 3000.0f
  };
  Motor_Init (&motor, &motor_config);

  // Inicializar sensor Hall e criar interface
  Hall_Init (&hall_sensor);
  sensor_interface = Hall_CreateSensorInterface (&hall_sensor);

  // Configurar e inicializar driver Open-Loop FOC
  OpenLoopFOC_Config_t foc_config = {
      .htim = &htim1,
      .max_voltage = 12.0f,
      .ramp_rate = 5.0f  // 5V/s de rampa
  };
  OpenLoopFOC_Init (&foc_driver, &foc_config, &motor);
  OpenLoopFOC_AttachSensor (&foc_driver, &sensor_interface);

  // Iniciar PWM complementar (CH + CHN) nos 3 canais
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Canal complementar CH1N
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // Canal complementar CH2N
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // Canal complementar CH3N
  
  HAL_TIM_Base_Start_IT(&htim1);  // Habilita interrupção Update @ 20kHz

  USB_Comm_Print ("\r\n=== KINEMATECH TORQ ESC ===\r\n");
  USB_Comm_Print ("Firmware v0.1 - Oct 2025\r\n");
  USB_Comm_Print ("Hall Sensor ready. Rotate motor to test.\r\n\r\n");
  HAL_GPIO_TogglePin (LED_BLUE_GPIO_Port, LED_BLUE_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // ===== PROCESSAR DADOS DO HALL (main loop) =====
      Hall_ProcessData (&hall_sensor);

      // ===== TELEMETRIA VIA USB (100Hz) =====
      uint32_t current_time = HAL_GetTick ();
      if (current_time - last_telemetry_ms >= (1000 / TELEMETRY_RATE_HZ))
	{
	  last_telemetry_ms = current_time;

	  // Preencher telemetria usando sensor diretamente
	  telemetry.hall_state = hall_sensor.hall_state;
	  telemetry.hall_sector = Hall_GetSector(&hall_sensor);
	  telemetry.hall_angle = sensor_interface.get_angle_electrical(sensor_interface.instance);
	  telemetry.hall_velocity = Hall_GetMechanicalVelocity(&hall_sensor);  // RPM mecânico
	  telemetry.isr_counter = hall_sensor.isr_counter;
	  telemetry.uptime_ms = current_time;
	  telemetry.errors = 0;

	  USB_Comm_SendTelemetry (&telemetry);
	}

      // Sleep até próxima interrupção (economia de energia)
      __WFI();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USB_LP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

/* USER CODE BEGIN 4 */

// ===== CALLBACK TIM8 HALL INTERFACE =====
extern HallSensor_t hall_sensor;

/**
 * @brief Callback do TIM8 Hall Sensor Interface
 * @note Chamada automaticamente pela HAL a cada transição Hall
 */
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8)
    {
      hall_sensor.isr_counter++;
      Hall_TIM_CaptureCallback (&hall_sensor);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  extern OpenLoopFOC_t foc_driver;
  
  // TIM1 Update @ 20kHz - FOC Loop (prioridade 0)
  if (htim->Instance == TIM1)
  {
    OpenLoopFOC_TIM_Callback(&foc_driver);
    return;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
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
  __disable_irq ();
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
