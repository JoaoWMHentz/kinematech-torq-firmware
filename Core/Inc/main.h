/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_13
#define LED_BLUE_GPIO_Port GPIOC
#define PA_LIN_Pin GPIO_PIN_13
#define PA_LIN_GPIO_Port GPIOB
#define PB_LIN_Pin GPIO_PIN_14
#define PB_LIN_GPIO_Port GPIOB
#define PC_LIN_Pin GPIO_PIN_15
#define PC_LIN_GPIO_Port GPIOB
#define PA_HIN_Pin GPIO_PIN_8
#define PA_HIN_GPIO_Port GPIOA
#define PB_HIN_Pin GPIO_PIN_9
#define PB_HIN_GPIO_Port GPIOA
#define PC_HIN_Pin GPIO_PIN_10
#define PC_HIN_GPIO_Port GPIOA
#define ENCODER_SCL_Pin GPIO_PIN_15
#define ENCODER_SCL_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOB
#define ENCODER_SDA_Pin GPIO_PIN_7
#define ENCODER_SDA_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_8
#define HALL_B_GPIO_Port GPIOB
#define HALL_C_Pin GPIO_PIN_9
#define HALL_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
