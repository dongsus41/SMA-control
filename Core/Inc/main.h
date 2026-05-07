/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "common_defs.h"
#include "fdcan.h"
#include "max31855.h"
#include "temp_control.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Phase 6: Databuf_FDCAN_typedef 제거 (FDCAN 미사용) */
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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define FSW0_Pin GPIO_PIN_15
#define FSW0_GPIO_Port GPIOC
#define FSW1_Pin GPIO_PIN_0
#define FSW1_GPIO_Port GPIOA
#define FSW2_Pin GPIO_PIN_1
#define FSW2_GPIO_Port GPIOA
#define FSW3_Pin GPIO_PIN_2
#define FSW3_GPIO_Port GPIOA
#define FSW4_Pin GPIO_PIN_4
#define FSW4_GPIO_Port GPIOA
#define I2C2_INT_Pin GPIO_PIN_4
#define I2C2_INT_GPIO_Port GPIOC
#define FSW5_Pin GPIO_PIN_2
#define FSW5_GPIO_Port GPIOB
#define TMC_CS0_Pin GPIO_PIN_10
#define TMC_CS0_GPIO_Port GPIOB
#define TMC_CS1_Pin GPIO_PIN_11
#define TMC_CS1_GPIO_Port GPIOB
#define TMC_CS2_Pin GPIO_PIN_12
#define TMC_CS2_GPIO_Port GPIOB
#define TMC_SCK_Pin GPIO_PIN_13
#define TMC_SCK_GPIO_Port GPIOB
#define TMC_MISO_Pin GPIO_PIN_14
#define TMC_MISO_GPIO_Port GPIOB
#define TMC_CS3_Pin GPIO_PIN_15
#define TMC_CS3_GPIO_Port GPIOB
#define TMC_CS4_Pin GPIO_PIN_10
#define TMC_CS4_GPIO_Port GPIOA
#define TMC_CS5_Pin GPIO_PIN_11
#define TMC_CS5_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_12
#define SW_GPIO_Port GPIOA
#define SW_EXTI_IRQn EXTI15_10_IRQn
#define CAN_FLT_Pin GPIO_PIN_15
#define CAN_FLT_GPIO_Port GPIOA
#define CAN_FLT_EXTI_IRQn EXTI15_10_IRQn
#define UART3_TX_Pin GPIO_PIN_10
#define UART3_TX_GPIO_Port GPIOC
#define UART3_RX_Pin GPIO_PIN_11
#define UART3_RX_GPIO_Port GPIOC
#define CAN_RX_Pin GPIO_PIN_3
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_4
#define CAN_TX_GPIO_Port GPIOB
#define I2C4_INT_Pin GPIO_PIN_5
#define I2C4_INT_GPIO_Port GPIOB
#define I2C1_INT_Pin GPIO_PIN_6
#define I2C1_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum {
	SYSTEM_INIT  = 0,
	SYSTEM_READY = 1,
	SYSTEM_GO    = 2
} System_State_typedef;

/* Phase 6: 거대 글로벌 슬림화 — state_level + pnt_pwm만 유지.
 * dead 멤버 제거: buf_fdcan_tx (g_state로 대체), ctrl_param_now/save (g_cmd),
 * state_fsw / state_pwm (g_ctrl.cmd_*), lock_motion / flag_lock_motion / uart_char /
 * n_rx_motion(_limit) / t_count / t_target / t_sec (사용 안 함).
 * dead typedef 제거: Buf_FDCAN_Tx_*, Ctrl_Param_typedef, param_struct / union,
 * Flash 주소 매크로 (Flash_* 함수도 dead).
 */
typedef struct {
	System_State_typedef  state_level;
	volatile uint32_t*    pnt_pwm[CTRL_CH];
} System_typedef;

extern System_typedef system;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
