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
#include "stm32f2xx_hal.h"

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
#define Digital_input_3_Pin GPIO_PIN_2
#define Digital_input_3_GPIO_Port GPIOE
#define Digital_input_4_Pin GPIO_PIN_3
#define Digital_input_4_GPIO_Port GPIOE
#define Digital_input_5_Pin GPIO_PIN_4
#define Digital_input_5_GPIO_Port GPIOE
#define Digital_input_6_Pin GPIO_PIN_5
#define Digital_input_6_GPIO_Port GPIOE
#define Digital_input_7_Pin GPIO_PIN_6
#define Digital_input_7_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define Button_1_Pin GPIO_PIN_0
#define Button_1_GPIO_Port GPIOF
#define Button_2_Pin GPIO_PIN_1
#define Button_2_GPIO_Port GPIOF
#define Button_3_Pin GPIO_PIN_2
#define Button_3_GPIO_Port GPIOF
#define Button_4_Pin GPIO_PIN_3
#define Button_4_GPIO_Port GPIOF
#define Button_5_Pin GPIO_PIN_4
#define Button_5_GPIO_Port GPIOF
#define Button_6_Pin GPIO_PIN_5
#define Button_6_GPIO_Port GPIOF
#define Dry_contact_alarm_Pin GPIO_PIN_6
#define Dry_contact_alarm_GPIO_Port GPIOF
#define Soft_alarm_1_Pin GPIO_PIN_7
#define Soft_alarm_1_GPIO_Port GPIOF
#define Soft_alarm_2_Pin GPIO_PIN_8
#define Soft_alarm_2_GPIO_Port GPIOF
#define Soft_alarm_3_Pin GPIO_PIN_9
#define Soft_alarm_3_GPIO_Port GPIOF
#define Uart_ctrl_Pin GPIO_PIN_10
#define Uart_ctrl_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define Temprature_sensor_Pin GPIO_PIN_0
#define Temprature_sensor_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define Ambient_sensor_1_Pin GPIO_PIN_3
#define Ambient_sensor_1_GPIO_Port GPIOA
#define Ambient_sensor_2_Pin GPIO_PIN_4
#define Ambient_sensor_2_GPIO_Port GPIOA
#define Fan1_pwm_Pin GPIO_PIN_6
#define Fan1_pwm_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define Fan2_pwm_Pin GPIO_PIN_1
#define Fan2_pwm_GPIO_Port GPIOB
#define Digital_input_8_Pin GPIO_PIN_7
#define Digital_input_8_GPIO_Port GPIOE
#define Door_sensor_Pin GPIO_PIN_8
#define Door_sensor_GPIO_Port GPIOE
#define Smoke_sensor_Pin GPIO_PIN_9
#define Smoke_sensor_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Fan1_IP_Pin GPIO_PIN_12
#define Fan1_IP_GPIO_Port GPIOD
#define Fan2_IP_Pin GPIO_PIN_13
#define Fan2_IP_GPIO_Port GPIOD
#define Fan3_IP_Pin GPIO_PIN_14
#define Fan3_IP_GPIO_Port GPIOD
#define Fan4_IP_Pin GPIO_PIN_15
#define Fan4_IP_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Fan3_pwm_Pin GPIO_PIN_7
#define Fan3_pwm_GPIO_Port GPIOC
#define Fan4_pwm_Pin GPIO_PIN_8
#define Fan4_pwm_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_RS_Pin GPIO_PIN_4
#define LED_RS_GPIO_Port GPIOB
#define LED_Enable_Pin GPIO_PIN_5
#define LED_Enable_GPIO_Port GPIOB
#define LED_D4_Pin GPIO_PIN_6
#define LED_D4_GPIO_Port GPIOB
#define LED_D5_Pin GPIO_PIN_7
#define LED_D5_GPIO_Port GPIOB
#define LED_D6_Pin GPIO_PIN_8
#define LED_D6_GPIO_Port GPIOB
#define LED_D7_Pin GPIO_PIN_9
#define LED_D7_GPIO_Port GPIOB
#define Digital_input_1_Pin GPIO_PIN_0
#define Digital_input_1_GPIO_Port GPIOE
#define Digital_input_2_Pin GPIO_PIN_1
#define Digital_input_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
