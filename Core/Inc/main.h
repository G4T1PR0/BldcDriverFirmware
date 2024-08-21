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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vdrive_Switch_Pin GPIO_PIN_5
#define Vdrive_Switch_GPIO_Port GPIOE
#define EXT_Enc_MOSI2_Pin GPIO_PIN_1
#define EXT_Enc_MOSI2_GPIO_Port GPIOC
#define W_Phase_CurrentFeedback_Pin GPIO_PIN_2
#define W_Phase_CurrentFeedback_GPIO_Port GPIOC
#define Debug_Tx_Pin GPIO_PIN_2
#define Debug_Tx_GPIO_Port GPIOA
#define Debug_RX_Pin GPIO_PIN_3
#define Debug_RX_GPIO_Port GPIOA
#define V_Phase_CurrentFeedback_Pin GPIO_PIN_5
#define V_Phase_CurrentFeedback_GPIO_Port GPIOA
#define U_Phase_LoSide_Pin GPIO_PIN_7
#define U_Phase_LoSide_GPIO_Port GPIOA
#define U_Phase_CurrentFeedback_Pin GPIO_PIN_5
#define U_Phase_CurrentFeedback_GPIO_Port GPIOC
#define V_Phase_LoSide_Pin GPIO_PIN_0
#define V_Phase_LoSide_GPIO_Port GPIOB
#define W_Phase_LoSide_Pin GPIO_PIN_1
#define W_Phase_LoSide_GPIO_Port GPIOB
#define U_Phase_HiSide_Pin GPIO_PIN_9
#define U_Phase_HiSide_GPIO_Port GPIOE
#define V_Phase_Hiside_Pin GPIO_PIN_11
#define V_Phase_Hiside_GPIO_Port GPIOE
#define W_Phase_Hiside_Pin GPIO_PIN_13
#define W_Phase_Hiside_GPIO_Port GPIOE
#define Controller_TX_Pin GPIO_PIN_10
#define Controller_TX_GPIO_Port GPIOB
#define Controller_RX_Pin GPIO_PIN_11
#define Controller_RX_GPIO_Port GPIOB
#define EXT_Enc_SCK2_Pin GPIO_PIN_13
#define EXT_Enc_SCK2_GPIO_Port GPIOB
#define EXT_Enc_MISO2_Pin GPIO_PIN_14
#define EXT_Enc_MISO2_GPIO_Port GPIOB
#define Encoder_A_Pin GPIO_PIN_6
#define Encoder_A_GPIO_Port GPIOC
#define Encoder_B_Pin GPIO_PIN_7
#define Encoder_B_GPIO_Port GPIOC
#define HallSens_W_Pin GPIO_PIN_0
#define HallSens_W_GPIO_Port GPIOD
#define HallSens_V_Pin GPIO_PIN_2
#define HallSens_V_GPIO_Port GPIOD
#define HallSens_U_Pin GPIO_PIN_4
#define HallSens_U_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
