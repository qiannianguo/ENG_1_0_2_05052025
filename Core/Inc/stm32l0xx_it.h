/**
  ******************************************************************************
  * @file    stm32l0xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
 ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L0xx_IT_H
#define __STM32L0xx_IT_H

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

// 在tim22.h或全局头文件中添加
typedef enum {
    USONIC_MODE_DEFAULT = 0,    // 默认模式：100ms开关（保持原有行为）
	USONIC_MODE_0P5S_CYCLE ,     // 0.5秒周期模式
    USONIC_MODE_1S_CYCLE       // 1秒周期模式：1秒开1秒关

} usonic_mode_cmd_t;

/* 超声控制命令定义 */
typedef enum {
    USONIC_CMD_NONE = 0,
    USONIC_CMD_TURN_ON,         // 开启超声
    USONIC_CMD_TURN_OFF         // 关闭超声
} usonic_cmd_t;
/* 超声模式编译配置 */
// =============================================
// 选择开机后主函数将切换到的周期模式
// 可选：USONIC_MODE_1S_CYCLE 或 USONIC_MODE_0P5S_CYCLE
// =============================================
#define DEFAULT_CYCLE_MODE         USONIC_MODE_1S_CYCLE

/* 超声周期参数配置（根据上述选择自动设置） */
#if (DEFAULT_CYCLE_MODE == USONIC_MODE_0P5S_CYCLE)
    #define USONIC_CYCLE_PERIOD    1000    // 1秒周期
    #define USONIC_ON_TIME         500    // 0.5秒开启时间
    #define USONIC_STABLE_TIME     100     // 100ms稳定时间
#elif (DEFAULT_CYCLE_MODE == USONIC_MODE_1S_CYCLE)
    #define USONIC_CYCLE_PERIOD    2000    // 2秒周期
    #define USONIC_ON_TIME         1000     // 1秒开启时间
    #define USONIC_STABLE_TIME     200     // 200ms稳定时间
#endif
/* 全局变量声明 */
extern volatile usonic_mode_cmd_t usonic_mode_command;
extern volatile usonic_cmd_t usonic_command;
extern volatile uint8_t usonic_ready_for_detection;

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void TIM22_IRQHandler(void);
void TIM21_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#endif /* __STM32L0xx_IT_H */
