/*****************************************************************************
* @file			: MCU_init.h
* @short        : Header for MCU_init.c file.
*
******************************************************************************
*
* @details
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_INIT_H
#define __MCU_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#define DEBUG_MODE 1  // 1=调试模式 0=发布模式
void MCU_peripheral_init(void);

void Error_Handler(void);
void IWDG_Init(void);

/* Private defines -----------------------------------------------------------*/

// 定义硬件映射（可集中管理所有GPIO）
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioMap_t;

/* 安全操作宏 */
#define DEVICE_CTRL(device, state) do { \
    const GpioMap_t _dev = (device); \
    HAL_GPIO_WritePin(_dev.port, _dev.pin, (state)); \
} while (0)

#define NC_Pin GPIO_PIN_13
#define NC_GPIO_Port GPIOC
#define NCC14_Pin GPIO_PIN_14
#define NCC14_GPIO_Port GPIOC
#define NCC15_Pin GPIO_PIN_15
#define NCC15_GPIO_Port GPIOC

#define USONIC_PWR_PIN 					GPIO_PIN_2
#define USONIC_PWR_GPIO_Port 			GPIOC
#define USONIC_PWR_CTRL             	(GpioMap_t){USONIC_PWR_GPIO_Port, USONIC_PWR_PIN}
#define USONIC_PWR_ON()                 DEVICE_CTRL(USONIC_PWR_CTRL, GPIO_PIN_SET);
#define USONIC_PWR_OFF()                DEVICE_CTRL(USONIC_PWR_CTRL, GPIO_PIN_RESET);

#define NCC3_Pin GPIO_PIN_3
#define NCC3_GPIO_Port GPIOC
#define AD_IN0_Pin GPIO_PIN_0
#define AD_IN0_GPIO_Port GPIOA
#define AD_IN1_Pin GPIO_PIN_1
#define AD_IN1_GPIO_Port GPIOA
#define DA1_Pin GPIO_PIN_4
#define DA1_GPIO_Port GPIOA
#define AD_IN5_Pin GPIO_PIN_5
#define AD_IN5_GPIO_Port GPIOA
#define AD_IN6_Pin GPIO_PIN_6
#define AD_IN6_GPIO_Port GPIOA
#define AD_IN7_Pin GPIO_PIN_7
#define AD_IN7_GPIO_Port GPIOA
#define AD_IN14_Pin GPIO_PIN_4
#define AD_IN14_GPIO_Port GPIOC
#define AD_IN13_Pin GPIO_PIN_3
#define AD_IN13_GPIO_Port GPIOC
#define DO_Pin                       GPIO_PIN_5
#define DO_GPIO_Port                 GPIOC
#define LED_PWM_Pin          		 GPIO_PIN_5
#define LED_PWM_GPIO_Port 			 GPIOC
#define LED_PWM_CTRL             	 (GpioMap_t){LED_PWM_GPIO_Port, LED_PWM_Pin}
#define LED_PWM_ON()                 DEVICE_CTRL(LED_PWM_CTRL, GPIO_PIN_SET);
#define LED_PWM_OFF()                DEVICE_CTRL(LED_PWM_CTRL, GPIO_PIN_RESET);


#define DI_Pin                       GPIO_PIN_0
#define DI_GPIO_Port                 GPIOB
#define MAGNET_Pin          	     GPIO_PIN_0
#define MAGNET_Port    		 		 GPIOB
#define MAGNET_READ()       	     HAL_GPIO_ReadPin(MAGNET_Port, MAGNET_Pin)


#define DIB1_Pin                     GPIO_PIN_1
#define DIB1_GPIO_Port               GPIOB
#define POWER_SWITCH_Pin          	 GPIO_PIN_1
#define POWER_SWITCH_Port    		 GPIOB
#define POWER_SWITCH_READ()       	 HAL_GPIO_ReadPin(POWER_SWITCH_Port, POWER_SWITCH_Pin)

#define DIB2_Pin GPIO_PIN_2
#define DIB2_GPIO_Port GPIOB
#define AC_POWER_Pin          	 	 GPIO_PIN_2
#define AC_POWER_Port    		 	 GPIOB
#define AC_POWER_READ()       	 	 HAL_GPIO_ReadPin(AC_POWER_Port, AC_POWER_Pin)

#define DIB10_Pin 					 GPIO_PIN_10
#define DIB10_GPIO_Port 			 GPIOB
#define RUN_STOP_SWITCH_Pin          GPIO_PIN_10
#define RUN_STOP_SWITCH_GPIO_Port    GPIOB
#define RUN_STOP_SWITCH_READ()       HAL_GPIO_ReadPin(RUN_STOP_SWITCH_GPIO_Port, RUN_STOP_SWITCH_Pin)

#define DOB11_Pin GPIO_PIN_11
#define DOB11_GPIO_Port GPIOB
#define LED_HEAT_Pin          		 GPIO_PIN_11
#define LED_HEAT_GPIO_Port 			 GPIOB
#define LED_HEAT_CTRL             	 (GpioMap_t){LED_HEAT_GPIO_Port, LED_HEAT_Pin}
#define LED_HEAT_ON()                DEVICE_CTRL(LED_HEAT_CTRL, GPIO_PIN_SET);
#define LED_HEAT_OFF()               DEVICE_CTRL(LED_HEAT_CTRL, GPIO_PIN_RESET);


#define TPA_STEP_A_Pin GPIO_PIN_12
#define TPA_STEP_A_GPIO_Port GPIOB
#define TPA_STEP_B_Pin GPIO_PIN_13
#define TPA_STEP_B_GPIO_Port GPIOB
#define TPA_STEP_C_Pin GPIO_PIN_14
#define TPA_STEP_C_GPIO_Port GPIOB
#define TPA_STEP_D_Pin GPIO_PIN_15
#define TPA_STEP_D_GPIO_Port GPIOB
#define PUMP_STEP_A_Pin GPIO_PIN_6
#define PUMP_STEP_A_GPIO_Port GPIOC
#define PUMP_STEP_B_Pin GPIO_PIN_7
#define PUMP_STEP_B_GPIO_Port GPIOC
#define PUMP_STEP_C_Pin GPIO_PIN_8
#define PUMP_STEP_C_GPIO_Port GPIOC
#define PUMP_STEP_D_Pin GPIO_PIN_9
#define PUMP_STEP_D_GPIO_Port GPIOC

#define POWER_OFF_Pin           GPIO_PIN_8
#define POWER_OFF_GPIO_Port     GPIOA
#define POWER_CTRL              (GpioMap_t){POWER_OFF_GPIO_Port, POWER_OFF_Pin}
#define POWER_ON()              DEVICE_CTRL(POWER_CTRL, GPIO_PIN_RESET);
#define POWER_OFF()             DEVICE_CTRL(POWER_CTRL, GPIO_PIN_SET);


//#define LED_GRN_Pin             GPIO_PIN_9
//#define LED_GRN_GPIO_Port       GPIOA
//#define LED_GRN_CTRL            (GpioMap_t){LED_GRN_GPIO_Port, LED_GRN_Pin}

//#define LED_GRN_ON()            DEVICE_CTRL(LED_GRN_CTRL, GPIO_PIN_RESET);
//#define LED_GRN_OFF()           DEVICE_CTRL(LED_GRN_CTRL, GPIO_PIN_SET);

#define LED_AMB_Pin 			GPIO_PIN_9
#define LED_AMB_GPIO_Port		GPIOA
#define LED_AMB_CTRL            (GpioMap_t){LED_AMB_GPIO_Port, LED_AMB_Pin}
#define LED_AMB_ON()            DEVICE_CTRL(LED_AMB_CTRL, GPIO_PIN_SET);
#define LED_AMB_OFF()           DEVICE_CTRL(LED_AMB_CTRL, GPIO_PIN_RESET);

#define AUDIO_Pin GPIO_PIN_11
#define AUDIO_GPIO_Port GPIOA
#define VOLUME_Pin GPIO_PIN_12
#define VOLUME_GPIO_Port GPIOA

#define DOA15_Pin               GPIO_PIN_15
#define DOA15_GPIO_Port         GPIOA
#define READ_BATT_Pin           GPIO_PIN_15
#define READ_BATT_GPIO_Port     GPIOA
#define READ_BATT_CTRL              (GpioMap_t){READ_BATT_GPIO_Port, READ_BATT_Pin}
#define READ_BATT_ON()              DEVICE_CTRL(READ_BATT_CTRL, GPIO_PIN_SET);
#define READ_BATT_OFF()             DEVICE_CTRL(READ_BATT_CTRL, GPIO_PIN_RESET);


#define DIC10_Pin GPIO_PIN_10
#define DIC10_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_11
#define INT_GPIO_Port GPIOC
#define PD_Pin GPIO_PIN_12
#define PD_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define DOB8_Pin GPIO_PIN_8
#define DOB8_GPIO_Port GPIOB
#define NCB9_Pin GPIO_PIN_9
#define NCB9_GPIO_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MCU_INIT_H */
