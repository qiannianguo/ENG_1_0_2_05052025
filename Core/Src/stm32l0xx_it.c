
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "MCU_init.h"
#include "stm32l0xx_it.h"
#include "common.h"
#include "io_calls.h"
#include "functions.h"
#include <stdbool.h>
#include "stm32l0xx_hal_iwdg.h"  // 必须包含IWDG头文件
extern IWDG_HandleTypeDef hiwdg;  // 声明外部变量
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

 /* 电机相位控制表 - CCW旋转 */
 static const GPIO_PinState phase_table[4][4] = {
		 // A    B    C    D       // 相位说明（对应原始代码的case顺序）
	{SET,  SET,  RESET, RESET}, // 相位0: BA (A+B通电) - 对应原始case1
	{RESET, SET,  SET,  RESET}, // 相位1: CB (B+C通电) - 对应原始case2
	{RESET, RESET, SET,  SET},  // 相位2: DC (C+D通电) - 对应原始case3
	{SET,  RESET, RESET, SET}   // 相位3: AD (A+D通电) - 对应原始case0
  };

 // 全局状态变量  2025.10.30 jeffg
volatile usonic_cmd_t usonic_command = USONIC_CMD_NONE;
volatile usonic_mode_cmd_t usonic_mode_command = USONIC_MODE_DEFAULT;
volatile uint8_t usonic_ready_for_detection = 0;

extern volatile uint8_t G_tick5ms_counter;  //defined in main.c
extern volatile uint16_t G_motor_rate;
extern volatile uint16_t G_one_ml_count;
extern volatile uint8_t G_one_ml_flag;
extern volatile uint16_t G_alrm_mon_value;
extern volatile uint16_t G_ir_pd2_nocomp;
extern volatile uint16_t G_usonic_value, G_ir_pd2_value;
extern volatile uint16_t G_tp_value;
extern volatile uint16_t G_ntc_value;
extern uint8_t G_an_on;
extern volatile uint16_t G_v_battery, G_dac_cal_value, G_vpts;
extern volatile uint8_t G_dir;

extern volatile time_based_flow_cal_t G_time_cal;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim22;
extern TIM_HandleTypeDef htim21;
extern DAC_HandleTypeDef hdac;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
//void DMA1_Channel1_IRQHandler(void)
//{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
//}

/**
  * @brief TIM22全局中断处理函数
  * @note 每5ms触发一次，用于多通道ADC采样和数据处理
  */
void TIM22_IRQHandler(void)
{
  /* 静态变量声明区域 */
	static uint8_t an_count;          // ADC采样周期计数器（0-19循环，对应100ms）
	static uint16_t tp_sum;           // 触摸板温度ADC采样累加值
	static uint16_t usonic_sum;       // 超声波传感器ADC采样累加值
	static uint16_t sum2l, sum2d;     // PD2传感器（光感）亮/暗环境采样值
	static uint16_t sumpts;           // PTS传感器采样累加值
	static uint16_t ntc_sum;          // NTC温度传感器采样累加值
	static uint16_t alrm_mon_sum;     // 报警监控ADC采样累加值
	static uint32_t sumvbat;          // 电池电压采样累加值（32位防溢出）
	static uint16_t ir_pd2_dark;      // PD2传感器暗电流基准值
	static uint8_t v_sample_counter;  // 电池电压采样计数器（20次平均）

	   //新增 超声控制状态
	    static uint32_t usonic_on_stable_timer = 0;
	    static uint8_t usonic_power_state = 0;
	    static usonic_cmd_t last_command = USONIC_CMD_NONE;

#if !DEBUG_MODE
	static uint8_t feed_counter = 0;
	if (++feed_counter >= 40) {  // 200ms喂一次（40*5ms）
		HAL_IWDG_Refresh(&hiwdg);
		feed_counter = 0;
	}
#endif
	/* 中断处理 */
	HAL_TIM_IRQHandler(&htim22);
	G_tick5ms_counter++;  // 全局5ms计数器递增

    /* 处理超声控制通知 */
    if (usonic_command != last_command) {
        last_command = usonic_command;

        switch (usonic_command) {
            case USONIC_CMD_TURN_ON:
                if (!usonic_power_state) {
                    USONIC_PWR_ON();
                    usonic_power_state = 1;
                    usonic_on_stable_timer = 0;
                    usonic_ready_for_detection = 0;
                }
                break;

            case USONIC_CMD_TURN_OFF:
                if (usonic_power_state) {
                    USONIC_PWR_OFF();
                    usonic_power_state = 0;
                    usonic_ready_for_detection = 0;
                }
                break;

            default:
                break;
        }
    }

    /* 超声开启后等待稳定 */
    if (usonic_power_state) {
        usonic_on_stable_timer += 5;

        if (usonic_on_stable_timer > USONIC_STABLE_TIME) {
            usonic_ready_for_detection = 1;
        }
    }

	/* 模拟量采样使能检查（音频播放时禁用采样）*/
	if (G_an_on) {
		an_count++;
		if (an_count > 19) an_count = 0;  // 100ms周期循环（20x5ms）

		/* 分时采样调度 */
		switch (an_count) {
		/*----- 采样阶段 -----*/
		case 0:  // 0ms - 初始化阶段
			// 清空所有累加器
			tp_sum = usonic_sum = sum2l = sum2d = sumpts = 0;
			alrm_mon_sum = ntc_sum = 0;
			break;

		case 1:  // 5ms - NTC温度采样
			select_analog_channel(CH_TP_NTC);
			for (uint8_t i = 0; i < 5; i++) ntc_sum += read_analog_channel();
				break;

        case 2:  // 10ms - PD2亮环境采样 ,默认模式开启超声
            select_analog_channel(CH_PD2);
            for (uint8_t i = 0; i < 5; i++){
                sum2l += read_analog_channel();
            }
            if (usonic_mode_command == USONIC_MODE_DEFAULT) {
                USONIC_PWR_ON();
            }
            break;
			break;

		case 3:  // 15ms - 触摸板温度采样
			select_analog_channel(CH_TP_TEMP);
			for (uint8_t i = 0; i < 5; i++){
				tp_sum += read_analog_channel();

			}
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // 关闭DAC输出
			break;

		case 4:  // 20ms - 报警监控采样
			select_analog_channel(CH_ALRM_MON);
			for (uint8_t i = 0; i < 5; i++)	alrm_mon_sum += read_analog_channel();
				break;

		case 5:  // 25ms - 超声波传感器采样
            if (usonic_mode_command == USONIC_MODE_DEFAULT) {
                select_analog_channel(CH_USONIC);
                for (uint8_t i = 0; i < 5; i++) {
                    usonic_sum += read_analog_channel();
                }
                USONIC_PWR_OFF();
            } else {
                if (usonic_power_state) {
                    select_analog_channel(CH_USONIC);
                    for (uint8_t i = 0; i < 5; i++) {
                        usonic_sum += read_analog_channel();
                    }
                }
            }
            break;

		case 9:  // 45ms - 二次NTC采样
			select_analog_channel(CH_TP_NTC);
			for (uint8_t i = 0; i < 5; i++)	ntc_sum += read_analog_channel();
				break;

		case 10: // 50ms - PD2暗环境采样
			select_analog_channel(CH_PD2);
			for (uint8_t i = 0; i < 5; i++)	sum2d += read_analog_channel();
				break;

		case 11: // 55ms - 二次触摸板温度采样
			select_analog_channel(CH_TP_TEMP);
			for (uint8_t i = 0; i < 5; i++){
				tp_sum += read_analog_channel();

			}
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, G_dac_cal_value); // 恢复DAC输出
			break;

				/*----- 数据处理阶段 -----*/
		case 12: // 60ms - 传感器数据计算
			// PD2光感补偿计算（亮值-暗值）
			ir_pd2_dark = sum2d / 5;
			G_ir_pd2_nocomp = sum2l / 5;
			G_ir_pd2_value = (G_ir_pd2_nocomp > ir_pd2_dark) ?
                        (G_ir_pd2_nocomp - ir_pd2_dark) : 0;

			// 其他传感器平均值计算,更新超声波数值,只在超声开启时更新超声波数值
           if (usonic_mode_command == USONIC_MODE_DEFAULT) {
                // 默认模式：直接更新
                G_usonic_value = usonic_sum / 5;
            } else {
                // 周期模式：只在开启时更新
                if (usonic_power_state) {
                    G_usonic_value = usonic_sum / 5;
                }
            }

			G_alrm_mon_value = alrm_mon_sum / 5;
			G_tp_value = tp_sum / 10;  // 两次采样的10点平均

			test_background_ir(ir_pd2_dark);  // 红外背景光测试
			break;

		case 13: // 65ms - 电池电压采样
			select_analog_channel(CH_V_BAT);
			for (uint8_t i = 0; i < 5; i++){
				sumvbat += read_analog_channel();
				v_sample_counter++;
			}
			break;

		case 14: // 70ms - PTS传感器采样
			select_analog_channel(CH_PTS);
			for (uint8_t i = 0; i < 5; i++)
				sumpts += read_analog_channel();
			break;

		case 15: // 75ms - 数据最终处理
			if (v_sample_counter > 19) {  // 每100ms计算一次电池电压（20x5ms）
				v_sample_counter = 0;
				//G_v_battery = 10 * sumvbat / 238;  // 换算为实际电压（假设比例系数为10/238）
				G_v_battery = 5 * sumvbat / 238;  // 换算为实际电压（假设比例系数为10/238）
				sumvbat = 0;
			}
			G_vpts = sumpts / 5;  // PTS传感器平均值
			break;

		case 16: // 80ms - NTC最终计算
			G_ntc_value = ntc_sum / 10;  // 两次采样的10点平均
			break;

		default:
			break;
		}
	}
}

/**
  * @brief This function handles TIM22 global interrupt.
  */

#if(0)
void TIM22_IRQHandler(void)
{
  /* USER CODE BEGIN TIM22_IRQn 0 */
static uint8_t an_count;
static uint16_t tp_sum,usonic_sum,sum2l,sum2d,sumpts;
static uint16_t ntc_sum;
static uint16_t alrm_mon_sum;
static uint32_t sumvbat;
static uint16_t ir_pd2_dark;
static uint8_t v_sample_counter;
uint8_t i;
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_TIM_IRQHandler(&htim22);
  G_tick5ms_counter++;
  if (G_an_on){       //Analog conversions are off during the audiple
	  	  	  	  	  //signals to prevent sound distortion
    an_count++;
    if (an_count>19) an_count=0;    //18 = 100ms
    switch (an_count){                           //Take analog readings
      case 0:                                    //0ms
        tp_sum = 0;                                //Clear AD sum vars
        usonic_sum = 0;
        sum2l = 0;
        sum2d = 0;
        sumpts = 0;
        alrm_mon_sum = 0;
        ntc_sum=0;
        break;
      case 1:                                      //5ms
        //  select_analog_channel(CH_USONIC);
        //  for (i=0; i<5; i++) usonic_sum = usonic_sum + read_analog_channel();
    	select_analog_channel(CH_TP_NTC);
    	for (i=0; i<5; i++) ntc_sum = ntc_sum + read_analog_channel();
    	break;
      case 2:                                      //10ms
        select_analog_channel(CH_PD2);
        for (i=0; i<5; i++) sum2l = sum2l + read_analog_channel();
        HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_SET);   //Usonic sensor power on
        break;
      case 3: 										 //15ms
    	select_analog_channel(CH_TP_TEMP);
    	for (i=0; i<5; i++) tp_sum = tp_sum + read_analog_channel();
    	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); //clr dac
    	break;
      case 4:                                      //20ms
        select_analog_channel(CH_ALRM_MON);
        for (i=0; i<5; i++) alrm_mon_sum = alrm_mon_sum + read_analog_channel();
        break;
      case 5:                                      //25ms
        select_analog_channel(CH_USONIC);
        for (i=0; i<5; i++) usonic_sum = usonic_sum + read_analog_channel();
      HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_RESET);   //Usonic sensor power off
        break;
      case 9:
    	select_analog_channel(CH_TP_NTC);
    	for (i=0; i<5; i++) ntc_sum = ntc_sum + read_analog_channel();
    	break;

      case 10:                                    //50ms
        select_analog_channel(CH_PD2);
        for (i=0; i<5; i++) sum2d = sum2d + read_analog_channel();
        break;
      case 11: 										//55ms
        select_analog_channel(CH_TP_TEMP);
        for (i=0; i<5; i++) tp_sum = tp_sum + read_analog_channel();
        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1, DAC_ALIGN_12B_R, G_dac_cal_value); //
        break;
      case 12:   									//60ms
        ir_pd2_dark = sum2d/5 ;
        G_usonic_value = usonic_sum/5;
        G_alrm_mon_value = alrm_mon_sum / 5;

        G_ir_pd2_nocomp= sum2l/5;
        G_ir_pd2_value = G_ir_pd2_nocomp - ir_pd2_dark;
        if (G_ir_pd2_nocomp < ir_pd2_dark)  G_ir_pd2_value=0;
        test_background_ir(ir_pd2_dark);
        G_tp_value = tp_sum/10;
        break;
      case 13:										//65ms
    	select_analog_channel(CH_V_BAT);
    	for (i=0; i<5; i++) sumvbat = sumvbat + read_analog_channel();
    	v_sample_counter++;
    	break;
      case 14:										//70ms
        select_analog_channel(CH_PTS);
        for (i=0; i<5; i++) sumpts = sumpts + read_analog_channel();
        break;
      case 15:										//75ms
        if (v_sample_counter>19){
          v_sample_counter=0;
          G_v_battery = 10*sumvbat/238;
          sumvbat=0;
        }
        G_vpts = sumpts/5;
        break;
      case 16:
    	G_ntc_value = ntc_sum/10;
    	break;
      default :
        break;
    }
  }
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /* USER CODE END TIM22_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim22);
  /* USER CODE BEGIN TIM22_IRQn 1 */

  /* USER CODE END TIM22_IRQn 1 */
}
#endif
/**
  * @brief TIM21全局中断处理函数 - 优化版
  * @note 使用状态机驱动步进电机，优化了相位切换逻辑和能耗管理
  */
#if(1)
void TIM21_IRQHandler(void)
{
  /* 静态变量声明 */
  static uint8_t phase = 0;         // 当前电机相位(0-3)
  static uint16_t cycle_cnt = 1;    // 周期计数器(1-FULL_FOOD_RATE)
  static bool motor_active = false; // 电机激活状态标志

  /* HAL库中断处理 */
  HAL_TIM_IRQHandler(&htim21);



  /* 1. 占空比控制阶段 */
  if (cycle_cnt <= G_motor_rate) {
    if (!motor_active) {
      motor_active = true;
      // 电机激活时恢复相位状态
      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, phase_table[phase][0]);
      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, phase_table[phase][1]);
      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, phase_table[phase][2]);
      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, phase_table[phase][3]);
    }

    /* 步进电机相位切换 */
    uint8_t new_phase = (phase + 3) % 4;
    if (new_phase != phase) {
      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, phase_table[new_phase][0]);
      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, phase_table[new_phase][1]);
      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, phase_table[new_phase][2]);
      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, phase_table[new_phase][3]);
      phase = new_phase;
    }

    /* 液体计量处理 */
    if (++G_one_ml_count >= G_time_cal.one_ml_corrected) {
    //	if (++G_one_ml_count >= ONE_ML) {
      G_one_ml_count = 0;
      if (G_one_ml_flag > 0) {
        gen_err_handler(ERR_ML_COUNT_TO); // 超时错误处理
      }
      G_one_ml_flag = 1; // 主程序需在RUN模式清除此标志
    }
  }
  /* 2. 节能控制阶段 */
  else {
    /* 延迟关闭电机以降低开关损耗 */
    if (cycle_cnt == (G_motor_rate + MOTOR_OFF_STEP_COUNT)) {
      motor_active = false;
      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
    }

    /* 提前激活为下一个周期准备 */
    if (cycle_cnt == (FULL_FOOD_RATE - MOTOR_ON_STEP_COUNT)) {
      motor_active = true;
      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, phase_table[phase][0]);
      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, phase_table[phase][1]);
      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, phase_table[phase][2]);
      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, phase_table[phase][3]);
    }
  }

  /* 周期计数器管理 */
  cycle_cnt = (cycle_cnt >= FULL_FOOD_RATE) ? 1 : (cycle_cnt + 1);
}
#endif

/**
  * @brief This function handles TIM21 global interrupt.
  */
#if(0)
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
static uint8_t pump_phase = 0;
static uint16_t rate_count = 1;

  HAL_TIM_IRQHandler(&htim21);
  if (rate_count <= G_motor_rate){
    switch (pump_phase){
      case 3:							// full step CCW from position AD to position DC
   	    HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);
  	    HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_SET);  //
	    HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
	    pump_phase =2;
	    break;
	  case 2:							// full step CCW from position DC to position CB
	    HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_SET);//
	    HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);
	    pump_phase=1;
	    break;
	  case 1:							// full step CCW from position CB to position BA
	    HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_SET);//
	    HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	    pump_phase=0;
	    break;
	  case 0:							// full step CCW from position BA to position AD
	    HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_SET);//
	    HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	    pump_phase=3;
	    break;
	  default:
	    pump_phase=0;
	    HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);	//turn coil off and reset the pump phase
	    HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);	//if phase not defined
	    HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	    break;
    }
    G_one_ml_count++;
    if (G_one_ml_count >= ONE_ML){  //if one ml deliver (based on number of pump steps
      if (G_one_ml_flag > 0){
    	//place error handle here, one_ml_count timeout (main program not responding)
    	gen_err_handler(ERR_ML_COUNT_TO);
      }
      G_one_ml_count = 0;
      G_one_ml_flag = 1;              //this flag will be cleared in RUN mode after all volume counters
    }								//have been incremented
  }
  else{
	if (rate_count == (G_motor_rate + MOTOR_OFF_STEP_COUNT)){
	  HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);	//Wait few cycles then turn coils off to
	  HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);	//save power and to keep motor cooler.
	  HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	}
	if (rate_count == (FULL_FOOD_RATE - MOTOR_ON_STEP_COUNT)){  //Use food rate here since water rate dutycycle is always full
	  switch (pump_phase){               						//If dutycycle control is used for water then this has to be changed
	    case 2:
	      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);  //Turn power on few cycles before running the motor
	      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_SET);    //recover the correct phase
	      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
	      break;
	    case 1:
	      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);
	      break;
	    case 0:
	      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	      break;
	    case 3:
	      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	      break;
	    default:
	      pump_phase=0;
	      HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);	//turn coil off and reset the pump phase
	      HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);	//if phase not defined
	      HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	      break;
	  }
	}
  }
  rate_count++;
  if (rate_count > FULL_FOOD_RATE) rate_count = 1;    	//Use food rate here since water rate dutycycle is always full
}
#endif


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
