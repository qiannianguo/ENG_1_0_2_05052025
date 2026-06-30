/*****************************************************************************
* @file			: functions.c
* @short        : misc functions
******************************************************************************
*
* @details
*
*
*
******************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <main.h>
#include <stm32l0xx_hal.h>
#include <stm32l0xx_it.h>
#include "common.h"
#include "test_para.h"
#include "io_calls.h"
#include "functions.h"
#include "screens.h"
#include "eve.h"
#include <stdbool.h>
#include <float.h>  // 添加这行
#include <stdlib.h>
// 一阶IIR低通滤波器
#define ALPHA 0.7f  // 滤波系数(0~1，越小滤波越强)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static uint8_t skip_occlusion = 0;

/* 调节阶段阈值定义（基于50ms周期） */
#define INITIAL_DELAY     20    // 初始延迟20次计数（1秒）
#define SLOW_PHASE_END    70    // 慢速阶段结束点（3.5秒）
#define ACCEL_POINT_5S    100   // 5秒加速点（步长增至10）
#define ACCEL_POINT_8S    160   // 8秒加速点（步长增至50）

#define NORMAL_PEAK_INDEX 33   // 正常数据的峰值位置
// 定义稳定阶段和非稳定阶段的缩放比例
#define NORMAL_SCALE_FACTOR     600  // 非稳定阶段缩放比例
#define STABLE_SCALE_FACTOR     200  // 稳定阶段缩放比例
#define TRANSITION_SIZE         10
// 定义常量
#define STABLE_PHASE_MIN_LENGTH 20    // 稳定阶段最小长度
#define STABILITY_VARIANCE_THRESHOLD 12.0f // 稳定性方差阈值

extern UART_HandleTypeDef huart2; //^^*prt testing
extern TIM_HandleTypeDef htim22;
extern I2C_HandleTypeDef hi2c3;
extern volatile uint16_t G_one_ml_count;
extern int16_t G_tp_value;
extern int8_t G_an_on;
extern int8_t G_bg_ir_test_enabled;
extern  uint16_t G_motor_rate;
extern uint8_t G_one_ml_flag;
extern uint16_t G_volume, G_hour_volume,G_water,running_volume;
extern uint16_t G_food_rate;
extern uint16_t G_total_volume;
extern uint16_t G_acc_formula_ml;       			//accumulated formula volume in ml
extern uint16_t G_acc_bolus_ml;				    //accumulated bolus volume in ml
extern uint16_t G_dac_cal_value;
//extern uint8_t g_first_after_bolus;
extern volatile uint8_t air_usonic_detected_flag;
extern uint16_t G_low_flow_mode;
extern uint16_t G_dose_limit;

extern int16_t occl_comp_peak_offset;

extern uint8_t occl_comp_count_test,occl_peak_count_test;
extern volatile int16_t occl_comp_count_test_value[20];
extern volatile uint8_t occl_comp_count_test_position[20];

static void swap(int16_t *a, int16_t *b) ;
static void low_pass_filter(int16_t *data, uint16_t len);
//static int16_t preprocess_data(int16_t occl_data[], int16_t *baseline) ;
//static void normalize_data(int16_t occl_data[], int16_t max_value) ;
static void end_heating_phase(uint16_t* air_rate_inc);
static void start_test_phase(uint16_t *timer);
static bool should_check_air(uint16_t occl_timer);
static void handle_air_condition(uint16_t* occl_timer, uint16_t* air_rate_inc);
static uint8_t process_test_results(int16_t occl_data[], uint16_t* air_rate_inc);


static void detect_and_handle_invalid_values(int16_t occl_data[]);
//static uint16_t find_first_valid_index(int16_t data[]);
//static int16_t find_prev_valid_value(int16_t data[], uint16_t index);
//static int16_t find_next_valid_value(int16_t data[], uint16_t index);
//static int16_t find_last_valid_value(int16_t data[]);
static uint16_t find_end_of_initial_decline_enhanced(int16_t occl_data[]);
//static int16_t calculate_hybrid_baseline(int16_t occl_data[]) ;
//static int16_t calculate_initial_average(int16_t data[]);
//static int16_t calculate_weighted_average(int16_t initial_avg, int16_t stable_avg);
//static int16_t calculate_weighted_average(int16_t initial_avg, int16_t stable_avg, int16_t max_value);
//static int16_t calculate_hybrid_baseline_with_stable(int16_t occl_data[], uint16_t stable_start, uint16_t stable_end, bool has_stable_phase);

static int16_t calculate_hybrid_baseline_with_stable(int16_t data[], uint16_t front_start, uint16_t front_end,
                                                    uint16_t back_start, uint16_t back_end, bool has_stable_phase, int16_t max_value);
static int16_t get_representative_value(int16_t data[], uint16_t stable_start, uint16_t stable_end,
                                       bool has_stable_phase, bool is_front);
//static int16_t calculate_front_non_stable_avg(int16_t data[], uint16_t stable_start);
//static int16_t calculate_back_non_stable_avg(int16_t data[], uint16_t stable_end);
//static int16_t calculate_initial_baseline(int16_t data[]);
//static void normalize_data_flexible(int16_t occl_data[], int16_t max_value, uint16_t stable_start);
static void normalize_data_flexible(int16_t occl_data[], int16_t max_value,
                                   uint16_t front_start, uint16_t front_end,
                                   uint16_t back_start, uint16_t back_end,
                                   bool has_stable_phase);
static uint16_t find_peak_index(int16_t occl_data[]);
//static int16_t calculate_baseline_from_stable_phase(int16_t occl_data[]);
//static bool detect_stable_phase_with_adjust(int16_t data[], uint16_t *start_index, uint16_t *end_index);
static bool detect_stable_phase_with_adjust(int16_t data[], uint16_t *front_start, uint16_t *front_end,
                                           uint16_t *back_start, uint16_t *back_end);
//static bool detect_stable_phase(int16_t data[], uint16_t *start_index, uint16_t *end_index);
//static bool check_window_stability(int16_t data[], uint16_t start, uint8_t window_size,float variance_threshold, int16_t derivative_threshold);


//static uint16_t adjust_start_boundary(int16_t data[], uint16_t start, float variance_threshold);
//static uint16_t adjust_end_boundary(int16_t data[], uint16_t end, float variance_threshold);
//static float calculate_window_variance(int16_t data[], uint16_t start, uint8_t size);
//static int16_t calculate_mean(int16_t data[], uint16_t start, uint16_t end);
//static int16_t calculate_baseline_from_initial(int16_t data[]);
static int16_t find_peak_value(int16_t occl_data[]);
//static void smooth_pre_stable_phase(int16_t occl_data[],bool has_stable_phase,uint16_t stable_start, uint16_t stable_end);
static void smooth_pre_stable_phase(int16_t data[], bool has_stable_phase,
                                   uint16_t front_start, uint16_t front_end,
                                   uint16_t back_start, uint16_t back_end);
//static void sliding_average_filter_range(int16_t *data, uint16_t start, uint16_t end, uint8_t window_size);
//static void process_transition_region(int16_t *data, uint16_t smooth_end, uint16_t stable_start);

// 定义全局状态变量（本文件真正存储它）
occl_state_t g_occl_state = OCCL_STATE_IDLE;
// 获取当前闭塞检测状态的函数

occl_state_t get_occl_state(void) {
    return g_occl_state;
}

#ifndef DISABLE_8TH_TEST_CHECK
static uint8_t will_have_8th_test(dynamic_interval_control_t *control, uint32_t current_time, uint32_t time_remaining_ms);
static uint16_t calculate_detection_volume_current(dynamic_interval_control_t *control, uint32_t test_start_time, uint32_t hour_start_time);
static void calculate_predicted_errors(uint16_t current_volume, uint16_t predicted_expected,
                                     uint16_t volume_with_test, uint16_t volume_without_test,
                                     int16_t *error_with_test, int16_t *error_without_test);
static uint8_t low_flow_decision_logic(uint8_t will_have_8th_test, int16_t error_with_test,
                                     int16_t error_without_test, uint16_t time_remaining_sec,
                                     uint16_t current_volume, uint16_t G_food_rate,
                                     uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                     dynamic_interval_control_t *control, uint16_t predicted_expected,  // 添加这两个参数
                                     uint16_t *decision_interval);
static uint8_t high_flow_decision_logic(uint8_t will_have_8th_test, int16_t error_with_test,
                                      int16_t error_without_test, uint16_t time_remaining_sec,
                                      uint16_t current_volume, uint16_t G_food_rate,
                                      uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                      dynamic_interval_control_t *control, uint16_t predicted_expected,  // 添加这两个参数
                                      uint16_t *decision_interval);
static uint8_t check_accelerate_effect(uint16_t current_volume, uint16_t G_food_rate,
                                     uint32_t time_remaining_ms, uint16_t test_volume_ml);
static uint8_t check_accelerate_effect_high_flow(uint16_t current_volume, uint16_t G_food_rate,
                                               uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                               uint16_t predicted_expected, uint16_t *best_interval);
static uint8_t check_partial_delay_feasible(uint32_t delay_step_ms, uint32_t current_time,
                                          dynamic_interval_control_t *control);
static uint8_t handle_low_flow_full_delay(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                        dynamic_interval_control_t *control,
                                        uint16_t G_food_rate, uint16_t current_volume,
                                        uint16_t *decision_interval);
static uint8_t handle_low_flow_partial_delay(uint32_t time_remaining_ms, uint32_t current_time,
                                           dynamic_interval_control_t *control,
                                           uint16_t current_volume, uint16_t G_food_rate,
                                           uint16_t predicted_expected, uint16_t *decision_interval);
static uint8_t handle_high_flow_delay(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                    dynamic_interval_control_t *control,
                                    uint16_t G_food_rate, uint16_t current_volume,
                                    uint16_t *decision_interval);
static int16_t calculate_partial_delay_error(uint32_t delay_step_ms, uint32_t current_time,
                                           dynamic_interval_control_t *control, uint16_t current_volume,
                                           uint16_t G_food_rate, uint16_t predicted_expected);
static uint8_t handle_delay_decision(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                   int16_t error_with_test, int16_t error_without_test,
                                   uint16_t current_volume, uint16_t G_food_rate,
                                   dynamic_interval_control_t *control, uint16_t predicted_expected,
                                   uint32_t current_time, uint16_t *decision_interval);

static uint32_t estimate_next_test_time(dynamic_interval_control_t *control, uint32_t current_time);
#endif
static uint8_t process_occlusion_result(uint8_t ret, uint8_t *error_count, uint16_t *current_interval,
                                       uint16_t normal_interval, uint16_t fast_interval);

static void increment_air_count(uint8_t *air_ml_count);
static void increment_food_volumes(uint16_t *food_volume_increment, dynamic_interval_control_t *control);
static void increment_bolus_volumes(uint16_t *bolus_run_volume);
static void handle_low_flow_hour_complete(dynamic_interval_control_t *control);
static void handle_low_flow_dose_complete(dynamic_interval_control_t *control);
static void handle_no_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control);
static void handle_no_pause_hour_first(uint16_t *food_volume_increment, dynamic_interval_control_t *control);
static void handle_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control);
static void handle_pause_restart_motor(uint8_t pause_motor_off_flag);
static void handle_pause_early_complete(dynamic_interval_control_t *control, uint8_t *pause_motor_off_flag);
static void check_no_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control);
static void check_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control);


/**************************************
 * gen_err_handler()
 * @detail
 * Error handler for non HAL layer errors
 */
void gen_err_handler(uint8_t err_num)
{
	pump_motor_off();             //motor is stopped by disabling the TIM21 interrupt
	ir_heat_off();			    // which also stops incrementing the mL counters
	save_err_rec(err_num);
	// eve_initialize();                   //Required incase the handler is called during the display routine
	// show_error_number(err_num);         //!!This does not always reset the eve controller -> program gets stuck
  										//  work out the solution and activate the code later.
	power_off_pump();
}/**  End gen_err_handler()  **/
/**************************************************************
 * @brief 更新输送液体体积计数器
 * @param bolus_run 是否处于快速推注模式 (1=推注模式, 0=营养液模式)
 * @param air_ml_count 空气泡计数器指针(单位: mL)
 * @param bolus_run_volume 当前推注已输送水量指针(单位: mL)
 * @param food_volume_increment 营养液增量计数器指针(单位: mL)
 * @detail
 * - 每当检测到1mL液体输送时(G_one_ml_flag触发)，更新所有相关体积计数器
 * - 根据运行模式区分统计营养液体积和推注体积
 * - 空气泡计数上限为50mL（安全限制）
 */
#if(0)
void increment_volumes(uint8_t bolus_run, uint8_t *air_ml_count,
                      uint16_t *bolus_run_volume, uint16_t *food_volume_increment,dynamic_interval_control_t *control)
{
    // 仅当1mL输送标志置位时处理
    if (G_one_ml_flag) {
        // 重置标志位（必须放在最前面，避免递归调用）
        G_one_ml_flag = 0;

        /* 空气泡安全检测 */
        if (*air_ml_count < 50) {
            (*air_ml_count)++;  // 空气体积计数（最大50mL）
        }

        /* 模式区分处理 */
        if (!bolus_run) {
            // ===== 营养液输送模式 =====
            (*food_volume_increment)++;    // 当前批次营养液增量
            G_hour_volume++;               // 小时累计量
            G_volume++;                    // 当前会话总量
            running_volume++;
            if (running_volume > MAX_RUNNING_VOLUME) {
                running_volume = MAX_RUNNING_VOLUME;
            }
            G_total_volume++;              // 设备生命周期总量
            G_acc_formula_ml++;            // 累计营养液总量

            // ====== 低速模式：检查是否达到设定的小时剂量 ======
            // 低速模式检查逻辑简化
            if (G_low_flow_mode) {
                // 直接使用全局变量 G_hour_volume 和 G_food_rate
                if (G_hour_volume >= G_food_rate) {
                    // 达到小时剂量，立即停止马达
                    pump_motor_off();
                    uint32_t current_time = HAL_GetTick();
                    uint32_t elapsed_time = current_time - control->hour_start_time;

                    // 确保已运行时间不超过1小时
                    if (elapsed_time >= HOUR_DURATION_MS) {
                        elapsed_time = HOUR_DURATION_MS;
                    }

                    // 计算新的等待时间
                    uint32_t new_wait_time = HOUR_DURATION_MS - elapsed_time;


#ifdef LOW_FLOW_TEST_MODE
                    control->low_flow_wait.wait_duration = new_wait_time;
#else
                    uint32_t initial_wait_time = control->low_flow_wait.wait_duration;
                    // 比较差异是否超过1分钟（60000ms）
                    uint32_t time_diff = (new_wait_time > initial_wait_time) ?
                                        (new_wait_time - initial_wait_time) :
                                        (initial_wait_time - new_wait_time);

                    // 选择等待时间
                    control->low_flow_wait.wait_duration = (time_diff > 60000) ?
                                                          initial_wait_time :
                                                          new_wait_time;
#endif
                    // 设置等待状态
                    control->low_flow_wait.wait_start_time = current_time;
                    control->low_flow_wait.is_waiting = 1;
                }
      //            if (G_dose_limit < G_food_rate && G_volume >= G_dose_limit) {
                if ( G_volume >= G_dose_limit) {
                    // 情况B：第一个小时内就完成了总剂量
                    pump_motor_off();
                    uint32_t current_time = HAL_GetTick();
                    uint32_t elapsed_time = current_time - control->hour_start_time;
                    // 确保已运行时间不超过1小时
                    if (elapsed_time >= HOUR_DURATION_MS) {
                        elapsed_time = HOUR_DURATION_MS;
                    }
                    // 计算实际要运行时间
                    float actual_run_time_hours = (float)G_hour_volume / G_food_rate;
                    uint32_t actual_run_time_ms = (uint32_t)(actual_run_time_hours * HOUR_DURATION_MS);

                    // 计算等待时间 = 1小时 - 实际运行时间
                    uint32_t wait_time_ms = actual_run_time_ms - elapsed_time;

                    // 设置等待状态
                    control->low_flow_wait.is_waiting = 1;
                    control->low_flow_wait.wait_start_time = current_time;
                    control->low_flow_wait.wait_duration = wait_time_ms;

                    // 不需要设置 hour_dose_completed，因为剂量已经完成
                    // 让 handle_dose_check 处理剂量完成逻辑
                }
            }
            // 正常模式：不检查小时剂量，保持原有连续运行逻辑
       }
        else {
            // ===== 推注模式 =====
            G_acc_bolus_ml++;             // 累计推注总量
            (*bolus_run_volume)++;        // 当前推注运行量（用于中断恢复）
            G_water++;

            // 推注模式不受低速模式的小时剂量限制影响
        }
        if(!control->has_pause_this_period){
            	if(control->current_hour_tests>=7){
            		// 每次进入递增计数器
            		static uint16_t delay_counter = 0;  // 改为静态变量
            		// 递增计数器，防止溢出
            		if (delay_counter < 200) {
            			delay_counter++;
            		}
            		if (delay_counter >= 200) {
            			delay_counter = 0;
            			uint32_t current_time = HAL_GetTick();
            			uint32_t elapsed_time = current_time - control->hour_start_time;
            			if (elapsed_time >= HOUR_DURATION_MS) {
                        // 计算本小时实际应喂食量
            				int expected_volume = G_food_rate;

                        // 如果实际喂食量未达到目标
            				if (G_hour_volume < expected_volume) {
                            // 计算需要补足的量
            					int supplement_volume = expected_volume - G_hour_volume;

                            // 补足到目标量
            					G_hour_volume = expected_volume;

                            // 直接增加补充的量（不需要循环）
            					(*food_volume_increment) += supplement_volume;    // 当前批次营养液增量
            					G_volume += supplement_volume;                    // 当前会话总量
            					G_total_volume += supplement_volume;              // 设备生命周期总量
            					G_acc_formula_ml += supplement_volume;            // 累计营养液总量
            				}

                        // 重置小时计数器（如果需要进入下一小时）
                        // elapsed_time = 0;  // 这取决于您的计时器重置逻辑
            			}

            		}
            	}
            }
            else{
            	if(control->accumulated_work_time > PAUSE_ACCUMulated_TIME_THRESHOLD){
            		static uint16_t delay_counter = 0;  // 改为静态变量
            		static uint8_t pause_motor_off_flag = 0;
            		// 递增计数器，防止溢出
            		if (delay_counter < 200) {
            			delay_counter++;
            		}
            		if (delay_counter >= 200) {
            			delay_counter = 0;
            			uint32_t elapsed_time = control->accumulated_work_time;
            			if (elapsed_time >= HOUR_DURATION_MS) {
            				// 计算本小时实际应喂食量
            				int expected_volume = G_food_rate;

            				// 如果实际喂食量未达到目标
            				if (control->pause_flow_total < expected_volume) {
            					// 计算需要补足的量
            					int supplement_volume = expected_volume - control->pause_flow_total;

            					// 补足到目标量
            					G_hour_volume = expected_volume;

            					// 直接增加补充的量（不需要循环）
            					(*food_volume_increment) += supplement_volume;    // 当前批次营养液增量
            					G_volume += supplement_volume;                    // 当前会话总量
            					G_total_volume += supplement_volume;              // 设备生命周期总量
            					G_acc_formula_ml += supplement_volume;            // 累计营养液总量
            				}
            			    // 启动新输送模式
            				if(pause_motor_off_flag == 1){
                			    start_pump(G_motor_rate, MOTOR_INT_FORMULA);
                				// 重置小时计数器（如果需要进入下一小时）
                				// elapsed_time = 0;  // 这取决于您的计时器重置逻辑
                			    pause_motor_off_flag = 0;
            				}

            			}
            			else{
            				if(control->pause_flow_total>=G_food_rate){
            					pump_motor_off();
            					ir_heat_off();
            					pause_motor_off_flag = 1;
            				}
            			}
            		}
        		}

            }
    }

}
#endif
void increment_volumes(uint8_t bolus_run, uint8_t *air_ml_count,
                      uint16_t *bolus_run_volume, uint16_t *food_volume_increment,
                      dynamic_interval_control_t *control)
{
    if (G_one_ml_flag) {
        G_one_ml_flag = 0;

        increment_air_count(air_ml_count);

        if (!bolus_run) {
            // 营养液输送模式
            increment_food_volumes(food_volume_increment, control);



            if (G_low_flow_mode) {
                if (G_hour_volume >= G_food_rate) {
                    handle_low_flow_hour_complete(control);
                }
                if (G_volume >= G_dose_limit) {
                    handle_low_flow_dose_complete(control);
                }
            }
        } else {
            // 推注模式
            increment_bolus_volumes(bolus_run_volume);
        }

        // 小时结束判断
        if (!control->has_pause_this_period) {
            if (G_food_rate >=50&& !control->compensation_active) {
                check_no_pause_hour_end(food_volume_increment, control);
            }
        } else {
            // 有暂停时，先判断累计工作时间是否大于50分钟
            if (control->accumulated_work_time > 50 * 60000) {  // 50分钟 = 50 * 60000 毫秒
                check_pause_hour_end(food_volume_increment, control);
            }
            // 否则不处理，继续等待
        }
    }
    if (control->compensation_active && control->compensation_ml > 0) {
        // 每次额外 +1ml
        // 限制最大补偿量不超过 5
        uint8_t max_compensation = 8;
        if (control->compensation_ml > max_compensation) {
            control->compensation_ml = max_compensation;
        }
    	 static uint16_t delay_counter = 0;

    	 if (delay_counter < 60*400/G_food_rate) {
    	      delay_counter++;
    	 }
    	 if (delay_counter >= 60*400/G_food_rate) {
    		    // 在 1/3 和 2/3 之间更新
    		  if (G_one_ml_count >= ONE_ML / 3 && G_one_ml_count <= ONE_ML * 2 / 3) {
    			  delay_counter = 0;
    			  (*food_volume_increment)++;
    			  G_hour_volume++;
    			  G_volume++;
    			  running_volume++;
    			  if (running_volume > MAX_RUNNING_VOLUME) {
    				  running_volume = MAX_RUNNING_VOLUME;
    			  }
    			  G_total_volume++;
    			  G_acc_formula_ml++;
    			  control->compensation_ml--;

    			  if (control->compensation_ml == 0) {
    		        control->compensation_active = 0;
    		        delay_counter = 0;
    			  }
    		 }
    	}
    }
}


/**
 * @brief 空气泡安全计数
 * @param air_ml_count 空气体积计数指针
 */
static void increment_air_count(uint8_t *air_ml_count)
{
    if (*air_ml_count < 50) {
        (*air_ml_count)++;
    }
}

/**
 * @brief 营养液输送模式下的流量累加
 * @param food_volume_increment 当前批次营养液增量指针
 * @param control 动态间隔控制结构体指针
 */
static void increment_food_volumes(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
    (*food_volume_increment)++;
    G_hour_volume++;
    G_volume++;
    running_volume++;
    if (running_volume > MAX_RUNNING_VOLUME) {
        running_volume = MAX_RUNNING_VOLUME;
    }
    G_total_volume++;
    G_acc_formula_ml++;
}

/**
 * @brief 推注模式下的流量累加
 * @param bolus_run_volume 当前推注运行量指针
 */
static void increment_bolus_volumes(uint16_t *bolus_run_volume)
{
    G_acc_bolus_ml++;
    (*bolus_run_volume)++;
    G_water++;
}

/**
 * @brief 低速模式下小时剂量完成后的处理
 * @param control 动态间隔控制结构体指针
 */
static void handle_low_flow_hour_complete(dynamic_interval_control_t *control)
{
    pump_motor_off();
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - control->hour_start_time;
    if (elapsed_time >= HOUR_DURATION_MS) {
        elapsed_time = HOUR_DURATION_MS;
    }
    uint32_t new_wait_time = HOUR_DURATION_MS - elapsed_time;

#ifdef LOW_FLOW_TEST_MODE
    control->low_flow_wait.wait_duration = new_wait_time;
#else
    uint32_t initial_wait_time = control->low_flow_wait.wait_duration;
    uint32_t time_diff = (new_wait_time > initial_wait_time) ?
                        (new_wait_time - initial_wait_time) :
                        (initial_wait_time - new_wait_time);
    control->low_flow_wait.wait_duration = (time_diff > 60000) ?
                                          initial_wait_time : new_wait_time;
#endif
    control->low_flow_wait.wait_start_time = current_time;
    control->low_flow_wait.is_waiting = 1;
}

/**
 * @brief 低速模式下总剂量完成后的处理
 * @param control 动态间隔控制结构体指针
 */
static void handle_low_flow_dose_complete(dynamic_interval_control_t *control)
{
    pump_motor_off();
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - control->hour_start_time;
    if (elapsed_time >= HOUR_DURATION_MS) {
        elapsed_time = HOUR_DURATION_MS;
    }
    float actual_run_time_hours = (float)G_hour_volume / G_food_rate;
    uint32_t actual_run_time_ms = (uint32_t)(actual_run_time_hours * HOUR_DURATION_MS);
    uint32_t wait_time_ms = actual_run_time_ms - elapsed_time;

    control->low_flow_wait.is_waiting = 1;
    control->low_flow_wait.wait_start_time = current_time;
    control->low_flow_wait.wait_duration = wait_time_ms;
}

/**
 * @brief 无暂停情况下的小时结束补偿
 * @param food_volume_increment 当前批次营养液增量指针
 */
static void handle_no_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
    int expected_volume = G_food_rate;
    if (G_hour_volume < expected_volume) {
        // 先做一次 +1 补偿
/*        G_hour_volume++;
        (*food_volume_increment)++;
        G_volume++;
        running_volume++;
        if (running_volume > MAX_RUNNING_VOLUME) {
            running_volume = MAX_RUNNING_VOLUME;
        }
        G_total_volume++;
        G_acc_formula_ml++;*/

        // 如果还不够，激活补偿标志
        if (G_hour_volume < expected_volume) {
            uint16_t remaining_need = expected_volume - G_hour_volume;

            // 正常2分钟内能输送的流量
            uint32_t normal_2min_flow = (G_food_rate * 565 * 2) / (400 * 60);

            // 需要补偿的量（减去正常能输送的，再减1）
            int compensation_need = remaining_need - normal_2min_flow - 1;

            if (compensation_need > 0) {
                // 确定补偿多少 ml
                if (compensation_need <= normal_2min_flow) {
                    control->compensation_ml = compensation_need;

                } else {
                    control->compensation_ml = normal_2min_flow - 1;
                    if (G_food_rate > 200){
                    	control->compensation_occl_time = (control->compensation_ml -normal_2min_flow)*6;
                    }
                }
                control->compensation_active = 1;
            }
        }
    }
}

/**
 * @brief 无暂停情况下的小时结束补偿
 * @param food_volume_increment 当前批次营养液增量指针
 */
static void handle_no_pause_hour_first(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
    int expected_volume = G_food_rate/2;
    if (G_hour_volume < expected_volume) {
        // 先做一次 +1 补偿
        G_hour_volume++;
        (*food_volume_increment)++;
        G_volume++;
        running_volume++;
        if (running_volume > MAX_RUNNING_VOLUME) {
            running_volume = MAX_RUNNING_VOLUME;
        }
        if(running_volume > G_volume){
        	running_volume = G_volume;
        }
        G_total_volume++;
        G_acc_formula_ml++;

        // 如果还不够，激活补偿标志
        if (G_hour_volume < expected_volume) {
            uint16_t remaining_need = expected_volume - G_hour_volume;

            // 正常2分钟内能输送的流量
        //    uint32_t normal_2min_flow = (G_food_rate * 565 * 2) / (400 * 60);

            // 需要补偿的量（减去正常能输送的，再减1）
         //   int compensation_need = remaining_need - normal_2min_flow - 1;

            if (remaining_need > 0) {
                // 确定补偿多少 ml
                 if (G_food_rate > 200){
                	 control->compensation_active = 1;
                	 control->compensation_ml = remaining_need;
                 }

            }
        }
    }
}

/**
 * @brief 有暂停情况下的小时结束补偿
 * @param food_volume_increment 当前批次营养液增量指针
 * @param control 动态间隔控制结构体指针
 */
static void handle_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
	uint16_t expected_volume = G_food_rate;
    // 当前总流量 = 暂停前累计 + 本次运行新增
	uint16_t current_total_flow = control->pause_flow_total + (*food_volume_increment);
	if(current_total_flow !=G_hour_volume)
	{
		current_total_flow = G_hour_volume;
	}
    if (control->pause_flow_total < expected_volume) {
    	uint16_t supplement_volume = expected_volume - control->pause_flow_total;
        G_hour_volume += expected_volume;
        (*food_volume_increment) += supplement_volume;
        G_volume += supplement_volume;
        G_total_volume += supplement_volume;
        G_acc_formula_ml += supplement_volume;
    }
}

/**
 * @brief 有暂停后重启电机
 * @param pause_motor_off_flag 电机停止标志
 */
static void handle_pause_restart_motor(uint8_t pause_motor_off_flag)
{
    if (pause_motor_off_flag == 1) {
    	if(running_volume > G_volume){
    		running_volume = G_volume;
    	}
        start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);
    }
}

/**
 * @brief 有暂停情况下提前完成喂食目标的处理
 * @param control 动态间隔控制结构体指针
 * @param pause_motor_off_flag 电机停止标志指针
 */
static void handle_pause_early_complete(dynamic_interval_control_t *control, uint8_t *pause_motor_off_flag)
{
	if (G_hour_volume >= G_food_rate) {
        pump_motor_off();
        ir_heat_off();
        *pause_motor_off_flag = 1;
    }
}

/**
 * @brief 无暂停情况下的小时结束判断（带防抖）
 * @param food_volume_increment 当前批次营养液增量指针
 * @param control 动态间隔控制结构体指针
 */
static void check_no_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
/*    if (control->current_hour_tests == 4) {
        static uint16_t delay_first_counter = 0;
        if (delay_first_counter < 2) {
        	delay_first_counter++;
        }
        if (delay_first_counter >= 2) {
        	delay_first_counter = 0;
            uint32_t current_time = HAL_GetTick();
            uint32_t elapsed_time = current_time - control->hour_start_time;
            if (elapsed_time >= 1800000) {

            	handle_no_pause_hour_first(food_volume_increment,control);
            }
        }
    }*/
	if (control->current_hour_tests >= 7) {
        static uint16_t delay_counter = 0;
        if (delay_counter < 2) {
            delay_counter++;
        }
        if (delay_counter >= 2) {
            delay_counter = 0;
            uint32_t current_time = HAL_GetTick();
            uint32_t elapsed_time = current_time - control->hour_start_time;
            if (elapsed_time >= HOUR_TEST_TIME_THRESHOLD) {

                handle_no_pause_hour_end(food_volume_increment,control);
            }
        }
    }
}

/**
 * @brief 有暂停情况下的小时结束判断（带防抖）
 * @param food_volume_increment 当前批次营养液增量指针
 * @param control 动态间隔控制结构体指针
 */
static void check_pause_hour_end(uint16_t *food_volume_increment, dynamic_interval_control_t *control)
{
    static uint8_t pause_motor_off_flag = 0;
    uint32_t now = HAL_GetTick();
    uint32_t interval = now - control->last_enter_time;

    control->accumulated_work_time += interval;
    if(control->accumulated_work_time > PAUSE_ACCUMulated_TIME_THRESHOLD) {
         uint32_t elapsed_time = control->accumulated_work_time;

         if (elapsed_time >= HOUR_DURATION_MS) {
            	handle_pause_hour_end(food_volume_increment, control);
            	handle_pause_restart_motor(pause_motor_off_flag);
            	pause_motor_off_flag = 0;
           } else {
            	handle_pause_early_complete(control, &pause_motor_off_flag);
           }
     }

  }

/**************************************************************
 * increment_volumes()
 * @short		Increment delivered volumes
 * @param       uint8_t air bubble size in ml
 * @param 		uint16_t bolus_run_volume water volume pumped during current bolus run
 * @retval
 * @detail
 * Increment all volume counters after every one mL delivered.
 * Depending on run type increment accumulated bolus (bolus run) or
 * total and accumulated formula (food run).
 */

#if(0)
void increment_volumes(uint8_t bolus_run,uint8_t *air_ml_count, uint16_t *bolus_run_volume, uint16_t *food_volume_increment)
{
  if (G_one_ml_flag){                      //increment all volume counters for every  one mL delivered
    if ((*air_ml_count) < 50) (*air_ml_count)++; //used in air bubble detection
    G_one_ml_flag = 0;
    if (!bolus_run){
      (*food_volume_increment)++;
   	  G_hour_volume++;
      G_volume++;
      G_total_volume++;
      G_acc_formula_ml++;       	//accumulated formula volume in ml
    }
    else {
   	  G_acc_bolus_ml++;				//accumulated bolus volume in ml
   	  (*bolus_run_volume)++;        //this is required for capturing accurate daily volume
      							    //in case the bolus run is interrupted before completion
    }
  }
}
#endif
/** End increment_volumes() **/

/**
 * @brief 管路阻塞测试状态机
 * @param occl_timer 测试计时器指针
 * @param occl_data 压力数据数组
 * @param air_rate_inc 空气率增量指针
 * @return 测试状态码
 */
uint8_t occl_test(uint16_t *occl_timer, int16_t occl_data[OCCL_DATA_SIZE], uint16_t *air_rate_inc,dynamic_interval_control_t *interval_control,uint8_t air_result)
{
	static uint8_t error_count = 0;
	static uint16_t current_interval = OCCL_TEST_INTERVAL;
	const uint16_t NORMAL_INTERVAL = OCCL_TEST_INTERVAL;
	const uint16_t FAST_INTERVAL = OCCL_TEST_INTERVAL * 1 / 2;
	// 静态变量记录处理状态
	static uint8_t first_bolus_processed = 0;
	static uint8_t extend_compensation_processed = 0;

	uint16_t data_index;

	 // 函数开始时：如果检测到首次切换标志，立即清除并重新计算间隔
	if (g_first_after_bolus == 1&& !first_bolus_processed) {
	    g_first_after_bolus = 0; // 立即清除标志
	    first_bolus_processed = 1; // 标记已处理
	 // 重新计算间隔时间
	    if (error_count > 0) {
	        if (G_food_rate > 50) {
	         current_interval = FAST_INTERVAL;
	        } else {
	           current_interval = NORMAL_INTERVAL;
	          }
	     } else {
	       current_interval = NORMAL_INTERVAL;
	       }
	  }

	// 第一次堵塞检测补偿（只处理一次）
	if (interval_control && interval_control->need_extend_interval == 1 &&
	    interval_control->current_hour_tests == 0 && !extend_compensation_processed){
	    current_interval = OCCL_TEST_START_COUNT + interval_control->extended_interval;
	    interval_control->need_extend_interval = 0;
	    interval_control->extended_interval = 0;
	    extend_compensation_processed = 1; // 标记已处理
	}

    // 背景IR测试控制
    G_bg_ir_test_enabled = ((*occl_timer) < current_interval) &&
                          ((*occl_timer) > OCCL_TEST_HEAT_END);

    switch(g_occl_state) {
        case OCCL_STATE_IDLE:
            if (*occl_timer > current_interval) {
                start_test_phase(occl_timer);
                g_occl_state = OCCL_STATE_HEATING;
            }
            break;

        case OCCL_STATE_HEATING:
            if (*occl_timer == OCCL_TEST_HEAT_END) {
                end_heating_phase(air_rate_inc);
                g_occl_state = OCCL_STATE_COLLECTING;
            }
            break;

        case OCCL_STATE_COLLECTING:
            data_index = *occl_timer - OCCL_TEST_HEAT_END;
            if ((*occl_timer > OCCL_TEST_HEAT_END) && (*occl_timer < OCCL_TEST_END) ){
            	if (data_index < OCCL_DATA_SIZE) {
            		occl_data[data_index] = G_tp_value;
                    // 流速 > 100 且已完成 7 次检测且剩余时间不足 2 分钟，取消堵塞检测
                    if (G_food_rate > 32 && interval_control->current_hour_tests == 7) {
                        // 当 occl_timer == 90 时，检查是否需要提前结束堵塞检测
                        if (*occl_timer >= 60) {
                            uint32_t current_time = HAL_GetTick();
                            uint32_t remaining_time = HOUR_DURATION_MS - (current_time - interval_control->hour_start_time);

                            // 剩余时间不足 1分钟 且 有补偿需求
                            if (remaining_time < 60000 && interval_control->compensation_occl_time > 0) {
                                // 重置状态机
                                g_occl_state = OCCL_STATE_IDLE;
                                *occl_timer = OCCL_TEST_START_COUNT;
                                pump_motor_off();  // 确保泵停止
                                skip_occlusion = 1;
                                break;  // 跳出 for 循环
                            }
                        }
                    }
            		if (should_check_air(*occl_timer) && safe_distal_tube_empty()) {
            		     //   if (air_result == HOLD_MODE__TASK){
                        	handle_air_condition(occl_timer, air_rate_inc);
                        	return NORMAL_EXIT;  // 处理完空气后立即跳出，不执行后续状态判断
                    //    }

            		}
            	}
            }

            if (*occl_timer >= OCCL_TEST_END) {
            	g_occl_state = OCCL_STATE_PROCESSING;
            }
            else if(*occl_timer > OCCL_TEST_INTERVAL+ OCCL_TEST_END){
            	g_occl_state = OCCL_STATE_IDLE;
            	*occl_timer = OCCL_TEST_START_COUNT;
            }
            break;

        case OCCL_STATE_PROCESSING:
            // 检查标志，跳过状态转换
            if (skip_occlusion) {

            	skip_occlusion = 0;
                *air_rate_inc = (125 * G_food_rate) / 257;
            	if(running_volume > G_volume){
            		running_volume = G_volume;
            	}
                start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);
                // 直接返回，不进行后续状态处理
                return NORMAL_EXIT;
            }

        	uint8_t ret = process_test_results(occl_data, air_rate_inc);
            uint8_t return_value = process_occlusion_result(ret, &error_count, &current_interval,
                                                           NORMAL_INTERVAL, FAST_INTERVAL);

            // ==== 新增：检测完成后更新计数并动态调整间隔 ====
            if (interval_control) {
                interval_control->current_hour_tests++;

                // 第七次检测完毕后判断第8次检测间隔
                if (interval_control->current_hour_tests == 7&&current_interval>FAST_INTERVAL) {
                	if (G_food_rate >=50) {
                    	if (G_food_rate >200) {
                    	current_interval = current_interval - 120 +interval_control->compensation_occl_time; // 提前2分钟（120秒）
                    	}
                    	else{
                    		current_interval = current_interval - 120; // 提前2分钟（120秒）
                    	}
                	}
                	else
                	current_interval = current_interval - 60;
#ifndef DISABLE_8TH_TEST_CHECK
                    check_8th_test_interval(interval_control);

                    // 如果需要延长间隔，直接修改current_interval
                    if (interval_control->need_extend_interval) {
                        current_interval = interval_control->extended_interval;
                    }
#endif
                }
            }

            g_occl_state = OCCL_STATE_IDLE;
            return return_value;
            break;

        default:
        	g_occl_state = OCCL_STATE_IDLE;
		}
	return(NORMAL_EXIT);

}

/**
 * @brief 处理堵塞检测结果并更新错误计数
 * @param ret 检测结果
 * @param error_count 错误计数指针
 * @param current_interval 当前间隔指针
 * @param normal_interval 正常间隔值
 * @param fast_interval 快速间隔值
 * @return uint8_t 返回状态码
 */
static uint8_t process_occlusion_result(uint8_t ret, uint8_t *error_count, uint16_t *current_interval,
                                       uint16_t normal_interval, uint16_t fast_interval)
{
    uint8_t return_value = NORMAL_EXIT;

    // 原有的错误处理逻辑
    if (ret == OCCL_ERROR_SEVERE) {
        *error_count = 0;
        return_value = OCCL_ERROR;
    }
    else if (ret == OCCL_ERROR_MILD) {
        if (G_food_rate < 50){
            return_value = OCCL_ERROR;
        } else {
            (*error_count)++;
            if (*error_count == 1) {
                return_value = OCCL_ERROR_MILD;
            }
            else if (*error_count >= 2) {
                *error_count = 0;
                return_value = OCCL_ERROR;
            }
        }
    }
    else {
        *error_count = 0;
        return_value = NORMAL_EXIT;
    }

    // 使用传递过来的参数
    if (*error_count > 0) {
        if (G_food_rate > 50) {
            *current_interval = fast_interval;
        } else {
            *current_interval = normal_interval;
        }
    } else {
        *current_interval = normal_interval;
    }

    return return_value;
}
// 辅助函数示例
static void start_test_phase(uint16_t *timer) {
    pump_motor_off();
    ir_heat_on();
    *timer = 0;
}
/**
 * @brief 结束加热阶段操作
 * @param air_rate_inc 空气率增量指针
 */
static void end_heating_phase(uint16_t* air_rate_inc)
{
    ir_heat_off();
    G_motor_rate = OCCL_TEST_RATE;
    start_pump(G_motor_rate, MOTOR_INT_OCCL_TEST);
    *air_rate_inc = 6;  // 初始空气率增量

    // 记录调试信息（可选）
    #ifdef DEBUG_MODE
    log_event("HEATING_PHASE_END");
    #endif
}
/**
 * @brief 判断是否需要检查空气
 * @param occl_timer 当前测试计时器值
 * @return true-需要检查, false-跳过检查
 */
static bool should_check_air(uint16_t occl_timer)
{
    // 加热结束后至少5个周期才开始检测
    const uint16_t AIR_CHECK_DELAY = 5;
    return (occl_timer >= (OCCL_TEST_HEAT_END + AIR_CHECK_DELAY));
}

/**
 * @brief 处理检测到空气的条件
 * @param occl_timer 测试计时器指针
 * @param air_rate_inc 空气率增量指针
 */
static void handle_air_condition(uint16_t* occl_timer, uint16_t* air_rate_inc)
{
    *occl_timer = OCCL_TEST_START_COUNT;  // 重置测试
    g_occl_state = OCCL_STATE_IDLE;       // 强制状态为IDLE

    // 计算恢复供液速率
#if ENABLE_LOW_FLOW_MODE
    // 低速模式：固定为1（根据 init_run_mode_state 中的逻辑）
    // 正常模式：G_food_rate - 4，最小为0
    G_motor_rate = G_low_flow_mode ? 1 : ((G_food_rate > 4) ? (G_food_rate - 4) : 0);
#else
    // 未启用低速模式
    G_motor_rate = (G_food_rate > 4) ? (G_food_rate - 4) : 0;
#endif

    // 计算空气率增量（保留原始公式，建议后续优化为查表法）
    *air_rate_inc = (125 * G_food_rate) / 257;
	if(running_volume > G_volume){
		running_volume = G_volume;
	}
    start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);

    // 触发警报（可选）
    #ifdef ENABLE_ALARMS
    trigger_alarm(ALARM_AIR_DETECTED);
    #endif
}

/**
 * @brief 处理测试结果并返回状态码
 * @param occl_data 已采集的压力数据数组
 * @param air_rate_inc 空气率增量指针
 * @return 测试结果状态码
 */
static uint8_t process_test_results(int16_t occl_data[], uint16_t* air_rate_inc)
{
    pump_motor_off();  // 确保泵停止
    // 数据标准化处理
    memset((void*)occl_comp_count_test_value, 0, sizeof(occl_comp_count_test_value));
    memset((void*)occl_comp_count_test_position, 0, sizeof(occl_comp_count_test_position));
    occl_normalize_max(occl_data);
    uint16_t occl_peak_count = calc_occl_peaks(occl_data);
    uint16_t occl_comp_count = occl_normalize_tail_and_compare_limits(occl_data);

    // 调试输出（可选）
    #ifdef DEBUG_MODE
    char report[64];
    snprintf(report, sizeof(report),
            "OCCL_RESULTS: peaks=%u, anomalies=%u",
            occl_peak_count, occl_comp_count);
    uart_send(report);
    #endif

    // 恢复供液参数
#if ENABLE_LOW_FLOW_MODE
    // 低速模式：固定为1（根据 init_run_mode_state 中的逻辑）
    // 正常模式：G_food_rate - 4，最小为0
    G_motor_rate = G_low_flow_mode ? 1 : ((G_food_rate > 4) ? (G_food_rate - 4) : 0);
#else
    // 未启用低速模式
    G_motor_rate = (G_food_rate > 4) ? (G_food_rate - 4) : 0;
#endif
    *air_rate_inc = (125 * G_food_rate) / 257;
	if(running_volume > G_volume){
		running_volume = G_volume;
	}
    start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);

    // G_food_rate = 1 时：最低流速，直接报警，不分等级
    if (G_food_rate == 1) {
        // 最低流速下，任何堵塞迹象都直接报警
        if ((occl_comp_count > OCCL_COMP_LIMIT) || (occl_peak_count > OCCL_PEAK_LIMIT)) {
            return OCCL_ERROR_SEVERE;  // 直接严重报警

        }
        // 否则继续运行
        return NORMAL_EXIT;
    }
    uint8_t peak_limit = g_first_after_bolus ? (OCCL_PEAK_LIMIT + 1) : OCCL_PEAK_LIMIT;
    uint8_t	comp_limit = g_first_after_bolus ? (OCCL_COMP_LIMIT + 2) : OCCL_COMP_LIMIT;
    // 分级结果判定
	// 严重情况：直接报警
	if ((occl_comp_count > comp_limit + 2) && (occl_peak_count >= peak_limit + 2)) {
		occl_comp_count_test = occl_comp_count;
		occl_peak_count_test = occl_peak_count;
       	return OCCL_ERROR_SEVERE;  // 严重错误，直接报警
	}
	// 轻微情况：需要二次检测
	else if ((occl_comp_count > comp_limit) && (occl_peak_count > peak_limit)) {
		occl_comp_count_test = occl_comp_count;
		occl_peak_count_test = occl_peak_count;
	return OCCL_ERROR_MILD;    // 轻微错误，需要二次确认
	}
	else {
		return NORMAL_EXIT;
	}
}


/**************************************************************
 * median_filter()
 * @detail
 * 中值滤波
 */

void median_filter(int16_t *data, uint16_t len) {
    int16_t buffer[3];  // 3点中值滤波
    for (uint16_t i = 2; i < len ; i++) {
        buffer[0] = data[i - 1];
        buffer[1] = data[i];
        buffer[2] = data[i + 1];

        // 排序取中值
        if (buffer[0] > buffer[1]) swap(&buffer[0], &buffer[1]);
        if (buffer[1] > buffer[2]) swap(&buffer[1], &buffer[2]);
        if (buffer[0] > buffer[1]) swap(&buffer[0], &buffer[1]);

        data[i] = buffer[1];
    }
}

/**************************************************************
 * low_pass_filter()
 * @detail
 * 一阶滤波
 */
static void low_pass_filter(int16_t *data, uint16_t len) {
    float prev_out = 0;
    int16_t current_max = NORMALIZATION_MAX; // 保持归一化最大值
    for (uint16_t i = 1; i < len; i++) {
    	float filtered = ALPHA * data[i] + (1 - ALPHA) * prev_out;

        // 峰值保持逻辑
        if (data[i] == current_max) {
            filtered = current_max; // 强制保持最大值
        }
        data[i] = (int16_t)filtered;
        prev_out = filtered;
    }
}
/**************************************************************
 * occl_test()
 * @detail
 * Test occl_timer and run required part of the occl test (Heat on, heat off, collect data and
 * process data and return the result.
 */

#if(0)
uint8_t occl_test(uint16_t *occl_timer, int16_t occl_data[120], uint16_t *air_rate_inc){
uint16_t occl_counter;
uint16_t occl_comp_count, occl_peak_count;

//uint8_t str[16];  //^^***TESTING used for sendind test data to uart
//int size_len;     //^^***TESTING used for sendind test data to uart

  //******************disable background ir test during the IR heat
  if (((*occl_timer) < OCCL_TEST_INTERVAL) && ((*occl_timer) > OCCL_TEST_HEAT_END)) {
	G_bg_ir_test_enabled = 1;
  }
  else{
	G_bg_ir_test_enabled = 0;
  }


  if ((*occl_timer)>OCCL_TEST_INTERVAL) {      		//OCCL test starts
    pump_motor_off();						   		//PUMP OFF
    ir_heat_on();							   		//HEAT ON
    (*occl_timer)=0;
  }



  if ((*occl_timer)==OCCL_TEST_HEAT_END) {        	//OCCL test heating stop
    ir_heat_off();                             		//HEAT OFF
    G_motor_rate = OCCL_TEST_RATE;			   		//Start pumping w/ OCCL test rate
    start_pump(G_motor_rate,MOTOR_INT_OCCL_TEST);
    (*air_rate_inc) = 6;
  }

  if (((*occl_timer) > OCCL_TEST_HEAT_END)&&( (*occl_timer)<OCCL_TEST_END)){ //OCCL data collection
    occl_counter= (*occl_timer)-OCCL_TEST_HEAT_END;             			//collect PD2 data
    occl_data[occl_counter] = G_tp_value;
    if ((*occl_timer) >= (OCCL_TEST_HEAT_END+5)){
	  if (distal_tube_empty()){                	 //Distal tube empty (AIR)
		(*occl_timer) = OCCL_TEST_START_COUNT;                     //Cancel OCCL test
		G_motor_rate = G_food_rate-4;            //continue pumping with food rate
		(*air_rate_inc) = (125 * (G_food_rate))/257;   //??or should we stop w/empty/air alarm
		start_pump(G_motor_rate,MOTOR_INT_FORMULA);
	  }
    }
  }

  if ((*occl_timer)==OCCL_TEST_END) {             	//OCCL data collection ends
    pump_motor_off();						   		//PUMP OFF

    occl_normalize_max(occl_data);
    //occl_comp_count = occl_normalize(occl_data);   	//Process test data
    occl_peak_count = calc_occl_peaks(occl_data);   //
    occl_comp_count = occl_normalize_tail_and_compare_limits(occl_data);

    //size_len = sprintf (str,"%s%d%s%d\r\n", "OCCL*,",occl_comp_count,",",occl_peak_count);//^^prt used for sendind test data to uart
    //HAL_UART_Transmit (&huart2, (uint8_t *)str, size_len, HAL_MAX_DELAY); //^^prt used for sendind test data to uart
    //Test__ used for sendind test data to uart

    //size_len = sprintf (str,"%s%d\r\n", "F*,",G_food_rate);               //Test__ used for sendind test data to uart

    //HAL_UART_Transmit (&huart2, (uint8_t *)str, size_len, HAL_MAX_DELAY); //Test__ used for sendind test data to uart

    if ((occl_comp_count > OCCL_COMP_LIMIT) || (occl_peak_count > OCCL_PEAK_LIMIT)){   //OCCL Test fails
      return (OCCL_ERROR);
    }   //test__

    G_motor_rate = G_food_rate-4;                 //OCCL test ok -> continue pumping with food rate
    (*air_rate_inc) = (125 * (G_food_rate))/257;
    start_pump(G_motor_rate,MOTOR_INT_FORMULA);

  }
  return(NORMAL_EXIT);
}
#endif
/**  End occl_test()  **/

/**************************************************************
 * test_background_ir()
 * @detailo
 * Test if the background IR is too strong. Halt operation, save and display the error
 * then shutoff.
 */
void test_background_ir(uint16_t pd2_dark)
{
	if (G_bg_ir_test_enabled){
		if (pd2_dark > PD2_EXCESS_BG_IR_MAX){
			HAL_TIM_Base_Stop_IT(&htim22);
			gen_err_handler(ERR_EXCESS_BG_IR);
		}
	}
}
/**  End test_background_ir()  **/


/**
 * @file 变量调节控制模块
 * @brief 基于50ms周期的多级调速控制器
 * @note 所有函数均保持原始数值比较逻辑
 */



/**************************************************************
 * @brief 基础变量递减控制
 * @param count 按键持续时间计数（单位：50ms）
 * @param value 待调节变量指针
 * @note 调节规则：
 *       1. 首次按下立即减1
 *       2. 持续1秒内不响应（防误触）
 *       3. 1-3.5秒：每100ms减1
 *       4. 3.5秒后：每50ms减1
 *       5. 值范围强制限制在0-150
 替换所有位运算符|为逻辑或||的 原因
 *       根本区别：位运算 vs 逻辑运算
| 特性 | 位或 | | 逻辑或 || |
|--------------------|----------------------------------|-----------------------------------|
| 运算层级 | 按位操作（二进制位级） | 布尔逻辑操作 |
| 求值方式 | 总是计算两边操作数 | 短路求值（左为真则跳过右计算） |
| 返回值 | 整数（位组合结果） | 布尔值（0或1） |
| 典型用途 | 硬件寄存器操作、掩码组合 | 条件判断、流程控制 |

替换的核心原因：语义正确性，在条件判断中（如if），开发者意图通常是逻辑判断而非位操作
示例风险代码：
c
if (count % 2 == 0 | count > 70)  // 位运算可能产生非预期结果
当count=3时：3 % 2 == 0 → 0，3 > 70 → 0，0 | 0 → 0（条件不成立）；
但若误写为 count = 0x80 | 0x01：0x81 % 2 == 0 → 0，0x81 > 70 → 1，0 | 1 → 1（条件成立），而逻辑或 || 会稳定返回 0 或 1
短路求值安全性：|| 的短路特性可避免不必要的计算和副作用：
c
if (ptr != NULL || ptr->value > threshold)  // 安全：左为真时不解引用ptr| 会强制计算两边，可能导致空指针解引用；
代码可读性
使用 || 明确表达逻辑意图，符合K&R编码规范
静态分析工具（如Lint）会标记位运算在条件中的可疑使用
性能影响
| 场景 | 位或 | | 逻辑或 || |
|---------------------|---------------------|---------------------|
| 左操作数为真 | 计算两边 | 跳过右计算 |
| 需要位组合结果 | 必需 | 不可用 |
| 条件判断 | 潜在风险 | 推荐方式 |
 **************************************************************/
void variable_decr(uint8_t count, uint16_t *value)
{
    /* 阶段判断：满足操作条件时执行 */
    if ((count == 0) || (count > INITIAL_DELAY &&
                       (count > SLOW_PHASE_END || count % 2 == 0)))
    {
        /* 安全递减（确保无符号数不下溢）*/
        if (*value > 0) (*value)--;

        /* 值域限制（保持原始比较逻辑）*/
        *value = (*value < 10) ? 0 : (*value > 150) ? 150 : *value;

        /* 用户反馈：数值变化时触发提示音（最小值10除外）*/
        if (*value != 10) key_click();
    }
}

/**************************************************************
 * @brief 基础变量递增控制
 * @param 参数同variable_decr
 * @note 调节规则与递减对称，值范围限制在10-150
 **************************************************************/
void variable_incr(uint8_t count, uint16_t *value)
{
    if ((count == 0) || (count > INITIAL_DELAY &&
                       (count > SLOW_PHASE_END || count % 2 == 0)))
    {
        (*value)++;
        *value = (*value < 10) ? 10 : (*value > 150) ? 150 : *value;
        if (*value != 150) key_click(); // 未达最大值时反馈
    }
}

/**************************************************************
 * @brief 速率递增控制（带固定步长）
 * @param value 指向速率值的指针
 * @note 特殊限制：
 *       - 最小值固定为5
 *       - 最大值使用FULL_FOOD_RATE宏定义
 **************************************************************/
void variable_incr_rate(uint8_t count, uint16_t *value)
{
    if ((count == 0) || (count > INITIAL_DELAY &&
                       (count > SLOW_PHASE_END || count % 2 == 0)))
    {
        (*value)++;

#ifdef ENABLE_LOW_FLOW_MODE
        // 支持低速模式：最小值可以到1
        *value = (*value < 1) ? 1 : (*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value;

        // 反馈逻辑：除了到达最大值400时不反馈
        if (*value != FULL_FOOD_RATE) {
            key_click();
        }
#else
        // 不支持低速模式：原有逻辑
        *value = (*value < 5) ? 5 : (*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value;

        // 反馈逻辑：除了到达最大值400时不反馈
        if (*value != FULL_FOOD_RATE) {
            key_click();
        }
#endif
    }
}


/**************************************************************
 * variable_decr_rate()
 * @brief 速率递减控制（三阶段调速）
 * @param count 按键计数（基于50ms周期）
 * @param value 待调节的速率值指针
 * 调节规则：
 * - 首次按下立即减1
 * - 持续1秒内不响应（防误触）
 * - 1-3.5秒：每100ms减1（偶数次）
 * - 3.5秒后：每50ms减1
 * - 值范围限制：5 ~ FULL_FOOD_RATE
 **************************************************************/
void variable_decr_rate(uint8_t count, uint16_t *value)
{
    if ((count == 0) || (count > INITIAL_DELAY &&
                       (count > SLOW_PHASE_END || count % 2 == 0)))
    {
        (*value)--;

#ifdef ENABLE_LOW_FLOW_MODE
        // 支持低速模式：最小值可以到1
        *value = (*value < 1) ? 1 : (*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value;

        // 反馈逻辑：除了到达最小值1时不反馈
        if (*value != 1) {
            key_click();
        }
#else
        // 不支持低速模式：原有逻辑，最小值是5
        *value = (*value < 5) ? 5 : (*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value;

        // 反馈逻辑：除了到达最小值5时不反馈
        if (*value != 5) {
            key_click();
        }
#endif
    }
}
/**************************************************************
 * @brief 剂量智能递增控制（动态步长）
 * @note 步长变化规则：
 *       1. 0-5秒：+1/步
 *       2. 5-8秒：+10/步
 *       3. 8秒后：+50/步（根据当前值自动降级）
 *       - 值<100时强制步长=1
 *       - 值<1000时最大步长=10
 **************************************************************/
void variable_incr_dose(uint8_t count, uint16_t *value)
{
    uint8_t step = 1; // 默认步长

    /* 动态步长计算（保持原始加速逻辑）*/
    if (count > ACCEL_POINT_5S) step = 10;
    if (count > ACCEL_POINT_8S) step = 50;


    /* 执行增量操作 */
    if ((count % 2 == 0) || (count > SLOW_PHASE_END)) {
    	if ((count < 1) || (count > INITIAL_DELAY)) {
    		*value += step;
            /* 归零逻辑实现 */
#ifdef ENABLE_LOW_FLOW_MODE
    // 低速模式：根据FOOD_RATE决定阈值
    *value = (G_food_rate < 5) ?
             ((*value < 1) ? 1 : (*value > 9999) ? 9999 : *value) :
             ((*value < 10) ? 10 : (*value > 9999) ? 9999 : *value);
#else
    // 非低速模式：值<10归零，>9999设为9999
    *value = (*value < 10) ? 0 : (*value > 9999) ? 9999 : *value;
#endif
            // 反馈逻辑：除了归零时不反馈
            if (*value != 9999) {
                key_click();
            }
        }


    }
}

/**************************************************************
 * variable_incr_dose()
 * @short		Variable speed increment for dose limit
 * @param       uint8_t count how many loop iterations increment button has been pressed
 * 				uint16_t* incremented variable pointer
 * @retval      void
 * @detail
 * Three step increment speed for water rate. Times are based on 50ms loop time in calling routine.
 * At count 0 Increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * Then count on every count (every 50ms)
 */

/**
 * @brief 判断是否应该触发触摸计数事件
 * @param count 50ms为单位的触摸计数
 * @return 1-触发事件, 0-不触发
 *
 * 逻辑：前1秒不响应，1-5秒每100ms响应，5秒后每50ms响应
 */
uint8_t touch_count_event(uint16_t count)
{
    // 直接模仿原函数的条件判断
    if ((count % 2 == 0) || (count > 100)) {        // 100次 = 5秒
        if ((count < 1) || (count > 20)) {          // 20次 = 1秒
            return 1;  // 发生计数事件
        }
    }
    return 0;  // 无计数事件
}

#if(0)
void variable_incr_dose(uint8_t count,uint16_t *value){
uint8_t increment;
											//Select increment depending of time the key has been pressed
											//times based on 50ms loop time in calling routine
  increment = 1;							//initial increment is 1
  if (count>100) increment = 10;            //after 5 seconds +10
  if (count>160) increment = 50;			//after 8 seconds +50

  if ((count % 2 == 0) | (count>70)){		//if even count or count > 70
	  if ((count < 1)||(count>20)){       	//after first count pause for 20 seconds
		  *value= *value+increment;
		  if ((*value) < 10)  *value = 10;  	//min limit
		  if ((*value) > 9999) *value = 9999;	//max limit
		  if ((*value) != 9999) key_click();
	  }
  }
}
#endif/** End variable_incr_dose() **/

/**************************************************************
 * @brief 剂量智能递减控制（动态步长+归零逻辑）
 * @note 特殊处理：
 *       - 当值<1时直接归零
 *       - 超限值（>9999）也归零
 **************************************************************/
void variable_decr_dose(uint8_t count, uint16_t *value)
{
    uint8_t step = 1;

    /* 与递增相同的步长逻辑 */
    if (count > ACCEL_POINT_5S) step = 10;
	if (count > ACCEL_POINT_8S) step = 50;


    if ((count == 0) || (count > INITIAL_DELAY &&
                       (count > SLOW_PHASE_END || count % 2 == 0)))
    {
        *value -= step;
        /* 归零逻辑实现 */
#ifdef ENABLE_LOW_FLOW_MODE
    // 低速模式：根据FOOD_RATE决定阈值
    if (G_food_rate < 5) {
        // FOOD_RATE < 5时，value可以设置到1
        *value = (*value < 1 || *value > 9999) ? 0 : *value;
    } else {
        // FOOD_RATE >= 5时，value需要>=10
        *value = (*value < 10 || *value > 9999) ? 0 : *value;
    }
#else
    // 非低速模式：值<10或>9999时归零
    *value = (*value < 10 || *value > 9999) ? 0 : *value;
#endif
        // 反馈逻辑：除了归零时不反馈
        if (*value != 0) {
            key_click();
        }
    }
}

/**************************************************************
 * variable_decr_dose()
 * Three step decrement speed for water rate. Times are based on 50ms loop time in calling routine.
 * At count 0 Increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * Then count on every count (every 50ms)
 */
#if(0)
void variable_decr_dose(uint8_t count,uint16_t *value){
uint8_t increment;
											//Select decrement depending of time the key has been pressed
											//times based on 50ms loop time in calling routine
  increment = 1;							//decrement dose limit
  if (count>100) increment = 10;
  if (count>160){
	increment = 50;
	if ((*value)<1000) increment = 10;
	if ((*value)<100) increment = 1;
  }

  if ((count % 2 == 0) | (count>70)){
	if ((count < 1)||(count>20)){      		//if down arrow pressed, decrement once and then wait
	  *value= *value-increment;				//one second before starting continuous decrement
	  if ((*value) < 10)  *value = 0;
	  if ((*value) > 9999)  *value = 0;
	  if ((*value) != 0) key_click();
	}
  }
}/** End variable_decr_dose() **/
/**
 * @brief 变速递减函数
 * @param count 递减计数器（基于50ms的时间单位）
 * @param value 待递减的数值指针
 * @detail 实现三阶段变速递减逻辑：
 * 1. 初始阶段（count=0）：延迟1秒后开始递减
 * 2. 中间阶段（count=22-70）：每100ms递减一次（偶数计数时递减）
 * 3. 最终阶段（count>70）：每50ms递减一次
 * 同时处理数值的上下限限制（0-150）
 */


void variable_decr(uint8_t count, uint16_t *value)
{
    /* 条件说明：
     * - count % 2 == 0：偶数计数周期（50ms*2=100ms间隔）
     * - count > 70：进入快速递减阶段
     */
    if ((count % 2 == 0) || (count > 70)) {
        /* 排除初始延迟阶段（count=0-20）*/
        if ((count < 1) || (count > 20)) {
            /* 安全递减（防下溢）*/
            if (*value > 0) {
                (*value)--;
            }
            //if (*value < 10)  *value = 10;	//min limit
            if (*value < 10)  *value = 0;		//***TESTING
            if (*value > 150)  *value = 150;	//max limit

            /* 数值变化反馈（调试用）*/
            if (*value != 10) {
                key_click();
            }
        }
    }
}

#endif
/**************************************************************
 * variable_decr()
 * @detail      Variable speed value decrement
 *              Three step decrement speed for water rate.
 * 				Times are based on 50ms loop time in calling routine.
 * 				At count 0 decrement one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * 				At counts 22 to 70 decrement on even (24) counts (every 100ms (2x50ms)
 * 				Then decrement on every count (every 50ms)
 * 				Set min max limits
 ***************************************************************/
#if(0)
void variable_decr(uint8_t count,uint16_t *value)
{
	if ((count % 2 == 0) | (count>70)){       //if even count or count > 70
		if ((count < 1)|(count>20)){       	//after first count pause for 20 seconds
			if (*value > 0) (*value)--;
			//if (*value < 10)  *value = 10;	//min limit
			if (*value < 10)  *value = 0;		//***TESTING
			if (*value > 150)  *value = 150;	//max limit
			if (*value != 10) key_click();
		}
	}
}/** End variable_decr() **/
#endif
/**************************************************************
 * variable_incr()
 * @detail      变速增量函数
 *              用于水流量值的三阶段变速增量
 *              基于调用例程中50ms的循环时间
 *              阶段1：计数0时增量1 -> 等待1秒（20个计数，20x50ms=1s）
 *              阶段2：计数22到70时在偶数计数时增量（每100ms增量一次，2x50ms）
 *              阶段3：计数超过70后每个计数都增量（每50ms增量一次）
 *              同时设置最小最大值限制
 * @param count 当前计数（基于50ms的循环）
 * @param value 需要增量的值指针
 */

#if(0)
void variable_incr(uint8_t count, uint16_t *value)
{
    // 在偶数计数或计数>70时执行增量
    if ((count % 2 == 0) || (count > 70)) {
        // 跳过前20个计数（1秒延迟）后执行增量
        if ((count < 1) || (count > 20)) {
            (*value)++;

            // 应用最小/最大值限制
            *value = (*value < 10) ? 10 : (*value > 150) ? 150 : *value;

            // 当值未达到最大值时触发按键音
            if (*value != 150) {
                key_click();
            }
        }
    }
}
#endif
/** End variable_incr() **/

/**************************************************************
 * variable_incr()
 * @detail      Variable speed value increment
 * 				Three step increment speed for water rate.
 * 				Times are based on 50ms loop time in calling routine.
 * 				At count 0 Increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * 				At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * 				Then count on every count (every 50ms)
 * 				Set min max limits
 */
#if(0)
void variable_incr(uint8_t count,uint16_t *value)
{
	if ((count % 2 == 0) | (count>70)){       //if even count or count > 70
		if ((count < 1)|(count>20)){       	//after first count pause for 20 seconds
			(*value)++;
			if (*value < 10)  *value = 10;    //min limit
			if (*value > 150)  *value = 150;  //max limit
			if (*value != 150) key_click();
		}
	}
}
#endif
/** End variable_incr() **/

/**************************************************************
 * variable_incr_rate()
 * @detail      食物投放速率变速调节函数
 *              采用三阶段变速增量逻辑，基于50ms循环计时
 *              阶段1：计数0时增量1 -> 等待1秒（20个计数周期）
 *              阶段2：计数22-70时，在偶数计数时增量（每100ms一次）
 *              阶段3：计数>70后，每个计数周期都增量（每50ms一次）
 *              同时设置最小/最大速率限制
 * @param count 当前计数（基于50ms的循环周期）
 * @param value 待调节的速率值指针
 */
#if(0)
void variable_incr_rate(uint8_t count, uint16_t *value)
{
    // 满足增量条件：偶数计数或计数>70
    if ((count % 2 == 0) || (count > 70)) {
        // 跳过初始20个计数（1秒缓冲期）
        if ((count < 1) || (count > 20)) {
            // 安全增量：仅当当前值>0时增加
            if (*value > 0) {
                (*value)++;
            }

            // 应用速率限制
            *value = (*value < 5) ? 5 :
                   (*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value;

            // 未达最大速率时提供触觉反馈
            if (*value != FULL_FOOD_RATE) {
                key_click();
            }
        }
    }
}
#endif
/** End variable_incr_rate() **/

/**************************************************************
 * variable_incr_rate()
 * @detail
 * Three step increment speed for food rate. Times are based on 50ms loop time in calling routine.
 * At count 0 increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * Then increment on every count (every 50ms)
 */
#if(0)
void variable_incr_rate(uint8_t count,uint16_t *value){
	if ((count % 2 == 0) | (count>70)){       //if even count or count > 70
		if ((count < 1)|(count>20)){       		//after first count pause for 20 seconds
			if (*value > 0) (*value)++;
			if (*value < 5)  *value = 5;    		//min limit
			if (*value > FULL_FOOD_RATE)  *value = FULL_FOOD_RATE;  	//max limit
			if (*value != FULL_FOOD_RATE) key_click();
		}
	}
}
#endif
/** End variable_incr_rate() **/

/**************************************************************
 * variable_decr_rate()
 * 食物速率的3级递减速度控制函数。基于50ms的循环周期。
 * 递减规则：
 * 1. 第0次计数：立即递减1次
 * 2. 第1-20次计数：暂停递减（等待1秒）
 * 3. 第21-70次计数：每100ms递减1次（偶数计数时递减）
 * 4. 70次计数后：每50ms递减1次
 *
 * @param count 当前计数（0-255，50ms/次）
 * @param value 待调整的速率值指针
 **************************************************************/
#if(0)
void variable_decr_rate(uint8_t count, uint16_t *value)
{
    /* 决定是否执行递减 */
    bool should_decrement = false;

    if (count == 0) {
        // 第0次计数立即递减
        should_decrement = true;
    } else if (count > 20 && count <= 70) {
        // 21-70次计数时，偶数次递减
        should_decrement = (count % 2 == 0);
    } else if (count > 70) {
        // 70次计数后每次都递减
        should_decrement = true;
    }

    /* 执行递减操作 */
    if (should_decrement) {
        (*value)--;

        // 应用上下限约束
        *value = (*value < 5) ? 5 :
                ((*value > FULL_FOOD_RATE) ? FULL_FOOD_RATE : *value);

        // 非最小值时触发按键音
        if (*value != 5) {
            key_click();
        }
    }
}
#endif

/**************************************************************
 * variable_decr_rate()
 * Three step decrement speed for food rate. Times are based on 50ms loop time in calling routine.
 * At count 0 decrement one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 decrement on even (24) counts (every 100ms (2x50ms)
 * Then decrement on every count (every 50ms)
 */

#if(0)
void variable_decr_rate(uint8_t count,uint16_t *value)
{
	if ((count % 2 == 0) | (count>70)){       //if even count or count > 70
		if ((count < 1)|(count>20)){       		//after first count pause for 20 seconds
			(*value)--;
			if (*value < 5)  *value = 5;    		//min limit
			if (*value > FULL_FOOD_RATE)  *value = FULL_FOOD_RATE;  	//max limit
			if (*value != 5) key_click();
		}
	}
}
#endif
/** End variable_decr_rate() **/

/**************************************************************
 * variable_incr_dose()
 * @short		Variable speed increment for dose limit
 * @param       uint8_t count how many loop iterations increment button has been pressed
 * 				uint16_t* incremented variable pointer
 * @retval      void
 * @detail
 * Three step increment speed for water rate. Times are based on 50ms loop time in calling routine.
 * At count 0 Increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * Then count on every count (every 50ms)
 */

#if(0)
void variable_incr_dose(uint8_t count,uint16_t *value){
	uint8_t increment;
											//Select increment depending of time the key has been pressed
											//times based on 50ms loop time in calling routine
	increment = 1;							//initial increment is 1
	if (count>100) increment = 10;            //after 5 seconds +10
	if (count>160) increment = 50;			//after 8 seconds +50

	if ((count % 2 == 0) | (count>70)){		//if even count or count > 70
		if ((count < 1)||(count>20)){       	//after first count pause for 20 seconds
			*value= *value+increment;
			if ((*value) < 10)  *value = 10;  	//min limit
			if ((*value) > 9999) *value = 9999;	//max limit
			if ((*value) != 9999) key_click();
		}
	}
}
#endif
/** End variable_incr_dose() **/

/**************************************************************
 * variable_decr_dose()
 * Three step decrement speed for water rate. Times are based on 50ms loop time in calling routine.
 * At count 0 Increment one -> wait 1 second (20 counts, 20 x 50ms=1s)
 * At counts 22 to 70 increment on even (24) counts (every 100ms (2x50ms)
 * Then count on every count (every 50ms)
 */

#if(0)
void variable_decr_dose(uint8_t count,uint16_t *value){
	uint8_t increment;
											//Select decrement depending of time the key has been pressed
											//times based on 50ms loop time in calling routine
	increment = 1;							//decrement dose limit
	if (count>100) increment = 10;
	if (count>160){
		increment = 50;
		if ((*value)<1000) increment = 10;
		if ((*value)<100) increment = 1;
	}

	if ((count % 2 == 0) | (count>70)){
		if ((count < 1)||(count>20)){      		//if down arrow pressed, decrement once and then wait
			*value= *value-increment;				//one second before starting continuous decrement
			if ((*value) < 10)  *value = 0;
			if ((*value) > 9999)  *value = 0;
			if ((*value) != 0) key_click();
		}
	}
}
#endif
/** End variable_decr_dose() **/

/**************************************************************
 * reset_air_test_var()
 * @detail
 * Reset variables used for bubble and empty testing
 */
void reset_air_test_var(air_test_struct *air_test)
{

	air_test->air_ml_count = 0;
	air_test->air_cnt1 = 0;
	air_test->air_cnt2 = 0;
	air_test->air_cnt3 = 0;

}/** End reset_air_test_var() **/

/**************************************************************
 * inactivity_timeout()
 * @detail
 * Test if inactivity time has expired and display an idle timeout alarm.
 */
void inactivity_timeout(uint16_t *idle_count)
{
	if ((*idle_count) > INACTIVITY_TIMEOUT){
		show_timeout_message("IDLE TIMEOUT","");
		*idle_count=0;
	}
}/** End inactivity_timeout() **/
#if(0)
/**************************************************************
 * test_tir_cal_values()
 * @detail
 * Test if TIR calibration values are within limits. Return result CAL_OK or CAL_ERROR.
 * Save result to error log (EEPROM).
 */
uint8_t test_tir_cal_values(int16_t pd2, int16_t dac)
{

	if ((pd2 >= PD2_IR_CAL_MIN) && (pd2 <= PD2_IR_CAL_MAX)) {
		if ((dac >= DAC_IR_CAL_MIN) && (dac <= DAC_IR_CAL_MAX)){
			return (CAL_OK);
		}
	}
	save_err_rec(ERR_PD2_CAL_RD);
	return (CAL_ERROR);
}/** End test_tir_cal_values() **/
#endif
/**************************************************************
 * @brief 校验TIR校准值
 * @param pd2 PD2当前读数
 * @param dac DAC当前值
 * @param attempt 尝试次数(0=5%,1=10%,2=15%,3=20%,4=25%,5=30%)
 * @return CAL_OK 或 CAL_ERROR
 */
uint8_t test_tir_cal_values(int16_t pd2, int16_t dac, uint8_t attempt)
{
    // 动态计算PD2范围
    const float tolerances[] = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30};
    uint8_t num_tolerances = sizeof(tolerances) / sizeof(tolerances[0]);

    if (attempt >= num_tolerances) {
        attempt = num_tolerances - 1;
    }

    float tolerance = tolerances[attempt];
    int16_t min_allowed = (int16_t)(PD2_IR_CAL_TARGET * (1.0 - tolerance));
    int16_t max_allowed = (int16_t)(PD2_IR_CAL_TARGET * (1.0 + tolerance));

    // 保持原函数的判断结构：PD2和DAC都要检查
    if ((pd2 >= min_allowed) && (pd2 <= max_allowed)) {
        if ((dac >= DAC_IR_CAL_MIN) && (dac <= DAC_IR_CAL_MAX)) {
            return CAL_OK;
        }
    }

    save_err_rec(ERR_PD2_CAL_RD);
    return CAL_ERROR;
}
/**************************************************************
 * @brief 检查TIR传感器校准值有效性
 * @return uint8_t 返回校准状态 CAL_OK(成功) 或 CAL_ERROR(失败)
 * @note 功能说明：
 * 1. 从EEPROM读取校准参数和校验和
 * 2. 验证参数有效性和校验和
 * 3. 失败时记录错误日志并显示告警
 * 4. 成功时更新全局DAC校准值
 **************************************************************/
uint8_t check_tir_calibration(void)
{
    /* 局部变量声明 */
    int16_t pd2_cal = 0;      // PD2传感器校准值
    int16_t dac_cal = 0;      // DAC输出校准值
    int16_t stored_checksum = 0; // EEPROM存储的校验和
    int16_t calculated_sum = 0;  // 实时计算的校验和
    int8_t validation_status = CAL_ERROR; // 默认设为失败状态
    uint8_t attempt;           // 新增：尝试次数

    /* 步骤1：从EEPROM读取校准数据 */
    stored_checksum = read_tir_cal(&pd2_cal, &dac_cal);

    /* 步骤2：计算实时校验和 */
    calculated_sum = pd2_cal + dac_cal;

    /* 步骤3：验证校准值有效性 - 从严格到宽松依次尝试 */
    for (attempt = 0; attempt <= 5; attempt++) {
        validation_status = test_tir_cal_values(pd2_cal, dac_cal, attempt);
        if (validation_status == CAL_OK) {
            break;  // 找到可接受的标准就退出循环
        }
    }
    /* 校验流程 */
    if (validation_status == CAL_OK)
    {
        if (calculated_sum == stored_checksum)
        {
            /* 校验成功：更新全局DAC值 */
            G_dac_cal_value = dac_cal;
            return CAL_OK;
        }
    }

    /* 错误处理流程 */
    if (calculated_sum != stored_checksum)
    {
        save_err_rec(ERR_CS_CAL_RD); // 记录校验和错误
    }

    /* 显示持续告警（直到用户取消）*/
    show_error_message(UNTIL_CANCEL,
                     "CALIBRATION ERROR",
                     "RUN MODE DISABLED");

    return CAL_ERROR;
}


/**************************************************************
 * check_tir_calibration(void)
 * @detail
 * Test if TIR sensor has a valid calibration values.
 * Return status CAL_OK or CAL_ERROR. Save error to EEPROM log.
 */

#if(0)
uint8_t check_tir_calibration(void)
{
	int16_t pd2,dac,chksum,sum;
	int8_t stat;

	chksum = read_tir_cal(&pd2,&dac);           //Read TIR calibration values and CS from EEPROM
	sum = pd2+dac;
	stat = test_tir_cal_values(pd2,dac);


	if (stat==CAL_OK){
		if (sum == chksum){
			G_dac_cal_value = dac;
			return CAL_OK;
		}
	}
	if (sum != chksum){
		save_err_rec(ERR_CS_CAL_RD);
	}
	show_error_message(UNTIL_CANCEL,"CALIBRATION ERROR", "RUN MODE DISABLED");
	return CAL_ERROR;

}
#endif
/** End check_tir_calibration() **/

/**************************************************************
 * pic_i2c_read()
 * @detail
 * Read one byte data from PIC I2C data buffer.
 */
uint8_t pic_i2c_read(uint8_t addr)
{
	uint8_t ret, buf[3];

    buf[0] = addr; 														//data address
    ret = HAL_I2C_Master_Transmit(&hi2c3, 0x60, buf, 1, HAL_MAX_DELAY); //write data address to PIC
    if ( ret != HAL_OK ) {		 }
    HAL_Delay(50);

    ret = HAL_I2C_Master_Receive(&hi2c3, 0x60, buf, 1, HAL_MAX_DELAY);  //read data from PIC
    if ( ret != HAL_OK ) {		 }
    return(buf[0]);
}/** End pic_i2c_read() **/

/**************************************************************
 * start_analog_scan()
 * @detail
 * Clear the flag which controls the automatic analog readings.
 * Clearing the flag will stops processing the analog conversions
 * which are taken in TIM22 interrupt. TIM22 needs to stay on since
 * it is also used for 5ms timer.
 */
void start_analog_scan(void)
{
	G_an_on=1;
}/** End start_analog_scan() **/

/**************************************************************
 * stop_analog_scan()
 * @detail
 * Set the flag which controls the automatic analog readings.
 * Setting the flag will start processing the analog conversions
 * which are taken in TIM22 interrupt.
 */
void stop_analog_scan(void)
{
	G_an_on=0;
}/** End stop_analog_scan() **/
#if(0)
// 滑动平均 + 基线校正 (返回最大值)
static int16_t preprocess_data(int16_t occl_data[], int16_t *baseline)
{
	int16_t max_value = 0;
	// 改进的基线计算：前4个点的平均值（需确保数组长度足够）
	int32_t baseline_sum = 0;
	for (uint8_t i = DATA_START; i < BASELINE_POINTS; i++) {
	        baseline_sum += occl_data[i];
	    }
	*baseline = baseline_sum / (BASELINE_POINTS-1);

    for (uint8_t i = DATA_START; i <= DATA_END ; i++) {
        // 4点滑动平均
        int32_t sum = 0;
        for (uint8_t j = 0; j < WINDOW_SIZE; j++) {
            sum += occl_data[i + j];
        }
        int16_t smoothed = sum / WINDOW_SIZE;

        // 基线校正并更新最大值
        occl_data[i] = smoothed - *baseline;
        max_value = MAX(max_value, occl_data[i]);
    }
    // 处理全非正数据
       if (max_value <= 0) {
           return 0;  // 或定义错误码
       }
    return max_value;
}
#endif
#if(0)// 数据归一化
static void normalize_data(int16_t occl_data[], int16_t max_value)
{
    if (max_value <= 0) return;

    for (uint8_t i = DATA_START; i<END_POINT; i++) {
        int32_t normalized = ((int32_t)occl_data[i] * NORMALIZATION_MAX+ max_value / 2) / max_value;
        occl_data[i] = (int16_t)normalized;
    }
}
#endif
#if(1)
void occl_normalize_max(int16_t occl_data[])
{
	// 0. 检测和处理原始数据中的零值和负值
	detect_and_handle_invalid_values(occl_data);
	//   1. 中值滤波去除噪声 int16_t baseline = 0;

    median_filter(occl_data,END_POINT);

    // 2. 检测稳定阶段（前后端都检测）
    uint16_t front_start, front_end, back_start, back_end;
    bool has_stable_phase = detect_stable_phase_with_adjust(occl_data, &front_start, &front_end, &back_start, &back_end);

    // 3. 查找峰值用于归一化（在减去基线前获取最大值）
   int16_t max_value = find_peak_value(occl_data);

    // 4. 使用混合方法计算基线（传入稳定阶段信息和max_value）
   int16_t baseline = calculate_hybrid_baseline_with_stable(occl_data, front_start, front_end, back_start, back_end, has_stable_phase, max_value);

    // 5. 数据预处理（减去基线）
    for (uint8_t i = 0; i < END_POINT; i++) {
        occl_data[i] -= baseline;
    }
 // 6. 重新计算减去基线后的最大值
    max_value = find_peak_value(occl_data);

  // 7. 对稳定阶段之前的数据进行平滑处理
    smooth_pre_stable_phase(occl_data, has_stable_phase, front_start, front_end, back_start, back_end);

   // 8. 执行归一化（传入前后端稳定区域信息）
    normalize_data_flexible(occl_data, max_value, front_start, front_end, back_start, back_end, has_stable_phase);
    low_pass_filter(occl_data,END_POINT);
}
#endif

// 检测并处理原始数据中的零值和负值，使用临近有效数据替换
static void detect_and_handle_invalid_values(int16_t occl_data[]) {
	uint16_t first_valid_index = 0;

	    // 找到第一个有效值（大于等于1000）的索引
	    for (uint16_t i = 0; i < END_POINT; i++) {
	        if (occl_data[i] >= 1000) {
	            first_valid_index = i;
	            break;
	        }
	    }

	    // 如果第一个点就是有效值，检查后续是否有无效值
	    if (first_valid_index == 0 && occl_data[0] >= 1000) {
	        // 检查后续数据是否有无效值（<1000）
	        for (uint16_t i = 1; i < END_POINT; i++) {
	            if (occl_data[i] < 1000) {
	                // 使用前一个有效值替换无效值
	                occl_data[i] = occl_data[i-1];
	            }
	        }
	        return;
	    }

	    // 处理开头的无效值（零值、负值或小于1000的正值）
	    if (first_valid_index > 0) {
	        // 使用第一个有效值填充所有前面的无效值
	        int16_t first_valid_value = occl_data[first_valid_index];
	        for (uint16_t i = 0; i < first_valid_index; i++) {
	            occl_data[i] = first_valid_value;
	        }

	        // 处理后续可能出现的无效值
	        for (uint16_t i = first_valid_index + 1; i < END_POINT; i++) {
	            if (occl_data[i] < 1000) {
	                // 使用前一个有效值替换无效值
	                occl_data[i] = occl_data[i-1];
	            }
	        }
	    }
}
#if(0)
// 查找第一个有效值（大于0）的索引
static uint16_t find_first_valid_index(int16_t data[]) {
    for (uint16_t i = 0; i < END_POINT; i++) {
        if (data[i] > 0) {
            return i;
        }
    }
    return END_POINT; // 没有找到有效值
}

// 查找指定位置前最近的有效值
static int16_t find_prev_valid_value(int16_t data[], uint16_t index) {
    for (int16_t i = index - 1; i >= 0; i--) {
        if (data[i] > 0) {
            return data[i];
        }
    }
    return 0; // 没有找到有效值
}

// 查找指定位置后最近的有效值
static int16_t find_next_valid_value(int16_t data[], uint16_t index) {
    for (uint16_t i = index + 1; i < END_POINT; i++) {
        if (data[i] > 0) {
            return data[i];
        }
    }
    return 0; // 没有找到有效值
}

// 查找最后一个有效值
static int16_t find_last_valid_value(int16_t data[]) {
    for (int16_t i = END_POINT - 1; i >= 0; i--) {
        if (data[i] > 0) {
            return data[i];
        }
    }
    return 0; // 没有找到有效值
}

// 使用稳定阶段计算基线
static int16_t calculate_baseline_from_stable_phase(int16_t occl_data[]) {
    // 1. 检测稳定阶段
    uint16_t stable_start, stable_end;
    bool has_stable_phase = detect_stable_phase(occl_data, &stable_start, &stable_end);

    if (has_stable_phase) {
        // 2. 使用稳定阶段计算基线
        return calculate_mean(occl_data, stable_start, stable_end);
    } else {
        // 3. 回退策略：使用初始阶段计算基线
        return calculate_baseline_from_initial(occl_data);
    }
}

// 混合基线计算方法：稳定性阶段平均值与前几位数据平均值的加权平均
static int16_t calculate_hybrid_baseline(int16_t occl_data[]) {
    // 1. 检测稳定阶段
    uint16_t stable_start, stable_end;
    bool has_stable_phase = detect_stable_phase(occl_data, &stable_start, &stable_end);

    // 2. 计算前几位数据的平均值（跳过可能的无效值）
    int16_t initial_avg = calculate_initial_average(occl_data);

    if (has_stable_phase) {
        // 3. 计算稳定阶段的平均值
        int16_t stable_avg = calculate_mean(occl_data, stable_start, stable_end);

        // 4. 计算混合基线（加权平均）
        return calculate_weighted_average(initial_avg, stable_avg);
    } else {
        // 没有检测到稳定阶段，只使用初始平均值
        return initial_avg;
    }
}



// 计算加权平均值 - 根据与最大值差的相对大小分配权重
static int16_t calculate_weighted_average(int16_t initial_avg, int16_t stable_avg, int16_t max_value) {
	 if (max_value <= 0) {
	        // 默认权重
	        return (initial_avg * 3 + stable_avg * 7) / 10;
	    }

	    // 计算相对比例（值越大，权重越高）
	    float initial_ratio = (float)initial_avg / max_value;
	    float stable_ratio = (float)stable_avg / max_value;

	    // 根据比例计算权重（值大的权重高）
	    uint8_t initial_weight, stable_weight;

	    if (initial_avg >= stable_avg) {
	        // initial_avg更大，给initial更高权重
	        initial_weight = 9;
	        stable_weight = 1;
	    } else {
	        // stable_avg更大，给stable更高权重
	        initial_weight = 1;
	        stable_weight = 9;
	    }

	    // 进一步根据与最大值的接近程度调整
	    if (initial_ratio > 0.8f) initial_weight += 2;  // 接近最大值，增加权重
	    if (stable_ratio > 0.8f) stable_weight += 2;    // 接近最大值，增加权重

	    // 确保权重和不超过10
	    if (initial_weight + stable_weight > 10) {
	        float scale = 10.0f / (initial_weight + stable_weight);
	        initial_weight = (uint8_t)(initial_weight * scale);
	        stable_weight = (uint8_t)(stable_weight * scale);
	    }

	    // 计算加权平均值
	    int32_t weighted_sum = (int32_t)initial_avg * initial_weight +
	                          (int32_t)stable_avg * stable_weight;

	    return weighted_sum / (initial_weight + stable_weight);
}
// 计算加权平均值
static int16_t calculate_weighted_average(int16_t initial_avg, int16_t stable_avg) {
    // 根据数据可靠性分配权重
    // 稳定阶段通常更可靠，给予较高权重
    const uint8_t stable_weight = 7;    // 70%权重
    const uint8_t initial_weight = 3;   // 30%权重

    int32_t weighted_sum = (int32_t)stable_avg * stable_weight +
                          (int32_t)initial_avg * initial_weight;

    return weighted_sum / (stable_weight + initial_weight);
}

// 改进的稳定阶段检测（考虑数据稳定性）
static bool detect_stable_phase(int16_t data[], uint16_t *start_index, uint16_t *end_index) {
	 const uint8_t window_size = 10;
	    const uint16_t min_stable_length = STABLE_PHASE_MIN_LENGTH;

	    // 从数据末尾开始向前搜索稳定阶段
	    uint16_t search_start = (END_POINT > min_stable_length) ?
	                           (END_POINT - min_stable_length) : 0;

	    uint16_t current_end = END_POINT - 1;
	    uint16_t current_start = current_end;
	    bool found_stable = false;

	    // 从后向前滑动窗口检测稳定性
	    for (int16_t i = search_start; i >= window_size; i--) {
	        float variance = calculate_window_variance(data, i, window_size);

	        if (variance <= STABILITY_VARIANCE_THRESHOLD) {
	            // 找到稳定点，尝试扩展稳定区域
	            if (!found_stable) {
	                current_end = i + window_size - 1;
	                found_stable = true;
	            }
	            current_start = i;
	        } else {
	            // 遇到不稳定点，检查是否已找到足够长的稳定区域
	            if (found_stable && (current_end - current_start >= min_stable_length - 1)) {
	                break;
	            }
	            found_stable = false;
	        }
	    }

	    // 检查是否找到有效的稳定阶段
	    if (found_stable && (current_end - current_start >= min_stable_length - 1)) {
	        // 微调边界，确保完全在稳定区域内
	        *start_index = adjust_start_boundary(data, current_start, STABILITY_VARIANCE_THRESHOLD);
	        *end_index = adjust_end_boundary(data, current_end, STABILITY_VARIANCE_THRESHOLD);

	        // 最终长度检查
	        if (*end_index - *start_index >= min_stable_length - 1) {
	            return true;
	        }
	    }

	    return false;
}

// 调整起始边界，确保从稳定点开始
static uint16_t adjust_start_boundary(int16_t data[], uint16_t start, float variance_threshold) {
    const uint8_t check_window = 5;

    // 向前检查几个点，找到真正的稳定起始点
    for (uint16_t i = start; i > check_window; i--) {
        float variance = calculate_window_variance(data, i, check_window);
        if (variance > variance_threshold * 1.2f) {
            return i + 1; // 返回第一个稳定点
        }
    }
    return (start > check_window) ? (start - check_window) : 0;
}

// 调整结束边界，确保在稳定区域内结束
static uint16_t adjust_end_boundary(int16_t data[], uint16_t end, float variance_threshold) {
    const uint8_t check_window = 5;
    uint16_t max_index = (END_POINT < end + check_window) ? END_POINT : (end + check_window);

    for (uint16_t i = end; i < max_index; i++) {
        float variance = calculate_window_variance(data, i, check_window);
        if (variance > variance_threshold * 1.2f) {
            return (i > 0) ? (i - 1) : 0; // 返回最后一个稳定点
        }
    }
    return end;
}

// 计算窗口方差
static float calculate_window_variance(int16_t data[], uint16_t start, uint8_t size) {
    int32_t sum = 0;
    int32_t sum_sq = 0;

    for (uint8_t i = 0; i < size; i++) {
        sum += data[start + i];
        sum_sq += data[start + i] * data[start + i];
    }

    float mean = (float)sum / size;
    float variance = (float)sum_sq / size - mean * mean;

    return variance;
}
#endif

// 改进的稳定区域检测函数，同时检测前后端稳定区域
static bool detect_stable_phase_with_adjust(int16_t data[], uint16_t *front_start, uint16_t *front_end,
                                           uint16_t *back_start, uint16_t *back_end) {
	  	const uint16_t min_stable_length = 10;
	    const int16_t allowed_deviation = 5;
	    const int16_t pre_stable_deviation = 10; // 稳定区域前数据的波动阈值

	    if (END_POINT < min_stable_length) {
	        return false;
	    }

	    // 初始化输出参数
	    *front_start = *front_end = *back_start = *back_end = 0;

	    bool has_front_stable = false;
	    bool has_back_stable = false;

	    // 方法1：检测末端稳定区域（后端不修正）
	    int32_t sum_back = 0;
	    uint16_t back_sample_count = (END_POINT > 10) ? 10 : END_POINT;
	    for (uint16_t i = END_POINT - back_sample_count; i < END_POINT; i++) {
	        sum_back += data[i];
	    }
	    int16_t ref_value_back = sum_back / back_sample_count;
	    int16_t lower_bound_back = ref_value_back - allowed_deviation;
	    int16_t upper_bound_back = ref_value_back + allowed_deviation;

	    *back_end = END_POINT - 1;
	    *back_start = END_POINT - 1;

	    // 向前搜索末端稳定区域
	    for (int16_t i = END_POINT - 2; i >= 0; i--) {
	        if (data[i] >= lower_bound_back && data[i] <= upper_bound_back) {
	            *back_start = (uint16_t)i;
	        } else {
	            break;
	        }
	    }
	    uint16_t back_stable_length = *back_end - *back_start + 1;
	    has_back_stable = (back_stable_length >= min_stable_length);

	    // 方法2：检测前端稳定区域
	    int32_t sum_front = 0;
	    uint16_t front_sample_count = (END_POINT > 10) ? 10 : END_POINT;
	    for (uint16_t i = 0; i < front_sample_count; i++) {
	        sum_front += data[i];
	    }
	    int16_t ref_value_front = sum_front / front_sample_count;
	    int16_t lower_bound_front = ref_value_front - allowed_deviation;
	    int16_t upper_bound_front = ref_value_front + allowed_deviation;

	    *front_start = 0;
	    *front_end = 0;

	    // 向后搜索前端稳定区域
	    for (uint16_t i = 0; i < END_POINT; i++) {
	        if (data[i] >= lower_bound_front && data[i] <= upper_bound_front) {
	            if (!has_front_stable) {
	                *front_start = i;
	                has_front_stable = true;
	            }
	            *front_end = i;
	        } else if (has_front_stable) {
	            break;
	        }
	    }
	    uint16_t front_stable_length = (*front_end - *front_start + 1);
	    has_front_stable = (front_stable_length >= min_stable_length);

	    // 只修正前端稳定区域之前的数据（后端不修正）
	    if (has_front_stable && *front_start > 0) {
	        int16_t first_stable_value = data[*front_start]; // 稳定区域第一个值

	        // 向前检查稳定区域之前的数据
	        for (int16_t i = *front_start - 1; i >= 0; i--) {
	            int16_t diff = data[i] - first_stable_value;
	            if (diff >= -pre_stable_deviation && diff <= pre_stable_deviation) {
	                // 波动在±10之内，修正为稳定区域第一个值
	                data[i] = first_stable_value;
	                // 扩展稳定区域起始点
	                *front_start = (uint16_t)i;
	            } else {
	                // 波动超过阈值，停止修正
	                break;
	            }
	        }

	        // 更新前端稳定区域长度
	        front_stable_length = *front_end - *front_start + 1;
	        has_front_stable = (front_stable_length >= min_stable_length);
	    }

	    return has_front_stable || has_back_stable;
}
#if(0)
// 带边界微调的版本
static bool detect_stable_phase_with_adjust(int16_t data[], uint16_t *start_index, uint16_t *end_index) {
    const uint16_t min_stable_length = 10;
    const int16_t allowed_deviation = 8;  // 允许的偏差值±5

     if (END_POINT < min_stable_length) {
         return false;
     }

     // 计算参考值（最后10点平均值）
     int32_t sum = 0;
     for (uint16_t i = END_POINT - 10; i < END_POINT; i++) {
         sum += data[i];
     }
     int16_t ref_value = sum / 10;

     // 确定稳定区域的上下边界
     int16_t lower_bound = ref_value - allowed_deviation;  // 下限：参考值-5
     int16_t upper_bound = ref_value + allowed_deviation;  // 上限：参考值+5

     *end_index = END_POINT - 1;
     *start_index = END_POINT - 1;

     // 向前搜索：值在[lower_bound, upper_bound]范围内的点都算稳定
     for (int16_t i = END_POINT - 2; i >= 0; i--) {
         if (data[i] >= lower_bound && data[i] <= upper_bound) {
             *start_index = (uint16_t)i;
         } else {
             break;  // 遇到超出±5范围的点
         }
     }

     return (*end_index - *start_index + 1 >= min_stable_length);
}

// 改进的稳定阶段检测 - 专门针对您的数据特征
static bool detect_stable_phase(int16_t data[], uint16_t *start_index, uint16_t *end_index) {
    const uint8_t window_size = 5;  // 减小窗口大小，提高灵敏度
    const uint16_t min_stable_length = 15;  // 最小稳定长度
    const float variance_threshold = 1.0f;  // 降低方差阈值
    const int16_t derivative_threshold = 2;  // 导数阈值

    if (END_POINT < min_stable_length + window_size) {
           return false;
       }

       // 正确的初始化：使用标志变量而不是魔数
       bool found_stable_end = false;
       uint16_t stable_start = END_POINT;  // 初始化为一个明显无效的值
       uint16_t stable_end = 80;
       uint16_t consecutive_count = 0;

       uint16_t search_start = (END_POINT > 100) ? 100 : END_POINT - 10;

       for (int16_t i = (int16_t)search_start; i >= (int16_t)window_size - 1; i--) {
           uint16_t current_index = (uint16_t)i;

           if (current_index + window_size > END_POINT) {
               continue;
           }

           bool is_stable = check_window_stability(data, current_index, window_size,
                                                 variance_threshold, derivative_threshold);

           if (is_stable) {
               if (!found_stable_end) {
                   stable_end = current_index + window_size - 1;
                   if (stable_end >= END_POINT) stable_end = END_POINT - 1;
                   found_stable_end = true;
               }
               consecutive_count++;

               if (consecutive_count >= min_stable_length) {
                   stable_start = current_index;
                   break;
               }
           } else {
               if (consecutive_count >= min_stable_length / 2 && found_stable_end) {
                   stable_start = current_index + 1;
                   break;
               }
               found_stable_end = false;
               consecutive_count = 0;
           }
       }

       // 正确的验证逻辑
       if (found_stable_end && stable_start < END_POINT) {
           if (stable_start > stable_end) {
               uint16_t temp = stable_start;
               stable_start = stable_end;
               stable_end = temp;
           }

           stable_start = adjust_start_boundary(data, stable_start, variance_threshold);
           stable_end = adjust_end_boundary(data, stable_end, variance_threshold);

           if (stable_end - stable_start + 1 >= min_stable_length) {
               *start_index = stable_start;
               *end_index = stable_end;
               return true;
           }
       }

       return false;
}
// 调整起始边界
static uint16_t adjust_start_boundary(int16_t data[], uint16_t start, float variance_threshold) {
    const uint8_t check_window = 3;

    for (int16_t i = (int16_t)start; i >= check_window; i--) {
        if (i + check_window >= END_POINT) continue;

        float variance = calculate_window_variance(data, (uint16_t)i, check_window);
        if (variance > variance_threshold * 2.0f) {
            return (uint16_t)(i + 1);
        }
    }
    return (start > check_window) ? (start - check_window) : 0;
}

// 调整结束边界
static uint16_t adjust_end_boundary(int16_t data[], uint16_t end, float variance_threshold) {
    const uint8_t check_window = 3;

    for (uint16_t i = end; i < END_POINT - check_window; i++) {
        float variance = calculate_window_variance(data, i, check_window);
        if (variance > variance_threshold * 2.0f) {
            return (i > 0) ? (i - 1) : 0;
        }
    }
    return (END_POINT - 1);
}
// 检查窗口稳定性（多条件判断）
static bool check_window_stability(int16_t data[], uint16_t start, uint8_t window_size,
                                  float variance_threshold, int16_t derivative_threshold) {
    if (start + window_size > END_POINT) {
        return false;
    }

    // 1. 方差检查
    float variance = calculate_window_variance(data, start, window_size);
    if (variance > variance_threshold) {
        return false;
    }

    // 2. 最大变化检查
    int16_t max_diff = 0;
    for (uint8_t i = 1; i < window_size; i++) {
        int16_t diff = abs(data[start + i] - data[start + i - 1]);
        if (diff > max_diff) max_diff = diff;
    }
    if (max_diff > derivative_threshold) return false;

    // 3. 总体范围检查
    int16_t min_val = data[start];
    int16_t max_val = data[start];
    for (uint8_t i = 1; i < window_size; i++) {
        if (data[start + i] < min_val) min_val = data[start + i];
        if (data[start + i] > max_val) max_val = data[start + i];
    }
    if (max_val - min_val > 3) return false;

    return true;
}

// 计算窗口方差（改进版）
static float calculate_window_variance(int16_t data[], uint16_t start, uint8_t window_size) {
    if (start + window_size > END_POINT) {
        return FLT_MAX;
    }

    int32_t sum = 0;
    for (uint8_t i = 0; i < window_size; i++) {
        sum += data[start + i];
    }
    float mean = (float)sum / window_size;

    float variance = 0;
    for (uint8_t i = 0; i < window_size; i++) {
        float diff = data[start + i] - mean;
        variance += diff * diff;
    }

    return variance / window_size;
}
#endif
#if(0)// 计算数据段的平均值
static int16_t calculate_mean(int16_t data[], uint16_t start, uint16_t end) {
    if (start > end) return 0;

    int32_t sum = 0;
    uint16_t count = end - start + 1;

    for (uint16_t i = start; i <= end; i++) {
        sum += data[i];
    }

    return sum / count;
}

// 回退策略：使用初始阶段计算基线
static int16_t calculate_baseline_from_initial(int16_t data[]) {
    const uint8_t initial_window = 10;
    int32_t sum = 0;

    for (uint8_t i = 0; i < initial_window && i < END_POINT; i++) {
        sum += data[i];
    }

    return sum / MIN(initial_window, END_POINT);
}
#endif
// 查找峰值
static int16_t find_peak_value(int16_t occl_data[]) {
    int16_t max_value = 0;

    // 查找全局最大值
    for (uint8_t i = 0; i < END_POINT; i++) {
        if (occl_data[i] > max_value) {
            max_value = occl_data[i];
        }
    }

    return max_value;
}
// 完善版本 - 包含边界检查
static int16_t calculate_hybrid_baseline_with_stable(int16_t data[], uint16_t front_start, uint16_t front_end,
                                                    uint16_t back_start, uint16_t back_end, bool has_stable_phase, int16_t max_value)
{
    // 获取前端代表性值
    int16_t front_value = get_representative_value(data, front_start, front_end, has_stable_phase, true);

    // 获取后端代表性值
    int16_t back_value = get_representative_value(data, back_start, back_end, has_stable_phase, false);

    // 比较前后值，小的一端占比90%
    if (front_value <= back_value) {
        // 前端值较小，前端权重90%
        return (front_value * 9 + back_value * 1) / 10;
    } else {
        // 后端值较小，后端权重90%
        return (front_value * 1 + back_value * 9) / 10;
    }
}

// 获取代表性值的辅助函数
static int16_t get_representative_value(int16_t data[], uint16_t stable_start, uint16_t stable_end,
                                       bool has_stable_phase, bool is_front)
{
    if (has_stable_phase && stable_end >= stable_start && (stable_end - stable_start + 1) >= 10) {
        // 使用稳定区域平均值
        int32_t sum = 0;
        uint16_t count = stable_end - stable_start + 1;
        for (uint16_t i = stable_start; i <= stable_end; i++) {
            sum += data[i];
        }
        return sum / count;
    } else {
        // 使用端点附近5个点的平均值
        int32_t sum = 0;
        uint16_t count = 5;

        if (is_front) {
            // 前端：使用前5个点
            count = (END_POINT > 5) ? 5 : END_POINT;
            for (uint16_t i = 0; i < count; i++) {
                sum += data[i];
            }
        } else {
            // 后端：使用后5个点
            count = (END_POINT > 5) ? 5 : END_POINT;
            uint16_t start = (END_POINT > count) ? END_POINT - count : 0;
            for (uint16_t i = start; i < END_POINT; i++) {
                sum += data[i];
            }
        }

        return (count > 0) ? (sum / count) : 0;
    }
}
#if(0)
// 修改基线计算函数，支持前后端稳定区域
static int16_t calculate_hybrid_baseline_with_stable(int16_t data[], uint16_t front_start, uint16_t front_end,
                                                    uint16_t back_start, uint16_t back_end, bool has_stable_phase, int16_t max_value)
{
	 if (!has_stable_phase) {
	        // 如果没有稳定阶段，使用传统方法
	        return calculate_initial_baseline(data);
	    }

	    int16_t front_avg = 0, back_avg = 0;
	    bool use_front = false, use_back = false;

	    // 计算前端稳定区域平均值
	    if (front_end >= front_start && (front_end - front_start + 1) >= 10) {
	        int32_t front_sum = 0;
	        uint16_t front_count = front_end - front_start + 1;
	        for (uint16_t i = front_start; i <= front_end; i++) {
	            front_sum += data[i];
	        }
	        front_avg = front_sum / front_count;
	        use_front = true;
	    }

	    // 计算后端稳定区域平均值
	    if (back_end >= back_start && (back_end - back_start + 1) >= 10) {
	        int32_t back_sum = 0;
	        uint16_t back_count = back_end - back_start + 1;
	        for (uint16_t i = back_start; i <= back_end; i++) {
	            back_sum += data[i];
	        }
	        back_avg = back_sum / back_count;
	        use_back = true;
	    }

	    // 根据检测到的稳定区域情况计算基线
	    if (use_front && use_back) {
	        // 前后端都有稳定区域，使用加权平均
	        return calculate_weighted_average(front_avg, back_avg, max_value);
	    } else if (use_front) {
	        // 只有前端有稳定区域，比较前后值大小
	        int16_t front_non_stable_avg = calculate_front_non_stable_avg(data, front_start);
	        int16_t back_non_stable_avg = calculate_back_non_stable_avg(data, front_end);

	        // 使用较小的一方作为基线
	        if (front_non_stable_avg <= back_non_stable_avg) {
	            return front_non_stable_avg;
	        } else {
	            return back_non_stable_avg;
	        }
	    } else if (use_back) {
	        // 只有后端有稳定区域，比较前后值大小
	        int16_t front_non_stable_avg = calculate_front_non_stable_avg(data, back_start);
	        int16_t back_non_stable_avg = calculate_back_non_stable_avg(data, back_end);

	        // 使用较小的一方作为基线
	        if (front_non_stable_avg <= back_non_stable_avg) {
	            return front_non_stable_avg;
	        } else {
	            return back_non_stable_avg;
	        }
	    } else {
	        // 没有稳定区域，使用传统方法
	        return calculate_initial_baseline(data);
	    }
}

// 计算稳定区域前端的非稳定区域平均值
static int16_t calculate_front_non_stable_avg(int16_t data[], uint16_t stable_start) {
    if (stable_start <= 0) return 0;

    int32_t sum = 0;
    uint16_t count = (stable_start > 5) ? 5 : stable_start; // 取前5个点或尽可能多的点
    for (uint16_t i = 0; i < count; i++) {
        sum += data[i];
    }
    return sum / count;
}

// 计算稳定区域后端的非稳定区域平均值
static int16_t calculate_back_non_stable_avg(int16_t data[], uint16_t stable_end) {
    if (stable_end >= END_POINT - 1) return 0;

    int32_t sum = 0;
    uint16_t start = stable_end + 1;
    uint16_t count = (END_POINT - start > 5) ? 5: (END_POINT - start); // 取后10个点或尽可能多的点
    for (uint16_t i = start; i < start + count && i < END_POINT; i++) {
        sum += data[i];
    }
    return sum / count;
}

// 计算初始基线 - 参考原始归一化函数的逻辑
static int16_t calculate_initial_baseline(int16_t data[])
{
    // 使用原始函数中的方法：使用第一个4点滚动平均值作为基线（零偏移）
    if (END_POINT < 5) {
        return 0; // 数据点太少，返回0
    }

    // 计算第一个4点滚动平均值（对应原始函数中i=1时的occl_var）
    int32_t first_rolling_avg = (data[1] + data[2] + data[3] + data[4]) / 4;

    return (int16_t)first_rolling_avg;
}
#endif
// 对稳定阶段之前的数据进行平滑处理
static void smooth_pre_stable_phase(int16_t data[], bool has_stable_phase,
                                   uint16_t front_start, uint16_t front_end,
                                   uint16_t back_start, uint16_t back_end)
{
    if (!has_stable_phase) {
        return; // 没有稳定阶段，不需要平滑
    }

    // 确定需要平滑的区域
    uint16_t smooth_start = 0;
    uint16_t smooth_end = 0;

    if (front_end >= front_start && (front_end - front_start + 1) >= 10) {
        // 前端有稳定区域，平滑前端稳定区域之前的数据
        if (front_start > 0) {
            smooth_start = 0;
            smooth_end = front_start - 1;
        }
    } else if (back_end >= back_start && (back_end - back_start + 1) >= 10) {
        // 后端有稳定区域，平滑后端稳定区域之前的数据
        smooth_start = 0;
        smooth_end = back_start - 1;
    } else {
        return; // 没有有效的稳定区域
    }

    // 确保平滑区域有效
    if (smooth_end <= smooth_start || smooth_end >= END_POINT) {
        return;
    }

    // 使用移动平均进行平滑
    const uint8_t window_size = 5; // 滑动窗口大小
    int16_t smoothed_data[END_POINT];

    // 复制原始数据
    for (uint16_t i = 0; i < END_POINT; i++) {
        smoothed_data[i] = data[i];
    }

    // 对指定区域进行平滑
    for (uint16_t i = smooth_start; i <= smooth_end; i++) {
        int32_t sum = 0;
        uint8_t count = 0;

        // 计算滑动窗口内的平均值
        for (int16_t j = i - window_size/2; j <= i + window_size/2; j++) {
            if (j >= 0 && j < END_POINT) {
                sum += data[j];
                count++;
            }
        }

        if (count > 0) {
            smoothed_data[i] = sum / count;
        }
    }

    // 将平滑后的数据复制回原数组
    for (uint16_t i = smooth_start; i <= smooth_end; i++) {
        data[i] = smoothed_data[i];
    }
}
#if(0)
// 对指定数据范围进行滑动平均滤波
static void sliding_average_filter_range(int16_t *data, uint16_t start, uint16_t end, uint8_t window_size) {
    if (start >= end || window_size < 2) return;

    // 创建临时数组存储原始数据
    int16_t temp[end - start + 1];
    for (uint16_t i = start; i <= end; i++) {
        temp[i - start] = data[i];
    }

    // 应用滑动平均
    for (uint16_t i = start; i <= end - window_size + 1; i++) {
        int32_t sum = 0;
        for (uint8_t j = 0; j < window_size; j++) {
            sum += temp[i - start + j];
        }
        data[i + window_size/2] = sum / window_size;
    }
}

// 处理过渡区域（稳定阶段开始前的最后几个点）
static void process_transition_region(int16_t *data, uint16_t smooth_end, uint16_t stable_start) {
    const uint8_t transition_size = 5; // 过渡区域大小

    // 确保过渡区域不会超出数据范围
    uint16_t transition_start = (smooth_end > transition_size) ?
                                smooth_end - transition_size : 0;

    // 计算稳定阶段的初始平均值
    int32_t stable_sum = 0;
    uint16_t stable_count = MIN(transition_size, stable_start - smooth_end);

    for (uint16_t i = stable_start; i < stable_start + stable_count; i++) {
        stable_sum += data[i];
    }

    if (stable_count == 0) return;

    int16_t stable_avg = stable_sum / stable_count;

    // 对过渡区域进行渐变处理
    for (uint16_t i = transition_start; i <= smooth_end; i++) {
        // 计算到稳定阶段的距离权重
        float distance_ratio = (float)(i - transition_start) / (smooth_end - transition_start + 1);

        // 使用渐变权重混合当前值和稳定阶段平均值
        data[i] = (int16_t)(data[i] * (1 - distance_ratio) + stable_avg * distance_ratio);
    }
}
#endif
// 数据归一化 - 使用稳定区域值代替平均值
static void normalize_data_flexible(int16_t occl_data[], int16_t max_value,
                                   uint16_t front_start, uint16_t front_end,
                                   uint16_t back_start, uint16_t back_end,
                                   bool has_stable_phase)
{
	   if (max_value <= 0) return;

	    const int16_t DIFF_THRESHOLD = 50;

	    /* 第一步: 计算前后端平均值 */
	    int32_t front_sum = 0, back_sum = 0;
	    const uint8_t COMPARE_POINTS = 5;

	    for (uint8_t i = 0; i < COMPARE_POINTS && i < END_POINT; i++) {
	        front_sum += occl_data[i];
	    }
	    int16_t front_avg = (COMPARE_POINTS > 0) ? front_sum / COMPARE_POINTS : 0;

	    for (uint8_t i = 0; i < COMPARE_POINTS; i++) {
	        int index = END_POINT - 1 - i;
	        if (index >= 0) {
	            back_sum += occl_data[index];
	        }
	    }
	    int16_t back_avg = (COMPARE_POINTS > 0) ? back_sum / COMPARE_POINTS : 0;

	    int16_t diff = back_avg - front_avg;

	    /* 第二步: 根据差异决定缩放策略 */
	    bool scale_front = false;
	    bool scale_back = false;

	    if (diff > DIFF_THRESHOLD) {
	        scale_back = true;
	    } else if (diff < -DIFF_THRESHOLD) {
	        scale_front = true;
	    }

	    /* 第三步: 确定大的一端和后10位范围 */
	    uint16_t large_end_start = 0;
	    uint16_t large_end_end = 0;

	    if (!has_stable_phase) {
	        if (front_avg >= back_avg) {
	            // 前端值大，取前端后10位
	            large_end_start = (END_POINT > 10) ? END_POINT - 10 : 0;
	            large_end_end = END_POINT - 1;
	        } else {
	            // 后端值大，取后端后10位（其实就是最后10位）
	            large_end_start = (END_POINT > 10) ? END_POINT - 10 : 0;
	            large_end_end = END_POINT - 1;
	        }
	    }

	    /* 第四步: 执行归一化 */
	    for (uint8_t i = DATA_START; i < END_POINT; i++) {
	        int32_t normalized;
	        int32_t scale_factor = NORMAL_SCALE_FACTOR;

	        // 有稳定区域的情况
	        if (has_stable_phase) {
	            if (scale_front && front_end >= front_start) {
	                uint16_t stable_size = front_end - front_start + 1;
	                uint16_t position_in_stable = i - front_start;

	                if (i >= front_start && i <= front_end) {
	                    if (stable_size < 20) {
	                        scale_factor = 500;
	                    } else if (stable_size < 30) {
	                        if (position_in_stable < stable_size - 10) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    } else if (stable_size < 40) {
	                        if (position_in_stable < 10) {
	                            scale_factor = 300;
	                        } else if (position_in_stable < 20) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    } else {
	                        if (position_in_stable < 10) {
	                            scale_factor = 200;
	                        } else if (position_in_stable < 20) {
	                            scale_factor = 300;
	                        } else if (position_in_stable < 30) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    }
	                }
	                else if (i >= front_start - TRANSITION_SIZE && i < front_start) {
	                    float ratio = (float)(i - (front_start - TRANSITION_SIZE)) / TRANSITION_SIZE;
	                    int32_t start_scale = (stable_size < 40) ? 500 : 200;
	                    scale_factor = (int32_t)(NORMAL_SCALE_FACTOR * (1.0f - ratio) + start_scale * ratio);
	                }
	                else if (i > front_end && i <= front_end + TRANSITION_SIZE) {
	                    float ratio = (float)(i - front_end) / TRANSITION_SIZE;
	                    scale_factor = (int32_t)(500 * (1.0f - ratio) + NORMAL_SCALE_FACTOR * ratio);
	                }
	            }
	            else if (scale_back && back_end >= back_start) {
	                uint16_t stable_size = back_end - back_start + 1;
	                uint16_t position_in_stable = i - back_start;

	                if (i >= back_start && i <= back_end) {
	                    if (stable_size < 20) {
	                        scale_factor = 500;
	                    } else if (stable_size < 30) {
	                        if (position_in_stable < stable_size - 10) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    } else if (stable_size < 40) {
	                        if (position_in_stable < 10) {
	                            scale_factor = 300;
	                        } else if (position_in_stable < 20) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    } else {
	                        if (position_in_stable < 10) {
	                            scale_factor = 200;
	                        } else if (position_in_stable < 20) {
	                            scale_factor = 300;
	                        } else if (position_in_stable < 30) {
	                            scale_factor = 400;
	                        } else {
	                            scale_factor = 500;
	                        }
	                    }
	                }
	                else if (i >= back_start - TRANSITION_SIZE && i < back_start) {
	                    float ratio = (float)(i - (back_start - TRANSITION_SIZE)) / TRANSITION_SIZE;
	                    int32_t start_scale = (stable_size < 40) ? 500 : 200;
	                    scale_factor = (int32_t)(NORMAL_SCALE_FACTOR * (1.0f - ratio) + start_scale * ratio);
	                }
	                else if (i > back_end && i <= back_end + TRANSITION_SIZE) {
	                    float ratio = (float)(i - back_end) / TRANSITION_SIZE;
	                    scale_factor = (int32_t)(500 * (1.0f - ratio) + NORMAL_SCALE_FACTOR * ratio);
	                }
	            }
	        }
	        // 没有稳定区域的情况
	        else {
	            // 取大的一端的后10位用缩放因子500
	            if (i >= large_end_start && i <= large_end_end) {
	                scale_factor = 500;
	            }
	            // 其他区域用正常缩放因子
	            else {
	                scale_factor = NORMAL_SCALE_FACTOR;
	            }
	        }

	        // 归一化计算
	        normalized = ((int32_t)occl_data[i] * scale_factor + max_value / 2) / max_value;

	        // 只处理上限，不处理小于0的情况
	        if (normalized > NORMAL_SCALE_FACTOR) {
	            occl_data[i] = (int16_t)NORMAL_SCALE_FACTOR;
	        } else {
	            occl_data[i] = (int16_t)normalized;
	        }
	    }
}
#if(0)
// 数据归一化 - 可配置的缩放比例
static void normalize_data_flexible(int16_t occl_data[], int16_t max_value, uint16_t stable_start)
{
	 if (max_value <= 0) return;

	    // 确保过渡区域不会越界
	    uint16_t transition_start;
	    if (stable_start >= TRANSITION_SIZE) {
	        transition_start = stable_start - TRANSITION_SIZE;
	    } else {
	        transition_start = 0;  // 如果稳定阶段太靠前，缩小过渡区域
	    }

	    // 计算稳定区域大小
	    uint16_t stable_region_size = END_POINT - stable_start;

	    // 根据稳定区域大小确定缩放因子
	    int32_t stable_scale_factor;
	    if (stable_region_size < 10) {
	        stable_scale_factor = 100;
	    } else if (stable_region_size < 20) {
	        stable_scale_factor = 200;
	    } else if (stable_region_size < 30) {
	        stable_scale_factor = 300;
	    } else if (stable_region_size < 40) {
	        stable_scale_factor = 400;
	    } else {
	        stable_scale_factor = STABLE_SCALE_FACTOR;  // 使用默认的稳定缩放因子
	    }

	    for (uint8_t i = DATA_START; i < END_POINT; i++) {
	        int32_t normalized;
	        int32_t scale_factor;

	        if (i >= stable_start) {
	            // 稳定阶段 - 使用根据稳定区域大小确定的缩放因子
	            scale_factor = stable_scale_factor;
	        }
	        else if (i >= transition_start && i < stable_start) {
	            // 过渡区域：线性渐变
	            float transition_ratio = (float)(i - transition_start) / (stable_start - transition_start);
	            scale_factor = (int32_t)(NORMAL_SCALE_FACTOR * (1.0f - transition_ratio) +
	                                    stable_scale_factor * transition_ratio);

	            // 确保缩放因子在合理范围内
	            if (scale_factor < stable_scale_factor) scale_factor = stable_scale_factor;
	            if (scale_factor > NORMAL_SCALE_FACTOR) scale_factor = NORMAL_SCALE_FACTOR;
	        }
	        else {
	            // 非稳定阶段
	            scale_factor = NORMAL_SCALE_FACTOR;
	        }

	        normalized = ((int32_t)occl_data[i] * scale_factor + max_value / 2) / max_value;

	        // 限制归一化后的值在合理范围内（只处理上限，不处理小于0的情况）
	        if (normalized > NORMAL_SCALE_FACTOR) {
	            // 注意：上限使用较大的那个缩放因子
	            int32_t max_scale = (NORMAL_SCALE_FACTOR > stable_scale_factor) ? NORMAL_SCALE_FACTOR : stable_scale_factor;
	            occl_data[i] = (normalized > max_scale) ? (int16_t)max_scale : (int16_t)normalized;
	        } else {
	            occl_data[i] = (int16_t)normalized;
	        }
	    }
}
#endif
#if(1)
uint8_t occl_normalize_tail_and_compare_limits(int16_t occl_data[])
{
    int16_t baseline = 0;
    uint8_t anomaly_count = 0;
    (void)baseline;
    /* 第一步: 分析数据特征，判断是否需要进行比较 */
       bool should_compare = true;

    // 1. 查找峰值位置
      uint16_t peak_index = find_peak_index(occl_data);
      bool peak_in_70_85 = (peak_index >= 70 && peak_index <= 85);

      // 2. 判断峰值是否偏后（在80左右）
      if (peak_in_70_85) {
          // 3. 分析峰值前的趋势（是否一直上升）
          bool continuously_rising = true;
          int16_t prev_value = occl_data[0];

          for (uint16_t i = 1; i < peak_index; i++) {
              if (occl_data[i] < prev_value) {
                  continuously_rising = false;
                  break;
              }
              prev_value = occl_data[i];
          }

          // 4. 分析峰值后的趋势（是否趋于稳定）
          bool stable_after_peak = true;
          if (peak_index < DATA_END - 10) {
              int16_t peak_value = occl_data[peak_index];
              int16_t stability_threshold = peak_value * 0.80f; // 峰值95%作为稳定阈值

              for (uint16_t i = peak_index + 1; i <= DATA_END; i++) {
                  if (occl_data[i] < stability_threshold) {
                      stable_after_peak = false;
                      break;
                  }
              }
          }

          // 5. 如果满足条件：峰值在80左右 + 前面一直上升 + 后面趋于稳定，则不进行比较
          if (continuously_rising && stable_after_peak) {
              should_compare = false;
          }
      }

      // 如果不需要比较，直接返回0
      if (!should_compare) {
          return 0;
      }

    // 数据预处理 + 获取最大值 (排除尾部4个点)
//    int16_t max_value = preprocess_data(occl_data, &baseline);

    /* 尾部校正处理 */
    // 查找尾部起点 (从后向前第一个≥NORMALIZATION_MAX的点)
    /* 第一步: 找到上升波段起始点，跳过初始下降波段 */
    uint16_t rise_start ;

    if (peak_in_70_85) {
           // 峰值在70-85之间时，起始位从0开始
           rise_start = 0;
       } else {
           // 其他情况，进行正常运算找到起始位
           rise_start = find_end_of_initial_decline_enhanced(occl_data);
       }


    /* 第二步: 尾部校正处理 */
       // 查找尾部起点 (从后向前第一个≥NORMALIZATION_MAX的点)
    uint8_t tail_start = DATA_END;
    // 修改后：找到最后一个≥NORMALIZATION_MAX的点
    for (int16_t i = DATA_END; i >= (int16_t)rise_start; i--) {
        if (occl_data[i] >= NORMALIZATION_MAX) {
            tail_start = i;
            break;
        }
    }

    // 计算插值分母 (防零除)
    int16_t denom = NORMALIZATION_MAX - occl_data[DATA_END];
    if (denom == 0) denom = 1;

    // 执行尾部插值
    for (uint8_t i = tail_start; i <= DATA_END; i++) {
        int32_t delta = (int32_t)NORMALIZATION_MAX * (occl_data[i] - NORMALIZATION_MAX);
        occl_data[i] = NORMALIZATION_MAX + delta / denom;

    }
    /* 第三步: 异常检测 - 通过峰值对齐进行比较 */
    // 找到当前数据的峰值位置（归一化后，峰值应为NORMALIZATION_MAX）
    uint16_t current_peak_index = find_peak_index(occl_data);
    int16_t peak_offset = current_peak_index - NORMAL_PEAK_INDEX ;
    occl_comp_peak_offset = peak_offset;

    // 获取参考数组大小
    uint16_t ref_array_size = sizeof(Normal_Flow_Max) / sizeof(Normal_Flow_Max[0]);

    if (peak_offset > 0) {
        // 峰值提前：当前数据开始得早，需要跳过前面的一些数据点
        uint16_t adjusted_rise_start = rise_start + peak_offset;  // 跳过前面的点
        uint16_t adjusted_data_end = DATA_END;

        // 确保调整后的范围有效
        if (adjusted_rise_start > DATA_END) {
            return anomaly_count;  // 没有有效数据可比较
        }

        for (uint16_t i = adjusted_rise_start; i <= adjusted_data_end; i++) {
            // 当前数据索引i对应参考模板的(i - peak_offset)
            // 因为当前数据开始得早，需要减去偏移量来对齐
            uint16_t ref_index = i - peak_offset;

            if (ref_index >= ref_array_size) break;

            if (occl_data[i] > Normal_Flow_Max[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
            else if (occl_data[i] < Normal_Flow_Min[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
        }
    }
    else if (peak_offset < 0) {
        // 峰值延后：当前数据开始得晚，参考模板需要跳过前面的一些点
        uint16_t adjusted_rise_start = rise_start;
        uint16_t adjusted_data_end = DATA_END + peak_offset;  // peak_offset为负，所以是减少

        // 确保调整后的范围有效
        if (adjusted_data_end < rise_start) {
            adjusted_data_end = rise_start;
        }
        if (adjusted_data_end > DATA_END) {
            adjusted_data_end = DATA_END;
        }

        for (uint16_t i = adjusted_rise_start; i <= adjusted_data_end; i++) {
            // 当前数据索引i对应参考模板的(i - peak_offset)
            // 因为当前数据开始得晚，参考模板需要加上偏移量来对齐（peak_offset为负）
            uint16_t ref_index = i - peak_offset;  // 减去负数等于加上绝对值

            if (ref_index >= ref_array_size) break;

            if (occl_data[i] > Normal_Flow_Max[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
            else if (occl_data[i] < Normal_Flow_Min[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
        }
    }
    else {
        // 正常情况，峰值位置正确
        for (uint16_t i = rise_start; i <= DATA_END; i++) {
            uint16_t ref_index = i - rise_start;

            if (ref_index >= ref_array_size) break;

            if (occl_data[i] > Normal_Flow_Max[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
            else if (occl_data[i] < Normal_Flow_Min[ref_index]) {
                occl_comp_count_test_value[occl_comp_count_test] = (int16_t)occl_data[i];
                occl_comp_count_test_position[occl_comp_count_test] = i;
                occl_comp_count_test++;
                anomaly_count++;
            }
        }
    }

    return anomaly_count;
}
#endif

// 初始化控制结构
/**
 * @brief 初始化动态间隔控制结构
 * @param control 动态间隔控制结构体指针
 * @detail 在每小时开始时调用，重置所有计数和时间基准
 */
void init_dynamic_interval_control(dynamic_interval_control_t *control)
{
    control->current_hour_tests = 0;
    control->normal_interval = OCCL_TEST_INTERVAL;
    control->extended_interval = 0;
    control->test_volume_ml = 6;
    control->hour_start_time = HAL_GetTick();
    control->hour_dose_completed = 0;
    control->volume_at_hour_start = G_volume;
    control->need_extend_interval = 0;
 //   control->has_pause_this_period = 0;
    control->last_enter_time = HAL_GetTick();  // 初始化最后进入时间

    control->compensation_active = 0;
    control->compensation_ml = 0;
    control->compensation_occl_time = 0;

    // ====== 新增：低速模式判断 ======
#if ENABLE_LOW_FLOW_MODE
    // 低速模式计时初始化
    if (G_low_flow_mode) {
        control->low_flow_mode_active = 1;

        // 初始化低速模式等待状态
        init_low_flow_wait_state(&control->low_flow_wait);

        // 设置低速模式等待时长（直接使用全局变量 G_food_rate 计算）
        control->low_flow_wait.wait_duration = (uint32_t)(5 - G_food_rate) * 720000;;

        // 注意：不再需要 target_volume 和 current_volume
        // 注意：hour_start_time 和 volume_at_hour_start 在上面已经初始化过了
    } else {
        // 非低速模式时，也初始化结构体（防止未初始化访问）
        init_low_flow_wait_state(&control->low_flow_wait);
    }
#endif
    // 在主循环中添加冲洗相关变量
#if ENABLE_POST_FEED_FLUSH
    // 初始化喂水冲洗状态
    control->water_flush.flushing_mode = 0;
    control->water_flush.flush_volume_ml = 25;
    control->water_flush.is_flushing = 0;
    control->water_flush.flush_completed = 0;
    control->water_flush.water_fed_this_session = 0;
#endif

    static uint8_t first_init = 1;
    if (first_init) {
        control->feeding_start_volume = G_volume;
        control->feeding_start_time = HAL_GetTick();
        first_init = 0;
    }
}
#ifdef USE_UPDATE_DYNAMIC_CONTROL
void init_pause_related_params(dynamic_interval_control_t *control)
{
    control->has_pause_this_period = 0;
    control->flow_before_pause = 0;
    control->pause_flow_total = 0;
    control->accumulated_work_time = 0;
    control->pause_timeout = 0;
}
#endif
/**
 * @brief 更新动态间隔控制状态（非第一次进入时调用）
 * @param control 动态间隔控制结构体指针
 * @param current_time 当前时间
 * @param last_enter_time_ptr 上次进入时间的指针（用于更新）
 */
void update_dynamic_interval_control(dynamic_interval_control_t *control,
                                          uint32_t current_time,
                                          uint32_t last_enter_time)
{
    // 先重新初始化所有参数
    init_dynamic_interval_control(control);

    // 再判断暂停（判断的是：从上一次进入到现在的时间间隔）
    if ((current_time - last_enter_time) < PAUSE_THRESHOLD_MS) {
        control->has_pause_this_period = 1;  // 标记暂停
    }
    else{
    	control-> pause_timeout = 1;
    }

    // 更新最后进入时间
    control->last_enter_time = current_time;
}

#ifndef DISABLE_8TH_TEST_CHECK
// 子函数1：判断是否会有第8次检测
/**************************************************************
 * will_have_8th_test()
 * @brief      判断在当前小时内是否会发生第8次堵塞检测
 * @param[in]  control         动态间隔控制结构体指针
 * @param[in]  current_time    当前系统时间(ms)
 * @param[in]  time_remaining_ms 当前小时剩余时间(ms)
 * @return     1-会发生第8次检测, 0-不会发生
 */
static uint8_t will_have_8th_test(dynamic_interval_control_t *control, uint32_t current_time, uint32_t time_remaining_ms)
{
    // 基于当前检测次数和剩余时间判断
    // 假设检测间隔是固定的8分钟(480秒)
    uint32_t time_for_one_test = control->normal_interval * 1000 -FEEDING_DURATION_MS; // 转换为毫秒

    // 如果剩余时间足够进行一次检测（包括间隔时间），则认为会有第8次检测
    if (time_remaining_ms >= time_for_one_test) {
        return 1;
    }
    return 0;
}


// 子函数2：计算检测在当前小时内的流量贡献
/**************************************************************
 * calculate_detection_volume_current()
 * @brief      计算检测在当前小时内的流量贡献
 * @param[in]  control         动态间隔控制结构体指针
 * @param[in]  test_start_time 检测开始时间(ms)
 * @param[in]  hour_start_time 当前小时开始时间(ms)
 * @return     当前小时内获得的检测流量(mL)
 */
static uint16_t calculate_detection_volume_current(dynamic_interval_control_t *control, uint32_t test_start_time, uint32_t hour_start_time)
{
    uint32_t test_end_time = test_start_time + TOTAL_DETECTION_MS;
    uint32_t hour_end_time = hour_start_time + 3600000;
    uint32_t feeding_start_time = test_start_time + (OCCL_TEST_HEAT_END * 1000);

    uint32_t feeding_in_current_hour_ms = 0;

    if (test_end_time <= hour_end_time) {
        // 检测完全在当前小时内
        feeding_in_current_hour_ms = FEEDING_DURATION_MS;
    } else if (feeding_start_time < hour_end_time) {
        // 检测跨越小时，计算当前小时内的喂液部分
        feeding_in_current_hour_ms = hour_end_time - feeding_start_time;
    }

    return (control->test_volume_ml * feeding_in_current_hour_ms) / FEEDING_DURATION_MS;
}

// 子函数3：计算预测误差
/**************************************************************
 * calculate_predicted_errors()
 * @brief      计算包含和不包含第8次检测的预测误差
 * @param[in]  current_volume      当前累计流量(mL)
 * @param[in]  predicted_expected  预期流量(mL)
 * @param[in]  volume_with_test   包含检测的预测流量(mL)
 * @param[in]  volume_without_test 不包含检测的预测流量(mL)
 * @param[out] error_with_test    包含检测的预测误差(%)
 * @param[out] error_without_test 不包含检测的预测误差(%)
 */
static void calculate_predicted_errors(uint16_t current_volume, uint16_t predicted_expected,
                                     uint16_t volume_with_test, uint16_t volume_without_test,
                                     int16_t *error_with_test, int16_t *error_without_test)
{
    if (predicted_expected > 0) {
        *error_with_test = ((int16_t)volume_with_test - (int16_t)predicted_expected) * 100 / predicted_expected;
        *error_without_test = ((int16_t)volume_without_test - (int16_t)predicted_expected) * 100 / predicted_expected;
    } else {
        *error_with_test = 0;
        *error_without_test = 0;
    }
}

// 子函数4：低流速决策逻辑
/**************************************************************
 * low_flow_decision_logic()
 * @brief      低流速(≤16mL/h)决策逻辑
 * @param[in]  will_have_8th_test 是否会有第8次检测
 * @param[in]  error_with_test    包含检测的预测误差
 * @param[in]  error_without_test 不包含检测的预测误差
 * @param[in]  time_remaining_sec 剩余时间(s)
 * @param[in]  current_volume     当前累计流量
 * @param[in]  G_food_rate        设定流速
 * @param[in]  time_remaining_ms  剩余时间(ms)
 * @param[in]  test_volume_ml     单次检测流量
 * @param[out] decision_interval  决策间隔时间(s)
 * @return     决策类型: 0-无操作, 1-加速, 2-部分延迟, 3-完全延迟
 */
static uint8_t low_flow_decision_logic(uint8_t will_have_8th_test, int16_t error_with_test,
                                     int16_t error_without_test, uint16_t time_remaining_sec,
                                     uint16_t current_volume, uint16_t G_food_rate,
                                     uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                     dynamic_interval_control_t *control, uint16_t predicted_expected,  // 添加这两个参数
                                     uint16_t *decision_interval)
{
	uint8_t decision_type = 0;
	uint16_t decision_interval_local = 0;  // 本地变量
	uint32_t current_time = HAL_GetTick();
	if (will_have_8th_test) {
        // 已有第8次检测的情况
        if (error_with_test < -2 && time_remaining_sec >= MIN_REMAINING_FOR_ACCELERATE) {
            // 需要提前检测
            *decision_interval = 4 * 60;
            decision_type = 1; // ACCELERATE
        } else if (error_with_test > 7 && time_remaining_sec >= MIN_REMAINING_FOR_DELAY) {
            // 需要延迟检测 - 调用延迟决策子函数
        	// 修正调用方式
        	decision_type = handle_delay_decision(time_remaining_sec, time_remaining_ms,
        	                                    error_with_test, error_without_test,
        	                                    current_volume, G_food_rate,
        	                                    control, predicted_expected,
        	                                    current_time,  // 第9个参数
        	                                    &decision_interval_local);  // 第10个参数

        	// 然后设置输出参数
        	*decision_interval = decision_interval_local;
        }
    } else {
        // 没有第8次检测，但可能通过提前检测来改善负误差
        if (error_without_test < -2 && time_remaining_sec >= MIN_REMAINING_FOR_ACCELERATE) {
            if (check_accelerate_effect(current_volume, G_food_rate, time_remaining_ms, test_volume_ml)) {
                *decision_interval = 4 * 60;
                decision_type = 1; // ACCELERATE
            }
        }
    }
    return decision_type; // NO_ACTION
}

// 子函数5：高流速决策逻辑
/**************************************************************
 * high_flow_decision_logic()
 * @brief      高流速(>16mL/h)决策逻辑
 * @param[in]  will_have_8th_test 是否会有第8次检测
 * @param[in]  error_with_test    包含检测的预测误差
 * @param[in]  error_without_test 不包含检测的预测误差
 * @param[in]  time_remaining_sec 剩余时间(s)
 * @param[in]  current_volume     当前累计流量
 * @param[in]  G_food_rate        设定流速
 * @param[in]  time_remaining_ms  剩余时间(ms)
 * @param[in]  test_volume_ml     单次检测流量
 * @param[out] decision_interval  决策间隔时间(s)
 * @return     决策类型: 0-无操作, 1-加速, 2-部分延迟, 3-完全延迟
 */
static uint8_t high_flow_decision_logic(uint8_t will_have_8th_test, int16_t error_with_test,
                                      int16_t error_without_test, uint16_t time_remaining_sec,
                                      uint16_t current_volume, uint16_t G_food_rate,
                                      uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                      dynamic_interval_control_t *control, uint16_t predicted_expected,  // 添加这两个参数
                                      uint16_t *decision_interval)
{
	uint8_t decision_type = 0;
	uint16_t decision_interval_local = 0;  // 本地变量
	uint32_t current_time = HAL_GetTick();
	if (will_have_8th_test) {
        // 已有第8次检测的情况
	    if (error_with_test < -2) {
	        // 直接调用延迟决策，不需要时间判断
	    	decision_type = handle_delay_decision(time_remaining_sec, time_remaining_ms,
	    	                                    error_with_test, error_without_test,
	    	                                    current_volume, G_food_rate,
	    	                                    control, predicted_expected,
	    	                                    current_time,  // 第9个参数
	    	                                    &decision_interval_local);  // 第10个参数
	        *decision_interval = decision_interval_local;
	    }
    } else {
        // 没有第8次检测，但可能通过提前检测来改善正误差
    	// 在高流速决策中调用
    	if (error_without_test > 7 && time_remaining_sec >= MIN_REMAINING_FOR_ACCELERATE) {
    	    uint16_t best_accelerate_interval;
    	    if (check_accelerate_effect_high_flow(current_volume, G_food_rate, time_remaining_ms,
    	                                        control->test_volume_ml, predicted_expected,
    	                                        &best_accelerate_interval)) {
    	        *decision_interval = best_accelerate_interval;
    	        decision_type = 1; // ACCELERATE
    	    }
    	}
    }
    return decision_type; // NO_ACTION
}

// 子函数6：检查加速检测效果（低流速）
/**************************************************************
 * check_accelerate_effect()
 * @brief      检查低流速下加速检测的效果
 * @param[in]  current_volume     当前累计流量(mL)
 * @param[in]  G_food_rate        设定流速(mL/h)
 * @param[in]  time_remaining_ms  剩余时间(ms)
 * @param[in]  test_volume_ml     单次检测流量(mL)
 * @return     1-加速效果可接受, 0-加速效果不可接受
 */
static uint8_t check_accelerate_effect(uint16_t current_volume, uint16_t G_food_rate,
                                     uint32_t time_remaining_ms, uint16_t test_volume_ml)
{
    uint32_t accelerated_normal_feeding_ms = time_remaining_ms - (4 * 60 * 1000);
    uint16_t accelerated_normal_volume = (G_food_rate * accelerated_normal_feeding_ms) / 3600000;
    uint16_t predicted_volume_accelerated = current_volume + accelerated_normal_volume + test_volume_ml;
    uint16_t predicted_expected = G_food_rate;

    int16_t predicted_error_accelerated = 0;
    if (predicted_expected > 0) {
        predicted_error_accelerated = ((int16_t)predicted_volume_accelerated - (int16_t)predicted_expected) * 100 / predicted_expected;
    }

    return (predicted_error_accelerated <= MAX_ACCELERATE_ERROR) ? 1 : 0;
}

// 子函数7：检查加速检测效果（高流速）
/**************************************************************
 * check_accelerate_effect_high_flow()
 * @brief      检查高流速下提前检测的最佳效果和间隔
 * @param[in]  current_volume     当前累计流量(mL)
 * @param[in]  G_food_rate        设定流速(mL/h)
 * @param[in]  time_remaining_ms  剩余时间(ms)
 * @param[in]  test_volume_ml     单次检测流量(mL)
 * @param[in]  predicted_expected 预期流量(mL)
 * @param[out] best_interval      最优提前间隔(秒)
 * @return     1-找到改善方案, 0-无改善方案
 */
static uint8_t check_accelerate_effect_high_flow(uint16_t current_volume, uint16_t G_food_rate,
                                               uint32_t time_remaining_ms, uint16_t test_volume_ml,
                                               uint16_t predicted_expected, uint16_t *best_interval)
{
    uint16_t time_remaining_sec = time_remaining_ms / 1000;
    int16_t current_error = ((int16_t)(current_volume + (G_food_rate * time_remaining_ms) / 3600000) -
                           (int16_t)predicted_expected) * 100 / predicted_expected;

    uint16_t optimal_interval = 4 * 60; // 默认4分钟
    int16_t best_error = current_error;
    uint8_t found_improvement = 0;

    // 搜索最佳的提前检测时间（从4分钟到最大可能时间）
    for (uint16_t accelerate_interval = 4 * 60;
         accelerate_interval <= time_remaining_sec - (TOTAL_DETECTION_MS / 1000);
         accelerate_interval += 30) {

        // 计算提前检测后的流量
        uint32_t accelerated_normal_feeding_ms = time_remaining_ms - (accelerate_interval * 1000);
        uint16_t accelerated_normal_volume = (G_food_rate * accelerated_normal_feeding_ms) / 3600000;
        uint16_t predicted_volume_accelerated = current_volume + accelerated_normal_volume + test_volume_ml;

        // 计算提前检测后的误差
        int16_t predicted_error_accelerated = 0;
        if (predicted_expected > 0) {
            predicted_error_accelerated = ((int16_t)predicted_volume_accelerated - (int16_t)predicted_expected) * 100 / predicted_expected;
        }

        // 找到使误差最接近0且不超过5%的提前时间
        if (abs(predicted_error_accelerated) < abs(best_error) && predicted_error_accelerated <= 5) {
            optimal_interval = accelerate_interval;
            best_error = predicted_error_accelerated;
            found_improvement = 1;
        }
    }

    if (found_improvement) {
        *best_interval = optimal_interval;
        return 1;
    }

    return 0;
}
/**************************************************************
 * check_partial_delay_feasible()
 * @brief      检查部分延迟方案是否可行
 */
static uint8_t check_partial_delay_feasible(uint32_t delay_step_ms, uint32_t current_time,
                                          dynamic_interval_control_t *control)
{
    uint32_t time_elapsed = current_time - control->hour_start_time;
    uint32_t delayed_test_start_ms = time_elapsed + delay_step_ms;
    uint32_t delayed_test_end_ms = delayed_test_start_ms + TOTAL_DETECTION_MS;

    return (delayed_test_end_ms <= 3600000) ? 1 : 0;
}
/**************************************************************
 * handle_low_flow_full_delay()
 * @brief      处理低流速完全延迟决策
 */
static uint8_t handle_low_flow_full_delay(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                        dynamic_interval_control_t *control,
                                        uint16_t G_food_rate, uint16_t current_volume,
                                        uint16_t *decision_interval)
{
    *decision_interval = 240; // 4分钟 = 240秒
    return 3; // FULL_DELAY
}
/**************************************************************
 * handle_low_flow_partial_delay()
 * @brief      处理低流速部分延迟决策
 */
static uint8_t handle_low_flow_partial_delay(uint32_t time_remaining_ms, uint32_t current_time,
                                           dynamic_interval_control_t *control,
                                           uint16_t current_volume, uint16_t G_food_rate,
                                           uint16_t predicted_expected, uint16_t *decision_interval)
{
    // 计算最小和最大延迟时间范围
    uint32_t min_delay_ms = 0;
    uint32_t max_delay_ms = time_remaining_ms - (MIN_REMAINING_AFTER_TEST * 1000);

    if (min_delay_ms > time_remaining_ms) {
        min_delay_ms = 0;
    }

    // 搜索部分延迟方案
    for (uint32_t delay_step_ms = min_delay_ms; delay_step_ms <= max_delay_ms; delay_step_ms += PARTIAL_DELAY_STEP_MS) {

        if (!check_partial_delay_feasible(delay_step_ms, current_time, control)) {
            continue;
        }

        int16_t predicted_error_partial = calculate_partial_delay_error(delay_step_ms, current_time,
                                                                       control, current_volume,
                                                                       G_food_rate, predicted_expected);

        // 使用固定的误差范围
        int16_t min_error = -2;
        int16_t max_error = 7;

        if (predicted_error_partial >= min_error && predicted_error_partial <= max_error) {
            *decision_interval = delay_step_ms / 1000;
            return 2; // PARTIAL_DELAY
        }
    }

    return 0;
}
/**************************************************************
 * handle_high_flow_delay()
 * @brief      处理高流速延迟决策
 */
static uint8_t handle_high_flow_delay(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                    dynamic_interval_control_t *control,
                                    uint16_t G_food_rate, uint16_t current_volume,
                                    uint16_t *decision_interval)
{
    // 高流速情况下，直接延迟到下一小时开始
    *decision_interval = time_remaining_sec;
    return 3; // FULL_DELAY
}

/**************************************************************
 * calculate_partial_delay_error()
 * @brief      计算部分延迟后的预测误差
 */
static int16_t calculate_partial_delay_error(uint32_t delay_step_ms, uint32_t current_time,
                                           dynamic_interval_control_t *control, uint16_t current_volume,
                                           uint16_t G_food_rate, uint16_t predicted_expected)
{
    uint32_t time_elapsed = current_time - control->hour_start_time;
    uint32_t delayed_test_start_ms = time_elapsed + delay_step_ms;
    uint32_t delayed_test_end_ms = delayed_test_start_ms + TOTAL_DETECTION_MS;

    // 计算检测在当前小时内的喂液分配
    uint32_t feeding_start_ms = delayed_test_start_ms + (OCCL_TEST_HEAT_END * 1000);
    uint32_t feeding_end_ms = delayed_test_end_ms;
    uint32_t feeding_in_current_hour_ms = feeding_end_ms - feeding_start_ms;
    uint16_t detection_volume_current = (control->test_volume_ml * feeding_in_current_hour_ms) / FEEDING_DURATION_MS;

    // 计算各阶段流量
    uint32_t normal_feeding_before_test_ms = delay_step_ms;
    uint16_t normal_volume_before_test = (G_food_rate * normal_feeding_before_test_ms) / 3600000;

    uint32_t normal_feeding_after_test_ms = 3600000 - delayed_test_end_ms;
    uint16_t normal_volume_after_test = (G_food_rate * normal_feeding_after_test_ms) / 3600000;

    // 计算预测总量和误差
    uint16_t predicted_volume_partial = current_volume + normal_volume_before_test +
                                      detection_volume_current + normal_volume_after_test;

    int16_t predicted_error_partial = 0;
    if (predicted_expected > 0) {
        predicted_error_partial = ((int16_t)predicted_volume_partial - (int16_t)predicted_expected) * 100 / predicted_expected;
    }

    return predicted_error_partial;
}

/**************************************************************
 * handle_delay_decision()
 * @brief      处理延迟检测决策，包括完全延迟和部分延迟
 * @details    根据当前小时误差情况，决策是否延迟第8次检测：
 *             - 完全延迟：将检测延迟到下一小时，并优化延迟时间
 *             - 部分延迟：在当前小时内调整检测开始时间
 * @param[in]  time_remaining_sec 当前小时剩余时间(秒)
 * @param[in]  time_remaining_ms  当前小时剩余时间(毫秒)
 * @param[in]  error_with_test    包含第8次检测的预测误差(%)
 * @param[in]  error_without_test 不包含第8次检测的预测误差(%)
 * @param[in]  current_volume     当前累计流量(mL)
 * @param[in]  G_food_rate        设定流速(mL/h)
 * @param[in]  control            动态间隔控制结构体指针
 * @param[in]  predicted_expected 预期流量(mL)
 * @param[out] decision_interval  决策的延迟间隔时间(秒)
 * @return     决策类型: 0-无延迟, 2-部分延迟, 3-完全延迟
 */
static uint8_t handle_delay_decision(uint16_t time_remaining_sec, uint32_t time_remaining_ms,
                                   int16_t error_with_test, int16_t error_without_test,
                                   uint16_t current_volume, uint16_t G_food_rate,
                                   dynamic_interval_control_t *control, uint16_t predicted_expected,
                                   uint32_t current_time, uint16_t *decision_interval)
{
    if (G_food_rate <= HIGH_FLOW_RATE_THRESHOLD) {
        // 低流速延迟策略
        if (error_with_test > 5) {
            if (error_without_test < -1) {
                // 有测试误差大且无测试误差为负
                return handle_low_flow_partial_delay(time_remaining_ms, current_time,
                                                   control, current_volume, G_food_rate,
                                                   predicted_expected, decision_interval);
            } else {
                // 只有测试误差大
                return handle_low_flow_full_delay(time_remaining_sec, time_remaining_ms,
                                                control, G_food_rate, current_volume, decision_interval);
            }
        }
    } else {
        // 高流速延迟策略
        if (error_with_test < -2) {
            return handle_high_flow_delay(time_remaining_sec, time_remaining_ms,
                                        control, G_food_rate, current_volume, decision_interval);
        }
    }

    return 0;
}


/**************************************************************
 * estimate_next_test_time()
 * @brief      估算下一次检测的时间
 * @param[in]  control         动态间隔控制结构体指针
 * @param[in]  current_time    当前系统时间(ms)
 * @return     下一次检测的预估开始时间(ms)
 */
static uint32_t estimate_next_test_time(dynamic_interval_control_t *control, uint32_t current_time)
{
    // 基于固定间隔估算下一次检测时间
    // 使用正常间隔作为基础
    return current_time + (control->normal_interval * 1000)- (OCCL_TEST_END * 1000);
}

/**
 * @brief 计算有堵塞检测时的正常流速
 * @details 考虑堵塞检测期间流速降低的影响，计算实际正常流动期间的流速
 *          公式：正常流速 = (设定流速 - 检测期间流量) × 3600000 / 正常时间
 * @return uint16_t 正常期间的流速 (μl/h)
 */
static uint16_t calculate_normal_flow_rate_with_occlusion(void)
{
    uint32_t normal_volume_per_hour = G_food_rate - DETECTION_VOLUME_PER_HOUR;
    return (normal_volume_per_hour * 3600000) / NORMAL_TIME_MS;
}

/**
 * @brief 计算剩余正常时间
 * @details 从总剩余时间中扣除已发生的堵塞检测时间，得到实际正常流动时间
 * @param time_remaining_ms 总剩余时间(ms)
 * @return uint32_t 剩余正常流动时间(ms)
 */
static uint32_t calculate_remaining_normal_time(uint32_t time_remaining_ms)
{
    return (time_remaining_ms > TOTAL_DETECTION_TIME_MS) ?
           (time_remaining_ms - TOTAL_DETECTION_TIME_MS) : 0;
}

/**
 * @brief 计算剩余正常流量
 * @details 基于调整后的正常流速和剩余正常时间，计算正常流动期间的流量
 * @param time_remaining_ms 总剩余时间(ms)
 * @return uint16_t 剩余正常流动期间的流量(μl)
 */
static uint16_t calculate_remaining_normal_volume(uint32_t time_remaining_ms)
{
    uint16_t normal_flow_rate = calculate_normal_flow_rate_with_occlusion();
    uint32_t remaining_normal_time_ms = calculate_remaining_normal_time(time_remaining_ms);
    return (normal_flow_rate * remaining_normal_time_ms) / 3600000;
}

/**
 * @brief 计算有堵塞检测时的总流量
 * @details 包含当前已输注量、剩余正常流量和堵塞检测期间的流量
 * @param current_volume 当前已输注量(μl)
 * @param time_remaining_ms 剩余时间(ms)
 * @param detection_volume 堵塞检测期间的流量(μl)
 * @return uint32_t 有堵塞检测时的总流量(μl)
 */
static uint32_t calculate_volume_with_occlusion_test(uint16_t current_volume,
                                                   uint32_t time_remaining_ms,
                                                   uint16_t detection_volume)
{
    uint16_t remaining_normal_volume = calculate_remaining_normal_volume(time_remaining_ms);
    return current_volume + remaining_normal_volume + detection_volume;
}

/**
 * @brief 计算无堵塞检测时的总流量
 * @details 基于设定流速和剩余时间计算总流量，不考虑堵塞检测的影响
 * @param current_volume 当前已输注量(μl)
 * @param time_remaining_ms 剩余时间(ms)
 * @return uint32_t 无堵塞检测时的总流量(μl)
 */
static uint32_t calculate_volume_without_occlusion_test(uint16_t current_volume,
                                                      uint32_t time_remaining_ms)
{
    return current_volume + (G_food_rate * time_remaining_ms) / 3600000;
}
#endif

#ifndef DISABLE_8TH_TEST_CHECK
// 主函数
/**************************************************************
 * check_8th_test_interval()
 * @brief      检查并调整第8次堵塞检测的时间间隔
 * @details    根据当前流量误差预测，动态调整第8次检测的时间：
 *             - 低流速(≤16mL/h): 负误差加速, 正误差延迟
 *             - 高流速(>16mL/h): 正误差加速, 负误差延迟
 * @param[in]  control 动态间隔控制结构体指针
 */
void check_8th_test_interval(dynamic_interval_control_t *control)
{


	if (control->current_hour_tests != 7) {
        control->need_extend_interval = 0;
        return;
    }

    uint32_t current_time = HAL_GetTick();

    uint32_t time_elapsed = current_time - control->hour_start_time;

    uint32_t time_remaining_ms = 0;
    uint16_t time_remaining_sec = 0;

    if (time_elapsed < 3600000) {
        time_remaining_ms = 3600000 - time_elapsed;
        time_remaining_sec = time_remaining_ms / 1000;
    } else {
        time_remaining_ms = 0;
        time_remaining_sec = 0;
    }

    // 判断是否会有第8次检测
    uint8_t will_have_8th = will_have_8th_test(control, current_time, time_remaining_ms);

    // 直接计算精确的当前小时流量（0.1mL单位）
    uint16_t current_volume = (G_hour_volume * 10) + ((G_one_ml_count * 10) / ONE_ML);
    uint16_t predicted_expected = G_food_rate;

    // 计算预测流量和误差
    uint16_t volume_with_test, volume_without_test;
    int16_t error_with_test, error_without_test;

    if (will_have_8th) {
          uint32_t next_test_time = estimate_next_test_time(control, current_time);
          uint16_t detection_volume = calculate_detection_volume_current(control, next_test_time, control->hour_start_time);

          volume_with_test = calculate_volume_with_occlusion_test(current_volume, time_remaining_ms, detection_volume);
          volume_without_test = calculate_volume_without_occlusion_test(current_volume, time_remaining_ms);
      } else {
          volume_with_test = calculate_volume_without_occlusion_test(current_volume, time_remaining_ms);
          volume_without_test = volume_with_test;
      }

    calculate_predicted_errors(current_volume, predicted_expected, volume_with_test, volume_without_test,
                             &error_with_test, &error_without_test);

    // 决策逻辑
    uint8_t decision_type = 0;
    uint16_t decision_interval = 0;

    if (G_food_rate <= HIGH_FLOW_RATE_THRESHOLD) {
    	// 低流速调用
    	decision_type = low_flow_decision_logic(will_have_8th, error_with_test, error_without_test,
    	                                      time_remaining_sec, current_volume, G_food_rate,
    	                                      time_remaining_ms, control->test_volume_ml,
    	                                      control, predicted_expected, &decision_interval);  // 添加这两个参数
    } else {
    	// 高流速调用
    	decision_type = high_flow_decision_logic(will_have_8th, error_with_test, error_without_test,
    	                                       time_remaining_sec, current_volume, G_food_rate,
    	                                       time_remaining_ms, control->test_volume_ml,
    	                                       control, predicted_expected, &decision_interval);  // 添加这两个参数
    }

    // 执行决策
    if (decision_type != 0) {
        control->extended_interval = decision_interval;
        control->need_extend_interval = 1;
    } else {
        control->need_extend_interval = 0;
    }
}
#endif

// 第七次检测后判断第8次间隔（修正版）
/**
 * @brief 第七次检测后智能判断第8次检测间隔
 * @param control 动态间隔控制结构体指针
 * @detail
 * - 在第七次检测完成后调用
 * - 基于预测误差和流速关系决策是否延迟第8次检测
 * - 延迟条件：负误差+高流速(>16mL/h) 或 正误差>5%+低流速(<16mL/h)
 * - 延长间隔=剩余时间，让第8次检测在下一小时进行
 */
#if(0)
void check_8th_test_interval(dynamic_interval_control_t *control)
{
    if (control->current_hour_tests != 7) {
        control->need_extend_interval = 0;
        return;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t time_elapsed = current_time - control->hour_start_time;

    // 正确处理时间边界
    uint32_t time_remaining_ms = 0;
    uint16_t time_remaining_sec = 0;

    if (time_elapsed < 3600000) {
        time_remaining_ms = 3600000 - time_elapsed;
        time_remaining_sec = time_remaining_ms / 1000;
    } else {
        // 已经超过1小时，剩余时间为0
        time_remaining_ms = 0;
        time_remaining_sec = 0;
    }

    uint16_t current_volume = G_volume - control->volume_at_hour_start;

    // 预测总量（包含第8次检测）
    uint32_t normal_feeding_ms = (time_remaining_ms > TOTAL_DETECTION_MS) ?
                                (time_remaining_ms - TOTAL_DETECTION_MS) : 0;

    uint16_t remaining_normal_volume = (G_food_rate * normal_feeding_ms) / 3600000;
    uint16_t predicted_volume_with_test = current_volume + remaining_normal_volume + control->test_volume_ml;
    uint16_t predicted_expected = G_food_rate;

    int16_t predicted_error_with_test = 0;
    if (predicted_expected > 0) {
        predicted_error_with_test = ((int16_t)predicted_volume_with_test - (int16_t)predicted_expected) * 100 / predicted_expected;
    }

    // 计算延迟检测后的预测误差（不包含第8次检测）
    uint16_t predicted_volume_without_test = current_volume + (G_food_rate * time_remaining_ms) / 3600000;
    int16_t predicted_error_without_test = 0;
    if (predicted_expected > 0) {
        predicted_error_without_test = ((int16_t)predicted_volume_without_test - (int16_t)predicted_expected) * 100 / predicted_expected;
    }

    // 决策逻辑 - 确保误差在[-2%, +7%]范围内
    uint8_t should_delay = 0;
    uint8_t should_accelerate = 0;
    uint8_t should_partial_delay = 0;
    uint16_t partial_delay_interval = 0;
    uint16_t full_delay_interval = time_remaining_sec; // 默认完全延迟时间

    // 情况1：需要加速 - 当误差太负且正常检测可以改善（需要至少4分钟）
    if (time_remaining_sec >= MIN_REMAINING_FOR_ACCELERATE) {
        if (predicted_error_with_test < -2 && predicted_error_without_test < -2) {
            // 只有加速检测能改善负误差时才加速
            if (predicted_error_with_test > predicted_error_without_test) {
                should_accelerate = 1;
            }
        }
    }

    // 情况2和3：延迟相关决策（需要至少6分钟）
    if (time_remaining_sec >= MIN_REMAINING_FOR_DELAY) {
        // 情况2：需要完全延迟 - 当正常检测会导致误差超过+7%
        if (predicted_error_with_test > 7) {
            // 只有延迟后误差在安全范围内才考虑延迟
            if (predicted_error_without_test >= -2 && predicted_error_without_test <= 7) {
                should_delay = 1;

                // 完全延迟时间优化 - 只搜索前6分钟
                uint16_t best_delay_interval = time_remaining_sec; // 默认延迟到下一小时开始

                // 计算下一小时在立即检测情况下的基础误差
                uint16_t next_hour_base_volume = G_food_rate + control->test_volume_ml;
                int16_t next_hour_base_error = ((int16_t)next_hour_base_volume - (int16_t)G_food_rate) * 100 / G_food_rate;

                // 如果立即检测就会超限，需要优化延迟时间
                if (next_hour_base_error > 7) {
                    // 预计算常用值
                    uint32_t feeding_start_offset_ms = OCCL_TEST_HEAT_END * 1000;

                    // 搜索最优延迟时间（从0开始，15秒步长，最多24次循环）
                    uint16_t loop_count = 0;
                    for (uint32_t additional_delay_sec = 0;
                         additional_delay_sec <= FULL_DELAY_SEARCH_RANGE && loop_count < MAX_FULL_DELAY_LOOPS;
                         additional_delay_sec += (FULL_DELAY_STEP_MS / 1000), loop_count++) {

                        // 计算延迟后的检测在下一小时内的喂液时间
                        uint32_t next_hour_test_start_ms = additional_delay_sec * 1000;
                        uint32_t next_hour_feeding_start_ms = next_hour_test_start_ms + feeding_start_offset_ms;
                        uint32_t next_hour_feeding_end_ms = next_hour_test_start_ms + TOTAL_DETECTION_MS;

                        // 确保检测在下一小时内完成
                        if (next_hour_feeding_end_ms > 3600000) continue;

                        // 喂液完全在下一小时内
                        uint32_t feeding_in_next_hour_ms = next_hour_feeding_end_ms - next_hour_feeding_start_ms;
                        uint16_t detection_volume_next = (control->test_volume_ml * feeding_in_next_hour_ms) / FEEDING_DURATION_MS;

                        // 关键修正：正确的下一小时总流量评估
                        // 下一小时总流量 = 正常喂液流量 + 检测额外流量
                        uint16_t next_hour_total_volume = G_food_rate + detection_volume_next;

                        // 计算下一小时的预测误差
                        int16_t next_hour_error = ((int16_t)next_hour_total_volume - (int16_t)G_food_rate) * 100 / G_food_rate;

                        // 找到在安全范围内的延迟方案
                        if (next_hour_error >= -2 && next_hour_error <= 7) {
                            best_delay_interval = time_remaining_sec + additional_delay_sec;

                            // 找到满意解就提前退出（保守策略，选择第一个可行解）
                            break;
                        }
                    }
                }

                full_delay_interval = best_delay_interval;
            }
        }
        // 情况3：需要部分延迟 - 当正常检测误差>7%但完全延迟又太负
        else if (predicted_error_with_test > 7 && predicted_error_without_test < -2) {
            // 计算最小和最大延迟时间
            uint32_t min_delay_ms = time_remaining_ms - TOTAL_DETECTION_MS;
            uint32_t max_delay_ms = time_remaining_ms - (MIN_REMAINING_AFTER_TEST * 1000);

            // 确保最小延迟不小于0
            if (min_delay_ms > time_remaining_ms) {
                min_delay_ms = 0;
            }

            // 从最小延迟开始，按照15秒梯度增加延迟
            for (uint32_t delay_step_ms = min_delay_ms; delay_step_ms <= max_delay_ms; delay_step_ms += PARTIAL_DELAY_STEP_MS) {

                // 计算延迟后的检测开始时间
                uint32_t delayed_test_start_ms = time_elapsed + delay_step_ms;

                // 计算延迟后的检测完成时间（必须在本小时内）
                uint32_t delayed_test_end_ms = delayed_test_start_ms + TOTAL_DETECTION_MS;

                // 确保检测完成时间在当前小时内（安全检查）
                if (delayed_test_end_ms > 3600000) {
                    continue; // 这个延迟方案不可行，跳过
                }

                // 计算检测在当前小时内的喂液分配
                uint32_t feeding_start_ms = delayed_test_start_ms + (OCCL_TEST_HEAT_END * 1000);
                uint32_t feeding_end_ms = delayed_test_end_ms;

                // 喂液阶段完全在当前小时内
                uint32_t feeding_in_current_hour_ms = feeding_end_ms - feeding_start_ms;
                uint16_t detection_volume_current = (control->test_volume_ml * feeding_in_current_hour_ms) / FEEDING_DURATION_MS;

                // 计算延迟期间的正常喂液流量
                uint32_t normal_feeding_before_test_ms = delay_step_ms;
                uint16_t normal_volume_before_test = (G_food_rate * normal_feeding_before_test_ms) / 3600000;

                // 计算检测后的正常喂液时间
                uint32_t normal_feeding_after_test_ms = 3600000 - delayed_test_end_ms;
                uint16_t normal_volume_after_test = (G_food_rate * normal_feeding_after_test_ms) / 3600000;

                // 计算当前小时的预测总量
                uint16_t predicted_volume_partial = current_volume +
                                                  normal_volume_before_test +
                                                  detection_volume_current +
                                                  normal_volume_after_test;

                int16_t predicted_error_partial = 0;
                if (predicted_expected > 0) {
                    predicted_error_partial = ((int16_t)predicted_volume_partial - (int16_t)predicted_expected) * 100 / predicted_expected;
                }

                // 找到使当前小时误差在安全范围内的延迟方案
                if (predicted_error_partial >= -2 && predicted_error_partial <= 7) {
                    should_partial_delay = 1;
                    partial_delay_interval = delay_step_ms / 1000;
                    break;
                }
            }
        }
    }

    // 执行决策
    if (should_accelerate) {
        // 加速检测：将间隔缩短到4分钟
        control->extended_interval = 4 * 60;
        control->need_extend_interval = 1;
    }
    else if (should_delay) {
        // 完全延迟：使用优化后的延迟时间
        control->extended_interval = full_delay_interval;
        control->need_extend_interval = 1;
    }
    else if (should_partial_delay && partial_delay_interval > 0) {
        // 部分延迟
        control->extended_interval = partial_delay_interval;
        control->need_extend_interval = 1;
    } else {
        control->need_extend_interval = 0;
    }
}

#endif
// 每小时重置
/**
 * @brief 每小时重置间隔控制
 * @param control 动态间隔控制结构体指针
 * @detail 在每小时剂量完成时调用，清零计数并更新时间基准
 */
void reset_hourly_interval_control(dynamic_interval_control_t *control)
{
    control->current_hour_tests = 0;
    control->hour_start_time = HAL_GetTick();
    control->volume_at_hour_start = G_volume;
    control->compensation_occl_time = 0;
#ifdef USE_UPDATE_DYNAMIC_CONTROL
    // 清除暂停相关状态
    init_pause_related_params(control);
#endif
    control->last_enter_time = HAL_GetTick();  // 重置最后进入时间
}
/**************************************
 * occl_normalize()
 * @detail
 * Rolling average ahead by 4, offset to zero, find max, normalize data to 600 max.
 * Count no of data outside the limits.
 */

#if(0)
uint8_t occl_normalize_tail_and_compare_limits(int16_t occl_data[])
{
int16_t occl_var;
uint16_t occl_count;
int32_t div_upper;
int16_t div_lower;

uint8_t i, tail_start;
occl_count=0;
  tail_start = 1;

  for (i=1; i<117; i++){
	if (occl_data[i] >= 600){
	  tail_start = i;
	}
  }
  div_lower = 600 - occl_data[116];


  for (i=tail_start; i<117; i++){
	div_upper = 600 *(occl_data[i] - 600);
	occl_data[i] = 600 + div_upper / div_lower;
  }

  for (i=1; i<117; i++){
	occl_var =  occl_data[i];

    if (occl_var > Normal_Flow_Max[i]) {   										//Compare TP to max limits
      occl_count++;
    }
    if (occl_var < Normal_Flow_Min[i]) {   //Compare TP to min limits
      occl_count++;
    }
  }
  return occl_count;
}
#endif
/** End occl_normalize_tail_and_compare_limits() **/
// 辅助函数：检测初始下降波段结束点
uint16_t find_end_of_initial_decline_enhanced(int16_t occl_data[]) {
    // 1. 首先检测整个序列的全局最小值
    int16_t global_min = occl_data[0];
    int16_t global_max = occl_data[0];  // 这个变量可能后面用到，但根据警告，我们只删除未使用的
    uint16_t global_min_index = 0;
    // 删除 global_max_index 的声明，因为未使用
    // uint16_t global_max_index = 0;  // 这行删除或注释掉

    for (uint16_t i = 1; i < DATA_END; i++) {
        if (occl_data[i] < global_min) {
            global_min = occl_data[i];
            global_min_index = i;
        }
        if (occl_data[i] > global_max) {
            global_max = occl_data[i];
            // global_max_index = i;  // 这行也删除或注释掉
        }
    }

    // 2. 如果全局最小值出现在序列后半部分，则不是初始下降波段
    const uint16_t third_point = DATA_END / 3;
    if (global_min_index > third_point) {
        // 全局最小值出现在序列后半部分，可能不是初始下降波段
        // 改为寻找序列前1/3部分的最小值作为初始下降波段的最低点
        global_min = occl_data[0];
        global_min_index = 0;
        for (uint16_t i = 1; i < third_point; i++) {
            if (occl_data[i] < global_min) {
                global_min = occl_data[i];
                global_min_index = i;
            }
        }
    }

    // 3. 从找到的最低点开始寻找稳定的上升起点
    uint16_t rise_start = global_min_index;
    int consecutive_rise = 0;
    const uint8_t rise_threshold = 3; // 连续3个点上升确认为上升波段开始

    // 限制搜索范围，避免在序列末尾寻找上升
    uint16_t search_limit = MIN(global_min_index + 30, DATA_END - 1);

    for (uint16_t i = global_min_index + 1; i < search_limit; i++) {
        // 使用斜率而不是简单的大小比较，减少噪声影响
        int16_t slope = occl_data[i] - occl_data[i - 1];

        if (slope > 0) { // 上升
            consecutive_rise++;
        } else if (slope < -2) { // 明显下降，重置计数
            consecutive_rise = 0;
        }
        // 小幅下降或持平不重置计数，允许小幅波动

        if (consecutive_rise >= rise_threshold) {
            rise_start = i - rise_threshold + 1;
            break;
        }
    }

    // 4. 验证找到的上升起点是否合理
    // 检查从rise_start到rise_start+10点的总体趋势是否为上升
    int32_t trend_sum = 0;
    for (uint16_t i = rise_start; i < MIN(rise_start + 10, DATA_END - 1); i++) {
        trend_sum += (occl_data[i + 1] - occl_data[i]);
    }

    // 如果总体趋势不是上升，可能需要调整
    if (trend_sum <= 0) {
        // 寻找第一个明显上升的点
        for (uint16_t i = global_min_index + 1; i < search_limit; i++) {
            if (occl_data[i] > occl_data[i - 1] + 5) { // 明显上升
                rise_start = i;
                break;
            }
        }
    }

    return rise_start;
}


uint16_t find_peak_index(int16_t occl_data[]) {
    for (uint16_t i = 0; i <= DATA_END; i++) {
        if (occl_data[i] == NORMALIZATION_MAX) {
            return i;
        }
    }
    // 如果没有找到600，则寻找最大值的位置
    int16_t max_val = occl_data[0];
    uint16_t max_index = 0;
    for (uint16_t i = 1; i <= DATA_END; i++) {
        if (occl_data[i] > max_val) {
            max_val = occl_data[i];
            max_index = i;
        }
    }
    return max_index;
}
/**************************************
 * occl_normalize_max()
 * @detail
 * Rolling average ahead by 4, offset to zero, find max, normalize data to 600 max.
 */

#if(0)
void occl_normalize_max(int16_t occl_data[])
{
int16_t occl_var,occl_offset, occl_max;
uint16_t occl_count;

uint8_t i;
occl_count=0;
occl_max=0;

  for (i=1; i<117; i++){
    occl_var = (occl_data[i]+occl_data[i+1]+occl_data[i+2]+occl_data[i+3])/4; 	//4 point rolling average
    if (i==1) occl_offset=occl_var; 	                                        //read zero offset
    occl_var = occl_var - occl_offset;										    //add offset
    if (occl_var > occl_max) occl_max = occl_var;
    occl_data[i] = occl_var;													//save data
  }																				//ignore

  for (i=1; i<117; i++){
	occl_var =  occl_data[i];
  	occl_var = (600 * occl_var)/occl_max;         								//normalize data to 600 max
  	occl_data[i]= occl_var;
   // if (occl_var > Normal_Flow_Max[i]) {   										//Compare TP to max limits
   //   occl_count++;
   // }
   // if (occl_var < Normal_Flow_Min[i]) {   //Compare TP to min limits
   //   occl_count++;
   // }
  }
  return;
}
#endif
/** End occl_normalize_max() **/

/**
 * @brief 管路压力数据预处理和异常检测
 * @param occl_data 压力数据数组(长度≥120)
 * @return 超出阈值范围的数据点数量
 */
uint8_t occl_normalize(int16_t occl_data[])
{


    int16_t baseline = 0;
    int16_t max_value = INT16_MIN;
    uint8_t anomaly_count = 0;

    // 第一阶段：数据平滑和基线校正
    for (uint8_t i = DATA_START; i <= DATA_END; i++) {
        // 4点滑动平均
        int32_t sum = 0;
        for (uint8_t j = 0; j < WINDOW_SIZE; j++) {
            sum += occl_data[i + j];
        }
        int16_t smoothed = sum / WINDOW_SIZE;

        // 设置基线(使用第一个有效点的值)
        if (i == DATA_START) {
            baseline = smoothed;
        }

        // 基线校正并查找最大值
        int16_t adjusted = smoothed - baseline;
        occl_data[i] = adjusted;

        if (adjusted > max_value) {
            max_value = adjusted;
        }
    }

    // 安全校验：防止除零错误
    if (max_value == 0) {
        return INVALID_DATA;
    }

    // 第二阶段：归一化和异常检测
    for (uint8_t i = DATA_START; i <= DATA_END; i++) {
        // 归一化处理(使用32位防止溢出)
        int32_t normalized = ((int32_t)occl_data[i] * NORMALIZATION_MAX) / max_value;
        occl_data[i] = (int16_t)normalized;

        // 异常检测
        if (normalized > Normal_Flow_Max[i] || normalized < Normal_Flow_Min[i]) {
            anomaly_count++;
        }
    }

    return anomaly_count;
}

#if(0)
/**************************************
 * occl_normalize()
 * @detail
 * Rolling average ahead by 4, offset to zero, find max, normalize data to 600 max.
 * Count no of data outside the limits.
 */
uint8_t occl_normalize(int16_t occl_data[])
{
int16_t occl_var,occl_offset, occl_max;
uint16_t occl_count;

uint8_t i;
occl_count=0;
occl_max=0;

  for (i=1; i<117; i++){
    occl_var = (occl_data[i]+occl_data[i+1]+occl_data[i+2]+occl_data[i+3])/4; 	//4 point rolling average
    if (i==1) occl_offset=occl_var; 	                                        //read zero offset
    occl_var = occl_var - occl_offset;										    //add offset
    if (occl_var > occl_max) occl_max = occl_var;
    occl_data[i] = occl_var;													//save data
  }																				//ignore

  for (i=1; i<117; i++){
	occl_var =  occl_data[i];
  	occl_var = (600 * occl_var)/occl_max;         								//normalize data to 600 max
  	occl_data[i]= occl_var;
    if (occl_var > Normal_Flow_Max[i]) {   										//Compare TP to max limits
      occl_count++;
    }
    if (occl_var < Normal_Flow_Min[i]) {   //Compare TP to min limits
      occl_count++;
    }
    i=i;
  }
  return occl_count;
}/** End occl_normalize() **/
#endif
/**************************************
 * inc_bat_time()
 * @detail
 * Increment minutes pumped with battery power
 */
void inc_bat_time(uint32_t *minutes_on_battery)
{
	static uint8_t seconds_bat;

	seconds_bat++;
	if (seconds_bat>59) {
		seconds_bat=0;
		(*minutes_on_battery)++;                 //increment minutes pumped with battery power
	}
}/** End inc_bat_time() **/

/**
 * @brief 检测压力数据中的上升/下降峰值
 * @param occl_data 已归一化的压力数据数组
 * @return 检测到的峰值总数
 */
#if(1)
uint8_t calc_occl_peaks(int16_t occl_data[])
{


    // 局部变量
    uint8_t peak_count = 0;
    uint8_t slope_direction = SLOPE_UNDEFINED;
    int16_t min_val = occl_data[START_POINT - 1];
    int16_t max_val = min_val;
    int16_t current_val;
    // 根据流速动态调整峰值检测阈值
    int16_t dynamic_peak_def = OCCL_PEAK_DEF;
    if (G_food_rate == 1) {
        dynamic_peak_def -= 30;  // 低速时降低阈值，提高灵敏度
    }
    // 主检测循环
    for (uint8_t i = START_POINT; i < END_POINT; i++) {
        current_val = occl_data[i];

        // 检测下降峰值
        if (current_val < (max_val - dynamic_peak_def)) {
            if (slope_direction != SLOPE_DOWNWARD) {
                slope_direction = SLOPE_DOWNWARD;
                min_val = max_val = current_val;
                peak_count++;
                continue;  // 跳过后续检查
            }
        }
        // 检测上升峰值
        else if (current_val > (min_val + dynamic_peak_def)) {
            if (slope_direction != SLOPE_UPWARD) {
                slope_direction = SLOPE_UPWARD;
                min_val = max_val = current_val;
                peak_count++;
                continue;  // 跳过后续检查
            }
        }

        // 更新极值
        switch (slope_direction) {
            case SLOPE_UNDEFINED:
                if (current_val < min_val) min_val = current_val;
                if (current_val > max_val) max_val = current_val;
                break;

            case SLOPE_DOWNWARD:
                if (current_val < min_val) {
                    min_val = max_val = current_val;
                }
                break;

            case SLOPE_UPWARD:
                if (current_val > max_val) {
                    min_val = max_val = current_val;
                }
                break;
        }
    }

    return peak_count;
}
#endif

/**************************************
 * calc_occl_peaks()
 * @detail
 * Find up/down peaks from OCCL data. Return total no of peaks.
 */
#if(0)
uint8_t calc_occl_peaks(int16_t occl_data[])
{
uint8_t start_point;
uint8_t end_point;
int16_t min_val, max_val, new_val;
uint8_t slope_direction,occl_peak_count;
unsigned int i;


  start_point = 40;   //place these values to defs?
  end_point=117;
  min_val = occl_data[start_point-1];
  max_val = min_val;

  occl_peak_count=0;
  slope_direction=NOT_DEF;

  for (i = start_point; i<end_point; i++){
    new_val =  occl_data[i];
    if (new_val < (max_val - OCCL_PEAK_DEF)){   	//Down peak detected
      if (slope_direction != DOWNWARD){             // if new down peak
        slope_direction = DOWNWARD;					//  increment peak count and set min/max
        min_val = new_val;
        max_val = new_val;
        occl_peak_count++;
      }
    }
    if (new_val > (min_val + OCCL_PEAK_DEF)){		//Up peak detected
      if (slope_direction != UPWARD){				// if new up peak
        slope_direction = UPWARD;					//  increment peak count and set min/max
        min_val = new_val;
        max_val = new_val;
        occl_peak_count++;
      }
    }

    if (slope_direction == NOT_DEF){				//If no peaks yet, update min/max
      if (new_val < min_val) {
        min_val = new_val;
      }
      if (new_val > max_val) {
        max_val = new_val;
      }
    }

    if (slope_direction == DOWNWARD){         		//If last peak down and slope direction down,
      if (new_val < min_val) {						// set min/max
        max_val = new_val;
        min_val = new_val;
      }
    }
    if (slope_direction == UPWARD){					//If last peak up and slope direction up,
      if (new_val > max_val) {						// set min/max
        max_val = new_val;
        min_val = new_val;
      }
    }
  }
  return occl_peak_count;
}/** End calc_occl_peaks() **/
#endif
/**************************************
 * dec_to_bcd()
 * @detail
 * convert 8 bit decimal to 8 bit bcd
 */
uint8_t dec_to_bcd(uint8_t dec_num)
{
	uint8_t bcd_num;

	bcd_num=0;
	if (dec_num >99) return (0);         //return 0 if number > 99
	while (dec_num>9){
		bcd_num = bcd_num + 0x10;
		dec_num = dec_num - 10;
	}
	bcd_num = bcd_num + dec_num;
	return (bcd_num);
}/** End dec_to_bcd() **/

/**************************************
 * bcd_to_dec()
 * @detail
 * convert 8 bit bcd to 8 bit decimal
 */
uint8_t bcd_to_dec(uint8_t bcd_num)
{
	uint8_t dec_num,tens;
	tens = bcd_num>>4;
	dec_num = (bcd_num & 0x0f)+10*tens;
	return (dec_num);
}/** End bcd_to_dec() **/

/**************************************
 * get_saved_variables()
 * @detail
 * Get program variables from EEPROM.
 */
void get_saved_variables(void)
{
	read_nv_setting_vars();
	read_nv_volume_vars();
	read_nv_acc_volume_vars();
	check_nv_vars();
	check_err_data();
}/** End get_saved_variables() **/



/**************************************
 * detect_air_vol()
 * @detail
 * Based on volume 1 in 3 ml
 * Air if 1ml detected in 3ml deliver
 *
air_rate_inc（气泡计数器的增量值）
(*air_rate_inc) = (125 * (G_food_rate))/257
设计意图：输液速率越高，气泡风险权重越大，计数器累积越快。

2. 参数设计原理
参数	数值	设计考虑
125	分子	临床经验系数，与管路截面积和超声灵敏度相关
257	分母	标准化因子，确保在典型输液速率(如5-400mL/hr)下增量值处于合理范围

执行过程：
0-1mL阶段：air_ml_count从0→1，复位air_cnt1
1-2mL阶段：从1.2mL开始检测到气泡，三个计数器开始累加，在1.5mL时，air_cnt1=30 (<100)，air_ml_count从1→3，复位air_cnt2
2-3mL阶段：继续检测气泡，在2.3mL时，air_cnt1=110 (>100) → 触发报警，系统进入HOLD_MODE
 */
uint8_t detect_air(air_test_struct *air_test)
{
	uint8_t air_detected_flag;

	air_detected_flag=0;

	if (safe_distal_tube_empty()){   //Distal tube empty (AIR)
		air_test->air_cnt1 = air_test->air_cnt1 + air_test->air_rate_inc;
		air_test->air_cnt2 = air_test->air_cnt2 + air_test->air_rate_inc;
		air_test->air_cnt3 = air_test->air_cnt3 + air_test->air_rate_inc;
		if ((air_test->air_cnt1) > ONE_ML_AIR_COUNT ){
			air_detected_flag = 100;
		}
		if ((air_test->air_cnt2) > ONE_ML_AIR_COUNT ){
			air_detected_flag = 100;
		}
		if ((air_test->air_cnt3 )> ONE_ML_AIR_COUNT ){
			air_detected_flag = 100;
		}
	}
	if ((air_test->air_ml_count) == 1){   //1mL
		air_test->air_cnt1 = 0;
		air_test->air_ml_count = 2;
	}
	if ((air_test->air_ml_count) ==3){    //2mL
		air_test->air_cnt2 = 0;
		air_test->air_ml_count = 4;
	}
	if ((air_test->air_ml_count) > 4){    //3mL
		air_test->air_cnt3 = 0;
		air_test->air_ml_count = 0;
	}
	if (air_detected_flag){
		air_usonic_detected_flag = 1;
		air_in_tubing_screen();
		return (PRIME_MODE__TASK);
	}
		return (air_detected_flag);
}/** End detect_air() **/

static void swap(int16_t *a, int16_t *b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}
