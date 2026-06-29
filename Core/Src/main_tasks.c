/*****************************************************************************
* @file		: main_tasks.c
* @short    : Functions called from main.c
******************************************************************************
*
* @details
* The main functions which are called during the program execution.
*
******************************************************************************
**/

/* **************************************************************************
*  INCLUDE FILES
*/
#include "stdint.h"
#include <stdio.h>
#include "main.h"
#include "common.h"
#include "stm32l0xx_it.h"
#include "functions.h"
#include "screens.h"
#include "screen_helper.h"
#include "main_tasks.h"
#include "io_calls.h"
#include <stdbool.h>
#include <string.h>
extern UART_HandleTypeDef huart2;



uint8_t g_first_after_bolus =0 ;
extern uint8_t G_tick5ms_counter;  //defined in main.c
extern uint8_t G_prime_completed;
extern volatile uint16_t G_motor_rate;
extern volatile uint16_t G_one_ml_count;
extern volatile uint8_t G_one_ml_flag;
extern volatile uint8_t air_usonic_detected_flag;

//#define ENABLE_TEST_CODE  // 启用测试代码

extern uint16_t G_low_flow_mode;

extern uint16_t G_alrm_mon_value;

#if ENABLE_LOW_FLOW_MODE
extern uint16_t G_food_rate;
#endif

extern uint16_t G_volume, G_hour_volume,G_water,running_volume;
extern uint16_t G_total_volume;
extern uint16_t G_water_rate;
extern uint16_t G_dose_limit;
extern uint16_t G_v_battery;
extern uint16_t G_tp_value;
extern uint16_t G_ntc_value;
extern uint16_t G_usonic_value;
extern volatile uint16_t G_ir_pd2_value;
extern uint8_t G_alarm_vol;
extern uint8_t G_warning_vol;
extern uint8_t G_controls_locked,G_battery_alarm_muted;
extern uint8_t G_display_brightness;
extern uint8_t G_dose_complete_callback_index;
extern uint8_t G_svc_entry_delay;
extern uint8_t G_tpa_type,G_cal_status;
extern uint8_t G_an_on;
extern uint8_t G_ir_heat;
extern uint32_t G_acc_formula_ml, G_acc_bolus_ml,G_acc_bat_time_min;/* 条件编译宏 */


extern uint32_t G_battery_reminder_counter ;  // 电池报警提醒计数器
extern uint8_t G_battery_reminder_state ;     // 当前提醒状态 (0-2)

// 静态变量
static uint8_t last_ultrasonic_state = 0;
static uint8_t one_second_mode_phase = 0;

static bool is_first_call = true;  // 第一次调用标志

// #define ENABLE_TEST_CODE  // 取消注释启用测试代码
//static uint16_t bolus_time_remaining = 0;
static uint16_t bolus_run_volume = 0;
// 在头文件中定义
typedef struct {
    uint16_t remaining_sec;  // 明确单位
    uint8_t status;
} BolusTimer;

BolusTimer bolus_timer = {.remaining_sec = 0};
/* ===================== 状态结构体 ===================== */


typedef struct {
	uint16_t inactivity_count;
	uint16_t tot_vol_timer;
	uint8_t settings_flag;
	uint8_t rate_blinking_flag;
	uint8_t one_second_count;
	uint8_t key_count;
	uint8_t set_test_timer;
} HoldModeState;

static dynamic_interval_control_t interval_control = {0};

// Getter函数
dynamic_interval_control_t* get_interval_control(void)
{
    return &interval_control;
}


/**************************************
 * @brief 管路状态定义
 * @note 用于管路状态机判断
 */
typedef enum {
    TUBE_STATE_FILLED = 0,      // 管路有液体状态
	TUBE_STATE_EMPTY = 1,    // 管路空状态
	TUBE_STATE_INIT  = 0xFF // 初始状态
} TubeState;

typedef struct {
    uint8_t current_state;
    uint8_t debounce_counter;
} TubeStateMachine;

typedef struct {
    uint16_t value;  // 报警监控值
    uint16_t timer;  // 计时器
    // 可扩展其他字段（如报警状态标志）
} AlarmMonitor;

// 泵状态结构体
typedef struct {
    uint8_t active;      // 泵激活状态
    uint16_t current_rate; // 当前流速
} PumpState;

// 新增加热测试状态结构体
typedef struct {
    uint8_t  active;      // 测试激活标志
    uint16_t timer;       // 加热计时器
    uint16_t temp_change; // 温度变化值
    uint16_t start_temp;  // 起始温度
} HeatTest;

/* 流量传感器校准全局变量 */
time_based_flow_cal_t G_time_cal = {
    .one_ml_corrected = ONE_ML,   // 初始值为ONE_ML
    .valid_samples = 0
};


bool check_timeout(void);

static uint8_t handle_dose_check(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                uint16_t *food_volume_increment, water_flush_state_t *wfs,dynamic_interval_control_t *interval_control);
static inline uint8_t should_flush(water_flush_state_t *wfs);

static void handle_ultrasonic_control(uint8_t one_second_count);
/* ===================== 函数声明 ===================== */
static void update_HoldMode_display(HoldModeState* s);
static uint8_t process_input(uint8_t key, HoldModeState* s);
static uint8_t  process_unlocked_input(uint8_t key, HoldModeState* s);
static uint8_t handle_up_event(uint8_t key, HoldModeState* s);
static uint8_t handle_down_event(uint8_t key, HoldModeState* s);
static void update_state_machine(HoldModeState* s) ;
//static void handle_volume_press(HoldModeState* s);
//static void check_settings_entry(HoldModeState* s);

static uint8_t process_ready_task_input(uint16_t* set_timeout);
static uint8_t is_prime_key(uint8_t key);
void stop_pump(PumpState *pump);
void update_alarm_monitor(AlarmMonitor* alarm);

void start_heat_test(HeatTest* test);
void update_heat_test(HeatTest* test);

static uint16_t get_flush_volume(void);
/* 1. 初始化函数 */
static  void init_run_mode_state(uint8_t* one_second_count, air_test_struct* air_test);
static  void start_pump_sequence(void);

/* 2. 显示控制函数 */
static void update_display(uint8_t bolus_run, uint16_t bolus_time_remaining,
		uint8_t *tot_vol_timer,uint16_t *food_volume_increment);



/* 3. 按键处理函数 */
static uint8_t handle_key_input(uint16_t* key_timer);
static uint8_t process_key_actions(uint8_t key, uint16_t* key_timer,
                          uint16_t* food_volume_increment, uint8_t* tot_vol_timer,dynamic_interval_control_t *control);
static uint8_t handle_pause(dynamic_interval_control_t *control,uint16_t* food_volume_increment);
static void record_pause_params(dynamic_interval_control_t *control,uint16_t* food_volume_increment);
static void update_accumulated_work_time(dynamic_interval_control_t *control);

/* 4. 运行模式逻辑 */
static uint8_t process_bolus_mode(uint16_t *bolus_time_remaining, uint8_t* bolus_run,
                         air_test_struct* air_test, uint16_t* occl_timer,water_flush_state_t *water_flush);
static uint8_t process_formula_mode(uint8_t *tot_vol_timer,
                                  uint16_t *food_volume_increment,
                                  air_test_struct *air_test,
                                  uint16_t *occl_timer,
								  int16_t occl_data[OCCL_DATA_SIZE],
								  dynamic_interval_control_t  *interval_control,
								  uint8_t air_result);

static uint8_t check_first_occlusion_test_start(dynamic_interval_control_t *interval_control,
                                                uint16_t *occl_timer,
                                                uint32_t *last_processed_enter_time);

static void inline update_volume_display_timer(uint8_t *tot_vol_timer);
static uint8_t handle_low_flow_wait_end(uint16_t *food_volume_increment,
                                        dynamic_interval_control_t *interval_control,
                                        uint16_t *occl_timer);
static void handle_low_flow_calculate_next_hour(uint16_t remaining_total,
                                               dynamic_interval_control_t *interval_control);
static void execute_hourly_restart(dynamic_interval_control_t *interval_control,
                                   uint16_t *occl_timer);

static void update_one_ml(dynamic_interval_control_t *control);
static uint8_t handle_normal_flow_hourly_dose(uint16_t *food_volume_increment,
                                              dynamic_interval_control_t *interval_control,
                                              uint16_t *occl_timer);
static uint8_t handle_low_flow_hourly_dose(uint16_t *food_volume_increment,
                                           dynamic_interval_control_t *interval_control,
                                           uint16_t *occl_timer);
static uint8_t check_and_process_hourly_dose(uint16_t *food_volume_increment,
                                           dynamic_interval_control_t *interval_control,
                                           uint16_t *occl_timer);
static uint8_t execute_hourly_reset_routine(dynamic_interval_control_t *interval_control,
                                       uint16_t *occl_timer);
static void reset_occlusion_detection_state(uint16_t *occl_timer);
static void start_next_hour_pump_cycle(void);
static uint8_t process_occlusion_detection(uint16_t *occl_timer,
                                          int16_t occl_data[OCCL_DATA_SIZE],
                                          air_test_struct *air_test,
                                          dynamic_interval_control_t *interval_control,
                                          uint8_t air_result);
/* 5. 传感器检测 */



/* 6. 辅助函数 */
static void control_loop_timing(uint8_t* one_second_count);

/* 7. 数据存储 */
void update_daily_food_vol(uint16_t food_volume_increment);
void SaveNV_Volume(void);

/* 8. 电机控制 */


/* 9. 测试函数（条件编译） */
#ifdef ENABLE_TEST_CODE
void send_uart_data(const char* prefix, int value);
void measure_loop_speed(uint8_t* tc, uint8_t* mintc, uint8_t* maxtc);
#endif


PincherPosition handle_pincher_control(uint8_t key, PincherPosition curr_pos);
static uint8_t handle_auto_prime(uint8_t set_type);
static uint8_t handle_prime_error(void);
static void prepare_for_tube_removal(void);
static void reset_system_after_removal(void);
static void update_prime_status(void);
static void wait_for_tube_removal(void);
static uint8_t update_tube_state(TubeStateMachine* machine, uint8_t empty);

static void handle_magnet_error(void);

//static uint8_t process_hold_mode_input(HoldModeState* state) ;
//static uint8_t check_settings_entry(HoldModeState* state);
//static HoldModeState update_hold_mode_state(HoldModeState state);


static uint8_t handle_flush_process(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                    uint16_t *food_volume_increment, water_flush_state_t *wfs);
static uint8_t handle_dose_complete_process(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                          uint16_t *food_volume_increment, water_flush_state_t *wfs);
static uint8_t handle_low_flow_dose_check(dynamic_interval_control_t *interval_control);
static uint8_t handle_normal_flow_dose_check(dynamic_interval_control_t *interval_control);
static void execute_fast_shutdown(void);
static uint8_t show_shutdown_prompts(void);
static void perform_shutdown_sequence(void);
static uint8_t check_50ms_cycle(uint8_t cycles);
static inline void wait_50ms(void);


/* 函数声明 */
// 流量传感器更新（有条件编译）
#ifdef ENABLE_FLOW_SENSOR_CALIBRATION
static void update_flow_sensor_by_time_error(dynamic_interval_control_t *control, uint32_t completion_time);
#endif

uint32_t get_corrected_one_ml(void);
/**
 * @brief Handles priming sequence for both dual and single tube sets
 * @param set_type Type of set (DUAL_TPA or SINGLE_TPA)
 * @return uint8_t Next task state (always returns HOLD_MODE__TASK)
 *
 * @details
 * 1. If distal tube is empty, performs automatic prime based on TPA type
 * 2. After auto prime or if tubing wasn't empty, performs manual prime
 * 3. Sets prime completion flag and clears counters for new sets
 * 4. Positions pincher valve for formula delivery
 */
uint8_t prime_mode_task(uint8_t set_type)
{
    /* Perform automatic priming if tube is empty */
    uint8_t prime_status = handle_auto_prime(set_type);
#if(1)
    /* Handle prime errors */
    if (prime_status == PRIME_ERR) {
        if (handle_prime_error() == TUBE_REMOVED) {
            return HOLD_MODE__TASK;
        }
    }
#endif
    /* Perform manual priming */
    manualprime();

    /* Update system state based on prime results */
    update_prime_status();
    if(g_pincher_result.position != POS_RIGHT){
    /* Prepare for normal operation */
    	move_pincher_all_ccw();
    }


    return HOLD_MODE__TASK;
}

/* Helper Functions */

/**
 * @brief Performs automatic priming based on set type
 * @param set_type Type of set (DUAL_TPA/SINGLE_TPA)
 * @return uint8_t Prime status (PRIME_OK/PRIME_ERR)
 */
static uint8_t handle_auto_prime(uint8_t set_type)
{
    if (!distal_tube_empty()) {
        return PRIME_OK;
    }
    if(air_usonic_detected_flag){
    	air_usonic_detected_flag = 0;
        return PRIME_OK;
    }

    return (set_type == DUAL_TPA) ? autoprimedual() : autoprimesingle();
}

/**
 * @brief Handles prime error condition and tube removal
 * @return uint8_t Status indicating if tube was removed (TUBE_REMOVED/TUBE_NOT_REMOVED)
 */
static uint8_t handle_prime_error(void)
{
    show_error_message(TEN_SECOND, "EMPTY BAG OR", "TUBING NOT DETECTED ");

    if (show_message_yes_no("REMOVE SET") == YES &&
        show_message_yes_no("CLOSE QUICK-CLAMPS") == YES) {

        prepare_for_tube_removal();
        wait_for_tube_removal();
        reset_system_after_removal();
        return TUBE_REMOVED;
    }

    return TUBE_NOT_REMOVED;
}

/**
 * @brief Prepares system for tube removal
 */
static void prepare_for_tube_removal(void)
{
    move_pincher_all_cw();

    move_pincher_from_all_cw_to_magnet();  // Center position for TPA removal
}

/**************************************
 * @brief 管路移除等待处理
 * @note 显示倒计时提示，10秒超时
 */
static void wait_for_tube_removal(void) {
    uint8_t seconds = 0;
    while (seconds < 10 && pts_sensor() == TUBING_DETECTED) {
        show_message("REMOVE", "SET NOW");
        if (check_50ms_cycle(20)) {
            seconds++;
        }
    }
}

/**
 * @brief Resets system state after tube removal
 */
static void reset_system_after_removal(void)
{
    G_prime_completed = 0;
    G_tpa_type = NO_TPA;
}

/**
 * @brief Updates system state based on prime results
 */
static void update_prime_status(void)
{
    if (distal_tube_empty()) {
        G_prime_completed = 0;
    } else {
        G_prime_completed = 1;
        G_one_ml_count = 0;  // Reset counters for new set
        G_one_ml_flag = 0;
    }
}

/**************************************
 * @detail
 * If distal tube is empty: perform automatic prime for dual or single set depending on TPA type.
 * After auto prime or if tubing was not empty: perform manual prime.
 * Set/Clr prime completed flag, Clear one ml count if new set
 * Rotate pincher all CCW (formula)
 *
 */

/**************************************
 * @brief 运行前检查任务
 * @return uint8_t 下一任务模式
 * @note 检查设备状态是否满足运行条件：
 *       1. 校准状态正常
 *       2. TPA安装正确
 *       3. 管路安装且非空
 */
uint8_t pre_run_check_task(void) {
	static uint8_t set_error_enter_count = 0;
    // 校准失败直接进入HOLD模式
    if (G_cal_status == CAL_ERROR) {
        show_error_message(UNTIL_CANCEL, "CALIBRATION ERROR", "RUN MODE DISABLED");
        return HOLD_MODE__TASK;
    }
    //uint8_t tube_empty_condition = distal_tube_detected() && distal_tube_empty();
    // 检查TPA和管路状态
    uint8_t ready = (G_tpa_type != NO_TPA) &&(pts_sensor() == TUBING_DETECTED);

    if (!ready) {
        set_error_enter_count++;

        if (set_error_enter_count >= 2) {
            // 再次进入这个界面，转入 TEST_IF_SET_READY__TASK
            set_error_enter_count = 0;  // 重置计数
            return TEST_IF_SET_READY__TASK;
        } else {
            // 第一次进入，显示错误并返回 HOLD_MODE
            show_error_message(TEN_SECOND, "SET ERROR ?", "");
            G_tpa_type = NO_TPA;
            G_prime_completed = 0;
            return HOLD_MODE__TASK;
        }
    }

    // 检查预充状态
    return (G_prime_completed && !distal_tube_empty()) ?
           RUN_MODE__TASK : TEST_IF_SET_READY__TASK;
}


/**************************************
 * @short		Test if ready for run mode
 * @param		void
 * @retval  	void
 *
 * @detail		Function is called before entering the RUN mode. Check if set is installed.
 *				Select action depending on the set readiness.
 */


//  End pre_run_check_task

/**************************************
 * @brief 管路准备状态任务
 * @return uint8_t 返回下一任务模式状态码
 * @note 主状态机循环，处理三种管路状态：
 *       1. 空管路（TUBE_STATE_EMPTY）：显示自动预充界面
 *       2. 非空管路（TUBE_STATE_FILLED）：显示手动预充界面
 *       3. 无管路（超时未检测到）：显示错误提示并退出
 *
 * @detail 工作流程：
 *   - 每50ms循环检测管路状态
 *   - 通过状态机实现2秒防抖检测
 *   - 处理按键输入和超时事件
 *   - 根据状态切换显示界面
 */
#if(1)
uint8_t test_if_set_ready_task(void) {
    /* 状态机初始化 */
	static TubeStateMachine tube_machine = {TUBE_STATE_INIT, 0};  // 当前状态+防抖计数器
    static uint8_t tube_state = TUBE_STATE_INIT; // 当前有效状态（防抖后）
    uint16_t timeout = 0;            // 管路未连接超时计数器
    uint16_t timeout_counter = 0;    // 按键处理超时计数器

    while (1) {
        /*---- 状态检测与界面显示 ----*/
        if (distal_tube_detected()) {
            // 更新带防抖的状态机
            tube_state = update_tube_state(&tube_machine, distal_tube_empty());

            // 根据状态显示对应界面
            tube_state == TUBE_STATE_EMPTY ?
                auto_primescreen(SET_READY, 0) :    // 空管路-自动预充safe_distal_tube_empty
                manual_primescreen(SET_READY);      // 非空管路-手动预充

            timeout = 0;  // 重置无管路超时计数器
        } else {
 /*       	// 未检测到管路的情况，判断 safe_distal_tube_empty() 是否不为空
        	    if (!safe_distal_tube_empty()) {
        	        // safe_distal_tube_empty() 不为空，说明管路内有液体，进入手动预充
        	        manual_primescreen(SET_READY);

        	    } else {
        	        // safe_distal_tube_empty() 为空，确认管路已真正移除
        	        manual_primescreen(SET_NOT_READY);  // 显示管路未就绪界面

        	        // 超时处理（SET_NOT_READY_TIMEOUT=600对应30秒）
        	        if (++timeout > SET_NOT_READY_TIMEOUT) {
        	            show_error_message(TEN_SECOND, "SET ERROR", ""); // 显示10秒错误提示
        	            return REMOVE_SET__TASK;  // 进入管路移除任务
        	        }
        	    }*/
        	 //manual_primescreen(SET_READY);
        	 manualprime();
        	 ++timeout;
        	    /* Update system state based on prime results */
        	    update_prime_status();
        	    if(g_pincher_result.position != POS_RIGHT){
        	    /* Prepare for normal operation */
        	    	move_pincher_all_ccw();
        	    }
        	    return HOLD_MODE__TASK;

        }

        /*---- 按键事件处理 ----*/
        uint8_t key = process_ready_task_input(&timeout_counter);
        switch (key) {
            case RIGHT_ARROW_RELEASED:  // 右箭头退出
            	return HOLD_MODE__TASK;
            case INACTIVITY_EXIT:       // 操作超时退出
                return HOLD_MODE__TASK; // 返回保持模式

            case TIMEOUT_ERROR:         // 管路检测超时
                return REMOVE_SET__TASK;

            default:
                // 预充按键处理（仅在管路就绪时响应）
                if (is_prime_key(key) && !timeout) {
                    return PRIME_MODE__TASK;  // 进入预充模式
                }
        }

        /*---- 周期控制 ----*/
        wait_50ms();  // 严格50ms循环延迟
    }
}

/**************************************
 * @brief 管路状态机更新函数（带防抖）
 * @param machine 状态机实例指针
 * @param empty 当前检测到的原始状态（1=空，0=非空）
 * @return uint8_t 防抖处理后的有效状态
 *
 * @detail 防抖机制：
 *   - 状态变化时启动2秒防抖计时（40*50ms）
 *   - 持续保持新状态超过2秒才确认状态切换
 *   - 防止因管路内气泡等导致的误检测
 */
static uint8_t update_tube_state(TubeStateMachine* machine, uint8_t empty) {
    if (machine->current_state == empty) {
        // 状态未变化时重置防抖计数器
        machine->debounce_counter = 0;
    } else {
        // 状态变化时递增计数器（达到40即2秒后切换状态）
        if (++machine->debounce_counter > 40) {
            machine->current_state = empty;
            machine->debounce_counter = 0;
        }
    }
    return machine->current_state;
}

/**************************************
 * @brief 处理准备任务的按键输入
 * @param set_timeout [in/out] 管路检测超时计数器指针
 * @return uint8_t 事件状态码：
 *   - KEY_CODE: 普通按键值
 *   - TIMEOUT_ERROR: 管路检测超时
 *   - INACTIVITY_EXIT: 用户操作超时
 *
 * @note 管理两个超时机制：
 *   1. 管路检测超时（set_timeout）
 *   2. 用户无操作超时（内部静态变量）
 */
static uint8_t process_ready_task_input(uint16_t* set_timeout) {
    static uint16_t exit_timeout_count = 0;  // 用户无操作计时器
    uint8_t key = read_all_keys(&exit_timeout_count);  // 读取按键（会自动重置exit_timeout_count）

    /* 管路检测超时处理 */
    if (key == NOT_PRESSED) {
        // 无按键时递增超时计数器
        if (++(*set_timeout) > SET_NOT_READY_TIMEOUT) {
            return TIMEOUT_ERROR;  // 触发管路检测超时
        }
    } else {
        *set_timeout = 0;  // 有按键时重置管路检测超时
    }

    /* 用户无操作超时处理 */
    if (exit_timeout_count > EDIT_SCREENS_TIMEOUT) {
        return INACTIVITY_EXIT;  // 默认3分钟无操作超时（EDIT_SCREENS_TIMEOUT=3600）
    }

    return key;  // 返回原始按键值
}

/**************************************
 * @brief 预充按键判断函数
 * @param key 按键值
 * @return uint8_t 1-是预充按键 / 0-不是预充按键
 *
 * @note 检测三种预充按键：
 *   - PRIME_TAG：常规预充
 *   - PRIME_TAG_WATER：水预充
 *   - PRIME_TAG_FOOD：食物预充
 */
static uint8_t is_prime_key(uint8_t key) {
    return (key == PRIME_TAG) ||
           (key == PRIME_TAG_WATER) ||
           (key == PRIME_TAG_FOOD);
}
#endif
/**************************************
 * @detail
 *If distal tube installed and distal tube empty(new set): Calibrate PD2 and show auto prime
 *If distal tube installed but tube not empty: show manual prime
 *If distal tube not detected: after timeout show error and exit to remove set
 *If right arrow or timeout: exit to hold mode
 */


// End test_if_set_ready_task
/**
 * @brief 服务页面1任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 夹爪位置控制（左/中/右）
 *       2. 泵启停控制
 *       3. 传感器数据显示
 */
uint8_t service_page_one_task(void) {
    static PincherPosition position = CAM_CENTER;
    static PumpState pump = {0};
    uint16_t timeout_counter = 0;

    do {
        // 1. 更新显示
        service_page_one_screen(position);

        // 2. 按键处理
        uint8_t key = read_all_keys(&timeout_counter);

        // 3. 无操作超时处理
        if (key == NO_TOUCH) {
            if (pump.active) stop_pump(&pump);
            if (++timeout_counter > EDIT_SCREENS_TIMEOUT) {
                stop_pump(&pump);
                return HOLD_MODE__TASK;
            }
        } else {
            timeout_counter = 0;
        }

        // 4. 导航控制
        if (key == RIGHT_ARROW_RELEASED) return HOLD_MODE__TASK;
        if (key == DOWN_ARROW_RELEASED) return SERVICE_PAGE_TWO__TASK;

        // 5. 夹爪控制
        position = handle_pincher_control(key, position);

        // 6. 泵控制
        if (key == PUMP_TAG && !pump.active) {
            start_pump(FULL_FOOD_RATE, MOTOR_INT_BOLUS);
        }

        // 7. 延时控制
        wait_50ms();

    } while(1);
}


/**
 * @brief 处理夹爪控制
 * @param key 当前按键值
 * @param curr_pos 当前位置
 * @return PincherPosition 新位置
 */
PincherPosition handle_pincher_control(uint8_t key, PincherPosition curr_pos) {
    switch(key) {
        case WATER_TAG:
            if (curr_pos == CAM_LEFT) {
                move_pincher_from_all_cw_to_magnet();
                return CAM_CENTER;
            } else {
                move_pincher_all_cw();

                return CAM_LEFT;
            }

        case FORMULA_TAG:
            if (curr_pos == CAM_RIGHT) {
                move_pincher_from_all_ccw_to_magnet(&g_pincher_result);
                return CAM_CENTER;
            } else {
                move_pincher_all_ccw();

                return CAM_RIGHT;
            }

        default:
            return curr_pos;
    }
}/**
 * @brief 停止泵
 * @param pump 泵状态结构体指针
 */
void stop_pump(PumpState *pump) {
    // 硬件控制部分
	pump_motor_off();

    // 更新状态
    pump->active = 0;
}




/**************************************
 * @detail
 * Service page one: 	Operate pincher valve
 * 						Operate pump motor
 * 						Display PTS, ULTRASONIC and PD2 values
 */


// End service_page_one_task

/**************************************
 * @brief 服务页面2任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 报警器测试
 *       2. 累计量显示
 *       3. 日志查看
 */
uint8_t service_page_two_task(void) {
    AlarmMonitor alarm = { .value = 0, .timer = 200 };

    do {
    	service_page_two_screen(
            G_acc_formula_ml / 1000,
            G_acc_bolus_ml / 1000,
            G_acc_bat_time_min / 60,
            alarm.value
        );

        // 按键处理
        uint8_t key = read_all_keys(&alarm.timer);
        if (key == UP_ARROW_RELEASED) return SERVICE_PAGE_ONE__TASK;
        if (key == DOWN_ARROW_RELEASED) return SERVICE_PAGE_THREE__TASK;
        switch (key) {
                    case PIEZO_TAG:
                        alarm.value = beep_1khz_1sec_mon(G_warning_vol);
                        alarm.timer = 0;  // 重置报警计时器
                        break;

                    case DAY_LOG_TAG:
                    	daily_volume_screen();
                        break;

                    case ERR_LOG_TAG:
                    	error_screen();
                        break;

                    case CAL_LOG_TAG:
                    	cal_rec_screen();
                        break;
        }
        // 报警监控更新
        update_alarm_monitor(&alarm);

        wait_50ms();
    } while (1);
}

// 报警监控更新模块化
void update_alarm_monitor(AlarmMonitor* alarm) {
    if (++(alarm->timer) > 200) {
        alarm->timer = 0;
        alarm->value = G_alrm_mon_value;
    }
}

/**************************************
 * detail
 * Service page two: 	Test flow sensor    TBD*******
 * 						Test Piezo alarm
 * 						Display accumulated volumes
 *						Display no of hours on battery power
 */


// End service_page_two_task
/**************************************
 * @brief 服务页面3任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 红外校准
 *       2. 加热测试
 *       3. 传感器数据显示
 */
uint8_t service_page_three_task(void) {
    // 初始化状态
    HeatTest heat = {
        .active = 0,
        .timer = 500,  // 初始化为非激活状态
        .temp_change = 0
    };
    uint16_t pd2_cal = ReadNV16(PD2_CALIBRATION_VALUE);

    do {
        // 1. 更新显示
    	service_page_three_screen(pd2_cal, heat.temp_change);

        // 2. 按键处理
        uint8_t key = read_all_keys(NULL);  // 此页面不需要超时控制
 /*
        关键区别：
        原代码强制要求传入计时器参数，即使某些页面不需要超时功能（如本页面注释掉的超时逻辑）
        优化后通过 NULL 明确表示"本页面不需要超时管理".
        Q：为什么 timeout_count=0 时不需要超时控制？
		A：因为：外层不检查：虽然 read_all_keys() 会修改 timeout_count，但外层代码注释掉了超时检查逻辑
           初始值无关：无论 timeout_count 初始设为0还是200，只要不检查它的值，就相当于禁用超时
*/
        // 3. 导航控制
        if (key == (UP_ARROW_RELEASED))
            return SERVICE_PAGE_TWO__TASK;
        if (key == (DOWN_ARROW_RELEASED))
            return SET_TIME__TASK;

        // 4. 功能按键处理
        // 在触摸事件处理代码中添加
        if (key == TOUCH_CALIBRATE_TAG ||key == RUN_STOP_SWITCH) {
            EVE_calibrate(true);  // 执行触摸校准
        }

        if (key == IR_CAL_TAG) {
        	cal_tir_sensor();
            pd2_cal = ReadNV16(PD2_CALIBRATION_VALUE);  // 刷新校准值
        }

        if (key == (IR_HEAT_TAG | RELEASED_BIT)) {
            start_heat_test(&heat);  // 封装加热测试启动逻辑
        }

        // 5. 加热测试状态机
        update_heat_test(&heat);

        // 6. 严格50ms周期控制
        wait_50ms();
    } while (1);
}

// 启动加热测试
void start_heat_test(HeatTest* test) {
    test->active = 1;
    test->timer = 0;
    test->start_temp = G_tp_value;  // 记录起始温度
    test->temp_change = 0;
    ir_heat_on();  // 开启加热器
}

// 更新加热测试状态
void update_heat_test(HeatTest* test) {
    if (!test->active) return;

    if (test->timer < 400) {
        test->timer++;
    }
    else if (test->timer == 400) {
        test->temp_change = G_tp_value - test->start_temp;
        ir_heat_off();
        test->active = 0;
        test->timer = 500;  // 重置为非激活状态
    }
}
/**************************************
 * @detail
 * Service page three: 	Display both, IR on and off, values for ULTRASONIC and PD2
 * 						Display last calibration values for PD2
 * 						Display current DAC register value (last calibration)
 *						IR current Calibration routine.
 *						Heat pulse test.
 *						Up and down arrows to move between service pages.
 */


// End service_page_three_task
/**************************************
 * @brief 水流速设置任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 水流速增减控制
 *       2. 参数保存
 */


uint8_t set_water_rate_task() {
    if (G_tpa_type == SINGLE_TPA) {
        return CLEAR_RATES__TASK; // 单TPA跳过设置
    }

    // 状态变量
	uint16_t timeout_count = 0;
	uint16_t key_delay = 0; // 长按加速计数器
/*
	key_delay 主要用于控制 当用户长按上下箭头键时，水流速的调整速度会逐渐加快。这是嵌入式设备中常见的人机交互优化策略，具体表现：
	短按：每次按键触发单次调整（基础步长）
	长按：按住按键时，调整速度会随时间推移加速（类似手机的滚轮加速效果）*/
    do {
    	// 1. 显示当前速率
    	set_water_rate_screen();

    	// 2. 读取按键（带超时计数）
    	uint8_t key = read_all_keys(&timeout_count);
    	if (key == NOT_PRESSED){
    		  key_delay=0;
    		}

        if (key == RIGHT_ARROW_RELEASED) {
            SaveNV16(G_water_rate, WATER_RATE_NV_ADDR);
            return CLEAR_RATES__TASK;
        }

        // 4. 速率调整逻辑（完全保留原始行为）
        if (key == UP_ARROW_TAG) {
        	variable_incr(key_delay, &G_water_rate);
        	key_delay = (key_delay > 250) ? 200 : (key_delay + 1);
        }
        else if (key == DOWN_ARROW_TAG) {
        	variable_decr(key_delay, &G_water_rate);
        	key_delay = (key_delay > 250) ? 200 : (key_delay + 1);
        }
        else if (key == NOT_PRESSED) {
        	key_delay = 0; // 松开按键时重置加速计数器
        }

        if (check_timeout()) {
            SaveNV16(G_water_rate, WATER_RATE_NV_ADDR);
            return HOLD_MODE__TASK;
        }

        wait_50ms();
    } while (1);
}



// 超时检测（复用全局超时机制）
bool check_timeout(void) {
    static uint16_t timeout = 0;
    if (++timeout > (EDIT_SCREENS_TIMEOUT / 2)) { // 10秒超时（假设EDIT_SCREENS_TIMEOUT=200）
        timeout = 0;
        return true;
    }
    return false;
}
/**************************************
 * @detail
 * Display and edit the bolus rate. Save to NV ram
 *
 */


//end set_water_rate_task


/**************************************
 * @brief 剂量限制设置任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 剂量限制增减控制
 *       2. 特殊组合进入服务模式
 */
uint8_t set_dose_limit_task() {
	uint16_t timeout_count = 0, key_delay = 0;
    const uint8_t svc_entry_delay_time[] = { SVC_MODE_DELAY1, SVC_MODE_DELAY2, SVC_MODE_DELAY3 };

    do {
    	set_dose_limit_screen();

        // 按键处理
        uint8_t key = read_all_keys(&timeout_count);
        if (key == NOT_PRESSED){
        	  key_delay=0;
        	}
        if (key == (RIGHT_ARROW_TAG))
        	{
            // 使用触摸计数事件控制反馈频率
            if (touch_count_event(key_delay)) {
                key_click();
            }

            key_delay++;
            if (key_delay > 250) key_delay = 200;  // 防溢出
        	}
        if (key == RIGHT_ARROW_RELEASED) {
            SaveNV16(G_dose_limit, DOSE_LIMIT_NV_ADDR);

            if (key_delay > (20 * svc_entry_delay_time[G_svc_entry_delay])){
          		  if ((G_dose_limit==13) & (G_food_rate==13))
          			  return(SERVICE_PAGE_ONE__TASK);
          	  }
          	  return (SET_WATER_RATE__TASK);
        }
        if (key == UP_ARROW_TAG)   {                   	//up arrow pressed
        	  variable_incr_dose(key_delay,&G_dose_limit);  //increment dose limit
        	  key_delay++;
        	  if (key_delay>250) key_delay=200;              //prevent overrun
        	}
    	if (key == DOWN_ARROW_TAG)   {                  //down arrow pressed
    	  variable_decr_dose(key_delay,&G_dose_limit);	//decrement dose limit
    	  key_delay++;
    	  if (key_delay>250) key_delay=200;              //prevent overrun
    	}
        if (check_timeout()) {
            SaveNV16(G_dose_limit, DOSE_LIMIT_NV_ADDR);
            return HOLD_MODE__TASK;
        }

        wait_50ms();
    } while (1);
}
/**************************************
  * @detail
 * Display and edit the dose limit. Save to NV ram.
 */


//end set_dose_limit_task

/**************************************
 * @brief 清零累计量任务
 * @return uint8_t 下一任务模式
 * @note 功能：
 *       1. 显示当前累计量
 *       2. 提供单次/总量清零功能
 *       3. 50ms循环响应按键
 */
#if USE_HOLD_MODE_NEW_SCREEN
uint8_t clear_rates_task(void) {
    uint16_t timeout_count = 0;

    do {
        // 显示清零界面
        clear_rates_screen();

        // 按键处理
        uint8_t key = read_all_keys(&timeout_count);

        /* 右键确认保存并退出 */
        if (key == (RIGHT_ARROW_PRESSED)) key_click();
        if (key == (RIGHT_ARROW_RELEASED)) {
            SaveNV_Volume();
            return HOLD_MODE__TASK;
        }

        /* 清零操作组 */
        if (key == (CLR_VOLUME_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_volume = G_hour_volume = G_water= 0;
            // 同步更新控制结构
            interval_control.volume_at_hour_start = 0;
            interval_control.feeding_start_volume = 0;
            init_pause_related_params(&interval_control);
        }
        if (key == (CLR_DOSE_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_dose_limit = 0;
        }
        if (key == (CLR_TOT_VOLUME_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_total_volume = 0;
        }

        /* 超时处理 */
        if (timeout_count++ > EDIT_SCREENS_TIMEOUT) {
            SaveNV_Volume();
            return HOLD_MODE__TASK;
        }

        // 50ms周期控制
        wait_50ms();
    } while (1);
}
#else
uint8_t clear_rates_task(void) {
    uint16_t timeout_count = 0;

    do {
        // 显示清零界面
        clear_rates_screen();

        // 按键处理
        uint8_t key = read_all_keys(&timeout_count);

        /* 右键确认保存并退出 */
        if (key == (RIGHT_ARROW_PRESSED)) key_click();
        if (key == (RIGHT_ARROW_RELEASED)) {
            SaveNV_Volume();
            return HOLD_MODE__TASK;
        }

        /* 清零操作组 */
        if (key == (CLR_VOLUME_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_volume = G_hour_volume = G_water= 0;
            // 同步更新控制结构
            interval_control.volume_at_hour_start = 0;
            interval_control.feeding_start_volume = 0;
            init_pause_related_params(&interval_control);
        }
        if (key == (CLR_DOSE_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_dose_limit = 0;
        }
        if (key == (CLR_TOT_VOLUME_TAG | PRESSED_BIT)) {
            beep_1khz_1sec(G_warning_vol);
            G_total_volume = 0;
        }

        /* 超时处理 */
        if (timeout_count++ > EDIT_SCREENS_TIMEOUT) {
            SaveNV_Volume();
            return HOLD_MODE__TASK;
        }

        // 50ms周期控制
        wait_50ms();
    } while (1);
}
#endif
/**************************************
 * clear_rates_task()
 * @detail
 * Display volume, dose limit and total volume. Display reset button for each value. Save changes to NV ram
 */

/**  End clear_rate_task()  **/
/**************************************
 * @brief 待机模式主任务
 * @return uint8_t 下一任务模式
 * @note 核心功能：
 *       1. 电源状态监控
 *       2. 速率调整
 *       3. 模式切换准备
 *       4. 60秒无操作自动锁定
 */
uint8_t hold_mode_task() {
    HoldModeState state = {
        .tot_vol_timer = NO_TOT_VOL_DISPLAY,
        .set_test_timer = SET_TEST_DELAY
    };

    while (1) {
        /*------------ 显示控制 ------------*/
    	update_HoldMode_display(&state);

        /*------------ 硬件操作 ------------*/
        power_test();  // 硬件电源检测

        /*------------ 输入处理 ------------*/
        uint8_t key = read_all_keys(&state.inactivity_count);
        uint8_t new_task = process_input(key, &state);
        if (new_task != HOLD_MODE__TASK) {
        	return new_task; // 直接返回新任务
        }

        /*------------ 状态更新 ------------*/
        update_state_machine(&state);

        /*------------ 50ms延迟控制 ------------*/
        wait_50ms();
    }

}

/* ===================== 模块化函数实现 ===================== */

/**
 * @brief 更新显示内容
 * @param s 状态结构体指针
 * @note 根据状态决定显示总流量或保持界面
 */
static void update_HoldMode_display(HoldModeState* s) {
    if (s->tot_vol_timer < NO_TOT_VOL_DISPLAY) {
        show_total_volume();
        s->set_test_timer = 0;
    } else {
#if USE_HOLD_MODE_NEW_SCREEN
    hold_mode_new_screen(s->rate_blinking_flag);
#else

    hold_mode_screen(s->rate_blinking_flag);
#endif

    }
}

/* ===================== 输入处理函数 ===================== */
static uint8_t process_input(uint8_t key, HoldModeState* s) {
    // ====== 新增：静音超时检查（每次调用都检查） ======
      static uint32_t mute_duration_counter = 0;

      if (G_battery_alarm_muted) {
          mute_duration_counter++;

          // 2分钟 = 2400个50ms周期（假设process_key_actions每50ms调用一次）
          if (mute_duration_counter >= 2200) {
              // 自动撤销静音
              G_battery_alarm_muted = 0;
              mute_duration_counter = 0;

              // 播放提示音
              key_click();

              // 重置提醒计数器
              G_battery_reminder_counter = 0;
              G_battery_reminder_state = 0;
          }
      } else {
          // 不在静音状态时重置计数器
          mute_duration_counter = 0;
      }


	/* 按键释放处理 */
    if (key == NOT_PRESSED) {
        s->key_count = 0;
        s->settings_flag = 0;
        // 进入空闲状态时自动解锁
        if (G_controls_locked) {
            G_controls_locked = 0;  // 自动解锁
        }
        return HOLD_MODE__TASK;
    }

    /* 公共按键处理 */
    s->rate_blinking_flag = 0;
    s->one_second_count = 0;
    s->set_test_timer = 0;
    /* 电源键优先处理（即使锁定也响应） */
	if (key == (POWER_SWITCH | PRESSED_BIT)) {
				return MCU_OFF__TASK;
	}
	/* 控制锁定时的特殊处理 */
	if (G_controls_locked) {
		return HOLD_MODE__TASK;; // 锁定状态下仅响应电源键
	}

	/* 正常功能处理 */
	return process_unlocked_input(key, s);
}
    /* 特殊按键处理 */
/* ===================== 非锁定状态输入处理 ===================== */
static uint8_t  process_unlocked_input(uint8_t key, HoldModeState* s) {
	uint8_t ret = HOLD_MODE__TASK;  // 默认返回值
    switch (key) {
    case (RUN_STOP_SWITCH_RELEASED):
		    ret = (s->key_count < 20) ? PRE_RUN_CHECK__TASK : HOLD_MODE__TASK;
    		break;

	case (RATE_TEXT_TAG | PRESSED_BIT):
			key_click();
			toggle_dsp_brightness();
			break;

	case (VOLUME_TEXT_TAG | RELEASED_BIT):
			if (s->key_count < 40) {
				key_click();
				s->tot_vol_timer = 0;
			}
			break;

	case (VOLUME_TEXT_TAG):
			if (++s->key_count == 40) {
				beep_1khz_1sec(G_warning_vol);
			}
			break;


			   // ====== 新增：电池静音触摸按键处理（参考BOLUS_ARROW_TAG） ======
	case  (BATTERY_MUTE_TAG | RELEASED_BIT ):  // 触摸释放时触发
			        // 切换电池报警静音状态
		   G_battery_alarm_muted = !G_battery_alarm_muted;

			        // 播放确认音
			key_click();

			        // 重置提醒计数器
			G_battery_reminder_counter = 0;
			G_battery_reminder_state = 0;

			        // 如果取消静音，立即触发一次提醒
			if (!G_battery_alarm_muted) {
			    key_click();
			    }

			        // 注意：这里不return，让程序继续运行
    		break;

	case (RIGHT_ARROW_RELEASED):
		    ret = SET_DOSE_LIMIT__TASK;
	        break;

	case  UP_ARROW_TAG:
	case  UP_ARROW_PRESSED:
	case  UP_ARROW_RELEASED:
		   ret = handle_up_event(key, s);
			break;

	case DOWN_ARROW_TAG:
		ret = handle_down_event(key, s);
			break;
	}
    return ret;
}
/* ===================== 事件处理函数 ===================== */
static uint8_t handle_up_event(uint8_t key, HoldModeState* s) {
    switch(key) {
        case UP_ARROW_PRESSED:
            stop_analog_scan();
            if (G_food_rate == FULL_FOOD_RATE) {
                key_click();
                s->settings_flag = 1;
            }
            s->key_count = 0; // 重置计数器
            return HOLD_MODE__TASK;

        case UP_ARROW_TAG: // 持续按下处理
            variable_incr_rate(s->key_count, &G_food_rate);
            if (s->key_count < 250) s->key_count++;

            // 检查是否满足进入设置的条件
            if (s->settings_flag && (s->key_count > 40)) {
                key_click(); // 准备进入设置时的反馈
            }

            G_hour_volume = 0;
            SaveNV_Volume();
            return HOLD_MODE__TASK;

        case UP_ARROW_RELEASED:
            start_analog_scan();
            if (s->settings_flag && (s->key_count > 40)) {
                s->settings_flag = 0; // 重置标志
                s->key_count = 0;
                return SETTING_SCREEN__TASK;
            }
            return HOLD_MODE__TASK;

        default:
            return HOLD_MODE__TASK;
    }
}

/* ===================== 新增下箭头处理函数 ===================== */
static uint8_t handle_down_event(uint8_t key, HoldModeState* s){
	if (key == (DOWN_ARROW_PRESSED)) {
	        stop_analog_scan();
	        return HOLD_MODE__TASK;
	    }

	    if (key == (DOWN_ARROW_RELEASED)) {
	        start_analog_scan();
	        return HOLD_MODE__TASK;
	    }

	    variable_decr_rate(s->key_count, &G_food_rate);
	    s->key_count = (s->key_count < 250) ? s->key_count + 1 : 250;
	    G_hour_volume = 0;
	    SaveNV_Volume();
	    return HOLD_MODE__TASK;
}


/* ===================== 状态机更新 ===================== */
static void update_state_machine(HoldModeState* s) {
	static uint8_t set_installed = 0;
	/* 1秒定时处理 */
    if (++s->one_second_count > ONE_SECOND) {
        s->rate_blinking_flag ^= 1;
        if (s->tot_vol_timer < NO_TOT_VOL_DISPLAY) {
            s->tot_vol_timer++;
        }
        s->one_second_count = 0;
    }

    /* 设置检测 */
    test_if_set_installed(&s->set_test_timer, &set_installed);

    /* 不活动检测 */
    inactivity_timeout(&s->inactivity_count);
}
/**************************************
 * hold_mode_task()
 * @detail
 * Function enters pump to Hold Mode. Function executes until stopped by by an
 * user or an other event. Function returns the number for the next task.
 *
 */


/**  End hold_mode_task()  **/

/**************************************
 * run_mode_task()
 * @detail
 * Function enters pump to Run Mode. Function executes until stopped by by an
 * user or an other event. Function returns the number for the next task.
 */
#if(1)
uint8_t run_mode_task()
{
    // 系统状态变量
    uint8_t ret, key, power_status;
    uint16_t key_timer = 0;
    uint8_t tot_vol_timer = NO_TOT_VOL_DISPLAY;
    uint8_t one_second_count = 0;
    uint8_t bolus_run = 0;
    uint16_t food_volume_increment = 0;


    // 闭塞测试相关
    uint16_t occl_timer = OCCL_TEST_START_COUNT;
    g_occl_state = OCCL_STATE_IDLE;

    // 验证重置结果
    if (g_occl_state != OCCL_STATE_IDLE || occl_timer != OCCL_TEST_START_COUNT) {
        #ifdef DEBUG_ERROR
        printf("ERROR: State reset failed - state=%d, timer=%d\n",
               g_occl_state, *occl_timer);
        #endif
        // 强制修正
        g_occl_state = OCCL_STATE_IDLE;
        occl_timer = OCCL_TEST_START_COUNT;
    }

    uint8_t occlusion_confirmation_count = 0;
    uint8_t occlusion_confirmation_pending = 0;
    int16_t occl_data[OCCL_DATA_SIZE];
    memset(occl_data, 0, sizeof(occl_data));




    // 空气检测相关
    air_test_struct air_test;
    // 添加 current_time 声明

#ifdef ENABLE_TEST_CODE
    // 测试相关变量
    uint8_t tc, mintc = 100, maxtc = 0;
    char str[16];
    int size_len;
#endif

    // ==== 在任务开始时配置一次 ====
    static uint8_t mode_initialized = 0;
    if (!mode_initialized) {
        usonic_mode_command = DEFAULT_CYCLE_MODE;
        mode_initialized = 1;
    }
    // 初始化系统状态
    init_run_mode_state(&one_second_count, &air_test);

     // 改用结构体中的变量
    water_flush_state_t *wfs = &interval_control.water_flush;

#ifdef USE_UPDATE_DYNAMIC_CONTROL
    // 第一次初始化
    static uint8_t control_initialized = 0;
    uint32_t current_time = HAL_GetTick();
    if (!control_initialized) {
        init_dynamic_interval_control(&interval_control);
        init_pause_related_params(&interval_control);
        interval_control.last_enter_time = current_time;
        control_initialized = 1;
    } else {
        // 后续使用更新函数
        update_dynamic_interval_control(&interval_control, current_time,
                                       interval_control.last_enter_time);
        if(interval_control.accumulated_work_time >= PAUSE_THRESHOLD_MS){

        	init_pause_related_params(&interval_control);

        }
    }

    update_one_ml(&interval_control);
#else
    // 默认：每次都重新初始化
    init_dynamic_interval_control(&interval_control);
#endif


#ifdef   TEST_Compensation_Volume
    interval_control.current_hour_tests = 7;

#endif
    // 计算推注时间
    uint16_t bolus_time = (3600 * G_water_rate) / BOLUS_RATE;

    // 启动泵
    start_pump_sequence();

    // 根据系统状态决定是否开启超声检测
    usonic_mode_command = USONIC_MODE_1S_CYCLE;

#ifdef ENABLE_TEST_CODE
    // 测试代码：发送初始食物速率到UART
    size_len = sprintf(str, "%s%d\r\n", "F*,", G_food_rate);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, size_len, HAL_MAX_DELAY);
#endif

    // 主循环
    while (1) {
        // 更新显示
        update_display(bolus_run, bolus_timer.remaining_sec, &tot_vol_timer,&food_volume_increment);
        /* 所有模式通用剂量检查 */
        uint8_t dose_check_result = handle_dose_check(&bolus_run, &bolus_timer,
                                                     &food_volume_increment, wfs,&interval_control);
        if (dose_check_result != CONTINUE_RUNNING) {
            return dose_check_result;
        }
        // 处理按键输入
        key = handle_key_input(&key_timer);
        if (process_key_actions(key, &key_timer, &food_volume_increment, &tot_vol_timer,&interval_control) != CONTINUE_RUNNING) {
            return HOLD_MODE__TASK;
        }

        // 更新体积计数
        increment_volumes(bolus_run, &air_test.air_ml_count, &bolus_run_volume, &food_volume_increment,&interval_control);
        // 开启超声逻辑
        handle_ultrasonic_control(one_second_count);
        // 每秒处理
        if (one_second_count >= ONE_SECOND) {
            one_second_count = 0;

#ifdef ENABLE_TEST_CODE
            // 测试代码：发送传感器数据到UART
            size_len = sprintf(str, "%d%s%d%s%d\r\n", G_tp_value, ",", G_ir_pd2_value, ",", G_usonic_value);
            HAL_UART_Transmit(&huart2, (uint8_t *)str, size_len, HAL_MAX_DELAY);

            // 测试代码：测量循环速度
            tc = G_tick5ms_counter;
            if (tc > maxtc) maxtc = tc;
            if (tc < mintc) mintc = tc;
            tc = 0;
#endif

            // 1秒模式：在计数重置时切换相位
            if (usonic_mode_command == USONIC_MODE_1S_CYCLE) {
                one_second_mode_phase = !one_second_mode_phase;
            }

            // 电池时间计数
            if (!ac_connected()) {
                inc_bat_time(&G_acc_bat_time_min);
            }

            // 推注模式处理
            if (bolus_run) {
                if (!process_bolus_mode(&bolus_timer.remaining_sec, &bolus_run, &air_test, &occl_timer,&interval_control.water_flush)) {
                    continue;
                }
            }
            // 配方模式处理
            else {

            	uint8_t formula_ret = process_formula_mode(&tot_vol_timer, &food_volume_increment,&air_test,
            	                                        &occl_timer, occl_data,&interval_control,ret);

            	switch(formula_ret) {
            	    case REQUEST_BOLUS_SWITCH:

            	        bolus_run = swap_formula_to_bolus();
            	        bolus_timer.remaining_sec = bolus_time; ;
            	        reset_air_test_var(&air_test);
            	        air_test.air_rate_inc = 260;  // 保持原始值
            	        break;

            	    case OCCLUSION_ALARM:
            	    	 if (!occlusion_confirmation_pending) {
            	    	            // 第一次进入堵塞报警，开始确认流程
            	    	            occlusion_confirmation_pending = 1;
            	    	            occlusion_confirmation_count = 0;
            	    	        }
            	    	        break;

            	    case CONTINUE_RUNNING:

            	    default:
            	        if (occlusion_confirmation_pending) {
            	            // 在确认过程中，进行空气检测
            	            uint32_t now = HAL_GetTick();

            	            // 开机 1 分钟内，忽略堵塞确认（不清除 pending，只是跳过检测）
            	            if ((now - interval_control.hour_start_time) < 60000) {
            	                // 什么都不做，直接跳出继续运行
            	                break;
            	            }
            	            uint8_t air_result = detect_air(&air_test);

            	            if (air_result == HOLD_MODE__TASK) {
            	                   // 检测到空气，堵塞计数也加1
            	                   occlusion_confirmation_count++;

            	                   // 检查是否达到堵塞确认阈值
            	                   if (occlusion_confirmation_count >= 2) {
            	                       // 连续两次检测（空气或非空都算）达到阈值，同时触发堵塞报警和气泡报警
            	                	   air_occlusion_in_line_screen();    // 堵塞报警
            	                               // 气泡报警

            	                       occlusion_confirmation_pending = 0;
            	                       occlusion_confirmation_count = 0;
            	                       usonic_mode_command = USONIC_MODE_DEFAULT;

            	                       return HOLD_MODE__TASK;
            	                   }
            	            }else {
            	                // 检测到非空，计数加1
            	                occlusion_confirmation_count++;

            	                if (occlusion_confirmation_count >= 2) {
            	                    // 连续两次检测都非空，确认为真实堵塞
            	                    occlusion_in_line_screen();
            	                    usonic_mode_command = USONIC_MODE_DEFAULT;
            	                    occlusion_confirmation_pending = 0;
            	                    occlusion_confirmation_count = 0;
            	                    return HOLD_MODE__TASK;
            	                }
            	            }
            	        }
            	        if (formula_ret == CONTINUE_RUNNING) {
            	            break;  // 或者 return CONTINUE_RUNNING;
            	        }
            	}

            }

            // 远端管路检测
            if (distal_tube_removed(&occl_timer)) {
            	usonic_mode_command = USONIC_MODE_DEFAULT;  // 发生异常，切回默认模式
                return HOLD_MODE__TASK;
            }

            // 空气检测（在关闭超声前完成检测）
             if (usonic_ready_for_detection) {
                 ret = detect_air(&air_test);
                 if (ret == PRIME_MODE__TASK) {
                     usonic_mode_command = USONIC_MODE_DEFAULT;
                     return PRIME_MODE__TASK;
                 }

                 // 检测完成后关闭超声（0.5秒模式和1秒模式的关闭相位）
                 if ((usonic_mode_command == USONIC_MODE_0P5S_CYCLE) ||
                     (usonic_mode_command == USONIC_MODE_1S_CYCLE && one_second_mode_phase == 1)) {
                     if (last_ultrasonic_state != 0) {
                         usonic_command = USONIC_CMD_TURN_OFF;
                         last_ultrasonic_state = 0;
                     }
                 }
             }

            // 电源状态检测
            if(g_occl_state != OCCL_STATE_HEATING && g_occl_state != OCCL_STATE_COLLECTING && (G_hour_volume < G_food_rate)){
            	power_status = power_test();
            	if (power_status == AC_RESTORED) {
            		usonic_mode_command = USONIC_MODE_DEFAULT;  // 发生异常，切回默认模式
            		return HOLD_MODE__TASK;
            	}
            }
        }

        // 循环计时控制
        control_loop_timing(&one_second_count);
    }
}

/**
 * @brief 初始化低速模式等待状态
 * @param wait_state 低速模式等待状态指针
 */
void init_low_flow_wait_state(LowFlowWaitState_t *wait_state)
{
    wait_state->is_waiting = 0;
    wait_state->wait_start_time = 0;
    wait_state->wait_duration = 0;
}



#if(0)
/***************************** 子函数实现 *****************************/

/**
 * @brief 处理剂量检查逻辑
 * @param bolus_run 推注模式标志
 * @param bolus_timer 推注重时器
 * @param food_volume_increment 食物体积增量
 * @param wfs 冲洗状态
 * @return 任务返回码，CONTINUE_RUNNING表示继续运行
 */
static uint8_t handle_dose_check(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                uint16_t *food_volume_increment, water_flush_state_t *wfs,dynamic_interval_control_t *interval_control)
{
    if (G_dose_limit != 0 && G_volume >= G_dose_limit) {

#if ENABLE_LOW_FLOW_MODE
        // ====== 低速模式特殊处理 ======
        if (G_low_flow_mode) {
            // 1. 检查是否已经在等待状态
            if (interval_control->low_flow_wait.is_waiting) {
                return CONTINUE_RUNNING;  // 等待 check_and_process_hourly_dose 处理
            }

            // 2. 剂量小于流速：单小时内完成，设置等待
            if (G_dose_limit < G_food_rate) {
                uint32_t current_ms = HAL_GetTick();
                float total_time_hours = (float)G_dose_limit / G_food_rate;
                uint32_t total_time_ms = (uint32_t)(total_time_hours * 3600000);
                uint32_t wait_time_ms = 3600000 - total_time_ms;

                interval_control->low_flow_wait.is_waiting = 1;
                interval_control->low_flow_wait.wait_start_time = current_ms;
                interval_control->low_flow_wait.wait_duration = wait_time_ms;

                return CONTINUE_RUNNING;
            }

            // 3. 剂量大于等于流速：需要检查小时剂量是否完成
            if (interval_control->hour_dose_completed == 0) {
                return CONTINUE_RUNNING;  // 等待小时剂量完成
            }
            // 小时剂量已完成，清除标志
            interval_control->hour_dose_completed = 0;

            // 低速模式通过检查，执行剂量完成处理
            // 注意：这里不返回，继续执行下面的剂量完成处理
        } else
#endif
        {
            // ====== 非低速模式处理 ======
            // 非低速模式：剂量小于流速时直接完成，否则需要检查小时剂量
            if (G_dose_limit >= G_food_rate) {
                // 需要小时剂量完成判断
                if (interval_control->hour_dose_completed == 0) {
                    return CONTINUE_RUNNING;  // 等待小时剂量完成
                }
                // 小时剂量已完成，清除标志
                interval_control->hour_dose_completed = 0;
            }
            // 非低速模式通过检查，继续执行下面的剂量完成处理
        }
        if (should_flush(wfs)) {
            #if ENABLE_POST_FEED_FLUSH
            if (!wfs->flushing_mode) {
                // 第一次达到剂量，启动冲洗模式
                wfs->flushing_mode = 1;
                wfs->is_flushing = 1;
                wfs->flush_completed = 0;

                // 保存当前喂食完成信息
                update_daily_food_vol(*food_volume_increment);

                // 设置冲洗参数
                wfs->flush_volume_ml = get_flush_volume();

                // 切换到冲洗推注模式
                *bolus_run = swap_formula_to_bolus();
                bolus_timer->remaining_sec = (wfs->flush_volume_ml * 3600) / BOLUS_RATE;
                bolus_run_volume = 0;

                return CONTINUE_RUNNING_WITH_CONTINUE;
            } else if (*bolus_run == 0 && wfs->flush_completed == 1) {
                // 冲洗完成
                wfs->flushing_mode = 0;
                wfs->is_flushing = 0;
                *food_volume_increment = 0;
                return DOSE_COMPLETE_ALARM__TASK;
            }
            #else
            // 冲洗功能被编译禁用
            update_daily_food_vol(*food_volume_increment);
            *food_volume_increment = 0;
            return DOSE_COMPLETE_ALARM__TASK;
            #endif
        } else {
            // 不满足冲洗条件
            update_daily_food_vol(*food_volume_increment);
            *food_volume_increment = 0;
            return DOSE_COMPLETE_ALARM__TASK;
        }
    }
    return CONTINUE_RUNNING;
}

/**
 * @brief 处理剂量完成后的冲洗流程
 * @param bolus_run 推注模式标志
 * @param bolus_timer 推注计时器
 * @param food_volume_increment 食物体积增量
 * @param wfs 冲洗状态
 * @return 任务返回码
 */
#endif
static uint8_t handle_flush_process(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                    uint16_t *food_volume_increment, water_flush_state_t *wfs)
{
    if (!wfs->flushing_mode) {
        // 第一次达到剂量，启动冲洗模式
        wfs->flushing_mode = 1;
        wfs->is_flushing = 1;
        wfs->flush_completed = 0;

        // 保存当前喂食完成信息
        update_daily_food_vol(*food_volume_increment);

        // 设置冲洗参数
        wfs->flush_volume_ml = get_flush_volume();

        // 切换到冲洗推注模式
        *bolus_run = swap_formula_to_bolus();
        bolus_timer->remaining_sec = (wfs->flush_volume_ml * 3600) / BOLUS_RATE;
        bolus_run_volume = 0;

        return CONTINUE_RUNNING;
    } else if (*bolus_run == 0 && wfs->flush_completed == 1) {
        // 冲洗完成
        wfs->flushing_mode = 0;
        wfs->is_flushing = 0;
        *food_volume_increment = 0;
        return DOSE_COMPLETE_ALARM__TASK;
    }

    return CONTINUE_RUNNING;
}

/**
 * @brief 处理剂量完成后的通用流程
 * @param bolus_run 推注模式标志
 * @param bolus_timer 推注计时器
 * @param food_volume_increment 食物体积增量
 * @param wfs 冲洗状态
 * @return 任务返回码
 */
static uint8_t handle_dose_complete_process(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                          uint16_t *food_volume_increment, water_flush_state_t *wfs)
{
    if (should_flush(wfs)) {
        #if ENABLE_POST_FEED_FLUSH
        return handle_flush_process(bolus_run, bolus_timer, food_volume_increment, wfs);
        #else
        // 冲洗功能被编译禁用
        update_daily_food_vol(*food_volume_increment);
        *food_volume_increment = 0;
        return DOSE_COMPLETE_ALARM__TASK;
        #endif
    } else {
        // 不满足冲洗条件
        update_daily_food_vol(*food_volume_increment);
        *food_volume_increment = 0;
        return DOSE_COMPLETE_ALARM__TASK;
    }
}

/**
 * @brief 处理低速模式剂量检查的特殊逻辑
 * @param interval_control 间隔控制结构体
 * @return 是否需要继续等待
 */
static uint8_t handle_low_flow_dose_check(dynamic_interval_control_t *interval_control)
{
    // 1. 首先检查冲洗是否已完成 - 这是最高优先级
    if (interval_control->water_flush.flush_completed == 1) {
        return 0;  // 冲洗已完成，可以继续（不管小时剂量标志是什么状态）
    }

    // 2. 然后检查小时剂量是否已完成
    if (interval_control->hour_dose_completed == 1) {
        interval_control->hour_dose_completed = 0;  // 清除标志
        return 0;  // 小时剂量已完成
    }

    return 1;  // 需要继续等待
}

/**
 * @brief 处理非低速模式剂量检查的特殊逻辑
 * @param interval_control 间隔控制结构体
 * @return 是否需要继续等待
 */
static uint8_t handle_normal_flow_dose_check(dynamic_interval_control_t *interval_control)
{
	static uint8_t wait_hour_dose_counter = 0;  // 小时剂量等待计数器
	// 非低速模式：剂量小于流速时直接完成，否则需要检查小时剂量
    // 1. 首先检查冲洗是否已完成 - 这是最高优先级
    if (interval_control->water_flush.flush_completed == 1) {
        return 0;  // 冲洗已完成，可以继续（不管小时剂量标志是什么状态）
    }

    if (G_dose_limit >= G_food_rate) {
        // 需要小时剂量完成判断
        if (interval_control->hour_dose_completed == 0) {
            // 增加等待计数
            wait_hour_dose_counter++;
            // 检查是否超时（50次 × 50ms = 2500ms = 2.5秒）
            if (wait_hour_dose_counter >= 50) {
                // 超时，跳过等待，直接返回继续处理
                wait_hour_dose_counter = 0;  // 重置计数器
                interval_control->hour_dose_completed = 0;  // 清除标志
                return 0;  // 超时，不再等待
            }
            return 1;  // 需要等待小时剂量完成
        }
        // 小时剂量已完成，清除标志
        interval_control->hour_dose_completed = 0;
    }

    return 0;  // 不需要等待，可以继续处理
}

/**
 * @brief 处理剂量检查逻辑
 * @param bolus_run 推注模式标志
 * @param bolus_timer 推注计时器
 * @param food_volume_increment 食物体积增量
 * @param wfs 冲洗状态
 * @param interval_control 间隔控制结构体
 * @return 任务返回码，CONTINUE_RUNNING表示继续运行
 */
static uint8_t handle_dose_check(uint8_t *bolus_run, BolusTimer *bolus_timer,
                                uint16_t *food_volume_increment, water_flush_state_t *wfs,
                                dynamic_interval_control_t *interval_control)
{
    // 检查总剂量是否完成
    if (G_dose_limit != 0 && G_volume >= G_dose_limit) {

        uint8_t need_wait = 0;  // 是否需要继续等待

#if ENABLE_LOW_FLOW_MODE
        // ====== 低速模式特殊处理 ======
        if (G_low_flow_mode) {
            need_wait = handle_low_flow_dose_check(interval_control);
        } else
#endif
        {
            // ====== 非低速模式处理 ======
            need_wait = handle_normal_flow_dose_check(interval_control);
        }
        if(interval_control->volume_at_hour_start >= G_dose_limit)
        {
        	need_wait = 0;
        }
        // 如果还需要等待，返回 CONTINUE_RUNNING
        if (need_wait) {
            return CONTINUE_RUNNING;
        }

        // ====== 剂量完成后的统一处理（冲洗等） ======
        return handle_dose_complete_process(bolus_run, bolus_timer, food_volume_increment, wfs);
    }

    return CONTINUE_RUNNING;
}

/**
 * @brief 处理超声开启逻辑
 * @param one_second_count 秒计数器
 */
static void handle_ultrasonic_control(uint8_t one_second_count)
{
    if (one_second_count == 10) {
        // 计数到10时开启超声
        if ((usonic_mode_command == USONIC_MODE_0P5S_CYCLE) ||
            (usonic_mode_command == USONIC_MODE_1S_CYCLE && one_second_mode_phase == 0)) {
            if (last_ultrasonic_state != 1) {
                usonic_command = USONIC_CMD_TURN_ON;
                last_ultrasonic_state = 1;
            }
        }
    }
}

/**
 * @brief 检查是否应该开始冲洗
 * @return 1: 应该冲洗, 0: 不应该冲洗
 */
static inline uint8_t should_flush(water_flush_state_t *wfs)
{
    // 条件1：有水可用于冲洗
    if (G_water_rate == 0) {
        return 0;  // 没有水，不能冲洗
    }

    // 条件2：本次喂食过程中有水喂养
    if (wfs->water_fed_this_session == 0) {
        return 0;  // 没有水喂养，不能冲洗
    }

    // 条件3：冲洗还未完成
    if (wfs->flush_completed) {
        return 0;  // 已经冲洗过了，不再冲洗
    }

    // 条件4：完成至少一次喂食周期
    if (G_volume <= G_food_rate) {
        return 0;  // 未完成1小时喂食，不能冲洗
    }

    return 1;  // 可以冲洗
}

/**
 * 初始化运行模式状态
 */
static void init_run_mode_state(uint8_t *one_second_count, air_test_struct *air_test)
{
    *one_second_count = 0;
    // 只有在没有暂停状态时才清零 G_one_ml_count 和 G_one_ml_flag
    if (!interval_control.has_pause_this_period) {
        G_one_ml_count = 0;
        G_one_ml_flag = 0;
    }
    G_tick5ms_counter = 0;
    reset_air_test_var(air_test);

#if ENABLE_LOW_FLOW_MODE
    // 支持低速模式
    if (G_food_rate < 5) {
        G_low_flow_mode = 1;
        G_motor_rate = 1;  // 5-4=1
        air_test->air_rate_inc = (125 * 5) / 257;  // 按5ml/min计算
    } else {
        G_low_flow_mode = 0;
        G_motor_rate = G_food_rate - 4;
        air_test->air_rate_inc = (125 * G_food_rate) / 257;
    }
#else
    // 不支持低速模式：强制最小为5
    if (G_food_rate < 5) {
        G_food_rate = 5;  // 自动修正
        SaveNV16(5, FOOD_RATE_NV_ADDR);  // 保存修正值
    }
    G_low_flow_mode = 0;
    G_motor_rate = G_food_rate - 4;
    air_test->air_rate_inc = (125 * G_food_rate) / 257;
#endif
}

/**
 * 启动泵序列
 */
static void start_pump_sequence(void)
{
    move_pincher_all_ccw();                         // 关闭推注管路
	if(running_volume > G_volume){
		running_volume = G_volume;
	}
    start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);    // 以食物速率启动泵
}

/**
 * 更新显示
 */
static void update_display(uint8_t bolus_run, uint16_t bolus_time_remaining, uint8_t *tot_vol_timer,uint16_t *food_volume_increment)
{
    if (bolus_run) {
        show_bolus_time(bolus_time_remaining);      // 推注时间屏幕
        *tot_vol_timer = NO_TOT_VOL_DISPLAY;       // 防止显示总体积
    } else {
        if (*tot_vol_timer < NO_TOT_VOL_DISPLAY) {
            show_total_volume();                    // 显示体积屏幕
        } else {

#if USE_HOLD_MODE_NEW_SCREEN
        	run_mode_new_screen();
#else
        	run_mode_screen();
#endif
//                                  // 运行模式屏幕

        }
    }
}


/**
 * 处理按键输入
 */
static uint8_t handle_key_input(uint16_t *key_timer)
{
    // 读取所有按键输入（键盘优先，然后是触摸按钮）
    // 递增key_timer，如果没有按键则重置key_timer
    return read_all_keys(key_timer);
}

/**
 * 处理按键动作
 */
static uint8_t process_key_actions(uint8_t key, uint16_t *key_timer,
                                  uint16_t *food_volume_increment, uint8_t *tot_vol_timer,dynamic_interval_control_t *control)
{
    // 长按RUN/STOP开关处理
    static uint16_t run_stop_hold_counter = 0;
    static uint8_t run_stop_active = 0;  // 标记RUN/STOP键是否处于激活状态
    // ====== 新增：静音超时检查（每次调用都检查） ======
      static uint32_t mute_duration_counter = 0;

      if (G_battery_alarm_muted) {
          mute_duration_counter++;

          // 2分钟 = 2400个50ms周期（假设process_key_actions每50ms调用一次）
          if (mute_duration_counter >= 1866) {
              // 自动撤销静音
              G_battery_alarm_muted = 0;
              mute_duration_counter = 0;

              // 播放提示音
              key_click();

              // 重置提醒计数器
              G_battery_reminder_counter = 0;
              G_battery_reminder_state = 0;
          }
      } else {
          // 不在静音状态时重置计数器
          mute_duration_counter = 0;
      }

    // ====== 新增：电池静音触摸按键处理（参考BOLUS_ARROW_TAG） ======
    if (key == (BATTERY_MUTE_TAG | RELEASED_BIT)) {  // 触摸释放时触发
        // 切换电池报警静音状态
        G_battery_alarm_muted = !G_battery_alarm_muted;

        // 播放确认音
        key_click();

        // 重置提醒计数器
        G_battery_reminder_counter = 0;
        G_battery_reminder_state = 0;

        // 如果取消静音，立即触发一次提醒
        if (!G_battery_alarm_muted) {
        	key_click();
        }

        // 注意：这里不return，让程序继续运行
    }

    // ====== 电源键处理（只处理释放关机） ======
    if (key == (POWER_SWITCH_RELEASED)) {
        // 电源键短按关机
        uint8_t result = handle_pause(control,food_volume_increment);
        run_stop_hold_counter = 0;
        run_stop_active = 0;
        return result;
    }

    // RUN/STOP开关处理 - 按下检测
    if (key == RUN_STOP_SWITCH) {
        // 按键持续按住
        if (run_stop_active == 0) {
            // 第一次检测到按键，初始化
            run_stop_active = 1;
            run_stop_hold_counter = 1;
        } else {
            // 持续按住，递增计数器
            if (run_stop_hold_counter < 1000) {
                run_stop_hold_counter++;
            }

            // 长按触发
            if (run_stop_hold_counter == 60) {
                G_controls_locked = controls_locked_screen(G_controls_locked);
            }
        }
    }
    // RUN/STOP开关处理 - 释放检测
    else if (key == (RUN_STOP_SWITCH_RELEASED)) {
        // 按键释放
        if (run_stop_active && run_stop_hold_counter > 0 && run_stop_hold_counter < 20) {
            // 短按
            uint8_t result = handle_pause(control,food_volume_increment);
            run_stop_hold_counter = 0;
            run_stop_active = 0;
            return result;
        }

        // 长按或无效状态
        run_stop_hold_counter = 0;
        run_stop_active = 0;
    }

    // 推注箭头标签处理
    if (key == (BOLUS_ARROW_TAG | RELEASED_BIT)) {
        return handle_pause(control,food_volume_increment);
    }

    // 未锁定时处理触摸输入
    if (!G_controls_locked) {
        if (key == (RATE_TEXT_TAG | PRESSED_BIT)) {
            key_click();
            toggle_dsp_brightness();
        }
        if (key == (VOLUME_TEXT_TAG | RELEASED_BIT)) {
            key_click();
            *tot_vol_timer = 0;  // 重置计时器以显示总体积
        }
    }

    return CONTINUE_RUNNING;
}

/**
 * 处理推注模式
 */
static uint8_t process_bolus_mode(uint16_t *bolus_time_remaining, uint8_t *bolus_run,
                                 air_test_struct *air_test, uint16_t *occl_timer,water_flush_state_t *water_flush)
{
    if (*bolus_time_remaining > 0) {
        (*bolus_time_remaining)--;
        return 1;
    }

    // 推注时间结束

    #if ENABLE_POST_FEED_FLUSH
    if (water_flush->is_flushing) {
        // ==== 冲洗模式结束处理 ====
        #ifdef DEBUG_FLUSH
        printf("[FLUSH] Flush completed\n");
        #endif

        // 冲洗模式：直接结束推注
        *bolus_run = 0;
        water_flush->flush_completed = 1;

        // 冲洗模式不需要bolus_run_volume、空气检测等重置
        // 因为整个任务即将结束
        return 0;
    }
    #endif

    // ==== 正常推注模式结束处理 ====
    // 切换回配方模式
    *bolus_run = swap_bolus_to_formula();
    bolus_run_volume = 0;
    reset_air_test_var(air_test);

    // 状态和计时器同步验证
    g_occl_state = OCCL_STATE_IDLE;
    *occl_timer = OCCL_TEST_START_COUNT;

    // 验证重置结果
    if (g_occl_state != OCCL_STATE_IDLE || *occl_timer != OCCL_TEST_START_COUNT) {
        #ifdef DEBUG_ERROR
        printf("ERROR: State reset failed - state=%d, timer=%d\n",
               g_occl_state, *occl_timer);
        #endif
        // 强制修正
        g_occl_state = OCCL_STATE_IDLE;
        *occl_timer = OCCL_TEST_START_COUNT;
    }

    // 设置首次切换标志
    g_first_after_bolus = 1;
    air_test->air_rate_inc = (125 * G_food_rate) / 257;
    water_flush->water_fed_this_session = 1;

    return 0;
}

#if ENABLE_POST_FEED_FLUSH
/**
 * @brief 获取冲洗体积配置
 * @return 冲洗体积（ml）
 */
static uint16_t get_flush_volume(void)
{
    // 可以从配置获取，这里使用默认值
    #ifdef CONFIG_FLUSH_VOLUME
    return CONFIG_FLUSH_VOLUME;
    #else
    return 25;  // 默认25ml
    #endif
}
#endif


/**
 * @brief 处理配方模式逻辑（保留原始行为优化版）
 *
 * 本函数负责配方模式下的主要逻辑处理，包括：
 * 1. 第一个小时堵塞检测启动检查
 * 2. 体积显示计时处理
 * 3. 每小时剂量检查与处理
 * 4. 闭塞检测处理
 *
 * @param tot_vol_timer 总体积显示计时器
 *        - 类型: uint8_t* (指针)
 *        - 范围: 0 ~ NO_TOT_VOL_DISPLAY
 *        - 功能: 控制总体积显示的时间，达到阈值后停止显示
 *        - 单位: 循环计数
 *
 * @param food_volume_increment 食物体积增量
 *        - 类型: uint16_t* (指针)
 *        - 功能: 记录当前小时内累计的食物体积增量
 *        - 重置: 每小时剂量达标后重置为0
 *        - 单位: 体积单位 (如mL)
 *
 * @param air_test 空气检测结构体指针
 *        - 类型: air_test_struct*
 *        - 功能: 包含空气检测相关数据，用于闭塞检测计算
 *        - 重要字段: air_rate_inc (空气率增量)
 *
 * @param occl_timer 闭塞检测计时器
 *        - 类型: uint16_t* (指针)
 *        - 范围: 0 ~ OCCL_TEST_INTERVAL (通常为600，即10分钟)
 *        - 功能: 控制闭塞检测的执行频率
 *        - 重置条件: 每小时剂量达标或检测到闭塞
 *
 * @param occl_data 闭塞检测数据缓存
 *        - 类型: int16_t[OCCL_DATA_SIZE] (数组)
 *        - 大小: OCCL_DATA_SIZE (通常为32或64)
 *        - 功能: 存储闭塞检测的原始数据样本
 *        - 生命周期: 在闭塞检测函数中填充和读取
 *
 * @param interval_control 动态间隔控制结构体指针
 *        - 类型: dynamic_interval_control_t*
 *        - 功能: 控制测试间隔和状态管理
 *        - 重要字段:
 *          - last_enter_time: 上次进入时间戳 (ms)
 *          - current_hour_tests: 当前小时测试次数
 *          - need_extend_interval: 是否需要延长间隔标志
 *          - extended_interval: 延长间隔值 (ms)
 *
 * @param air_result 空气检测结果
 *        - 类型: uint8_t
 *        - 取值: 通常为AIR_OK(0)或AIR_ERROR(1)
 *        - 功能: 传递给闭塞检测函数，影响检测逻辑
 *
 * @return uint8_t 返回状态码:
 *        - CONTINUE_RUNNING (0): 正常继续运行
 *        - REQUEST_BOLUS_SWITCH: 请求切换到大剂量模式
 *        - OCCLUSION_ALARM: 检测到闭塞，触发报警
 *        - 其他错误状态码 (根据系统定义)
 *
 * @note 重要说明:
 *       1. 第一个小时堵塞检测的特殊处理:
 *          - 在进入配方模式后，预期在240秒 + 加热20秒 + 10秒余量内完成第一次检测
 *          - 如果检测未启动，会强制启动第一次堵塞检测
 *
 *       2. 每小时剂量处理流程:
 *          - 当G_hour_volume >= G_food_rate时触发
 *          - 更新每日食物体积统计
 *          - 重置相关计数器和状态
 *          - 检查是否需要切换到大剂量模式
 *
 *       3. 状态重置机制:
 *          - 每小时剂量达标后，会重置闭塞检测状态和计时器
 *          - 重置泵马达状态和流速传感器
 *          - 更新动态间隔控制
 *
 *       4. 双TPA模式特殊处理:
 *          - 当G_tpa_type == DUAL_TPA且G_water_rate > 0时
 *          - 不执行常规重置流程，直接返回REQUEST_BOLUS_SWITCH
 *          - 由调用者处理模式切换
 *
 * @warning 注意事项:
 *        - 函数内部使用静态变量last_processed_enter_time跟踪处理状态
 *        - 调用者需确保传入的参数指针有效
 *        - 函数可能修改全局变量G_hour_volume、G_motor_rate等
 *        - 在双TPA模式下，不会执行每小时的重置流程
 *
 * @see occl_test() - 闭塞检测核心函数
 * @see update_daily_food_vol() - 每日食物体积更新函数
 * @see start_pump() - 泵启动函数
 * @see save_nv() - 非易失性存储保存函数
 */
static uint8_t process_formula_mode(uint8_t *tot_vol_timer,
                                  uint16_t *food_volume_increment,
                                  air_test_struct *air_test,
                                  uint16_t *occl_timer,
                                  int16_t occl_data[OCCL_DATA_SIZE],
                                  dynamic_interval_control_t *interval_control,
                                  uint8_t air_result)
{
    uint8_t ret_status = CONTINUE_RUNNING;
    static uint32_t last_processed_enter_time = 0;

    // 检查暂停标志，如果为真则重置 last_processed_enter_time
    if (interval_control-> pause_timeout) {
        last_processed_enter_time = 0;
    }

    if ( interval_control->water_flush.flush_completed == 1)
    {
    	return CONTINUE_RUNNING;
    }

    /* 1. 第一个小时堵塞检测启动检查 */
    ret_status = check_first_occlusion_test_start(interval_control, occl_timer,
                                                 &last_processed_enter_time);
    if (ret_status != CONTINUE_RUNNING) return ret_status;

    /* 2. 体积显示计时处理 */
    update_volume_display_timer(tot_vol_timer);

    /* 3. 每小时剂量检查与处理 */
    ret_status = check_and_process_hourly_dose(food_volume_increment,
                                              interval_control, occl_timer);
    if (ret_status != CONTINUE_RUNNING) return ret_status;

    /* 4. 闭塞检测处理（低速模式下等待时暂停） */
    #if ENABLE_LOW_FLOW_MODE
        // 如果是低速模式且正在等待，跳过闭塞检测
        if (G_low_flow_mode && interval_control->low_flow_wait.is_waiting) {
            // 等待期间暂停闭塞检测
            // 可以重置或保持闭塞检测状态不变
            // reset_occlusion_detection_state(occl_timer); // 可选：重置状态
        	return ret_status;
        } else {
            // 正常执行闭塞检测
            ret_status = process_occlusion_detection(occl_timer, occl_data,
                                                    air_test, interval_control, air_result);
        }
    #else
        // 未启用低速模式，正常执行闭塞检测
        ret_status = process_occlusion_detection(occl_timer, occl_data,
                                                air_test, interval_control, air_result);
    #endif
       if( USE_SPEED_INTERVAL ==1){
    	   return ret_status;
       }
       else{
    	   return ret_status;
       }
}

/************************* 子函数定义 *************************/

/**
 * @brief 检查并处理第一个小时堵塞检测启动
 *
 * 检查是否需要在第一个小时强制启动堵塞检测。当系统进入配方模式后，
 * 如果在预期时间内没有完成第一次堵塞检测，会强制启动检测。
 *
 * @param interval_control 动态间隔控制结构体指针
 *        - 类型: dynamic_interval_control_t*
 *        - 输入/输出: 既用于读取状态，也用于设置延长间隔标志
 *        - 关键字段:
 *          - last_enter_time: 记录配方模式开始时间
 *          - current_hour_tests: 当前小时已完成的测试次数
 *          - need_extend_interval: 输出标志，指示是否需要延长间隔
 *          - extended_interval: 输出值，延长时间(ms)
 *
 * @param occl_timer 闭塞检测计时器指针
 *        - 类型: uint16_t*
 *        - 输出: 如果检测未启动，会被设置为OCCL_TEST_INTERVAL + 1
 *        - 作用: 强制启动闭塞检测的计数
 *
 * @param last_processed_enter_time 上次已处理的进入时间指针
 *        - 类型: uint32_t*
 *        - 输出: 更新为当前interval_control->last_enter_time值
 *        - 作用: 避免重复处理相同的进入时间
 *
 * @return uint8_t 返回状态码:
 *        - CONTINUE_RUNNING (0): 正常，继续执行后续逻辑
 *        - 注意: 本函数总是返回CONTINUE_RUNNING，错误通过修改参数处理
 *
 * @note 特殊逻辑:
 *       1. 只在进入时间变化时执行检查（避免每循环都检查）
 *       2. 检查时间窗口: 进入后240秒(等待)+20秒(加热)+10秒(余量)=270秒
 *       3. 如果时间已到但检测计数为0，强制启动检测
 *       4. 设置延长间隔标志，为后续处理提供依据
 *
 * @see process_formula_mode() - 调用者
 * @see occl_test() - 后续调用的检测函数
 */
static uint8_t check_first_occlusion_test_start(dynamic_interval_control_t *interval_control,
                                                uint16_t *occl_timer,
                                                uint32_t *last_processed_enter_time)
{
    // 如果已经处理过这次进入时间，直接返回
    if (*last_processed_enter_time == interval_control->last_enter_time) {
        return CONTINUE_RUNNING;
    }

    // 静态累加器，每秒进入一次，每30次（30秒）执行一次检查
    static uint8_t check_counter = 0;

    check_counter++;
    if (check_counter < 30) {
        return CONTINUE_RUNNING;
    }
    check_counter = 0;  // 重置计数器

    // 每30秒执行一次下面的代码
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - interval_control->last_enter_time;

        // 判断是否达到了应该完成第一次检测的时间
        if (elapsed_time >= FIRST_TEST_EXPECTED_COMPLETION) {
            // 时间已经到了，需要进行处理

            // 检查是否需要强制启动检测
            if ((g_occl_state == OCCL_STATE_IDLE) && (interval_control->current_hour_tests == 0)) {
                // 应该已经完成检测，但检测计数还是0，说明检测没启动
                // 强制启动第一次堵塞检测
                g_occl_state = OCCL_STATE_IDLE;
                *occl_timer = OCCL_TEST_START_COUNT;

                #ifdef DEBUG_FIRST_TEST_FIX
                printf("First test should have completed by now, forcing start\n");
                #endif
            }

            // 无论是否强制启动检测，都要记录已处理，避免重复进入
            *last_processed_enter_time = interval_control->last_enter_time;
            interval_control->need_extend_interval = 1;
            interval_control->extended_interval = OCCL_TEST_HEAT_END;
        }

    return CONTINUE_RUNNING;

   }

/**
 * @brief 更新体积显示计时器
 *
 * 递增总体积显示计时器，控制屏幕上总体积信息的显示时间。
 * 当计时器达到阈值后，停止显示总体积信息。
 *
 * @param tot_vol_timer 总体积显示计时器指针
 *        - 类型: uint8_t*
 *        - 输入/输出: 读取当前值并递增
 *        - 范围: 0 ~ NO_TOT_VOL_DISPLAY (通常为某个阈值，如10)
 *        - 递增条件: 当前值小于NO_TOT_VOL_DISPLAY
 *        - 作用: 控制显示持续时间，避免屏幕信息长时间占用
 *
 * @note 设计说明:
 *       1. 这是一个简单的递增计数器
 *       2. 达到阈值后停止递增，避免溢出
 *       3. 调用者负责在适当时机重置计时器
 *       4. 通常与屏幕刷新周期同步
 *
 * @see process_formula_mode() - 调用者
 * @see NO_TOT_VOL_DISPLAY - 相关常量定义
 */
static void inline update_volume_display_timer(uint8_t *tot_vol_timer)
{
    if (*tot_vol_timer < NO_TOT_VOL_DISPLAY) {
        (*tot_vol_timer)++;
    }
}

/**
 * @brief 检查并处理每小时剂量
 *
 * 检查当前小时的食物体积是否达到预设速率，如果达到则:
 * 1. 更新每日食物体积统计
 * 2. 重置相关计数器和状态
 * 3. 检查是否需要切换到双TPA大剂量模式
 *
 * @param food_volume_increment 食物体积增量指针
 *        - 类型: uint16_t*
 *        - 输入: 当前小时累计的增量值
 *        - 输出: 达标后重置为0
 *        - 作用: 记录自上次重置以来的体积变化
 *
 * @param interval_control 动态间隔控制结构体指针
 *        - 类型: dynamic_interval_control_t*
 *        - 输出: 用于传递给execute_hourly_reset_routine
 *        - 作用: 包含时间控制和测试状态信息
 *
 * @param occl_timer 闭塞检测计时器指针
 *        - 类型: uint16_t*
 *        - 输出: 用于传递给execute_hourly_reset_routine
 *        - 作用: 每小时重置闭塞检测状态
 *
 * @return uint8_t 返回状态码:
 *        - CONTINUE_RUNNING: 未达到剂量或常规重置完成
 *        - REQUEST_BOLUS_SWITCH: 需要切换到双TPA大剂量模式
 *
 * @note 剂量判断逻辑:
 *       判断条件: G_hour_volume >= G_food_rate
 *       G_hour_volume: 全局变量，当前小时累计体积
 *       G_food_rate: 全局变量，预设的食物输送速率
 *
 * @note 双TPA模式特殊处理:
 *       当G_tpa_type == DUAL_TPA且G_water_rate > 0时
 *       不执行常规重置，直接返回REQUEST_BOLUS_SWITCH
 *       由调用者处理模式切换
 *
 * @see execute_hourly_reset_routine() - 被调用执行重置
 * @see update_daily_food_vol() - 被调用更新每日统计
 */
#if(0)
static uint8_t check_and_process_hourly_dose(uint16_t *food_volume_increment,
                                           dynamic_interval_control_t *interval_control,
                                           uint16_t *occl_timer)
{
    uint8_t ret_status = CONTINUE_RUNNING;

#if ENABLE_LOW_FLOW_MODE
    // ====== 处理低速模式等待状态 ======
    if (G_low_flow_mode && interval_control->low_flow_wait.is_waiting) {
        uint32_t current_ms = HAL_GetTick();
        uint32_t elapsed_wait_time = current_ms - interval_control->low_flow_wait.wait_start_time;

        // 检查等待时间是否结束
        if (elapsed_wait_time >= interval_control->low_flow_wait.wait_duration) {
            // 等待时间到了，结束等待状态
            interval_control->low_flow_wait.is_waiting = 0;

            // 更新每日食物体积
            update_daily_food_vol(*food_volume_increment);
            *food_volume_increment = 0;

            // ====== 关键：只计算和设置，不返回剂量状态 ======
            // 检查是否还有剩余剂量需要输送
            if (G_dose_limit > 0) {
                uint16_t remaining_total = G_dose_limit - G_volume;

                if (remaining_total > 0) {
                    // 还有剩余剂量，计算下一个小时计划
                    // 双TPA切换判断 - 只有在还有剩余剂量时才检查
                    if ((G_tpa_type == DUAL_TPA) && (G_water_rate > 0)) {
                        ret_status = REQUEST_BOLUS_SWITCH;
                        // 注意：这里不返回，继续执行后续的计算逻辑
                    }

                    if (remaining_total < G_food_rate) {
                        // 剩余剂量小于流速（如：剩余1ml，流速2ml）
                        // 需要重新计算下个小时的等待时间

                        // 下个小时剂量 = 剩余剂量
                        uint16_t next_hour_dose = remaining_total;

                        // 计算运行时间 = 剩余剂量 ÷ 流速
                        float run_time_hours = (float)next_hour_dose / G_food_rate;
                        uint32_t run_time_ms = (uint32_t)(run_time_hours * 3600000);
                        uint32_t wait_time_ms = 3600000 - run_time_ms;

                        // 保存重新计算的等待时间
                        interval_control->low_flow_wait.wait_duration = wait_time_ms;

                    } else {
                        // 需要设置标志，避免 handle_dose_check 先执行
                        interval_control->hour_dose_completed = 1;
                    }
                }
            } else {
                // 没有剂量限制，正常重启
           //     interval_control->low_flow_wait.wait_duration = 0;  // 满流速，不等待
                G_hour_volume = 0;
                execute_hourly_reset_routine(interval_control, occl_timer);
                start_next_hour_pump_cycle();
                save_nv();
            }
        }

        return ret_status; ;  // 总是返回 CONTINUE_RUNNING
    }
#endif

    // ====== 非低速模式：检查小时剂量 ======
    if (G_hour_volume >= G_food_rate) {
        // 只有当总剂量刚好同时完成时才需要设置标志
        if (G_dose_limit > 0 && (G_dose_limit - G_volume) == 0) {
            // 总剂量刚好完成，设置标志让 handle_dose_check 知道
            interval_control->hour_dose_completed = 1;
        } else {
            // 还有剩余剂量或没有剂量限制，检查双TPA切换
            if ((G_tpa_type == DUAL_TPA) && (G_water_rate > 0)) {
                ret_status = REQUEST_BOLUS_SWITCH;
            }
        }
        G_hour_volume = 0;
        execute_hourly_reset_routine(interval_control, occl_timer);
        start_next_hour_pump_cycle();
        save_nv();
    }

    return ret_status;
}
#endif
/**
 * @brief 处理低速模式下等待时间结束后的逻辑
 * @param food_volume_increment 食物体积增量
 * @param interval_control 间隔控制结构体
 * @param occl_timer 堵塞计时器
 * @return 返回状态码
 */
static uint8_t handle_low_flow_wait_end(uint16_t *food_volume_increment,
                                        dynamic_interval_control_t *interval_control,
                                        uint16_t *occl_timer)
{
    uint8_t ret_status = CONTINUE_RUNNING;

    // 等待时间到了，结束等待状态
    interval_control->low_flow_wait.is_waiting = 0;

    // 更新每日食物体积
    update_daily_food_vol(*food_volume_increment);
    *food_volume_increment = 0;

    // 检查是否还有剩余剂量需要输送
    if (G_dose_limit > 0) {
        uint16_t remaining_total = G_dose_limit - G_volume;

        if (remaining_total > 0) {
            // 还有剩余剂量，计算下一个小时计划
            // 双TPA切换判断 - 只有在还有剩余剂量时才检查
            if ((G_tpa_type == DUAL_TPA) && (G_water_rate > 0)) {
                ret_status = REQUEST_BOLUS_SWITCH;
            }

            if (remaining_total < G_food_rate) {
                // 剩余剂量小于流速，重新计算下个小时的等待时间
                handle_low_flow_calculate_next_hour(remaining_total, interval_control);
            }

            execute_hourly_restart(interval_control, occl_timer);
        }
        else{
            // 需要设置标志，避免 handle_dose_check 先执行
        interval_control->hour_dose_completed = 1;
        G_hour_volume = 0;
        save_nv();
        }
    } else {
        // 没有剂量限制，正常重启
        execute_hourly_restart(interval_control, occl_timer);
    }

    return ret_status;
}

/**
 * @brief 计算低速模式下下一个小时的等待时间
 * @param remaining_total 剩余总剂量
 * @param interval_control 间隔控制结构体
 */
static void handle_low_flow_calculate_next_hour(uint16_t remaining_total,
                                               dynamic_interval_control_t *interval_control)
{
    // 下个小时剂量 = 剩余剂量
    uint16_t next_hour_dose = remaining_total;

    // 计算运行时间 = 剩余剂量 ÷ 流速
    float run_time_hours = (float)next_hour_dose / G_food_rate;
    uint32_t run_time_ms = (uint32_t)(run_time_hours * 3600000);
    uint32_t wait_time_ms = 3600000 - run_time_ms;

    // 保存重新计算的等待时间
    interval_control->low_flow_wait.wait_duration = wait_time_ms;
}

/**
 * @brief 执行每小时重置和重启流程
 * @param interval_control 间隔控制结构体
 * @param occl_timer 堵塞计时器
 */
static void execute_hourly_restart(dynamic_interval_control_t *interval_control,
                                   uint16_t *occl_timer)
{
    execute_hourly_reset_routine(interval_control, occl_timer);
    update_one_ml(interval_control);
    start_next_hour_pump_cycle();
    save_nv();

}

// 在运行时（例如 tasks 初始化或 running_volume 变化时）更新
static void update_one_ml(dynamic_interval_control_t *control)
{
	static uint32_t macro_factor = 100;
	uint32_t result;
    // 读取时确保不超过上限
    if (running_volume > G_volume) {
        running_volume = G_volume;
    }
    // 1. 计算基础值
    if (running_volume < 1000) {
        result = ONE_ML;                    // 1800
    } else {
        uint32_t factor = 100 + (running_volume / 1000);
        result = ONE_ML * factor / 100;     // 随 volume 增加
    }

    // 2. 第一次调用：保存宏系数
    if (is_first_call || control->has_pause_this_period) {
        uint32_t with_macro = APPLY_FOOD_RATE(result);
        macro_factor = (with_macro * 100) / result;
        is_first_call = false;
        result = with_macro;
    } else {
        // 后续调用：应用宏系数（无论 volume 多少，宏的影响都在！）
        result = (result * macro_factor) / 100;
    }

    G_time_cal.one_ml_corrected = result;
    // ========== 分段范围保护 ==========
    if (G_food_rate < 32) {
        // 固定为 ONE_ML（已经在上面设置了，这里只是保险）
        G_time_cal.one_ml_corrected = ONE_ML;
    } else if (G_food_rate <= 200) {
        // 100~200：最大 1854
        if (G_time_cal.one_ml_corrected > 1854) {
            G_time_cal.one_ml_corrected = 1854;
        }
        if (G_time_cal.one_ml_corrected < ONE_ML) {
            G_time_cal.one_ml_corrected = ONE_ML;  // 保底下限
        }
    } else {
        // > 200：最大 2000
        if (G_time_cal.one_ml_corrected > 2000) {
            G_time_cal.one_ml_corrected = 2000;
        }
        if (G_time_cal.one_ml_corrected < ONE_ML) {
            G_time_cal.one_ml_corrected = ONE_ML;  // 保底下限
        }
    }
}

/**
 * @brief 处理非低速模式的小时剂量检查
 * @param food_volume_increment 食物体积增量
 * @param interval_control 间隔控制结构体
 * @param occl_timer 堵塞计时器
 * @return 返回状态码
 */
static uint8_t handle_normal_flow_hourly_dose(uint16_t *food_volume_increment,
                                              dynamic_interval_control_t *interval_control,
                                              uint16_t *occl_timer)
{
    uint8_t ret_status = CONTINUE_RUNNING;

    // 只有当总剂量刚好同时完成时才需要设置标志
    if (G_dose_limit > 0 && (G_dose_limit - G_volume) == 0) {
        // 总剂量刚好完成，设置标志让 handle_dose_check 知道
        interval_control->hour_dose_completed = 1;
    } else {
        // 还有剩余剂量或没有剂量限制，检查双TPA切换
        if ((G_tpa_type == DUAL_TPA) && (G_water_rate > 0)) {
            ret_status = REQUEST_BOLUS_SWITCH;
        }
    }

    // 执行每小时重启流程（使用统一的函数）
    execute_hourly_restart(interval_control, occl_timer);

    return ret_status;
}

/**
 * @brief 处理低速模式的小时剂量检查
 * @param food_volume_increment 食物体积增量
 * @param interval_control 间隔控制结构体
 * @param occl_timer 堵塞计时器
 * @return 返回状态码
 */
static uint8_t handle_low_flow_hourly_dose(uint16_t *food_volume_increment,
                                           dynamic_interval_control_t *interval_control,
                                           uint16_t *occl_timer)
{
    uint8_t ret_status = CONTINUE_RUNNING;

    // 检查是否在等待状态
    if (interval_control->low_flow_wait.is_waiting) {
        uint32_t current_ms = HAL_GetTick();
        uint32_t elapsed_wait_time = current_ms - interval_control->low_flow_wait.wait_start_time;

        // 检查等待时间是否结束
        if (elapsed_wait_time >= interval_control->low_flow_wait.wait_duration) {
            // 等待时间结束，处理结束逻辑
            ret_status = handle_low_flow_wait_end(food_volume_increment, interval_control, occl_timer);
        }
        // 仍在等待中，返回 CONTINUE_RUNNING
    }

    return ret_status;
}

/***************************** 主函数实现 *****************************/

/**
 * @brief 检查和处理小时剂量逻辑
 * @param food_volume_increment 食物体积增量
 * @param interval_control 间隔控制结构体
 * @param occl_timer 堵塞计时器
 * @return 任务返回码：
 *         - CONTINUE_RUNNING：继续运行
 *         - REQUEST_BOLUS_SWITCH：请求切换到推注模式
 *
 * @note 该函数根据当前模式（低速/非低速）处理小时剂量逻辑：
 *       1. 低速模式：处理等待状态和小剂量完成逻辑
 *       2. 非低速模式：处理标准的小时剂量完成逻辑
 *       3. 负责双TPA切换判断和剂量完成标志设置
 */
static uint8_t check_and_process_hourly_dose(uint16_t *food_volume_increment,
                                           dynamic_interval_control_t *interval_control,
                                           uint16_t *occl_timer)
{
    uint8_t ret_status = CONTINUE_RUNNING;

#if ENABLE_LOW_FLOW_MODE
    // ====== 低速模式处理 ======
    if (G_low_flow_mode) {
        ret_status = handle_low_flow_hourly_dose(food_volume_increment, interval_control, occl_timer);
    } else
#endif
    {
        // ====== 非低速模式处理 ======
        // 检查小时剂量是否达到
        if (G_hour_volume >= G_food_rate) {
            if(*occl_timer > OCCL_TEST_SAFE_MIN && *occl_timer < OCCL_TEST_END ){
            	return CONTINUE_RUNNING ;
            }
            if(!interval_control->has_pause_this_period ){
              	// ===== 停止电机后，检查是否满足1小时 =====
            	uint32_t current_time = HAL_GetTick();
            	uint32_t elapsed_time = current_time - interval_control->hour_start_time;

            	// 56分钟到58分钟，继续跑，但总量小于1.02
            	if ((elapsed_time > 56 * 60000) && (elapsed_time < (HOUR_DURATION_MS - MAX_WAIT_TIME_MS))) {
            		return CONTINUE_RUNNING ;
            	}
            	//58分钟到小时，如果流速大于50，继续跑
            	else if((elapsed_time >= (HOUR_DURATION_MS -MAX_WAIT_TIME_MS)) && (elapsed_time < HOUR_DURATION_MS)){
            		if(G_food_rate >50 && (G_hour_volume * 100 < 102 * G_food_rate)){
            			return CONTINUE_RUNNING ;
            		}
            		else{
            			if(G_one_ml_count < G_time_cal.one_ml_corrected *0.01)
            			return CONTINUE_RUNNING ;
            		}
            	}
          }
            ret_status = handle_normal_flow_hourly_dose(food_volume_increment, interval_control, occl_timer);
        }
    }

    return ret_status;
}
/**
 * @brief 执行每小时重置流程
 *
 * 在每小时剂量达标后，执行一系列重置操作:
 * 1. 更新流速传感器状态
 * 2. 关闭泵马达
 * 3. 重置闭塞检测状态
 * 4. 重置间隔控制
 *
 * @param interval_control 动态间隔控制结构体指针
 *        - 类型: dynamic_interval_control_t*
 *        - 输入/输出: 既读取当前状态，也用于重置
 *        - 关键操作: 调用reset_hourly_interval_control进行重置
 *
 * @param occl_timer 闭塞检测计时器指针
 *        - 类型: uint16_t*
 *        - 输出: 调用reset_occlusion_detection_state进行重置
 *        - 目标值: OCCL_TEST_START_COUNT
 *
 * @note 重置序列说明:
 *       1. 获取当前时间戳
 *       2. 更新流速传感器（基于时间误差）
 *       3. 关闭泵马达
 *       4. 重置闭塞检测状态（状态机和计时器）
 *       5. 重置间隔控制结构
 *
 * @note 为什么需要重置:
 *       确保每个小时周期从干净的状态开始
 *       避免上一小时的状态影响下一小时的操作
 *
 * @see reset_occlusion_detection_state() - 重置闭塞检测
 * @see reset_hourly_interval_control() - 重置间隔控制
 * @see update_flow_sensor_by_time_error() - 更新流速传感器
 * @see pump_motor_off() - 关闭马达
 */
static uint8_t execute_hourly_reset_routine(dynamic_interval_control_t *interval_control,
                                       uint16_t *occl_timer)
{
    // 流量传感器更新（有条件编译）
    #ifdef ENABLE_FLOW_SENSOR_CALIBRATION
	uint32_t current_time = HAL_GetTick();

    // 只在启用流量传感器校准时执行
    update_flow_sensor_by_time_error(interval_control, current_time);
    #endif


    pump_motor_off();
    G_hour_volume = 0;
#if(1)
    if(!interval_control->has_pause_this_period ){
        uint32_t wait_start = HAL_GetTick();
        uint32_t max_wait_ms = 3 * 60000;  // 最多等3分钟
    	while(1){
    	// ===== 停止电机后，检查是否满足1小时 =====
    		uint32_t current_time = HAL_GetTick();
    		uint32_t elapsed_time = current_time - interval_control->hour_start_time;

    		bool in_wait_zone1 =(elapsed_time > 54 * 60000 && elapsed_time <= 56*60000 );
    		bool in_wait_zone2 = (elapsed_time > HOUR_DURATION_MS -MAX_WAIT_TIME_MS) && (elapsed_time <HOUR_DURATION_MS );

    	// 未满1小时，就在这里等待
    	    if (!in_wait_zone1 && !in_wait_zone2) {
    	        break;  // 不在等待区间，退出循环
    	    }
            // 超时保护
            if ((current_time - wait_start) > max_wait_ms) {
            	if(G_food_rate >=5 ){
            		break;
            	}


            }

    	    // 在等待区间，延时检查
    	    HAL_Delay(100);
    	    run_mode_screen();

    	    // 检测停止按键
    	    if (read_keypad() == RUN_STOP_SWITCH) {
    	        break;
    	    }
    	}

    }
    if(interval_control-> pause_timeout){

    }
#else
    if (!interval_control->has_pause_this_period) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed_time = current_time - interval_control->hour_start_time;

        // 不等待，直接返回
        if (elapsed_time < HOUR_DURATION_MS) {
            return NORMAL_EXIT;  // 时间未到，直接返回
        }
    }
#endif
    // 重置堵塞检测状态和计时器
    reset_occlusion_detection_state(occl_timer);

    // 重置间隔控制
    reset_hourly_interval_control(interval_control);
    return NORMAL_EXIT;  // 或 RETURN_OK
}

/**
 * @brief 重置闭塞检测状态和计时器
 *
 * 将闭塞检测恢复到初始状态，包括:
 * 1. 设置状态机为OCCL_STATE_IDLE
 * 2. 重置计时器为OCCL_TEST_START_COUNT
 * 3. 验证重置结果，必要时强制修正
 *
 * @param occl_timer 闭塞检测计时器指针
 *        - 类型: uint16_t*
 *        - 输出: 被设置为OCCL_TEST_START_COUNT
 *        - 作用: 控制下次检测的时间点
 *
 * @note 重置目标值说明:
 *       g_occl_state: 全局状态变量，目标值OCCL_STATE_IDLE
 *       *occl_timer: 局部计时器，目标值OCCL_TEST_START_COUNT
 *
 * @note 验证和修正机制:
 *       1. 重置后立即验证状态和计时器
 *       2. 如果验证失败，输出调试信息
 *       3. 强制再次设置正确值（双重保障）
 *       4. 确保状态一致性
 *
 * @warning 重要性:
 *       闭塞检测状态的正确性直接关系到系统安全
 *       错误的状态可能导致漏检或误检
 *
 * @see g_occl_state - 全局闭塞状态变量
 * @see OCCL_STATE_IDLE - 状态常量定义
 * @see OCCL_TEST_START_COUNT - 计时器起始值
 */
static void reset_occlusion_detection_state(uint16_t *occl_timer)
{
    // 重置状态和计时器
    g_occl_state = OCCL_STATE_IDLE;
    *occl_timer = OCCL_TEST_START_COUNT;

    // 验证重置结果
    if (g_occl_state != OCCL_STATE_IDLE || *occl_timer != OCCL_TEST_START_COUNT) {
        #ifdef DEBUG_ERROR
        printf("ERROR: State reset failed - state=%d, timer=%d\n",
               g_occl_state, *occl_timer);
        #endif
        // 强制修正
        g_occl_state = OCCL_STATE_IDLE;
        *occl_timer = OCCL_TEST_START_COUNT;
    }
}

/**
 * @brief 启动下一个小时泵循环
 *
 * 计算并启动下一个小时的泵送操作:
 * 1. 计算马达速率（基于食物速率）
 * 2. 启动泵马达
 *
 * @note 马达速率计算:
 *       公式: G_motor_rate = (G_food_rate > 4) ? (G_food_rate - 4) : 0
 *       说明: 如果食物速率大于4，减去4作为马达速率
 *             否则马达速率为0（保护机制）
 *
 * @note 启动参数:
 *       速率: G_motor_rate (计算得到)
 *       模式: MOTOR_INT_FORMULA (配方模式)
 *
 * @see start_pump() - 实际的泵启动函数
 * @see G_food_rate - 全局食物速率变量
 * @see G_motor_rate - 全局马达速率变量（输出）
 */
static void start_next_hour_pump_cycle(void)
{
#if ENABLE_LOW_FLOW_MODE
    // 低速模式：固定为1（根据 init_run_mode_state 中的逻辑）
    // 正常模式：G_food_rate - 4，最小为0
    G_motor_rate = G_low_flow_mode ? 1 : ((G_food_rate > 4) ? (G_food_rate - 4) : 0);
#else
    // 未启用低速模式
    G_motor_rate = (G_food_rate > 4) ? (G_food_rate - 4) : 0;
#endif
	if(running_volume > G_volume){
		running_volume = G_volume;
	}
    // 启动泵
    start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);
}

/**
 * @brief 处理堵塞检测
 *
 * 执行闭塞检测的主要逻辑:
 * 1. 递增闭塞检测计时器
 * 2. 调用核心检测函数
 * 3. 处理检测结果
 *
 * @param occl_timer 闭塞检测计时器指针
 *        - 类型: uint16_t*
 *        - 输入/输出: 递增后传递给occl_test函数
 *        - 递增: 每次调用递增1
 *        - 作用: 控制检测执行频率
 *
 * @param occl_data 闭塞检测数据缓存数组
 *        - 类型: int16_t[]
 *        - 大小: OCCL_DATA_SIZE
 *        - 输入/输出: 传递给occl_test函数使用
 *        - 作用: 存储原始检测数据样本
 *
 * @param air_test 空气检测结构体指针
 *        - 类型: air_test_struct*
 *        - 输入: 提供空气检测相关数据
 *        - 关键字段: air_rate_inc (空气率增量)
 *        - 作用: 影响闭塞检测的计算
 *
 * @param interval_control 动态间隔控制结构体指针
 *        - 类型: dynamic_interval_control_t*
 *        - 输入: 提供时间控制和状态信息
 *        - 作用: 影响检测间隔和条件判断
 *
 * @param air_result 空气检测结果
 *        - 类型: uint8_t
 *        - 取值: AIR_OK 或 AIR_ERROR
 *        - 作用: 传递给occl_test，影响检测逻辑
 *
 * @return uint8_t 返回状态码:
 *        - CONTINUE_RUNNING: 检测正常，无闭塞
 *        - OCCLUSION_ALARM: 检测到闭塞，需要触发报警
 *
 * @note 核心调用:
 *       调用occl_test()执行实际的闭塞检测
 *       该函数返回OCCL_ERROR时表示检测到闭塞
 *
 * @see occl_test() - 核心检测函数
 * @see OCCL_ERROR - 检测结果常量
 */
static uint8_t process_occlusion_detection(uint16_t *occl_timer,
                                          int16_t occl_data[OCCL_DATA_SIZE],
                                          air_test_struct *air_test,
                                          dynamic_interval_control_t *interval_control,
                                          uint8_t air_result)
{
    uint8_t ret_status = CONTINUE_RUNNING;

    // 递增计时器并执行检测
    (*occl_timer)++;
    uint8_t occl_result = occl_test(occl_timer, occl_data,
                                   &air_test->air_rate_inc,
                                   interval_control, air_result);

    // 检查检测结果
    if (occl_result == OCCL_ERROR) {
        ret_status = OCCLUSION_ALARM;
    }
    return ret_status;
}


/**
 * @brief 获取修正后的ONE_ML值
 */
uint32_t get_corrected_one_ml(void)
{
    return (G_time_cal.valid_samples >= 2) ? G_time_cal.one_ml_corrected : ONE_ML;
}

/**
 * @brief 基于时间误差更新流量传感器修正
 * @param control 动态间隔控制结构体（包含hour_start_time）
 * @param completion_time 当前完成时间戳(ms)
 */

#ifdef ENABLE_FLOW_SENSOR_CALIBRATION
static void update_flow_sensor_by_time_error(dynamic_interval_control_t *control, uint32_t completion_time)
{
    // 如果有暂停，跳过本次修正
    if (control->has_pause_this_period) {
        return;
    }

    if (control->hour_start_time == 0) {
        return;
    }

    // 计算实际完成时间（毫秒）
    uint32_t actual_time_ms = completion_time - control->hour_start_time;
    uint32_t expected_time_ms = 3600000;

    // 计算时间误差（分钟）
    int32_t time_error_minutes = (int32_t)((actual_time_ms - expected_time_ms) / 60000);

    // 精细修正：每分钟误差调整10个计数
    int32_t adjustment = time_error_minutes * (-10);  // 负号因为：提前完成要增加ONE_ML

    // 应用修正
    G_time_cal.one_ml_corrected = ONE_ML + adjustment;

    // 限制修正范围在 1780-1869 之间
    if (G_time_cal.one_ml_corrected < 1780) {
        G_time_cal.one_ml_corrected = 1780;
    } else if (G_time_cal.one_ml_corrected > 1869) {
        G_time_cal.one_ml_corrected = 1869;
    }

    G_time_cal.valid_samples++;
}
#endif
/**
 * 控制循环计时
 */
static void control_loop_timing(uint8_t *one_second_count)
{
    (*one_second_count)++;
    wait_50ms();
}

/**
 * 处理暂停操作
 */
static uint8_t handle_pause(dynamic_interval_control_t *control,uint16_t *food_volume_increment)
{
    pump_motor_off();
    ir_heat_off();
    update_daily_food_vol(*food_volume_increment);
#ifdef USE_UPDATE_DYNAMIC_CONTROL

    // 记录暂停相关参数
    record_pause_params(control,food_volume_increment);

#endif
    *food_volume_increment = 0;
    SaveNV_Volume();
    return HOLD_MODE__TASK;
}

#endif

static void record_pause_params(dynamic_interval_control_t *control, uint16_t *food_volume_increment)
{
    // 记录本次暂停前的流量
    control->flow_before_pause = *food_volume_increment;

    // 累加到暂停流量总和
    control->pause_flow_total += control->flow_before_pause;

    update_accumulated_work_time(control);

    // 标记发生过暂停
    control->has_pause_this_period = 1;
}


static void update_accumulated_work_time(dynamic_interval_control_t *control)
{
    uint32_t now = HAL_GetTick();
    uint32_t interval = now - control->last_enter_time;

    control->accumulated_work_time += interval;

}

/**************************************
 * run_mode_task()
 * @detail
 * Function enters pump to Run Mode. Function executes until stopped by by an
 * user or an other event. Function returns the number for the next task.
 */


/**************************************
 * @brief 切换至营养液输送模式
 * @return uint8_t 固定返回0（FOOD模式标识）
 * @note 操作流程：
 *       1. 停止当前泵送
 *       2. 更新累计量
 *       3. 调整夹爪位置
 *       4. 按新参数启动泵
 */
uint8_t swap_bolus_to_formula(void) {
    pump_motor_off();
    ir_heat_off();

    // 记录当前输送量
    update_daily_water_vol(G_water_rate);

    // 初始化新模式参数
    G_one_ml_count = G_one_ml_flag = 0;
    // 计算马达速率（低速模式已初始化）
#if ENABLE_LOW_FLOW_MODE
    G_motor_rate = G_low_flow_mode ? 1 : ((G_food_rate > 4) ? (G_food_rate - 4) : 0);
#else
    G_motor_rate = (G_food_rate > 4) ? (G_food_rate - 4) : 0;
#endif

    // 调整夹爪位置
    move_pincher_all_ccw();

	if(running_volume > G_volume){
		running_volume = G_volume;
	}
    // 启动新输送模式
    start_pump(G_motor_rate, SAFE_MOTOR_INT_FORMULA);
    return 0; // FOOD模式标识
}

/************************************
 * swap_bolus_to_formula()
 * @short			Change from water delivery to formula
 * @param			void
 * @retval  	    uint8_t return delivery mode 0 (FOOD)
 * @detail
 * Initialize from water delivery to formula. Update daily water volume.
 * Set motor rate for selected food rate. Move pincher CCW (close water tubing, open food
 * tubing. Start pumping.
 */
#if(0)
uint8_t swap_bolus_to_formula(void)
{
  pump_motor_off();
  ir_heat_off();			//in case the OCCL test was started
  update_daily_water_vol(G_water_rate);
  G_one_ml_count=0;
  G_one_ml_flag=0;
  G_motor_rate = G_food_rate-4;
  //move_pincher_from_all_cw_to_magnet();
  move_pincher_all_ccw();
  start_pump(G_motor_rate,MOTOR_INT_FORMULA);
  return 0;
}
#endif
/**  End swap_bolus_to_formula()  **/
/**************************************
 * @brief 切换至冲洗液输送模式
 * @return uint8_t 固定返回1（BOLUS模式标识）
 * @note 操作流程：
 *       1. 停止当前泵送
 *       2. 调整夹爪位置
 *       3. 按最大速率启动泵
 */
uint8_t swap_formula_to_bolus(void) {
    pump_motor_off();
    ir_heat_off();

    // 调整夹爪位置
    move_pincher_all_cw();


    // 启动冲洗模式（固定高速率）
    start_pump(FULL_FOOD_RATE, MOTOR_INT_BOLUS);
    return 1; // BOLUS模式标识
}

/************************************
 * swap_formula_to_bolus()
 * @short			Change from formula delivery to water
 * @param			void
 * @retval  	    uint8_t return delivery mode 1 (BOLUS)
 * @detail
 * Initialize from food delivery to bolus.
 * Move pincher CW (close formula tubing, open water tubing
 * Start pumping.
 */
#if(0)
uint8_t swap_formula_to_bolus(void){
  pump_motor_off();
  ir_heat_off();			//in case the OCCL test was started
  move_pincher_all_cw();
  start_pump(FULL_FOOD_RATE,MOTOR_INT_BOLUS);
  return 1;
}
#endif
/**  End swap_formula_to_bolus()  **/

/**************************************
 * @brief 设备关机任务
 * @return uint8_t 固定返回HOLD_MODE__TASK
 * @note 关机流程：
 *       1. 显示关机确认界面
 *       2. 检查管路移除状态
 *       3. 保存配置并切断电源
 */
uint8_t MCU_off_task(void) {
    // 显示关机进度条
    uint8_t result = MCU_off_bar();
    if (result == HOLD_MODE__TASK) {
        return HOLD_MODE__TASK; // 提前取消关机
    }

    // 停止所有输出
    pump_motor_off();

    /* 管路已移除时的快速关机 */
    if (pts_sensor() == TUBING_NOT_DETECTED) {
        execute_fast_shutdown();
        return HOLD_MODE__TASK;
    }

    /* 正常关机流程 */
    if (show_shutdown_prompts() == SHUTDOWN_CONFIRMED) {
        perform_shutdown_sequence();
    }

    return HOLD_MODE__TASK;
}
/**************************************
 * @brief 执行快速关机流程
 * @note 包含：
 *       1. 夹爪复位
 *       2. 传感器校准
 *       3. 数据保存
 */
static void execute_fast_shutdown(void) {
    move_pincher_to_center();
    show_message("Shutting down...", "");
    cal_tir_sensor();
    G_tpa_type = NO_TPA;
    save_nv();
    power_off_pump();
}

/**************************************
 * @brief 显示关机确认提示
 * @return uint8_t YES-确认关机 NO-取消
 * @note 顺序显示：
 *       1. 移除管路确认
 *       2. 关闭夹具确认
 */
static uint8_t show_shutdown_prompts(void) {
    if (show_message_yes_no("REMOVE SET?") != YES) {
        return NO;
    }
    if (show_message_yes_no("CLOSE CLAMPS?") != YES) {
        return NO;
    }
    return YES;
}

/**************************************
 * @brief 执行关机序列
 * @note 包含：
 *       1. 夹爪复位
 *       2. 10秒等待管路移除
 *       3. 最终关机操作
 */
static void perform_shutdown_sequence(void) {
    move_pincher_all_cw();

    uint8_t status = move_pincher_from_all_cw_to_magnet();
    if (status == ERR_NO_MAGNET) {
        show_error_message(TEN_SECOND, "ERROR 100", "");
        save_nv();
        power_off_pump();
    }

    // 等待管路移除
    uint8_t seconds = 0;
    while (seconds < 10 && pts_sensor() == TUBING_DETECTED) {
        show_message("REMOVE", "SET NOW");
        if (check_50ms_cycle(20)) {
            seconds++;
        }
    }

    // 最终关机
    if (pts_sensor() == TUBING_NOT_DETECTED) {
        show_message("Shutting down...", "");
        cal_tir_sensor();
    }
    G_tpa_type = NO_TPA;
    save_nv();
    power_off_pump();
}
/**************************************
 * @brief 检查50ms周期完成
 * @param cycles 需要等待的周期数
 * @return uint8_t 1-周期完成 0-未完成
 * @note 用于精确计时等待
 */
static uint8_t check_50ms_cycle(uint8_t cycles) {
    static uint8_t count = 0;
    wait_50ms();
    if (++count >= cycles) {
        count = 0;
        return 1;
    }
    return 0;
}


/**************************************
 * MCU_off_task()
 * @detail
 * Prepare pump for power down. Prompt for set removal. Save parameters to NVRAM
 */


/**  End MCU_off_task()  **/

/**************************************
 * @brief 管路移除任务
 * @return uint8_t 固定返回HOLD_MODE__TASK
 * @note 操作流程：
 *       1. 显示移除提示
 *       2. 复位夹爪位置
 *       3. 等待管路移除或超时
 *       4. 状态复位
 */
uint8_t remove_set_task(void) {
    // 停止泵送并显示提示
    pump_motor_off();
    if (show_message_yes_no("REMOVE SET?") != YES) {
        return HOLD_MODE__TASK;
    }

    // 夹爪复位
    uint8_t status = move_pincher_to_center();
    if (status == ERR_NO_MAGNET) {
        handle_magnet_error();
        return HOLD_MODE__TASK;
    }

    // 等待管路移除
    wait_for_tube_removal();

    // 状态复位
    G_prime_completed = 0;
    return HOLD_MODE__TASK;
}


/**************************************
 * @brief 处理磁铁检测错误
 * @note 显示错误信息并强制关机
 */
static void handle_magnet_error(void) {
    show_error_message(TEN_SECOND, "ERROR 100", "");
    save_nv();
    power_off_pump();
}
/**************************************
 * @detail
 * Display "REMOVE SET" screen and release the set (rotate pincher to center) if
 * user wants to remove the set
 *
 */

#if(0)
uint8_t remove_set_task(void){
  uint8_t one_second_count,seconds_time;
  uint8_t status;
  const char ERROR_100[] = "ERROR 100";
  const char REMOVE_SET[] = "REMOVE SET?";
  const char REMOVE_SET_NOW[] = "REMOVE SET NOW";
  const char CLOSE_CLAMPS[] = "CLOSE QUICK-CLAMPS";

  one_second_count=0;
  seconds_time = 0;
  pump_motor_off();
  if (show_message_yes_no(REMOVE_SET) == NO){     //"NO" or timeout returns HOLD_MODE_TASK
    return(HOLD_MODE__TASK);
  }
  if (show_message_yes_no(CLOSE_CLAMPS) == NO){     //"NO" or timeout returns HOLD_MODE_TASK
	return(HOLD_MODE__TASK);
  }
  move_pincher_all_cw();
  status = move_pincher_from_all_cw_to_magnet();
  if (status == ERR_NO_MAGNET){
  	show_error_message(TEN_SECOND,ERROR_100,"");
  	save_nv();
  	power_off_pump();
  }
  G_tpa_type = NO_TPA;
  while ((seconds_time < 10) && (pts_sensor() == TUBING_DETECTED)){
    show_message(REMOVE_SET_NOW,"");
    one_second_count++;
    if (one_second_count>20){
      one_second_count=0;
      seconds_time++;
    }

	while (G_tick5ms_counter<10);    //50ms loop
	G_tick5ms_counter = 0;
  }
  G_prime_completed = 0;

  return HOLD_MODE__TASK;
}
#endif
/**  End remove_set_task()  **/

__attribute__((always_inline))
static inline void wait_50ms(void)
{
    while(G_tick5ms_counter < 10);  // 等待50ms
    G_tick5ms_counter = 0;
}
