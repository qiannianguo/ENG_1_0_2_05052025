/*****************************************************************************
* @file			: io_calls.c
* @short        : io and other hardware related functions
******************************************************************************
*
* @details
* T
*
*
******************************************************************************
**/
#include <stdint.h>
#include <string.h>
#include "MCU_init.h"
#include <stm32l0xx_hal.h>
#include <stm32l0xx_it.h>
#include "common.h"
#include "functions.h"
#include "main_tasks.h"
#include "io_calls.h"
#include "screen_helper.h"
#include "screens.h"
#include <stdbool.h>
#include <stdlib.h>

const char ERROR_100[] = "ERROR 100";
const char NO_TPA_DETECTED[] = "NO TPA DETECTED";
const char SET_NOT_[] = "SET NOT";
const char _INSTALLED[] = "INSTALLED";

static uint8_t pinch_phase = 0;

static const GPIO_PinState STEP_PHASE_TABLE[4][4] = {
		 /* Phase 0 (BA) */
		    {GPIO_PIN_SET,     GPIO_PIN_RESET,   GPIO_PIN_RESET,   GPIO_PIN_SET},
		    /* Phase 1 (CB) */
		    {GPIO_PIN_SET,     GPIO_PIN_SET,     GPIO_PIN_RESET,   GPIO_PIN_RESET},
		    /* Phase 2 (DC) */
		    {GPIO_PIN_RESET,   GPIO_PIN_SET,     GPIO_PIN_SET,     GPIO_PIN_RESET},
		    /* Phase 3 (AD) */
		    {GPIO_PIN_RESET,   GPIO_PIN_RESET,   GPIO_PIN_SET,     GPIO_PIN_SET}
};

// Constants for better maintainability
enum {
    TEST_DELAY = SET_TEST_DELAY,
    ERROR_DISPLAY_TIME = TEN_SECOND
};

/*--------- 校准参数配置 ---------*/
enum {
    INITIAL_DAC_VALUE = 2000,    // 初始DAC值
    DELAY_AFTER_DAC_SET = 200,    // DAC设置后的延时(ms)
    BINARY_SEARCH_STEPS = 11,     // 二分查找迭代次数
    ADC_SETTLE_TIME_TICKS = 40,   // ADC稳定时间(~170ms = 40*5ms)
    PD2_TARGET = PD2_IR_CAL_TARGET // PD2目标值
};

/* 操作状态枚举 */
typedef enum {
    RTC_OP_READY,      // 就绪状态
    RTC_OP_BUSY,       // 操作进行中
    RTC_OP_SUCCESS,    // 操作成功
    RTC_OP_FAILED,     // 操作失败
    RTC_OP_TIMEOUT     // 操作超时
} rtc_status_t;

volatile rtc_status_t g_rtc_status;
extern UART_HandleTypeDef huart2;
void delay_us(uint16_t);
extern TIM_HandleTypeDef htim22;
extern TIM_HandleTypeDef htim21;
extern I2C_HandleTypeDef hi2c3;
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc;

//extern uint16_t adc_buffer[18];   //dma buffer for adc 6ch x 3 samples
extern uint8_t G_tick5ms_counter;  //defined in main.c
extern uint16_t G_ir_pd2_nocomp;
extern volatile uint16_t G_usonic_value, G_ir_pd2_value;
extern uint16_t G_tp_value, G_dac_cal_value, G_vpts;
extern uint8_t G_an_on;

extern  uint16_t G_motor_rate;
extern  uint16_t G_food_rate;
extern  uint16_t G_volume,G_hour_volume,G_water,running_volume;
extern  uint16_t G_total_volume;
extern  uint16_t G_water_rate;
extern  uint16_t G_dose_limit;

extern  uint8_t G_alarm_vol;
extern  uint8_t G_warning_vol;
extern uint8_t G_controls_locked,G_battery_alarm_muted;
extern  uint8_t G_display_brightness;
extern  uint8_t G_dose_complete_callback_index;
extern  uint8_t G_svc_entry_delay;
extern uint8_t G_prime_completed;
extern uint8_t G_cal_status;
extern uint8_t G_tpa_type;
extern uint16_t G_v_battery;
extern uint32_t G_acc_formula_ml, G_acc_bolus_ml, G_acc_bat_time_min;
extern volatile uint16_t G_one_ml_count;
extern volatile uint8_t G_one_ml_flag;

#if (MOTOR_TYPE == 1)
static const uint16_t kPincherMaxSteps = 250;
static const uint16_t kWarningThreshold = 150;

#elif(MOTOR_TYPE == 2)
static const uint16_t kPincherMaxSteps = 236;
static const uint16_t kWarningThreshold = 141;

#endif
#if(MOTOR_TYPE == 1)
static inline void apply_motor_phase(uint8_t phase);
#endif

static inline void power_off_motor_coils(void);
/* 静态子函数声明 */
static uint8_t detect_tpa_type(void);
static void handle_tpa_status(uint8_t status, uint8_t pts_wakeup);
static void adjust_pincher_position(uint8_t status);
static void check_tubing_empty(void);
static void check_distal_tubing(void);
static void handle_tubing_removal(uint8_t* set_installed);

static void handle_missing_tube(uint16_t* occl_timer);
static uint8_t initiate_tube_removal_sequence(void);
static void perform_tube_removal(void);
static void reset_system_state(void);
static void resume_pumping(void);

static bool process_water_stage(uint16_t* timer, uint16_t* full_counter);
static void transition_to_formula_stage(void);
static bool process_formula_stage(uint16_t* timer, uint16_t* full_counter, uint16_t* cont_counter);
static bool process_final_stage(uint16_t* timer, uint16_t cont_counter, uint16_t* air_counter);

static bool ProcessFormulaStage(uint16_t* pTimer, uint16_t* pFullCnt, uint16_t* pContCnt);
static bool ShouldAbortFormulaStage(uint16_t timer, uint16_t full_cnt);

static void start_pumping(uint16_t rate);
static void stop_pumping(void);
static void wait_one_second(void);
static inline void delay_50ms(void);

static void beep_tone(uint32_t freq_hz, uint32_t duration_ms, uint8_t loud);
static void beep_tone_with_envelope(uint32_t freq_hz, uint32_t duration_ms, uint8_t loud);

uint8_t distal_tube_detected_with_confirmation(void);

PincherDetectionResult g_pincher_result = {
    .magnet_detected = false,
    .step_count = 0,
    .position = POS_UNKNOWN,
    .offset_from_mid = 0
};

static void rotate_to_magnet(PincherDetectionResult *result);
static uint8_t evaluate_pincher_position(const PincherDetectionResult *result);
static bool magnet_sensor_detected(void);

/* ========== Helper Structures and Functions ========== */
/**************************************
 * autoprimedual()
 * @brief 双通道自动灌注功能
 * @return 灌注状态: PRIME_OK/PRIME_ERR/PRIME_EXIT
 * @detail
 * 1. 阶段1: 灌注水剂至TPA位置(定时/传感器检测)
 * 2. 阶段2: 灌注营养液至传感器+3mL位置(定时+连续流检测)
 * 3. 阶段3: 完成灌注至管道末端(检测空气或完成)
 */
uint8_t autoprimedual(void)
{
    prime_stage_t stage = PRIME_STAGE_WATER;
    uint8_t key, one_second_count = 0;
    uint16_t prime_timer = 0;
    uint16_t full_counter = 0, air_counter = 0;
    uint16_t cont_counter = 0, timeout_count;

    // 初始化: 关闭营养侧，启动水剂灌注
    move_pincher_all_cw();

    start_pump(FULL_FOOD_RATE, MOTOR_INT_BOLUS);

    while(1) {
        auto_primescreen(SET_READY, 1);
        key = read_all_keys(&timeout_count);

        /* 处理右键退出 */
        if (key == (RIGHT_ARROW_PRESSED)) {
            key_click();
        }
        if (key == (RIGHT_ARROW_RELEASED)) {
            pump_motor_off();
            return PRIME_EXIT;
        }

        /* 每秒处理逻辑 */
        if (one_second_count >= 20) {
            one_second_count = 0;
            prime_timer++;

            switch(stage) {
                case PRIME_STAGE_WATER:
                    if (process_water_stage(&prime_timer, &full_counter)) {
                        transition_to_formula_stage();
                        stage = PRIME_STAGE_FORMULA;
                        prime_timer = 0;        // 重置计时器
                        full_counter = 0;       // 重置计数器
                    }
                    break;

                case PRIME_STAGE_FORMULA:
                    if (process_formula_stage(&prime_timer, &full_counter, &cont_counter)) {
                        stage = PRIME_STAGE_FINAL;
                        prime_timer = 0;        // 重置计时器
                        air_counter = 0;        // 重置空气计数器
                    } else if (prime_timer > (PRIME_TIME_BAG_TO_SENSOR + PRIME_TIMED_BAG_TO_SENSOR) &&
                            full_counter <= REQUIRED_LIQUID_COUNT) {
                        pump_motor_off();
                        return PRIME_ERR;
                    }
                    break;

                case PRIME_STAGE_FINAL:
                    if (process_final_stage(&prime_timer, cont_counter, &air_counter)) {
                        pump_motor_off();
                        return PRIME_OK;
                    } else if (air_counter > 50) {
                        pump_motor_off();
                        return PRIME_ERR;
                    }
                    break;
            }
        }
        /* 处理1mL标志 */
        if (G_one_ml_flag) {
            G_one_ml_flag = 0;
        }
        one_second_count++;
        delay_50ms();
    }
}

/** 辅助函数 **/
/* 阶段1处理: 水剂灌注 */
static bool process_water_stage(uint16_t* timer, uint16_t* full_counter)
{
    if (!distal_tube_empty()) {
        (*full_counter)++;
    } else {
        *full_counter = 0;
    }

    return (*timer >= (PRIME_TIME_BAG_TO_TPA + 1)) || (*full_counter > 4);
}

/* 切换到营养液阶段 */
static void transition_to_formula_stage(void)
{
    pump_motor_off();
    move_pincher_all_ccw();  // 关闭水剂侧，打开营养侧

    start_pump(FULL_FOOD_RATE, MOTOR_INT_BOLUS);
}

/* 阶段2处理: 营养液灌注 */
static bool process_formula_stage(uint16_t* timer, uint16_t* full_counter, uint16_t* cont_counter)
{
	static uint16_t bubble_counter = 0;
	static uint16_t overrun_count = 0;  // 总气泡计数（新增）
	static uint16_t effective_liquid_count = 0;  // 有效液体计数（新增）
	static bool initial_period_passed = false;  // 标记前12秒是否已过
	  // 检查前10秒是否已过
	if (*timer >= 10) {
	    initial_period_passed = true;
	}
	if (!distal_tube_empty()) {
		(*full_counter)++;
		effective_liquid_count++;     // 有效液体计数
		bubble_counter = 0; // 检测到液体时重置气泡计数
		overrun_count = 0;
	} else {
		bubble_counter++;
		if (bubble_counter > MAX_BUBBLE_TOLERANCE) {
		     // 超过连续气泡限制
			overrun_count++;
			if (effective_liquid_count > 0) {
			    effective_liquid_count--;  // 超限时减少有效计数
			    }
		    if ((overrun_count > MAX_OVERRUN_COUNT) && !initial_period_passed) {
		   // 超限次数过多，彻底清零
		       *full_counter = 0;
		       bubble_counter = 0;
		       overrun_count = 0;
		       effective_liquid_count = 0;
		       } else {
		     // 允许一定次数的超限，只重置气泡计数，不处理full_counter
		         bubble_counter = 0;
 		    }
		}
		// 12秒后，即使检测到气泡也让full_counter增加
		if (initial_period_passed) {
			(*full_counter)++;
		 }
	}

    if (*timer >= (PRIME_TIME_BAG_TO_SENSOR + PRIME_TIMED_BAG_TO_SENSOR)) {
        if (effective_liquid_count > REQUIRED_LIQUID_COUNT) {
        	if(*full_counter >= PRIME_TIME_FORMULA){
            *cont_counter = *full_counter;
        	}else{
        		*cont_counter = PRIME_TIME_FORMULA;
        	}
            *full_counter = effective_liquid_count;
            return true;
        }else{
        *full_counter = effective_liquid_count;
        }

    }
    return false;
}

/* 阶段3处理: 最终灌注 */
static bool process_final_stage(uint16_t* timer, uint16_t cont_counter, uint16_t* air_counter)
{
    if (distal_tube_empty()) {
        (*air_counter)++;
    }
    else {
            // ========== 修正7: 完善空气检测逻辑 ==========
            *air_counter = 0;  // 重置空气计数器当检测到液体时
            // ==========================================
        }
    return *timer >=(PRIME_TIME_SENSOR_TO_END - cont_counter);
}

/**************************************
 * autoprimedual()
 * @detail
 * Automatically prime the dual set (bolus-nutrition). Start priming after prime
 * button is pressed. Close nutrition side TPA and fill the bolus tubing from bag
 * to the TPA. Then close the bolus side TPA and pump nutrition until feeding tube
 * is filled. Operation is based on predetermined timing.
 */

/**
  * @brief 单通道自动灌注功能
  * @return 灌注状态:
  *         PRIME_OK - 灌注成功
  *         PRIME_ERR - 灌注失败
  *         PRIME_EXIT - 用户中止
  * @detail
  * 1. 阶段1: 灌注营养液直到传感器检测到连续流
  * 2. 阶段2: 完成灌注直到管道末端，检测空气
  */
uint8_t autoprimesingle(void)
{
	prime_stage_t stage = PRIME_STAGE_FORMULA;
    uint8_t key;
    uint16_t timeout_count;
    uint16_t timer = 0;
    uint16_t full_counter = 0;
    uint16_t air_counter = 0;
    uint16_t cont_counter = 0;
    uint8_t one_second_count = 0;

    /* 初始化硬件 */
    move_pincher_all_cw();

    start_pump(FULL_FOOD_RATE, MOTOR_INT_BOLUS);

    while(1) {
        /* 更新用户界面 */
    	auto_primescreen(SET_READY,1);;

        /* 读取按键输入 */
    	key = read_all_keys(&timeout_count);

        /* 处理用户中止 */
        if (key == (RIGHT_ARROW_PRESSED)) {
        	key_click();
        }
        if (key == (RIGHT_ARROW_RELEASED)) {
        	pump_motor_off();
            return PRIME_EXIT;
        }

        /* 每秒定时处理 */
        if (++one_second_count >= 20) {  // 20*50ms=1s
            one_second_count = 0;
            timer++;

            switch(stage) {
                case PRIME_STAGE_WATER:

                    break;
                case PRIME_STAGE_FORMULA:
                    if (ProcessFormulaStage(&timer, &full_counter, &cont_counter)) {
                        stage = PRIME_STAGE_FINAL;
                        timer = 0;  // 重置计时器进入下一阶段
                    } else if (ShouldAbortFormulaStage(timer, full_counter)) {
                    	pump_motor_off();
                        return PRIME_ERR;
                    }
                    break;

                case PRIME_STAGE_FINAL:
                    if (process_final_stage(&timer, cont_counter, &air_counter)) {
                    	pump_motor_off();
                        return PRIME_OK;
                    } else if (air_counter > 5) {
                    	pump_motor_off();
                        return PRIME_ERR;
                    }
                    break;
            }
        }

        /* 处理1mL标志 */
        if (G_one_ml_flag) {
                   G_one_ml_flag = 0;
               }

        delay_50ms();  // 延迟50ms
    }
}

/** 辅助函数 **/

/**
  * @brief 处理营养液灌注阶段
  */
static bool ProcessFormulaStage(uint16_t* pTimer, uint16_t* pFullCnt, uint16_t* pContCnt)
{
    if (!distal_tube_empty()) {
        (*pFullCnt)++;
    } else {
        *pFullCnt = 0;
    }

    if (*pTimer > (PRIME_TIME_BAG_TO_SENSOR + 15)) {
        if (*pFullCnt > 10) {
            *pContCnt = *pFullCnt;
            return true;  // 阶段完成
        }
    }
    return false;  // 阶段未完成
}

/**
  * @brief 判断是否需要中止营养液阶段
  */
static bool ShouldAbortFormulaStage(uint16_t timer, uint16_t full_cnt)
{
    return (timer > (PRIME_TIME_BAG_TO_SENSOR + 15)) && (full_cnt <= 10);
}


/**************************************
 * autoprimesingle()
 * @short		Auto prime single set.
 * @param		void
 * @retval  	void
 * @detail
 * Automatically prime the single set (nutrition only). Start priming after prime
 * button is pressed. Close the bolus side TPA and pump nutrition until feeding tube
 * is filled. Operation is based on predetermined timing.
 */



/**************************************************************
 * @brief 保存触摸屏校准参数到EEPROM
 * @param calib 校准数据结构体指针
 * @return 校验和
 * @note 保存6个变换矩阵参数并计算校验和
 **************************************************************/
uint32_t eve_calib_write(struct touchscreen_calibration *calib)
{
    uint32_t chksum = 0;

    // 保存6个变换参数并累加校验和
    for (uint8_t i = 0; i < 6; i++) {
        SaveNV32(calib->transform[i], EVE_TRANSFORM_REG_A + i*4);
        chksum += calib->transform[i];
    }

    SaveNV32(chksum, EVE_TRANSFORM_CS);
    return chksum;
}



/**************************************************************
 * @brief 从EEPROM读取触摸屏校准参数
 * @param calib 校准数据结构体指针
 * @return 0-校验成功 非0-校验失败
 * @note 校验失败会记录错误日志
 **************************************************************/
int8_t eve_calib_read(struct touchscreen_calibration *calib)
{
    uint32_t chksum = 0;

    // 读取并累加6个变换参数
    for (uint8_t i = 0; i < 6; i++) {
        calib->transform[i] = ReadNV32(EVE_TRANSFORM_REG_A + i*4);
        chksum += calib->transform[i];
    }

    // 校验检查
    int32_t ret = chksum - ReadNV32(EVE_TRANSFORM_CS);
    if (ret != 0) {
        save_err_rec(ERR_TOUCH_CAL);
    }
    return ret;
}



/**************************************
 * @brief 非易失性变量范围检查
 * @note 使用宏定义统一检查逻辑：
 *       - CHECK_VOLUME: 通用体积参数检查（0-9999）
 *       - CHECK_SETTING: 设置参数检查（0-max）
 *       特殊处理：
 *       - G_hour_volume 单独处理（上限400）
 */
void check_nv_vars()
{
	int8_t vol_changed = 0;    // 体积相关变量变更标志
	int8_t setting_changed = 0; // 设置类变量变更标志
	int8_t acc_changed = 0;     // 累计变量变更标志

    /* 食物流速检查 */
    if (G_food_rate < FOOD_RATE_DEFAULT || G_food_rate > FULL_FOOD_RATE) {
        G_food_rate = FOOD_RATE_DEFAULT;
        vol_changed = 1;
    }

    /* 通用体积参数检查 */
	CHECK_VOLUME(G_volume, vol_changed);
	CHECK_VOLUME(G_dose_limit, vol_changed);
	CHECK_VOLUME(G_total_volume, vol_changed);
	CHECK_VOLUME(G_water, vol_changed);

    /* 特殊处理 - 小时体积（单独检查） */
    if (G_hour_volume < 0 || G_hour_volume > HOUR_VOLUME_MAX) {
        G_hour_volume = 0;
        vol_changed = 1;
    }

    /* 水流速检查 */
    if (G_water_rate < WATER_RATE_DEFAULT || G_water_rate > MAX_WATER_RATE) {
        G_water_rate = WATER_RATE_DEFAULT;
        vol_changed = 1;
    }

    /* 设置参数检查 */
	CHECK_SETTING(G_alarm_vol, 1, setting_changed);
	CHECK_SETTING(G_warning_vol, 1, setting_changed);
	CHECK_SETTING(G_display_brightness, 1, setting_changed);
	CHECK_SETTING(G_dose_complete_callback_index, 2, setting_changed);
	CHECK_SETTING(G_svc_entry_delay, 2, setting_changed);
	/* 累计变量检查 */
	if (G_acc_formula_ml > 10000000) {
		G_acc_formula_ml = 0;
		acc_changed = 1;
	}
	if (G_acc_bolus_ml > 10000000) {
		G_acc_bolus_ml = 0;
		acc_changed = 1;
	}
	if (G_acc_bat_time_min > 200000) {
		G_acc_bat_time_min = 0;
		acc_changed = 1;
	}

	/* 分段保存 */
	if (vol_changed)    SaveNV_Volume();
	if (setting_changed) SaveNV_Settings();
	if (acc_changed)    SaveNV_Acc_Volume();
}


/**************************************
 * check_nv_vars()
 * @detail
 * Check if variables are within the range. Restore default value and save in EEPROM.
 */


/**************************************
 *check_err_data()
 *@detail
 *Check if error data has been initialized (head address <> 0xffff). Set head to 0 if no initial value.
 *Head equal to 0 indicates that no data exist
 */
void check_err_data(void)
{
uint16_t last_error_location;

  last_error_location = ReadNV16(ERR_MSG_HEAD);
  if (last_error_location > ERR_MSG_MAX_NO_OF){
	  last_error_location = 0;
	  SaveNV16(0,ERR_MSG_HEAD);
  }
}/** End check_err_data() **/


/**************************************
 * end_startup_init()
 * @detail
 * Execute remaining startup tasks and wait to display the start screen additional few
 * seconds.
 */
void end_startup_init(void){

  beep_1khz_1sec(NORMAL_TONE);


  return ;
}/** End end_startup_init()  **/

/**************************************************************
 * @brief 电源测试（启动时）
 * @note 检测电池电压，电压不足时显示警告
 **************************************************************/
void initial_power_test(void)
{
    uint16_t battery_voltage = read_battery_voltage();

    // AC电源状态指示
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,
        ac_connected() ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // 电池电压不足处理
    if (!ac_connected() && battery_voltage <= EMPTY_BATTERY_TEST_VOLTAGE) {//LOW_BATTERY_VOLTAGE
        show_plug_ac_message();
        if (!ac_connected()) {
            save_err_rec(ERR_EMPTY_BAT);
            power_off_pump();
        }
    }
}



/**************************************
 * @short			StartMode
 * @param			void
 * @retval  	    uint8_t task_num
 *
 * @detail
 * Display start screen and retrieve the data from NVRAM
 *
 */
/**
 * @brief 电源状态检测
 * @return 电源状态枚举值
 * @note 控制LED指示状态，低电压时警告
 */
/**************************************
 * power_test()
 * @detail
 * Green led if AC power used. Amber on if on battery power.
 * check battery voltage. service battery indicator. Shut off pump if 15 with low battery
 */
uint8_t power_test(void)
{
#if(0)	 // Guard Clause 1: 检查AC供电
    if (ac_connected()) {
       // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    	LED_AMB_ON()();
        return AC_CONNECTED;
    }
    /* 注意：此处省略else，因为前序条件已通过return退出 */
   // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    else{
    	LED_AMB_OFF();
    }
#endif
    // Guard Clause 2: 检查电池电压
    if (G_processed_battery_voltage  < EMPTY_BATTERY_VOLTAGE) {
        pump_motor_off();
        ir_heat_off();
        uint8_t status = show_empty_battery_warning();

        if (status == AC_CONNECTED) return AC_RESTORED;
        if (status == EMPTY_BATTERY) {
            save_nv();
            power_off_pump();
        }
    }

    return (G_v_battery < LOW_BATTERY_VOLTAGE) ? LOW_BATT : BATTERY_OK;
}




/**************************************
 * @brief 保存单字节数据到EEPROM
 * @param var8 要保存的8位数据
 * @param addr EEPROM目标地址
 * @note 实际写入时间约5ms（含保护延时）
 *       函数原型：void SaveNV8(uint8_t var8, uint16_t addr)
 */

/**************************************
 * SaveNV8()
 * @detail      Save single 8-bit data byte to given address in eeprom.
 */
void SaveNV8(uint8_t var8,uint16_t addr)
{
	//HAL_StatusTypeDef ret;
	uint8_t buf[5];

	buf[0] = var8;
	HAL_I2C_Mem_Write(&hi2c3,0xae, addr,2, buf, 1, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}/** End SaveNV8() **/

/**************************************
 * SaveNV16()
 * @brief      保存16位数据到EEPROM指定地址
 * @detail     以小端模式存储16位数据到EEPROM
 * @param[in]  var16 要保存的16位数据
 * @param[in]  addr EEPROM存储地址
 */
void SaveNV16(uint16_t var16, uint16_t addr)
{
	uint8_t buf[2];

	// 小端模式存储：低字节在前
	buf[0] = var16 & 0xFF;        // 存储低8位
	buf[1] = (var16 >> 8) & 0xFF; // 存储高8位

	// 使用I2C写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 2, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY); // 写入延迟
}


/**************************************
 * SaveNV32()
 * @brief      保存32位数据到EEPROM指定地址
 * @detail     以小端模式存储32位数据到EEPROM
 * @param[in]  var32 要保存的32位数据
 * @param[in]  addr EEPROM存储地址
 */
void SaveNV32(uint32_t var32, uint16_t addr)
{
	uint8_t buf[4];

	// 小端模式存储：低字节在前
	buf[0] = var32 & 0xFF;         // 0-7位
	buf[1] = (var32 >> 8) & 0xFF;  // 8-15位
	buf[2] = (var32 >> 16) & 0xFF; // 16-23位
	buf[3] = (var32 >> 24) & 0xFF; // 24-31位

	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 4, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}



/**************************************************************
 * save_tir_cal()
 * @brief      保存TIR校准值和校验和到EEPROM
 * @param[in]  pd2 PD2校准值
 * @param[in]  dac DAC校准值
 */
void save_tir_cal(uint16_t pd2, uint16_t dac)
{
	int16_t chksum;

	// 保存两个校准值
	SaveNV16(pd2, PD2_CALIBRATION_VALUE);
	SaveNV16(dac, DAC_CALIBRATION_VALUE);

	// 计算并保存校验和（两个值的和）
	chksum = pd2 + dac;
	SaveNV16(chksum, CALIBRATION_CHECK_SUM);
}

/**************************************************************
 * read_tir_cal()
 * @brief      从EEPROM读取TIR校准值和校验和
 * @param[out] pd2  PD2校准值指针
 * @param[out] dac  DAC校准值指针
 * @return     校验和
 */
uint16_t read_tir_cal(int16_t *pd2, int16_t *dac)
{
	int16_t chksum;

	// 读取两个校准值
	*pd2 = ReadNV16(PD2_CALIBRATION_VALUE);
	*dac = ReadNV16(DAC_CALIBRATION_VALUE);

	// 读取校验和
	chksum = ReadNV16(CALIBRATION_CHECK_SUM);
	return chksum;
}

  /**************************************************************
   * read_acc_formula_volume()
   * @brief      读取累计配方体积（转换为升）
   * @return     累计配方体积（升）
   */
uint16_t read_acc_formula_volume()
{
    uint32_t acc_formula_volume;

    acc_formula_volume = ReadNV32(ACC_FORMULA_VOLUME_ADDR);
    acc_formula_volume = acc_formula_volume / 1000; // 毫升转升

    // 确保不超过16位最大值
    if (acc_formula_volume > 0xFFFF)
      acc_formula_volume = 0xFFFF;

    return (uint16_t)acc_formula_volume;
}


/**************************************************************
 * read_acc_bolus_volume()
 * @brief      读取累计推注体积（转换为升）
 * @return     累计推注体积（升）
 */
uint16_t read_acc_bolus_volume()
{
	uint32_t acc_bolus_volume;

	acc_bolus_volume = ReadNV32(ACC_BOLUS_VOLUME_ADDR);
	acc_bolus_volume = acc_bolus_volume / 1000; // 毫升转升

	// 确保不超过16位最大值
	if (acc_bolus_volume > 0xFFFF)
		acc_bolus_volume = 0xFFFF;

	return (uint16_t)acc_bolus_volume;
}

/**************************************************************
 * read_acc_bat_time()
 * @brief      读取累计电池供电时间（转换为小时）
 * @return     累计电池供电时间（小时）
 */
uint16_t read_acc_bat_time()
{
	uint32_t acc_bat_time;

	acc_bat_time = ReadNV32(ACC_TIME_ON_BAT_ADDR);
	acc_bat_time = acc_bat_time / 60; // 分钟转小时

	// 确保不超过16位最大值
	if (acc_bat_time > 0xFFFF)
		acc_bat_time = 0xFFFF;

	return (uint16_t)acc_bat_time;
}



/**************************************
 * SaveNV_Settings()
 * @brief      保存程序设置到EEPROM
 */
void SaveNV_Settings(void)
{
//	HAL_StatusTypeDef ret;
	uint8_t buf[5];
	uint16_t addr = SETTINGS_START_ADDR;

	// 打包设置数据
	buf[0] = G_alarm_vol;
	buf[1] = G_warning_vol;
	buf[2] = G_display_brightness;
	buf[3] = G_dose_complete_callback_index;
	buf[4] = G_svc_entry_delay;

	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 5, 1000);

	HAL_Delay(EEPROM_WRITE_DELAY);
}

/**************************************
为什么去掉ret？
无效的错误处理：第二个版本虽然获取了返回值ret，但if (ret){}实际上不做任何处理，这种空错误处理和不检查返回值本质上没有区别
代码简洁性：第一个版本更简洁，去掉了无实际作用的变量和判断减少了不必要的代码行数，提高可读性
性能考虑：去掉ret可以节省少量栈空间（不需要存储返回值），减少一个条件判断的指令周期
实际应用场景：在EEPROM写入操作中，如果失败通常只能重试或记录错误，如果应用场景对写入成功率要求不高，可以简化处理 */


/**************************************
 * SaveNV_Volume()
 * @brief      保存当前食物速率和体积到EEPROM
 */
void SaveNV_Volume(void)
{
	uint8_t buf[14];
	uint16_t addr = VOLUME_START_ADDR;

	// 打包体积数据（全部16位值，小端模式）
	buf[0] = G_food_rate & 0xFF;
	buf[1] = (G_food_rate >> 8) & 0xFF;

	buf[2] = G_volume & 0xFF;
	buf[3] = (G_volume >> 8) & 0xFF;

	buf[4] = G_total_volume & 0xFF;
	buf[5] = (G_total_volume >> 8) & 0xFF;

	buf[6] = G_dose_limit & 0xFF;
	buf[7] = (G_dose_limit >> 8) & 0xFF;

	buf[8] = G_water_rate & 0xFF;
	buf[9] = (G_water_rate >> 8) & 0xFF;

	buf[10] = G_hour_volume & 0xFF;
	buf[11] = (G_hour_volume >> 8) & 0xFF;

	buf[12] = G_water & 0xFF;
	buf[13] = (G_water >> 8) & 0xFF;


	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 14, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}



/**************************************
 * SaveNV_Acc_Volume()
 * @brief      保存累计体积数据到EEPROM
 * @detail     包括配方体积(mL)、推注体积(mL)和电池供电时间(min)
 */
void SaveNV_Acc_Volume(void)
{
	uint8_t buf[12];
	uint16_t addr = ACC_VOLUME_START_ADDR;

	// 打包累计数据（全部32位值，小端模式）
	buf[0] = G_acc_formula_ml & 0xFF;
	buf[1] = (G_acc_formula_ml >> 8) & 0xFF;
	buf[2] = (G_acc_formula_ml >> 16) & 0xFF;
	buf[3] = (G_acc_formula_ml >> 24) & 0xFF;

	buf[4] = G_acc_bolus_ml & 0xFF;
	buf[5] = (G_acc_bolus_ml >> 8) & 0xFF;
	buf[6] = (G_acc_bolus_ml >> 16) & 0xFF;
	buf[7] = (G_acc_bolus_ml >> 24) & 0xFF;

	buf[8] = G_acc_bat_time_min & 0xFF;
	buf[9] = (G_acc_bat_time_min >> 8) & 0xFF;
	buf[10] = (G_acc_bat_time_min >> 16) & 0xFF;
	buf[11] = (G_acc_bat_time_min >> 24) & 0xFF;

	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 12, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}


/**************************************
 * save_cal_rec()
 * @brief      保存校准记录到EEPROM
 * @param[in]  dac_val DAC值
 * @param[in]  pd2_val PD2值
 */
void save_cal_rec(uint16_t dac_val, uint16_t pd2_val)
{
	chip_time timo;
	uint8_t buf[8];
	uint16_t addr, head;

	// 获取当前记录头指针
	head = ReadNV16(CAL_REC_HEAD);

	// 检查指针有效性
	if ((head == 0) || (head > (CAL_REC_MAX_NO_OF - 1))) {
		head = 0;
	}
	head++; // 移动到下一个记录位置

	// 保存新指针
	SaveNV16(head, CAL_REC_HEAD);

	// 计算绝对地址
	addr = (head - 1) * 8 + CAL_REC_DATA_START;

	// 获取当前时间
	get_time(&timo);

	// 打包记录数据
	buf[0] = dac_val & 0xFF;
	buf[1] = (dac_val >> 8) & 0xFF;
	buf[2] = pd2_val & 0xFF;
	buf[3] = (pd2_val >> 8) & 0xFF;
	buf[4] = timo.months;
	buf[5] = timo.days;
	buf[6] = timo.years;

	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 8, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}

/**************************************
 * save_err_rec()
 * @brief      保存错误记录到EEPROM
 * @param[in]  err_num 错误号
 */
void save_err_rec(uint8_t err_num)
{
	chip_time timo;
	uint8_t buf[8];
	uint16_t addr, head;

	// 获取当前错误记录头指针
	head = ReadNV16(ERR_MSG_HEAD);

	// 检查指针有效性
	if ((head == 0) || (head > (ERR_MSG_MAX_NO_OF - 1))) {
		head = 0;
	}
	head++; // 移动到下一个记录位置

	// 保存新指针
	SaveNV16(head, ERR_MSG_HEAD);

	// 计算绝对地址
	addr = (head - 1) * 8 + ERR_MSG_DATA_START;

	// 获取当前时间
	get_time(&timo);

	// 打包错误记录
	buf[0] = err_num;
	buf[1] = timo.hours;
	buf[2] = timo.minutes;
	buf[3] = timo.period;
	buf[4] = timo.months;
	buf[5] = timo.days;
	buf[6] = timo.years;

	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 8, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}



/**************************************
 * save_daily_vol_rec()
 * @brief      保存每日体积记录到EEPROM
 * @param[in]  rec_no 记录号
 * @param[in]  rec 体积记录指针
 */
void save_daily_vol_rec(uint16_t rec_no, vol_rec *rec)
{
	uint8_t buf[7];
	uint16_t addr;

	// 检查记录号有效性
	if (rec_no > (DLVD_VOL_MAX_NO_OF - 1)) {
		rec_no = 0;
		SaveNV16(rec_no, DLVD_VOL_HEAD);
	}

	// 计算绝对地址
	addr = (rec_no - 1) * 8 + DLVD_DAILY_DATA_START;

	// 打包记录数据
	buf[0] = rec->food_vol & 0xFF;
	buf[1] = (rec->food_vol >> 8) & 0xFF;
	buf[2] = rec->water_vol & 0xFF;
	buf[3] = (rec->water_vol >> 8) & 0xFF;
	buf[4] = rec->months;
	buf[5] = rec->days;
	buf[6] = rec->years;

	// 写入EEPROM
	HAL_I2C_Mem_Write(&hi2c3, 0xAE, addr, 2, buf, 7, 1000);
	HAL_Delay(EEPROM_WRITE_DELAY);
}


/**************************************
 * update_daily_volumes()
 * @brief      更新每日食物和水的体积记录
 * @param[in]  food_vol 食物体积
 * @param[in]  water_vol 水体积
 */
/**************************************
 * update_daily_volumes()
 * @detail
 * Use to save any partial food or water volume to daily volume log.
 * Normally the daily volume is saved after formula or bolus cycle ends.
 * This function is called if the pumping is interupted by user or by another event.
 */
void update_daily_volumes(uint16_t food_vol, uint16_t water_vol)
{
	update_daily_food_vol(food_vol);
	update_daily_water_vol(water_vol);
}/** End update_daily_volumes() **/


/**************************************
 * update_daily_food_vol()
 * @brief      更新每日食物体积记录
 * @param[in]  vol 要添加的食物体积
 */
void update_daily_food_vol(uint16_t vol)
{
	vol_rec rec;
	chip_time timo;
	uint16_t rec_no;

	// 获取当前时间
	get_time(&timo);

	// 读取当前记录指针
	rec_no = ReadNV16(DLVD_VOL_HEAD);

	// 检查指针有效性
	if (rec_no > (DLVD_VOL_MAX_NO_OF - 1)) {
		rec_no = 0;
	}

	// 获取当前记录
	get_vol_rec(rec_no, &rec);

	// 检查是否为同一天
	if ((timo.months == rec.months) && (timo.days == rec.days) && (timo.years == rec.years)) {
		// 同一天，累加体积
		rec.food_vol += vol;
	} else {
    // 新的一天，创建新记录
		rec_no++;
		if (rec_no > (DLVD_VOL_MAX_NO_OF - 1)) {
			rec_no = 0;
		}
		rec.food_vol = vol;
		rec.water_vol = 0;
		rec.months = timo.months;
		rec.days = timo.days;
		rec.years = timo.years;
	}

  // 保存更新后的记录
  SaveNV16(rec_no, DLVD_VOL_HEAD);
  save_daily_vol_rec(rec_no, &rec);
}

/**************************************
 * update_daily_water_vol()
 * @brief      更新每日水体积记录
 * @param[in]  vol 要添加的水体积
 */
void update_daily_water_vol(uint16_t vol)
{
	vol_rec rec;
	chip_time timo;
	uint16_t rec_no;

	// 获取当前时间
	get_time(&timo);

	// 读取当前记录指针
	rec_no = ReadNV16(DLVD_VOL_HEAD);

	// 检查指针有效性
	if (rec_no > (DLVD_VOL_MAX_NO_OF - 1)) {
		rec_no = 0;
	}

	// 获取当前记录
	get_vol_rec(rec_no, &rec);

	// 检查是否为同一天
	if ((timo.months == rec.months) && (timo.days == rec.days) && (timo.years == rec.years)) {
		// 同一天，累加体积
		rec.water_vol += vol;
	} else {
		// 新的一天，创建新记录
		rec_no++;
		if (rec_no > (DLVD_VOL_MAX_NO_OF - 1)) {
			rec_no = 0;
		}
		rec.food_vol = 0;
		rec.water_vol = vol;
		rec.months = timo.months;
		rec.days = timo.days;
		rec.years = timo.years;
	}

	// 保存更新后的记录
	SaveNV16(rec_no, DLVD_VOL_HEAD);
	save_daily_vol_rec(rec_no, &rec);
}


/**
 * @brief 从EEPROM指定地址读取8位数据
 * @param addr 要读取的EEPROM地址
 * @return 读取到的8位数据
 * @detail 使用I2C从EEPROM读取单个字节数据
 */
/**************************************
 * ReadNV8()
 * @detail      read single 8-bit data byte from given address in eeprom.
 */
uint8_t ReadNV8(uint16_t addr)
{

	uint8_t buf[5];
	uint8_t var8;

	HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 1, 1000);
	var8 = buf[0];
	return var8;
}/** End ReadNV8() **/

/**
 * @brief 从EEPROM指定地址读取16位数据
 * @param addr 要读取的EEPROM地址
 * @return 读取到的16位数据
 * @detail 使用I2C从EEPROM读取两个字节数据并组合成16位值(小端格式)
 */
uint16_t ReadNV16(uint16_t addr)
{
    uint8_t buf[5];
    uint16_t var16;

    HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 2, 1000);
    var16 = (buf[1] << 8) | buf[0];  // 合并为16位值
    return var16;
}

/**************************************
 * ReadNV16()
 * @detail      read single 16-bit data byte from given address in eeprom.
 */

#if(0)
uint16_t ReadNV16(uint16_t addr)
{
  uint8_t buf[5];
  uint16_t var16;
  HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 2, 1000);
  var16 = buf[1];
  var16 = var16 << 8;
  var16 = var16 + buf[0];
  return var16;
}
#endif
/** End ReadNV16() **/

/**
 * @brief 从EEPROM指定地址读取32位数据
 * @param addr 要读取的EEPROM地址
 * @return 读取到的32位数据
 * @detail 使用I2C从EEPROM读取四个字节数据并组合成32位值(小端格式)
 */
uint32_t ReadNV32(uint16_t addr)
{
    uint8_t buf[5];
    uint32_t var32;

    HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 4, 1000);
    var32 = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];  // 合并为32位值
    return var32;
}

/**************************************
 * ReadNV32()
 * @detail      read single 32-bit data byte from given address in eeprom.
 */

#if(0)
uint32_t ReadNV32(uint16_t addr)
{
  uint8_t buf[5];
  uint32_t var32;
  HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 4, 1000);
  var32 = buf[3];
  var32 = var32 << 8;
  var32 = var32 + buf[2];
  var32 = var32 << 8;
  var32 = var32 + buf[1];
  var32 = var32 << 8;
  var32 = var32 + buf[0];
  return var32;
}
#endif
/** End ReadNV32() **/

/**
 * @brief 从EEPROM读取程序设置参数到全局变量
 * @detail 从EEPROM的设置起始地址读取5个设置参数到全局变量
 */
/**************************************
 * read_nv_setting_vars()
 * @detail
 * Read program setting parameters from eeprom to global variables
 */
void read_nv_setting_vars(void)
{
	uint8_t buf[7];
	uint16_t addr;

	addr = SETTINGS_START_ADDR;

	HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 5, 1000);

	G_alarm_vol = buf[0];
	G_warning_vol = buf[1];
	G_display_brightness = buf[2];
	G_dose_complete_callback_index = buf[3];
	G_svc_entry_delay = buf[4];
}/** End read_nv_setting_vars() **/


/**
 * @brief 从EEPROM读取体积参数到全局变量
 * @detail 从EEPROM的体积起始地址读取6个16位体积参数到全局变量
 */
void read_nv_volume_vars(void)
{
    uint8_t buf[14];
    uint16_t addr = VOLUME_START_ADDR;
  //  uint16_t var16;

    HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 14, 1000);

    // 读取并组合各16位参数
    G_food_rate = (buf[1] << 8) | buf[0];
    G_volume = (buf[3] << 8) | buf[2];
    //G_volume = 2800;
    G_total_volume = (buf[5] << 8) | buf[4];
    //G_total_volume = 2800;
    G_dose_limit = (buf[7] << 8) | buf[6];
    G_water_rate = (buf[9] << 8) | buf[8];
    G_hour_volume = (buf[11] << 8) | buf[10];
    G_water =(buf[13] << 8) | buf[12];
    if(G_water<10){
    	G_water = 0;
    }
}
/**************************************
 * read_nv_volume_vars()
 @detail
 * Read volume parameters from eeprom to global variables
 */

#if(0)
void read_nv_volume_vars(void)
{
  uint8_t buf[12];
  uint16_t addr,var16;
  addr = VOLUME_START_ADDR;
  HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 12, 1000);
  var16 = buf[1];
  var16 = (var16 << 8) + buf[0];
  G_food_rate = var16;     				//data
  var16 = buf[3];
  var16 = (var16 << 8) + buf[2];
  G_volume = var16;     				//data
  var16 = buf[5];
  var16 = (var16 << 8) + buf[4];
  G_total_volume = var16;     			//data
  var16 = buf[7];
  var16 = (var16 << 8) + buf[6];
  G_dose_limit = var16;    				//data
  var16 = buf[9];
  var16 = (var16 << 8) + buf[8];
  G_water_rate = var16;    				//data
  var16 = buf[11];
  var16 = (var16 << 8) + buf[10];
  G_hour_volume = var16;    			//data
}
#endif
/** End read_nv_volume_vars() **/

/**
 * @brief 从EEPROM读取累计体积参数
 * @detail 从EEPROM的累计体积起始地址读取3个32位累计参数到全局变量
 */
void read_nv_acc_volume_vars(void)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[14];
    uint16_t addr = ACC_VOLUME_START_ADDR;
  //  uint32_t var32;

    ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 12, 1000);
    if (ret != HAL_OK) { /* 处理错误 */ }

    // 读取并组合各32位参数
    G_acc_formula_ml = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    G_acc_bolus_ml = (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | buf[4];
    G_acc_bat_time_min = (buf[11] << 24) | (buf[10] << 16) | (buf[9] << 8) | buf[8];
}
/**************************************
 * read_nv_acc_volume_vars()
 * @detail
 * Read accumulated volume parameters from EEPROM
 */

#if(0)
void read_nv_acc_volume_vars(void)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[14];
  uint16_t addr;
  uint32_t var32;
  addr = ACC_VOLUME_START_ADDR;
  ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 12, 1000);
  if ( ret != HAL_OK ) {		 }
  var32 = buf[3];
  var32 = var32 << 8;
  var32 = var32 + buf[2];
  var32 = var32 << 8;
  var32 = var32 + buf[1];
  var32 = var32 << 8;
  var32 = var32 + buf[0];
  G_acc_formula_ml = var32;
  var32 = buf[7];
  var32 = var32 << 8;
  var32 = var32 + buf[6];
  var32 = var32 << 8;
  var32 = var32 + buf[5];
  var32 = var32 << 8;
  var32 = var32 + buf[4];
  G_acc_bolus_ml = var32;
  var32 = buf[11];
  var32 = var32 << 8;
  var32 = var32 + buf[10];
  var32 = var32 << 8;
  var32 = var32 + buf[9];
  var32 = var32 << 8;
  var32 = var32 + buf[8];
  G_acc_bat_time_min = var32;
}
#endif
/** End read_nv_acc_volume_vars() **/
/**
 * @brief 初始化板级IO
 * @detail 初始化各种硬件IO状态，包括电池测量、IR LED、超声波传感器电源等
 */
void board_io_initialize()
{
 //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // 使能电池测量
	READ_BATT_ON();
    HAL_Delay(100);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // 设置IR LED，管道检测电流
    LED_HEAT_OFF();  // 关闭IR高电流
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // 开启IR
    LED_PWM_ON();
   // HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_SET);   // 开启超声波传感器电源
    USONIC_PWR_ON();
}
/**************************************
 * board_io_initialize()
 * @detail
 * Initialize board io
 */

#if(0)
void board_io_initialize()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   //enable battery measurement
  HAL_Delay(100);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); //set IR LED, tubing detection current
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);  //IR hi off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   //IR on
  HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_SET);   //Usonic sensor power on
}
#endif
/** End board_io_initialize() **/

/**************************************
 * ir_heat_off()
 * @detail
 * IR heat off (IR LED high current off).
 */
void ir_heat_off(void)
{
	LED_HEAT_OFF();
}/** End ir_heat_off() **/

/**************************************
 * ir_heat_on()
 * @detail
 * IR heat power on (IR LED high current on).
 */
void ir_heat_on(void)
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	LED_HEAT_ON();
}/** End ir_heat_on() **/

/**
 * @brief 读取一条校准记录
 * @param rec_no 记录编号
 * @param rec 存储读取结果的校准记录结构体指针
 * @detail 从EEPROM读取指定编号的校准记录
 */
void get_cal_rec(uint16_t rec_no, cal_rec *rec)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[9];
    uint16_t addr = ((rec_no - 1) * 8) + CAL_REC_DATA_START;

    ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 7, 1000);
    if (ret != HAL_OK) { /* 处理错误 */ }

    // 填充记录结构体
    rec->dac_cal = (buf[1] << 8) | buf[0];
    rec->pd2_cal = (buf[3] << 8) | buf[2];
    rec->months = buf[4];
    rec->days = buf[5];
    rec->years = buf[6];
}

/**************************************
 * get_cal_rec()
 * @detail
 * Read one calibration record
 */
#if(0)
void get_cal_rec(uint16_t rec_no,cal_rec *rec)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[9];
  uint16_t addr,var16;
  addr = ((rec_no - 1) * 8) + CAL_REC_DATA_START;
  ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 7, 1000);
  if ( ret != HAL_OK ) {		 }
  var16 = buf[1];
  var16 = var16 << 8;
  rec->dac_cal = var16 + buf[0];
  var16 = buf[3];
  var16 = var16 << 8;
  rec->pd2_cal = var16 + buf[2];
  rec->months = buf[4];
  rec->days = buf[5];
  rec->years =buf[6];
}
#endif
/** End get_cal_rec **/
/**
 * @brief 读取一条错误记录
 * @param rec_no 记录编号
 * @param rec 存储读取结果的错误记录结构体指针
 * @detail 从EEPROM读取指定编号的错误记录
 */
void get_err_rec(uint16_t rec_no, err_rec *rec)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[9];
    uint16_t addr = ((rec_no - 1) * 8) + ERR_MSG_DATA_START;

    ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 8, 1000);
    if (ret != HAL_OK) { /* 处理错误 */ }

    // 填充记录结构体
    rec->err_num = buf[0];
    rec->hours = buf[1];
    rec->minutes = buf[2];
    rec->period = buf[3];
    rec->months = buf[4];
    rec->days = buf[5];
    rec->years = buf[6];
}

/**************************************
 * get_err_rec()
 * @detail
 * Read one error record
 */
#if(0)
void get_err_rec(uint16_t rec_no,err_rec *rec)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[9];
  uint16_t addr;
  addr = ((rec_no - 1) * 8) + ERR_MSG_DATA_START;
  ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 8, 1000);
  if ( ret != HAL_OK ) {		 }
  rec->err_num = buf[0];
  rec->hours = buf[1];
  rec->minutes = buf[2];
  rec->period = buf[3];
  rec->months = buf[4];
  rec->days = buf[5];
  rec->years =buf[6];
}
#endif
/** End get_err_rec **/


/**
 * @brief 读取一条每日体积记录
 * @param rec_no 记录编号
 * @param rec 存储读取结果的体积记录结构体指针
 * @detail 从EEPROM读取指定编号的每日体积记录
 */
void get_vol_rec(uint16_t rec_no, vol_rec *rec)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[9];
    uint16_t addr = ((rec_no - 1) * 8) + DLVD_DAILY_DATA_START;

    ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 7, 1000);
    if (ret != HAL_OK) { /* 处理错误 */ }

    // 填充记录结构体
    rec->food_vol = (buf[1] << 8) | buf[0];
    rec->water_vol = (buf[3] << 8) | buf[2];
    rec->months = buf[4];
    rec->days = buf[5];
    rec->years = buf[6];
}
/**************************************
 * get_vol_rec()
 * @detail
 * Read one daily volume record
 */

#if(0)
void get_vol_rec(uint16_t rec_no,vol_rec *rec)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[9];
  uint16_t addr,var16;
  addr = ((rec_no - 1) * 8) + DLVD_DAILY_DATA_START;
  ret = HAL_I2C_Mem_Read(&hi2c3, 0xae, addr, 2, buf, 7, 1000);
  if ( ret != HAL_OK ) {		 }
  var16 = buf[1];
  var16 = var16 << 8;
  rec->food_vol = var16 + buf[0];
  var16 = buf[3];
  var16 = var16 << 8;
  rec->water_vol = var16 + buf[2];
  rec->months = buf[4];
  rec->days = buf[5];
  rec->years =buf[6];
}
#endif
/** End get_vol_rec **/

/**
  * @brief  从RTC读取时间
  * @param  hi2c: I2C句柄指针
  * @param  tim: 时间结构体指针
  * @retval HAL状态
  */
void get_time(chip_time *tim)
{
	uint8_t buf[2];
	g_rtc_status = RTC_OP_BUSY;

	/* 1. 读取分钟 (0x05) */
	buf[0] = RTC_MIN_REG;
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}
	if(HAL_I2C_Master_Receive(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
		tim->minutes = bcd_to_dec(buf[0] & 0x7F); // 清除无效位
		tim->minutes = (tim->minutes > 59) ? 0 : tim->minutes; // 合法性检查
	}

	/* 2. 读取小时和AM/PM (0x06) */
	buf[0] = RTC_HOUR_REG;
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}
	if(HAL_I2C_Master_Receive(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
		tim->period = (buf[0] >> 5) & 0x01;    // 提取AM/PM位
		tim->hours = bcd_to_dec(buf[0] & 0x1F); // 提取小时
		tim->hours = (tim->hours > 12) ? 12 : tim->hours;
	}

	/* 3. 读取日期 (0x07) */
	buf[0] = RTC_DATE_REG;
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}
	if(HAL_I2C_Master_Receive(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
		tim->days = bcd_to_dec(buf[0] & 0x3F); // 清除控制位
		tim->days = (tim->days > 31) ? 1 : tim->days;
	}

	/* 4. 读取月份 (0x09) */
	buf[0] = RTC_MONTH_REG;
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}
	if(HAL_I2C_Master_Receive(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
		tim->months = bcd_to_dec(buf[0] & 0x1F); // 清除控制位
		tim->months = (tim->months > 12) ? 1 : tim->months;
	}

	/* 5. 读取年份 (0x0A) */
	buf[0] = RTC_YEAR_REG;
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}
	if(HAL_I2C_Master_Receive(&hi2c3, RTC_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
		tim->years = bcd_to_dec(buf[0]);
	}

		g_rtc_status = RTC_OP_SUCCESS;

}

/**************************************
 * @detail
 * read RTC and convert the  register values from BCD to decimal
 * Minutes, hours, am/pm, day, year.
 */

#if(0)
void get_time(chip_time *tim)
{
  uint8_t buf[5];
  int16_t val,tens;
  HAL_StatusTypeDef ret;
  tim->minutes = 3;
  buf[0] = 5; 									//register 5, minutes
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {		 }
  else {
    // Read  bytes from the minutes register
	ret = HAL_I2C_Master_Receive(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {		   }
	else {
      tim->minutes =bcd_to_dec(buf[0]);
      if (tim->minutes > 59) tim->minutes = 0;
	}
  }
  buf[0] = 6; 									//register 6, hours
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {		 }
  else {
	// Read  bytes from the seconds register
	ret = HAL_I2C_Master_Receive(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {		   }
	else {
	  val =buf[0];
	  tim->period = val>>5;  					// bit5 am=0 pm=1
	  tim->hours = val & 0x1F;    		//remove am-pm bit
	  tim->hours = bcd_to_dec(tim->hours);
	  if (tim->hours > 12) tim->hours = 12;
	}
  }
  buf[0] = 7; 									//register 7, date
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {		 }
  else {
	// Read  bytes from the seconds register
	ret = HAL_I2C_Master_Receive(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {		   }
	else {
	  tim->days = bcd_to_dec(buf[0]);
	  if (tim->days > 31) tim->days = 1;
	}
  }
  buf[0] = 9; 									//register 9, month
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {		 }
  else {
    // Read  bytes from the seconds register
    ret = HAL_I2C_Master_Receive(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {		   }
    else {
      val =buf[0];
	  tens = val>>4;
	  tim->months = (val & 15)+10*tens;
	  tim->months = bcd_to_dec(tim->months);
	  if (tim->months > 12) tim->months = 1;
	}
  }
  buf[0] = 10; 									//register 10, year
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {		 }
  else {
	 // Read  bytes from the seconds register
	 ret = HAL_I2C_Master_Receive(&hi2c3, 0xa2, buf, 1, HAL_MAX_DELAY);
     if ( ret != HAL_OK ) {		   }
	 else {
	   tim->years = bcd_to_dec(buf[0]);
	 }
  }
}
#endif
/** End get_time() **/


/**
  * @brief  保存时间到RTC
  * @param  tim: 时间结构体指针
  * @retval HAL状态
  */
void save_time(chip_time *tim)
{
	uint8_t buf[2];
	g_rtc_status = RTC_OP_BUSY;

	/* 0. 设置12小时模式 (0x00) */
	buf[0] = RTC_CTRL1_REG;
	buf[1] = 0x02; // 12小时模式
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	/* 1. 写入年份 (0x0A) */
	buf[0] = RTC_YEAR_REG;
	buf[1] = dec_to_bcd(tim->years);
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	/* 2. 写入月份 (0x09) */
	buf[0] = RTC_MONTH_REG;
	buf[1] = dec_to_bcd(tim->months);
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	/* 3. 写入日期 (0x07) */
	buf[0] = RTC_DATE_REG;
	buf[1] = dec_to_bcd(tim->days);
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	/* 4. 写入小时和AM/PM (0x06) */
	buf[0] = RTC_HOUR_REG;
	buf[1] = dec_to_bcd(tim->hours) | (tim->period << 5);
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	/* 5. 写入分钟 (0x05) */
	buf[0] = RTC_MIN_REG;
	buf[1] = dec_to_bcd(tim->minutes);
	if(HAL_I2C_Master_Transmit(&hi2c3, RTC_I2C_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		g_rtc_status = RTC_OP_FAILED;
		return;
	}

	g_rtc_status = RTC_OP_SUCCESS;
}


/**************************************
 * save_time()
 * @detail
 * Convert the  time values from decimal to BCD and save to RTC.
 * Minutes, hours, am/pm, day, year.
 */

#if(0)
void save_time(chip_time *tim)
{
  uint8_t buf[5];
  int16_t val;
  HAL_StatusTypeDef ret;
  buf[0] = 0; 			//register 0, control1
  buf[1] =0x02; 		//bit 1 12 hour mode
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  buf[0] = 10; 			//register 10, year
  buf[1] = dec_to_bcd(tim->years);
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  buf[0] = 9; 			//register 9, month
  buf[1] =dec_to_bcd(tim->months);
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  buf[0] = 7; 			//register 7, date
  buf[1] =dec_to_bcd(tim->days);
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  buf[0] = 6; 			        //register 6, hour
  val =tim->period <<5;  		// bit5 am=0 pm=1
  tim->hours = dec_to_bcd(tim->hours) | val;
  buf[1] =tim->hours;
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  buf[0] = 5; 			//register 5, minute
  buf[1] =dec_to_bcd(tim->minutes);
  ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
  if ((ret) != HAL_OK) {  }
  }
#endif

/** End save_time() **/

/**************************************************************
 * @brief 生成按键提示音（触觉反馈）
 * @note 通过GPIO引脚快速切换产生短脉冲声
 **************************************************************/
void key_click()
{
    uint16_t i;
    // 同步两个GPIO引脚状态
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }
    // 生成2个500us周期的脉冲
    for (i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        delay_us(500);
    }
}

/**************************************
 * key_click()
 * @short		Generate audiple click. Used for touch feedback.
 * @param		void
 * @retval  	void
 * @detail
 * Generate audiple click. Used for touch feedback.
 */

#if(0)
void key_click()
{
uint16_t i;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  for (i=0; i<2; i++){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	 /* if (loud){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	  } */
	delay_us(500);
  }
}
#endif
/** End key_click() **/

/**************************************************************
 * @brief 生成1kHz提示音并读取报警监控值
 * @param loud 是否启用大声模式
 * @return 报警监控ADC平均值
 * @note 持续1秒（2000个500us周期）
 **************************************************************/
uint16_t beep_1khz_1sec_mon(uint8_t loud)
{
    uint16_t i, alrm_mon_sum = 0;

    stop_analog_scan();
    // 初始化GPIO状态
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 生成1kHz音调（2000个周期）
    for (i = 0; i < 2000; i++) {
        delay_us(500);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 读取报警监控通道
    select_analog_channel(CH_ALRM_MON);
    for (i = 0; i < 5; i++)
        alrm_mon_sum += read_analog_channel();

    start_analog_scan();
    return alrm_mon_sum / 5;
}

/**************************************
 * beep_1khz_1sec_mon()
 * @detail
 * Generate 1khz audiple tone for a half second period.
 * Then read the alarm monitor and return the value.
 */

#if(0)
uint16_t beep_1khz_1sec_mon(uint8_t loud)
{
  uint16_t i, alrm_mon_sum;
  stop_analog_scan();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)){
  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  for (i=0; i<2000; i++){
	delay_us(500);
  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
  	if (loud){
  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  	}
  }
  select_analog_channel(CH_ALRM_MON);
  for (i=0; i<5; i++) alrm_mon_sum = alrm_mon_sum + read_analog_channel();
  alrm_mon_sum = alrm_mon_sum/5;
  start_analog_scan();
  return alrm_mon_sum;
}
#endif
/** End beep_1khz_1sec_mon **/

/**************************************************************
 * @brief 生成1kHz持续音频（1秒）
 * @param loud 是否启用高音量模式
 * @note 期间检测RUN/STOP按键可提前终止
 **************************************************************/
void beep_1khz_1sec(uint8_t loud)
{
    stop_analog_scan();

    // 初始化GPIO相位
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 生成2000个周期（1秒）的1kHz方波
    for (uint16_t i = 0; i < 2000; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        delay_us(500);

        // 检测RUN/STOP按键释放
        if (read_keypad() == (RUN_STOP_SWITCH_RELEASED)) break;
    }

    start_analog_scan();
}
/**************************************************************
 * @brief 播放指定频率的音频（带包络，模拟尾音效果）
 * @param freq_hz 频率(Hz)，0表示静默
 * @param duration_ms 持续时间(ms)
 * @param loud 是否启用高音量模式
 * @note 脉冲包络：上升段占10%，持续段占50%，下降段占40%
 *       下降段产生自然的“尾音”衰减效果
 *       期间检测RUN/STOP按键可提前终止
 **************************************************************/
static void beep_tone_with_envelope(uint32_t freq_hz, uint32_t duration_ms, uint8_t loud)
{
    if (freq_hz == 0) {
        for (uint32_t i = 0; i < duration_ms; i++) {
            delay_us(1000);
        }
        return;
    }

    // 固定尾音时长：15ms（不过感觉明显，但三个音一致）
    uint32_t tail_ms = 15;
    uint32_t main_ms = duration_ms - tail_ms;

    // 主体段周期数
    uint32_t main_cycles = (main_ms * freq_hz) / 1000;
    if (main_cycles % 2 != 0) main_cycles++;

    // 尾音段周期数
    uint32_t tail_cycles = (tail_ms * freq_hz) / 1000;
    if (tail_cycles % 2 != 0) tail_cycles++;

    uint32_t half_period_us = 500000 / freq_hz;

    // 初始化GPIO相位
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 主体段：满音量
    for (uint32_t i = 0; i < main_cycles / 2; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        delay_us(half_period_us);
        if (read_keypad() == RUN_STOP_SWITCH_RELEASED) return;
    }

    // 尾音段：逐渐衰减
    for (uint32_t i = 0; i < tail_cycles / 2; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud && i < tail_cycles / 4) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        }
        delay_us(half_period_us);
        if (read_keypad() == RUN_STOP_SWITCH_RELEASED) return;
    }
}
/**************************************************************
 * @brief 播放指定频率的音频（指定时长）
 * @param freq_hz 频率(Hz)，0表示静默
 * @param duration_ms 持续时间(ms)
 * @param loud 是否启用高音量模式
 * @note 期间检测RUN/STOP按键可提前终止
 **************************************************************/
static void beep_tone(uint32_t freq_hz, uint32_t duration_ms, uint8_t loud)
{
    if (freq_hz == 0) {
        // 静默：用delay_us实现毫秒延时
        for (uint32_t i = 0; i < duration_ms; i++) {
            delay_us(1000);
        }
        return;
    }

    // 计算需要生成的周期数（每个周期包含两次GPIO翻转）
    // 周期数 = 时长(ms) × 频率(Hz) / 1000
    uint32_t periods = (duration_ms * freq_hz) / 1000;
    if (periods % 2 != 0) periods++;  // 确保偶数次翻转，结束时电平状态不变

    // 初始化GPIO相位：若两引脚电平相同则翻转PIN_12使两者反相
    // 反相驱动可提升蜂鸣器两端电压摆幅，增大音量
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 生成方波：每个循环产生两个半周期 = 一个完整周期
    for (uint32_t i = 0; i < periods / 2; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);  // 翻转主引脚
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);  // 高音量模式同步翻转副引脚
        delay_us(500000 / freq_hz);  // 半周期延时(us) = 0.5s / 频率

        // 检测RUN/STOP按键释放，支持提前终止
        if (read_keypad() == RUN_STOP_SWITCH_RELEASED) break;
    }
}

/**************************************************************
 * @brief 喂食泵中优先级报警（符合IEC 60601-1-8标准）
 * @param loud 是否启用高音量模式
 * @note 播放C-d-g三音符旋律（Do-Re-Sol）
 *       每个脉冲时长200ms（符合125~250ms要求）
 *       音符间隔50ms，确保脉冲清晰可辨
 *       期间检测RUN/STOP按键可提前终止
 **************************************************************/
void beep_medium_priority_feeding_pump(uint8_t loud)
{
    stop_analog_scan();  // 停止模拟扫描，避免PWM/ADC干扰蜂鸣器时序

    // 喂食泵对应药物/液体输送类别，旋律为C-d-g
    const uint32_t melody[] = {400, 320, 300};  // C(Do), d(Re), g(Sol)

    for (int note_idx = 0; note_idx < 3; note_idx++) {
    	beep_tone_with_envelope(melody[note_idx], 1000, loud);  // 500ms脉冲

        if (note_idx < 2) {
            // 50ms音符间隔（使用delay_us实现）
            for (int i = 0; i < 300; i++) {
                delay_us(1000);
            }
            // 静默期间也检测按键
            if (read_keypad() == RUN_STOP_SWITCH_RELEASED) break;
        }
    }

    start_analog_scan();  // 恢复模拟扫描
}



/**************************************
 * beep_1khz_1sec()
 * @detail
 * Generate 1khz audiple tone for a half of second period
 */

#if(0)
void beep_1khz_1sec(uint8_t loud)
{
  uint16_t i;
  stop_analog_scan();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)){
  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  for (i=0; i<2000; i++){
	delay_us(500);
  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
  	if (loud){
  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  	}
  	if (read_keypad() == (RUN_STOP_SWITCH_RELEASED)) break;
  }
  start_analog_scan();
}
#endif
/** End beep_1khz_1sec **/
/**************************************************************
 * @brief 生成1kHz持续音频（0.5秒）
 * @param loud 是否启用高音量模式
 * @note 固定时长无中断检测
 **************************************************************/
void beep_1khz_p5sec(uint8_t loud)
{
    stop_analog_scan();

    // GPIO相位同步
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 生成1000个周期（0.5秒）的1kHz方波
    for (uint16_t i = 0; i < 1000; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        delay_us(500);
    }

    start_analog_scan();
}

/**************************************************************
 * @brief 生成1kHz持续音频（0.5秒）
 * @param loud 是否启用高音量模式
 * @note 固定时长无中断检测
 **************************************************************/
void beep_1khz_150ms(uint8_t loud)
{
    stop_analog_scan();

    // GPIO相位同步
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    // 生成1000个周期（0.5秒）的1kHz方波
    for (uint16_t i = 0; i < 150; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        if (loud) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        delay_us(1000);
    }

    start_analog_scan();
}

/**************************************
 * beep_1khz_p5sec()
 * @detail
 * Generate 1khz audiple tone for a half of second period
 */

#if(0)
void beep_1khz_p5sec(uint8_t loud)
{
  uint16_t i;

  stop_analog_scan();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  for (i=0; i<1000; i++){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
    if (loud){
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }
    delay_us(500);
  }
  start_analog_scan();
}
#endif
/** End beep_1khz_p5sec() **/


/**************************************************************
 * @brief 选择ADC通道
 * @param channel 要选择的通道号
 * @note 先禁用所有通道再启用目标通道
 **************************************************************/
void select_analog_channel(uint8_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 禁用所有可能使用的ADC通道
    const uint32_t channels[] = {
        ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_5,
        ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_13,
        ADC_CHANNEL_14
    };

    for (uint8_t i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        sConfig.Channel = channels[i];
        sConfig.Rank = ADC_RANK_NONE;
        HAL_ADC_ConfigChannel(&hadc, &sConfig);
    }

    // 配置目标通道
    switch (channel) {
        case CH_USONIC:    sConfig.Channel = ADC_CHANNEL_0;  break;
        case CH_PD2:       sConfig.Channel = ADC_CHANNEL_1;  break;
        case CH_V_BAT:     sConfig.Channel = ADC_CHANNEL_5;  break;
        case CH_TP_NTC:    sConfig.Channel = ADC_CHANNEL_6;  break;
        case CH_TP_TEMP:   sConfig.Channel = ADC_CHANNEL_7;  break;
        case CH_PTS:       sConfig.Channel = ADC_CHANNEL_14; break;
        case CH_ALRM_MON:  sConfig.Channel = ADC_CHANNEL_13; break;
        default:           sConfig.Channel = ADC_CHANNEL_0;  break;
    }

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**************************************
 * select_analog_channel()
 * @detail
 * Select analog channel
 */

#if(0)
void select_analog_channel(uint8_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};             //0us >
  sConfig.Channel =ADC_CHANNEL_0;					//None ranking of 6 input takes 60us
  sConfig.Rank = ADC_RANK_NONE;						//this is required if using polled ADC
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_13;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  sConfig.Channel =ADC_CHANNEL_14;
  sConfig.Rank = ADC_RANK_NONE;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);			//> 60us
  switch (channel){
    case CH_USONIC:
      sConfig.Channel =ADC_CHANNEL_0;
      break;
    case CH_PD2:
      sConfig.Channel =ADC_CHANNEL_1;
      break;
    case CH_V_BAT:
      sConfig.Channel =ADC_CHANNEL_5;
      break;
    case CH_TP_NTC:
      sConfig.Channel =ADC_CHANNEL_6;
      break;
    case CH_TP_TEMP:
      sConfig.Channel =ADC_CHANNEL_7;
      break;
    case CH_PTS:
      sConfig.Channel =ADC_CHANNEL_14;
      break;
    case CH_ALRM_MON:
      sConfig.Channel =ADC_CHANNEL_13;
      break;
    default:
      sConfig.Channel =ADC_CHANNEL_0;
  }
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
    Error_Handler();
  }
}
#endif
/** End select_analog_channel() **/
/**
 * @brief 校准管路红外传感器（需保持管路为空）
 * @details 通过调整IR LED电流(DAC1)使PD2读数达到目标范围，
 *          并将校准后的PD2和DAC值保存至EEPROM，记录校准结果
 */
void cal_tir_sensor(void)
{
    uint16_t previous_dac = G_dac_cal_value;
    uint16_t best_dac = 0;
    uint16_t best_pd2 = 0;
    uint16_t pd2_reading = 0;
    uint16_t best_error = 0xFFFF;
    uint8_t attempt;
    uint8_t found = 0;  // 是否找到可接受的标准

    /* 从最严格到最宽松依次尝试校准 */
    for (attempt = 0; attempt <= 5; attempt++) {
        // 每次重新初始化
        best_error = 0xFFFF;
        G_dac_cal_value = INITIAL_DAC_VALUE;
        HAL_Delay(DELAY_AFTER_DAC_SET);

        uint16_t adjustment_step = 1024;

        /* 第一阶段：二分查找粗调 */
        for (uint8_t step = 0; step < BINARY_SEARCH_STEPS; step++) {
            while (G_tick5ms_counter < ADC_SETTLE_TIME_TICKS);
            G_tick5ms_counter = 0;

            pd2_reading = G_ir_pd2_value;

            // 记录最佳值
            uint16_t error = abs((int)pd2_reading - (int)PD2_TARGET);
            if (error < best_error) {
                best_error = error;
                best_dac = G_dac_cal_value;
                best_pd2 = pd2_reading;
            }
            if (pd2_reading > 700) {
                adjustment_step /= 2;
            }
            // 二分查找
            if (pd2_reading < PD2_TARGET) {
                G_dac_cal_value += adjustment_step;
            } else {
                // 减法可能溢出：检查是否会低于0
                if (G_dac_cal_value >= adjustment_step) {
                    G_dac_cal_value -= adjustment_step;
                }
            }
        }
        /* 尝试不同标准的校验 */
        // 先用严格标准(5%)校验粗调结果
        if (test_tir_cal_values(best_pd2, best_dac, attempt) == CAL_OK) {
            save_tir_cal(best_pd2, best_dac);
            G_dac_cal_value = best_dac;
            G_cal_status = CAL_OK;
            return;
        }

        /* 第二阶段：精细搜索 */
        uint16_t start_dac = best_dac;
        best_error = 0xFFFF;

        // 精细搜索（步长64，范围±500）
        for (int16_t offset = -500; offset <= 500; offset += 64) {
            int32_t new_dac = start_dac + offset;
            if (new_dac < DAC_IR_CAL_MIN) new_dac = DAC_IR_CAL_MIN;
            if (new_dac > DAC_IR_CAL_MAX) new_dac = DAC_IR_CAL_MAX;
            G_dac_cal_value = (uint16_t)new_dac;

            HAL_Delay(50);
            while (G_tick5ms_counter < 40);
            G_tick5ms_counter = 0;

            pd2_reading = G_ir_pd2_value;
            uint16_t error = abs(pd2_reading - PD2_TARGET);

            if (error < best_error) {
                best_error = error;
                best_dac = G_dac_cal_value;
                best_pd2 = pd2_reading;
            }
        }

        /* 第三阶段：超精细搜索 */
        start_dac = best_dac;

        for (int16_t fine_offset = -60; fine_offset <= 60; fine_offset += 8) {
            int32_t new_dac = start_dac + fine_offset;
            if (new_dac < DAC_IR_CAL_MIN) new_dac = DAC_IR_CAL_MIN;
            if (new_dac > DAC_IR_CAL_MAX) new_dac = DAC_IR_CAL_MAX;
            G_dac_cal_value = (uint16_t)new_dac;

            HAL_Delay(30);
            while (G_tick5ms_counter < 40);
            G_tick5ms_counter = 0;

            pd2_reading = G_ir_pd2_value;
            uint16_t error = abs(pd2_reading - PD2_TARGET);

            if (error < best_error) {
                best_error = error;
                best_dac = G_dac_cal_value;
                best_pd2 = pd2_reading;
            }
        }

        /* 用当前attempt标准校验最终结果 */
        if (test_tir_cal_values(best_pd2, best_dac, attempt) == CAL_OK) {
            save_tir_cal(best_pd2, best_dac);
            G_dac_cal_value = best_dac;
            G_cal_status = CAL_OK;
            return;
        }

        // 如果当前标准失败，继续下一次循环（更宽松的标准）
    }

    /* 所有标准都失败，但检查是否比原来好 */
    uint16_t old_error = abs(G_ir_pd2_value - PD2_TARGET);
    if (best_error < old_error) {
        // 比原来好，就使用最佳值
        save_tir_cal(best_pd2, best_dac);
        G_dac_cal_value = best_dac;
        G_cal_status = CAL_OK;
    } else {
        // 不比原来好，恢复原值
        G_dac_cal_value = previous_dac;
        G_cal_status = CAL_ERROR;
    }
}
#if(0)
/**
 * @brief 校准管路红外传感器（需保持管路为空）
 * @details 通过调整IR LED电流(DAC1)使PD2读数达到目标范围，
 *          并将校准后的PD2和DAC值保存至EEPROM，记录校准结果
 */
void cal_tir_sensor(void)
{


    /*--------- 变量初始化 ---------*/
    uint16_t previous_dac = G_dac_cal_value;  // 保存原始DAC值
    uint16_t adjustment_step = 1024;          // 初始调整步长(2048/2)
    uint16_t pd2_reading = 0;                 // PD2当前读数
    int8_t result;                 // 校准结果

    /*--------- 校准初始化 ---------*/
    G_dac_cal_value = INITIAL_DAC_VALUE;      // 设置初始DAC值
    HAL_Delay(DELAY_AFTER_DAC_SET);           // 等待DAC稳定

    /*--------- 二分查找算法 ---------*/
    for (uint8_t step = 0; step < BINARY_SEARCH_STEPS; step++) {
        // 等待ADC读数稳定
        while (G_tick5ms_counter < ADC_SETTLE_TIME_TICKS) {
            // 空等待（实际项目中可替换为RTOS延时）
        }
        G_tick5ms_counter = 0;  // 重置计数器

        // 获取当前PD2读数
        pd2_reading = G_ir_pd2_value;

        /* 动态调整DAC值 */
        if(pd2_reading > 700){
        	adjustment_step /= 2;  // 每次迭代步长减半
        }
        if (pd2_reading < PD2_TARGET) {
            G_dac_cal_value += adjustment_step;  // 读数偏低则增加电流
        } else {
            G_dac_cal_value -= adjustment_step;  // 读数偏高则减小电流
        }
    }

    /*--------- 校准结果处理 ---------*/
    result = test_tir_cal_values(pd2_reading, G_dac_cal_value);
    save_cal_rec(pd2_reading, G_dac_cal_value);  // 保存校准记录

    if (result == CAL_OK) {
        save_tir_cal(pd2_reading, G_dac_cal_value);  // 保存至EEPROM
        G_cal_status = CAL_OK;  // 更新全局状态
    } else {
        /*--------- 第一次失败，尝试第二次更详细的校准 ---------*/

        // 使用第一次的结果作为起点，进行更精细的校准
        uint16_t start_dac = G_dac_cal_value;  // 以第一次的结果为起点
        uint16_t best_dac = start_dac;
        uint16_t best_pd2 = pd2_reading;
        uint16_t min_error = abs((int)pd2_reading - (int)PD2_TARGET);

        /*--------- 精细校准：在第一次结果附近搜索 ---------*/
        for (int16_t offset = -500; offset <= 500; offset += 64) {
            G_dac_cal_value = start_dac + offset;

            HAL_Delay(50);  // 等待稳定
            while (G_tick5ms_counter < 40);  // 等待ADC
            G_tick5ms_counter = 0;

            pd2_reading = G_ir_pd2_value;
            uint16_t error = abs((int)pd2_reading - (int)PD2_TARGET);

            // 记录最佳值
            if (error < min_error) {
                min_error = error;
                best_dac = G_dac_cal_value;
                best_pd2 = pd2_reading;
            }
        }

        /*--------- 超精细校准：在最佳值附近进一步优化 ---------*/
        for (int16_t fine_offset = -60; fine_offset <= 60; fine_offset += 8) {
            G_dac_cal_value = best_dac + fine_offset;

            HAL_Delay(30);
            while (G_tick5ms_counter < 40);
            G_tick5ms_counter = 0;

            pd2_reading = G_ir_pd2_value;
            uint16_t error = abs((int)pd2_reading - (int)PD2_TARGET);

            if (error < min_error) {
                min_error = error;
                best_dac = G_dac_cal_value;
                best_pd2 = pd2_reading;
            }
        }

        // 使用找到的最佳值
        G_dac_cal_value = best_dac;
        pd2_reading = best_pd2;

        /*--------- 第二次校准结果处理 ---------*/
        result = test_tir_cal_values(pd2_reading, G_dac_cal_value);
        save_cal_rec(pd2_reading, G_dac_cal_value);

        if (result == CAL_OK) {
            save_tir_cal(pd2_reading, G_dac_cal_value);
            G_cal_status = CAL_OK;
        } else {
            /*--------- 两次都失败，恢复原值 ---------*/
            G_dac_cal_value = previous_dac;
            G_cal_status = CAL_ERROR;
        }
    }
}

/**************************************
 * cal_tir_sensor()
 * @detail
 * Calibrate the tubing ir sensor while empty tubing is installed in TIR.
 * Adjust IR led current (DAC1) until PD2 reading is in specified window.
 * Save PD2 and DAC values to eeprom. Save calibration result to error log.
 */

void cal_tir_sensor(void){
uint16_t i,a_adj,pd2,dac_old;
int8_t result;
  dac_old = G_dac_cal_value ;
  G_dac_cal_value = 2000;
  HAL_Delay(200);
  a_adj=2048;
  for (i=0; i<11; i++){
    a_adj= a_adj/2;
    while (G_tick5ms_counter<40);    //170ms loop. This delay must be long enough for new AD conversion
    G_tick5ms_counter = 0;            //to start with a new DAC value. 2 x conversion cycle (2 x 85).
    pd2 = G_ir_pd2_value;
    if (pd2 < PD2_IR_CAL_TARGET){
        G_dac_cal_value = G_dac_cal_value +a_adj;
    }
    else{
   	  G_dac_cal_value = G_dac_cal_value - a_adj;
    }
  }
  result = test_tir_cal_values(pd2,G_dac_cal_value);
  save_cal_rec(pd2,G_dac_cal_value);
  if (result==CAL_OK){
	  save_tir_cal(pd2,G_dac_cal_value);
	  G_cal_status = CAL_OK;
  }
  else {
	G_dac_cal_value = dac_old;              //Calibration failed, restore old calibration value
	G_cal_status = CAL_ERROR;
  }
}
#endif
/** End cal_tir_sensor **/

/**************************************************************
 * @brief 读取电池电压
 * @return 电池电压(mV)
 * @note 100次采样平均值，转换系数235
 **************************************************************/
uint16_t read_v_bat(void)
{
    uint32_t vbat_sum = 0;

    // 关键修复：增加ADC稳定时间
    HAL_ADC_Stop(&hadc);
    HAL_Delay(2); // 确保ADC完全停止

    HAL_ADC_Start(&hadc);
    HAL_Delay(2); // 等待ADC稳定启动

    // 丢弃前几个读数
    for (uint8_t i = 0; i < 5; i++) {
        HAL_ADC_PollForConversion(&hadc, 10);
        HAL_ADC_GetValue(&hadc);
    }

    // 正式采样
    for (uint16_t i = 0; i < 100; i++) {
        HAL_ADC_PollForConversion(&hadc, 10);
        vbat_sum += HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    return vbat_sum / 235;
}


/**************************************
 * read_v_bat()
 * @detail
 * Read battery voltage from analog converter. Average of 100 readings converted to millivolts.
 */
#if(0)
uint16_t read_v_bat(void){
  uint16_t i,k;
  uint32_t vbat_sum;
  HAL_ADC_Start(&hadc);
  vbat_sum=0;
  for (i=0; i<100; i++){
    HAL_ADC_PollForConversion(&hadc, 1);
    vbat_sum= vbat_sum + HAL_ADC_GetValue(&hadc);
  }
  k=vbat_sum/235;
  HAL_ADC_Stop(&hadc);                                      //> 80us for 1 conv (120us for 5)
  return k;
}
#endif

/** End read_v_bat() **/

/**************************************************************
 * @brief ADC通道采样（5次平均）
 * @return 12位ADC采样值（0-4095）
 * @note 单次采样耗时约120us
 **************************************************************/
uint16_t read_analog_channel(void)
{
    uint32_t sum = 0;

    HAL_ADC_Start(&hadc);
    for (uint8_t i = 0; i < 5; i++) {
        HAL_ADC_PollForConversion(&hadc, 1);
        sum += HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    return (uint16_t)(sum / 5);
}

/**************************************
 * read_analog_channel()
 * @detail
 * Read 5 sample average from analog converter
 */

#if(0)
uint16_t read_analog_channel(void)
{
  uint16_t i,k;
  uint32_t j;
  HAL_ADC_Start(&hadc);
  j=0;
  for (i=0; i<5; i++){
    HAL_ADC_PollForConversion(&hadc, 1);
    j= j + HAL_ADC_GetValue(&hadc);
  }
  k=j/5;
  HAL_ADC_Stop(&hadc);
  return k;
}
#endif
/** End read_analog_channel() **/
/**************************************************************
 * @brief 超声波传感器状态检测（结合红外判断）
 * @param infrared_detected 红外是否检测到物体（true: 检测到, false: 未检测到）
 * @return 管路状态 TUBING_EMPTY / TUBING_NOT_EMPTY
 * @note 需先开启传感器电源
 **************************************************************/
uint8_t usonic_sensor_read_with_infrared()
{
    uint16_t adc_value;
    uint16_t empty_threshold = EMPTY_TUBING_MAX + 50;

    // 电源控制序列
    USONIC_PWR_ON();
    HAL_Delay(15);  // 稳定时间

    // 采样并判断
    select_analog_channel(CH_USONIC);
    adc_value = read_analog_channel();

    return (adc_value <= empty_threshold) ? TUBING_EMPTY : TUBING_NOT_EMPTY;
}
/**************************************************************
 * @brief 超声波传感器状态检测
 * @return 管路状态 TUBING_EMPTY / TUBING_NOT_EMPTY
 * @note 需先开启传感器电源
 **************************************************************/
uint8_t usonic_sensor_read()
{
    uint16_t adc_value;

    // 电源控制序列
   // HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_SET);
    USONIC_PWR_ON();
    HAL_Delay(15);  // 稳定时间

    // 采样并判断
    select_analog_channel(CH_USONIC);
    adc_value = read_analog_channel();

    return (adc_value <= EMPTY_TUBING_MAX) ? TUBING_EMPTY : TUBING_NOT_EMPTY;
}

/**************************************
 * usonic_sensor_read()
 * @detail
 * Read ultrasonic sensor from ADC. This is used during the start up while the background ADC is not running.
 */

#if(0)
uint8_t usonic_sensor_read()
{
  uint16_t adc_value;
  HAL_GPIO_WritePin(USONIC_PWR_GPIO_Port, USONIC_PWR_PIN, GPIO_PIN_SET);   //Usonic sensor power on
  HAL_Delay(15);
  select_analog_channel(CH_USONIC);
  adc_value = read_analog_channel();
  if (adc_value <= EMPTY_TUBING_MAX) {
  	return TUBING_EMPTY;
    }
  else{
    return (TUBING_NOT_EMPTY);
  }
}
#endif
/** End usonic_sensor_read() **/


/**************************************************************
 * @brief 压力传感器状态检测
 * @return 管路状态 TUBING_DETECTED / NOT_DETECTED
 * @note 使用启动时直接采样模式
 **************************************************************/
uint8_t pts_sensor_read()
{
    select_analog_channel(CH_PTS);
    uint16_t adc_value = read_analog_channel();

    return (adc_value < PTS_THRESHOLD) ? TUBING_DETECTED : TUBING_NOT_DETECTED;
}
/**************************************
 * pts_sensor_read()
 * @detail
 * Read PTS sensor from ADC. This is used during the start up while the background ADC is not running.
 */
#if(0)
uint8_t pts_sensor_read()
{
  uint16_t adc_value;
  select_analog_channel(CH_PTS);
  adc_value = read_analog_channel();
  if (adc_value < PTS_THRESHOLD){
	return (TUBING_DETECTED);
  }
  else{
	return (TUBING_NOT_DETECTED);
  }
}
#endif
/** End pts_sensor_read() **/
/**************************************************************
 * @brief 压力传感器状态检测（缓冲数据版）
 * @return 管路状态 TUBING_DETECTED / NOT_DETECTED
 * @note 使用后台扫描的缓冲数据
 **************************************************************/
uint8_t pts_sensor()
{
    return (G_vpts < PTS_THRESHOLD) ? TUBING_DETECTED : TUBING_NOT_DETECTED;
}



/**************************************
 * pts_sensor()
 * @detail
 * Read PTS sensor value from buffer. Compare measured value to threshold and return the status.
 */
#if(0)
uint8_t pts_sensor(){
  if (G_vpts < PTS_THRESHOLD){
	return (TUBING_DETECTED);
  }
  else{
	return (TUBING_NOT_DETECTED);
  }
}
#endif

/** End pts_sensor() **/

/**************************************************************
 * @brief 检测管路状态
 * @return 管路状态枚举值
 * @note 基于超声波传感器阈值判断
 **************************************************************/
uint8_t distal_tube_empty()
{

	return (G_usonic_value <= EMPTY_TUBING_MAX) ?
           TUBING_EMPTY : TUBING_NOT_EMPTY;
}

/**************************************
 * safe_distal_tube_empty()
 * @short		Safe test if the distal tube is empty with ultrasonic state check
 * @param		void
 * @retval  	uint8_t TUBING_EMPTY or TUBING_NOT_EMPTY
 * @note		Only returns valid result when ultrasonic is ON and stable
 *              Returns TUBING_NOT_EMPTY when ultrasonic is OFF or unstable
 **************************************/
//安全的超声检测函数
uint8_t safe_distal_tube_empty(void)
{
    // 确保超声在开启状态且数据稳定
    if (!usonic_ready_for_detection) {
        return TUBING_NOT_EMPTY;  // 超声关闭或未稳定时默认管路非空
    }

    return (G_usonic_value <= EMPTY_TUBING_MAX) ? TUBING_EMPTY : TUBING_NOT_EMPTY;
}

/**************************************
 * distal_tube_empty()
 * @detail
 *Test if the distal tube is empty (no fluid)
 */

#if(0)
uint8_t distal_tube_empty()
{
  if (G_usonic_value <= EMPTY_TUBING_MAX) {
	return TUBING_EMPTY;
  }
  return (TUBING_NOT_EMPTY);
}
#endif
/** End distal_tube_empty() **/

/**************************************************************
 * @brief 检测管路状态
 * @return 管路状态枚举值
 * @note 基于PD2传感器阈值判断
 **************************************************************/
uint8_t distal_tube_detected()
{
    return (G_ir_pd2_value >  PD2_NO_TUBING_MIN) ?
    		TUBING_NOT_DETECTED : TUBING_DETECTED;
}

/**************************************
 * distal_tube_detected()
 * @detail
 * Test if distal tube installed
 */
#if(0)
uint8_t distal_tube_detected (void)
{
  if ( G_ir_pd2_value >  PD2_NO_TUBING_MIN){
    return TUBING_NOT_DETECTED;
  }
  return (TUBING_DETECTED);
}
#endif
/** End distal_tube_detected() **/

/**************************************
 * save_nv()
 * @detail
 * Save pumping variables to eeprom.
 */
void save_nv(void)
{

	SaveNV_Volume();
	SaveNV_Settings();
    SaveNV_Acc_Volume();


}/** End save_nv() **/


/**************************************************************
 * @brief 系统关机流程
 * @note 关闭所有外设后进入死循环
 **************************************************************/
void power_off_pump(void)
{
    pump_motor_off();
    ir_heat_off();
    HAL_Delay(100);  // 等待外设完全关闭
    POWER_OFF();
    HAL_Delay(1000);  // 等待外设完全关闭
    while(1) {  // 进入永久等待
    // 如果断电不彻底，则进行系统复位
        // 注意：根据硬件设计，可能不需要死循环
        // 而是直接复位
    	HAL_Delay(100);  // 等待外设完全关闭
    	NVIC_SystemReset(); // 调用系统复位
    }
}

/**************************************
 * power_off_pump()
 * @detail
 * Power off the pump hardware.
 */
#if(0)
void power_off_pump(void){
	pump_motor_off();
	ir_heat_off();
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	while(1);
}
#endif
/** End power_off_pump() **/

/**************************************************************
 * @brief 检测AC电源状态
 * @return 1-AC已连接 0-电池供电
 **************************************************************/
uint8_t ac_connected(void)
{
    return AC_POWER_READ();
}

/**************************************
 * ac_connected()
 * @detail
 * Test if wall dc adapter is connected
 */
#if(0)
uint8_t ac_connected(void)
{
  uint8_t ac_power;
  ac_power=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
  return(ac_power);
}
#endif
/** End ac_connected() **/

/**************************************************************
 * @brief 读取电池电压
 * @return 电池电压(mV)
 * @note 使用专用采样函数read_v_bat()
 **************************************************************/
uint16_t read_battery_voltage(void)
{
    select_analog_channel(CH_V_BAT);
    return read_v_bat();  // 直接返回采样结果
}

/**************************************
 * read_battery_voltage()
 * @detail
 * Read and return battery voltage in mV.
 */

#if(0)
uint16_t read_battery_voltage(void)
{
  static uint16_t vbat_value;
  select_analog_channel(CH_V_BAT);
  vbat_value = read_v_bat();
  return(vbat_value);
}
#endif
/** End read_battery_voltage() **/


/**************************************************************
 * @brief 综合输入检测
 * @return 1-有输入 0-无输入
 * @note 检测键盘和触摸屏
 **************************************************************/
uint8_t any_key_or_touch(void)
{
    uint16_t dummy_timer;
    return (read_all_keys(&dummy_timer) || screen_touched());
}
/**************************************
 * any_key_or_touch()
 * @detail
 * Read keys and the touch screen. Return 1 if key pressed or touched. Otherwise return 0.
 */
#if(0)
uint8_t any_key_or_touch(void)
{
  uint16_t timeout_count;
  if (read_all_keys(&timeout_count)) return (1);
  if (screen_touched()) return (1);
  return(0);
}
#endif
/** End any_key_or_touch() **/


/**************************************************************
 * @brief 键盘扫描与状态机
 * @param key_timer 按键计时器指针
 * @return 按键编码或0（无按键）
 * @note 优先级：硬件按键 > 触摸屏
 **************************************************************/
uint8_t read_all_keys(uint16_t *key_timer)
{

	uint8_t key = read_keypad();  // 优先检测硬件按键

    if (key == NOT_PRESSED) {
        key = eve_read_tag();     // 次优先检测触摸屏
    }

    // 更新按键计时器
    if (key != NOT_PRESSED) {

    	*key_timer = 0;  // 释放清零
        } else {

        *key_timer = (*key_timer < 10000) ? (*key_timer + 1) : 10000;  // 按下递增
        }
    return key;
}
/**************************************
 * @detail
 * Read keys and the touch screen. Return the key (touch tag) code or 0 for no key or touch.
 * Increment key_timer while key pressed, reset if key released.
 */

#if(0)
uint8_t read_all_keys(uint16_t *key_timer)
{
  uint8_t key;
  key = read_keypad();              //keypad (RUN/STP and PWR) has first priority.
  if (key == NOT_PRESSED){          //If keypad not pressed
	key = eve_read_tag();           //check touch screen
  }
  if (key != NOT_PRESSED){
    *key_timer = 0;
  }
  else{
    if ((*key_timer)<10000) (*key_timer)++;
  }
  return key;
}
#endif
/** End read_all_keys() **/

/**
 * @brief Reads keypad state with edge detection
 * @return uint8_t Key code with press/release status:
 *
 * @details Implements debouncing and edge detection:
 *          - Sets PRESSED_BIT on first detection
 *          - Sets RELEASED_BIT after key release
 *          - Returns raw key code while key remains pressed
 */
uint8_t read_keypad(void)
{
    static uint8_t old_key = NOT_PRESSED;
    uint8_t current_key = NOT_PRESSED;

    // Read physical key states (active-low)
    if (!RUN_STOP_SWITCH_READ()) {
        current_key = RUN_STOP_SWITCH;
    } else if (!POWER_SWITCH_READ()) {
        current_key = POWER_SWITCH;
    }

    // Edge detection state machine
    uint8_t key_status = NOT_PRESSED;
    if (current_key == NOT_PRESSED) {
        key_status = (old_key != NOT_PRESSED) ? (old_key | RELEASED_BIT) : NOT_PRESSED;
    } else {
        key_status = (old_key == NOT_PRESSED) ? (current_key | PRESSED_BIT) : current_key;
    }

    old_key = current_key;
    return key_status;
}


/**************************************
 * @detail
 * Read keys. Return the key  code or 0 for no key pressed.
 * Set bit if key pressed first time or if the key has been released
 */

#if(0)
uint8_t read_keypad(void)
{
  static uint8_t old_key;      //previously read key
  uint8_t read_key;            //newly read key
  uint8_t send_key;            //output containing key info plus released/pressed bit

  read_key = NOT_PRESSED;
  if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))){
	read_key = RUN_STOP_SWITCH;
  }
  if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))){
	read_key = POWER_SWITCH;
  }
  if (read_key==NOT_PRESSED){
    if (old_key==NOT_PRESSED) {
    send_key = NOT_PRESSED;                   // switch not pressed
    }
    else{
      send_key = (old_key | RELEASED_BIT);
    }
  }
  else{
    if (old_key==NOT_PRESSED){
   	  send_key = (read_key | PRESSED_BIT);
    }
    else{
   	  send_key = read_key;
    }
  }
  old_key=read_key;
  return send_key;
}
#endif
/** End read_keypad() **/

/**
 * @brief Verifies set installation status and detects TPA type
 * @param set_test_count Pointer to test counter (for delayed execution)
 * @param set_installed Pointer to set installation status flag
 * @details Checks for set removal/new installation during hold mode:
 *          - Detects new TPA type if set installed
 *          - Handles error messaging for removed sets
 *          - Manages set removal procedure
 */
void test_if_set_installed(uint8_t* set_test_count, uint8_t* set_installed)
{

    // Early exit if not time to test yet
    if (++(*set_test_count) <= TEST_DELAY) {
        return;
    }
    *set_test_count = 0;

    /* Case 1: No TPA currently set */
    if (G_tpa_type == NO_TPA) {
        if (pts_sensor() == TUBING_NOT_DETECTED) {
            *set_installed = 0;
            return;
        }
        detect_tpa(); // New set detected - identify TPA type
        return;
    }

    /* Case 2: Distal tubing check */
/*    if (distal_tube_detected_with_confirmation() ==  TUBING_NOT_DETECTED) {
        handle_tubing_removal(set_installed);
    } else {
        *set_installed = 1; // Set properly installed
    } */
}

/* 远端管路确认检测函数 */
uint8_t distal_tube_detected_with_confirmation(void)
{
    static uint8_t missing_count = 0;
    const uint8_t CONFIRMATION_COUNT = 3;

    uint8_t current_status = distal_tube_detected();

    if (current_status == TUBING_DETECTED) {
        // 检测到管路存在：重置计数器
        missing_count = 0;
        return TUBING_DETECTED;
    } else {
        // 检测到管路缺失
        missing_count++;

        if (missing_count >= CONFIRMATION_COUNT) {
            // 第三次确认缺失
            missing_count = 0;
            return TUBING_NOT_DETECTED;
        } else {
            // 前两次仍然返回"检测到"
            return TUBING_DETECTED;
        }
    }
}


/**
 * @brief Handles tubing removal scenario
 */
static void handle_tubing_removal(uint8_t* set_installed)
{
    if (!(*set_installed)) {
        show_error_message(ERROR_DISPLAY_TIME, "DISTAL TUBING", "NOT INSTALLED");

        if (pts_sensor() == TUBING_NOT_DETECTED) {
            show_error_message(ERROR_DISPLAY_TIME, "SET NOT", "INSTALLED");
        }
    }

    remove_set_task(); // Initiate set removal procedure
    *set_installed = 0;
}

/**************************************
 * test_if_set_installed()
 * @detail
 * Called during the hold mode to verify that the set has not been removed or if new set installed.
 * If new set installed, detect the TPA type. If current set is removed, show error message and allow
 * set removal or correction.
 */

#if(0)
void  test_if_set_installed(uint8_t *set_test_count,uint8_t *set_installed)
{
  (*set_test_count)++;
  if ((*set_test_count) > SET_TEST_DELAY){         //check if it's time to execute the test
    *set_test_count = 0;
    if (G_tpa_type == NO_TPA){                     //TPA type not set, detect TPA if set installed (PTS)
      if (pts_sensor() == TUBING_NOT_DETECTED){    //set not installed return
    	*set_installed = 0;
	    return;
	  }
	  detect_tpa();									//new set detected -> check TPA type
	  return;
    }
    //if the distal tube is not detected:
    //  first time prompt for set removal (set_installed=1)
    //  after that show error message "DISTAL TUBING NOT INSTALLED" (set_installed=0)
    if (distal_tube_detected() == TUBING_NOT_DETECTED){
      if (!(*set_installed)){
	    show_error_message(TEN_SECOND,"DISTAL TUBING","NOT INSTALLED");
	    if (pts_sensor() == TUBING_NOT_DETECTED){
	      show_error_message(TEN_SECOND,SET_NOT_,_INSTALLED);
	    }
      }
	  remove_set_task();                           //remove set if distal tubing removed
	  *set_installed = 0;
    }
    else *set_installed = 1;
  return ;
  }
}
#endif
/** End test_if_set_installed() **/

/**************************************************************
 * @brief 检测TPA(管路适配器)类型
 * @note 通过夹爪运动检测磁铁位置判断TPA类型
 **************************************************************/
void detect_tpa(void)
{
    uint8_t status = move_pincher_all_ccw();

    status = move_pincher_from_all_ccw_to_magnet(&g_pincher_result);

    G_tpa_type = status;
    G_prime_completed = 0;

    if (status == NO_TPA) {
        show_error_message(TEN_SECOND, "NO TPA DETECTED", "");
        remove_set_task();
    }

    move_pincher_all_ccw();

}

/**************************************************************
 * detect_tpa()
 * @detail
 * Detect if TPA installed and identify the TPA type.
 * Set G_tpa_type, NO_TPA, DUAL_TPA or SINGLE_TPA
 */

#if(0)
void detect_tpa(void)
{
  uint8_t status;
  status = move_pincher_all_ccw();
  status = move_pincher_from_all_ccw_to_magnet();
  G_tpa_type = status;
  G_prime_completed=0;
  if (status == NO_TPA){
	show_error_message(TEN_SECOND,"NO TPA DETECTED","");
	status = remove_set_task();
	return ;
  }
  // if (G_tpa_type == SINGLE_TPA) G_water_rate=0;
  move_pincher_all_ccw();
  return;
}
#endif
/** End detect_tpa_type() **/

/***************************************************************/
uint8_t check_pts_wake_up(void)
{
    uint8_t status, pts_wakeup = 0;
    uint8_t detected_count = 0;
    uint8_t sensor_check_count = 0;
    bool is_stable_wakeup = false;
    bool tubing_not_detected = false;
    const uint8_t STABLE_CHECK_COUNT = 5;
    const uint8_t SENSOR_DEBOUNCE_COUNT = 5;

    // 1. 检测唤醒方式
    pts_wakeup = (pic_i2c_read(0) == 0x0a); // 简化条件表达式//0x0a = PTS Wake_Up, 0x00 = Power Switch Wake-Up

    // 2. 如果是PTS唤醒，通过pts_sensor_read来排除干扰
    if (pts_wakeup) {
     // 通过多次读取pts_sensor来确认是否是稳定唤醒
        for (int i = 0; i < STABLE_CHECK_COUNT; i++) {
            if (pts_sensor_read() == TUBING_DETECTED) {
                detected_count++;
            }
            HAL_Delay(10);
        }

        // 如果多次检测都显示管路已安装，认为是稳定唤醒
        // 如果管路未安装，可能是干扰引起的异常状态
        is_stable_wakeup = (detected_count >= STABLE_CHECK_COUNT);

        if (!is_stable_wakeup) {
            // 干扰引起的唤醒，直接关机
            HAL_Delay(50);
            power_off_pump();
           //
        }
    }

    // . 正常检查管路安装状态（只有稳定唤醒才执行）
    for (int i = 0; i < SENSOR_DEBOUNCE_COUNT; i++) {
        if (pts_sensor_read() == TUBING_NOT_DETECTED) {
            sensor_check_count++;
        }
        HAL_Delay(2);
    }

    tubing_not_detected = (sensor_check_count >= SENSOR_DEBOUNCE_COUNT);

    if (tubing_not_detected) {
    	show_error_message(TEN_SECOND, SET_NOT_, _INSTALLED);
        if (pts_wakeup) {
            HAL_Delay(100);
            power_off_pump();
        }
    }
    // 3. 检测TPA类型
    status = detect_tpa_type();

    // 4. 处理不同状态
    handle_tpa_status(status, pts_wakeup);

    // 5. 显示启动屏幕
    display_start_screen();


    // 6. 根据TPA类型移动夹管阀
    adjust_pincher_position(status);

    HAL_Delay(2000);

    // 7. PTS唤醒时的额外检查
    if (pts_wakeup) {
        check_tubing_empty();
    }

    // 8. 检查远端管路
    check_distal_tubing();

    return status;
}
/***************************************************************/
// 检测TPA类型
static uint8_t detect_tpa_type(void)
{
#if (MOTOR_TYPE == 2)
	Motor_Init();
	HAL_Delay(30);
#endif
	move_pincher_all_ccw();
    return move_pincher_from_all_ccw_to_magnet(&g_pincher_result);
}
/***************************************************************/
// 处理TPA状态
static void handle_tpa_status(uint8_t status, uint8_t pts_wakeup)
{
    switch(status) {
        case NO_TPA:
            if (!pts_wakeup && pts_sensor_read() == TUBING_DETECTED) {
            	show_error_message(TEN_SECOND, NO_TPA_DETECTED, "");
            }
            power_off_pump();
            break;

        case ERR_NO_MAGNET:
        	show_error_message(TEN_SECOND, ERROR_100, "");
            save_err_rec(ERR_NO_MAGNET);
            power_off_pump();
            break;

        default:
            if (pts_sensor_read() == TUBING_NOT_DETECTED) {
            	show_error_message(TEN_SECOND, "REMOVE TPA", "");
                power_off_pump();
            }
            break;
    }
}
/***************************************************************/
// 调整夹管阀位置
static void adjust_pincher_position(uint8_t status)
{
    if (status == DUAL_TPA) {
        move_pincher_all_ccw(); // 双管:完全逆时针关闭bolus管路


    } else {
        move_pincher_all_cw();  // 单管:完全顺时针关闭food管路

    }
}
/***************************************************************/
// 检查管路是否为空
static void check_tubing_empty(void)
{
    while (usonic_sensor_read() != TUBING_EMPTY) {
    	show_error_message(TEN_SECOND, "DISTAL TUBING NOT EMPTY", "INSTALL NEW SET");
        move_pincher_to_center();
        power_off_pump();
    }
}
/***************************************************************/
// 检查远端管路
static void check_distal_tubing(void)
{
    while (!distal_tube_detected()&&(usonic_sensor_read_with_infrared() == TUBING_EMPTY)) {
    	show_error_message(TEN_SECOND, "DISTAL TUBING NOT DETECTED", "INSTALL DISTAL TUBING");
    		uint8_t ret = show_message_yes_no("TRY AGAIN?");
    		if (ret == NO) {
    			move_pincher_to_center();
    			power_off_pump();
    		}
     }
}

/**************************************************************
 * check_pts_wake_up()
 * @detail
 * Pump was powered automatically after set installation or by pressing the PWR button.
 * Inspect that the set is installed. Operate the pincher valve and detect
 * the Set type (Dual or Single). Shut of power if the set not detected.
 */

#if(0)
uint8_t check_pts_wake_up(void)
{
  uint8_t ret,status,pts_wakeup;
  pts_wakeup = 0;
  if (pic_i2c_read(0) == 0x0a){							//0x0a = PTS Wake_Up, 0x00 = Power Switch Wake-Up
	pts_wakeup=1;
  }
  if (pts_sensor_read() == TUBING_NOT_DETECTED){        //Test PTS sensor if tubing installed on rotor
	if (pts_wakeup) power_off_pump();
	show_error_message(TEN_SECOND,SET_NOT_,_INSTALLED);		 //If set not installed, display error message and shut off pump
  }
  //Move pincher to center. Read TPA status (type)
  status = move_pincher_all_ccw();
  status = move_pincher_from_all_ccw_to_magnet();       //Move to center. Test if TPA is installed.
  	  	  	  	  	  	  	  	  	  	  	  	  	  	//TPA detected by counting motor steps from all CCW to magnet

  if (status == NO_TPA){									//If TPA is not detected and
	if (!pts_wakeup){										//1. Manual power on
	  if (pts_sensor_read() == TUBING_DETECTED){			//   -> Show error message and power off
		show_error_message(TEN_SECOND,NO_TPA_DETECTED,"");	//
	  }														//2. PTS wake-up
	}														//   -> Power off without activating the screen.
	power_off_pump();
  }
  else{
	if (pts_sensor_read() == TUBING_NOT_DETECTED){			//If TPA is detected but the peristaltic
	  show_error_message(TEN_SECOND,"REMOVE TPA","");		//tubing not loaded, prompt TPA removal and
	  power_off_pump();										//shut off the pump.
	}
  }

  if (status == ERR_NO_MAGNET){							//If magnet was not detected, .
  	show_error_message(TEN_SECOND,ERROR_100,"");        //display error message and shut off.
  	save_err_rec(ERR_NO_MAGNET);
  	power_off_pump();
  }
  display_start_screen();
  if (status==DUAL_TPA)  {                  //Set installed (TPA and PTS) -> close pincher valve
    move_pincher_all_ccw();					//DUAL TPA: Move pincher all CCW (Close the bolus tubing)
  }
  else {
	move_pincher_all_cw();					//SINGLE TPA: Move pincher all CW (Close the food tubing)
  }
  HAL_Delay(2000);                        //Delay to allow time for the START screen
  if (pts_wakeup){
    while (usonic_sensor_read() != TUBING_EMPTY ){
      show_error_message(TEN_SECOND,"DISTAL TUBING NOT EMPTY","INSTALL NEW SET");
      move_pincher_to_center();
      power_off_pump();
    }
  }
  while (!distal_tube_detected() ){
    show_error_message(TEN_SECOND,"DISTAL TUBING NOT DETECTED","INSTALL DISTAL TUBING");
  	ret = show_message_yes_no("TRY AGAIN?");
  	if (ret == NO){
  	  move_pincher_to_center();
  	  power_off_pump();
  	}
  }
  return(status);				//Return TPA type. Dual or Single TPA or NO_TPA
}/** check_pts_wake_up() **/
#endif

/**
 * @brief Checks if distal tubing is installed and handles removal process
 * @param occl_timer Pointer to occlusion test timer (won't test during IR heating)
 * @return uint8_t YES if tube was removed, NO otherwise
 */

uint8_t distal_tube_removed(uint16_t* occl_timer)
{
    /* Skip check during IR heating when sensor is saturated */
    if (*occl_timer <= OCCL_TEST_HEAT_END) {
        return NO;
    }

    static uint8_t consecutive_missing = 0;
    const uint8_t CONFIRMATION_COUNT = 10;

    /* 单次检测 */
    if (distal_tube_detected() == TUBING_DETECTED) {
        // 检测到管路存在：重置计数器，返回NO
        consecutive_missing = 0;
        return NO;
    } else {
    	if(!safe_distal_tube_empty()){
            // 超声检测到管路存在：重置计数器，返回NO
            consecutive_missing = 0;
            return NO;
    	}
    	else{
    		// 检测到管路缺失：增加计数器
    		consecutive_missing++;

    		if (consecutive_missing < CONFIRMATION_COUNT) {
    			// 第1、2次检测：返回NO（假装设备还在）
    			return NO;
    		} else {
    			// 第3次检测：确认缺失，返回YES
    			consecutive_missing = 0;  // 重置计数器

    			// 执行缺失处理
    			handle_missing_tube(occl_timer);
    			if (initiate_tube_removal_sequence() == YES) {
    				perform_tube_removal();
    				reset_system_state();
    			}
    			resume_pumping();

    			return YES;
    		}
    	}
    }
}
#if(0)
uint8_t distal_tube_removed(uint16_t* occl_timer)
{
    /* Skip check during IR heating when sensor is saturated */
    if (*occl_timer <= OCCL_TEST_HEAT_END) {
        return NO;
    }
    /* Check for tube presence */
    while (!distal_tube_detected()) {
        handle_missing_tube(occl_timer);

        if (initiate_tube_removal_sequence() == YES) {
            perform_tube_removal();
            reset_system_state();
            return YES;
        }
        resume_pumping();
    }
    return NO;
}
#endif
/* Helper Functions */

/**
 * @brief Handles actions when tube is not detected
 */
static void handle_missing_tube(uint16_t* occl_timer)
{
    /* Reset occlusion timer if it was interrupted */
    if (*occl_timer < 150) {
        *occl_timer = OCCL_TEST_START_COUNT;
    }

    pump_motor_off();
    show_error_message(TEN_SECOND, "DISTAL TUBING", "NOT INSTALLED");
}

/**
 * @brief Prompts user for tube removal confirmation
 * @return uint8_t YES if user confirms removal, NO otherwise
 */
static uint8_t initiate_tube_removal_sequence(void)
{
    if (show_message_yes_no("REMOVE SET") == YES) {
        return show_message_yes_no("CLOSE QUICK-CLAMPS");
    }
    return NO;
}

/**
 * @brief Executes the tube removal procedure
 */
static void perform_tube_removal(void)
{
	 // 1. 定位夹管器
	    move_pincher_all_cw();

	    move_pincher_from_all_cw_to_magnet();

	    // 2. 显示提示信息
#if USE_HOLD_MODE_NEW_SCREEN
	    show_message("REMOVE", "SET NOW");
#else
	    show_message("REMOVE SET NOW", "");
#endif
	    // 3. 10秒超时检测
	    uint8_t seconds_elapsed = 0;
	    while ((seconds_elapsed < 10) && (pts_sensor() == TUBING_DETECTED)) {
	        wait_one_second(); // 替换原有的计时逻辑
	        seconds_elapsed++;
	    }
}

/**
 * @brief Resets system state after tube removal
 */
static void reset_system_state(void)
{
    G_prime_completed = 0;
    G_tpa_type = NO_TPA;
}

/**
 * @brief Resumes normal pumping operation
 */
static void resume_pumping(void)
{
    start_pump_w_old_rate();
}

/**************************************
 * distal_tube_removed()
 * @detail
 * Test if distal tubing installed. Allow removal if not installed.
 */

#if(0)
uint8_t distal_tube_removed(uint16_t *occl_timer){
  uint8_t seconds_time, one_second_count;

  if ((*occl_timer)>OCCL_TEST_HEAT_END){            //DO NOT TEST DURING IR HEAT. IR sensor is saturated
    while (!distal_tube_detected()){				//_and does not work.
      if ((*occl_timer)< 150) *occl_timer=OCCL_TEST_START_COUNT;    //OCCL test was interrupted reset the occl_timer
      pump_motor_off();
      show_error_message(TEN_SECOND,"DISTAL TUBING","NOT INSTALLED");
      if (show_message_yes_no("REMOVE SET") == YES){
        if (show_message_yes_no("CLOSE QUICK-CLAMPS") == YES){
    	  move_pincher_all_cw();
    	  move_pincher_from_all_cw_to_magnet();		//rotate pincher to center position for TPA removal
          seconds_time=0;
          while ((seconds_time < 10) && (pts_sensor() == TUBING_DETECTED)){
    	    show_message("REMOVE SET NOW","");
     	    one_second_count++;
       	    if (one_second_count>20){
    	      one_second_count=0;
    	      seconds_time++;
    	    }
    	    while (G_tick5ms_counter<10);    //50ms loop
       	    G_tick5ms_counter = 0;
          }
          G_prime_completed=0;
          G_tpa_type = NO_TPA;
          return (YES);                          //Set removed, return YES
        }
      }
      start_pump_w_old_rate();                 //Set not removed continue pumping, return NO
    }
  }
  return (NO);
}/** End distal_tube_removed() **/
#endif
/**************************************
 * empty_test_w_heat()
 * @brief 通过加热测试判断管道状态
 * @return uint8_t 1-管道空，0-管道满
 * @detail
 * 1. 停止泵并加热5秒
 * 2. 以25ml/h启动泵
 * 3. 监测20秒内温升
 *   - 温升>30单位: 管道满(返回0)
 *   - 超时未达温升: 管道空(返回1)
 */
uint8_t empty_test_w_heat(void)
{
    uint8_t seconds_elapsed = 0;
    uint16_t initial_temp;

    /* 阶段1：停止泵并加热5秒 */
    pump_motor_off();
    ir_heat_on();

    while(seconds_elapsed < 5) {
        wait_one_second();  // 原有50ms*20的等待逻辑
        seconds_elapsed++;
    }

    /* 阶段2：停止加热并启动泵 */
    ir_heat_off();
    initial_temp = G_tp_value;
    G_motor_rate = 25;
    start_pump(G_motor_rate, MOTOR_INT_OCCL_TEST);

    /* 阶段3：温度监测（最多20秒）*/
    for(seconds_elapsed = 0; seconds_elapsed < 20; seconds_elapsed++) {
        wait_one_second();

        if ((G_tp_value - initial_temp) > 30) {
            return 0;  // 检测到温升→管道满
        }
    }

    return 1;  // 未检测到温升→管道空
}

/**
 * @brief 原有1秒等待逻辑的封装
 */
static void wait_one_second(void)
{
    uint8_t count = 0;
    while(count < 20) {  // 20*50ms=1s
        while(G_tick5ms_counter < 10);  // 等待50ms
        G_tick5ms_counter = 0;
        count++;
    }
}


/**************************************
 * empty_test_w_heat()
 * @detail
 * Short heat signature test to verify if the tubing is empty or full. Pump motor is stopped and the
 * IR heat is activated for a 5 seconds. Pump motor is started and the TP temperature is monitored for
 * a heatrise. "1" is returned if the heat rise is detected (tubing is full). "0" is
 * for no heat rise (empty tubing). Temperature is monitored for 25 seconds or less if the specified
 * heat rise is detected earlier.
 */
#if(0)
uint8_t empty_test_w_heat(void)
{
  uint8_t test_timer;
  uint8_t one_second_count;
  uint16_t occl_offset;
  test_timer=0;
  one_second_count=0;
  pump_motor_off();
  ir_heat_on();									//test HEAT ON
  do{
	if (one_second_count >= ONE_SECOND){
	  test_timer++;
	  one_second_count = 0;
	  if (test_timer==5) {            			//test HEAT Off stat motor at 25ml/h
		ir_heat_off();
		occl_offset=G_tp_value;
		G_motor_rate = 25;
		start_pump(G_motor_rate,MOTOR_INT_OCCL_TEST);
	  }
	  if (test_timer> 5){
		if ((G_tp_value-occl_offset) > 30){     //Heat rise detected
		  return (0);                         	//tubing full, return 0
		}
	  }
	  if (test_timer > 25){                     //Test time exceeded
		  return(1);							//tubing empty, return 1
	  }
	}
	while (G_tick5ms_counter<10);    //50ms loop
    G_tick5ms_counter = 0;
    one_second_count++;

  } while(1);
}
#endif
/** End empty_test_w_heat() **/

/**************************************************************
 * @brief 霍尔传感器检测
 * @return 1-检测到磁铁 0-未检测到
 * @note 通过GPIOB_PIN0电平判断磁铁状态
 **************************************************************/
uint8_t magnet(void)
{
   // return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1 : 0;
    return (MAGNET_READ()== GPIO_PIN_RESET) ? 1 : 0;
}

/**************************************
 * magnet()
 * @detail
 * Read hall sensor. 1 magnet detected, 0 no magnet
 */
#if(0)
uint8_t magnet(void){
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)){
	  return(0);                                //no magnet
	}
	else{
	  return (1);								//magnet detected
	}
}
#endif
/** End magnet() **/

/**************************************
 * manualprime()
 * @brief 手动灌注控制函数
 * @detail
 * 1. 配方键按下: 关闭水剂管道，启动配方灌注(750mL/h)
 * 2. 水剂键按下: 关闭配方管道，启动水剂灌注(750mL/h)
 * 3. 无按键时停止泵
 * 4. 右键按下退出功能
 */
void manualprime()
{
    typedef enum {IDLE, PRIME_FOOD, PRIME_WATER} prime_state_t;
    prime_state_t state = IDLE;
    uint8_t key;
    uint16_t timeout_count;

    while(1) {
        manual_primescreen(SET_READY);
        key = read_all_keys(&timeout_count);

        /* 处理按键输入 */
        switch(key) {
            case PRIME_TAG_FOOD:  // 配方灌注键
           // 	key_click();
                if(state != PRIME_FOOD) {
                    move_pincher_all_ccw();  // 关闭水剂管道

                    state = PRIME_FOOD;
                }
                start_pumping(FULL_FOOD_RATE);
                break;

            case PRIME_TAG_WATER:  // 水剂灌注键
                if(state != PRIME_WATER) {
                    move_pincher_all_cw();  // 关闭配方管道

                    state = PRIME_WATER;
                }
                start_pumping(FULL_FOOD_RATE);
                break;

            case NO_TOUCH:  // 无按键
                stop_pumping();
                state = IDLE;
                break;

            case (RIGHT_ARROW_PRESSED):
                key_click();
                break;

            case (RIGHT_ARROW_RELEASED):  // 右键释放
                stop_pumping();
                return;  // 退出函数
        }

        /* 处理1ml标志 */
        if(G_one_ml_flag) {
            G_one_ml_flag = 0;  // 重置标志
        }

        delay_50ms();  // 50ms延时
    }
}

/** 辅助函数 **/
static void start_pumping(uint16_t rate)
{
    start_pump(rate, MOTOR_INT_BOLUS);
}

static void stop_pumping(void)
{
    pump_motor_off();
}


/**************************************
 * manualprime()
 * @detail
 * Manual prime. Run pump and actuate pincher valve to manually prime formula and bolus
 * Exit if right arrow pressed or after idle time out (key not pressed)
 */

#if(0)
void manualprime(){
  uint8_t key,pincher_flag,pump_flag;
  uint16_t timeout_count;
  pincher_flag=0;
  pump_flag=0;
  while(1){
	manual_primescreen(SET_READY);
	key=read_all_keys(&timeout_count);

	if (key == PRIME_TAG_FOOD ) {       //prime formula button pressed
	  if (pincher_flag!=1){     //if prime formula button pressed 1st time, close bolus tubing
		move_pincher_all_ccw();
	    pincher_flag=1;
	  }
	  if (pump_flag==0){
		pump_flag=1;
	    start_pump(FULL_FOOD_RATE,MOTOR_INT_BOLUS);      //start motor 750mL/h
	  }
	}
	if (key == PRIME_TAG_WATER ) {           			//prime bolus button pressed
	  if (pincher_flag!=2){                      //if prime bolus pressed 1st time pinch formula
		move_pincher_all_cw();
		pincher_flag=2;
	  }
	  if (pump_flag==0){
	  	pump_flag=1;
	  	start_pump(FULL_FOOD_RATE,MOTOR_INT_BOLUS);   //start motor 750mL/h
	  }
	}
	if (key == NO_TOUCH) {           			//key released stop motor and clear flag
      pump_flag=0;
	  pump_motor_off();
	}
	if (key == (RIGHT_ARROW_PRESSED)) key_click();
	if (key == (RIGHT_ARROW_RELEASED)) { 	//right arrow pressed
	  pump_motor_off();
	  return;
	}
	if (G_one_ml_flag){
	  G_one_ml_flag = 0;  //reset one mL flag to prevent one mL counter watchdog reset
	}

	while (G_tick5ms_counter<10);    //50ms loop
	G_tick5ms_counter = 0;
  }
}/**  End manualprime()  **/
#endif

/**
 * @brief Rotate pincher to fully closed (CW) position
 * @details Moves pincher clockwise until fully closed or magnet detected.
 *          If magnet is detected during movement, completes full 220-step rotation.
 * @return uint8_t Status code (always returns ALL_CW)
 */
uint8_t move_pincher_all_cw(void)
{
    // Initial movement until magnet detection or full rotation
    for (uint16_t step = 0; step < FULL_ROTATION_STEPS; step++) {
#if (MOTOR_TYPE == 1)
    pinch_n_steps(1, PINCH_DIR_CW);
#elif(MOTOR_TYPE == 2)
    pinch_n_steps(6, PINCH_DIR_CW);
#endif
        if (magnet()) {
            // If magnet detected, complete the remaining steps
        	/* 仅在检测到磁铁时计算剩余步数（节省栈空间） */
            uint16_t remaining_steps = FULL_ROTATION_STEPS - step - 1;
            if (remaining_steps > 0) {
#if(MOTOR_TYPE == 2)
            	Motor_Init();
#endif
                for(uint16_t step = 0; step < remaining_steps; step++)
#if (MOTOR_TYPE == 1)
    pinch_n_steps(1, PINCH_DIR_CW);
#elif(MOTOR_TYPE == 2)
    pinch_n_steps(6, PINCH_DIR_CW);
#endif
          }
            break;
        }
    }
    g_pincher_result.position = POS_LEFT;
    return ALL_CW;
}


/**************************************
 * move_pincher_all_cw()
 * @detail
 * Rotate pincher all clockwise (Food tubing closed). To reduce required motor steps,
 * read the hall sensor(center position) and restart the the step count if magnet detected.
  */
#if(0)
uint8_t move_pincher_all_cw(void)
{
  uint8_t magnet_flag;
  uint16_t step_count;
  step_count=0;
  magnet_flag=0;
  while ((step_count < 220) && (!magnet_flag)){  	//Rotate CW until limit or
	step_count++;									//magnet detected
    pinch_n_steps(1,PLEFT);
    if (magnet()) magnet_flag=1;
  }
  if (magnet_flag) pinch_n_steps(220,PLEFT);
  return (ALL_CW);
}
#endif
/** End move_pincher_all_cw() **/
/**
 * @brief Rotate pincher to fully open (CCW) position
 * @details Moves pincher counter-clockwise until fully open or magnet detected.
 *          If magnet is detected during movement, completes full 220-step rotation.
 * @return uint8_t Status code (always returns ALL_CCW)
 */
#if(1)
uint8_t move_pincher_all_ccw(void)
{
	   uint16_t steps_to_move;

	    // 根据当前位置决定要走的步数
	    if (g_pincher_result.position == POS_LEFT) {
	        // 从左边到右边：一整圈
	        steps_to_move = FULL_ROTATION_STEPS;
	    } else {
	        // 从中间或右边：都只转半圈
	        // 从中间到右边：半圈
	        // 从右边再转半圈：挤压
	        steps_to_move = FULL_ROTATION_STEPS * 3 / 4;
	    }
    // Initial movement until magnet detection or full rotation
    for (uint16_t step = 0; step < steps_to_move; step++) {
	#if (MOTOR_TYPE == 1)
        pinch_n_steps(1, PINCH_DIR_CCW);
    #elif(MOTOR_TYPE == 2)
        pinch_n_steps(6, PINCH_DIR_CCW);
	#endif

        if (magnet()) {
            // Complete remaining rotation if magnet detected mid-way
        	/* 仅在检测到磁铁时计算剩余步数（节省栈空间） */
            //uint16_t remaining_steps = steps_to_move - step - 1;
            uint16_t remaining_steps = FULL_ROTATION_STEPS / 2 ;
            if (remaining_steps > 0) {
#if(MOTOR_TYPE == 2)
            	Motor_Init();
#endif
                for(uint16_t step = 0; step < remaining_steps; step++)
#if (MOTOR_TYPE == 1)
    pinch_n_steps(1, PINCH_DIR_CCW);
#elif(MOTOR_TYPE == 2)
    pinch_n_steps(6, PINCH_DIR_CCW);
#endif
 //               	pinch_n_steps(remaining_steps, PINCH_DIR_CCW);
            }
            break;
        }
    }
    g_pincher_result.position = POS_RIGHT;
    return ALL_CCW;
}

#endif
/**************************************
 * move_pincher_all_ccw()
 * @detail
 * Rotate pincher all counter clockwise (Bolus tubing closed). To reduce required motor steps,
 * read the hall sensor(center position) and restart the the step count if magnet detected.
 */
#if(0)
uint8_t move_pincher_all_ccw(void)
{
  uint8_t magnet_flag;
  uint16_t step_count;
  step_count=0;
  magnet_flag=0;
  while ((step_count < 220) && (!magnet_flag)){ //Rotate CCW until limit or
	step_count++;								//magnet detected
    pinch_n_steps(1,PRIGHT);
    if (magnet()) magnet_flag=1;
  }
  if (magnet_flag) pinch_n_steps(220,PRIGHT);   //If magnet detected, rotate all CCW
  return (ALL_CCW);
}
#endif
/** End move_pincher_all_ccw() **/


/**
 * @brief Rotates pincher from fully CW position to center magnet position
 * @details Moves pincher counter-clockwise (PRIGHT) until magnet is detected or max steps reached
 * @return uint8_t Status code:
 *         - AT_MAGNET: Magnet found within normal range (≤150 steps)
 *         - NO_TPA: Magnet found but took too many steps (>150 steps)
 *         - ERR_NO_MAGNET: Magnet not found within 250 steps
 */
uint8_t move_pincher_from_all_cw_to_magnet(void)
{

    for (uint16_t step = 0; step < kPincherMaxSteps; step++) {
#if(MOTOR_TYPE ==1)
        pinch_n_steps(1, PINCH_DIR_CCW);
#elif(MOTOR_TYPE ==2)
        pinch_n_steps(6, PINCH_DIR_CCW);
#endif

        if (magnet()) {
            return (step > kWarningThreshold) ? NO_TPA : AT_MAGNET;
        }

        // Optional: Add small delay between steps if motor requires it
        // delay_ms(MOTOR_STEP_DELAY_MS);
    }

    return ERR_NO_MAGNET;
}

 /**************************************
  * move_pincher_from_all_cw_to_magnet()
  * @detail      Rotate pincher from left (food tubing closed) position to center.
  * 				It is assumed that the pincher has been actuated to all CW position before
  * 				this function is called.
  * 				Pincher motor is actuated CCW direction until the the sensor magnet is detected
  * 				(center). Required steps are counted to detect the status of the move.
  */

#if(0)
uint8_t move_pincher_from_all_cw_to_magnet(void)
{
  uint8_t magnet_flag;
  uint16_t step_count;
  magnet_flag = 0;
  step_count=0;
  while ((step_count < 250) && (!magnet_flag)){			//Step CCW until magnet1
	pinch_n_steps(1,PRIGHT);
    step_count++;
    if (magnet()) magnet_flag=1;
  }
  if (magnet_flag){
	if (step_count > 150){
	  return(NO_TPA);
	}
	else{
	  return (AT_MAGNET);
	}
  }
  else{
	return(ERR_NO_MAGNET);
  }
}
#endif
/** End move_pincher_from_all_cw_to_magnet() **/

/**
 * @brief Rotates pincher from fully CCW position to center magnet position
 * @return uint8_t Detection result (NO_TPA/SINGLE_TPA/DUAL_TPA/ERR_NO_MAGNET)
 *
 * @details
 * 1. Rotates pincher clockwise until magnet is detected or maximum steps reached
 * 2. Counts steps to determine TPA type
 * 3. Returns error if magnet not found within expected range
 *
 * @note Assumes pincher is already in full CCW position before calling
 */
uint8_t move_pincher_from_all_ccw_to_magnet(PincherDetectionResult *result)
{
 //   PincherDetectionResult result = {
 //       .magnet_detected = false,
 //       .step_count = 0
 //   };

    /* Rotate until magnet found or max steps reached */
    rotate_to_magnet(result);

    /* Analyze results and return appropriate status */
    return evaluate_pincher_position(result);
}


/**
 * @brief Rotates pincher while monitoring for magnet detection
 * @param result Pointer to store detection results
 */
static void rotate_to_magnet(PincherDetectionResult *result)
{
    while ((result->step_count < PINCHER_HALF_MOVE) && !result->magnet_detected) {
#if(MOTOR_TYPE ==1)
        pinch_n_steps(1, PINCH_DIR_CW);
#elif(MOTOR_TYPE ==2)
        pinch_n_steps(6, PINCH_DIR_CW);
#endif
        result->step_count++;
        result->magnet_detected = magnet_sensor_detected();
        if(result->magnet_detected){
        	result->position = POS_MID;
        }

        /* Small delay for sensor stabilization */
       // delay_microseconds(SENSOR_READ_DELAY_US);
    }
}

/**
 * @brief Evaluates pincher position based on step count
 * @param result Detection results from rotation
 * @return uint8_t TPA type or error code
 */
static uint8_t evaluate_pincher_position(const PincherDetectionResult *result)
{
    if (!result->magnet_detected) {

        return ERR_NO_MAGNET;
    }

    if (result->step_count > NO_TPA_TH) {
        return NO_TPA;
    }

    if (result->step_count < SINGLE_TPA_TH) {
        return SINGLE_TPA;
    }

    return DUAL_TPA;
}

/**
 * @brief Checks magnet sensor status
 * @return bool True if magnet detected, false otherwise
 */
static bool magnet_sensor_detected(void)
{
    return magnet() != 0;
}

/**************************************
 * move_pincher_from_all_ccw_to_magnet()
 * @detail      Rotate pincher from right (bolus tubing closed) position to center.
 * 				It is assumed that the pincher has been actuated to all CCW position before
 * 				this function is called.
 * 				Pincher motor is actuated CW direction until the the sensor magnet is detected
 * 				(center). Required steps are counted to detect the TPA type or posible error
 * 				if the magnet is not detected.
 */
#if(0)
uint8_t move_pincher_from_all_ccw_to_magnet(void)
{
  uint8_t magnet_flag;
  uint16_t step_count;
  magnet_flag = 0;
  step_count=0;
  while ((step_count < PINCHER_HALF_MOVE) && (!magnet_flag)){
	pinch_n_steps(1,PLEFT);
    step_count++;
    if (magnet()) magnet_flag=1;
  }
  if (magnet_flag){
	if (step_count > NO_TPA_TH){
	  return(NO_TPA);
	}
	if (step_count<90){
	  return (SINGLE_TPA_TH);
	}
	else{
	  return (DUAL_TPA);
	}
  }
  else{
	return(ERR_NO_MAGNET);
  }
}
#endif
/** End move_pincher_from_all_ccw_to_magnet() **/

/**
 * @brief Rotate pincher to find center position using magnet sensor
 * @details Rotates pincher left until magnet is detected or half move completed.
 *          If not found, rotates right to search. Assumes pincher starts at CCW position.
 * @return uint8_t Status code: AT_MAGNET if found, ERR_NO_MAGNET if not found, ERR_OTHER if found in first search
 */
uint8_t move_pincher_to_center(void)
{
    // First search: rotate clockwise (PLEFT)
    for (uint16_t step_count = 0; step_count < PINCHER_HALF_MOVE; step_count++) {
#if(MOTOR_TYPE ==1)
        pinch_n_steps(1, PINCH_DIR_CW);
#elif(MOTOR_TYPE ==2)
        pinch_n_steps(6, PINCH_DIR_CW);
#endif
    	    if (magnet()) {
            return AT_MAGNET;
        }
    }

    // Second search: rotate counter-clockwise (PRIGHT)
    for (uint16_t step_count = 0; step_count < PINCHER_HALF_MOVE; step_count++) {
#if(MOTOR_TYPE ==1)
        pinch_n_steps(1, PINCH_DIR_CCW);
#elif(MOTOR_TYPE ==2)
        pinch_n_steps(6, PINCH_DIR_CCW);
#endif
        if (magnet()) {
            return AT_MAGNET;
        }
    }

    return ERR_NO_MAGNET;
}
/** End move_pincher_to_center() **/


/**************************************
 * move_pincher_to_center()
 * @detail      Rotate pincher to left until magnet or half move.
 * 				It is assumed that the pincher has been actuated to all CCW position before
 * 				this function is called.
 * 				Pincher motor is actuated CW direction until the the sensor magnet is detected
 * 				(center). If the center has not been found after 90 degree move, same is
 * 				repeated CCW direction.
 */

#if(0)
uint8_t move_pincher_to_center(void)
{
  uint8_t magnet_flag;
  uint16_t step_count;
  magnet_flag = 0;  step_count=0;
  while ((step_count < PINCHER_HALF_MOVE) && (!magnet_flag)){
	pinch_n_steps(1,PLEFT);    step_count++;
    if (magnet()) magnet_flag=1;  }
  if (!magnet_flag){
	magnet_flag = 0;	step_count=0;
	while ((step_count < PINCHER_HALF_MOVE) && (!magnet_flag)){
	  pinch_n_steps(1,PRIGHT);  step_count++;
	  if (magnet()) magnet_flag=1;
	}
	if (magnet_flag){ return(AT_MAGNET);
	}
	else{
	  return(ERR_NO_MAGNET);
    }
  }
  return(ERR_OTHER);
}
#endif
/** End move_pincher_to_center() **/

/**
 * @brief Step pinch motor in specified direction for given steps
 * @param steps Number of steps to move (0 to 65535)
 * @param direction Movement direction (PLEFT_CW or PRIGHT_CCW)
 * @note Automatically powers off coils after movement
 */
#if(0)
void pinch_n_steps(uint16_t steps, PinchDirection direction)
{
    static uint8_t phase_state = 0;
    if (steps == 0) {  return; // Early exit for zero steps
    }
    // Optimized step generation loop
    while (steps--) {   if (direction == PINCH_DIR_CW) {
            phase_state = (phase_state + 1) % 4; // CW phase advance
        } else {
            phase_state = (phase_state - 1) % 4; // CCW phase retreat
        }
        apply_motor_phase(phase_state);
    }    power_off_motor_coils();
}
#endif

/**************************************
 * pinch_n_steps()
 * @detail
 * Step pinch motor n steps left (CW) or right (CCW) clockwise
 */
#if(1)
void pinch_n_steps(uint16_t steps, uint8_t pinch_side)
{
	uint16_t i;
//	static uint8_t pinch_phase;

	for (i=0; i<steps; i++){
		if (pinch_side == PLEFT) {
			pinch_cw(&pinch_phase);
		}
		else{
			pinch_ccw(&pinch_phase);
		}
	}
	//power off all coils
#if(MOTOR_TYPE == 1)
	HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
#endif
}
#endif
/** End pinch_n_steps() **/

#if (MOTOR_TYPE == 2)

void Motor_Init(void)
{
    // 设置步1状态：只有D导通
    HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_SET);   // D导通
    //pinch_phase = 0;  // 全局变量，需要声明为外部可访问
    Motor_SetPhase(0);
}

void Motor_SetPhase(uint8_t phase)
{
    pinch_phase = phase;
}
#endif

/**
 * @brief 应用电机相位（安全版本）
 */
#if(MOTOR_TYPE == 1)

static inline void apply_motor_phase(uint8_t phase)
{
	 /* Ensure phase is within 0-3 range (defensive programming) */
	phase %= 4;
    HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, STEP_PHASE_TABLE[phase][0]);
    HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, STEP_PHASE_TABLE[phase][1]);
    HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, STEP_PHASE_TABLE[phase][2]);
    HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, STEP_PHASE_TABLE[phase][3]);
}

#endif
/**
 * @brief Powers off all motor coils
 */
static inline void power_off_motor_coils(void)
{
    const GPIO_PinState OFF_STATE = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, OFF_STATE);
    HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, OFF_STATE);
    HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, OFF_STATE);
    HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, OFF_STATE);
}
/**
 * @brief 逆时针旋转一步
 * @param pinch_phase 当前相位指针 (自动更新)
 */
#if (MOTOR_TYPE == 2)
void pinch_ccw(uint8_t *pinch_phase)
{
    switch (*pinch_phase)
    {
    case 0: // 步0 -> 步7：蓝 → 黑+蓝
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_SET);   // 黑
        // 蓝保持 SET，黄、棕保持 RESET
        *pinch_phase = 7;
        break;

    case 7: // 步7 -> 步6：黑+蓝 → 黑
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET); // 蓝关
        // 黑保持 SET，黄、棕保持 RESET
        *pinch_phase = 6;
        break;

    case 6: // 步6 -> 步5：黑 → 黄+黑
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_SET);   // 黄
        // 黑保持 SET，蓝、棕保持 RESET
        *pinch_phase = 5;
        break;

    case 5: // 步5 -> 步4：黄+黑 → 黄
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET); // 黑关
        // 黄保持 SET，蓝、棕保持 RESET
        *pinch_phase = 4;
        break;

    case 4: // 步4 -> 步3：黄 → 棕+黄
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_SET);   // 棕
        // 黄保持 SET，蓝、黑保持 RESET
        *pinch_phase = 3;
        break;

    case 3: // 步3 -> 步2：棕+黄 → 棕
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET); // 黄关
        // 棕保持 SET，蓝、黑保持 RESET
        *pinch_phase = 2;
        break;

    case 2: // 步2 -> 步1：棕 → 蓝+棕
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_SET);   // 蓝
        // 棕保持 SET，黄、黑保持 RESET
        *pinch_phase = 1;
        break;

    case 1: // 步1 -> 步0：蓝+棕 → 蓝
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET); // 棕关
        // 蓝保持 SET，黄、黑保持 RESET
        *pinch_phase = 0;
        break;

    default:
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
        *pinch_phase = 0;
        break;
    }

#if (MOTOR_TYPE == 2)
 //   HAL_Delay(2);
    delay_us(1500);
#else
    HAL_Delay(3);
#endif
}
#else
void pinch_ccw(uint8_t *pinch_phase)
{
	*pinch_phase = (*pinch_phase == 0) ? 3 : (*pinch_phase - 1);  // 确保相位在0-3范围内;
    apply_motor_phase(*pinch_phase);
    HAL_Delay(3); // 保持步进间隔
}

#endif


/**************************************
 * pinch_ccw()
 * @detail      Step pinch motor one step counter clockwise
 *
 */
#if(0)
void pinch_ccw(uint8_t *pinch_phase)
{
	switch (*pinch_phase){
	case 3:				// full step CCW from position AD to position DC
		HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
		*pinch_phase =2;
		break;
	case 2:							// full step CCW from position DC to position CB
		HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
		*pinch_phase=1;
		break;
	case 1:							// full step CCW from position CB to position BA
		HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
		*pinch_phase=0;
		break;
	case 0:							// full step CCW from position BA to position AD
		HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
		*pinch_phase=3;
		break;
	default:						//reset phase counter if phase not defind
		HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
		*pinch_phase=0;
		break;
	}
	HAL_Delay(3);
}
#endif
/** End pinch_ccw() **/
/**
 * @brief 顺时针旋转一步
 * @param pinch_phase 当前相位指针 (自动更新)
 */
#if (MOTOR_TYPE == 2)

void pinch_cw(uint8_t *pinch_phase)
{
    switch (*pinch_phase)
    {
    case 0: // 步0 -> 步1：蓝 → 蓝+棕
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_SET);   // 棕
        // 蓝保持 SET，黄、黑保持 RESET
        *pinch_phase = 1;
        break;

    case 1: // 步1 -> 步2：蓝+棕 → 棕
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET); // 蓝关
        // 棕保持 SET，黄、黑保持 RESET
        *pinch_phase = 2;
        break;

    case 2: // 步2 -> 步3：棕 → 棕+黄
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_SET);   // 黄
        // 棕保持 SET，蓝、黑保持 RESET
        *pinch_phase = 3;
        break;

    case 3: // 步3 -> 步4：棕+黄 → 黄
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET); // 棕关
        // 黄保持 SET，蓝、黑保持 RESET
        *pinch_phase = 4;
        break;

    case 4: // 步4 -> 步5：黄 → 黄+黑
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_SET);   // 黑
        // 黄保持 SET，蓝、棕保持 RESET
        *pinch_phase = 5;
        break;

    case 5: // 步5 -> 步6：黄+黑 → 黑
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET); // 黄关
        // 黑保持 SET，蓝、棕保持 RESET
        *pinch_phase = 6;
        break;

    case 6: // 步6 -> 步7：黑 → 黑+蓝
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_SET);   // 蓝
        // 黑保持 SET，黄、棕保持 RESET
        *pinch_phase = 7;
        break;

    case 7: // 步7 -> 步0：黑+蓝 → 蓝
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET); // 黑关
        // 蓝保持 SET，黄、棕保持 RESET
        *pinch_phase = 0;
        break;

    default:
        HAL_GPIO_WritePin(TPA_STEP_A_GPIO_Port, TPA_STEP_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_B_GPIO_Port, TPA_STEP_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_C_GPIO_Port, TPA_STEP_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPA_STEP_D_GPIO_Port, TPA_STEP_D_Pin, GPIO_PIN_RESET);
        *pinch_phase = 0;
        break;
    }
#if (MOTOR_TYPE == 2)
 //   HAL_Delay(2); //   HAL_Delay(2);
    delay_us(1500);
#else
    HAL_Delay(3);
#endif
}

#else

#if(1)
void pinch_cw(uint8_t *pinch_phase)
{
	 *pinch_phase = (*pinch_phase + 1) % 4;  // 确保相位在0-3范围内; // 相位递增
    apply_motor_phase(*pinch_phase);
    HAL_Delay(3); // 保持步进间隔
}
#endif

#endif
/**************************************
 * pinch_cw()
 * @detail      Step pinch motor one step clockwise
 *
 */

/** End pinch_cw() **/

/**************************************
 * pump_motor_off()
 * @detail
 * Stop pump motor by disabling the interrupt and turn the coil power off.
 */
void pump_motor_off(void){
	stop_pump_motor();
	HAL_GPIO_WritePin(PUMP_STEP_A_GPIO_Port, PUMP_STEP_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PUMP_STEP_B_GPIO_Port, PUMP_STEP_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PUMP_STEP_C_GPIO_Port, PUMP_STEP_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PUMP_STEP_D_GPIO_Port, PUMP_STEP_D_Pin, GPIO_PIN_RESET);
	ir_heat_off();
}/** End pump_motor_off() **/

/**************************************
 * service_mode_only()
 * @detail
 * When the pump is started with RUN?STOP button held down, it starts in service mo displaying the
 * service pages only.
 */

void service_mode_only(void)
{
uint8_t menu_no;
  menu_no = SERVICE_PAGE_ONE__TASK;
  while (1){
	switch (menu_no){
	  case SERVICE_PAGE_ONE__TASK:
	    menu_no = service_page_one_task();
	    break;
	  case SERVICE_PAGE_TWO__TASK:
	    menu_no =service_page_two_task();
	    break;
	  case SERVICE_PAGE_THREE__TASK:
	    menu_no =service_page_three_task();
	    break;
	  case SET_TIME__TASK:
	    menu_no = set_time_task();
	    menu_no = SERVICE_PAGE_ONE__TASK;
	    break;
	  case HOLD_MODE__TASK:
		power_off_pump();
	  	break;

	  default:
	    power_off_pump();
	    break;
	}
  }
}/** End service_mode_only() **/

/**************************************
 * service_mode_req()
 * @detail
 * Return 1 if RUN/STOP pressed during power on.
 */
uint8_t service_mode_req(void)
{
  uint8_t key;
  uint16_t timeout_count;
  // 添加短暂延迟，确保按键状态更新
  if(G_cal_status==CAL_ERROR){
	 HAL_Delay(5000);
  }

  // 多次读取以确保获取正确的按键状态
   for (int i = 0; i < 5; i++) {
	   key = read_all_keys(&timeout_count);

 // if ((key & RUN_STOP_SWITCH) == RUN_STOP_SWITCH){
 //   beep_1khz_p5sec(NORMAL_TONE);
 //   return 1;
 // }
	   if (key == (RUN_STOP_SWITCH | PRESSED_BIT)) return 1;
	   if (key == RUN_STOP_SWITCH ) return 1;
	   // 短暂延迟后再次尝试
	       HAL_Delay(10);
	     }
  return 0;
}/**  End service_mode_req()  **/

/**************************************
 * start_pump_w_old_rate()
 * @detail
 * Start pump motor with existing rate by enabling motor interrupt (TIM21).
 */
void start_pump_w_old_rate(void)
{
  HAL_TIM_Base_Start_IT(&htim21);
}/** End start_pump_w_old_rate() **/

/**************************************
 * start_pump()
 * @detail
 * Start pump motor. Set pumping rate and the timer interrupt reload value TIM21.
 */
void start_pump(uint16_t rate,uint16_t int_time_ds)
{
  G_motor_rate = rate;
 // htim21.Init.Period = int_time_ds;
  TIM21->ARR = int_time_ds;
  HAL_TIM_Base_Start_IT(&htim21);
}/** End start_pump() **/

/**************************************
 * stop_pump()
 * @detail		Stop pump motor by disabling the interrupt. Coils stay powered.
 */
void stop_pump_motor()
{
  HAL_TIM_Base_Stop_IT(&htim21);
}/**  End stop_pump() **/

__attribute__((always_inline))
static inline void delay_50ms(void)
{
    while(G_tick5ms_counter < 10);  // 等待50ms
    G_tick5ms_counter = 0;
}
