/*****************************************************************************
* @file			: common.h
* @short        : defines used in multiple files
******************************************************************************
*
* @details
* This file consist commonly used defines
*
******************************************************************************
*/

#ifndef _COMMON_H
#define _COMMON_H

// ===== 电机参数配置 =====
// 选择使用哪种电机：1-旧电机，2-新电机
#define MOTOR_TYPE   1   // 1:旧电机(7.5° 1:15)，2:新电机(BH-20BYJ46 1:85)

#define USE_SPEED_INTERVAL  0 // 1=启用20%加速，0=使用原间隔，2=减速

//#define TEST_MODE_volume 0   //可以修改


//#define TEST_Compensation_Volume 1   //可以修改
// 在头文件或配置文件开头定义
#define USE_HOLD_MODE_NEW_SCREEN    1   // 1: 使用新显示, 0: 使用旧显示

#define USE_UPDATE_DYNAMIC_CONTROL  1   // 1: 使用暂停功能, 0: 不适用使用旧显示

// 定义速度模式
#define SPEED_MODE_NORMAL 0
#define SPEED_MODE_FAST 1
#define SPEED_MODE_SLOW 2

// 设置当前速度模式
#define CURRENT_SPEED_MODE SPEED_MODE_FAST

#define CH_USONIC 0					//STM32 channel ADC_CHANNEL_0
#define CH_PD2 1					//STM32 channel ADC_CHANNEL_1
#define CH_V_BAT 2					//STM32 channel ADC_CHANNEL_5
#define CH_TP_NTC 3					//STM32 channel ADC_CHANNEL_6
#define CH_TP_TEMP 4				//STM32 channel ADC_CHANNEL_7
#define CH_PTS 5					//STM32 channel ADC_CHANNEL_14
#define CH_ALRM_MON 6               //STM32 channel ADC_CHANNEL_13

									//MAXIMUX TAG NO is 63, bits 7 and 8 are used for first pressed
									//and released flags
									//touch tag numbers 0x01 to 0x3f
									//bit 6 (0x40)mpreserved for pressed
									//bit 7 (0x80) preserved for released
#define BOLUS_ARROW_TAG 0x04
#define UP_ARROW_TAG 0x05			//Tag number for the up arrow touch button
#define RIGHT_ARROW_TAG 0x06		//Tag number for the left arrow touch button
#define DOWN_ARROW_TAG 0x07			//Tag number for the down arrow touch button
#define RATE_TEXT_TAG 0x08			//tag number for the rate numeric text
#define VOLUME_TEXT_TAG 0x09		//tag number for the volume numeric text
#define PRIME_TAG 0x0a		        //tag number for the prime button
#define CLR_VOLUME_TAG 0x0b
#define CLR_DOSE_TAG 0x0c
#define CLR_TOT_VOLUME_TAG 0x0d
#define YES_TAG 0x0e
#define NO_TAG 0x0f
#define ANN_LOW_TAG 0x10
#define ANN_HI_TAG 0x11
#define DSP_BRG_LOW_TAG 0x12
#define DSP_BRG_HI_TAG 0x13
#define ALARM_LOW_TAG 0x14
#define ALARM_HI_TAG 0x15
#define DOSE_COMP_ALARM_5S_TAG 0x16
#define DOSE_COMP_ALARM_60S_TAG 0x17
#define DOSE_COMP_ALARM_5M_TAG 0x18
#define WATER_TAG 0x19
#define FORMULA_TAG 0x1a
#define IR_CAL_TAG 0x1b
#define HOUR_PLUS_TAG 0x1c
#define HOUR_MINUS_TAG 0x1d
#define MINUTE_PLUS_TAG 0x1e
#define MINUTE_MINUS_TAG 0x1f
#define DAY_PLUS_TAG 0x20
#define DAY_MINUS_TAG 0x21
#define MONTH_PLUS_TAG 0x22
#define MONTH_MINUS_TAG 0x23
#define YEAR_PLUS_TAG 0x24
#define YEAR_MINUS_TAG 0x25
#define EXIT_TAG 0x26
#define RESET_TAG 0x27
#define SET_TAG 0x28
#define SVC_ENTRY_3S_TAG 0x29
#define SVC_ENTRY_6S_TAG 0x2a
#define SVC_ENTRY_9S_TAG 0x2b
#define FLOW_SENSOR_TAG 0x2c
#define PIEZO_TAG 0x2d
#define NULL_TAG 0x2e
#define PRIME_TAG_WATER 0x2f
#define PRIME_TAG_FOOD 0x30
#define PUMP_TAG 0x31
#define DAY_LOG_TAG 0x32
#define ERR_LOG_TAG 0x33
#define CAL_LOG_TAG 0x34
#define IR_HEAT_TAG 0x35
#define TOUCH_CALIBRATE_TAG 0x36
#define BATTERY_MUTE_TAG  0x37
#define ERROR_MUTE_TAG    0x38
#define RUN_LOCK_TAG    0x39
#define CLR_TOT_WATER_TAG 0x3a

// 体积参数检查
#define CHECK_VOLUME(var, flag) \
    if (var < 0 || var > 9999) { var = 0; flag = 1; }

#define CHECK_SETTING(var, max_val, flag) \
    if (var > max_val) { var = max_val; flag = 1; }
/* 特殊常量定义 */
#define HOUR_VOLUME_MAX     400  // 业务特殊要求
#define FOOD_RATE_DEFAULT   5
#define WATER_RATE_DEFAULT  10




typedef enum {
    PRIME_STAGE_WATER,    // 阶段1: 水剂灌注
    PRIME_STAGE_FORMULA,  // 阶段2: 营养液灌注
    PRIME_STAGE_FINAL    // 阶段3: 最终灌注
} prime_stage_t;


typedef enum {
    PINCHER_LEFT,    // 左侧位置
    PINCHER_CENTER,  // 中心位置
    PINCHER_RIGHT    // 右侧位置
} PincherPosition;

#define YES                 1       // 确认操作
#define NO                  0       // 取消操作
#define SHUTDOWN_CONFIRMED  YES     // 关机确认状态

#define CAM_LEFT 0
#define CAM_RIGHT 1
#define CAM_CENTER 2

#define PLEFT 0
#define PRIGHT 1



#define NOT_PRESSED 0           	//No switch pressed
#define NO_TOUCH 0					//no touch input pressed
#define RUN_STOP_SWITCH 10    		//RUN/STOP switch pressed
#define POWER_SWITCH 11         	//POWER switch pressed

// 在配置文件或头文件中定义
#define ENABLE_LOW_FLOW_MODE  1  // 1=支持低速模式(1-4ml/min)，0=不支持



#define DISPLAY_LOW_FLOW_INFO 1

#ifdef ENABLE_LOW_FLOW_MODE
    #ifndef DISPLAY_LOW_FLOW_INFO
        #define DISPLAY_LOW_FLOW_INFO 1  // 默认启用
    #endif
#else
    #define DISPLAY_LOW_FLOW_INFO 0  // 低速模式禁用时自动关闭显示
#endif



#ifdef LOW_FLOW_TEST_MODE
    #define HOUR_DURATION_MS  1000000  // 测试模式：20分钟
#else
    #define HOUR_DURATION_MS  3600000  // 正常模式：1小时
#endif
#define PAUSE_ACCUMulated_TIME_THRESHOLD  3000000  // 正常模式：50分钟

#define HOUR_TEST_TIME_THRESHOLD  3360000  // 正常模式：56分钟

#define MAX_WAIT_TIME_MS 120000    // 2分钟，最大等待时间
#define MIN_WAIT_TIME_MS 10000    // 10s，最小大等待时间

#define PRESSED_BIT 0x40        	//bit 6 indicating that key or touch was activated 1st time
#define RELEASED_BIT 0x80  			//bit 7 indicating that key was released

									//function exit parameters
#define NORMAL_EXIT 2				//normal controlled exit i.e. exit button
#define INACTIVITY_EXIT 2       	//exit after inactivity timeout
#define OCCL_ERROR 0
#define OCCL_ERROR_SEVERE   1  // 或者使用其他不冲突的值
#define OCCL_ERROR_MILD     0

#define HOLD_MODE__TASK 10			//identification numbers for main loop tasks
#define RUN_MODE__TASK 11
#define MCU_OFF__TASK 12
#define SET_DOSE_LIMIT__TASK 13
#define SET_WATER_RATE__TASK 14
#define CLEAR_RATES__TASK 15
#define SETTING_SCREEN__TASK 16
#define TEST_IF_SET_READY__TASK 17
#define PRIME_MODE__TASK 18
#define SERVICE_PAGE_ONE__TASK 19
#define SERVICE_PAGE_TWO__TASK 20
#define SERVICE_PAGE_THREE__TASK 21
#define SET_TIME__TASK 23
#define PRE_RUN_CHECK__TASK 24
#define OCCLUSION__ALARM 25
#define DOSE_COMPLETE_ALARM__TASK 26
#define REMOVE_SET__TASK 27

#define TUBING_DETECTED 1
#define TUBING_NOT_DETECTED 0
#define TUBING_EMPTY 1
#define TUBING_NOT_EMPTY 0

#define NO_TPA 0
#define SINGLE_TPA 1
#define DUAL_TPA 2
#define AT_MAGNET 20
#define ALL_CW 21
#define ALL_CCW 22
									//Error codes:
#define ERR_DAC_CAL_RD 120			//DAC calibration value out of limits, reading from eeprom
#define ERR_PD2_CAL_RD 121			//PD2 calibration value out of limits, reading from eeprom
#define ERR_CS_CAL_RD 122			//Calibration value checksum error, reading cal from eeprom
#define ERR_DAC_CAL_LIMIT 123		//DAC calibration value out of limits, during calibration
#define ERR_PD2_CAL_LIMIT 124		//PD2 calibration value out of limits, during calibration
#define ERR_TOUCH_CAL 125
#define ERR_NO_MAGNET 126			//Magnet not found during eccentric movement
#define ERR_ML_COUNT_TO 127     	//mL count incremented before previous count processed
#define ERR_EXCESS_BG_IR 128        //Background IR is too high for compensation to work
#define ERR_EMPTY_BAT 129           //Shut off because of empty battery
#define ERR_OTHER 130

#define CAL_OK 1					//Save successful calibration date to error log
#define CAL_ERROR 0

#define UNTIL_CANCEL 0
#define TEN_SECOND 10


#define SET_READY 1
#define SET_NOT_READY 0

#define INACTIVITY_TIMEOUT 3600		//3 minute inactivity(idle) timeout. timed by 50ms loop counter
//#define EMPTY_BATTERY_TIMEOUT 15    //empty battery screen timeout time /2
#define EMPTY_BATTERY_TIMEOUT 180      //3分钟 180

// ====== 新增配置 ======
#define OPTION_SCREEN_OFF_ONLY 0  // 1=只关屏幕不关机，0=原有逻辑（超时关机）

#define SET_TEST_DELAY 40           //test set every 2 second 40 x 0.050 = 2
#define EDIT_SCREENS_TIMEOUT 3600 	//at 50ms looptime 3600 x 0.05ms = 180 seconds (3min)
#define SET_NOT_READY_TIMEOUT 600   //at 50ms looptime 600 x 0.05ms = 30 seconds

#define TIMEOUT_ERROR        0x55


#define PTS_THRESHOLD  1500  		//2743

#define LOW_BATTERY_VOLTAGE 1240  //1050	//1150
#define EMPTY_BATTERY_VOLTAGE 1200//2068 //950 //1030彻底没电 1100差10秒没电
#define EMPTY_BATTERY_TEST_VOLTAGE 1180//2068 //950 //1030彻底没电 1100差10秒没电
#define LOW_BATT 1
#define EMPTY_BATTERY 0
#define BATTERY_OK  2
#define AC_RESTORED 3
#define AC_CONNECTED 4

#define ONE_SECOND 20					//one second count based on 50ms loop time
#define LOOP_COUNT 10					//50ms loop time with 5ms interupt
#define NO_TOT_VOL_DISPLAY 6            //Total volume display time in seconds

#define EEPROM_WRITE_DELAY 6            //M24256 max write time is 5ms

#define SETTINGS_START_ADDR 0           //start address of settings data (eeprom page 1)
#define ALARM_VOL 0						//8 bits
#define WARNING_VOL 1					//8 bits
#define DISPLAY_BRIGHNESS 2				//8 bits
#define DOSE_COMPLETED_CALLBACK 3		//8 bits
#define SVC_ENTRY_DELAY 4				//8 bits

#define VOLUME_START_ADDR 64 			//start address of rate and volume data (eeprom page 2)
#define RATE_NV_ADDR 64					//16 bits (little-endian,ie,lower bits in lower address)
#define VOLUME_NV_ADDR 66				//16 bits
#define TOTAL_VOLUME_NV_ADDR 68			//16 bits
#define DOSE_LIMIT_NV_ADDR 70			//16 bits
#define WATER_RATE_NV_ADDR 72			//16 bits


#define ACC_VOLUME_START_ADDR 128		//start address of accumulated volume data (eeprom page 3)
#define ACC_FORMULA_VOLUME_ADDR 128		//32 bits (little-endian,ie,lower bits in lower address)
#define ACC_BOLUS_VOLUME_ADDR 132		//32 bits (little-endian,ie,lower bits in lower address)
#define ACC_TIME_ON_BAT_ADDR 136		//32 bits



#define CALIBRATION_START_ADDR 192		//starting address of calibration data (eeprom page 4)
										//16 bits (little-endian,ie,lower half in lower address)
#define PD2_CALIBRATION_VALUE 192		//PD2 output after calibration 16bit
#define DAC_CALIBRATION_VALUE 194       //DAC value after calibration 16bit
#define CALIBRATION_CHECK_SUM 196       //sum of PD2 and DAC values 16bit

#define EVE_TRANSFORM_START_ADDR 256	//Starting address of touch screen calibration data (eeprom page 5)
#define EVE_TRANSFORM_REG_A 256         //EVE_REG_TOUCH_TRANSFORM_A 32 bit
#define EVE_TRANSFORM_REG_B 260         //EVE_REG_TOUCH_TRANSFORM_B 32 bit
#define EVE_TRANSFORM_REG_C 264         //EVE_REG_TOUCH_TRANSFORM_C 32 bit
#define EVE_TRANSFORM_REG_D 268         //EVE_REG_TOUCH_TRANSFORM_D 32 bit
#define EVE_TRANSFORM_REG_E 272         //EVE_REG_TOUCH_TRANSFORM_E 32 bit
#define EVE_TRANSFORM_REG_F 276         //EVE_REG_TOUCH_TRANSFORM_F 32 bit
#define EVE_TRANSFORM_CS 280            //EVE touch screen calibration checksum 32 bit

#define ERR_MSGS_START_ADDR 576			//starting address of error message data (eeprom page 10-110)
										//7 bytes per message. Use 8 to fit even 8 messages per page
										//3 pages required for 24 messages. Reserve 100 pages
										//for a possible expansion of 800 messages.
#define ERR_MSG_HEAD 576				//address of last error record
#define ERR_MSG_DATA_START 640			//error number data starts at page 11
#define ERR_MSG_MAX_NO_OF 90            //number of error messages stored (max 800)
#define ERR_SCRN_NO_OF_LINES 15			//15 records shown per screen.
//#define ERR_MSG_DATA_END 7040-1


#define DLVD_DAILY_VOL_START_ADDR 7296	//starting address of daily vol data (eeprom page 115-215)
									    //7 bytes per message. Use 8 to fit even 8 messages per page
										//3 pages required for 24 messages. Reserve 100 pages
										//for a possible expansion of 800 messages.
#define DLVD_VOL_HEAD 7296				//address of last vol record
#define DLVD_DAILY_DATA_START 7360		//data starts at page 116
#define DLVD_VOL_MAX_NO_OF 90          //number of error messages stored (max 800)
#define DLVD_VOL_NO_OF_LINES 15        //15 records shown per screen.
//#define DLVD_VOL_DATA_END 13760-1

#define CAL_REC_START_ADDR 14016    	//starting address of daily vol data (eeprom page 220-232)
									    //7 bytes per message. Use 8 to fit even 8 messages per page
										//3 pages required for 24 messages. Reserve 12 pages
										//for a possible expansion of 96 messages.
#define CAL_REC_HEAD 14016				//address of last cal record
#define CAL_REC_DATA_START 14080		//data starts at page 221
#define CAL_REC_MAX_NO_OF 90            //number of error messages stored (max 96)
#define CAL_REC_NO_OF_LINES 15           //15 records shown per screen.
//#define DLVD_VOL_DATA_END 14848-1

#define PD2_IR_CAL_TARGET 1000				//PD2 output target value for IR current calibration
#define PD2_IR_CAL_MIN PD2_IR_CAL_TARGET * 0.95 //950			        //Min allowed PD2 output for IR current calibration
#define PD2_IR_CAL_MAX PD2_IR_CAL_TARGET * 1.05//1050			    //Max allowed PD2 output for IR current calibration
#define DAC_IR_CAL_MIN 500					//Min allowed DAC value for IR current calibration
#define DAC_IR_CAL_MAX 3300 //2500				//Max allowed DAC value for IR current calibration

#define PD2_EXCESS_BG_IR_MAX 1500       //Maximum background IR level
#define PD2_NO_TUBING_MIN PD2_IR_CAL_TARGET * 0.8
#define PD2_NO_TUBING_MAX PD2_IR_CAL_TARGET * 2
//#define EMPTY_TUBING_MIN 2000
#define EMPTY_TUBING_MAX 2000
#define NO_TUBING_MIN 750

/**************************************
 * @brief 按键定义
 * @note 集中管理所有按键标签
 */
#define RIGHT_ARROW_PRESSED   (RIGHT_ARROW_TAG | PRESSED_BIT)
#define RIGHT_ARROW_RELEASED  (RIGHT_ARROW_TAG | RELEASED_BIT)
#define DOWN_ARROW_PRESSED    (DOWN_ARROW_TAG | PRESSED_BIT)
#define DOWN_ARROW_RELEASED   (DOWN_ARROW_TAG | RELEASED_BIT)
#define UP_ARROW_PRESSED      (UP_ARROW_TAG | PRESSED_BIT)
#define UP_ARROW_RELEASED     (UP_ARROW_TAG | RELEASED_BIT)

#define RUN_STOP_SWITCH_RELEASED    (RUN_STOP_SWITCH | RELEASED_BIT)
#define POWER_SWITCH_RELEASED       (POWER_SWITCH | RELEASED_BIT)
#define PRIME_KEYS_MASK       (PRIME_TAG | PRIME_TAG_WATER | PRIME_TAG_FOOD)

#define LOUD_TONE 1
#define NORMAL_TONE 0
#define NO 0
#define YES 1
#define UPWARD 2
#define DOWNWARD 1
#define NOT_DEF 0
#define PRIME_OK 1
#define PRIME_ERR 0
#define PRIME_EXIT 2

#define TUBE_REMOVED 0
#define TUBE_NOT_REMOVED 1


// ===== 旧电机参数 (7.5° 1:15减速) =====
#if (MOTOR_TYPE == 1)
#define NO_TPA_TH 150
#define SINGLE_TPA_TH 50
#define PINCHER_HALF_MOVE 250

#define FULL_ROTATION_STEPS 220

// ===== 新电机参数 (BH-20BYJ46 7.5° 1:85减速) =====
#elif (MOTOR_TYPE == 2)


    #define NO_TPA_TH           150     // 150 × 5.67
    #define SINGLE_TPA_TH     50    // 50  × 5.67
    #define PINCHER_HALF_MOVE   236    // 250 ×5.67

    #define FULL_ROTATION_STEPS 206
#else
    #error "MOTOR_TYPE must be 1 or 2"
#endif



//#define PRIME_TIME_BAG_TO_TPA 22      	//About 26 (0.6604m)inches 750mL/hr priming time for bag to TPA tubing
//#define PRIME_TIME_SENSOR_TO_END 57    	//About 55(1.397m) (end - 12) inches at 7500mL/hr
//#define PRIME_TIME_BAG_TO_SENSOR 29    	//About 34 (0.8636m)inches
//#define PRIME_TIME_TPA_TO_SENSOR 7     	//About 8 (0.2032m)inches
#define MAX_BUBBLE_TOLERANCE  3         // 允许的连续气泡数
#define MAX_OVERRUN_COUNT    3     // 总气泡容忍数（新增）
#define REQUIRED_LIQUID_COUNT     10     // 所需有效液体计数
#define PRIME_TIMED_BAG_TO_SENSOR  15
#define PRIME_TIME_FORMULA         40     // 第二阶段最短时间

#define PRIME_TIME_BAG_TO_TPA 22      	//About 26 (0.6604m)inches 750mL/hr priming time for bag to TPA tubing
#define PRIME_TIME_SENSOR_TO_END 84    	//About 50(1.197m) (end - 12) inches at 7500mL/hr
#define PRIME_TIME_BAG_TO_SENSOR 29    	//About 34 (0.8636m)inches
#define PRIME_TIME_TPA_TO_SENSOR 7     	//About 8 (0.2032m)inches

#define MOTOR_OFF_STEP_COUNT 5         	//No of steps motor gets powered after it has stopped
#define MOTOR_ON_STEP_COUNT 5		   	//No of steps motor gets powered after it has stopped

#if  USE_SPEED_INTERVAL == SPEED_MODE_FAST
#define FULL_FOOD_RATE	600
#else
#define FULL_FOOD_RATE	400				//Maximum formula rate mL/h
#endif
#define MAX_WATER_RATE 150				//Maximum bolus volume delivered per hour

#if  USE_SPEED_INTERVAL == SPEED_MODE_FAST
#define BOLUS_RATE 1000					//Bolus rate mL/h
#else
#define BOLUS_RATE 750					//Bolus rate mL/h
#endif

#define DOSE_COMP_CALL_BACK1 5
#define DOSE_COMP_CALL_BACK2 60
#define DOSE_COMP_CALL_BACK3 300
#define SVC_MODE_DELAY1 3
#define SVC_MODE_DELAY2 6
#define SVC_MODE_DELAY3 9

#define OCCL_TEST_START_COUNT 240       //initial start position for occlusion cycle
#define OCCL_TEST_HEAT_END 20
#define OCCL_TEST_END 140
#define OCCL_TEST_INTERVAL 480

#define OCCL_TEST_SAFE_MIN  80

#define OCCL_TEST_RATE 16
#define OCCL_COMP_LIMIT 2
#define OCCL_PEAK_LIMIT 2
#define OCCL_PEAK_DEF 60  				//Peak detection minimum for OCCL test

#define OCCL_DATA_SIZE         120

#define HEATING_DURATION_MS      (OCCL_TEST_HEAT_END * 1000)      // 20000ms
#define TOTAL_DETECTION_MS       (OCCL_TEST_END * 1000)           // 140000ms
#define FEEDING_DURATION_MS      (TOTAL_DETECTION_MS - HEATING_DURATION_MS) // 120000ms

#define FIRST_TEST_EXPECTED_COMPLETION      (240000 + TOTAL_DETECTION_MS + 20000)

#define DISABLE_8TH_TEST_CHECK
// 完全延迟优化相关宏定义
#define FULL_DELAY_STEP_MS              15000     // 完全延迟步长15秒
#define FULL_DELAY_SEARCH_RANGE         (6 * 60)  // 只搜索前6分钟
#define MAX_FULL_DELAY_LOOPS            24        // 6分钟 ÷ 15秒 = 24次循环

#define MIN_REMAINING_FOR_ACCELERATE    (4 * 60)  // 加速需要至少4分钟
#define MIN_REMAINING_FOR_DELAY         (4 * 60)  // 延迟需要至少4分钟
#define PARTIAL_DELAY_STEP_MS           15000     // 部分延迟步长15秒
#define MIN_REMAINING_AFTER_TEST        90        // 检测完成后最小剩余时间90秒
#define HIGH_FLOW_RATE_THRESHOLD        14        // 高流速阈值16mL/h,正常喂食和堵塞检测喂食的临界点
#define MAX_ACCELERATE_ERROR            5         // 加速后最大允许误差5%

#define DETECTION_COUNT                 7         // 最少7次堵塞检测

#define TOTAL_DETECTION_TIME_MS    (DETECTION_COUNT * TOTAL_DETECTION_MS)    // 总检测时间(ms)
#define NORMAL_TIME_MS             (3600000 - TOTAL_DETECTION_TIME_MS)          // 1小时内正常流动时间(ms)
#define DETECTION_VOLUME_PER_HOUR  ((OCCL_TEST_RATE * TOTAL_DETECTION_TIME_MS) / 3600000)  // 检测期间每小时流量(μl)

#define PAUSE_THRESHOLD_MS (30 * 60 * 1000)  // 30分钟阈值
#define MIN_VALID_INTERVAL_MS 1000           // 最小有效间隔(避免除零)

/**
 * @brief 喂水冲洗状态结构体  //20251230 增加jeffg
 */
typedef struct {
    uint8_t flushing_mode;        // 冲洗模式标志（本地状态）
    uint16_t flush_volume_ml;     // 冲洗体积（ml）
    uint8_t is_flushing;          // 冲洗进行中标志（全局状态）
    uint8_t flush_completed;      // 冲洗已完成标志
    uint8_t water_fed_this_session; // 本次喂食过程中是否有喂水
} water_flush_state_t;

/**
 * @brief 低速模式等待状态结构体
 * @note  用于管理低速模式下完成小时剂量后的等待状态
 */
typedef struct {
    uint8_t is_waiting;           /**< 是否正在等待 (1:等待中, 0:运行中) */
    uint32_t wait_start_time;     /**< 等待开始的时间戳 (ms) */
    uint32_t wait_duration;       /**< 需要等待的总时长 (ms) */
} LowFlowWaitState_t;

/**
 * @brief 正常模式等待状态结构体
 * @note  用于管理低速模式下完成小时剂量后的等待状态
 */

#ifdef ENABLE_NORMAL_FLOW_STARTUP_CHECK
typedef struct {
    uint8_t is_waiting;           /**< 是否正在等待 (1:等待中, 0:运行中) */
    uint32_t wait_start_time;     /**< 等待开始的时间戳 (ms) */
    uint32_t wait_duration;       /**< 需要等待的总时长 (ms) */
} NormalFlowWaitState_t;

#endif


// 动态间隔控制结构 20251104 jeffg
typedef struct {
    uint8_t current_hour_tests;      // 当前小时检测次数
    uint16_t normal_interval;        // 正常间隔(480秒)
    uint32_t extended_interval;      // 延长间隔  第一次堵塞补偿用
    uint16_t test_volume_ml;         // 单次检测流量(6=0.6mL)
    uint32_t hour_start_time;        // 小时开始时间
    uint16_t volume_at_hour_start;   // 小时开始体积
    uint8_t need_extend_interval;    // // 0=正常，1=延长间隔，2=低速模式

    uint8_t hour_dose_completed;  // 新增：小时剂量完成标志
    uint8_t low_flow_mode_active;    // 新增：低速模式活动标志 (0=非低速，1=低速模式)
    uint16_t feeding_start_volume;  // 喂养开始时的累计流量
    uint32_t feeding_start_time;    // 喂养开始时间
    // 暂停检测相关字段
    uint8_t has_pause_this_period;
    uint32_t last_enter_time;      // 上次进入任务的时间
    uint16_t flow_before_pause;    // 本次暂停前该小时已累计的流量（ml）
    uint16_t pause_flow_total;     // 本小时所有暂停前的累计流量总和
    uint32_t accumulated_work_time;   // 本小时累计有效工作时间（毫秒）
    uint8_t pause_timeout;             // 暂停超时标志（1=超过半小时）
    // 新增：喂水冲洗状态
    water_flush_state_t water_flush;

    // ====== 新增：低速模式等待状态 ======
    LowFlowWaitState_t low_flow_wait;

    uint8_t compensation_active;     // 补偿激活标志
    uint8_t compensation_ml;         //补偿流量
    uint8_t compensation_occl_time;  //堵塞补偿时间

} dynamic_interval_control_t;

    #define RTC_I2C_ADDR    0xa2
	// RTC 寄存器地址定义
	#define RTC_CTRL1_REG   0x00
	#define RTC_MIN_REG     0x05
	#define RTC_HOUR_REG    0x06
	#define RTC_DATE_REG    0x07
	#define RTC_MONTH_REG   0x09
	#define RTC_YEAR_REG    0x0A

    #define START_POINT         40
    #define END_POINT           117
    #define SLOPE_UNDEFINED     0
    #define SLOPE_UPWARD        1
    #define SLOPE_DOWNWARD      2

    // 常量定义
    #define WINDOW_SIZE       4       // 滑动平均窗口大小
	#define BASELINE_POINTS   6  // 用于计算基线的点数
    #define DATA_START        0       // 有效数据起始索引
    #define DATA_END          116     // 有效数据结束索引
    #define NORMALIZATION_MAX 600     // 归一化最大值
    #define INVALID_DATA      0xFF    // 无效数据返回值

	#ifndef MAX
	#define MAX(a, b) ((a) > (b) ? (a) : (b))
	#endif

// 使用状态机替代时间判断
typedef enum {
    OCCL_STATE_IDLE,
    OCCL_STATE_HEATING,
    OCCL_STATE_COLLECTING,
    OCCL_STATE_PROCESSING
} occl_state_t;
// 声明全局状态变量（extern 表示定义在其他文件）
extern occl_state_t g_occl_state;

// 声明状态获取函数
extern occl_state_t get_occl_state(void);

extern uint8_t g_first_after_bolus ;


typedef enum {
    PINCH_DIR_CW = 0,  // 顺时针（关闭食物管路）
    PINCH_DIR_CCW      // 逆时针（关闭推注管路）
} PinchDirection;


/* 运行模式返回状态码 */
#define CONTINUE_RUNNING       0  // 继续正常运行
#define OCCLUSION_ALARM       1  // 触发闭塞报警
#define REQUEST_BOLUS_SWITCH  2  // 请求切换到推注模式
#define AIR_DETECTED_ALARM    3  // 空气检测报警


/* 按键状态位 */

#define BOLUS_ARROW_TAG          0x04
#define RATE_TEXT_TAG            0x08

#if  USE_SPEED_INTERVAL == SPEED_MODE_FAST   // 1=启用20%加速，0=使用原间隔
//#define MOTOR_INT_BOLUS      192			//bolus run motor interrupt 1000mL/h
//#define MOTOR_INT_OCCL_TEST   472      	//OCCL test motor interrupt 400mL/h, rate 16 (16mL/h)
//#define MOTOR_INT_FORMULA     192      //formula motor interrupt 864mL/h
//#define ONE_ML 1800    //

#define MOTOR_INT_BOLUS      192			//bolus run motor interrupt 1000mL/h
#define MOTOR_INT_OCCL_TEST   472      	//OCCL test motor interrupt 400mL/h, rate 16 (16mL/h)
#define MOTOR_INT_FORMULA     222      //formula motor interrupt 864mL/h
#define ONE_ML 1800    //
#elif USE_SPEED_INTERVAL == SPEED_MODE_NORMAL
#define MOTOR_INT_BOLUS 257			//bolus run motor interrupt 750mL/h
#define MOTOR_INT_OCCL_TEST 482		//OCCL test motor interrupt 400mL/h, rate 16 (16mL/h)
#define BASE_MOTOR_INT_FORMULA 341       //formula motor interrupt 565mL/h

#define ONE_ML 1800   //1869 手工管
#else
#define MOTOR_INT_BOLUS 257			//bolus run motor interrupt 750mL/h
//#define MOTOR_INT_BOLUS 192			//bolus run motor interrupt 1000mL/h
//#define MOTOR_INT_BOLUS 154			//bolus run motor interrupt 1250mL/h
#define MOTOR_INT_OCCL_TEST 472		//OCCL test motor interrupt 400mL/h, rate 16 (16mL/h)
#define MOTOR_INT_FORMULA 355       //formula motor interrupt 565mL/h 358
#define ONE_ML 1800                 //9月底精度测试时是1800     4台测试是1833  351  462   1800 358 472
#endif
//T中断 = (160+1)×(341+1)/16,000,000 = 161×42/16,000,000  =3.44ms
//#define ONE_ML 300    //测试提前
/* 流量传感器校准结构体 */
#define SAFE_MOTOR_INT_FORMULA \
    ({ \
        int32_t _result = BASE_MOTOR_INT_FORMULA; \
        uint16_t _vol = running_volume;  /* 流量值 */ \
        uint16_t _rate = G_food_rate;    /* 进食速率 */ \
        \
        /* ========== 1. 速率修正系数 (默认100，表示不调整) ========== */ \
        int32_t _rate_coef = 100; \
        if (_rate > 200) { \
            _rate_coef = 97; \
        } else if (_rate >= 125 && _rate <= 200) { \
            _rate_coef = 98; \
        } else if (_rate >= 32 && _rate < 125) { \
            /* 注意：这个分支依赖流量 _vol */ \
            if (_vol < _rate * 2) { \
                _rate_coef = 99;  /* 不调整 */ \
            } else if (_vol > _rate * 2) { \
                _rate_coef = 98;   /* 微调 */ \
            } \
            /* _vol == _rate * 2 时不调整 */ \
        } \
        /* 其他情况（_rate < 32）不调整，_rate_coef 保持 100 */ \
        \
        /* ========== 2. 流量修正系数 (默认100) ========== */ \
        int32_t _vol_coef = 100; \
        if (_vol >= 800) { \
            _vol_coef = 100 - (_vol / 800); \
            /* 根据你的注释，vol最大8000，coef最小90，不会负数 */ \
            if (_vol_coef < 90) _vol_coef = 90; \
        } \
        /* _vol < 800 时不调整 */ \
        \
        /* ========== 3. 综合计算 ========== */ \
        _result = (_result * _rate_coef) / 100; \
        _result = (_result * _vol_coef) / 100; \
        \
        /* ========== 4. 限幅 ========== */ \
        if (_result < 290) _result = 290; \
        if (_result > BASE_MOTOR_INT_FORMULA + 14) _result = BASE_MOTOR_INT_FORMULA + 14; \
        \
        _result; \
    })
#define MAX_RUNNING_VOLUME  8000

// 食物速率调整宏（包含所有区间）
#define APPLY_FOOD_RATE(value) \
    ({ \
        uint32_t __val = value; \
        if (G_food_rate < 32) { \
            __val = ONE_ML; \
        } else if (G_food_rate <= 200) { \
            __val = __val * 101 / 100; \
        } else { \
            __val = __val * 102 / 100; \
        } \
        __val; \
    })

typedef struct {
    uint32_t one_ml_corrected;    // 修正后的值
    uint8_t valid_samples;        // 有效样本数
} time_based_flow_cal_t;


//Motor steps required to deliver one mL of formula or water
#define ONE_ML_AIR_COUNT 625 //1250 //2550           //Count required to detect one mL of air during the air/empty test
#define EMPTY_LIMIT 10000000
#define AIR_LIMIT 2500000
#define AIR_LIMIT_BOLUS 5000000
#define TEST_COUNT_PER_LOOP 2222
#define RUN_ML_COUNT_PER_LOOP 139

// 在配置头文件中添加
#define ENABLE_POST_FEED_FLUSH  1  // 1=启用喂食完成后冲洗，0=禁用


/*typedef struct {     //^^set as global in main.c for now
  uint16_t rate;
  uint16_t volume;
  uint16_t total_volume;
  uint16_t water_rate;
  uint16_t dose_limit;
} volume_struct;

typedef struct {
  uint8_t alarm_vol;
  uint8_t warning_vol;
  uint8_t display_brightness;
  uint8_t dose_complete_callback;
  uint8_t svc_entry_delay;
} setting_struct;*/

typedef struct {
  uint8_t minutes;
  uint8_t hours;
  uint8_t days;
  uint8_t months;
  uint8_t years;        //2 last digits only
  uint8_t period;		//0=am 1=pm
} chip_time;

typedef struct {
  uint8_t rev_d1;       //d1. .
  uint8_t rev_d2;       // .d2.
  uint8_t rev_d3;       // . .d3
} software_rev;

typedef struct {
  uint8_t err_num;
  uint8_t minutes;
  uint8_t hours;
  uint8_t days;
  uint8_t months;
  uint8_t years;        //2 last digits only
  uint8_t period;		//0=am 1=pm
} err_rec;

typedef struct {
  uint16_t food_vol;
  uint16_t water_vol;
  uint8_t days;
  uint8_t months;
  uint8_t years;        //2 last digits only
} vol_rec;

typedef struct {
  uint16_t dac_cal;
  uint16_t pd2_cal;
  uint8_t days;
  uint8_t months;
  uint8_t years;        //2 last digits only
} cal_rec;

typedef struct {
  uint16_t air_rate_inc;
  uint16_t air_cnt1;
  uint16_t air_cnt2;
  uint16_t air_cnt3;
  uint8_t air_ml_count;

} air_test_struct;
/*(*air_rate_inc) = (125 * (G_food_rate))/257;
 * 参数	数值	设计考虑
125	分子	临床经验系数，与管路截面积和超声灵敏度相关
257	分母	标准化因子，确保在典型输液速率(如50-100mL/hr)下增量值处于合理范围
G_food_rate	变量	系统实时监测的输液速率，反映当前流体动力学状态 */
/**
 @brief Key for identifying if touchscreen calibration values are programmed correctly.
 */
#define VALID_KEY_TOUCHSCREEN 0xd72f91a3

/**
 @brief Structure to hold touchscreen calibration settings.
 @details This is used to store the touchscreen calibration settings persistently
 in Flash and identify if the calibration needs to be re-performed.
 */
struct touchscreen_calibration {
	uint32_t key; // VALID_KEY_TOUCHSCREEN
	uint32_t transform[6];
};

#endif     /* COMMON_H  */
