/*****************************************************************************
* @file			: screens.c
* @short        :
******************************************************************************
*
* @details
*
******************************************************************************
*/

#include "screens.h"
#include "screen_helper.h"
#include "fonts_and_bitmaps.h"
#include <stdint.h>
#include <string.h>
#include "EVE.h"
#include "HAL.h"
#include "MCU.h"
#include <stm32l0xx_hal.h>
#include "common.h"
#include "io_calls.h"
#include <math.h>
#include "main_tasks.h"

extern I2C_HandleTypeDef hi2c3;
extern uint8_t G_tick5ms_counter;  //defined in main.c
extern uint16_t G_ir_pd2_nocomp;
extern uint16_t G_dac_cal_value;
extern uint16_t G_usonic_value, G_ir_pd2_value;
extern uint16_t G_tp_value;
extern uint16_t G_v_battery, G_vpts;
extern uint8_t G_an_on, G_tpa_type ;
extern volatile uint16_t G_motor_rate;
extern uint16_t G_food_rate,G_hour_volume;
extern uint16_t G_volume,G_water,running_volume;
extern uint16_t G_total_volume;
extern uint16_t G_water_rate;
extern uint16_t G_dose_limit;
extern uint8_t G_alarm_vol;
extern uint8_t G_warning_vol;
extern uint8_t G_display_brightness;
extern uint8_t G_dose_complete_callback_index;
extern uint8_t G_svc_entry_delay;

extern uint8_t occl_comp_count_test,occl_peak_count_test;
extern int16_t occl_comp_peak_offset;
extern volatile int16_t occl_comp_count_test_value[20];

extern volatile uint8_t occl_comp_count_test_position[20];
extern uint8_t G_controls_locked;
extern volatile time_based_flow_cal_t G_time_cal;
#if ENABLE_LOW_FLOW_MODE
extern uint16_t G_low_flow_mode;

#endif

static ScreenConfig default_config = {
    .bg_r = 0x00, .bg_g = 0x00, .bg_b = 0x00,    // 黑色
    .text_r = 0x00, .text_g = 0xFF, .text_b = 0x00, // 绿色
    .alarm_r = 0xFF, .alarm_g = 0xFF, .alarm_b = 0x00 // 黄色
};


static ErrorBellCache error_bell_cache = {0};
static ErrorWarningIconCache error_warning_cache = {0};

static void draw_speaker_icon(uint16_t center_x, uint16_t center_y, uint8_t is_muted);
static void update_error_bell_cache(uint16_t x, uint16_t y, uint8_t size);
static void initialize_error_bell_cache(uint16_t x, uint16_t y, uint8_t size);
static void draw_error_mute_bell(uint16_t x, uint16_t y, uint8_t size);

void pump_motor_off(void);
static inline void delay_50ms(void);
static void draw_arrow_button_touch(uint8_t tag, uint8_t handle, uint32_t bitmap_offset, uint16_t x, uint16_t y);
const EVE_GPU_FONT_HEADER *font0_hdr = (const EVE_GPU_FONT_HEADER *)font0;
const EVE_GPU_KEY_RIGHT_HEADER *keyright0_hdr = (const EVE_GPU_KEY_RIGHT_HEADER *)keyright0;
const EVE_GPU_KEY_UP_HEADER *keyup0_hdr = (const EVE_GPU_KEY_UP_HEADER *)keyup0;
const EVE_GPU_KEY_DOWN_HEADER *keydown0_hdr = (const EVE_GPU_KEY_DOWN_HEADER *)keydown0;
const uint32_t font0_offset = 0; // Taken from conversion settings in EVE Asset Builder, where the font is in EVE RAM_G
const uint32_t keyright0_offset = 0x3800L;
const uint32_t keyup0_offset = 0x3a00L;
const uint32_t keydown0_offset = 0x3c00L;
const uint32_t battery_100_bitmap_offset = 0x3e00L;  // 电池图标地址
const uint32_t plus_bitmap_offset = 0x4000L;  // 加号图标地址
const uint32_t minus_bitmap_offset = 0x4200L;  // 加号图标地址
const uint32_t rightarrow_bitmap_offset = 0x4400L;  // 加号图标地址
const uint32_t unlock_bitmap_offset = 0x4600L;  // 加号图标地址
const uint32_t battery_080_bitmap_offset = 0x4800L;  // 80% 电量
const uint32_t battery_060_bitmap_offset = 0x4a00L;  // 60% 电量
const uint32_t battery_040_bitmap_offset = 0x4c00L;  // 40% 电量
const uint32_t battery_020_bitmap_offset = 0x4e00L;  // 20% 电量
const uint32_t start_bitmap_offset = 0x6000L;  // 20% 电量
const uint32_t alarm1_bitmap_offset = 0x6200L;  //1级警报
const uint32_t bell_bitmap_offset = 0x6400L;   //铃声
const uint32_t mute_bitmap_offset = 0x6600L;   //静音
const uint32_t alarm3_bitmap_offset = 0x6800L;  //1级警报

static void initialize_error_warning_cache(uint16_t center_x, uint16_t center_y,
                                          uint16_t triangle_side, uint16_t triangle_height
                                          );

static void update_error_warning_cache(uint16_t center_x, uint16_t center_y,
                                      uint16_t triangle_side, uint16_t triangle_height
                                      );

void draw_cached_error_warning_icon(uint16_t center_x, uint16_t center_y,
                                   uint16_t triangle_side, uint16_t triangle_height,
                                   uint32_t color24);

void draw_warning_icon(uint16_t center_x, uint16_t center_y, uint16_t triangle_side, uint16_t triangle_height,uint32_t color24);
/**************************************
 * air_in_tubing_screen()
 * @detail
 * Display AIR IN LINE message until cancelled. Shut pump motor power off. Home pincher.
 */
void air_in_tubing_screen(void)
{
  const char AIR_IN_LINE[] = "AIR IN LINE";
  const char OR_EMPTY_BAG[] = "OR EMPTY BAG";

  pump_motor_off();
  move_pincher_all_ccw();
  g_pincher_result.position = POS_RIGHT;
  show_error_message(UNTIL_CANCEL,AIR_IN_LINE,OR_EMPTY_BAG);
}/**  End air_in_tubing_screen()  **/

/**************************************
 * empty_tubing_screen()
 * @detail
 * Display EMPTY FORMULA or EMPTY BOLUS message until cancelled.
 * Shut pump motor power off. Move pincher CCW.
 */
void empty_tubing_screen(uint8_t bolus_mode)
{
  const char EMPTY_FORMULA[] = "EMPTY FORMULA";
  const char EMPTY_WATER[] = "EMPTY WATER";

  pump_motor_off();
  move_pincher_all_ccw();
  g_pincher_result.position = POS_RIGHT;
  if (bolus_mode) show_error_message(UNTIL_CANCEL,EMPTY_WATER,"");
  else show_error_message(UNTIL_CANCEL,EMPTY_FORMULA,"");
}/**  End empty_tubing_screen()  **/

/**
  * @brief 读取触摸标签状态
  * @details 检测触摸状态变化，返回包含标签号和动作状态的组合值
  * @return uint8_t 返回值格式：
  *   - 位0-5: 标签编号(1-63)
  *   - 位6(PRESSED_BIT): 新按下标志
  *   - 位7(RELEASED_BIT): 释放标志
  *   - 0(NO_TOUCH): 无触摸
  */
uint8_t eve_read_tag(void)
{
  static uint8_t old_tag = NO_TOUCH;    // 保存上一次读取的标签状态
  uint8_t current_tag;                  // 当前读取的标签值
  uint8_t result_tag = NO_TOUCH;        // 要返回的结果值

  /* 从EVE寄存器读取当前触摸标签值 */
  current_tag = HAL_MemRead8(EVE_REG_TOUCH_TAG);

  /* 状态机处理四种可能情况 */
  if (current_tag == NO_TOUCH) {
    if (old_tag == NO_TOUCH) {
      // 情况1: 持续无触摸状态
      result_tag = NO_TOUCH;
    } else {
      // 情况2: 从有触摸变为无触摸(释放动作)
      result_tag = (old_tag & 0x3F) | RELEASED_BIT; // 保留原标签号，设置释放标志
    }
  } else {
    if (old_tag == NO_TOUCH) {
      // 情况3: 从无触摸变为有触摸(按下动作)
      result_tag = (current_tag & 0x3F) | PRESSED_BIT; // 保留标签号，设置按下标志
    } else {
      // 情况4: 持续触摸状态(仅返回标签号)
      result_tag = current_tag & 0x3F; // 只保留标签号部分
    }
  }

  /* 更新历史状态 */
  old_tag = current_tag;

  return result_tag;
}

/**************************************
 * @detail
 * read touch tag. Return tag info if touched.
 * returned parameter contains the tag number (bits 0 to 5) and additional
 * bits for new touch (bit 6) or release of touch (bit 7)
 */



/**************************************
 * @brief 剂量完成报警任务
 * @details
 * - 显示"DOSE COMPLETE"报警信息
 * - 按照dose_complete_callback_time定义的间隔发出双蜂鸣声
 * - 仅可通过触摸屏幕、RUN/STOP或PWR ON/OFF退出
 * - 注意：低电量关机功能未实现（标记为NOT IMPLEMENTED）
 **************************************/
uint8_t dose_complete_alarm_task()
{
  /* 常量定义 */
  const char DOSE_COMPLETE[] = "DOSE COMPLETE";
  const uint16_t dose_complete_callback_time[] = {5, 60, 300};

  /* 变量声明（完全保留原始命名） */
  uint16_t call_back_count, one_second_count;
  uint8_t batt_status, task_mode, key;

  (void)batt_status; // 明确标记未使用变量

  /* 初始化阶段 */
  task_mode = DOSE_COMPLETE_ALARM__TASK;
  pump_motor_off();
  ir_heat_off();

  // 首次进入强制触发双蜂鸣
  call_back_count = dose_complete_callback_time[G_dose_complete_callback_index] + 1;
  one_second_count = 0;

  /* 主任务循环 */
  while(task_mode == DOSE_COMPLETE_ALARM__TASK)
  {
    /* 1. 电源检测（保留未实现功能） */
    batt_status = power_test(); // 实际未使用但保留
#if USE_HOLD_MODE_NEW_SCREEN

    begin_screen(default_config);
    // ========== 简洁界面 ==========

    EVE_COLOR_RGB(40,40,40);
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);

         /* 电池符号显示 */
     draw_batt_symbol(G_v_battery);

       // 警告图标
     if (call_back_count % 2 == 0 ||
    	 call_back_count > dose_complete_callback_time[G_dose_complete_callback_index]) {
    	 EVE_COLOR_RGB(BLACK_COLOR);
     }else {
    	 EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
      }
     draw_alarm1(90, 20);

          // ====== 5. 显示错误文本（黑色常亮）=====
               //EVE_CMD_ROMFONT(1, 31);

          /* 显示逻辑（保持原有条件不变） */
      if (call_back_count % 2 == 0 ||
    	  call_back_count > dose_complete_callback_time[G_dose_complete_callback_index]) {
    	  EVE_COLOR_RGB(BLACK_COLOR);
       }else {
          EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
        }
      draw_text(240, 112, "DOSE", EVE_OPT_CENTERX, 1);
      draw_text(240, 172, "COMPLETE", EVE_OPT_CENTERX, 1);

#else
    /* 2. 图形界面渲染（保持原始EVE命令序列） */
    begin_screen(default_config);
    EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);

    /* 显示逻辑（保持原有条件不变） */
    if (call_back_count % 2 == 0 ||
        call_back_count > dose_complete_callback_time[G_dose_complete_callback_index]) {
      draw_text(240, 112, DOSE_COMPLETE, EVE_OPT_CENTERX, 1);
    }
    /* 蜂鸣期间的强制显示 */
#endif
    end_screen();

    /* 3. 双蜂鸣控制（保持原有逻辑） */
    if (call_back_count > dose_complete_callback_time[G_dose_complete_callback_index]) {
      beep_1khz_1sec(G_alarm_vol);
      HAL_Delay(1000);
      beep_1khz_1sec(G_alarm_vol);
      call_back_count = 0;
    }

    /* 4. 输入检测（完全保留原始实现） */

    key = read_keypad();
    if (key == (RUN_STOP_SWITCH_RELEASED)) task_mode = REMOVE_SET__TASK;
    if (key == (POWER_SWITCH_RELEASED)) task_mode = REMOVE_SET__TASK;

    /* 5. 时间计数（保持原始计时方式） */
    one_second_count++;
    if (one_second_count > 20) {
      one_second_count = 0;
      call_back_count++;
    }

    /* 6. 循环速率控制（保留50ms周期） */
    delay_50ms(); // 等待50ms

  }

  return task_mode;
}
/**************************************
 * dose_complete_alarm_task()
 * @detail
 * Display "DOSE COMPLETE" alarm. Double beep by time interval
 * defined by dose_complete_call_back_time.
 * exit by screen touch, RUN/STOP or PWR ON/OFF only.
 * Shut off if dead battery !!!!!!!!!!!!!!!!!!!!NOT IMPLEMENTED
 */


/** End dose_complete_alarm_task()  **/

/**************************************
 * occlusion_in_line_screen()
 * @detail
 * Display OCCLUSION IN LINE message until cancelled.
 * Shut pump motor power off. Move pincher CCW.
 */
void air_occlusion_in_line_screen(void)
{
  const char OCCLUSION_IN_LINE[] = "OCCLUSION IN LINE";
  const char OR_AIR_IN_LINE[] = "OR AIR IN LINE";

  pump_motor_off();
  move_pincher_all_ccw();
  g_pincher_result.position = POS_RIGHT;
  show_error_message(UNTIL_CANCEL,OCCLUSION_IN_LINE,OR_AIR_IN_LINE);


}/**  End occlusion_in_line_screen()  **/


/**************************************
 * occlusion_in_line_screen()
 * @detail
 * Display OCCLUSION IN LINE message until cancelled.
 * Shut pump motor power off. Move pincher CCW.
 */
void occlusion_in_line_screen(void)
{
  const char OCCLUSION_IN_LINE[] = "OCCLUSION IN LINE";

  pump_motor_off();
  move_pincher_all_ccw();
  g_pincher_result.position = POS_RIGHT;
  show_error_message(UNTIL_CANCEL,OCCLUSION_IN_LINE,"");


}/**  End occlusion_in_line_screen()  **/

/**************************************
 * @brief 运行模式界面显示函数
 * @detail 显示主运行界面，包括速率、体积值和操作按钮
 */
void run_mode_screen()
{
	begin_screen(default_config);                   // 清除颜色、模版和深度缓冲区

    /* 设置数字字体并显示速率和体积值 */
    EVE_CMD_SETFONT2(5, font0_offset, 48);   // 设置字体句柄5，从RAM_G的font0_offset位置加载，起始ASCII码48('0')
    EVE_TAG_MASK(1);                         // 启用标签掩码

    // 显示速率值（带标签RATE_TEXT_TAG）
    EVE_TAG(RATE_TEXT_TAG);
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);      // 设置文本颜色
    EVE_CMD_NUMBER(38, 25, 5, 3, G_food_rate); // 在(38,25)显示3位数字，右对齐

    // 显示体积值（带标签VOLUME_TEXT_TAG）
    EVE_TAG(VOLUME_TEXT_TAG);
    EVE_CMD_NUMBER(38, 155, 5, 4, G_volume);  // 在(38,155)显示4位数字，右对齐

    EVE_TAG_MASK(0);                         // 禁用标签掩码

    /* 显示标签文字（垂直排列） */
    EVE_CMD_TEXT(12, 19, 29, EVE_OPT_CENTERX, "R");  // 显示"RATE"标签
    EVE_CMD_TEXT(12, 44, 29, EVE_OPT_CENTERX, "A");
    EVE_CMD_TEXT(12, 69, 29, EVE_OPT_CENTERX, "T");
    EVE_CMD_TEXT(12, 94, 29, EVE_OPT_CENTERX, "E");

    EVE_CMD_TEXT(12, 162, 29, EVE_OPT_CENTERX, "V"); // 显示"VOL"标签
    EVE_CMD_TEXT(12, 187, 29, EVE_OPT_CENTERX, "O");
    EVE_CMD_TEXT(12, 212, 29, EVE_OPT_CENTERX, "L");

    /* 显示单位标签和分隔线 */
    EVE_CMD_TEXT(283, 52, 30, EVE_OPT_CENTERX, "ML");  // 显示"ML/HR"单位
    EVE_VERTEX_FORMAT(0);                     // 设置顶点格式
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);          // 开始绘制线条
    EVE_VERTEX2F(263,86);                     // 绘制水平分隔线
    EVE_VERTEX2F(303,86);
    EVE_CMD_TEXT(283, 86, 30, EVE_OPT_CENTERX, "HR");
    EVE_CMD_TEXT(353, 215, 30, EVE_OPT_CENTERX, "ML");

#ifdef TEST_MODE_volume

    EVE_CMD_TEXT(220, 265, 20, EVE_OPT_CENTER, "test_volume");
#endif

    draw_flow_indicator();                    // 绘制流量指示器
   // draw_flow_new_indicator();                    // 绘制流量指示器
    /* 绘制操作按钮（右、上、下箭头） */
    EVE_TAG_MASK(1);                         // 启用标签掩码
    draw_arrow_button(RIGHT_ARROW_TAG, 2, keyright0_offset, 410, 112, 0);
    // 右箭头按钮（带标签RIGHT_ARROW_TAG）
    draw_arrow_button(UP_ARROW_TAG, 3, keyup0_offset, 410, 30, 0);
    // 上箭头按钮（带标签UP_ARROW_TAG）
    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194, 0);
    // 下箭头按钮（带标签DOWN_ARROW_TAG）

    EVE_TAG_MASK(0);                         // 禁用标签掩码

    /* 新增：在锁定状态下显示小锁标志 */
    if (G_controls_locked) {
        // 在屏幕右上角显示锁图标
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);  // 红色锁图标，或使用其他醒目颜色

        // 方法1：使用字符（如果有锁字符）
        // EVE_CMD_TEXT(470, 10, 31, 0, "\x1A");  // 如果字体包含锁字符


        // 方法3：显示文字"LOCKED"
        EVE_CMD_TEXT(283, 30, 26, EVE_OPT_CENTERX, "LOCK");

        // 方法4：使用位图（如果有锁图标位图）
        // if (lock_icon_offset != 0) {
        //     EVE_BEGIN(EVE_BEGIN_BITMAPS);
        //     EVE_VERTEX2II(455, 15, 1, lock_icon_offset);
        // }
    }

   	draw_batt_symbol(G_v_battery);           // 绘制电池符号

    /******************************************************
     * 新增：低速模式剩余时间显示
     ******************************************************/
#if DISPLAY_LOW_FLOW_INFO
    // 通过getter获取
    dynamic_interval_control_t *interval_ctrl = get_interval_control();

    // 只在真正处于低速模式时显示
    if (G_low_flow_mode) {
        if (interval_ctrl->low_flow_wait.is_waiting) {
            // 计算剩余时间
            uint32_t current_ms = HAL_GetTick();
            uint32_t elapsed = current_ms - interval_ctrl->low_flow_wait.wait_start_time;
            uint32_t remaining_ms = 0;

            if (elapsed < interval_ctrl->low_flow_wait.wait_duration) {
                remaining_ms = interval_ctrl->low_flow_wait.wait_duration - elapsed;
            }

            // 转换为分钟和秒
            uint16_t remaining_minutes = remaining_ms / 60000;
       //     uint16_t remaining_seconds = (remaining_ms % 60000) / 1000;

            // 显示剩余时间框
            EVE_COLOR_RGB(0, 100, 200);  // 蓝色背景
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(50, 125);
            EVE_VERTEX2F(300, 150);

            // 显示剩余时间文字
            EVE_COLOR_RGB(255, 255, 255);  // 白色文字
            EVE_CMD_TEXT(80, 140, 28, EVE_OPT_CENTER, "WAIT");

            // 显示时间 mm:ss
            // 分钟数字
            EVE_CMD_ROMFONT(1, 26);
            EVE_CMD_NUMBER(250, 135, 1, 0, remaining_minutes);

            // 显示提示文字
            EVE_COLOR_RGB(200, 200, 200);  // 灰色文字
            EVE_CMD_TEXT(170, 140, 26, EVE_OPT_CENTER, "Low Flow ");
        }
        else{
            uint32_t current_ms = HAL_GetTick();
            uint32_t elapsed = current_ms - interval_ctrl->hour_start_time;
            // 转换为分钟和秒
            uint16_t elapsed_minutes = elapsed / 60000;
        	// 显示剩余时间框
            EVE_COLOR_RGB(0, 100, 200);  // 蓝色背景
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(50, 125);
            EVE_VERTEX2F(300, 150);

            // 显示剩余时间文字
            EVE_COLOR_RGB(255, 255, 255);  // 白色文字
            EVE_CMD_TEXT(80, 140, 28, EVE_OPT_CENTER, "Elapsed");

            // 显示时间 mm:ss
            // 分钟数字
            EVE_CMD_ROMFONT(1, 26);
            EVE_CMD_NUMBER(250, 135, 1, 0, elapsed_minutes);

            // 显示提示文字
            EVE_COLOR_RGB(200, 200, 200);  // 灰色文字
            EVE_CMD_TEXT(170, 140, 26, EVE_OPT_CENTER, "Low Flow ");
        }


    }
    else{
        uint32_t current_ms = HAL_GetTick();
        uint32_t elapsed;
        if(!interval_ctrl->has_pause_this_period){
        	elapsed = current_ms - interval_ctrl->hour_start_time;
        }
        else{
        	elapsed = current_ms - interval_ctrl->hour_start_time + interval_ctrl->accumulated_work_time;
        }
        // 转换为分钟和秒
        uint16_t elapsed_minutes = elapsed / 60000;
    	// 显示剩余时间框
        EVE_COLOR_RGB(0, 100, 200);  // 蓝色背景
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(50, 125);
        EVE_VERTEX2F(300, 150);

        // 显示剩余时间文字
        EVE_COLOR_RGB(255, 255, 255);  // 白色文字
        EVE_CMD_TEXT(80, 140, 28, EVE_OPT_CENTER, "Elapsed");

        // 显示时间 mm:ss
        // 分钟数字
        EVE_CMD_ROMFONT(1, 26);
        EVE_CMD_NUMBER(250, 135, 1, 0, elapsed_minutes);

        // 显示提示文字
        EVE_COLOR_RGB(200, 200, 200);  // 灰色文字
        EVE_CMD_TEXT(170, 140, 26, EVE_OPT_CENTER, "Normal Flow ");
    }
    // 如果不在低速模式，这里什么都不显示
#endif


    // 显示
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);  // 白色文字
    EVE_CMD_TEXT(40, 265, 26, EVE_OPT_CENTER, "DoseLim");

    EVE_CMD_TEXT(130, 265, 20, EVE_OPT_CENTER, "ML");

    EVE_CMD_ROMFONT(1, 26);
    EVE_CMD_NUMBER(80, 257, 1, 0, G_dose_limit);

    EVE_CMD_TEXT(340, 265, 20, EVE_OPT_CENTER, "W");

    EVE_CMD_ROMFONT(1, 26);
    EVE_CMD_NUMBER(380, 257, 1, 0, G_water);

    EVE_CMD_TEXT(430, 265, 20, EVE_OPT_CENTER, "ML");


    end_screen();
}


void run_mode_new_screen()
{
    begin_screen(default_config);

    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);


    EVE_TAG(RUN_LOCK_TAG);
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 64, 0, 0);
    EVE_VERTEX2II(480, 272, 0, 0);


    /* 数字显示设置 */
    EVE_CMD_SETFONT2(5, font0_offset, 48);  // 数字字体
    EVE_TAG_MASK(1);
    // 通过getter获取
    dynamic_interval_control_t *interval_ctrl = get_interval_control();
    // 计算剩余时间
    uint32_t current_ms = HAL_GetTick();
    uint32_t elapsed ;
    if(!interval_ctrl->has_pause_this_period){
    	elapsed = current_ms - interval_ctrl->hour_start_time;
    }
    else{
    	elapsed = current_ms - interval_ctrl->hour_start_time + interval_ctrl->accumulated_work_time;
    }
    // 转换为分钟和秒
    uint16_t elapsed_minutes = elapsed / 60000;

    // 定义静态变量
    static uint32_t first_entry_time = 0;
    static uint32_t total_accumulated = 0;

    // 第一次进入时记录
    if (first_entry_time == 0) {
        first_entry_time = HAL_GetTick();
    }

    // 计算累计时间
    total_accumulated = HAL_GetTick() - first_entry_time;

    // 打印或查看
    uint16_t total_minutes = total_accumulated / 60000;

    // 速率数值 - 位置根据标注 (151, 83之后的位置，实际数值在Y=151左右)
    EVE_TAG(RATE_TEXT_TAG);
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_NUMBER(141, 68, 5, 3, G_food_rate);  // X=151, Y=151

    EVE_TAG(VOLUME_TEXT_TAG);
    EVE_CMD_NUMBER(86, 175, 5, 4, G_volume); // X=285, Y=151
    EVE_TAG(NO_TAG);
    /* ========== RATE区域 ========== */
    // "RATE" 文字 - 使用英文字体 ROBOTO REGULAR 35PT
    EVE_CMD_TEXT(23, 74, 30, 0, "RATE");
    EVE_CMD_NUMBER(300, 74, 29, 0, G_time_cal.one_ml_corrected);
    EVE_CMD_NUMBER(300, 120, 29, 0, running_volume);


    // "mL/hr" 单位
    EVE_CMD_TEXT(23, 120, 29, 0, "mL/hr");

       /* ========== VOL区域 ========== */
    // "VOL" 文字
    EVE_CMD_TEXT(23, 180, 30, 0, "VOL");

    // "mL" 单位
    EVE_CMD_TEXT(23, 227, 29, 0, "mL");


    // 分隔线 "/"
    EVE_CMD_TEXT(287, 220, 30, 0, "/");

    // 目标体积值 (3000)
    EVE_CMD_NUMBER(300, 220, 30, 0, G_dose_limit);     // X=355, Y=151

    /* ========== 按钮区域 ========== */
    EVE_TAG_MASK(1);
    draw_flow_new_indicator();

//    draw_right_arrow_16x23(86, 20);
    EVE_COLOR_RGB(255, 255, 255);
    draw_unlock_icon(405, 134);
    EVE_TAG_MASK(0);
    // 显示剩余时间文字
    EVE_COLOR_RGB(255, 255, 255);  // 白色文字
    EVE_CMD_TEXT(400, 190, 28, 0, "Elapsed");

    // 显示时间 mm:ss
    // 分钟数字
    EVE_CMD_ROMFONT(1, 26);
    EVE_CMD_NUMBER(410, 227, 1, 0, elapsed_minutes);

    EVE_CMD_ROMFONT(1, 26);
    EVE_CMD_NUMBER(410, 78, 1, 0, total_minutes);

    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);

    /* 完成显示列表 */
    end_screen();
}
/**************************************
 * run_mode_screen()
 * @detail
 * Display RUN MODE screen
 */


/**  End run_mode_screen()  **/
/**************************************
 * @brief 显示超时消息函数
 * @param strline1 第一行消息文本
 * @param strline2 第二行消息文本（可为空）
 * @detail
 * - 显示两行文本消息（黑底白字）
 * - 屏幕闪烁并伴随1Hz蜂鸣声
 * - 可通过触摸屏幕、RUN/STOP或PWR按钮退出
 */
#if(1)
void show_timeout_message(const char *strline1, const char *strline2)
{
    uint16_t i;
    uint8_t batt_status;
    (void)batt_status;  // 明确标记未使用变量

    static uint16_t second_counter = 0;  // 秒计数器
    static uint8_t triple_beep_flag = 0;
    static uint8_t triple_beep_i_count = 0;
    static uint16_t   mute_duration_counter = 0;
    static uint8_t is_muted = 0;             // 0=正常报警，1=静音中


    while(1) {
        batt_status = power_test();  // 电源检测（保留但未使用）

        if (second_counter == 0 && !is_muted) {
            triple_beep_flag = 1;
            triple_beep_i_count = 0;
        }

        /* 闪烁显示周期（20*50ms=1秒） */
        for(i = 0; i < 20; i++) {
        	begin_screen(default_config);

            // ========== hold_mode_new_screen 的内容（直接移植）==========
            EVE_COLOR_RGB(255, 170, 0);  // 黄色

            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(0, 0, 0, 0);
            EVE_VERTEX2II(480, 60, 0, 0);

            EVE_COLOR_RGB(40,40, 40);  // 灰色
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(396, 64, 0, 0);
            EVE_VERTEX2II(480, 272, 0, 0);


            /* 数字显示设置 */
            EVE_CMD_SETFONT2(5, font0_offset, 48);  // 数字字体
            EVE_TAG_MASK(1);

            // 速率数值
            EVE_TAG(RATE_TEXT_TAG);
            EVE_COLOR_RGB(WHITE_COLOR);
            EVE_CMD_NUMBER(141, 68, 5, 3, G_food_rate);

            EVE_TAG(VOLUME_TEXT_TAG);
            EVE_CMD_NUMBER(86, 175, 5, 4, G_volume);
            EVE_TAG(NO_TAG);

            /* ========== RATE区域 ========== */
            EVE_CMD_TEXT(23, 74, 30, 0, "RATE");
            EVE_CMD_TEXT(23, 120, 29, 0, "mL/hr");

            /* ========== VOL区域 ========== */
            EVE_CMD_TEXT(23, 180, 30, 0, "VOL");
            EVE_CMD_TEXT(23, 227, 29, 0, "mL");

            // 分隔线 "/"
            EVE_CMD_TEXT(287, 220, 30, 0, "/");

            // 目标体积值 (3000)
            EVE_CMD_NUMBER(300, 220, 30, 0, G_dose_limit);

            /* ========== 按钮区域 ========== */
            EVE_TAG_MASK(1);
            EVE_TAG(BATTERY_MUTE_TAG);          // 设置标签 静音标识
            if(!is_muted){
            	draw_bell(430, 134);
            }
            else{
            	draw_mute(430, 134);
            }

            //draw_minus_icon(430, 134);
            //draw_right_arrow_22x34(430, 207);
            EVE_TAG_MASK(0);

            /* 电池符号显示 */
            draw_batt_symbol(G_v_battery);
            // ========== hold_mode_new_screen 内容结束 ==========

            /* ========== 倒计时显示（静音时） ========== */
            if (is_muted && mute_duration_counter > 0) {
            	uint16_t remain_sec = (2400 - mute_duration_counter) / 20;
                //EVE_CMD_SETFONT2(5, font0_offset, 24);  // 设置数字字体（如果还没设置）
                EVE_CMD_NUMBER(400, 200, 29, 3, remain_sec);  // 显示秒数，3位数（如 120, 099）
                EVE_CMD_TEXT(470, 200, 29, 0, "s");     // 添加单位 "s"
            }
            /* 只在闪烁周期的第一帧显示文本 */
            if (i < 10) {

                // 添加警告图标 - 在屏幕顶部中间
            	draw_alarm1(90, 20);

                EVE_COLOR_RGB(WHITE_COLOR);  // 设置报警文本颜色
               // EVE_CMD_ROMFONT(1,32);               // 使用ROM字体1，大小32

                if (strlen(strline2) == 0) {
                    // 单行文本居中显示
                    EVE_CMD_TEXT(136, 20, 29, 0, strline1);
                } else {
                    // 双行文本居中显示
                    EVE_CMD_TEXT(136, 20, 29, 0, strline1);
                    EVE_CMD_TEXT(136, 20, 29, 0, strline2);
                }
            }

            /* 完成显示列表 */
            end_screen();

            /* 检测退出条件 */
            if (read_keypad() == (RUN_STOP_SWITCH_RELEASED)) return;
            if (read_keypad() == (POWER_SWITCH_RELEASED)) return;
            /* ========== 触摸静音按钮处理 ========== */
            if (eve_read_tag() == BATTERY_MUTE_TAG) {
                key_click();  // 按键反馈音

                if (!is_muted) {
                    // 进入静音模式
                    is_muted = 1;
                    mute_duration_counter = 0;      // 从0开始计时
                    triple_beep_flag = 0;            // 停止当前报警
                    triple_beep_i_count = 0;
                } else {
                    // 退出静音模式（手动取消）
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;            // 恢复报警
                    triple_beep_i_count = 0;
                }

                // 等待触摸释放，避免重复触发
                while (eve_read_tag() == BATTERY_MUTE_TAG) {
                    delay_50ms();
                }
            }

            /* ========== 报警声和静音倒计时控制 ========== */
            if (is_muted) {
                // 静音模式：累加计时器
                mute_duration_counter++;

                // 2分钟 = 120秒 = 120 * 20 = 2400 个50ms周期
                if (mute_duration_counter >= 2400) {
                    // 自动退出静音模式
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;     // 重新开始报警
                    triple_beep_i_count = 0;

                    // 播放恢复提示音
                    beep_1khz_150ms(0);
                }
            } else {
                // 正常报警模式
                // 重置静音计时器（确保不在静音状态时归零）
                mute_duration_counter = 0;

                // 报警声逻辑
                if (triple_beep_flag) {
                    triple_beep_i_count++;

                    if (triple_beep_i_count == 1) {      // 第1声
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 8) { // 第2声（约350ms后）
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 16) { // 第3声（约350ms后）
                        beep_1khz_150ms(0);
                        triple_beep_flag = 0;   // 完成一组三连音
                    }
                }
            }

            /* 50ms延时控制 */
            delay_50ms(); // 等待50ms
            }
        /* 更新秒计数器 */
        second_counter++;
        if (second_counter >= 25) {  // 3分钟 = 180秒
            second_counter = 0;       // 重置计数器
        }
    }
}
#endif
/**************************************
 * show_timeout_message()
 * @detail
 * Display message text until screen touched or RUN/STOP
 * or PWR button pressed. Two lines of text can be displayed. Font size adjust to
 * the number of characters on line.
 * Black text on white back ground. Screen flashes on/off and alarm beeps at 1hz.
 */
#if(0)
void show_timeout_message(const char *strline1,const char *strline2)
{
  uint16_t i;
  uint8_t batt_status;
  (void)batt_status;
  while(1){
	batt_status=power_test();
    i=0;
	while( i++ <20){
	  EVE_LIB_BeginCoProList();
	  EVE_CMD_DLSTART();                       //set pointer to display list start
	  EVE_CLEAR_COLOR_RGB(HIGH_VIS_BG_COLOR);     //Screen background color black
	  EVE_CLEAR(1,1,1);

      EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
	  if (i==1){
	  	EVE_CMD_ROMFONT(1,32);
	    if (strlen(strline2) == 0){
	      EVE_CMD_TEXT(240, 112, 1, EVE_OPT_CENTERX, strline1);
	   	}
	    else{
	      EVE_CMD_TEXT(240, 82, 1, EVE_OPT_CENTERX, strline1);
	      EVE_CMD_TEXT(240, 142, 1, EVE_OPT_CENTERX, strline2);
      	}
	  }
	  else{
      }

	  EVE_DISPLAY();
	  EVE_CMD_SWAP();
	  EVE_LIB_EndCoProList();
	  EVE_LIB_AwaitCoProEmpty();
      if (i==1){
    	beep_1khz_1sec(G_alarm_vol);
      }
	  if (screen_touched()) {
		key_click();
		while (screen_touched());
		return;
	  }
	  if (read_keypad() == (RUN_STOP_SWITCH | RELEASED_BIT)) return;
	  if (read_keypad() == (POWER_SWITCH | RELEASED_BIT)) return;

	  while (G_tick5ms_counter<10);    //50ms loop
	  G_tick5ms_counter = 0;
	}
  }
  return;
}
#endif/**  End show_timeout_message()  **/
/**************************************
 * @brief 加载图形资源到RAM_G函数
 * @detail 将自定义字体和位图加载到图形控制器内存
 */
void load_RAMG(void)
{
    /* 加载字体和位图数据到RAM_G */
    EVE_LIB_WriteDataToRAMG(font0, sizeof(font0), font0_offset);       // 加载数字字体
    EVE_LIB_WriteDataToRAMG(keyright0, sizeof(keyright0), keyright0_offset);  // 加载右箭头位图
    EVE_LIB_WriteDataToRAMG(keyup0, sizeof(keyup0), keyup0_offset);    // 加载上箭头位图
    EVE_LIB_WriteDataToRAMG(keydown0, sizeof(keydown0), keydown0_offset); // 加载下箭头位图
#if USE_HOLD_MODE_NEW_SCREEN
    /* 加载电池图标位图 */
    EVE_LIB_WriteDataToRAMG(batt_100_bitmap, sizeof(batt_100_bitmap), battery_100_bitmap_offset);  // 加载电池图标
    EVE_LIB_WriteDataToRAMG(batt_080_bitmap, sizeof(batt_080_bitmap), battery_080_bitmap_offset);  // 加载电池图标
    EVE_LIB_WriteDataToRAMG(batt_060_bitmap, sizeof(batt_060_bitmap), battery_060_bitmap_offset);  // 加载电池图标
    EVE_LIB_WriteDataToRAMG(batt_040_bitmap, sizeof(batt_040_bitmap), battery_040_bitmap_offset);  // 加载电池图标
    EVE_LIB_WriteDataToRAMG(batt_020_bitmap, sizeof(batt_020_bitmap), battery_020_bitmap_offset);  // 加载电池图标
    EVE_LIB_WriteDataToRAMG(plus_bitmap, sizeof(plus_bitmap), plus_bitmap_offset);  // 加载加号图标
    EVE_LIB_WriteDataToRAMG(minus_bitmap, sizeof(minus_bitmap), minus_bitmap_offset);  // 加载减号电池图标
    EVE_LIB_WriteDataToRAMG(rightarrow_bitmap, sizeof(rightarrow_bitmap), rightarrow_bitmap_offset);  // 加载右箭头电池图标
    EVE_LIB_WriteDataToRAMG(unlock_bitmap, sizeof(unlock_bitmap), unlock_bitmap_offset);// 加载未上锁右箭头电池图标
    EVE_LIB_WriteDataToRAMG(alarm1_bitmap, sizeof(alarm1_bitmap), alarm1_bitmap_offset);// 加载警报1图标
    EVE_LIB_WriteDataToRAMG(alarm3_bitmap, sizeof(alarm3_bitmap), alarm3_bitmap_offset);// 加载警报1图标
    EVE_LIB_WriteDataToRAMG(bell_bitmap, sizeof(bell_bitmap), bell_bitmap_offset);// 加载铃声图标
    EVE_LIB_WriteDataToRAMG(mute_bitmap, sizeof(mute_bitmap), mute_bitmap_offset);// 加载静音图标

#endif
}

/**************************************
 * load_RAMG()
 * @detail
 * Load custom  (large) font and bitmaps to graphics controller memory
 */
#if(0)
void load_RAMG(void){
	uint32_t font0_size = sizeof(font0);
	uint32_t keyright0_size = sizeof(keyright0);
	uint32_t keyup0_size = sizeof(keyup0);
	uint32_t keydown0_size = sizeof(keydown0);
	EVE_LIB_WriteDataToRAMG(font0, font0_size, font0_offset);
	EVE_LIB_WriteDataToRAMG(keyright0, keyright0_size, keyright0_offset);
	EVE_LIB_WriteDataToRAMG(keyup0, keyup0_size, keyup0_offset);
	EVE_LIB_WriteDataToRAMG(keydown0, keydown0_size, keydown0_offset);
}/**  End load_RAMG()  **/
#endif

/**
  * @brief 保持模式界面显示函数
  * @param rate_blinking_flag 速率值闪烁标志
  * @detail
  * - 显示与运行模式相似的界面，但支持速率值闪烁
  * - 严格保留原始触摸区域设置（不进行透明矩形优化）
  * - 包含箭头按钮和电池符号显示
  */
void hold_mode_screen(uint8_t rate_blinking_flag)
{
	begin_screen(default_config);

    /* 数字显示设置 */
    EVE_CMD_SETFONT2(5, font0_offset, 48);
    EVE_TAG_MASK(1);

    /* 速率值显示（受闪烁标志控制） */
    EVE_TAG(RATE_TEXT_TAG);
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    if (!rate_blinking_flag) {
        EVE_CMD_NUMBER(38, 25, 5, 3, G_food_rate);
    }

    /* 体积值显示 */
    EVE_TAG(VOLUME_TEXT_TAG);
    EVE_CMD_NUMBER(38, 155, 5, 4, G_volume);
    EVE_TAG(NO_TAG);

#if(0)
    // 显示体积值（带标签VOLUME_TEXT_TAG）- 改用 ROM 字体显示
    EVE_TAG(VOLUME_TEXT_TAG);
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_ROMFONT(1, 34);                  // 设置 ROM 字体（字体1，点阵大小34）
    EVE_CMD_NUMBER(38, 155, 1, 4, G_volume);  // 使用字体句柄1显示4位数字
#endif
    /* 标签文字显示（保持原始垂直布局） */
    EVE_CMD_TEXT(12, 19, 29, EVE_OPT_CENTERX, "R");
    EVE_CMD_TEXT(12, 44, 29, EVE_OPT_CENTERX, "A");
    EVE_CMD_TEXT(12, 69, 29, EVE_OPT_CENTERX, "T");
    EVE_CMD_TEXT(12, 94, 29, EVE_OPT_CENTERX, "E");

    EVE_CMD_TEXT(12, 162, 29, EVE_OPT_CENTERX, "V");
    EVE_CMD_TEXT(12, 187, 29, EVE_OPT_CENTERX, "O");
    EVE_CMD_TEXT(12, 212, 29, EVE_OPT_CENTERX, "L");

    /* 单位标签和分隔线 */
    EVE_CMD_TEXT(283, 52, 30, EVE_OPT_CENTERX, "ML");
    EVE_VERTEX_FORMAT(0);
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_VERTEX2F(263,86);
    EVE_VERTEX2F(303,86);
    EVE_CMD_TEXT(283, 86, 30, EVE_OPT_CENTERX, "HR");
    EVE_CMD_TEXT(353, 215, 30, EVE_OPT_CENTERX, "ML");

    /* 按钮绘制（严格保持原始触摸区域） */
    EVE_TAG_MASK(1);
    draw_arrow_button(RIGHT_ARROW_TAG, 2, keyright0_offset, 410, 112, 0);
    // 右箭头按钮（带标签RIGHT_ARROW_TAG）
    draw_arrow_button(UP_ARROW_TAG, 3, keyup0_offset, 410, 30, 0);
    // 上箭头按钮（带标签UP_ARROW_TAG）
    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194, 0);
    // 下箭头按钮（带标签DOWN_ARROW_TAG）

    EVE_TAG_MASK(0);

    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);

    /* 完成显示列表 */
    end_screen();
}


void hold_mode_new_screen(uint8_t rate_blinking_flag)
{
    begin_screen(default_config);

    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);


    /* 数字显示设置 */
    EVE_CMD_SETFONT2(5, font0_offset, 48);  // 数字字体
    EVE_TAG_MASK(1);

    // 速率数值 - 位置根据标注 (151, 83之后的位置，实际数值在Y=151左右)
    EVE_TAG(RATE_TEXT_TAG);
    EVE_COLOR_RGB(WHITE_COLOR);
    if (!rate_blinking_flag) {
        EVE_CMD_NUMBER(141, 68, 5, 3, G_food_rate);  // X=151, Y=151
    }

    EVE_TAG(VOLUME_TEXT_TAG);
    EVE_CMD_NUMBER(86, 175, 5, 4, G_volume); // X=285, Y=151
    EVE_TAG(NO_TAG);
    /* ========== RATE区域 ========== */
    // "RATE" 文字 - 使用英文字体 ROBOTO REGULAR 35PT
    EVE_CMD_TEXT(23, 74, 30, 0, "RATE");

    // "mL/hr" 单位
    EVE_CMD_TEXT(23, 120, 29, 0, "mL/hr");
    /* ========== VOL区域 ========== */
    // "VOL" 文字
    EVE_CMD_TEXT(23, 180, 30, 0, "VOL");

    // "mL" 单位
    EVE_CMD_TEXT(23, 227, 29, 0, "mL");


    // 分隔线 "/"
    EVE_CMD_TEXT(287, 220, 30, 0, "/");

    // 目标体积值 (3000)
    EVE_CMD_NUMBER(300, 220, 30, 0, G_dose_limit);     // X=355, Y=151

    /* ========== 按钮区域 ========== */
    EVE_TAG_MASK(1);

    EVE_TAG(UP_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 64, 0, 0);
    EVE_VERTEX2II(480, 114, 0, 0);
    EVE_COLOR_RGB(WHITE_COLOR);
    draw_plus_icon(430, 77);

    EVE_TAG(DOWN_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 118, 0, 0);
    EVE_VERTEX2II(480, 169, 0, 0);
    EVE_COLOR_RGB(WHITE_COLOR);
    draw_minus_icon(430, 134);
 //   draw_rightarrow_icon(430, 207);
    EVE_TAG(RIGHT_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 173, 0, 0);
    EVE_VERTEX2II(480, 272, 0, 0);

    draw_right_arrow_22x34(430, 207);
  //  EVE_CMD_TEXT(440, 217, 1, EVE_OPT_CENTER, ">");
  //  draw_battery_simple(25,20,90,0);

    EVE_TAG_MASK(0);

    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);

    /* 完成显示列表 */
    end_screen();
}
/**************************************
 * hold_mode_screen()
 * @detail
 * Display HOLD MODE screen
 */
#if(0)
void hold_mode_screen(uint8_t rate_blinking_flag)
{
  EVE_LIB_BeginCoProList();
  EVE_CMD_DLSTART();                       //set pointer to display list start
  EVE_CLEAR_COLOR_RGB(HIGH_VIS_BG_COLOR);     //Screen background color black
  EVE_CLEAR(1,1,1);

  EVE_CMD_SETFONT2(5, font0_offset, 48);	 //Font to handle 5, font resides at RAM_G + 0, first char is ASCII 48 which is number '0'
  EVE_TAG_MASK(1);
  EVE_TAG(RATE_TEXT_TAG);
  EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
  if (!rate_blinking_flag){
    EVE_CMD_NUMBER(38, 25,	5,  3, G_food_rate);	// Print the counter with font 5, right justified, with 5 digits (leading 0s will be added)
  }
  EVE_TAG(VOLUME_TEXT_TAG);

  EVE_CMD_NUMBER(38, 155,	5,  4, G_volume);

  //EVE_TAG_MASK(0);
  EVE_TAG(NO_TAG);
  	//	EVE_CMD_ROMFONT(1,34);
  	//	EVE_CMD_TEXT(120, 120, 1, 0, "R123");

  EVE_CMD_TEXT(12, 19, 29, EVE_OPT_CENTERX, "R");
  EVE_CMD_TEXT(12, 44, 29, EVE_OPT_CENTERX, "A");
  EVE_CMD_TEXT(12, 69, 29, EVE_OPT_CENTERX, "T");
  EVE_CMD_TEXT(12, 94, 29, EVE_OPT_CENTERX, "E");

  EVE_CMD_TEXT(12, 162, 29, EVE_OPT_CENTERX, "V");
  EVE_CMD_TEXT(12, 187, 29, EVE_OPT_CENTERX, "O");
  EVE_CMD_TEXT(12, 212, 29, EVE_OPT_CENTERX, "L");

  EVE_CMD_TEXT(283, 52, 30, EVE_OPT_CENTERX, "ML");
  EVE_VERTEX_FORMAT(0);
  EVE_BEGIN(EVE_BEGIN_LINE_STRIP);          // DRAW LINES
  EVE_VERTEX2F(263,86);  // Draw horizontal line between "ML" and "HR"
  EVE_VERTEX2F(303,86);
  EVE_CMD_TEXT(283, 86, 30, EVE_OPT_CENTERX, "HR");
  EVE_CMD_TEXT(353, 215, 30, EVE_OPT_CENTERX, "ML");

  EVE_TAG_MASK(1);

  EVE_TAG(RIGHT_ARROW_TAG);						//Assign touch tag for right arrow//display right arrow
  EVE_BITMAP_HANDLE(2);							//assign handle for the bitmap
  EVE_BITMAP_SOURCE(keyright0_offset);			//bitmap data offset
  EVE_BITMAP_LAYOUT(1,6,48);
  EVE_BITMAP_SIZE(0,0,0,48,48);
  EVE_BEGIN(EVE_BEGIN_BITMAPS);
  EVE_VERTEX2II(410,112,2,0);
  EVE_COLOR_MASK(0,0,0,0);                      //increase effective touch area
  EVE_BEGIN(EVE_BEGIN_RECTS);                  	//by drawing an invisible rectangle
  EVE_VERTEX2F(390,100);     					//around the visible target.
  EVE_VERTEX2F(478,172);     					//
  EVE_COLOR_MASK(1,1,1,1);

  EVE_TAG(UP_ARROW_TAG);
  EVE_BITMAP_HANDLE(3);							//display up arrow
  EVE_BITMAP_SOURCE(keyup0_offset);
  EVE_BITMAP_LAYOUT(1,6,48);
  EVE_BITMAP_SIZE(0,0,0,48,48);
  EVE_BEGIN(EVE_BEGIN_BITMAPS);
  EVE_VERTEX2II(410,30,3,0);
  EVE_COLOR_MASK(0,0,0,0);                      //increase effective touch area
  EVE_BEGIN(EVE_BEGIN_RECTS);                   //by drawing an invisible rectangle
  EVE_VERTEX2F(390,18);                         //around the visible target.
  EVE_VERTEX2F(478,90);                         //
  EVE_COLOR_MASK(1,1,1,1);

  EVE_TAG(DOWN_ARROW_TAG);
  EVE_BITMAP_HANDLE(4);							//display down arrow
  EVE_BITMAP_SOURCE(keydown0_offset);
  EVE_BITMAP_LAYOUT(1,6,48);
  EVE_BITMAP_SIZE(0,0,0,48,48);
  EVE_BEGIN(EVE_BEGIN_BITMAPS);
  EVE_VERTEX2II(410,194,4,0);
  EVE_COLOR_MASK(0,0,0,0);                      //increase effective touch area
  EVE_BEGIN(EVE_BEGIN_RECTS);                   //by drawing an invisible rectangle
  EVE_VERTEX2F(390,182);                        //around the visible target.
  EVE_VERTEX2F(478,254);                        //
  EVE_COLOR_MASK(1,1,1,1);

  EVE_TAG_MASK(0);
  draw_batt_symbol(G_v_battery);

  EVE_DISPLAY();
  EVE_CMD_SWAP();
  EVE_LIB_EndCoProList();
  EVE_LIB_AwaitCoProEmpty();
}
#endif
/**************************************
 * @brief 服务页面1显示
 * @param cam_position 摄像头位置状态
 * @detail 功能：
 * - 水剂选择按钮
 * - 配方选择按钮
 * - 泵操作按钮
 * - 显示PTS传感器值
 * - 显示超声波和PD2传感器值
 * - 下箭头进入服务页面2
 * - 右箭头返回保持模式
 */
void service_page_one_screen(uint8_t cam_position)
{
	begin_screen(default_config);
    /* 页面标题和操作指引 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(10, 10, 29, 0, "SERVICE page 1");          // 保持原英文
    EVE_CMD_TEXT(10, 55, 27, 0, "PRESS TO SELECT WATER");   // 保持原英文
    EVE_CMD_TEXT(10, 98, 27, 0, "PRESS TO SELECT FORMULA"); // 保持原英文
    EVE_CMD_TEXT(10, 141, 27, 0, "PRESS AND HOLD TO PUMP"); // 保持原英文

    /* 传感器数值显示区域 */
    EVE_CMD_TEXT(10, 181, 27, 0, "PTS SENSOR");
    EVE_CMD_NUMBER(300, 181, 27, EVE_OPT_CENTERX, G_vpts);

    EVE_CMD_TEXT(10, 204, 27, 0, "TUBING ULTRA SONIC");
    EVE_CMD_NUMBER(300, 204, 27, EVE_OPT_CENTERX, G_usonic_value);

    EVE_CMD_TEXT(10, 227, 27, 0, "TUBING IR SENSOR PD2");
    EVE_CMD_NUMBER(300, 227, 27, EVE_OPT_CENTERX, G_ir_pd2_value);

    EVE_CMD_TEXT(10, 250, 27, 0, "BATTERY VOLTAGE [mV]");
    EVE_CMD_NUMBER(300, 250, 27, EVE_OPT_CENTERX, G_v_battery);

    /* 导航按钮（右箭头/下箭头）*/
    EVE_TAG_MASK(1);  // 启用标签检测
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);
    // 右箭头按钮（返回保持模式）

    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194, 0);
    // 下箭头按钮（进入页面2）

    /* 功能按钮（水剂/配方/泵）*/
    EVE_BEGIN(EVE_BEGIN_POINTS);  // 开始绘制点状按钮

    // 水剂选择按钮（根据摄像头位置调整大小）
    if (cam_position != CAM_LEFT) {
        EVE_POINT_SIZE(250);  // 默认大小
    } else {
        EVE_POINT_SIZE(300);  // 选中状态放大
    }
    EVE_TAG(WATER_TAG);
    EVE_VERTEX2II(300,65,2,0);  // 指定按钮位置

    // 配方选择按钮
    if (cam_position != CAM_RIGHT) {
        EVE_POINT_SIZE(250);
    } else {
        EVE_POINT_SIZE(300);  // 选中状态放大
    }
    EVE_TAG(FORMULA_TAG);
    EVE_VERTEX2II(300,108,2,0);

    // 泵操作按钮（固定大小）
    EVE_POINT_SIZE(250);
    EVE_TAG(PUMP_TAG);
    EVE_VERTEX2II(300,151,2,0);

    /* 完成显示列表并交换缓冲区 */
    end_screen();
}



/**  End service_page_one_screen()  **/
/**************************************
 * @brief 服务页面2显示函数
 * @param total_formula_delivered 总配方输送量
 * @param total_bolus_delivered 总水剂输送量
 * @param total_hours_battery 电池供电小时数
 * @param alrm_mon_value 报警监控值
 * @detail 功能：
 * - 压电报警测试按钮
 * - 显示输送统计数据
 * - 日志查看按钮
 * - 上下箭头切换页面
 */
void service_page_two_screen(uint16_t total_formula_delivered,
                           uint16_t total_bolus_delivered,
                           uint16_t total_hours_battery,
                           uint16_t alrm_mon_value)
{
	begin_screen(default_config);

    /* 页面标题和数据显示（保留原始英文） */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(10, 10, 29, 0, "SERVICE page 2");
    EVE_CMD_TEXT(10, 50, 27, 0, "CHECK PIEZO ALARM");

    /* 输送统计数据 */
    EVE_CMD_TEXT(10, 90, 27, 0, "TOTAL FORMULA DELIVERED");
    EVE_CMD_NUMBER(300, 90, 27, EVE_OPT_CENTERX, total_formula_delivered);

    EVE_CMD_TEXT(10, 115, 27, 0, "TOTAL BOLUS DELIVERED");
    EVE_CMD_NUMBER(300, 115, 27, EVE_OPT_CENTERX, total_bolus_delivered);

    EVE_CMD_TEXT(10, 140, 27, 0, "HOURS ON BATTERY POWER");
    EVE_CMD_NUMBER(300, 140, 27, EVE_OPT_CENTERX, total_hours_battery);

    EVE_CMD_TEXT(10, 170, 27, 0, "ALARM MONITOR");
    EVE_CMD_NUMBER(300, 170, 27, EVE_OPT_CENTERX, alrm_mon_value);

    /* 导航按钮设置 */
    EVE_TAG_MASK(1);

    // 上箭头按钮（返回页面1）
    draw_arrow_button_ex(UP_ARROW_TAG, 3, keyup0_offset,410,30, 0, HIGH_VIS_TEXT_COLOR);

    // 下箭头按钮（进入页面3）
    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194, 0);



    /* 功能按钮区域 */
    EVE_BEGIN(EVE_BEGIN_POINTS);

    // 压电报警测试按钮
    EVE_POINT_SIZE(250);
    EVE_TAG(PIEZO_TAG);
    EVE_VERTEX2II(300,65,2,0);

    /* 日志查看按钮 */
    EVE_CMD_FGCOLOR(ROYAL_BLUE_COLOR_24bit);
    EVE_COLOR_RGB(WHITE_COLOR);

    EVE_TAG(DAY_LOG_TAG);
    EVE_CMD_BUTTON(10,210,100,40,28,0,"DAY LOG");

    EVE_TAG(ERR_LOG_TAG);
    EVE_CMD_BUTTON(130,210,100,40,28,0,"ERR LOG");

    EVE_TAG(CAL_LOG_TAG);
    EVE_CMD_BUTTON(250,210,100,40,28,0,"CAL LOG");

    /* 完成显示列表 */
    end_screen();
}

/**************************************
 * @brief 服务页面3显示函数
 * @param pd2 PD2传感器值
 * @param tp_heat 加热温度值
 * @detail 功能：
 * - 显示传感器原始/补偿值
 * - 显示校准数据
 * - 校准按钮
 * - 20秒加热按钮
 * - 上下箭头切换页面
 */
void service_page_three_screen(uint16_t pd2, uint16_t tp_heat)
{
	begin_screen(default_config);

    /* 页面标题和实时数据（保留原始英文） */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(10, 10, 29, 0, "SERVICE page 3");
    EVE_CMD_TEXT(10, 55, 27, 0, "LIFE DATA:");
    EVE_CMD_TEXT(190, 55, 27, EVE_OPT_CENTERX, "ULTRASONIC");
    EVE_CMD_NUMBER(188, 80, 27, 0, G_usonic_value);

    /* PD2传感器数据显示 */
    EVE_CMD_TEXT(290, 55, 27, EVE_OPT_CENTERX, "PD2");
    EVE_CMD_NUMBER(246, 80, 27, 0, G_ir_pd2_value);
    EVE_CMD_NUMBER(331, 80, 27, EVE_OPT_RIGHTX, G_ir_pd2_nocomp - G_ir_pd2_value);

    /* 校准数据显示 */
    EVE_CMD_TEXT(10, 115, 27, 0, "CALIB DATA:");
    EVE_CMD_TEXT(190, 115, 27, EVE_OPT_CENTERX, "DAC");
    EVE_CMD_NUMBER(190, 140, 27, EVE_OPT_CENTERX, G_dac_cal_value);

    EVE_CMD_TEXT(290, 115, 27, EVE_OPT_CENTERX, "PD2");
    EVE_CMD_NUMBER(290, 140, 27, EVE_OPT_CENTERX, pd2);

    /* 温度数据显示 */
    EVE_CMD_TEXT(10, 165, 27, 0, "TP");
    EVE_CMD_NUMBER(190, 165, 27, EVE_OPT_CENTERX, G_tp_value);
    EVE_CMD_NUMBER(290, 165, 27, EVE_OPT_CENTERX, tp_heat);

    /* 导航按钮 */
    EVE_TAG_MASK(1);

    // 上箭头按钮（返回页面2）
    draw_arrow_button_ex(UP_ARROW_TAG, 3, keyup0_offset,410,30, 0, HIGH_VIS_TEXT_COLOR);


    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 140, 0);

    /* 功能按钮区域 */
    EVE_CMD_FGCOLOR(ROYAL_BLUE_COLOR_24bit);
    EVE_COLOR_RGB(WHITE_COLOR);

    // 触摸校验按钮（新增）
	EVE_TAG(TOUCH_CALIBRATE_TAG);  // 定义一个新的标签常量
	EVE_CMD_BUTTON(330, 210, 120, 40, 24, 0, "CalTouch");

    // 校准按钮
    EVE_TAG(IR_CAL_TAG);
    EVE_CMD_BUTTON(10,210,180,40,24,0,"CALIBRATE");

    // 加热按钮
    EVE_TAG(IR_HEAT_TAG);
    EVE_CMD_BUTTON(210,210,100,40,24,0,"20s Heat");

    /* 完成显示列表 */
    end_screen();
}


/**************************************
 * @brief 显示静态消息函数
 * @param strline1 第一行消息文本
 * @param strline2 第二行消息文本（可选）
 * @detail 特性：
 * - 支持单行/双行文本显示
 * - 自动根据文本长度调整字体大小
 * - 白底黑字显示
 * - 保持显示直到被其他界面替换
 */
#if USE_HOLD_MODE_NEW_SCREEN
void show_message(const char *strline1, const char *strline2)
{
    uint8_t len, len2;
    uint16_t font = 29;  // 默认字体大小（每行33字符）

    /* 自动计算最佳字体大小 */
    len = strlen(strline1);
    len2 = strlen(strline2);
    if (len2 > len) len = len2;

    if (len < 28) font = 30;  // 27字符/行
    if (len < 20) font = 31;  // 19字符/行
    if (len < 16) font = 1;   // 15字符/行

    begin_screen(default_config);

    /* 文本显示（保留原始英文） */
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_ROMFONT(1,32);

    if (strlen(strline2) == 0) {
        // 单行文本居中显示
        EVE_CMD_TEXT(240, 112, font, EVE_OPT_CENTERX, strline1);
    } else {
        // 双行文本居中显示
        EVE_CMD_TEXT(240, 82, font, EVE_OPT_CENTERX, strline1);
        EVE_CMD_TEXT(240, 142, font, EVE_OPT_CENTERX, strline2);
    }

    /* 完成显示 */
    end_screen();
}
#else

void show_message(const char *strline1, const char *strline2)
{
    uint8_t len, len2;
    uint16_t font = 29;  // 默认字体大小（每行33字符）

    /* 自动计算最佳字体大小 */
    len = strlen(strline1);
    len2 = strlen(strline2);
    if (len2 > len) len = len2;

    if (len < 28) font = 30;  // 27字符/行
    if (len < 20) font = 31;  // 19字符/行
    if (len < 16) font = 1;   // 15字符/行

    begin_screen(default_config);

    /* 文本显示（保留原始英文） */
    EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
    EVE_CMD_ROMFONT(1,32);

    if (strlen(strline2) == 0) {
        // 单行文本居中显示
        EVE_CMD_TEXT(240, 112, font, EVE_OPT_CENTERX, strline1);
    } else {
        // 双行文本居中显示
        EVE_CMD_TEXT(240, 82, font, EVE_OPT_CENTERX, strline1);
        EVE_CMD_TEXT(240, 142, font, EVE_OPT_CENTERX, strline2);
    }

    /* 完成显示 */
    end_screen();
}
#endif




/**************************************
 * show_total_volume()
 * show_total_volume()
 * @detail
 * Show total volume on display
 */
void show_total_volume(void)
{

	begin_screen(default_config);

   	EVE_CMD_SETFONT2(5, font0_offset, 48);	 //Font to handle 5, font resides at RAM_G + 0, first char is ASCII 48 which is number '0'
																														// Tag the numbers with tag 100
	EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);				//Set print color																				// Red color

  	EVE_CMD_NUMBER(80, 86,	5,  4, G_total_volume);

  	EVE_TAG_MASK(0);

	EVE_CMD_TEXT(12, 70, 31, EVE_OPT_CENTERX, "T");
	EVE_CMD_TEXT(12, 110, 31, EVE_OPT_CENTERX, "O");
	EVE_CMD_TEXT(12, 150, 31, EVE_OPT_CENTERX, "T");

	EVE_CMD_TEXT(40, 95, 29, EVE_OPT_CENTERX, "V");
	EVE_CMD_TEXT(40, 120, 29, EVE_OPT_CENTERX, "O");
	EVE_CMD_TEXT(40, 145, 29, EVE_OPT_CENTERX, "L");

	EVE_CMD_TEXT(393, 148, 30, EVE_OPT_CENTERX, "ML");

	end_screen();
}/**  End show_total_volume()  **/
/**************************************
 * @brief 显示启动界面
 * @detail 功能：
 * - 显示设备名称和版本号
 * - 显示当前时间（12小时制）
 * - 显示日期（日-月-年格式）
 */

// 根据主开关自动定义字体宏
#if USE_HOLD_MODE_NEW_SCREEN
void display_start_screen(void)
{
    chip_time timo;  // 时间结构体
    int16_t year;
    // 月份缩写文本数组
    const char txt_month[][12] = {"jan", "feb", "mar", "apr", "may", "jun",
                                 "jul", "aug", "sep", "oct", "nov", "dec"};

    /* 获取当前时间 */
    get_time(&timo);
    timo.months--;  // 月份调整为0-11
    year = 2000 + timo.years;  // 计算完整年份

    begin_screen(default_config);




    /* 设置文本颜色并显示设备信息 */
    EVE_COLOR_RGB(WHITE_COLOR);
    //EVE_CMD_TEXT(25, 100, 1, 0, "SENTRY @ hydrator");
    EVE_CMD_TEXT(25, 220, 29, 0, "ENG 1.0.8 ");
    EVE_CMD_TEXT(205, 220, 29, 0, "test 20260618");

    /* 时间显示（时:分 am/pm）*/
    EVE_CMD_NUMBER(372, 22, 29, EVE_OPT_RIGHTX, timo.hours);  // 小时
    EVE_CMD_TEXT(377, 22, 29, EVE_OPT_CENTERX, ":");         // 冒号分隔符
    EVE_CMD_NUMBER(385, 22, 29, 2, timo.minutes);            // 分钟（2位显示）
    // 上午/下午标识
    if (timo.period == 0)
        EVE_CMD_TEXT(455, 22, 29, EVE_OPT_RIGHTX, "AM");
    else
        EVE_CMD_TEXT(455, 22, 29, EVE_OPT_RIGHTX, "PM");

    /* 日期显示（日 月 年）*/
    EVE_CMD_NUMBER(25, 22, 29, 0, timo.days);  // 日
    EVE_CMD_TEXT(75, 22, 29, EVE_OPT_CENTERX, txt_month[timo.months]);  // 月（缩写）
    EVE_CMD_NUMBER(100, 22, 29, 0, year);  // 年

#if(1)
    //EVE_CMD_ROMFONT(1,32);  // 设置字体

    // 显示设备名称和版本号（居中显示）
    // 显示设备名称
    EVE_CMD_TEXT(25, 115, 31, 0, "SENTINEL");
    // 显示斜体hydrator（使用字体1的斜体版本）
    EVE_CMD_TEXT(455, 115, 31, EVE_OPT_RIGHTX, "Hydrator");  // 字体30可能是斜体
    // 绘制空心圆（绘制圆圈边框）
    EVE_BEGIN(EVE_BEGIN_POINTS);
    EVE_POINT_SIZE(1.2 * 16);  // 细线条
    // 绘制360度的点组成圆环
    int r = 10;  // 半径
    for(int i = 0; i < 360; i += 10) {
        int x = 230 + r * cos(i * 3.14159 / 180);
        int y = 130 + r * sin(i * 3.14159 / 180);
        EVE_VERTEX2F(x * 16, y * 16);
    }
    // 绘制白色R
    EVE_CMD_TEXT(230, 130, 26, EVE_OPT_CENTERX | EVE_OPT_CENTERY, "R");

#else
    // 加载位图到FT810内存地址0
    //EVE_CMD_APPEND(start_bitmap_offset, sizeof(start_bitmap));  // 注意第二个参数是数据大小
    EVE_LIB_WriteDataToRAMG(start_bitmap, sizeof(start_bitmap), start_bitmap_offset);//
   // delay_50ms();
    //EVE_COLOR_RGB(WHITE_COLOR);
    EVE_BITMAP_HANDLE(START_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(start_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 54, START_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, START_BITMAP_WIDTH, START_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(25, 20, START_BITMAP_HANDLE, 0);

#endif

    /* 完成显示列表 */
    end_screen();
    delay_50ms();
    if (G_display_brightness == 0){
      HAL_MemWrite8(EVE_REG_PWM_DUTY, 30);
    }
    else{
      HAL_MemWrite8(EVE_REG_PWM_DUTY, 60);
    }
    delay_50ms();

}

#else
void display_start_screen(void)
{
    chip_time timo;  // 时间结构体
    int16_t year;
    // 月份缩写文本数组
    const char txt_month[][12] = {"jan", "feb", "mar", "apr", "may", "jun",
                                 "jul", "aug", "sep", "oct", "nov", "dec"};

    /* 获取当前时间 */
    get_time(&timo);
    timo.months--;  // 月份调整为0-11
    year = 2000 + timo.years;  // 计算完整年份

    begin_screen(default_config);

    /* 设置文本颜色并显示设备信息 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_ROMFONT(1,32);  // 设置字体

    // 显示设备名称和版本号（居中显示）
    EVE_CMD_TEXT(240, 30, 1, EVE_OPT_CENTERX, "SENTRYhydrator");
    EVE_CMD_TEXT(240, 120, 1, EVE_OPT_CENTERX, "ENG 1.0.8 ");
    EVE_CMD_TEXT(420, 140, 28, EVE_OPT_CENTERX, "test 022026");

    /* 时间显示（时:分 am/pm）*/
    EVE_CMD_NUMBER(105, 220, 28, EVE_OPT_RIGHTX, timo.hours);  // 小时
    EVE_CMD_TEXT(109, 220, 28, EVE_OPT_CENTERX, ":");         // 冒号分隔符
    EVE_CMD_NUMBER(113, 220, 28, 2, timo.minutes);            // 分钟（2位显示）
    // 上午/下午标识
    if (timo.period == 0)
        EVE_CMD_TEXT(138, 220, 28, 0, "am");
    else
        EVE_CMD_TEXT(138, 220, 28, 0, "pm");

    /* 日期显示（日 月 年）*/
    EVE_CMD_NUMBER(304, 220, 28, EVE_OPT_RIGHTX, timo.days);  // 日
    EVE_CMD_TEXT(324, 220, 28, EVE_OPT_CENTERX, txt_month[timo.months]);  // 月（缩写）
    EVE_CMD_NUMBER(344, 220, 28, 0, year);  // 年

    /* 完成显示列表 */
    end_screen();
    delay_50ms();
    if (G_display_brightness == 0){
      HAL_MemWrite8(EVE_REG_PWM_DUTY, 30);
    }
    else{
      HAL_MemWrite8(EVE_REG_PWM_DUTY, 60);
    }
    delay_50ms();

}
#endif

/**************************************
 * @brief 显示水剂输送剩余时间
 * @param time_remaining 剩余时间（秒）
 * @detail 功能：
 * - 显示"WATER DELIVERY"标题
 * - 根据剩余时间显示秒或分钟
 * - 右侧显示导航箭头
 */
void show_bolus_time(uint16_t time_remaining)
{
    uint8_t t;  // 临时变量

    begin_screen(default_config);

    /* 设置文本颜色并显示标题 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(25, 25, 30, 0, "WATER DELIVERY");
    EVE_CMD_TEXT(25, 75, 30, 0, "TIME REMAINING");

    /* 根据剩余时间显示不同单位 */
    if (time_remaining < 60) {
        // 小于1分钟：显示秒数
        EVE_CMD_TEXT(220, 215, 30, 0, "SECONDS");
        EVE_CMD_ROMFONT(1,34);
        EVE_CMD_NUMBER(195, 158, 1, EVE_OPT_RIGHTX, time_remaining);
    } else {
        // 大于1分钟：转换为分钟显示
        t = 1 + time_remaining / 60;
        EVE_CMD_TEXT(220, 215, 30, 0, "MINUTES");
        EVE_CMD_ROMFONT(1,34);
        EVE_CMD_NUMBER(195, 158, 1, EVE_OPT_RIGHTX, t);
    }

    /* 右侧导航箭头 */
    EVE_TAG_MASK(1);
    draw_arrow_button_ex(BOLUS_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    /* 完成显示列表 */
    end_screen();
}
/**************************************
 * show_bolus_time()
 * @detail
 * Show bolus time screen
 */
/** End show_bolus_time()  **/


/**************************************
 * @brief 显示自动预充界面
 * @param set_ready 设备就绪状态
 * @param priming 预充进行中状态
 * @detail 功能：
 * - 根据状态显示"SET READY"或"SET NOT READY"
 * - 显示"AUTO PRIME"或"PRIME"标题
 * - 右侧显示导航箭头
 * - 就绪状态下显示圆形预充按钮
 * - 非就绪状态下显示禁用图标
 */
#if USE_HOLD_MODE_NEW_SCREEN
void auto_primescreen(uint8_t set_ready, uint8_t priming)
{
	begin_screen(default_config);
    /* 设置文本颜色并显示状态信息 */
    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);

    EVE_COLOR_RGB(WHITE_COLOR);
    if (set_ready) {
        if (!priming) {
            EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "SET READY");
        } else {
            EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "WAIT");

        }
        EVE_CMD_TEXT(200, 148, 31, EVE_OPT_CENTERX, "AUTO PRIME");
    } else {
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "SET NOT READY");
        EVE_CMD_TEXT(200, 148, 31, EVE_OPT_CENTERX, "PRIME");
    }

    /* 右侧导航箭头 */
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_TAG_MASK(1);
	EVE_TAG(RIGHT_ARROW_TAG);
	EVE_BEGIN(EVE_BEGIN_RECTS);
    if (!priming) {
    	EVE_VERTEX2II(396, 173, 0, 0);
    	EVE_VERTEX2II(480, 272, 0, 0);
    	EVE_COLOR_RGB(WHITE_COLOR);
    	draw_right_arrow_22x34(430, 207);
    }else{
    	EVE_VERTEX2II(396, 67, 0, 0);
    	EVE_VERTEX2II(480, 272, 0, 0);
    	EVE_COLOR_RGB(WHITE_COLOR);
    	draw_right_arrow_22x34(430, 152);
    }


    /* 预充按钮区域 */
    if (!priming) {
        if (set_ready) {
            EVE_TAG(PRIME_TAG);
        }
        EVE_COLOR_RGB(48, 48, 48);  // 灰色
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(396, 64, 0, 0);
        EVE_VERTEX2II(480, 169, 0, 0);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_TEXT(437, 116, 29, EVE_OPT_CENTERX|EVE_OPT_CENTERY, "RUN");
    }

    /* 非就绪状态显示禁用图标（红色叉号） */
    if (!set_ready) {
        EVE_COLOR_RGB(RED_COLOR);
        EVE_VERTEX_FORMAT(0);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,155);
        EVE_VERTEX2F(225,205);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,205);
        EVE_VERTEX2F(225,155);
    }
    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);
    /* 完成显示列表 */
    end_screen();
}
#else
void auto_primescreen(uint8_t set_ready, uint8_t priming)
{
	begin_screen(default_config);
    /* 设置文本颜色并显示状态信息 */

    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    if (set_ready) {
        if (!priming) {
            EVE_CMD_TEXT(12, 10, 31, 0, "SET READY");
        } else {
            EVE_CMD_TEXT(200, 180, 31, EVE_OPT_CENTERX, "WAIT");

        }
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "AUTO PRIME");
    } else {
        EVE_CMD_TEXT(12, 10, 31, 0, "SET NOT READY");
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "PRIME");
    }

    /* 右侧导航箭头 */

    EVE_TAG_MASK(1);
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    /* 预充按钮区域 */
    if (!priming) {
        if (set_ready) {
            EVE_TAG(PRIME_TAG);
        }
        // 绘制圆形按钮
        EVE_POINT_SIZE(400);
        EVE_BEGIN(EVE_BEGIN_POINTS);
        EVE_VERTEX2II(200,180,2,0);
    }

    /* 非就绪状态显示禁用图标（红色叉号） */
    if (!set_ready) {
        EVE_COLOR_RGB(RED_COLOR);
        EVE_VERTEX_FORMAT(0);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,155);
        EVE_VERTEX2F(225,205);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,205);
        EVE_VERTEX2F(225,155);
    }

    /* 完成显示列表 */
    end_screen();
}

#endif

/**************************************
 * @brief 显示手动预充界面
 * @param set_ready 设备就绪状态
 * @detail 功能：
 * - 根据状态显示就绪信息
 * - 双TPA类型时显示"FOOD"和"WATER"选项
 * - 右侧显示导航箭头
 * - 非就绪状态显示禁用图标
 */

#if USE_HOLD_MODE_NEW_SCREEN
void manual_primescreen(uint8_t set_ready)
{
    begin_screen(default_config);

    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);


    /* 设置文本颜色并显示状态信息 */

     /* 按钮区域处理 */
    if (!set_ready) {
        // 非就绪状态：显示禁用按钮
        EVE_TAG(NULL_TAG);
        EVE_POINT_SIZE(400);
        EVE_BEGIN(EVE_BEGIN_POINTS);
        EVE_VERTEX2II(200,180,2,0);

        // 绘制红色禁用图标
        EVE_COLOR_RGB(RED_COLOR);
        EVE_VERTEX_FORMAT(0);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,155);
        EVE_VERTEX2F(225,205);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,205);
        EVE_VERTEX2F(225,155);
    } else {
    	EVE_COLOR_RGB(48, 48, 48);  // 灰色
        // 就绪状态：根据TPA类型显示按钮
    	EVE_TAG_MASK(1);
        if (G_tpa_type == DUAL_TPA) {

        	EVE_TAG(PRIME_TAG_FOOD);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(396, 64, 0, 0);
            EVE_VERTEX2II(480, 114, 0, 0);
        }
        EVE_TAG(PRIME_TAG_WATER);
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(396, 118, 0, 0);
        EVE_VERTEX2II(480, 169, 0, 0);

        EVE_TAG(RIGHT_ARROW_TAG);
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(396, 173, 0, 0);
        EVE_VERTEX2II(480, 272, 0, 0);
        draw_right_arrow_22x34(430, 207);
        EVE_TAG_MASK(0);
    }

    EVE_COLOR_RGB(WHITE_COLOR);
    if (set_ready) {
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "SET READY");
        EVE_CMD_TEXT(200, 150, 31, EVE_OPT_CENTERX, "MANUAL PRIME");
        EVE_CMD_TEXT(200, 223, 28, EVE_OPT_CENTERX, "(Press and hold)");
        // 双TPA类型显示额外选项
        if (G_tpa_type == DUAL_TPA) {
            EVE_CMD_TEXT(437, 89, 29, EVE_OPT_CENTERX|EVE_OPT_CENTERY, "FOOD");
            EVE_CMD_TEXT(437, 143, 29, EVE_OPT_CENTERX|EVE_OPT_CENTERY, "WATER");

        }
    } else {
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "SET NOT READY");
        EVE_CMD_TEXT(200, 150, 31, EVE_OPT_CENTERX, "PRIME");
    }
    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);
    /* 完成显示列表 */
    end_screen();
}
#else
void manual_primescreen(uint8_t set_ready)
{
	begin_screen(default_config);

    /* 设置文本颜色并显示状态信息 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    if (set_ready) {
        EVE_CMD_TEXT(12, 10, 31, 0, "SET READY");
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "MANUAL PRIME");
        // 双TPA类型显示额外选项
        if (G_tpa_type == DUAL_TPA) {
            EVE_CMD_TEXT(120, 215, 31, EVE_OPT_CENTERX, "FOOD");
            EVE_CMD_TEXT(280, 215, 31, EVE_OPT_CENTERX, "WATER");
        }
    } else {
        EVE_CMD_TEXT(12, 10, 31, 0, "SET NOT READY");
        EVE_CMD_TEXT(200, 100, 31, EVE_OPT_CENTERX, "PRIME");
    }

    /* 右侧导航箭头 */
    EVE_TAG_MASK(1);
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    /* 按钮区域处理 */
    if (!set_ready) {
        // 非就绪状态：显示禁用按钮
        EVE_TAG(NULL_TAG);
        EVE_POINT_SIZE(400);
        EVE_BEGIN(EVE_BEGIN_POINTS);
        EVE_VERTEX2II(200,180,2,0);

        // 绘制红色禁用图标
        EVE_COLOR_RGB(RED_COLOR);
        EVE_VERTEX_FORMAT(0);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,155);
        EVE_VERTEX2F(225,205);
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_VERTEX2F(175,205);
        EVE_VERTEX2F(225,155);
    } else {
        // 就绪状态：根据TPA类型显示按钮
        if (G_tpa_type == DUAL_TPA) {
            EVE_TAG(PRIME_TAG_FOOD);
            EVE_POINT_SIZE(400);
            EVE_BEGIN(EVE_BEGIN_POINTS);
            EVE_VERTEX2II(120,180,2,0);
        }
        EVE_TAG(PRIME_TAG_WATER);
        EVE_POINT_SIZE(400);
        EVE_BEGIN(EVE_BEGIN_POINTS);
        EVE_VERTEX2II(280,180,2,0);
    }

    /* 完成显示列表 */
    end_screen();
}
#endif
/**************************************
 * @brief 设置剂量限制界面
 * @detail 功能：
 * - 显示当前剂量限制值
 * - 提供上下箭头调整数值
 * - 右箭头确认设置
 */


#if USE_HOLD_MODE_NEW_SCREEN
void set_dose_limit_screen()
{
	begin_screen(default_config);

    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);

    /* 设置文本颜色和标题 */
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_TEXT(200, 73, 30, EVE_OPT_CENTERX, "SET DOSE LIMIT");
    EVE_CMD_TEXT(268, 210, 30, 0, "mL");  // 单位显示

    /* 显示当前剂量限制值 */
    EVE_CMD_ROMFONT(1,34);
    EVE_CMD_NUMBER(268, 159, 1, EVE_OPT_RIGHTX, G_dose_limit);

    /* ========== 按钮区域 ========== */
    EVE_TAG_MASK(1);
    EVE_TAG(UP_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 64, 0, 0);
    EVE_VERTEX2II(480, 114, 0, 0);
    EVE_COLOR_RGB(255, 255, 255);
    draw_plus_icon(430, 77);
  //  EVE_CMD_ROMFONT(1, 32);
  //  EVE_CMD_TEXT(440, 140, 1, EVE_OPT_CENTER, "-");
    EVE_TAG(DOWN_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 118, 0, 0);
    EVE_VERTEX2II(480, 169, 0, 0);
    EVE_COLOR_RGB(255, 255, 255);
    draw_minus_icon(430, 134);
 //   draw_rightarrow_icon(430, 207);
    EVE_TAG(RIGHT_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 173, 0, 0);
    EVE_VERTEX2II(480, 272, 0, 0);
    draw_right_arrow_22x34(430, 207);
  //  EVE_CMD_TEXT(440, 217, 1, EVE_OPT_CENTER, ">");
  //  draw_battery_simple(25,20,90,0);

    EVE_TAG_MASK(0);

    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);
    // 下箭头按钮（减少数值）

    /* 完成显示列表 */
    end_screen();
}
#else
void set_dose_limit_screen()
{
	begin_screen(default_config);

    /* 设置文本颜色和标题 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(25, 25, 31, 0, "SET DOSE LIMIT");
    EVE_CMD_TEXT(335, 40, 29, 0, "ML");  // 单位显示

    /* 显示当前剂量限制值 */
    EVE_CMD_ROMFONT(1,34);
    EVE_CMD_NUMBER(200, 135, 1, EVE_OPT_CENTERX, G_dose_limit);

    /* 导航和调整按钮 */
    EVE_TAG_MASK(1);  // 启用标签检测

    // 右箭头按钮（确认）
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    draw_arrow_button(UP_ARROW_TAG, 3, keyup0_offset, 330, 163, 0);
    // 上箭头按钮（增加数值）
    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 30, 163, 0);
    // 下箭头按钮（减少数值）

    /* 完成显示列表 */
    end_screen();
}
#endif
/**************************************
 * @brief 设置每小时水剂量界面
 * @detail 功能：
 * - 显示当前水剂量设置
 * - 提供上下箭头调整数值
 * - 右箭头确认设置
 */
#if USE_HOLD_MODE_NEW_SCREEN
void set_water_rate_screen(void)
{
	begin_screen(default_config);

    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);



    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 64, 0, 0);
    EVE_VERTEX2II(480, 114, 0, 0);


    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 118, 0, 0);
    EVE_VERTEX2II(480, 169, 0, 0);
    /* 设置文本颜色和标题 */
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_TEXT(200, 73, 30, EVE_OPT_CENTERX, "SET HOURLY");
    EVE_CMD_TEXT(200, 119, 30, EVE_OPT_CENTERX, "WATER VOLUME");
    EVE_CMD_TEXT(268, 210, 30, 0, "ML");  // 单位显示

    /* 显示当前水剂量值 */
    EVE_CMD_ROMFONT(1,34);
    EVE_CMD_NUMBER(67, 157, 1, 0, G_water_rate);

    /* 导航和调整按钮 */
    EVE_TAG_MASK(1);  // 启用标签检测

    EVE_TAG(UP_ARROW_TAG);
    draw_plus_icon(430, 77);

    EVE_TAG(DOWN_ARROW_TAG);
    draw_minus_icon(430, 134);
 //   draw_rightarrow_icon(430, 207);
    EVE_TAG(RIGHT_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 173, 0, 0);
    EVE_VERTEX2II(480, 272, 0, 0);
    draw_right_arrow_22x34(430, 207);


    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);
    // 下箭头按钮（减少数值）

    /* 完成显示列表 */
    end_screen();
}

#else
void set_water_rate_screen(void)
{
	begin_screen(default_config);

    /* 设置文本颜色和标题 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(25, 25, 30, 0, "SET HOURLY");
    EVE_CMD_TEXT(25, 60, 30, 0, "WATER VOLUME");
    EVE_CMD_TEXT(270, 69, 28, 0, "ML");  // 单位显示

    /* 显示当前水剂量值 */
    EVE_CMD_ROMFONT(1,34);
    EVE_CMD_NUMBER(200, 135, 1, EVE_OPT_CENTERX, G_water_rate);

    /* 导航和调整按钮 */
    EVE_TAG_MASK(1);  // 启用标签检测

    // 右箭头按钮（确认）
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    draw_arrow_button(UP_ARROW_TAG, 3, keyup0_offset, 330, 163, 0);
    // 上箭头按钮（增加数值）
    draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 30, 163, 0);
    // 下箭头按钮（减少数值）


    /* 完成显示列表 */
    end_screen();
}
#endif
/**************************************
 * set_water_rate_screen()
 * @detail
 * Display edit bolus screen
  */
/**  End set_water_rate_screen()  **/
/**************************************
 * @brief 清除数值界面
 * @detail 功能：
 * - 显示可清除的数值选项（体积、剂量限制、总体积）
 * - 每个选项旁有清除按钮
 * - 右箭头退出界面
 */
#if USE_HOLD_MODE_NEW_SCREEN
void clear_rates_screen(void)
{
	begin_screen(default_config);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(0, 0, 0, 0);
    EVE_VERTEX2II(480, 60, 0, 0);

    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 64, 0, 0);
    EVE_VERTEX2II(480, 272, 0, 0);

    /* 设置文本颜色和选项列表 */
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_TEXT(80, 20, 29, 0, "CLEAR ");

    /* 设置文本颜色和选项列表 */
    EVE_COLOR_RGB(WHITE_COLOR);
    EVE_CMD_TEXT(23, 83, 29, 0, "VOLUME");
    EVE_CMD_NUMBER(377, 83, 28, EVE_OPT_RIGHTX, G_volume);

    EVE_CMD_TEXT(23, 130, 29, 0, "DOSE LIMIT");
    EVE_CMD_NUMBER(377, 130, 28, EVE_OPT_RIGHTX, G_dose_limit);

    EVE_CMD_TEXT(23, 177, 29, 0, "TOTAL VOLUME");
    EVE_CMD_NUMBER(377, 177, 28, EVE_OPT_RIGHTX, G_total_volume);

    EVE_CMD_TEXT(23, 224, 29, 0, "TOTAL WATER");
    EVE_CMD_NUMBER(377, 224, 28, EVE_OPT_RIGHTX, G_water);


    /* 导航和功能按钮 */
    EVE_TAG_MASK(1);  // 启用标签检测

    EVE_TAG(RIGHT_ARROW_TAG);
    EVE_COLOR_RGB(48, 48, 48);  // 灰色
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2II(396, 118, 0, 0);
    EVE_VERTEX2II(480, 169, 0, 0);

    EVE_COLOR_RGB(WHITE_COLOR);
    // 右箭头按钮（退出）
    draw_right_arrow_22x34(426, 147);

    /* 清除按钮（使用点绘制） */
    EVE_POINT_SIZE(200);
    EVE_BEGIN(EVE_BEGIN_POINTS);

    // 清除体积按钮
    EVE_TAG(CLR_VOLUME_TAG);
    EVE_VERTEX2II(300,95,0,0);

    // 清除剂量限制按钮
    EVE_TAG(CLR_DOSE_TAG);
    EVE_VERTEX2II(300,142,0,0);

    // 清除总体积按钮
    EVE_TAG(CLR_TOT_VOLUME_TAG);
    EVE_VERTEX2II(300,189,0,0);

    // 清除总体积按钮
    EVE_TAG(CLR_TOT_WATER_TAG);
    EVE_VERTEX2II(300,236,0,0);

    /* 电池符号显示 */
    draw_batt_symbol(G_v_battery);

    /* 完成显示列表 */
    end_screen();
}
#else
void clear_rates_screen(void)
{
	begin_screen(default_config);

    /* 设置文本颜色和选项列表 */
    EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
    EVE_CMD_TEXT(63, 25, 29, 0, "CLR VOLUME");
    EVE_CMD_NUMBER(300, 29, 28, 0, G_volume);

    EVE_CMD_TEXT(63, 89, 29, 0, "CLR DOSE LIMIT");
    EVE_CMD_NUMBER(300, 94, 28, 0, G_dose_limit);

    EVE_CMD_TEXT(63, 153, 29, 0, "CLR TOTAL VOLUME");
    EVE_CMD_NUMBER(300, 157, 28, 0, G_total_volume);

    /* 导航和功能按钮 */
    EVE_TAG_MASK(1);  // 启用标签检测

    // 右箭头按钮（退出）
    draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,112, 0, HIGH_VIS_TEXT_COLOR);

    /* 清除按钮（使用点绘制） */
    EVE_POINT_SIZE(250);
    EVE_BEGIN(EVE_BEGIN_POINTS);

    // 清除体积按钮
    EVE_TAG(CLR_VOLUME_TAG);
    EVE_VERTEX2II(32,39,0,0);

    // 清除剂量限制按钮
    EVE_TAG(CLR_DOSE_TAG);
    EVE_VERTEX2II(32,103,0,0);

    // 清除总体积按钮
    EVE_TAG(CLR_TOT_VOLUME_TAG);
    EVE_VERTEX2II(32,167,0,0);

    /* 完成显示列表 */
    end_screen();
}
#endif
/**************************************
 * clear_rates_screen()
 * @detail
 * Display volume, dose limit and total volume. Display reset button for each variable.
 */
/** End clear_rates_screen()  **/
/**************************************
 * @brief 控制锁定状态界面
 * @param LockState 当前锁定状态
 * @return uint8_t 新的锁定状态
 * @detail 功能：
 * - 显示当前锁定/解锁状态
 * - 通过RUN/STOP按钮切换状态
 * - 超时自动返回
 */
uint8_t controls_locked_screen(uint8_t LockState)
{
    uint16_t timeout_count = 0;
    uint8_t key = 0;
    uint8_t key_pressed = 0;  // 新增：记录按键是否已按下
    uint16_t debounce_counter = 0;  // 新增：去抖计数器

    // 播放状态切换提示音
    beep_1khz_1sec(G_warning_vol);

    while(1) {
    	begin_screen(default_config);

        /* 显示锁定状态 */
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
        EVE_CMD_TEXT(240, 85, 31, EVE_OPT_CENTERX, "CONTROLS");

        if (LockState) {
            EVE_CMD_TEXT(240, 155, 31, EVE_OPT_CENTERX, "LOCKED");
        } else {
            EVE_CMD_TEXT(240, 155, 31, EVE_OPT_CENTERX, "UNLOCKED");
        }

        /* 完成显示列表 */
        end_screen();

        /* 处理用户输入 */
        key = read_keypad();
        if (key == RUN_STOP_SWITCH) {
        	if (!key_pressed) {  // 只有当按键之前未被按下时才处理
        		if (timeout_count >= 40) {
        			key_click();
        			LockState = !LockState;  // 切换锁定状态
        			key_pressed = 1;  // 标记按键已按下
        			debounce_counter = 0;  // 重置去抖计数器
        			timeout_count = 0;
        		}
        	} else {
        		// 按键保持按下状态，增加去抖计数器
        		if (debounce_counter < 20) {  // 大约1秒(20*50ms)的去抖时间
        			debounce_counter++;
        		}
        	}
        } else {
        	// 按键释放或没有按键
        	key_pressed = 0;
        	if (timeout_count > 80) return LockState;
        }

        /* 超时处理 */
        if (timeout_count < 10000) timeout_count++;
        if (timeout_count > 200) return LockState;

        /* 50ms延时控制 */
        delay_50ms(); // 等待50ms
    }

    return LockState;
}

/**************************************
 * controls_locked_screen()
 * @detail
 * Display CONNTROLS LOCKED / CONTROLS UNLOCKED screen
 */
/**  End controls_locked_screen()  **/
/**************************************
 * @brief 关机进度条界面
 * @return uint8_t 任务状态
 * @detail 功能：
 * - 显示关机进度条
 * - 长按电源键触发关机
 * - 中途松开取消关机
 */
uint8_t MCU_off_bar(void)
{
    uint8_t key, bar_count = 1, one_second_count = 0;

    key = read_keypad();
    while (key == POWER_SWITCH) {  // 持续检测电源键按下
        /* 构建显示列表 */
    	begin_screen(default_config);

        /* 显示进度条 */
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
        EVE_CMD_PROGRESS(115, 121, 250, 30, 0, bar_count, 5);

        /* 完成显示列表 */
        end_screen();

        /* 更新进度条 */
        key = read_keypad();
        one_second_count++;
        if (one_second_count > 12) {
            one_second_count = 0;
            bar_count++;
        }

        /* 进度完成触发关机 */
        if (bar_count > 5) return MCU_OFF__TASK;

        /* 50ms延时控制 */
        delay_50ms(); // 等待50ms
    }

    return HOLD_MODE__TASK;  // 取消关机
}
/**************************************
 * MCU_off_bar()
 * @detail
 * Prepare pump to power down. Prompt for set removal. Save parameters to NVRAM
 */
/**  End MCU_off_bar()  **/

/**************************************
 * @brief 设置界面任务
 * @return uint8_t 返回任务代码
 * @detail 功能：
 * - 显示并编辑系统各项设置
 * - 包含显示亮度、音量等参数设置
 * - 右箭头按钮返回保持模式
 */

#if USE_HOLD_MODE_NEW_SCREEN
uint8_t setting_screen_task()
{
    uint8_t key;
    uint16_t timeout_count = 0;

    do {
    	begin_screen(default_config);
        EVE_COLOR_RGB(48, 48, 48);  // 灰色

        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(0, 0, 0, 0);
        EVE_VERTEX2II(480, 60, 0, 0);

        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(396, 64, 0, 0);
        EVE_VERTEX2II(480, 272, 0, 0);

        /* 设置文本颜色和选项列表 */
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_TEXT(80, 20, 29, 0, "SETTINGS ");

        /* 设置文本颜色和标题 */
        EVE_CMD_TEXT(15, 115, 27, 0, "DISPLAY BRIGHTNESS");
        EVE_CMD_TEXT(15, 158, 27, 0, "VOLUME - ALARMS");
        EVE_CMD_TEXT(15, 201, 27, 0, "VOLUME - ANNUNCIATIONS");

        /* 设置选项按钮 */
        EVE_COLOR_RGB(BLACK_COLOR);

        // 显示亮度设置
        set_fgcolor_for(G_display_brightness, COLUMN_0);
        EVE_TAG(DSP_BRG_LOW_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(250,105,60,36,28,256,"LOW");
        set_fgcolor_for(G_display_brightness, COLUMN_1);
        EVE_TAG(DSP_BRG_HI_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(320,105,60,36,28,256,"HIGH");

        // 报警音量设置
        set_fgcolor_for(G_alarm_vol, COLUMN_0);
        EVE_TAG(ALARM_LOW_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(250,148,60,36,28,256,"LOW");
        set_fgcolor_for(G_alarm_vol, COLUMN_1);
        EVE_TAG(ALARM_HI_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(320,148,60,36,28,256,"HIGH");

        // 提示音量设置
        set_fgcolor_for(G_warning_vol, COLUMN_0);
        EVE_TAG(ANN_LOW_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(250,191,60,36,28,256,"LOW");
        set_fgcolor_for(G_warning_vol, COLUMN_1);
        EVE_TAG(ANN_HI_TAG);
        EVE_COLOR_RGB(WHITE_COLOR);
        EVE_CMD_BUTTON(320,191,60,36,28,256,"HIGH");

        EVE_TAG(RIGHT_ARROW_TAG);
        EVE_COLOR_RGB(48, 48, 48);  // 灰色
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2II(396, 118, 0, 0);
        EVE_VERTEX2II(480, 169, 0, 0);

        EVE_COLOR_RGB(WHITE_COLOR);
        // 右箭头按钮（退出）
        draw_right_arrow_22x34(426, 147);

        /* 电池符号显示 */
        draw_batt_symbol(G_v_battery);
        end_screen();

        /* 处理用户输入 */
        key = read_all_keys(&timeout_count);

        switch (key) {
            case DSP_BRG_LOW_TAG:
            	G_display_brightness = 0;
            	HAL_MemWrite8(EVE_REG_PWM_DUTY, 30);
            	delay_50ms(); // 等待50ms
            break;
            case DSP_BRG_HI_TAG:
            	G_display_brightness = 1;
            	HAL_MemWrite8(EVE_REG_PWM_DUTY, 60);
            	delay_50ms(); // 等待50ms
            	break;
            case ALARM_LOW_TAG:
            	G_alarm_vol = 0;
            	beep_1khz_1sec(G_alarm_vol);
            	delay_50ms(); // 等待50ms
            	break;
            case ALARM_HI_TAG:
            	G_alarm_vol = 1;
        		beep_1khz_1sec(G_alarm_vol);
        		delay_50ms(); // 等待50ms
        		break;
            case ANN_LOW_TAG:
            	G_warning_vol = 0;
        		beep_1khz_1sec(G_warning_vol);
        		delay_50ms(); // 等待50ms
            	break;
            case ANN_HI_TAG:
            	G_warning_vol = 1;
        		beep_1khz_1sec(G_warning_vol);
        		delay_50ms(); // 等待50ms
            	break;
        }

        /* 退出条件处理 */
        if (key == (RIGHT_ARROW_PRESSED)) key_click();
        if (key == (RIGHT_ARROW_RELEASED)) {
            SaveNV_Settings();
            return HOLD_MODE__TASK;
        }
        if (timeout_count > EDIT_SCREENS_TIMEOUT) {
            SaveNV_Settings();
            return HOLD_MODE__TASK;
        }

        /* 50ms延时控制 */
        delay_50ms(); // 等待50ms
    } while(1);
}

#else
uint8_t setting_screen_task()
{
    uint8_t key;
    uint16_t timeout_count = 0;

    do {
    	begin_screen(default_config);

        /* 设置文本颜色和标题 */
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);
        EVE_CMD_TEXT(15, 15, 29, 0, "SETTINGS:");
        EVE_CMD_TEXT(15, 60, 27, 0, "DISPLAY BRIGHTNESS");
        EVE_CMD_TEXT(15, 103, 27, 0, "VOLUME - ALARMS");
        EVE_CMD_TEXT(15, 146, 27, 0, "VOLUME - ANNUNCIATIONS");
        EVE_CMD_TEXT(15, 189, 27, 0, "DOSE COMPLETE CALLBACK");
        EVE_CMD_TEXT(15, 232, 27, 0, "ENTER SVC MODE HOLD TIME");

        /* 设置选项按钮 */
        EVE_COLOR_RGB(BLACK_COLOR);

        // 显示亮度设置
        set_fgcolor_for(G_display_brightness, COLUMN_0);
        EVE_TAG(DSP_BRG_LOW_TAG);
        EVE_CMD_BUTTON(250,50,60,36,28,256,"LOW");
        set_fgcolor_for(G_display_brightness, COLUMN_1);
        EVE_TAG(DSP_BRG_HI_TAG);
        EVE_CMD_BUTTON(320,50,60,36,28,256,"HIGH");

        // 报警音量设置
        set_fgcolor_for(G_alarm_vol, COLUMN_0);
        EVE_TAG(ALARM_LOW_TAG);
        EVE_CMD_BUTTON(250,93,60,36,28,256,"LOW");
        set_fgcolor_for(G_alarm_vol, COLUMN_1);
        EVE_TAG(ALARM_HI_TAG);
        EVE_CMD_BUTTON(320,93,60,36,28,256,"HIGH");

        // 提示音量设置
        set_fgcolor_for(G_warning_vol, COLUMN_0);
        EVE_TAG(ANN_LOW_TAG);
        EVE_CMD_BUTTON(250,136,60,36,28,256,"LOW");
        set_fgcolor_for(G_warning_vol, COLUMN_1);
        EVE_TAG(ANN_HI_TAG);
        EVE_CMD_BUTTON(320,136,60,36,28,256,"HIGH");

        // 剂量完成回调时间设置
        set_fgcolor_for(G_dose_complete_callback_index, COLUMN_0);
        EVE_TAG(DOSE_COMP_ALARM_5S_TAG);
        EVE_CMD_BUTTON(250,179,60,36,28,256,"5sec");
        set_fgcolor_for(G_dose_complete_callback_index, COLUMN_1);
        EVE_TAG(DOSE_COMP_ALARM_60S_TAG);
        EVE_CMD_BUTTON(320,179,60,36,28,256,"60sec");
        set_fgcolor_for(G_dose_complete_callback_index, COLUMN_2);
        EVE_TAG(DOSE_COMP_ALARM_5M_TAG);
        EVE_CMD_BUTTON(390,179,60,36,28,256,"5min");

        // 服务模式进入延迟设置
        set_fgcolor_for(G_svc_entry_delay, COLUMN_0);
        EVE_TAG(SVC_ENTRY_3S_TAG);
        EVE_CMD_BUTTON(250,222,60,36,28,256,"3sec");
        set_fgcolor_for(G_svc_entry_delay, COLUMN_1);
        EVE_TAG(SVC_ENTRY_6S_TAG);
        EVE_CMD_BUTTON(320,222,60,36,28,256,"6sec");
        set_fgcolor_for(G_svc_entry_delay, COLUMN_2);
        EVE_TAG(SVC_ENTRY_9S_TAG);
        EVE_CMD_BUTTON(390,222,60,36,28,256,"9sec");

        /* 右箭头按钮（退出）*/
        EVE_TAG(RIGHT_ARROW_TAG);
        draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,25, 0, HIGH_VIS_TEXT_COLOR);

        /* 完成显示列表 */
        end_screen();

        /* 处理用户输入 */
        key = read_all_keys(&timeout_count);

        switch (key) {
            case DSP_BRG_LOW_TAG:
            	G_display_brightness = 0;
            	HAL_MemWrite8(EVE_REG_PWM_DUTY, 30);
            	delay_50ms(); // 等待50ms
            break;
            case DSP_BRG_HI_TAG:
            	G_display_brightness = 1;
            	HAL_MemWrite8(EVE_REG_PWM_DUTY, 60);
            	delay_50ms(); // 等待50ms
            	break;
            case ALARM_LOW_TAG:
            	G_alarm_vol = 0;
            	beep_1khz_1sec(G_alarm_vol);
            	delay_50ms(); // 等待50ms
            	break;
            case ALARM_HI_TAG:
            	G_alarm_vol = 1;
        		beep_1khz_1sec(G_alarm_vol);
        		delay_50ms(); // 等待50ms
        		break;
            case ANN_LOW_TAG:
            	G_warning_vol = 0;
        		beep_1khz_1sec(G_warning_vol);
        		delay_50ms(); // 等待50ms
            	break;
            case ANN_HI_TAG:
            	G_warning_vol = 1;
        		beep_1khz_1sec(G_warning_vol);
        		delay_50ms(); // 等待50ms
            	break;
            case DOSE_COMP_ALARM_5S_TAG:  G_dose_complete_callback_index = 0; break;
            case DOSE_COMP_ALARM_60S_TAG: G_dose_complete_callback_index = 1; break;
            case DOSE_COMP_ALARM_5M_TAG:  G_dose_complete_callback_index = 2; break;
            case SVC_ENTRY_3S_TAG: G_svc_entry_delay = 0; break;
            case SVC_ENTRY_6S_TAG: G_svc_entry_delay = 1; break;
            case SVC_ENTRY_9S_TAG: G_svc_entry_delay = 2; break;
        }

        /* 退出条件处理 */
        if (key == (RIGHT_ARROW_PRESSED)) key_click();
        if (key == (RIGHT_ARROW_RELEASED)) {
            SaveNV_Settings();
            return HOLD_MODE__TASK;
        }
        if (timeout_count > EDIT_SCREENS_TIMEOUT) {
            SaveNV_Settings();
            return HOLD_MODE__TASK;
        }

        /* 50ms延时控制 */
        delay_50ms(); // 等待50ms
    } while(1);
}
#endif
/**************************************
 * @brief 特殊字符显示界面
 * @return uint8_t 返回任务代码
 * @detail 功能：
 * - 演示FT810特殊字符字体
 * - 右箭头按钮返回保持模式
 */
uint8_t special_char_screen(void)
{
    uint8_t key;
    uint16_t timeout_count = 0;
    // 特殊字符测试数据
    const char r1[] = {97,98,99,100,101,102,103,104,0};
    const char r2[] = {136,137,138,139,140,141,142,143,0};
    const char r3[] = {144,145,146,147,148,149,150,151,0};
    const char r4[] = {152,153,154,155,156,157,158,159,0};
    const char r5[] = {160,161,162,163,164,165,166,167,0};
    const char r6[] = {168,169,170,171,172,173,174,175,0};
    const char r7[] = {224,225,226,227,228,229,230,231,0};
    const char r8[] = {232,233,234,235,236,237,238,239,0};

    do {
    	begin_screen(default_config);

        /* 显示特殊字符行 */
        EVE_CMD_TEXT(15, 10, 19, 0, r1);
        EVE_CMD_TEXT(15, 40, 19, 0, r2);
        EVE_CMD_TEXT(15, 70, 19, 0, r3);
        EVE_CMD_TEXT(15, 100, 19, 0, r4);
        EVE_CMD_TEXT(15, 130, 19, 0, r5);
        EVE_CMD_TEXT(15, 160, 19, 0, r6);
        EVE_CMD_TEXT(15, 190, 19, 0, r7);
        EVE_CMD_TEXT(15, 220, 19, 0, r8);

        /* 右箭头按钮（退出）*/
        draw_arrow_button_ex(RIGHT_ARROW_TAG, 2, keyright0_offset,410,25, 0, HIGH_VIS_TEXT_COLOR);

        /* 完成显示列表 */
        end_screen();

        /* 处理用户输入 */
        key = read_all_keys(&timeout_count);

        if (key == (RIGHT_ARROW_PRESSED)) key_click();
        if (key == (RIGHT_ARROW_RELEASED)) {
            return HOLD_MODE__TASK;
        }
        if (timeout_count > EDIT_SCREENS_TIMEOUT) {
            return HOLD_MODE__TASK;
        }

        /* 50ms延时控制 */
        delay_50ms(); // 等待50ms
    } while(1);
}


/**************************************
 * @brief 错误信息界面
 * @return uint8_t 返回任务代码
 *   - NORMAL_EXIT: 正常退出
 *   - INACTIVITY_EXIT: 超时退出
 * @detail 功能：
 * - 显示存储在EEPROM中的错误信息
 * - 每屏显示15条错误记录
 * - 支持上下箭头翻页浏览
 * - 右箭头退出界面
 * - 自动处理记录循环（首尾相接）
 */
uint8_t error_screen(void)
{
    err_rec rec;                    // 错误记录结构体
    uint8_t i, k = 26;              // k: 字体大小
    uint8_t y, key, new_screen;     // y: 行位置坐标
    uint16_t timeout_count = 0;     // 超时计数器
    int16_t rec_no;                 // 当前记录编号

    /* 初始化记录位置 */
    rec_no = ReadNV16(ERR_MSG_HEAD);  // 从EEPROM读取最后错误位置
    if ((rec_no == 0) || (rec_no > ERR_MSG_MAX_NO_OF)) {
        rec_no = 1;  // 无效值处理
    }

    do {
        new_screen = 0;  // 页面刷新标志

        begin_screen(default_config);
        EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);  // 错误信息使用报警颜色

        /* 显示15条错误记录 */
        for (i = 0; i < 15; i++) {
            /* 获取错误记录 */
            get_err_rec(rec_no, &rec);
            rec_no--;  // 逆序显示（最新记录在前）

            /* 记录号循环处理 */
            if (rec_no < 1) {
                rec_no = ERR_MSG_MAX_NO_OF;
            }

            /* 计算当前行Y坐标 */
            y = 9 + i * 17;  // 每行间隔17像素

            /* 空记录显示分隔线 */
            if (rec.err_num == 0xff) {
                EVE_CMD_TEXT(5, y, k, 0, "---------------");
            }
            /* 校准成功特殊标记 */
            else if (rec.err_num == CAL_OK) {
                EVE_CMD_TEXT(5, y, k, 0, "CAL OK");
            }
            /* 正常错误记录显示 */
            else {
                // 错误编号
                EVE_CMD_TEXT(5, y, k, 0, "ERR");
                EVE_CMD_NUMBER(30, y, k, 0, rec.err_num);

                // 时间信息（时:分 AM/PM）
                EVE_CMD_NUMBER(75, y, k, 2048 | 2, rec.hours);  // 2048=右对齐
                EVE_CMD_TEXT(81, y, k, 512, ":");               // 512=居中
                EVE_CMD_NUMBER(87, y, k, 0 | 2, rec.minutes);   // 2=固定2位
                EVE_CMD_TEXT(106, y, k, 0, (rec.period == 0) ? "AM" : "PM");

                // 日期信息（月/日/年）
                EVE_CMD_NUMBER(145, y, k, 512 | 2, rec.months);
                EVE_CMD_TEXT(159, y, k, 512, "/");
                EVE_CMD_NUMBER(173, y, k, 512 | 2, rec.days);
                EVE_CMD_TEXT(187, y, k, 512, "/");
                EVE_CMD_NUMBER(201, y, k, 512, rec.years);
            }
        }

        /*--- 导航按钮区域 ---*/
        EVE_TAG_MASK(1);  // 启用标签检测
        draw_arrow_button(RIGHT_ARROW_TAG, 2, keyright0_offset, 410, 112, 0);
        // 扩大触摸区域（透明矩形）
        EVE_COLOR_MASK(0,0,0,0);       // 禁用颜色写入
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(390,100);         // 左上角
        EVE_VERTEX2F(478,172);         // 右下角
        EVE_COLOR_MASK(1,1,1,1);       // 恢复颜色写入
        // 右箭头按钮（退出）

        draw_arrow_button(UP_ARROW_TAG, 3, keyup0_offset, 410, 30, 0);
        // 扩大触摸区域
        EVE_COLOR_MASK(0,0,0,0);
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(390,18);
        EVE_VERTEX2F(478,90);
        EVE_COLOR_MASK(1,1,1,1);
        // 上箭头按钮（向上翻页)

        draw_arrow_button(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194, 0);

        // 扩大触摸区域
        EVE_COLOR_MASK(0,0,0,0);
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(390,182);
        EVE_VERTEX2F(478,254);
        EVE_COLOR_MASK(1,1,1,1);

        /* 完成显示列表 */
        end_screen();

        /*--- 用户输入处理循环 ---*/
        do {
            // 读取按键（同时更新超时计数器）
            key = read_all_keys(&timeout_count);

            /* 下箭头处理（向下翻页） */
            if (key == (DOWN_ARROW_PRESSED)) {
                key_click();  // 按键音反馈
            }
            if (key == (DOWN_ARROW_RELEASED)) {
                new_screen = 1;  // 标记需要刷新界面
            }

            /* 上箭头处理（向上翻页） */
            if (key == (UP_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (UP_ARROW_RELEASED)) {
                // 跳转到更早的记录（每次跳转30条记录）
                rec_no += 2 * ERR_SCRN_NO_OF_LINES;
                if (rec_no > ERR_MSG_MAX_NO_OF) {
                    rec_no -= ERR_MSG_MAX_NO_OF;  // 循环处理
                }
                new_screen = 1;
            }

            /* 右箭头处理（退出界面） */
            if (key == (RIGHT_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (RIGHT_ARROW_RELEASED)) {
                return NORMAL_EXIT;  // 正常退出
            }

            /* 超时处理 */
            if (timeout_count > EDIT_SCREENS_TIMEOUT) {
                return INACTIVITY_EXIT;  // 超时退出
            }

            /* 50ms延时控制 */
            delay_50ms(); // 等待50ms

        } while (!new_screen);  // 无新页面刷新时保持循环

    } while(1);  // 主循环
}
/**************************************
 * error_screen()
 * @short		Error message screen
 * @param		location of last saved error message
 * @retval  	uint8_t, return task code.
 * @detail
 * Display error messages saved in eprom. 15 errors per screen. Scroll up/down with
 * up and down arrows. Exit with right arrow.
 */

/**************************************
 * daily_volume_screen()
 * @detail 显示存储在EPROM中的每日输送食物和药液体积记录，每屏显示15条
 * 使用上下箭头翻页，右箭头退出。最多保存90天数据，之后覆盖最旧记录
 */
uint8_t daily_volume_screen(void)
{
    vol_rec rec;
    uint8_t i, k, y, key, new_screen;
    uint16_t timeout_count;
    int16_t rec_no;

    timeout_count = 0;                     // 重置不活动超时计数器
    k = 26;                                // 字体大小
    rec_no = ReadNV16(DLVD_VOL_HEAD);      // 读取最后一条记录位置

    // 验证记录号有效性
    if ((rec_no == 0) || (rec_no > DLVD_VOL_MAX_NO_OF)) {
        rec_no = 1;
    }

    do {
        new_screen = 0;

        begin_screen(default_config);
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);   // 设置文本颜色

        // 显示15条每日记录
        for (i = 0; i < 15; i++) {
            get_vol_rec(rec_no, &rec);        // 获取记录数据
            rec_no--;

            // 循环缓冲区处理
            if (rec_no < 1) {
                rec_no = DLVD_VOL_MAX_NO_OF;
            }

            y = 9 + i * 17;                  // 计算行位置

            if (rec.food_vol == 0xffff) {    // 空记录显示
                EVE_CMD_TEXT(5, y, k, 0, "---------------");
            } else {
                // 显示食物体积(F)
                EVE_CMD_TEXT(5, y, k, 0, "F");
                EVE_CMD_NUMBER(55, y, k, EVE_OPT_RIGHTX, rec.food_vol);

                // 显示水体积(W)
                EVE_CMD_TEXT(70, y, k, 0, "W");
                EVE_CMD_NUMBER(124, y, k, EVE_OPT_RIGHTX, rec.water_vol);

                // 显示日期(mm/dd/yy)
                EVE_CMD_NUMBER(152, y, k, 512 | 2, rec.months);
                EVE_CMD_TEXT(166, y, k, 512, "/");
                EVE_CMD_NUMBER(180, y, k, 512 | 2, rec.days);
                EVE_CMD_TEXT(194, y, k, 512, "/");
                EVE_CMD_NUMBER(208, y, k, 512, rec.years);
            }
        }

        // 绘制右侧箭头按钮(退出功能)
        draw_arrow_button_touch(RIGHT_ARROW_TAG, 2, keyright0_offset, 410, 112);

        // 绘制上箭头按钮(上翻页)
        draw_arrow_button_touch(UP_ARROW_TAG, 3, keyup0_offset, 410, 30);

        // 绘制下箭头按钮(下翻页)
        draw_arrow_button_touch(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194);

        end_screen();
        // 处理用户输入
        do {
            key = read_all_keys(&timeout_count);

            // 下箭头按钮处理
            if (key == (DOWN_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (DOWN_ARROW_RELEASED)) {
                new_screen = 1;  // 标记需要刷新屏幕
            }

            // 上箭头按钮处理
            if (key == (UP_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (UP_ARROW_RELEASED)) {
                // 计算上翻页后的记录位置
                rec_no = rec_no + 2 * DLVD_VOL_NO_OF_LINES;
                if (rec_no > DLVD_VOL_MAX_NO_OF) {
                    rec_no -= DLVD_VOL_MAX_NO_OF;
                }
                new_screen = 1;  // 标记需要刷新屏幕
            }

            // 右箭头按钮处理(退出)
            if (key == (RIGHT_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (RIGHT_ARROW_RELEASED)) {
                return NORMAL_EXIT;  // 正常退出
            }

            // 超时检查
            if (timeout_count > EDIT_SCREENS_TIMEOUT) {
                return INACTIVITY_EXIT;  // 超时退出
            }

            // 50ms延迟
            delay_50ms(); // 等待50ms
        } while (!new_screen);
    } while(1);
}


/**************************************
 * cal_rec_screen()
 * @detail 显示校准记录历史，每屏15条
 * 使用上下箭头翻页，右箭头退出。最多保存90条记录
 */
uint8_t cal_rec_screen(void)
{
    cal_rec rec;
    uint8_t i, k, y, key, new_screen;
    uint16_t timeout_count;
    int16_t rec_no;

    timeout_count = 0;
    k = 26;
    rec_no = ReadNV16(CAL_REC_HEAD);  // 读取最后一条校准记录位置

    // 验证记录号有效性
    if ((rec_no == 0) || (rec_no > CAL_REC_MAX_NO_OF)) {
        rec_no = 1;
    }

    do {
        new_screen = 0;

        begin_screen(default_config);
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);

        // 显示15条校准记录
        for (i = 0; i < 15; i++) {
            get_cal_rec(rec_no, &rec);  // 获取校准记录
            rec_no--;

            // 循环缓冲区处理
            if (rec_no < 1) {
                rec_no = CAL_REC_MAX_NO_OF;
            }

            y = 9 + i * 17;  // 计算行位置

            if (rec.dac_cal == 0xffff) {  // 空记录显示
                EVE_CMD_TEXT(5, y, k, 0, "---------------");
            } else {
                // 显示DAC校准值
                EVE_CMD_TEXT(5, y, k, 0, "DAC");
                EVE_CMD_NUMBER(75, y, k, EVE_OPT_RIGHTX | 4, rec.dac_cal);

                // 显示PD2校准值
                EVE_CMD_TEXT(90, y, k, 0 | 4, "PD2");
                EVE_CMD_NUMBER(159, y, k, EVE_OPT_RIGHTX, rec.pd2_cal);

                // 显示校准日期(mm/dd/yy)
                EVE_CMD_NUMBER(187, y, k, 512 | 2, rec.months);
                EVE_CMD_TEXT(201, y, k, 512, "/");
                EVE_CMD_NUMBER(215, y, k, 512 | 2, rec.days);
                EVE_CMD_TEXT(229, y, k, 512, "/");
                EVE_CMD_NUMBER(243, y, k, 512, rec.years);
            }
        }

        // 绘制控制按钮(与daily_volume_screen相同)
        draw_arrow_button_touch(RIGHT_ARROW_TAG, 2, keyright0_offset, 410, 112);
        draw_arrow_button_touch(UP_ARROW_TAG, 3, keyup0_offset, 410, 30);
        draw_arrow_button_touch(DOWN_ARROW_TAG, 4, keydown0_offset, 410, 194);

        end_screen();

        // 处理用户输入(逻辑与daily_volume_screen相同)
        do {
            key = read_all_keys(&timeout_count);

            if (key == (DOWN_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (DOWN_ARROW_RELEASED)) {
                new_screen = 1;
            }

            if (key == (UP_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (UP_ARROW_RELEASED)) {
                rec_no = rec_no + 2 * CAL_REC_NO_OF_LINES;
                if (rec_no > CAL_REC_MAX_NO_OF) {
                    rec_no -= CAL_REC_MAX_NO_OF;
                }
                new_screen = 1;
            }

            if (key == (RIGHT_ARROW_PRESSED)) {
                key_click();
            }
            if (key == (RIGHT_ARROW_RELEASED)) {
                return NORMAL_EXIT;
            }

            if (timeout_count > EDIT_SCREENS_TIMEOUT) {
                return INACTIVITY_EXIT;
            }

            delay_50ms(); // 等待50ms
        } while (!new_screen);
    } while(1);
}

/**************************************
 * draw_arrow_button()
 * @detail 绘制箭头按钮（通用函数）
 * @param tag 按钮标签
 * @param handle 位图句柄
 * @param bitmap_offset 位图偏移量
 * @param x 按钮x坐标
 * @param y 按钮y坐标
 */
static void draw_arrow_button_touch(uint8_t tag, uint8_t handle, uint32_t bitmap_offset, uint16_t x, uint16_t y)
{
    EVE_TAG(tag);
    EVE_BITMAP_HANDLE(handle);
    EVE_BITMAP_SOURCE(bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 6, 48);
    EVE_BITMAP_SIZE(0, 0, 0, 48, 48);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, handle, 0);

    // 增加触摸区域（绘制不可见矩形）
    EVE_COLOR_MASK(0, 0, 0, 0);
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2F(x - 20, y - 12);
    EVE_VERTEX2F(x + 68, y + 60);
    EVE_COLOR_MASK(1, 1, 1, 1);
}

/**************************************
 * set_time_task()
 * @detail 设置RTC时间界面，可调整时、分、日、月、年
 * 包含保存、复位和退出功能
 */
uint8_t set_time_task(void)
{
    chip_time timo;
    const char txt_month[][12] = {
        "jan", "feb", "mar", "apr", "may", "jun",
        "jul", "aug", "sep", "oct", "nov", "dec"
    };

    uint8_t key, key_delay = 0;
    uint8_t num;
    uint8_t tim_minute;
    uint8_t tim_hour;
    uint8_t tim_day;
    uint8_t tim_month;
    uint8_t am_pm;
    uint16_t tim_year;
    HAL_StatusTypeDef ret;
    uint8_t buf[5];
    uint16_t timeout_count = 0;
    uint8_t edit_flag = 0;  // 编辑标志

    // 初始化时间变量
    get_time(&timo);
    tim_year = timo.years;
    tim_month = timo.months;
    tim_day = timo.days;
    tim_hour = timo.hours;
    tim_minute = timo.minutes;
    am_pm = timo.period;

    do {
    	begin_screen(default_config);

        // 设置文本颜色(编辑状态为绿色)
        EVE_COLOR_RGB_24(edit_flag ? GREEN_COLOR_24bit : HIGH_VIS_TEXT_COLOR_24bit);

        // 显示时间(HH:MM [am/pm])
        num = tim_hour;
        EVE_CMD_NUMBER(55, 75, 31, 2 | EVE_OPT_CENTERX, num);
        EVE_CMD_TEXT(88, 75, 31, 2 | EVE_OPT_CENTERX, ":");

        num = tim_minute;
        EVE_CMD_NUMBER(120, 75, 31, 2 | EVE_OPT_CENTERX, num);

        if (am_pm) {
            EVE_CMD_TEXT(180, 75, 31, EVE_OPT_CENTERX, "pm");
        } else {
            EVE_CMD_TEXT(180, 75, 31, EVE_OPT_CENTERX, "am");
        }

        // 显示日期(DD MMM YYYY)
        num = tim_day;
        EVE_CMD_NUMBER(245, 75, 31, EVE_OPT_CENTERX, num);

        num = tim_month;
        EVE_CMD_TEXT(315, 75, 31, EVE_OPT_CENTERX, txt_month[num-1]);

        num = tim_year;
        EVE_CMD_NUMBER(400, 75, 31, 4 | EVE_OPT_CENTERX, 2000 + num);

        // 设置按钮颜色
        EVE_COLOR_RGB(BLACK_COLOR);
        EVE_CMD_FGCOLOR(HIGH_VIS_TEXT_COLOR_24bit);

        // 绘制所有控制按钮
        // 小时+/-按钮
        draw_text_button(35, 20, 40, 40, "+",  HOUR_PLUS_TAG,  28);
        draw_text_button(35, 140, 40, 40, "-",  HOUR_MINUS_TAG,  28);


        // 分钟+/-按钮
        draw_text_button(100, 20, 40, 40, "+",  MINUTE_PLUS_TAG,  28);
        draw_text_button(100, 140, 40, 40, "-",  MINUTE_MINUS_TAG,  28);

        // 日+/-按钮
        draw_text_button(230, 20, 40, 40, "+",  DAY_PLUS_TAG,  28);
        draw_text_button(230, 140, 40, 40, "-",  DAY_MINUS_TAG,  28);

        // 月+/-按钮
        draw_text_button(295, 20, 40, 40, "+",  MONTH_PLUS_TAG,  28);
        draw_text_button(295, 140, 40, 40, "-",  MONTH_MINUS_TAG,  28);

        // 年+/-按钮
        draw_text_button(380, 20, 40, 40, "+",  YEAR_PLUS_TAG,  28);
        draw_text_button(380, 140, 40, 40, "-",  YEAR_MINUS_TAG,  28);

        // 功能按钮
        draw_text_button(35, 220, 40, 40, "R",  RESET_TAG,  28); // 复位
        draw_text_button(150, 220, 100, 40, "SAVE",  SET_TAG,  28); // 保存
        draw_text_button(320, 220, 100, 40, "EXIT",  EXIT_TAG,  28);  // 退出

        end_screen();

        // 处理用户输入
        key = read_all_keys(&timeout_count);
        if (key == NO_TOUCH) {
            key_delay = 0;
        }

        // 超时检查
        timeout_count++;
        if (timeout_count > EDIT_SCREENS_TIMEOUT) {
            pump_motor_off();
            return HOLD_MODE__TASK;
        }

        // 处理各按钮事件
        // 小时+按钮
        if (key == HOUR_PLUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_hour++;

                if (tim_hour > 12) tim_hour = 1;
                if (tim_hour == 12) {
                    am_pm = !am_pm;  // 切换am/pm
                }
            }
            if (key_delay < 200) key_delay++;
        }

        // 小时-按钮
        if (key == HOUR_MINUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_hour--;

                if (tim_hour < 1) tim_hour = 12;
                if (tim_hour == 11) {
                    am_pm = !am_pm;  // 切换am/pm
                }
            }
            if (key_delay < 200) key_delay++;
        }

        // 分钟+按钮
        if (key == MINUTE_PLUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_minute++;

                if (tim_minute > 59) tim_minute = 0;
            }
            if (key_delay < 200) key_delay++;
        }

        // 分钟-按钮
        if (key == MINUTE_MINUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                if (tim_minute == 0) tim_minute = 59;
                else tim_minute--;
            }
            if (key_delay < 200) key_delay++;
        }

        // 日+按钮
        if (key == DAY_PLUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_day++;

                // 简单日期验证（不考虑月份天数差异）
                if (tim_day > 31) tim_day = 1;
            }
            if (key_delay < 200) key_delay++;
        }

        // 日-按钮
        if (key == DAY_MINUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_day--;

                if (tim_day == 0) tim_day = 31;
            }
            if (key_delay < 200) key_delay++;
        }

        // 月+按钮
        if (key == MONTH_PLUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_month++;

                if (tim_month > 12) tim_month = 1;
            }
            if (key_delay < 200) key_delay++;
        }

        // 月-按钮
        if (key == MONTH_MINUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_month--;

                if (tim_month == 0) tim_month = 12;
            }
            if (key_delay < 200) key_delay++;
        }

        // 年+按钮
        if (key == YEAR_PLUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_year++;

                if (tim_year > 99) tim_year = 23;  // 假设年份范围23-99
            }
            if (key_delay < 200) key_delay++;
        }

        // 年-按钮
        if (key == YEAR_MINUS_TAG) {
            if ((key_delay == 0) || (key_delay > 15)) {
                if (key_delay == 0) key_delay = 1;
                else key_delay = 12;

                edit_flag = 1;
                key_click();
                tim_year--;

                if (tim_year < 23) tim_year = 99;  // 假设年份范围23-99
            }
            if (key_delay < 200) key_delay++;
        }

        // 复位按钮
        if (key == (RESET_TAG | PRESSED_BIT)) {
            key_click();
            // 复位RTC芯片
            buf[0] = 0;       // 寄存器0，control1
            buf[1] = 0x58;     // 复位值58h + bit 1 (02h) 12小时模式
            ret = HAL_I2C_Master_Transmit(&hi2c3, 0xa2, buf, 2, HAL_MAX_DELAY);
            if (ret != HAL_OK) {
                // 错误处理
            }
        }

        // 保存按钮
        if (key == (SET_TAG | PRESSED_BIT)) {
            key_click();
            // 保存时间设置
            timo.years = tim_year;
            timo.months = tim_month;
            timo.days = tim_day;
            timo.hours = tim_hour;
            timo.minutes = tim_minute;
            timo.period = am_pm;
            save_time(&timo);
            edit_flag = 0;  // 重置编辑标志
        }

        // 退出按钮
        if (key == (EXIT_TAG | PRESSED_BIT)) {
            key_click();
        }
        if (key == (EXIT_TAG | RELEASED_BIT)) {
            return HOLD_MODE__TASK;  // 返回保持模式
        }

        // 如果未编辑，则持续更新时间显示
        if (!edit_flag) {
            get_time(&timo);
            tim_year = timo.years;
            tim_month = timo.months;
            tim_day = timo.days;
            tim_hour = timo.hours;
            tim_minute = timo.minutes;
            am_pm = timo.period;
        }

        // 50ms延迟
        delay_50ms(); // 等待50ms
    } while(1);
}


/**************************************
 * show_empty_battery_warning
 * @detail
 * Display PLUG INTO AC POWER NOW" 1 second on 1 second off. Audible sound every 10 seconds
 * 1khz 0.5seconds. If AC power restored, exit return AC_CONNECTED.Timeout after 30 seconds
 * returning EMPTY_BATTERY
 */
uint8_t show_empty_battery_warning(void)
{
    // 添加：立即检查电源连接
    if (ac_connected()) {
        // 延迟一小段时间确认不是误检测
        for (uint8_t i = 0; i < 5; i++) {
            delay_50ms();
            if (!ac_connected()) break;  // 如果断开，继续报警
        }
        if (ac_connected()) {
            return AC_CONNECTED;
        }
    }

	uint8_t two_second_count, time_count, signal_time_count;
	uint8_t screen_off_flag = 0;  // 屏幕关闭标志
//	uint32_t screen_off_start_time = 0;  // 屏幕关闭开始时间

	two_second_count = 0;
	time_count = 0;
	signal_time_count = 5;

    uint32_t warning_start_time = HAL_GetTick();  // 报警开始时间
    uint8_t minutes_remaining = 3;  // 剩余分钟数


 //   uint32_t loop_start_time = 0;
 //   uint32_t loop_end_time = 0;
 //   uint32_t loop_duration = 0;
 //   uint32_t last_print_time = 0;

	while (time_count < EMPTY_BATTERY_TIMEOUT) {

	//	loop_start_time = HAL_GetTick();  // 循环开始时间
		if (two_second_count > 19) {
			two_second_count = 0;
		    if (!screen_off_flag) {  // 屏幕关闭后停止蜂鸣
		      if(signal_time_count == 5 || signal_time_count == 6 ){
		    	  beep_1khz_p5sec(G_alarm_vol);
		      }
		    }
			signal_time_count++;

			time_count++;

            uint32_t elapsed_seconds = (HAL_GetTick() - warning_start_time) / 1000;
            minutes_remaining = 3 - (elapsed_seconds / 60);
            if (minutes_remaining > 3) minutes_remaining = 0;
            // 检查是否达到3分钟
            if (minutes_remaining == 0 || minutes_remaining > 3) {
                return EMPTY_BATTERY;  // 3分钟到，退出
            }
	        // 在循环中也保持快速检查
	        if (ac_connected()) {
	            return AC_CONNECTED;
	        }
		}
		two_second_count++;

    // ====== 原有显示逻辑 ======
    if (!screen_off_flag) {  // 只在屏幕未关闭时显示

    	// 计算剩余分钟数（3分钟倒计时）
  //          uint8_t minutes_remaining = 3 - (time_count / 60);
  //          if (minutes_remaining > 3) minutes_remaining = 0;

            // 2Hz闪烁，50%占空比
            // 前2个计数亮，后2个计数灭
            uint8_t blink_on = (two_second_count % 10 < 5) ? 1 : 0;  // 10×50ms=500ms
            uint16_t countdown_width = (minutes_remaining * 140) / 3;

            begin_screen(default_config);
            EVE_CMD_ROMFONT(1, 32);

            EVE_CMD_DLSTART();

            // 1. 背景颜色（参考网页的浅粉色渐变）
            EVE_CLEAR_COLOR_RGB(255, 248, 248);  // #FFF8F8
            EVE_CLEAR(1, 1, 1);

            // 2. 顶部红色警示条（闪烁效果）
            if (blink_on) {
                EVE_COLOR_RGB(211, 47, 47);  // #D32F2F
            } else {
                EVE_COLOR_RGB(255, 248, 248);  // 与背景色相同，实现"不显示"效果
            }
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(0, 0);
            EVE_VERTEX2F(480 * 16, 6 * 16);  // 高度6像素

            EVE_COLOR_RGB(BLACK_COLOR);
            // "Time remaining:"文字
            EVE_CMD_TEXT(50, 170, 24, 0, "Time remaining:");

            // 横条背景（稍深的灰色）
            EVE_COLOR_RGB(100, 100, 100);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(240 * 16, 180 * 16);
            EVE_VERTEX2F(380 * 16, 190 * 16);

            // 横条前景（红色）
            EVE_COLOR_RGB(RED_COLOR);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(240 * 16, 180 * 16);
            EVE_VERTEX2F((240 + countdown_width) * 16, 190 * 16);

            // 分钟数字
            EVE_CMD_ROMFONT(1, 26);
            EVE_CMD_NUMBER(392, 178, 1, 0, minutes_remaining);

            // 单位（根据分钟数使用单数或复数）
            if (minutes_remaining == 1) {
                EVE_CMD_TEXT(410, 175, 22, 0, "minute");
            } else {
                EVE_CMD_TEXT(410, 175, 22, 0, "minutes");
            }

            // 4. 主白色卡片（网页设计风格）
            EVE_COLOR_RGB(255, 255, 255);  // 白色卡片
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_LINE_WIDTH(3 * 16);  // 红色边框
            EVE_VERTEX2F(40 * 16, 40 * 16);     // 从y=50开始（在倒计时下方）
            EVE_VERTEX2F(440 * 16, 140 * 16);   // 调整高度

            // 5. 卡片红色标题区域（也实现闪烁）
            if (blink_on) {
                EVE_COLOR_RGB(244, 67, 54);  // #F44336
            } else {
                EVE_COLOR_RGB(255, 255, 255);  // 白色（与卡片背景相同）
            }
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(40 * 16, 30 * 16);
            EVE_VERTEX2F(440 * 16, 60 * 16);

            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(0, 266 * 16);  // y=266
            EVE_VERTEX2F(480 * 16, 272 * 16);  // y=272

            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(0, 6 * 16);  // y=266
            EVE_VERTEX2F(6* 16, 272 * 16);  // y=272

            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(474* 16, 6 * 16);  // y=266
            EVE_VERTEX2F(480* 16, 272 * 16);  // y=272

            // 6. 警告图标（只在闪烁时显示）
            if (blink_on) {
                // 第一个警告图标
     //           draw_warning_icon(180, 45, 25, 20, WHITE_COLOR_24bit);  // 白色图标
                // 第二个警告图标
            	draw_cached_error_warning_icon(240, 45, 25, 20, WHITE_COLOR_24bit);  // 白色图标
                // 第三个警告图标
     //           draw_warning_icon(300, 45, 25, 20, WHITE_COLOR_24bit);  // 白色图标
            }
            else{
                // 第一个警告图标
       //         draw_warning_icon(180, 45, 25, 20, RED_COLOR_24bit);  // 白色图标
                // 第二个警告图标
            	draw_cached_error_warning_icon(240, 45, 25, 20, RED_COLOR_24bit);  // 白色图标
                // 第三个警告图标
        //        draw_warning_icon(300, 45, 25, 20, RED_COLOR_24bit);  // 白色图标
            }
            // 7. 卡片标题文字（闪烁时白色，不闪烁时红色）
            if (blink_on) {
                EVE_COLOR_RGB(255, 255, 255);  // 闪烁时白色
            } else {
                EVE_COLOR_RGB(244, 67, 54);    // 不闪烁时红色
            }

            // 8. 原代码的警告图标和文字（保持常亮，不闪烁）
     //       draw_warning_icon(240, 105, 40, 35, RED_COLOR_24bit);

            // 9. 原代码的主警告文字（保持常亮）
            EVE_COLOR_RGB(RED_COLOR);
            EVE_CMD_TEXT(240, 95, 31, EVE_OPT_CENTERX, "OUT OF BATTERY");

            // 10. 提示文字 - 字体减小
            EVE_COLOR_RGB(BLACK_COLOR);
            EVE_CMD_TEXT(320, 210, 23, EVE_OPT_CENTERX, "Screen will turn off");
            EVE_CMD_TEXT(320, 235, 23, EVE_OPT_CENTERX, "Connect charger to continue");


            EVE_DISPLAY();
            EVE_CMD_SWAP();
            end_screen();

    }

    // ====== 蜂鸣器逻辑 ======
    if (signal_time_count > 15) {
    	signal_time_count = 5;
      }

  //    two_second_count = 10;

    // ====== 检查充电器连接 ======
    if (ac_connected()) {
      return AC_CONNECTED;
    }

    HAL_Delay(10);  // 11ms loop
 //   loop_end_time = HAL_GetTick();  // 循环结束时间
 //   loop_duration = loop_end_time - loop_start_time;
  }


  return EMPTY_BATTERY;  // 原有逻辑：超时关机
}/** End show_empty_battery_warning()  **/

// =============================================================================
// 函数名：initialize_error_warning_cache
// 功能：初始化错误消息警告图标缓存
// 参数：
//   center_x - 警告图标中心X坐标
//   center_y - 警告图标中心Y坐标
//   triangle_side - 三角形边长
//   triangle_height - 三角形高度
//   color24 - 24位颜色值
// 返回值：无
// =============================================================================
static void initialize_error_warning_cache(uint16_t center_x, uint16_t center_y,
                                          uint16_t triangle_side, uint16_t triangle_height
                                          )
{
    error_warning_cache.is_valid = 0;

    // 1. 保存参考参数
    error_warning_cache.ref_center_x = center_x;
    error_warning_cache.ref_center_y = center_y;
    error_warning_cache.ref_triangle_side = triangle_side;
    error_warning_cache.ref_triangle_height = triangle_height;


    // 2. 计算三角形顶点坐标
    error_warning_cache.tri_center_x = center_x;
    error_warning_cache.tri_bottom_y = center_y + triangle_side/2;
    error_warning_cache.tri_top_x = center_x;
    error_warning_cache.tri_top_y = error_warning_cache.tri_bottom_y - triangle_height;
    error_warning_cache.tri_left_x = center_x - triangle_side/2;
    error_warning_cache.tri_left_y = error_warning_cache.tri_bottom_y;
    error_warning_cache.tri_right_x = center_x + triangle_side/2;
    error_warning_cache.tri_right_y = error_warning_cache.tri_bottom_y;

    // 3. 计算圆弧参数
    error_warning_cache.arc_radius = triangle_side / 2 + 2;
    error_warning_cache.arc_center_y = error_warning_cache.tri_bottom_y - triangle_height/3;

    // 4. 计算圆弧点坐标（31个点）
    for (uint8_t i = 0; i <= 30; i++) {
        float angle = 10.0f - 75.0f * i / 30;
        float rad = angle * 3.14159f / 180.0f;
        error_warning_cache.arc_points[i][0] = center_x + (int16_t)(error_warning_cache.arc_radius * cosf(rad));
        error_warning_cache.arc_points[i][1] = error_warning_cache.arc_center_y + (int16_t)(error_warning_cache.arc_radius * sinf(rad));
    }

    // 5. 计算感叹号坐标 - 改为直线端点方式！
    uint16_t exclamation_spacing = triangle_side * 12 / 40;
    uint16_t first_exclamation_x = error_warning_cache.tri_right_x + triangle_side * 10 / 40;

    for (uint8_t ex_num = 0; ex_num < 3; ex_num++) {
        error_warning_cache.exclamation_x[ex_num] = first_exclamation_x + ex_num * exclamation_spacing;
    }

    // 预计算直线端点坐标（只需要顶部和底部！）
    error_warning_cache.drop_top_y = error_warning_cache.tri_top_y + triangle_height * 8 / 35;
    error_warning_cache.line_bottom_y = error_warning_cache.tri_bottom_y - triangle_height * 10 / 35;
    error_warning_cache.dot_y = error_warning_cache.tri_bottom_y - triangle_height * 3 / 35;


    // 6. 标记缓存有效
    error_warning_cache.is_valid = 1;
}
// =============================================================================
// 函数名：update_error_warning_cache
// 功能：更新错误消息警告图标缓存状态
// 参数：
//   center_x - 警告图标中心X坐标
//   center_y - 警告图标中心Y坐标
//   triangle_side - 三角形边长
//   triangle_height - 三角形高度
//   color24 - 24位颜色值
// 返回值：无
// 说明：
//   1. 检查缓存是否有效，无效则初始化
//   2. 比较新参数与缓存参数是否一致
//   3. 参数变化时标记缓存无效并重新初始化
//   4. 参数未变化时直接使用现有缓存
// =============================================================================
static void update_error_warning_cache(uint16_t center_x, uint16_t center_y,
                                      uint16_t triangle_side, uint16_t triangle_height
                                      )
{
    if (!error_warning_cache.is_valid) {
        initialize_error_warning_cache(center_x, center_y, triangle_side, triangle_height);
        return;
    }

    if (error_warning_cache.ref_center_x == center_x &&
        error_warning_cache.ref_center_y == center_y &&
        error_warning_cache.ref_triangle_side == triangle_side &&
        error_warning_cache.ref_triangle_height == triangle_height
        ) {
        return;
    }

    error_warning_cache.is_valid = 0;
    initialize_error_warning_cache(center_x, center_y, triangle_side, triangle_height);
}
// =============================================================================
// 函数名：draw_cached_error_warning_icon
// 功能：绘制带缓存的错误消息警告图标
// 参数：
//   center_x - 警告图标中心X坐标
//   center_y - 警告图标中心Y坐标
//   triangle_side - 三角形边长
//   triangle_height - 三角形高度
//   color24 - 24位颜色值
// 返回值：无
// =============================================================================
void draw_cached_error_warning_icon(uint16_t center_x, uint16_t center_y,
                                   uint16_t triangle_side, uint16_t triangle_height,
                                   uint32_t color24)
{
    uint8_t r = (color24 >> 16) & 0xFF;
    uint8_t g = (color24 >> 8) & 0xFF;
    uint8_t b = color24 & 0xFF;

    // 1. 更新缓存
    update_error_warning_cache(center_x, center_y, triangle_side, triangle_height);

    // 2. 绘制三角形
    EVE_VERTEX_FORMAT(0);
    EVE_COLOR_RGB(r, g, b);
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 1);

    EVE_VERTEX2F(error_warning_cache.tri_top_x, error_warning_cache.tri_top_y);
    EVE_VERTEX2F(error_warning_cache.tri_left_x, error_warning_cache.tri_left_y);
    EVE_VERTEX2F(error_warning_cache.tri_right_x, error_warning_cache.tri_right_y);
    EVE_VERTEX2F(error_warning_cache.tri_top_x, error_warning_cache.tri_top_y);

 //   EVE_END();

    // 3. 绘制圆弧
 //   EVE_VERTEX_FORMAT(0);
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 0.8);

    for (uint8_t i = 0; i <= 30; i++) {
        EVE_VERTEX2F(error_warning_cache.arc_points[i][0], error_warning_cache.arc_points[i][1]);
    }

  //  EVE_END();

    // 4. 绘制三个感叹号
 //   EVE_VERTEX_FORMAT(0);

    // 绘制三个感叹号 - 每条线只需要2个顶点！
    for (uint8_t ex_num = 0; ex_num < 3; ex_num++) {
        uint16_t current_x = error_warning_cache.exclamation_x[ex_num];

        // 绘制竖线 - 只需要2个点！
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
        EVE_LINE_WIDTH(16 * 1);  // 固定线宽
        EVE_VERTEX2F(current_x, error_warning_cache.drop_top_y);     // 顶部点
        EVE_VERTEX2F(current_x, error_warning_cache.line_bottom_y);  // 底部点
        //EVE_END();  // 如果需要

        // 绘制点
        EVE_BEGIN(EVE_BEGIN_POINTS);
        EVE_POINT_SIZE(16 * 2.5);
        EVE_VERTEX2F(current_x, error_warning_cache.dot_y);
 //       EVE_END();
    }
}
/*--------- 绘制警告图标函数 ---------*/
/*
 * 功能：在指定位置绘制三角形警告图标，包含圆弧和感叹号
 * 参数：
 *   center_x - 图标中心点的X坐标（像素）
 *   center_y - 图标中心点的Y坐标（像素）
 * 说明：
 *   1. 绘制一个红色等边三角形作为警告符号主体
 *   2. 在三角形内部绘制圆弧，增强视觉效果
 *   3. 在三角形右侧绘制感叹号，表示紧急警告
 *   4. 所有图形使用线条绘制，适合低功耗显示
 */

void draw_warning_icon(uint16_t center_x, uint16_t center_y, uint16_t triangle_side, uint16_t triangle_height,uint32_t color24)
{
    uint8_t r = (color24 >> 16) & 0xFF;
    uint8_t g = (color24 >> 8) & 0xFF;
    uint8_t b = color24 & 0xFF;
	/*--------- 第一部分：绘制等边三角形 ---------*/
    // 设置图形绘制格式和颜色
    EVE_VERTEX_FORMAT(0);                // 使用默认顶点格式
    EVE_COLOR_RGB(r,g,b);           // 设置为红色（警告色）
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);     // 开始绘制连续线条
    EVE_LINE_WIDTH(16 * 2);              // 设置线条宽度为2像素（加粗）

    // 计算三角形三个顶点的坐标：
    // 三角形放置策略：以center_x为中心，center_y为三角形底部中心
    uint16_t tri_center_x = center_x;                    // 中心点X坐标
    uint16_t tri_bottom_y = center_y + triangle_side/2;  // 底部Y坐标（在中心点下方）
    uint16_t tri_top_x = tri_center_x;                   // 顶点X坐标（与中心对齐）
    uint16_t tri_top_y = tri_bottom_y - triangle_height; // 顶点Y坐标（在底部上方）
    uint16_t tri_left_x = tri_center_x - triangle_side/2;  // 左下顶点X坐标
    uint16_t tri_left_y = tri_bottom_y;                  // 左下顶点Y坐标
    uint16_t tri_right_x = tri_center_x + triangle_side/2; // 右下顶点X坐标
    uint16_t tri_right_y = tri_bottom_y;                 // 右下顶点Y坐标

    // 绘制三角形轮廓（从顶点开始，顺时针绘制）
    EVE_VERTEX2F(tri_top_x, tri_top_y);      // 1. 顶点
    EVE_VERTEX2F(tri_left_x, tri_left_y);    // 2. 左下角
    EVE_VERTEX2F(tri_right_x, tri_right_y);  // 3. 右下角
    EVE_VERTEX2F(tri_top_x, tri_top_y);      // 4. 回到顶点，闭合图形

    EVE_END(); // 结束三角形绘制

    /*--------- 第二部分：绘制内部圆弧 ---------*/
    // 设置新的绘制状态
    EVE_VERTEX_FORMAT(0);                // 保持顶点格式
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);     // 开始新的线条绘制
    EVE_LINE_WIDTH(16 * 1.5);            // 稍粗的线条（1.5像素）

    // 圆弧参数计算
    uint16_t arc_radius = triangle_side / 2 + 2;  // 圆弧半径 = 三角形半径+2像素
    uint16_t correct_center_y = tri_bottom_y - triangle_height/3; // 圆弧中心Y坐标（在三角形内部）

    // 绘制圆弧：从右上到左上的弧线
    for (uint8_t i = 0; i <= 30; i++) {
        // 计算当前点的角度（从10度到-65度，共75度跨度）
        float angle = 10.0f - 75.0f * i / 30;
        // 角度转弧度
        float rad = angle * 3.14159f / 180.0f;
        // 计算圆弧上的点坐标
        uint16_t point_x = tri_center_x + (int16_t)(arc_radius * cosf(rad));
        uint16_t point_y = correct_center_y + (int16_t)(arc_radius * sinf(rad));
        // 绘制该点
        EVE_VERTEX2F(point_x, point_y);
    }

    EVE_END(); // 结束圆弧绘制

    /*--------- 第三部分：绘制3个感叹号（在三角形右侧，不是在内部） ---------*/
    EVE_VERTEX_FORMAT(0);                // 保持顶点格式

    // 感叹号参数 - 基于三角形尺寸计算相关值
    uint16_t exclamation_spacing = triangle_side * 12 / 40;   // 感叹号之间的间距（按比例缩放）
    uint16_t first_exclamation_x = tri_right_x + triangle_side * 10 / 40;  // 第一个感叹号X坐标（在三角形右侧按比例缩放）

    // 修改1：调整Y坐标，让竖线和点分开
    uint16_t drop_top_y = tri_top_y + triangle_height * 8 / 35;      // 感叹号顶部Y坐标
    uint16_t line_bottom_y = tri_bottom_y - triangle_height * 10 / 35; // 竖线底部Y坐标
    uint16_t dot_y = tri_bottom_y - triangle_height * 3 / 35;        // 点的Y坐标

    // 绘制三个感叹号
    for (uint8_t ex_num = 0; ex_num < 3; ex_num++) {
        // 计算当前感叹号的X坐标
        uint16_t current_exclamation_x = first_exclamation_x + ex_num * exclamation_spacing;

        // 开始绘制当前感叹号的竖线
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);

        // 绘制感叹号主体（从粗到细的竖线）
        for (uint8_t i = 0; i <= 16; i++) {
            // 计算当前点在线条上的位置比例
            float progress = (float)i / 16;
            // 计算当前Y坐标（从上到下）
            uint16_t current_y = drop_top_y + (uint16_t)((line_bottom_y - drop_top_y) * progress);
            // 计算线条宽度（从粗到细）
            float width_progress = 1.0f - progress;  // 1.0表示顶部，0.0表示底部
            uint8_t line_width = 1 + (uint8_t)(2 * width_progress);  // 宽度从3像素到1像素
            EVE_LINE_WIDTH(16 * line_width);         // 设置动态宽度
            EVE_VERTEX2F(current_exclamation_x, current_y);  // 绘制当前点
        }

        EVE_END(); // 结束当前感叹号竖线绘制

        // 绘制感叹号底部的点
        EVE_BEGIN(EVE_BEGIN_POINTS);         // 切换到点绘制模式
        EVE_POINT_SIZE(16 * 3);              // 设置点大小为3像素
        EVE_VERTEX2F(current_exclamation_x, dot_y);  // 在竖线下方绘制点
        EVE_END(); // 结束点绘制
    }
}

// =============================================================================
// 函数名：show_error_message
// 功能：显示全屏错误消息，支持高优先级报警闪烁和静音功能
// 参数：
//   timeout - 超时时间（单位：10秒周期），0表示无限循环
//   strline1 - 第一行错误信息文本
//   strline2 - 第二行错误信息文本（可为空）
// 返回值：无
// 说明：
//   1. 显示周期为10秒，屏幕刷新率2Hz（每0.5秒一次）
//   2. 报警闪烁规格：
//      - 频率：2Hz（每0.5秒闪烁一次）
//      - 占空比：50%（亮250ms，灭250ms）
//      - 红色：#D32F2F（亮），背景色：#FFF8F8（灭）
//   3. 蜂鸣逻辑：前2秒每秒响一次（step=0和step=2时），每次0.5秒
//   4. 静音功能：点击右下角喇叭图标可静音2分钟
//      - 静音期间显示铃铛图标和剩余时间
//      - 再次点击喇叭图标取消静音
//      - 2分钟超时自动恢复蜂鸣
//   5. 按键退出：运行/停止键或电源键可立即退出
//   6. 字体自适应：根据文本长度自动选择合适字号
// 触摸标签：
//   - ERROR_MUTE_TAG：喇叭图标触摸标签
//   - beep_1khz_p5sec()/beep_1khz_150ms()：蜂鸣控制
//   - read_keypad()：物理按键读取
//   - eve_read_tag()：触摸标签读取
//   - ac_connected()：充电状态检测
// =============================================================================
#if USE_HOLD_MODE_NEW_SCREEN
void show_error_message(uint8_t timeout, const char *strline1, const char *strline2)
{
    uint16_t i;
    uint8_t batt_status;
    (void)batt_status;
    uint8_t len, len2;
    uint16_t font;

    static uint16_t second_counter = 0;
    static uint8_t triple_beep_flag = 0;
    static uint8_t triple_beep_i_count = 0;
    static uint16_t mute_duration_counter = 0;
    static uint8_t is_muted = 0;

    uint8_t tim_count = (timeout + 1) * 2;

    font = 29;
    len = strlen(strline1);
    len2 = strlen(strline2);
    if (len2 > len) len = len2;

    if (len < 28) font = 29;
    if (len < 20) font = 30;
    if (len < 16) font = 31;


    while (tim_count > 0) {
        if (timeout != 0) tim_count--;

        batt_status = power_test();

        if (second_counter == 0 && !is_muted) {
            triple_beep_flag = 1;
            triple_beep_i_count = 0;
        }

        for (i = 0; i < 20; i++) {
            begin_screen(default_config);

            // ========== 简洁界面 ==========

            // 顶部红色警示条（闪烁）
            if (i < 10) {
                EVE_COLOR_RGB(BLACK_COLOR);
            } else {
                EVE_COLOR_RGB(RED_COLOR);
            }
            EVE_COLOR_RGB(RED_COLOR);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(0, 0, 0, 0);
            EVE_VERTEX2II(480, 60, 0, 0);

            EVE_COLOR_RGB(40,40, 40);  // 灰色
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(396, 64, 0, 0);
            EVE_VERTEX2II(480, 272, 0, 0);

            // 警告图标
            if(i<10){
            	EVE_COLOR_RGB(RED_COLOR);
            } else{
            	EVE_COLOR_RGB(WHITE_COLOR);
            }
            draw_alarm3(90, 20);

            // ====== 5. 显示错误文本（黑色常亮）=====
            if (i < 10) {
                EVE_COLOR_RGB(BLACK_COLOR);
            } else {
                EVE_COLOR_RGB(RED_COLOR);
            }
            //EVE_CMD_ROMFONT(1, 31);

            if (strlen(strline2) == 0) {
                EVE_CMD_TEXT(198, 142, font, EVE_OPT_CENTERX, strline1);
            } else {
                EVE_CMD_TEXT(198, 112, font, EVE_OPT_CENTERX, strline1);
                EVE_CMD_TEXT(198, 172, font, EVE_OPT_CENTERX, strline2);
            }

            // 静音按钮（使用 draw_bell / draw_mute）
            EVE_TAG_MASK(1);
            EVE_TAG(ERROR_MUTE_TAG);
            EVE_COLOR_RGB(WHITE_COLOR);
            if (!is_muted) {
                draw_bell(430, 134);
            } else {
                draw_mute(430, 134);
            }
            EVE_TAG_MASK(0);
            /* 电池符号显示 */
            draw_batt_symbol(G_v_battery);
            // 静音倒计时
            if (is_muted && mute_duration_counter > 0) {
                uint16_t remain_sec = (2400 - mute_duration_counter) / 20;
                EVE_CMD_ROMFONT(1, 28);
                EVE_CMD_NUMBER(420, 200, 1, 3, remain_sec);
                EVE_CMD_TEXT(460, 200, 29, 0, "s");
            }

            end_screen();

            // 退出条件
            if (read_keypad() == (RUN_STOP_SWITCH_RELEASED)) return;
            if (read_keypad() == (POWER_SWITCH_RELEASED)) return;

            // 静音按钮处理
            if (eve_read_tag() == ERROR_MUTE_TAG) {
                key_click();

                if (!is_muted) {
                    is_muted = 1;
                    mute_duration_counter = 0;
                    triple_beep_flag = 0;
                    triple_beep_i_count = 0;
                } else {
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;
                    triple_beep_i_count = 0;
                }

                while (eve_read_tag() == ERROR_MUTE_TAG) {
                    delay_50ms();
                }
            }

            // 报警声控制
            if (is_muted) {
                mute_duration_counter++;
                if (mute_duration_counter >= 2400) {
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;
                    triple_beep_i_count = 0;
                    beep_1khz_150ms(0);
                }
            } else {
                mute_duration_counter = 0;
                if (triple_beep_flag) {
                    triple_beep_i_count++;
                    if (triple_beep_i_count == 1) {
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 8) {
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 16) {
                        beep_1khz_150ms(0);
                        triple_beep_flag = 0;
                    }
                }
            }

            delay_50ms();
        }

        second_counter++;
        if (second_counter >= 25) {
            second_counter = 0;
        }
    }
}
#else
void show_error_message(uint8_t timeout, const char *strline1, const char *strline2)
{
    uint8_t sec_count, step, len, len2, tim_count;
    uint16_t font;
    static uint32_t mute_start_time = 0;
    static uint8_t is_muted = 0;

    // 闪烁状态：0=红色显示，1=白色（不显示）
    static uint8_t blink_on = 0;

  //  uint32_t loop_start_time = 0;
  //  uint32_t loop_end_time = 0;
 //   uint32_t loop_duration = 0;


    // 1. 检查静音状态
    if (is_muted) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - mute_start_time > 120000) {
            is_muted = 0;
        }
    }

    // 2. 自适应字体选择
    font = 29;
    len = strlen(strline1);
    len2 = strlen(strline2);
    if (len2 > len) len = len2;

    if (len < 28) font = 30;
    if (len < 20) font = 31;
    if (len < 16) font = 1;

    // 3. 主显示循环
    step = 0;

    tim_count = (timeout + 1)* 2;
    while (tim_count > 0) {
        if (timeout != 0) tim_count--;

  //      if(step == 0){
  //      	loop_start_time = HAL_GetTick();
  //      }
        // 更新闪烁状态（每秒切换）
        blink_on = (step % 2 == 0) ? 1 : 0;  // 偶数秒：红色，奇数秒：白色

        begin_screen(default_config);

        // 1. 背景颜色（参考网页的浅粉色渐变）
        EVE_CLEAR_COLOR_RGB(255, 248, 248);  // #FFF8F8
        EVE_CLEAR(1, 1, 1);

        // 2. 顶部红色警示条（闪烁效果）
        if (blink_on) {
            EVE_COLOR_RGB(211, 47, 47);  // #D32F2F
        } else {
            EVE_COLOR_RGB(255, 248, 248);  // 与背景色相同，实现"不显示"效果
        }
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(0, 0);
        EVE_VERTEX2F(480 * 16, 6 * 16);  // 高度6像素

        // ====== 3. 四边红色边框（闪烁）=====
        if (blink_on) {
            EVE_COLOR_RGB(211, 47, 47);
        } else {
            EVE_COLOR_RGB(255, 248, 248);
        }

        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(40 * 16, 30 * 16);
        EVE_VERTEX2F(440 * 16, 65 * 16);

        // 左边框
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(0, 0);
        EVE_VERTEX2F(6 * 16, 272 * 16);

        // 右边框
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(474 * 16, 0);
        EVE_VERTEX2F(480 * 16, 272 * 16);

        // 下边框
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(0, 266 * 16);
        EVE_VERTEX2F(480 * 16, 272 * 16);

        if (blink_on) {
          //  draw_warning_icon(240, 45, 25, 20, WHITE_COLOR_24bit);  // 白色图标
            draw_cached_error_warning_icon(240, 45, 30, 26, WHITE_COLOR_24bit);
        }
        else{

            //draw_warning_icon(240, 45, 25, 20, RED_COLOR_24bit);  // 白色图标
            draw_cached_error_warning_icon(240, 45, 30, 26, RED_COLOR_24bit);
        }
        // ====== 5. 显示错误文本（黑色常亮）=====
        EVE_COLOR_RGB(0, 0, 0);  // 黑色文字，常亮不闪烁
        EVE_CMD_ROMFONT(1, 32);

        if (strlen(strline2) == 0) {
            EVE_CMD_TEXT(240, 112, font, EVE_OPT_CENTERX, strline1);
        } else {
            EVE_CMD_TEXT(240, 82, font, EVE_OPT_CENTERX, strline1);
            EVE_CMD_TEXT(240, 142, font, EVE_OPT_CENTERX, strline2);
        }

        // ====== 6. 在右下角显示喇叭图标（可点击）=====
        EVE_TAG_MASK(1);
        EVE_TAG(ERROR_MUTE_TAG);

        // 调用已有的喇叭绘制函数
        draw_speaker_icon(420, 220, is_muted);

        // ====== 6. 显示遮挡数据（仅当两个值都大于0时）======
        if (occl_comp_count_test > 0 ||occl_peak_count_test > 0 ) {
            // 可选：添加一个浅色半透明背景框
            EVE_COLOR_RGB(240, 240, 240);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2F(200 * 16, 175 * 16);
            EVE_VERTEX2F(440 * 16, 245 * 16);

            // 显示数据标签和数值
            EVE_COLOR_RGB(0, 0, 0);

            EVE_CMD_TEXT(140, 80, 27, 0, "peak_offset");
            if (occl_comp_peak_offset < 0) {
                EVE_CMD_TEXT(290, 80, 27, EVE_OPT_CENTERX, "-");
                EVE_CMD_NUMBER(310, 80, 27, EVE_OPT_CENTERX, -occl_comp_peak_offset);
            } else {
                EVE_CMD_NUMBER(300, 80, 27, EVE_OPT_CENTERX, occl_comp_peak_offset);
            }
            EVE_CMD_TEXT(140, 165, 27, 0, "comp");
            EVE_CMD_NUMBER(200, 165, 27, EVE_OPT_CENTERX, occl_comp_count_test);
            int y = 190;
               for (int i = 0; i < occl_comp_count_test; i += 2) {
                   // 左边
                   EVE_CMD_NUMBER(40, y, 27, 0, occl_comp_count_test_position[i]);
                   EVE_CMD_NUMBER(120, y, 27, 0, occl_comp_count_test_value[i]);

                   // 右边（如果有）
                   if (i+1 < occl_comp_count_test) {
                       EVE_CMD_NUMBER(250, y, 27, 0, occl_comp_count_test_position[i+1]);
                       EVE_CMD_NUMBER(320, y, 27, 0, occl_comp_count_test_value[i+1]);
                   }
                   y += 20;
               }

            EVE_CMD_TEXT(240, 170, 27, 0, "peak");
            EVE_CMD_NUMBER(320, 170, 27, EVE_OPT_CENTERX, occl_peak_count_test);
        }

        // ====== 7. 如果静音中，显示静音铃铛图标 ======
        if (is_muted) {
            // 调用已有的铃铛绘制函数
            draw_error_mute_bell(450, 220, 8);

            // 显示剩余静音时间
            uint32_t remaining = 120 - (HAL_GetTick() - mute_start_time) / 1000;
            if (remaining > 0) {
                // 分钟数字
                EVE_CMD_ROMFONT(1, 28);
                EVE_CMD_NUMBER(352, 210, 1, 0, remaining);
            } else {
                // 时间到了，恢复铃声
                is_muted = 0;
            }
        }

        EVE_TAG_MASK(0);
        end_screen();

        // ====== 8. 处理蜂鸣逻辑（10秒周期：前2秒各响一次，后8秒静音）=====
        if (!is_muted) {
            if (step == 0 || step == 2 ) {  // 前2秒蜂鸣（现在是4个step）
                //beep_1khz_p5sec(G_alarm_vol);
            	beep_medium_priority_feeding_pump(G_alarm_vol);

            }
        }

        // ====== 9. 按键检测 ======
        sec_count = 0;
        while (sec_count < 5) {  // 10×50ms = 0.5秒
            sec_count++;

            // 9.1 检查物理按键
            uint8_t key = read_keypad();
            if (key == RUN_STOP_SWITCH_RELEASED || key == POWER_SWITCH_RELEASED) {
            	error_bell_cache.is_valid = 0;
            	error_warning_cache.is_valid = 0;
                return;
            }

            // 9.2 检查触摸屏（喇叭图标）- 修改：静音状态下触发是取消静音
            uint8_t tag = eve_read_tag();
            if (tag == ERROR_MUTE_TAG) {
                if (is_muted) {
                    // 如果已经在静音状态，点击则取消静音
                    is_muted = 0;
                    beep_1khz_150ms(1);  // 短提示音确认取消
                } else {
                    // 如果未静音，点击则开始静音2分钟
                    is_muted = 1;
                    mute_start_time = HAL_GetTick();
                    beep_1khz_150ms(1);  // 短提示音确认
                }
            }

            // 9.3 检查静音是否超时
            if (is_muted) {
                uint32_t current_time = HAL_GetTick();
                if (current_time - mute_start_time > 120000) {
                    is_muted = 0;  // 2分钟到了，自动恢复
                }
            }

            delay_50ms();
        }

        // ====== 10. 更新步骤 ======
        step++;
        if (step >= 36) {

 //       	 loop_end_time = HAL_GetTick();  // 循环结束时间
  //      	 loop_duration = loop_end_time - loop_start_time;
        	 step = 0;
        }
    }
}
#endif

void show_error_message_without_alarm(uint8_t timeout, const char *strline1, const char *strline2)
{
    uint16_t i;
    uint8_t batt_status;
    (void)batt_status;

    static uint16_t second_counter = 0;
    static uint8_t triple_beep_flag = 0;
    static uint8_t triple_beep_i_count = 0;
    static uint16_t mute_duration_counter = 0;
    static uint8_t is_muted = 0;

    uint8_t tim_count = (timeout + 1) * 2;

    while (tim_count > 0) {
        if (timeout != 0) tim_count--;

        batt_status = power_test();

        if (second_counter == 0 && !is_muted) {
            triple_beep_flag = 1;
            triple_beep_i_count = 0;
        }

        for (i = 0; i < 20; i++) {
            begin_screen(default_config);

            // ========== 简洁界面 ==========

            // 顶部红色警示条（闪烁）
            if (i < 10) {
                EVE_COLOR_RGB(BLACK_COLOR);
            } else {
                EVE_COLOR_RGB(RED_COLOR);
            }
            EVE_COLOR_RGB(RED_COLOR);
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(0, 0, 0, 0);
            EVE_VERTEX2II(480, 60, 0, 0);

            EVE_COLOR_RGB(40,40, 40);  // 灰色
            EVE_BEGIN(EVE_BEGIN_RECTS);
            EVE_VERTEX2II(396, 64, 0, 0);
            EVE_VERTEX2II(480, 272, 0, 0);

            // 警告图标
            if(i<10){
            	EVE_COLOR_RGB(RED_COLOR);
            } else{
            	EVE_COLOR_RGB(WHITE_COLOR);
            }

            // ====== 5. 显示错误文本（黑色常亮）=====
            if (i < 10) {
                EVE_COLOR_RGB(BLACK_COLOR);
            } else {
                EVE_COLOR_RGB(RED_COLOR);
            }
            EVE_CMD_ROMFONT(1, 31);

            if (strlen(strline2) == 0) {
                EVE_CMD_TEXT(198, 142, 1, EVE_OPT_CENTERX, strline1);
            } else {
                EVE_CMD_TEXT(198, 112, 1, EVE_OPT_CENTERX, strline1);
                EVE_CMD_TEXT(198, 172, 1, EVE_OPT_CENTERX, strline2);
            }

            // 静音按钮（使用 draw_bell / draw_mute）
            EVE_TAG_MASK(1);
            EVE_TAG(ERROR_MUTE_TAG);
            EVE_COLOR_RGB(WHITE_COLOR);
            if (!is_muted) {
                draw_bell(430, 134);
            } else {
                draw_mute(430, 134);
            }
            EVE_TAG_MASK(0);

            // 静音倒计时
            if (is_muted && mute_duration_counter > 0) {
                uint16_t remain_sec = (2400 - mute_duration_counter) / 20;
                EVE_CMD_ROMFONT(1, 28);
                EVE_CMD_NUMBER(420, 200, 1, 3, remain_sec);
                EVE_CMD_TEXT(460, 200, 29, 0, "s");
            }

            end_screen();

            // 退出条件
            if (read_keypad() == (RUN_STOP_SWITCH_RELEASED)) return;
            if (read_keypad() == (POWER_SWITCH_RELEASED)) return;

            // 静音按钮处理
            if (eve_read_tag() == ERROR_MUTE_TAG) {
                key_click();

                if (!is_muted) {
                    is_muted = 1;
                    mute_duration_counter = 0;
                    triple_beep_flag = 0;
                    triple_beep_i_count = 0;
                } else {
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;
                    triple_beep_i_count = 0;
                }

                while (eve_read_tag() == ERROR_MUTE_TAG) {
                    delay_50ms();
                }
            }

            // 报警声控制
            if (is_muted) {
                mute_duration_counter++;
                if (mute_duration_counter >= 2400) {
                    is_muted = 0;
                    mute_duration_counter = 0;
                    triple_beep_flag = 1;
                    triple_beep_i_count = 0;
                    beep_1khz_150ms(0);
                }
            } else {
                mute_duration_counter = 0;
                if (triple_beep_flag) {
                    triple_beep_i_count++;
                    if (triple_beep_i_count == 1) {
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 8) {
                        beep_1khz_150ms(0);
                    } else if (triple_beep_i_count == 16) {
                        beep_1khz_150ms(0);
                        triple_beep_flag = 0;
                    }
                }
            }

            delay_50ms();
        }

        second_counter++;
        if (second_counter >= 25) {
            second_counter = 0;
        }
    }
}

// =============================================================================
// 函数名：update_error_bell_cache
// 功能：更新错误消息铃铛缓存
// =============================================================================
static void update_error_bell_cache(uint16_t x, uint16_t y, uint8_t size)
{
    if (!error_bell_cache.is_valid) {
        initialize_error_bell_cache(x, y, size);
        return;
    }

    if (error_bell_cache.ref_x == x &&
        error_bell_cache.ref_y == y &&
        error_bell_cache.ref_size == size) {
        return;
    }

    error_bell_cache.is_valid = 0;
    initialize_error_bell_cache(x, y, size);
}

// =============================================================================
// 函数名：initialize_error_bell_cache
// 功能：初始化错误消息铃铛缓存（9个点版本）
// 参数：
//   x - 铃铛中心X坐标
//   y - 铃铛中心Y坐标
//   size - 铃铛基本尺寸
// 返回值：无
// 说明：
//   1. 计算9个点的铃铛轮廓坐标，形成钟形
//   2. 计算交叉线4个端点坐标
//   3. 计算底部圆点坐标
//   4. 保存关键尺寸供绘制使用
// =============================================================================
static void initialize_error_bell_cache(uint16_t x, uint16_t y, uint8_t size)
{
    // 计算关键尺寸
    uint16_t bell_top_width = size;
    uint16_t bell_bottom_width = size * 2;
    uint16_t bell_height = size * 2.5;
    uint16_t exceed = size * 0.3;
    uint16_t dot_radius = bell_bottom_width / 10;

    // ====== 1. 铃铛轮廓点（9个点）=====
    // 点0: 顶部连接点（竖线下端点）
    error_bell_cache.bell_points[0].x = x;
    error_bell_cache.bell_points[0].y = y - bell_height/2;

    // 点1: 左上角 - 上端弧线开始
    error_bell_cache.bell_points[1].x = x - bell_top_width/2.8;
    error_bell_cache.bell_points[1].y = y - bell_height/2 + size*0.2;

    // 点2: 左侧上弧线点 - 增加弧线控制
    error_bell_cache.bell_points[2].x = x - bell_bottom_width/2.3;
    error_bell_cache.bell_points[2].y = y - bell_height/3.2;

    // 点3: 左侧中间点 - 保持斜度
    error_bell_cache.bell_points[3].x = x - bell_bottom_width/2.2;
    error_bell_cache.bell_points[3].y = y - size*0.15;

    // 点4: 底部横线左端点
    error_bell_cache.bell_points[4].x = x - size*1.2;
    error_bell_cache.bell_points[4].y = y + bell_height/2;

    // 点5: 底部横线右端点
    error_bell_cache.bell_points[5].x = x + size*1.2;
    error_bell_cache.bell_points[5].y = y + bell_height/2;

    // 点6: 右侧中间点（点3的对称）
    error_bell_cache.bell_points[6].x = x + bell_bottom_width/2.2;
    error_bell_cache.bell_points[6].y = y - size*0.15;

    // 点7: 右侧上弧线点（点2的对称）
    error_bell_cache.bell_points[7].x = x + bell_bottom_width/2.3;
    error_bell_cache.bell_points[7].y = y - bell_height/3.2;

    // 点8: 右上角（点1的对称）
    error_bell_cache.bell_points[8].x = x + bell_top_width/2.8;
    error_bell_cache.bell_points[8].y = y - bell_height/2 + size*0.2;

    // ====== 2. 交叉线端点（4个点）=====
    error_bell_cache.cross_lines[0].x = x - bell_bottom_width/2 - exceed;
    error_bell_cache.cross_lines[0].y = y - bell_height/3 - exceed;

    error_bell_cache.cross_lines[1].x = x + bell_bottom_width/2 + exceed;
    error_bell_cache.cross_lines[1].y = y + bell_height/3 + exceed;

    error_bell_cache.cross_lines[2].x = x - bell_bottom_width/2 - exceed;
    error_bell_cache.cross_lines[2].y = y + bell_height/3 + exceed;

    error_bell_cache.cross_lines[3].x = x + bell_bottom_width/2 + exceed;
    error_bell_cache.cross_lines[3].y = y - bell_height/3 - exceed;

    // ====== 3. 底部实心圆点 ======
    error_bell_cache.bottom_dot.x = x;
    error_bell_cache.bottom_dot.y = y + bell_height/2 - dot_radius;

    // ====== 4. 保存关键尺寸 ======
    error_bell_cache.bell_height = bell_height;
    error_bell_cache.bell_bottom_width = bell_bottom_width;

    // ====== 5. 保存参考值 ======
    error_bell_cache.ref_x = x;
    error_bell_cache.ref_y = y;
    error_bell_cache.ref_size = size;
    error_bell_cache.is_valid = 1;
}

/**************************************
 * show_error_number()
 * @detail
 * Display text "ERROR" followed by error number. 3 step function:
 * 0 to 1s beep and text, 1 to 2s text only, 2 to 3s blank.
 * repeat the steps until indicated time has elapsed, then exit.
 */
void show_error_number(uint8_t error_number)
{
  uint8_t i;
  uint16_t font;

  font = 31;                        //19 characters fit per line

  for (i = 0; i<3; i++){

	begin_screen(default_config);
    EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
    EVE_CMD_TEXT(240, 82, font, EVE_OPT_CENTERX, "ERROR");
    EVE_CMD_NUMBER(240, 142,font,  EVE_OPT_CENTERX,  error_number);
    EVE_DISPLAY();
    EVE_CMD_SWAP();
    EVE_LIB_EndCoProList();
    EVE_LIB_AwaitCoProEmpty();
    beep_1khz_1sec(G_alarm_vol);

    HAL_Delay(1000);

    begin_screen(default_config);
    EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);

    end_screen();

    HAL_Delay(1000);
  }
  return;
}/**  End show_error_number()  **/

/**************************************
 * show_message_yes_no()
 * @detail
 * Display message text w/ "YES" and "NO" buttons.
 * Font size adjust to the number of characters on line.
 * Black text on white back ground. Screen flashes on/off and alarm beeps at 1hz.
 */

#if USE_HOLD_MODE_NEW_SCREEN
uint8_t show_message_yes_no(const char *strline1)
{
  uint16_t timeout_count,font;
  uint8_t key,len;

  font = 29;                        //33 characters fit per line
  len= strlen(strline1);            //get length of the longest string

  if (len < 28 ) font = 30;			//27 characters fit per line
  if (len < 20 ) font = 31;			//19 characters fit per line
  if (len < 16 ) font = 1;			//15 characters fit per line


  timeout_count = 0;
  while( 1){
	begin_screen(default_config);

	EVE_COLOR_RGB(WHITE_COLOR);  					//Set print color

	EVE_CMD_ROMFONT(1,32);
	EVE_CMD_TEXT(240, 90, font, EVE_OPT_CENTERX, strline1);
    EVE_CMD_FGCOLOR(0x404040);                     //button color
    EVE_COLOR_RGB(WHITE_COLOR);  							//button text color

	draw_text_button(10,200,230,65, "YES", YES_TAG, 31);
	draw_text_button(250,200,230,65, "NO", NO_TAG, 31);

	end_screen();
	key=read_all_keys(&timeout_count);

	if (read_keypad() == RUN_STOP_SWITCH) return(NO);
	if (key == (YES_TAG | PRESSED_BIT) ) key_click();
    if (key == (YES_TAG | RELEASED_BIT) ) return (YES);
    if (key == (NO_TAG | PRESSED_BIT) ) key_click();
    if (key == (NO_TAG | RELEASED_BIT)) return (NO);

    if (timeout_count > EDIT_SCREENS_TIMEOUT){
      return (NO);
    }

    delay_50ms();    //50ms loop
  }
  return(NO);
}/**  End show_message_yes_no  **/
#else
uint8_t show_message_yes_no(const char *strline1)
{
  uint16_t timeout_count,font;
  uint8_t key,len;

  font = 29;                        //33 characters fit per line
  len= strlen(strline1);            //get length of the longest string

  if (len < 28 ) font = 30;			//27 characters fit per line
  if (len < 20 ) font = 31;			//19 characters fit per line
  if (len < 16 ) font = 1;			//15 characters fit per line


  timeout_count = 0;
  while( 1){
	begin_screen(default_config);

	EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR);  					//Set print color

	EVE_CMD_ROMFONT(1,32);
	EVE_CMD_TEXT(240, 90, font, EVE_OPT_CENTERX, strline1);
    EVE_CMD_FGCOLOR(HIGH_VIS_ALARM_COLOR_24bit);                     //button color
    EVE_COLOR_RGB(BLACK_COLOR);  							//button text color

	draw_text_button(100,170,80,60, "YES", YES_TAG, 29);
	draw_text_button(300,170,80,60, "NO", NO_TAG, 29);

	end_screen();
	key=read_all_keys(&timeout_count);

	if (read_keypad() == RUN_STOP_SWITCH) return(NO);
	if (key == (YES_TAG | PRESSED_BIT) ) key_click();
    if (key == (YES_TAG | RELEASED_BIT) ) return (YES);
    if (key == (NO_TAG | PRESSED_BIT) ) key_click();
    if (key == (NO_TAG | RELEASED_BIT)) return (NO);

    if (timeout_count > EDIT_SCREENS_TIMEOUT){
      return (NO);
    }

    delay_50ms();    //50ms loop
  }
  return(NO);
}/**  End show_message_yes_no  **/
#endif
/**************************************
 * show_plug_ac_message()
 * @detail
 * Display "PLUG INTO AC POWER NOW" message text for 10 seconds or
 * until PWR button pressed. Message will clear if the AC power is restored.
 * Otherwise the power shuts down after the time out or if the power button is pressed.
 * Black text on white back ground flashes on/off and alarm beeps at 1hz.
 */
void show_plug_ac_message(void)
{
  const char strline1[] = "PLUG INTO AC";
  const char strline2[] = "POWER NOW";
  uint16_t i,j;

  for (j=0; j<5; j++){               		   //repeat 1s on - 1s off 5 times
    i=0;

	while( i++ <20){
	  begin_screen(default_config);

      EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);
	  if (i==1){
	  	EVE_CMD_ROMFONT(1,32);
        EVE_CMD_TEXT(240, 82, 1, EVE_OPT_CENTERX, strline1);
	    EVE_CMD_TEXT(240, 142, 1, EVE_OPT_CENTERX, strline2);
      }
	  else{
      }

	  end_screen();
      if (i==1){
    	beep_1khz_1sec(G_alarm_vol);
      }

	  if (read_keypad() == (POWER_SWITCH_RELEASED)) return;
	  if (ac_connected()) return;      //no need to return the value, AC connection
	                                   //is checked again in calling routine

	  delay_50ms();    //50ms loop
	  }
  }
  return;
}/**  End show_plug_ac_message()  **/

// 初始化屏幕
void begin_screen(ScreenConfig config) {
    EVE_LIB_BeginCoProList();  // 开始协处理器命令列表

    /* 显示列表初始化 */
    EVE_CMD_DLSTART();                       // 设置显示列表起始指针
    EVE_CLEAR_COLOR_RGB(HIGH_VIS_BG_COLOR);  // 设置背景颜色（黑色）
    EVE_CLEAR(1,1,1);                       // 清除颜色、模版和深度缓冲区
}

// 结束屏幕显示
void end_screen() {
	/* 完成显示列表并交换缓冲区 */
	EVE_DISPLAY();                           // 结束显示列表
	EVE_CMD_SWAP();                          // 交换显示缓冲区
	EVE_LIB_EndCoProList();                  // 结束协处理器命令列表
	EVE_LIB_AwaitCoProEmpty();               // 等待协处理器完成
}

// 专门绘制箭头按钮
void draw_arrow_button(uint8_t tag, uint8_t handle, uint32_t bitmap_offset,
                      uint16_t x, uint16_t y, uint8_t extend_touch) {
    EVE_TAG(tag);
    EVE_BITMAP_HANDLE(handle);
    EVE_BITMAP_SOURCE(bitmap_offset);
    EVE_BITMAP_LAYOUT(1,6,ARROW_BTN_SIZE);
    EVE_BITMAP_SIZE(0,0,0,ARROW_BTN_SIZE,ARROW_BTN_SIZE);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x,y,handle,0);

    if (extend_touch) {
        extend_touch_area(x, y, ARROW_BTN_SIZE, ARROW_BTN_SIZE); // 提取触摸区域扩展为单独函数
    }
}

// 根据电量获取对应的图片地址
uint32_t get_battery_offset(uint8_t percent) {
    if (percent > 80) return battery_100_bitmap_offset;
    else if (percent > 60) return battery_080_bitmap_offset;
    else if (percent > 40) return battery_060_bitmap_offset;
    else if (percent >20) return battery_040_bitmap_offset;
    else return battery_020_bitmap_offset;
}


// 绘制电池图标（带电量 + 充电状态）
void draw_battery_simple(uint16_t x, uint16_t y, uint8_t percent) {
    uint32_t offset = get_battery_offset(percent);

    // 根据状态设置颜色
    if (ac_connected()) {
        EVE_COLOR_RGB(255, 255, 255);      // 充电：绿色
    } else if (percent < 15) {
        EVE_COLOR_RGB(255, 0, 0);      // 低电量：红色
    } else {
        EVE_COLOR_RGB(255, 255, 255);  // 正常：白色
    }

    EVE_BITMAP_HANDLE(BATTERY_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(offset);
    EVE_BITMAP_LAYOUT(1, 6, BATTERY_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, BATTERY_BITMAP_WIDTH, BATTERY_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, BATTERY_BITMAP_HANDLE, 0);
}

// 绘制加号（不加颜色设置）
void draw_plus_icon(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(PLUS_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(plus_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 3, PLUS_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, PLUS_BITMAP_WIDTH, PLUS_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, PLUS_BITMAP_HANDLE, 0);
}

// 绘制减号
void draw_minus_icon(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(MINUS_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(minus_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 3, MINUS_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, MINUS_BITMAP_WIDTH, MINUS_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, MINUS_BITMAP_HANDLE, 0);
}

// 绘制右箭头
void draw_icon(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(RIGHTARROW_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(rightarrow_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 3, RIGHTARROW_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, RIGHTARROW_BITMAP_WIDTH, RIGHTARROW_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, RIGHTARROW_BITMAP_HANDLE, 0);
}

// 绘制警告
void draw_alarm1(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(ALARM1_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(alarm1_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 5, ALARM1_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, ALARM1_BITMAP_WIDTH, ALARM1_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, ALARM1_BITMAP_HANDLE, 0);
}

// 绘制警告
void draw_alarm3(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(ALARM3_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(alarm3_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 6, ALARM3_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, ALARM3_BITMAP_WIDTH, ALARM3_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, ALARM3_BITMAP_HANDLE, 0);
}

// 绘制铃声
void draw_bell(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(BELL_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(bell_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 5, BELL_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, BELL_BITMAP_WIDTH, BELL_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, BELL_BITMAP_HANDLE, 0);
}

// 绘制静音
void draw_mute(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(MUTE_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(mute_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 5, MUTE_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, MUTE_BITMAP_WIDTH, MUTE_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, MUTE_BITMAP_HANDLE, 0);
}

void draw_right_arrow_22x34(uint16_t x, uint16_t y) {

    // 设置线宽（如果需要粗线）
    EVE_LINE_WIDTH(16);  // 2像素宽
    EVE_COLOR_RGB(255, 255, 255);
    // 绘制 ">" 形状
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_VERTEX2II(x + 2, y, 0, 0);           // 左上
    EVE_VERTEX2II(x + 18, y + 17, 0, 0);     // 中间尖角
    EVE_VERTEX2II(x + 2, y + 33, 0, 0);      // 右下
}

void draw_right_arrow_16x23(uint16_t x, uint16_t y) {

    EVE_COLOR_RGB(255, 255, 255);

    // 设置线宽为 2 像素
    EVE_LINE_WIDTH(32);  // 32/16 = 2px

    // 绘制 ">" 形状
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_VERTEX2II(x + 2, y, 0, 0);           // 左上 (2, 0)
    EVE_VERTEX2II(x + 14, y + 11, 0, 0);     // 箭头尖角 (14, 11)
    EVE_VERTEX2II(x + 2, y + 22, 0, 0);      // 右下 (2, 22)
}

// 绘制未上锁
void draw_unlock_icon(uint16_t x, uint16_t y) {
    EVE_BITMAP_HANDLE(UNLOCK_BITMAP_HANDLE);
    EVE_BITMAP_SOURCE(unlock_bitmap_offset);
    EVE_BITMAP_LAYOUT(1, 7, UNLOCK_BITMAP_HEIGHT);
    EVE_BITMAP_SIZE(0, 0, 0, UNLOCK_BITMAP_WIDTH, UNLOCK_BITMAP_HEIGHT);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x, y, UNLOCK_BITMAP_HANDLE, 0);
}
// 增强版箭头按钮绘制函数
void draw_arrow_button_ex(uint8_t tag, uint8_t handle, uint32_t bitmap_offset,
                         uint16_t x, uint16_t y, uint8_t extend_touch,
						 uint8_t r, uint8_t g, uint8_t b) {
    EVE_COLOR_RGB(r, g, b);  // 新增颜色设置
    EVE_TAG(tag);
    EVE_BITMAP_HANDLE(handle);
    EVE_BITMAP_SOURCE(bitmap_offset);
    EVE_BITMAP_LAYOUT(1,6,ARROW_BTN_SIZE);
    EVE_BITMAP_SIZE(0,0,0,ARROW_BTN_SIZE,ARROW_BTN_SIZE);
    EVE_BEGIN(EVE_BEGIN_BITMAPS);
    EVE_VERTEX2II(x,y,handle,0);

    if (extend_touch) {
        extend_touch_area(x, y, ARROW_BTN_SIZE, ARROW_BTN_SIZE);
    }
}
// 专门绘制文本按钮
void draw_text_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     const char* text, uint8_t tag, uint8_t font_size) {
    EVE_TAG(tag);
    EVE_CMD_BUTTON(x, y, w, h, font_size, EVE_OPT_CENTERX, text);
}

// 公共触摸区域扩展函数
void extend_touch_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    EVE_COLOR_MASK(0,0,0,0);
    EVE_BEGIN(EVE_BEGIN_RECTS);
    EVE_VERTEX2F(x-TOUCH_PADDING, y-TOUCH_PADDING);
    EVE_VERTEX2F(x+w+TOUCH_PADDING, y+h+TOUCH_PADDING);
    EVE_COLOR_MASK(1,1,1,1);
}

// 绘制文本（自动调整大小）
void draw_text(uint16_t x, uint16_t y, const char* text, uint16_t options, uint8_t large) {
    uint16_t font = large ? 31 : 29; // 简化版自动调整
    EVE_CMD_TEXT(x, y, font, options, text);
}

// 绘制数字
void draw_number(uint16_t x, uint16_t y, uint16_t value,uint8_t digits, uint8_t options) {
    EVE_CMD_NUMBER(x, y, 29, options, value);
}

/**************************************
 * @detail
 * Initialize the display driver (FT810)
 */
void eve_initialize(void)
{
  EVE_Init();				// Initialise the display
  EVE_calibrate(false);			// Calibrate the display
}/**  End eve_initialize()  **/

__attribute__((always_inline))

static inline void delay_50ms(void)
{
    while(G_tick5ms_counter < 10);  // 等待50ms
    G_tick5ms_counter = 0;
}
