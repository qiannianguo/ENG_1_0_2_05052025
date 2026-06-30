/*****************************************************************************
* @file			: screen_helper.c
* @short        :
******************************************************************************
*
* @details
*
******************************************************************************
*/

#ifndef _SCREEN_HELPER_H
#define _SCREEN_HELPER_H

#include "HAL.h"
#include "screens.h"
#include "screen_helper.h"
#include <stdint.h>
#include "EVE.h"
#include "common.h"
#include <math.h>
#include "io_calls.h"
#include "main.h"
#include <stdlib.h>
#include "MCU_init.h"
#include <string.h>



// 定义全局变量
uint16_t G_processed_battery_voltage = 1300;
uint8_t G_battery_should_flash = 0;

uint32_t G_battery_reminder_counter = 0;  // 电池报警提醒计数器
uint8_t G_battery_reminder_state = 0;     // 当前提醒状态 (0-2)



void draw_arrow_with_color(uint16_t x, uint16_t y, uint8_t color_type);

// 完整初始化所有字段为0
static ChargeDetect_t charge_detect = {
    .is_fully_charged = 0,
    .last_voltage = 0,
    .same_value_count = 0,
    .detection_started = 0,
    .check_count = 0,
    .phase = 0,
    .phase_start_voltage = 0
};




// ====== 放电测试配置 ======
#if ENABLE_DISCHARGE_TEST
    #define DISCHARGE_MAX_SAMPLES     512
    #define DISCHARGE_START_VOLTAGE   1450   // 1275mV
    #define DISCHARGE_END_VOLTAGE     1180   // 1180mV
    #define DISCHARGE_RECORD_INTERVAL (2 * 60 * 1000)  // 2分钟

    // 放电测试状态（只记录电压）
    static struct {
        uint8_t is_testing;          // 是否正在测试
        uint8_t is_recording;        // 是否正在记录
        uint16_t current_index;      // 当前采样索引
        uint16_t total_samples;      // 总采样数
        uint32_t last_record_time;   // 上次记录时间

        // 只记录电压值
        uint16_t voltage_samples[DISCHARGE_MAX_SAMPLES];
    } discharge_test = {0};
#endif

    // ====== 充电测试配置 ======
    #if ENABLE_CHARGE_TEST
        #define CHARGE_MAX_SAMPLES        512
        #define CHARGE_START_VOLTAGE      1300   // 开始充电的电压
        #define CHARGE_END_VOLTAGE        1450   // 充电完成的电压
        #define CHARGE_RECORD_INTERVAL    (1 * 60 * 1000)  // 1分钟记录一次

        // 充电测试状态
        static struct {
            uint8_t is_testing;          // 是否正在充电测试
            uint8_t is_recording;        // 是否正在记录
            uint16_t current_index;      // 当前采样索引
            uint16_t total_samples;      // 总采样数
            uint32_t last_record_time;   // 上次记录时间

            // 只记录电压值
            uint16_t voltage_samples[CHARGE_MAX_SAMPLES];
        } charge_test = {0};
    #endif

// 状态变量
static uint8_t voltage_locked = 0;
static uint16_t locked_voltage = 0;
static uint8_t last_is_charging = 0;
static uint8_t low_voltage_confirm_count = 0;  // 低电压确认计数器
static uint8_t voltage_change_confirm = 0;




static uint16_t voltage_buffer[BATTERY_AVG_POINTS] = {0};
static uint8_t buffer_idx = 0;
static uint8_t buffer_count = 0;

// 静态全局变量
static uint8_t anim_percent = 0;
static uint8_t anim_direction = 1;
static uint32_t anim_last_time = 0;

static uint16_t disp_voltage = 0;
static uint8_t disp_percent = 0;
static uint32_t disp_last_time = 0;

static uint8_t flash_timer = 0;
static uint8_t should_flash = 0;

static uint32_t led_blink_timer = 0;
static uint8_t led_on_time = 0;

// ====== 静态缓存实例 ======
static MuteBellCache bell_cache = { .is_valid = 0 };
static WarningIconCache warning_cache = {0};

#if(1)
static LedBlinkConfig_t led_config = {
    .blink_interval = 500,    // 500ms周期
    .duty_cycle = 50,         // 50%占空比
    .mode = 0
};

#endif

static void update_led_with_config(uint8_t is_charging, uint8_t should_flash);
static uint16_t manage_voltage_locking(uint16_t filtered_vbatt, uint8_t is_charging);
static void handle_charging(uint16_t filtered_vbatt, uint32_t current_time,
                           uint8_t* battery_percent, uint16_t* disp_voltage,
                           uint8_t* disp_percent, uint32_t* disp_last_time);
static void handle_discharging(uint16_t filtered_vbatt, uint32_t current_time,
                              uint8_t* battery_percent, uint16_t* disp_voltage,
                              uint8_t* disp_percent, uint32_t* disp_last_time);
static void update_battery_state(uint16_t vbatt, uint8_t* out_percent, uint16_t* out_voltage, uint8_t* out_should_flash);
static void draw_battery_icon(uint8_t percent, uint8_t should_flash, uint8_t flash_phase);

static void initialize_bell_cache(uint16_t x, uint16_t y, uint8_t size);
static void update_bell_cache(uint16_t x, uint16_t y, uint8_t size);
static void draw_cached_mute_bell(uint16_t x, uint16_t y, uint8_t size);

static void update_warning_cache(uint16_t center_x, uint16_t ref_y);
static void initialize_warning_cache(uint16_t center_x, uint16_t ref_y);
static void draw_cached_warning_icon(uint16_t center_x, uint16_t ref_y);

static void draw_speaker_icon(uint16_t center_x, uint16_t center_y, uint8_t is_muted);

#if ENABLE_Voltage_display
static void draw_voltage_text(uint16_t voltage);
#endif


#if ENABLE_DISCHARGE_TEST
static void update_discharge_test(uint16_t current_voltage, uint8_t is_charging);



// ====== 放电测试核心函数（简化版） ======
/**
 * @brief  放电测试函数
 * @param  current_voltage 当前滤波后的电池电压 (mV)
 * @param  is_charging     充电状态标志 (1:充电中, 0:放电中)
 *
 * @note   功能说明:
 *         1. 监测放电过程，记录电压随时间变化数据
 *         2. 在非工作状态(g_occl_state非加热/收集)时进行放电测试
 *         3. 每2分钟记录一次电压值
 *         4. 达到结束电压或开始充电时停止测试
 *
 * @note   测试条件:
 *         - 开始条件: 放电状态且电压≤DISCHARGE_START_VOLTAGE
 *         - 结束条件: 电压≤DISCHARGE_END_VOLTAGE或开始充电
 *         - 不测试条件: 设备处于加热或收集状态
 *
 * @note   数据记录:
 *         - 采样间隔: 2分钟
 *         - 最大采样数: DISCHARGE_MAX_SAMPLES (512个)
 *         - 存储内容: 仅电压值
 */
static void update_discharge_test(uint16_t current_voltage, uint8_t is_charging)
{
    // 1. 检查是否应该开始测试
    if (!is_charging && !discharge_test.is_testing) {
        if (current_voltage <= DISCHARGE_START_VOLTAGE) {
            // 开始测试
            discharge_test.is_testing = 1;
            discharge_test.is_recording = 1;
            discharge_test.current_index = 0;
            discharge_test.total_samples = 0;

            // 记录第一个电压值
            if (discharge_test.current_index < DISCHARGE_MAX_SAMPLES) {
                discharge_test.voltage_samples[discharge_test.current_index] = current_voltage;
                discharge_test.current_index++;
                discharge_test.total_samples++;
            }
        }
        return;
    }

    // 2. 如果正在充电，停止测试
    if (is_charging && discharge_test.is_testing) {
        discharge_test.is_testing = 0;
        discharge_test.is_recording = 0;
        return;
    }

    // 3. 如果正在测试
    if (discharge_test.is_testing && discharge_test.is_recording) {
        // 检查是否达到结束电压
    	if(g_occl_state != OCCL_STATE_HEATING && g_occl_state != OCCL_STATE_COLLECTING){
    		if (current_voltage <= DISCHARGE_END_VOLTAGE ) {
    			discharge_test.is_recording = 0;

    			// 记录最后一个电压值
    			if (discharge_test.current_index < DISCHARGE_MAX_SAMPLES) {
    				discharge_test.voltage_samples[discharge_test.current_index] = current_voltage;
    				discharge_test.current_index++;
    				discharge_test.total_samples++;
    			}
    			return;
    		}
    	}

        // 每2分钟记录一次电压
        static uint32_t last_record_time = 0;
        uint32_t current_time = HAL_GetTick();

        if (current_time - last_record_time >= DISCHARGE_RECORD_INTERVAL) {  // 2分钟
            // 记录当前电压值
            if (discharge_test.current_index < DISCHARGE_MAX_SAMPLES) {
                discharge_test.voltage_samples[discharge_test.current_index] = current_voltage;
                discharge_test.current_index++;
                discharge_test.total_samples++;

                last_record_time = current_time;
            } else {
                // 数组已满，停止记录
                discharge_test.is_recording = 0;
            }
        }
    }
}
#endif  // ENABLE_DISCHARGE_TEST

// ====== 充电测试函数 ======
/**
 * @brief  充电测试函数
 * @param  current_voltage 当前滤波后的电池电压 (mV)
 * @param  is_charging     充电状态标志 (1:充电中, 0:放电中)
 *
 * @note   功能说明:
 *         1. 监测充电过程，记录电压随时间变化数据
 *         2. 记录从开始充电到充满的电压变化曲线
 *         3. 每1分钟记录一次电压值
 *         4. 达到充满电压或停止充电时结束测试
 *
 * @note   测试条件:
 *         - 开始条件: 充电状态且电压≤CHARGE_START_VOLTAGE
 *         - 结束条件: 电压≥CHARGE_END_VOLTAGE或停止充电
 *
 * @note   数据记录:
 *         - 采样间隔: 1分钟 (充电过程较快)
 *         - 最大采样数: CHARGE_MAX_SAMPLES (512个)
 *         - 存储内容: 仅电压值
 *
 * @note   与放电测试区别:
 *         1. 采样间隔更短(1分钟 vs 2分钟)
 *         2. 触发条件相反(充电 vs 放电)
 *         3. 电压变化方向不同(上升 vs 下降)
 */
#if ENABLE_CHARGE_TEST
static void update_charge_test(uint16_t current_voltage, uint8_t is_charging)
{
    uint32_t current_time = HAL_GetTick();

    // 1. 检查是否应该开始充电测试
    if (is_charging && !charge_test.is_testing) {
        if (current_voltage <= CHARGE_START_VOLTAGE) {
            // 开始充电测试
            charge_test.is_testing = 1;
            charge_test.is_recording = 1;
            charge_test.current_index = 0;
            charge_test.total_samples = 0;

            // 记录第一个电压值
            if (charge_test.current_index < CHARGE_MAX_SAMPLES) {
                charge_test.voltage_samples[charge_test.current_index] = current_voltage;
                charge_test.current_index++;
                charge_test.total_samples++;

                // 设置第一次记录时间
                charge_test.last_record_time = current_time;
            }
        }
        return;
    }

    // 2. 如果停止充电，停止充电测试
    if (!is_charging && charge_test.is_testing) {
        charge_test.is_testing = 0;
        charge_test.is_recording = 0;
        return;
    }

    // 3. 如果正在充电测试
    if (charge_test.is_testing && charge_test.is_recording) {
        // 检查是否达到充电完成电压
        if (current_voltage >= CHARGE_END_VOLTAGE) {
            charge_test.is_recording = 0;

            // 记录最后一个电压值
            if (charge_test.current_index < CHARGE_MAX_SAMPLES) {
                charge_test.voltage_samples[charge_test.current_index] = current_voltage;
                charge_test.current_index++;
                charge_test.total_samples++;
            }
            return;
        }

        // 每1分钟记录一次电压（充电过程较快）
        if (current_time - charge_test.last_record_time >= CHARGE_RECORD_INTERVAL) {
            // 记录当前电压值
            if (charge_test.current_index < CHARGE_MAX_SAMPLES) {
                charge_test.voltage_samples[charge_test.current_index] = current_voltage;
                charge_test.current_index++;
                charge_test.total_samples++;

                charge_test.last_record_time = current_time;
            } else {
                // 数组已满，停止记录
                charge_test.is_recording = 0;
            }
        }
    }
}
#endif  // ENABLE_CHARGE_TEST

/**
 * @brief  简易电池电压滤波器
 * @param  raw_voltage 原始电池电压采样值 (mV)
 * @return uint16_t    滤波后的电压值 (mV)
 *
 * @note   功能说明:
 *         1. 实现滑动窗口平均滤波
 *         2. 减少电压采样噪声和瞬时波动
 *         3. 仅在有新的不同采样值时更新
 *
 * @note   算法原理:
 *         1. 使用循环缓冲区存储最近N个采样值
 *         2. 计算缓冲区中所有值的平均值
 *         3. 如果新采样值与上次相同，返回上次平均值
 *         4. 如果不同，更新缓冲区并重新计算平均值
 *
 * @note   参数配置:
 *         - 缓冲区大小: BATTERY_AVG_POINTS (6个点)
 *         - 时间窗口: 3秒 (假设50ms采样周期×6=300ms)
 *         - 可抑制: 瞬间波动、采样噪声
 *
 * @note   优点:
 *         1. 计算简单，资源消耗小
 *         2. 能有效平滑电压波动
 *         3. 相同值时不重复计算，节省CPU
 */
uint16_t simple_batt_filter(uint16_t raw_voltage) {
    static uint16_t last_raw = 0;
    static uint16_t last_avg = 0;

    if (raw_voltage != last_raw) {
        last_raw = raw_voltage;

        // 存入缓冲区
        voltage_buffer[buffer_idx] = raw_voltage;
        buffer_idx = (buffer_idx + 1) % BATTERY_AVG_POINTS;
        if (buffer_count < BATTERY_AVG_POINTS) buffer_count++;

        // 计算平均值
        uint32_t sum = 0;
        for (uint8_t i = 0; i < buffer_count; i++) {
            sum += voltage_buffer[i];
        }

        last_avg = sum / buffer_count;
    }

    return last_avg;
}


#if(0)
/**************************************
 * draw_batt_symbol()
 * @details
 * Draw battery symbol to specified coordinates.
 * Read battery voltage and draw rectangle indicating current battery voltage
 */
void draw_batt_symbol(uint16_t vbatt)
{
static uint8_t flash_batt_timer;
uint16_t xc,yc,v;
uint8_t battery_percent;  // 电池百分比
static uint8_t last_display_level = 0;    // 上次显示的电量百分比
static uint32_t last_update_time = 0;     // 上次更新时间
static uint8_t stable_battery_percent = 0;// 稳定的电量百分比
static uint8_t charge_animation_percent = 0;
static uint8_t animation_direction = 1;  // 1=增加，0=减少
static uint32_t last_animation_time = 0;

uint8_t is_charging = ac_connected();  // 获取AC电源状态

  if (vbatt < LOW_BATTERY_VOLTAGE){

    flash_batt_timer++;
    if (flash_batt_timer>40) flash_batt_timer = 0;
    if (flash_batt_timer>20) return;

    /*--------- 在电池上方绘制等边三角形（放大一倍）---------*/
    EVE_VERTEX_FORMAT(0);
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 1);  // 三角形线宽1像素

    // 等边三角形参数（放大一倍）
    uint16_t triangle_side = 24;  // 三角形边长24像素
    uint16_t triangle_height = 21; // 三角形高度约21像素

    // 等边三角形顶点坐标
    uint16_t tri_center_x = 355 + 9;          // 电池中心
    uint16_t tri_bottom_y = 116 - triangle_side-12; // 与电池图标相距一个边长的高度

    uint16_t tri_top_x = tri_center_x;
    uint16_t tri_top_y = tri_bottom_y - triangle_height;

    uint16_t tri_left_x = tri_center_x - triangle_side/2;
    uint16_t tri_left_y = tri_bottom_y;

    uint16_t tri_right_x = tri_center_x + triangle_side/2;
    uint16_t tri_right_y = tri_bottom_y;

    // 三角形中心点
    uint16_t center_x = tri_center_x;
    uint16_t center_y = tri_bottom_y - triangle_height/2;

    // 绘制等边三角形
    EVE_VERTEX2F(tri_top_x, tri_top_y);
    EVE_VERTEX2F(tri_left_x, tri_left_y);
    EVE_VERTEX2F(tri_right_x, tri_right_y);
    EVE_VERTEX2F(tri_top_x, tri_top_y);

    /*--------- 以三角形中心为圆心绘制圆弧（在右侧边外部）---------*/
    EVE_VERTEX_FORMAT(0);
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 0.8);  // 圆弧线宽1像素，与三角形相同

    // 圆弧参数 - 半径为三角形边长的一半
    uint16_t arc_radius = triangle_side / 2 +2;  // 12像素
    uint16_t correct_center_y  = tri_bottom_y - triangle_height/3;


    // 绘制从10°到50°的圆弧（去掉下方重叠部分）
    for (uint8_t i = 0; i <= 30; i++) {
      // 角度从10°到50°，逆时针方向
      float angle = 10.0f - 75.0f * i / 30;  // 10° → 50°
      float rad = angle * 3.14159f / 180.0f;

      uint16_t point_x = center_x + (int16_t)(arc_radius * cosf(rad));
      uint16_t point_y = correct_center_y  + (int16_t)(arc_radius * sinf(rad));
      EVE_VERTEX2F(point_x, point_y);
      }

    /*--------- 在三角形右侧添加平滑渐变水滴形感叹号 ---------*/
    EVE_VERTEX_FORMAT(0);

    // 感叹号位置与三角形右侧对齐
    uint16_t exclamation_x = tri_right_x + 10;  // 三角形右侧边向右偏移10像素

    // 绘制平滑渐变水滴形身体（上粗下细）
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);

    // 水滴底部提前结束，为圆点留出间隔
    uint16_t drop_bottom_y = tri_bottom_y - 5;  // 水滴底部抬高4像素

    // 使用16个点实现平滑渐变
    for (uint8_t i = 0; i <= 16; i++) {
        float progress = (float)i / 16;  // 0到1的进度

        // 从三角形顶部到抬高的水滴底部
        uint16_t current_y = tri_top_y + (uint16_t)((drop_bottom_y - tri_top_y) * progress);

        // 线宽从3像素平滑渐变到1像素
        float width_progress = 1.0f - progress;  // 从1到0，实现上粗下细
        uint8_t line_width = 1 + (uint8_t)(2 * width_progress);  // 1-3像素范围

        EVE_LINE_WIDTH(16 * line_width);
        EVE_VERTEX2F(exclamation_x, current_y);
    }

    // 绘制感叹号圆点（缩小一半，在水滴下方）
    EVE_BEGIN(EVE_BEGIN_POINTS);
    EVE_POINT_SIZE(16 * 3);  // 点大小2像素
    EVE_VERTEX2F(exclamation_x, tri_bottom_y);  // 圆点：与三角形底边平行
  }


#endif

  // ============================ 电池状态管理函数 ============================

  // 子函数1：电压锁定管理
  /**
   * @brief  电压锁定管理函数
   * @param  filtered_vbatt 滤波后的电池电压值 (mV)
   * @param  is_charging    充电状态标志 (1:充电中, 0:放电中)
   * @return uint16_t       锁定后的电压值 (mV)
   *
   * @note   功能说明:
   *         1. 在放电状态下锁定电压，防止电压波动导致的显示跳变
   *         2. 充电器插入时重置锁定状态
   *         3. 使用滞回和确认机制更新锁定电压
   *         4. 低电压时需连续确认才能更新
   *
   * @note   算法特点:
   *         - 上升阈值: 3次确认
   *         - 下降阈值: 2次确认
   *         - 低电压阈值: 连续确认后立即更新
   *         - 充电状态: 不锁定，返回实时电压
   */
  static uint16_t manage_voltage_locking(uint16_t filtered_vbatt, uint8_t is_charging)
  {
      // 1. 检测充电器插入
      uint8_t charger_inserted = (!last_is_charging && is_charging);
      last_is_charging = is_charging;

      // 2. 充电器插入时重置所有状态
      if (charger_inserted) {
          voltage_locked = 0;
          locked_voltage = 0;
          low_voltage_confirm_count = 0;
          voltage_change_confirm = 0;
          return filtered_vbatt;
      }

      // 3. 电压锁定管理
      if (!is_charging) {
          // 情况A：首次放电，初始化锁定电压
          if (voltage_locked == 0) {
              locked_voltage = filtered_vbatt;
              voltage_locked = 1;
              voltage_change_confirm = 0;
              low_voltage_confirm_count = 0;
          }
          // 情况B：已锁定，检查是否需要更新
          else {
              int16_t voltage_diff = filtered_vbatt - locked_voltage;

              // ====== 特殊情况：电压低于低电量阈值 ======
              if (filtered_vbatt < LOW_BATTERY_VOLTAGE) {
                  // 需要连续确认才能更新锁定电压
                  low_voltage_confirm_count++;

                  // 连续确认达到阈值，更新锁定电压
                  if (low_voltage_confirm_count >= LOW_VOLTAGE_CONFIRM_THRESHOLD) {
                      locked_voltage = filtered_vbatt;  // 更新锁定电压
                      low_voltage_confirm_count = 0;
                  }
              }
              // ====== 一般情况：电压变化超过滞回阈值 ======
              else {
                  // 重置低电压确认计数
                  low_voltage_confirm_count = 0;

                  // 电压下降超过滞回阈值
                  if (voltage_diff < -VOLTAGE_HYSTERESIS) {
                      voltage_change_confirm++;
                      if (voltage_change_confirm >= 2) {
                          locked_voltage = filtered_vbatt;  // 更新到更低值
                          voltage_change_confirm = 0;
                      }
                  }
                  // 电压上升超过滞回阈值
                  else if (voltage_diff > VOLTAGE_HYSTERESIS) {
                      voltage_change_confirm++;
                      if (voltage_change_confirm >= 3) {
                          locked_voltage = filtered_vbatt;  // 更新到更高值
                          voltage_change_confirm = 0;
                      }
                  }
                  // 电压在滞回范围内
                  else {
                      voltage_change_confirm = 0;
                  }
              }
          }

          // 放电状态下使用锁定电压显示
          return locked_voltage;
      }
      // 充电状态：使用实时电压，不锁定
      else {
          voltage_locked = 0;
          low_voltage_confirm_count = 0;
          voltage_change_confirm = 0;
          return filtered_vbatt;
      }
  }

  // 子函数2：充电处理
  /**
   * @brief  充电状态处理函数
   * @param  filtered_vbatt   滤波后的电池电压值 (mV)
   * @param  current_time     当前系统时间戳 (ms)
   * @param  battery_percent  输出参数: 计算得到的电池百分比
   * @param  disp_voltage     输出参数: 显示电压值 (mV)
   * @param  disp_percent     输出参数: 显示百分比值
   * @param  disp_last_time   输入输出参数: 上次显示更新时间
   *
   * @note   功能说明:
   *         1. 充电检测: 电压≥1395mV时开始检测
   *         2. 充电状态机: 分3个阶段检测充电过程
   *         3. 充电动画: 充电过程中显示动态动画
   *         4. 充满检测: 根据算法判断充电是否完成
   *
   * @note   充电阶段说明:
   *         阶段0: 初始状态，检测是否快速充电或进入缓慢期
   *         阶段1: 缓慢期，电压稳定，检测是否进入陡峭期
   *         阶段2: 陡峭期，电压快速上升，检测是否充满
   *
   * @note   安全条件: 电压≥1435mV时强制标记为充满
   */
  static void handle_charging(uint16_t filtered_vbatt, uint32_t current_time,
                             uint8_t* battery_percent, uint16_t* disp_voltage,
                             uint8_t* disp_percent, uint32_t* disp_last_time)
  {


      // 电压达到1430mV后开始检测
      if (filtered_vbatt >= CHARGE_START_DETECT_VOLTAGE && !charge_detect.detection_started) {
          charge_detect.detection_started = 1;
          charge_detect.last_voltage = filtered_vbatt;
          charge_detect.same_value_count = 0;
          charge_detect.phase = 0;
          charge_detect.phase_start_voltage = filtered_vbatt;
          charge_detect.check_count = 0;
      }

      // 如果检测已开始且还未标记为充满
      if (charge_detect.detection_started && !charge_detect.is_fully_charged) {
          // 每分钟检查一次：每1200次调用（50ms × 1200 = 60秒）
          charge_detect.check_count++;
          if (charge_detect.check_count >= 1200) {  // 1分钟到了
              charge_detect.check_count = 0;

              // 每分钟计算一次电压变化
              int16_t minute_voltage_diff = filtered_vbatt - charge_detect.last_voltage;

              // 检测电压是否相同或变化很小
              if (minute_voltage_diff == 0) {
                  charge_detect.same_value_count++;
                  if (charge_detect.same_value_count > MAX_SAME_VALUE_COUNT) {
                      charge_detect.same_value_count = MAX_SAME_VALUE_COUNT;
                  }
              } else {
                  charge_detect.same_value_count = 1;
              }

              // ==== 充电状态机（每分钟执行）====
              switch (charge_detect.phase) {
                  case 0:  // 初始状态
                      {
                          uint16_t phase0_total_rise = filtered_vbatt - charge_detect.phase_start_voltage;
                          if (phase0_total_rise >= 15) {
                              charge_detect.phase = 2;
                              charge_detect.phase_start_voltage = filtered_vbatt;
                          }
                          else if (charge_detect.same_value_count >= SLOW_THRESHOLD) {
                              charge_detect.phase = 1;
                              charge_detect.phase_start_voltage = filtered_vbatt;
                          }
                      }
                      break;

                  case 1:  // 缓慢期
                      if (charge_detect.same_value_count <= STEEP_THRESHOLD && minute_voltage_diff > 0) {
                          charge_detect.phase = 2;
                          charge_detect.phase_start_voltage = filtered_vbatt;
                      }
                      break;

                  case 2:  // 陡峭期
                      {
                          uint16_t phase2_total_rise = filtered_vbatt - charge_detect.phase_start_voltage;
                          if (phase2_total_rise >= 5) {
                              charge_detect.is_fully_charged = 1;
                          }
                      }
                      break;

                  default:
                      charge_detect.phase = 0;
                      charge_detect.same_value_count = 0;
                      charge_detect.phase_start_voltage = filtered_vbatt;
                      break;
              }

              // 更新上次电压
              charge_detect.last_voltage = filtered_vbatt;

              // 额外的安全条件
              if (filtered_vbatt >= 1445) {
                  charge_detect.is_fully_charged = 1;
              }
          }
      }

      // ====== 充电显示逻辑 ======
      if (charge_detect.is_fully_charged) {
          *battery_percent = 100;
          anim_percent = 100;
      } else {
    	    // 充电中：动画在20~100之间循环
    	    if (current_time - anim_last_time > ANIM_UPDATE_INTERVAL) {
    	        anim_last_time = current_time;
    	        if (anim_direction == 1) {
    	            // 上升：20→40→60→80→100
    	            if (anim_percent >= 100) {
    	                anim_percent = 100;
    	                anim_direction = 0;
    	            } else {
    	                anim_percent += 20;
    	            }
    	        } else {
    	            // 下降：100→80→60→40→20
    	            if (anim_percent <= 20) {
    	                anim_percent = 20;
    	                anim_direction = 1;
    	            } else {
    	                anim_percent -= 20;
    	            }
    	        }
    	    }
    	    *battery_percent = anim_percent;
      }

      *disp_voltage = filtered_vbatt;
      *disp_percent = *battery_percent;
      *disp_last_time = current_time;
  }

  // 子函数3：放电处理
  /**
   * @brief  放电状态处理函数
   * @param  filtered_vbatt   滤波后的电池电压值 (mV)
   * @param  current_time     当前系统时间戳 (ms)
   * @param  battery_percent  输出参数: 计算得到的电池百分比
   * @param  disp_voltage     输出参数: 显示电压值 (mV)
   * @param  disp_percent     输出参数: 显示百分比值
   * @param  disp_last_time   输入输出参数: 上次显示更新时间
   *
   * @note   功能说明:
   *         1. 重置充电检测状态
   *         2. 根据电压查表计算放电百分比
   *         3. 控制显示更新频率(每3秒更新一次)
   *         4. 重置充电动画状态
   *
   * @note   放电百分比对应表:
   *         电压(mV)  | 百分比
   *         ----------|-------
   *         ≥1340     | 100%
   *         1300-1340 | 80%
   *         1290-1300 | 80%
   *         1275-1290 | 55%
   *         1260-1275 | 25%
   *         1240-1260|  15%
   *         1200-1240 | 5%
   *         <1200     | 1%
   */
  static void handle_discharging(uint16_t filtered_vbatt, uint32_t current_time,
                                uint8_t* battery_percent, uint16_t* disp_voltage,
                                uint8_t* disp_percent, uint32_t* disp_last_time)
  {
       // 停止充电，重置检测状态
      memset(&charge_detect, 0, sizeof(charge_detect));

      // 放电时根据电压计算百分比
      if (filtered_vbatt >= 1340) *battery_percent = 100;
      else if (filtered_vbatt >= 1320) *battery_percent = 85;
      else if (filtered_vbatt >= 1290) *battery_percent = 70;
      else if (filtered_vbatt >= 1283) *battery_percent = 60;
      else if (filtered_vbatt >= 1275) *battery_percent = 50;
      else if (filtered_vbatt >= 1267) *battery_percent = 40;
      else if (filtered_vbatt >= 1260) *battery_percent = 30;
      else if (filtered_vbatt >= 1250) *battery_percent = 22;
      else if (filtered_vbatt >= 1240) *battery_percent = 15;
      else if (filtered_vbatt >= 1200) *battery_percent = 5;
      else *battery_percent = 1;

      anim_percent = 0;
      anim_direction = 1;

      // 放电时每3秒更新显示
      if (current_time - *disp_last_time >= DISPLAY_UPDATE_INTERVAL) {
          *disp_voltage = filtered_vbatt;
          *disp_percent = *battery_percent;
          *disp_last_time = current_time;
      }
  }

  // 主函数：更新电池状态
  /**
   * @brief  主函数 - 更新电池状态
   * @param  vbatt              原始电池电压采样值 (mV)
   * @param  out_percent        输出参数: 电池百分比 (0-100%)
   * @param  out_voltage        输出参数: 显示电压值 (mV)
   * @param  out_should_flash   输出参数: 低电量闪烁标志
   * @return void
   *
   * @note   功能说明:
   *         1. 获取充电状态和滤波电压
   *         2. 执行测试功能(如果使能)
   *         3. 调用电压锁定管理
   *         4. 根据充放电状态调用相应处理函数
   *         5. 计算低电量闪烁标志
   *         6. 更新全局变量
   *
   * @note   调用周期: 50ms (由主循环保证)
   *
   * @note   全局变量更新:
   *         G_processed_battery_voltage: 处理后的显示电压
   *         G_battery_should_flash:      低电量闪烁标志
   *
   * @note   测试功能:
   *         ENABLE_DISCHARGE_TEST: 放电测试记录
   *         ENABLE_CHARGE_TEST:    充电测试记录
   */
  static void update_battery_state(uint16_t vbatt, uint8_t* out_percent,
                                   uint16_t* out_voltage, uint8_t* out_should_flash_flag)
  {
      uint8_t is_charging = ac_connected();
      uint16_t filtered_vbatt = simple_batt_filter(vbatt);
      if (filtered_vbatt == 0) filtered_vbatt = vbatt;

      uint32_t current_time = HAL_GetTick();
      uint8_t battery_percent;

  #if ENABLE_DISCHARGE_TEST
      update_discharge_test(filtered_vbatt, is_charging);
  #endif

  #if ENABLE_CHARGE_TEST
      update_charge_test(filtered_vbatt, is_charging);
  #endif

      // 1. 电压锁定管理
      filtered_vbatt = manage_voltage_locking(filtered_vbatt, is_charging);

      // 2. 充电/放电处理
      if (is_charging) {
          handle_charging(filtered_vbatt, current_time, &battery_percent,
                         &disp_voltage, &disp_percent, &disp_last_time);
      } else {
          handle_discharging(filtered_vbatt, current_time, &battery_percent,
                            &disp_voltage, &disp_percent, &disp_last_time);
      }

      // 3. 闪烁标志处理
      flash_timer++;
      if (flash_timer > FLASH_MAX_TIMER) flash_timer = 0;

      uint8_t flash_flag = (filtered_vbatt < LOW_BATTERY_VOLTAGE && !is_charging);

      // 4. 输出结果
      *out_percent = disp_percent;
      *out_voltage = disp_voltage;
      *out_should_flash_flag = flash_flag;

      // 5. 更新全局变量
      G_processed_battery_voltage = *out_voltage;
      G_battery_should_flash = *out_should_flash_flag;
  }


// 子函数2：绘制电池图标（外框、填充、警告）
  /**
   * @brief  绘制电池图标函数
   * @param  percent      电池百分比 (0-100%)
   * @param  should_flash 低电量闪烁标志 (1:需要闪烁, 0:正常显示)
   * @param  flash_phase  闪烁相位 (1:亮起显示, 0:熄灭不显示)
   *
   * @note   功能说明:
   *         1. 绘制电池外框和填充区域
   *         2. 根据充电状态、电量百分比设置不同颜色
   *         3. 低电量时显示警告图标和文字
   *         4. 根据闪烁相位控制显示/隐藏
   *
   * @note   颜色方案:
   *         - 充电中: 绿色填充
   *         - 正常电量(≥20%): 绿色填充
   *         - 低电量(10-19%): 黄色填充
   *         - 极低电量(<10%): 红色填充
   *
   * @note   低电量警告:
   *         包括红色圆形、"LOW BATTERY"文字、三角形警告标志
   *         这些只在低电量且闪烁相位为亮起时显示
   */
static void draw_battery_icon(uint8_t percent, uint8_t should_flash, uint8_t flash_phase)
{

	// 计算填充高度
    uint16_t fill_v = 39 - (MAX_FILL_HEIGHT * percent / 100);
    if (fill_v < 6) fill_v = 6;
    if (fill_v > 39) fill_v = 39;

    // 只有在不是低电量状态，或者是低电量但处于显示相位时，才绘制电池
    if (!should_flash || (should_flash && flash_phase)) {
        /*--------- 绘制电池外框（白色） ---------*/
        EVE_VERTEX_FORMAT(0);
        EVE_COLOR_RGB(255, 255, 255);  // 白色外框
        EVE_BEGIN(EVE_BEGIN_LINE_STRIP);

        EVE_VERTEX2F(BATTERY_X, BATTERY_Y + 5);
        EVE_VERTEX2F(BATTERY_X + 6, BATTERY_Y + 5);
        EVE_VERTEX2F(BATTERY_X + 6, BATTERY_Y);
        EVE_VERTEX2F(BATTERY_X + 13, BATTERY_Y);
        EVE_VERTEX2F(BATTERY_X + 13, BATTERY_Y + 5);
        EVE_VERTEX2F(BATTERY_X + 19, BATTERY_Y + 5);
        EVE_VERTEX2F(BATTERY_X + 19, BATTERY_Y + 40);
        EVE_VERTEX2F(BATTERY_X, BATTERY_Y + 40);
        EVE_VERTEX2F(BATTERY_X, BATTERY_Y + 5);

        /*--------- 设置填充颜色 ---------*/
        if (ac_connected()) {
            EVE_COLOR_RGB(GREEN_COLOR);  // 充电时绿色
        } else if (percent < 15) {
            EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);  // 低电量黄色
        } else if (percent < 5) {
            EVE_COLOR_RGB(RED_COLOR);  // 极低电量红色
        } else {
            EVE_COLOR_RGB(GREEN_COLOR);  // 正常绿色
        }

        /*--------- 绘制填充 ---------*/
        EVE_VERTEX_FORMAT(0);
        EVE_BEGIN(EVE_BEGIN_RECTS);
        EVE_VERTEX2F(BATTERY_X + 1, BATTERY_Y + fill_v);
        EVE_VERTEX2F(BATTERY_X + 18, BATTERY_Y + 39);
    }
    static uint8_t draw_counter = 0;
    /*--------- 低电量时才显示的红色圆形和警告图标 ---------*/
    if (should_flash) {
        uint16_t center_x = BATTERY_X + 9;

        uint16_t text_y = BATTERY_Y - 30;  // 在电池上方20像素处
        /*--------- MUTE触摸控制区域 ---------*/
           // MUTE文字的位置
        uint16_t mute_x = center_x + 5;
        uint16_t mute_y = text_y + 100;

        EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);  // 黄色文字

        // 绘制 "LOW BATTERY" 文字
        // 方法1：单行显示（如果字体够小）
        // 方法2：分两行显示（如果单行太长）
        EVE_CMD_TEXT(center_x+5, text_y - 10, 26, EVE_OPT_CENTER, "LOW");
        EVE_CMD_TEXT(center_x+5, text_y + 10, 26, EVE_OPT_CENTER, "BATTERY");
        draw_cached_warning_icon(center_x, BATTERY_Y);


        // ====== 直接给MUTE文字设置触摸标签 ======
        EVE_TAG_MASK(1);                    // 启用标签掩码
        EVE_TAG(BATTERY_MUTE_TAG);          // 设置标签

        // 设置颜色
        if (G_battery_alarm_muted) {
            EVE_COLOR_RGB(150, 150, 150);      // 静音状态：灰色
            // >>> 在这里添加静音图标绘制 <<<
            // 图标位置与之前的喇叭按钮相同
            uint16_t icon_x = 250;    // 替换为你的实际X坐标
            uint16_t icon_y = 140;    // 替换为你的实际Y坐标
            uint8_t  icon_size = 10;           // 图标大小，可根据需要调整
    //          EVE_CMD_TEXT(icon_x, icon_y, 26, EVE_OPT_CENTER, "MUTED");
            draw_cached_mute_bell(icon_x, icon_y, icon_size);

        } else {
            EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR); // 激活状态：高亮色
        }
        // 使用缓存优化的喇叭图标绘制
        draw_speaker_icon(mute_x, mute_y, G_battery_alarm_muted);
        // 重要：必须递增计数器
        draw_counter++;

        // 防止溢出，定期重置
        if (draw_counter >= 200) {  // 每200次重置，大约10秒
            draw_counter = 0;
        }
        // 更紧凑的三次蜂鸣（间隔500ms）
        if (!G_battery_alarm_muted ) {
            G_battery_reminder_counter++;

            if (G_battery_reminder_counter >= 300) {  // 约25秒
                G_battery_reminder_counter = 0;
                G_battery_reminder_state = 0;
            }

            if (G_battery_reminder_state < 3) {
                // 500ms间隔：500ms ÷ 128ms ≈ 4次计数
                if ((G_battery_reminder_counter == 0 && G_battery_reminder_state == 0) ||
                    (G_battery_reminder_counter == 7 && G_battery_reminder_state == 1) ||   // 约0.5秒
                    (G_battery_reminder_counter == 14 && G_battery_reminder_state == 2)) {  // 约1秒

                    beep_1khz_150ms(0);
                    G_battery_reminder_state++;
                }
            }
        }
    }
}

// =============================================================================
// 函数名：update_warning_cache
// 功能：更新警告图标缓存状态
// 参数：
//   center_x - 警告图标中心X坐标
//   center_y - 警告图标底部Y坐标（三角形底部）
// 返回值：无
// =============================================================================
static void update_warning_cache(uint16_t center_x, uint16_t ref_y)
{
    if (!warning_cache.is_valid) {
        initialize_warning_cache(center_x, ref_y);
        return;
    }

    if (warning_cache.ref_center_x == center_x &&
        warning_cache.ref_center_y == ref_y) {
        return;
    }

    warning_cache.is_valid = 0;
    initialize_warning_cache(center_x, ref_y);
}

// =============================================================================
// 函数名：initialize_warning_cache
// 功能：初始化警告图标缓存
// 参数：
//   center_x - 警告图标中心X坐标
//   ref_y - 参考Y坐标（用于计算位置）
// 返回值：无
// =============================================================================
static void initialize_warning_cache(uint16_t center_x, uint16_t ref_y)
{
    warning_cache.is_valid = 0;

    // 计算位置（基于原始逻辑）
    uint16_t tri_bottom_y = ref_y - TRIANGLE_SIDE - 40;
    uint16_t tri_top_y = tri_bottom_y - TRIANGLE_HEIGHT;
    uint16_t tri_left_x = center_x - TRIANGLE_SIDE / 2;
    uint16_t tri_right_x = center_x + TRIANGLE_SIDE / 2;
    uint16_t arc_center_y = tri_bottom_y - TRIANGLE_HEIGHT / 3;
    uint16_t exclamation_x = tri_right_x + 8;

    // 1. 三角形坐标
    warning_cache.tri_points[0][0] = center_x;
    warning_cache.tri_points[0][1] = tri_top_y;
    warning_cache.tri_points[1][0] = tri_left_x;
    warning_cache.tri_points[1][1] = tri_bottom_y;
    warning_cache.tri_points[2][0] = tri_right_x;
    warning_cache.tri_points[2][1] = tri_bottom_y;
    warning_cache.tri_points[3][0] = center_x;
    warning_cache.tri_points[3][1] = tri_top_y;

    // 2. 圆弧坐标
    for (uint8_t i = 0; i <= 30; i++) {
        float angle = 10.0f - 75.0f * i / 30;
        float rad = angle * 3.14159f / 180.0f;
        warning_cache.arc_points[i][0] = center_x + (int16_t)(ARC_RADIUS * cosf(rad));
        warning_cache.arc_points[i][1] = arc_center_y + (int16_t)(ARC_RADIUS * sinf(rad));
    }

    // 3. 感叹号坐标和宽度
    uint16_t excl_top_y = tri_top_y + TRIANGLE_HEIGHT * 30 / 100;
    uint16_t excl_bottom_y = tri_bottom_y - TRIANGLE_HEIGHT * 30 / 100;
    uint16_t excl_dot_y = tri_bottom_y - TRIANGLE_HEIGHT * 10 / 100;

    for (uint8_t i = 0; i < 9; i++) {
        float progress = (float)i / 8.0f;
        warning_cache.excl_points[i][0] = exclamation_x;
        warning_cache.excl_points[i][1] = excl_top_y + (uint16_t)((excl_bottom_y - excl_top_y) * progress);

        float width_progress = 1.0f - progress;
        warning_cache.excl_widths[i] = 1 + (uint8_t)(1.5f * width_progress);
    }

    // 4. 感叹号底部的点
    warning_cache.excl_dot[0] = exclamation_x;
    warning_cache.excl_dot[1] = excl_dot_y;

    // 保存参考值
    warning_cache.ref_center_x = center_x;
    warning_cache.ref_center_y = ref_y;
    warning_cache.is_valid = 1;
}

// =============================================================================
// 函数名：draw_cached_warning_icon
// 功能：绘制带缓存的警告图标（固定黄色）
// 参数：
//   center_x - 警告图标中心X坐标
//   ref_y - 参考Y坐标（通常为BATTERY_Y）
// 返回值：无
// 说明：
//   1. 图标颜色固定为黄色（HIGH_VIS_ALARM_COLOR）
//   2. 包含三角形、圆弧、感叹号三部分
//   3. 使用缓存优化性能
// =============================================================================
static void draw_cached_warning_icon(uint16_t center_x, uint16_t ref_y)
{
    EVE_VERTEX_FORMAT(0);

    // 更新缓存
    update_warning_cache(center_x, ref_y);

    // 设置颜色（固定黄色）
    EVE_COLOR_RGB(HIGH_VIS_ALARM_COLOR);

    /* 1. 三角形 */
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 1);
    for (int i = 0; i < 4; i++) {
        EVE_VERTEX2F(warning_cache.tri_points[i][0], warning_cache.tri_points[i][1]);
    }

    /* 2. 圆弧 */
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 0.8);
    for (int i = 0; i < 31; i++) {
        EVE_VERTEX2F(warning_cache.arc_points[i][0], warning_cache.arc_points[i][1]);
    }

    /* 3. 感叹号（9个点，保留渐变） */
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    for (int i = 0; i < 9; i++) {
        EVE_LINE_WIDTH(16 * warning_cache.excl_widths[i]);
        EVE_VERTEX2F(warning_cache.excl_points[i][0], warning_cache.excl_points[i][1]);
    }

    /* 4. 感叹号底部的点 */
    EVE_BEGIN(EVE_BEGIN_POINTS);
    EVE_POINT_SIZE(16 * 2.5);
    EVE_VERTEX2F(warning_cache.excl_dot[0], warning_cache.excl_dot[1]);
}
// =============================================================================
// 函数名：draw_speaker_icon
// 功能：绘制喇叭图标（支持静音状态）
// 参数：
//   center_x - 喇叭图标中心X坐标
//   center_y - 喇叭图标中心Y坐标
//   is_muted - 是否静音状态（1=静音，0=正常）
// 返回值：无
// 说明：
//   1. 直接计算坐标，不使用缓存（因为计算简单）
//   2. 喇叭手柄：5个点构成矩形
//   3. 喇叭口：4个点构成喇叭形状
//   4. 叉号：两条交叉线，表示静音状态
//   5. 线宽固定为1像素
// =============================================================================
static void draw_speaker_icon(uint16_t center_x, uint16_t center_y, uint8_t is_muted)
{
    // ====== 1. 必需的EVE设置 ======
    EVE_VERTEX_FORMAT(0);

    // ====== 2. 设置颜色 ======
    if (is_muted) {
        EVE_COLOR_RGB(150, 150, 150);      // 静音状态：灰色
    } else {
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR); // 激活状态：高亮色
    }

    // ====== 3. 绘制喇叭手柄（5个点）=====
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 1);
    EVE_VERTEX2F(center_x - 13, center_y - 5);
    EVE_VERTEX2F(center_x - 7, center_y - 5);
    EVE_VERTEX2F(center_x - 7, center_y + 5);
    EVE_VERTEX2F(center_x - 13, center_y + 5);
    EVE_VERTEX2F(center_x - 13, center_y - 5);

    // ====== 4. 绘制喇叭口（4个点）=====
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * 1);
    EVE_VERTEX2F(center_x - 7, center_y - 5);
    EVE_VERTEX2F(center_x + 2, center_y - 12);
    EVE_VERTEX2F(center_x + 2, center_y + 12);
    EVE_VERTEX2F(center_x - 7, center_y + 5);

    // ====== 5. 绘制叉号（无论是否静音都显示）=====
    EVE_BEGIN(EVE_BEGIN_LINES);
    EVE_LINE_WIDTH(16 * 1.0);

    // 设置叉号颜色
    if (is_muted) {
        EVE_COLOR_RGB(150, 150, 150);      // 静音状态：灰色叉号
    } else {
        EVE_COLOR_RGB(HIGH_VIS_TEXT_COLOR); // 激活状态：高亮色叉号
    }

    // 第一条线：左上到右下
    EVE_VERTEX2F(center_x + 7, center_y - 5);
    EVE_VERTEX2F(center_x + 11, center_y + 5);

    // 第二条线：左下到右上
    EVE_VERTEX2F(center_x + 7, center_y + 5);
    EVE_VERTEX2F(center_x + 11, center_y - 5);
}
// =============================================================================
// 函数名：initialize_bell_cache
// 功能：初始化或更新铃铛图标缓存
// 参数：
//   x - 铃铛中心X坐标
//   y - 铃铛中心Y坐标
//   size - 铃铛基本尺寸
// 返回值：无
// 说明：
//   1. 首次调用时计算所有点坐标并缓存
//   2. 后续调用检查参数变化，未变化时使用缓存
//   3. 计算内容包括：
//      - 铃铛轮廓7个点坐标
//      - 交叉线4个端点坐标
//      - 底部圆点坐标
//      - 关键尺寸（高度、宽度）
//   4. 缓存有效标志和参考参数
// 注意：
//   - 铃铛形状由9个点定义，形成钟形轮廓
//   - 交叉线超出铃铛轮廓一定距离
//   - 底部圆点紧贴铃铛底部
// =============================================================================

// ====== 初始化缓存函数（仅第一次调用时执行） ======
static void initialize_bell_cache(uint16_t x, uint16_t y, uint8_t size)
{
    // 这个函数只会在第一次调用时执行
    if (bell_cache.is_valid) {
        return; // 已初始化，直接返回
    }

    // 计算关键尺寸（保持完全不变）
    uint16_t bell_top_width = size;
    uint16_t bell_bottom_width = size * 2;
    uint16_t bell_height = size * 2.5;
    uint16_t exceed = size * 0.3;


    // ====== 【增加弧线点】改为8个点 ======
    // 点0: 顶部连接点（竖线下端点） - 保持不变
    bell_cache.bell_points[0].x = x;
    bell_cache.bell_points[0].y = y - bell_height/2;

    // 点1: 左上角开始 - 明显弧度起点
    bell_cache.bell_points[1].x = x - bell_top_width/2.8; // 向内收一点
    bell_cache.bell_points[1].y = y - bell_height/2 + size*0.2; // 开始下弯

    // 点2: 左侧上弧线点 - 新增点，增加弧线控制
    bell_cache.bell_points[2].x = x - bell_bottom_width/2.5; // 向外扩展
    bell_cache.bell_points[2].y = y - bell_height/4; // 弧线中点

    // 点3: 左侧中间点 - 保持斜度
    bell_cache.bell_points[3].x = x - bell_bottom_width/2.2;
    bell_cache.bell_points[3].y = y - size*0.1;

    // 点4: 底部横线左端点 - 明显拉宽
    bell_cache.bell_points[4].x = x - size*1.4;
    bell_cache.bell_points[4].y = y + bell_height/2 + size*0.1;

    // 点5: 底部横线右端点
    bell_cache.bell_points[5].x = x + size*1.4;
    bell_cache.bell_points[5].y = y + bell_height/2 + size*0.1;

    // 点6: 右侧中间点 - 保持斜度（点3的对称）
    bell_cache.bell_points[6].x = x + bell_bottom_width/2.2;
    bell_cache.bell_points[6].y = y - size*0.1;

    // 点7: 右侧上弧线点 - 新增点，增加弧线控制（点2的对称）
    bell_cache.bell_points[7].x = x + bell_bottom_width/2.5;
    bell_cache.bell_points[7].y = y - bell_height/4;

    // 点8: 右上角结束 - 明显弧度终点（点1的对称）
    bell_cache.bell_points[8].x = x + bell_top_width/2.8;
    bell_cache.bell_points[8].y = y - bell_height/2 + size*0.2;

    // ====== 【注意】需要修改缓存结构，增加点数组大小 ======
    // 原来的 bell_cache.bell_points[7] 需要改为 bell_cache.bell_points[9]

    // ====== 【保持完全不变】交叉线端点（4个点） ======
    bell_cache.cross_lines[0].x = x - bell_bottom_width/2 - exceed;
    bell_cache.cross_lines[0].y = y - bell_height/3 - exceed;

    bell_cache.cross_lines[1].x = x + bell_bottom_width/2 + exceed;
    bell_cache.cross_lines[1].y = y + bell_height/3 + exceed;

    bell_cache.cross_lines[2].x = x - bell_bottom_width/2 - exceed;
    bell_cache.cross_lines[2].y = y + bell_height/3 + exceed;

    bell_cache.cross_lines[3].x = x + bell_bottom_width/2 + exceed;
    bell_cache.cross_lines[3].y = y - bell_height/3 - exceed;

    // ====== 【调整圆点位置】紧贴横线下方 ======
    bell_cache.bottom_dot.x = x;
    bell_cache.bottom_dot.y = y + bell_height/2 + size*0.4;

    // ====== 【保持完全不变】保存关键尺寸 ======
    bell_cache.bell_height = bell_height;
    bell_cache.bell_bottom_width = bell_bottom_width;

    // ====== 【保持完全不变】保存参考值 ======
    bell_cache.ref_x = x;
    bell_cache.ref_y = y;
    bell_cache.ref_size = size;
    bell_cache.is_valid = 1; // 标记为已初始化
}

// =============================================================================
// 函数名：update_bell_cache
// 功能：更新铃铛缓存状态（参数变化检查）
// 参数：
//   x - 新的铃铛中心X坐标
//   y - 新的铃铛中心Y坐标
//   size - 新的铃铛基本尺寸
// 返回值：无
// 说明：
//   1. 检查缓存是否有效，无效则初始化
//   2. 比较新参数与缓存参数是否一致
//   3. 参数变化时标记缓存无效并重新初始化
//   4. 参数未变化时直接使用现有缓存
// 设计目的：
//   - 避免不必要的缓存重新计算
//   - 确保参数变化时及时更新缓存
//   - 作为缓存管理的中介函数
// =============================================================================
static void update_bell_cache(uint16_t x, uint16_t y, uint8_t size)
{
    // 第一次调用：初始化缓存
    if (!bell_cache.is_valid) {
        initialize_bell_cache(x, y, size);
        return;
    }

    // 后续调用：检查参数是否变化
    if (bell_cache.ref_x == x &&
        bell_cache.ref_y == y &&
        bell_cache.ref_size == size) {
        return; // 参数未变，使用现有缓存
    }

    // 参数变化：重新初始化
    bell_cache.is_valid = 0; // 标记为需要重新初始化
    initialize_bell_cache(x, y, size);
}
// =============================================================================
// 函数名：draw_cached_mute_bell
// 功能：绘制带缓存的静音铃铛图标
// 参数：
//   x - 铃铛中心X坐标
//   y - 铃铛中心Y坐标
//   size - 铃铛基本尺寸（控制图标大小）
// 返回值：无
// 说明：
//   1. 绘制完整的静音铃铛图标，包括：
//      - 顶部竖线
//      - 铃铛轮廓（7个点）
//      - 交叉斜线（表示静音）
//      - 底部实心圆点
//   2. 使用缓存优化，提高绘制性能
//   3. 线宽根据size参数动态调整
// 绘制顺序：
//   1. 顶部竖线 → 2. 铃铛轮廓 → 3. 交叉线 → 4. 底部圆点
// =============================================================================
static void draw_cached_mute_bell(uint16_t x, uint16_t y, uint8_t size) {

	// ====== 1. 必需的EVE设置 ======
    EVE_VERTEX_FORMAT(0);  // 顶点格式，必须设置！

	// 1. 更新缓存
    update_bell_cache(x, y, size);

    // 2. 从缓存读取关键尺寸
    uint16_t bell_height = bell_cache.bell_height;

    // 3. 绘制顶部竖线
    EVE_BEGIN(EVE_BEGIN_LINES);
    EVE_LINE_WIDTH(16 * (size/12.0));
    // 上端点
    EVE_VERTEX2F(x , (y - bell_height/2 - size*0.8) );
    // 下端点 (铃铛起点)
    EVE_VERTEX2F(bell_cache.bell_points[0].x ,
                 bell_cache.bell_points[0].y );

    // 4. 绘制铃铛轮廓（9个点，形成更圆滑的钟形）
    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_LINE_WIDTH(16 * (size/10.0));
    // 从点0（顶部连接点）开始
    EVE_VERTEX2F(bell_cache.bell_points[0].x, bell_cache.bell_points[0].y);
    // 绘制左侧弧线（点1-4）
    for (int i = 1; i <= 4; i++) {
        EVE_VERTEX2F(bell_cache.bell_points[i].x, bell_cache.bell_points[i].y);
    }
    // 绘制底部横线（点4到点5）
    EVE_VERTEX2F(bell_cache.bell_points[5].x, bell_cache.bell_points[5].y);
    // 绘制右侧弧线（点6-8）
    for (int i = 6; i <= 8; i++) {
        EVE_VERTEX2F(bell_cache.bell_points[i].x, bell_cache.bell_points[i].y);
    }
    // 闭合到起点
    EVE_VERTEX2F(bell_cache.bell_points[0].x, bell_cache.bell_points[0].y);

    // 5. 绘制交叉线
    EVE_BEGIN(EVE_BEGIN_LINES);
    EVE_LINE_WIDTH(16 * (size/8.0));
    // 第一条斜线
    EVE_VERTEX2F(bell_cache.cross_lines[0].x ,
                 bell_cache.cross_lines[0].y );
    EVE_VERTEX2F(bell_cache.cross_lines[1].x ,
                 bell_cache.cross_lines[1].y );
    // 第二条斜线
    EVE_VERTEX2F(bell_cache.cross_lines[2].x ,
                 bell_cache.cross_lines[2].y );
    EVE_VERTEX2F(bell_cache.cross_lines[3].x ,
                 bell_cache.cross_lines[3].y );

    // 6. 绘制底部实心圆
    EVE_BEGIN(EVE_BEGIN_POINTS);
    EVE_POINT_SIZE(16 * (bell_cache.bell_bottom_width / 5));
    EVE_VERTEX2F(bell_cache.bottom_dot.x ,
                 bell_cache.bottom_dot.y );
}

// 子函数3：显示电压文本
/**
 * @brief  绘制电压文本函数
 * @param  voltage 显示电压值 (mV)
 *
 * @note   功能说明:
 *         1. 将电压值转换为字符串
 *         2. 在指定位置显示电压文本
 *         3. 电压值限制在9999mV以内
 *
 * @note   显示格式: "XXXXmV" (例如: "1300mV")
 *
 * @note   显示位置: 由TEXT_X和TEXT_Y常量定义
 */

#if ENABLE_Voltage_display
static void draw_voltage_text(uint16_t voltage)
{
    EVE_COLOR_RGB(255, 255, 255); // 白色文字

    char voltage_str[8];

    if (voltage > 9999) voltage = 9999;

    voltage_str[0] = '0' + voltage / 1000;
    voltage_str[1] = '0' + ((voltage % 1000) / 100);
    voltage_str[2] = '0' + ((voltage % 100) / 10);
    voltage_str[3] = '0' + voltage % 10;
    voltage_str[4] = 'm';
    voltage_str[5] = 'V';
    voltage_str[6] = '\0';

    EVE_CMD_TEXT(TEXT_X, TEXT_Y, TEXT_FONT, TEXT_OPTIONS, voltage_str);
}
#endif
// ==================== 主函数 ====================
/**
 * @brief  主函数 - 绘制电池符号
 * @param  vbatt 原始电池电压采样值 (mV)
 *
 * @note   功能说明:
 *         1. 开机延迟处理(3秒内显示固定电压)
 *         2. 更新电池状态(百分比、电压、闪烁标志)
 *         3. 计算闪烁相位
 *         4. 更新LED指示灯状态
 *         5. 绘制电池图标和电压文本
 *
 * @note   开机延迟:
 *         开机后3秒内显示固定电压1300mV
 *         防止开机瞬间电压不稳定导致显示异常
 *
 * @note   调用关系:
 *         1. update_battery_state() - 更新电池状态
 *         2. update_led_with_config() - 更新LED
 *         3. draw_battery_icon() - 绘制图标
 *         4. draw_voltage_text() - 显示电压
 */
void draw_batt_symbol(uint16_t vbatt)
{

	uint8_t battery_percent;
    uint16_t display_voltage;
    uint8_t should_flash_flag;
    uint8_t flash_phase;
    static uint8_t boot_delay_done = 0;
    static uint32_t boot_time = 0;

    // 开机后延迟2秒再开始正常显示
    if (!boot_delay_done) {
        if (boot_time == 0) {
            boot_time = HAL_GetTick();
        }

        if (HAL_GetTick() - boot_time < 3000) {  // 开机2秒内
            vbatt = 1340;

        } else {
            boot_delay_done = 1;

            // 开机完成，确保LED亮起
          //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            LED_AMB_ON();
        }
    }

    // 1. 更新所有电池状态
    update_battery_state(vbatt, &battery_percent, &display_voltage, &should_flash_flag);

    // 2. 计算闪烁相位
    flash_phase = (flash_timer <= FLASH_THRESHOLD);

    // 3. 更新LED状态
    update_led_with_config(ac_connected(), should_flash_flag);

    // 4. 绘制电池图标
#if USE_HOLD_MODE_NEW_SCREEN
    draw_battery_simple(25,20,battery_percent);
#else
    draw_battery_icon(battery_percent, should_flash_flag, flash_phase);
#endif


    // 5. 显示电压文本
#if ENABLE_Voltage_display
    draw_voltage_text(display_voltage);
#endif

}
/**
 * @brief  更新LED指示灯状态函数
 * @param  is_charging  充电状态标志 (1:充电中, 0:放电中)
 * @param  should_flash 低电量闪烁标志 (1:需要闪烁, 0:正常显示)
 *
 * @note   功能说明:
 *         1. 充电状态: LED关闭
 *         2. 低电量状态: LED以500ms周期闪烁(2Hz频率)
 *         3. 正常放电状态: LED常亮
 *
 * @note   LED闪烁参数:
 *         - 闪烁周期: 500ms
 *         - 占空比: 50%
 *         - 亮灭时间: 各250ms
 *
 * @note   硬件控制:
 *         通过LED_AMB_ON()和LED_AMB_OFF()宏控制LED
 */
static void update_led_with_config(uint8_t is_charging, uint8_t should_flash)
{
    uint32_t current_time = HAL_GetTick();

    if (is_charging) {
        // 充电：关闭
    	LED_AMB_OFF();
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        led_config.mode = 0;

    } else if (should_flash) {
        // 低电量：闪烁
        led_config.mode = 1;
        led_config.blink_interval = 500;  // 500ms周期
        led_config.duty_cycle = 50;       // 50%占空比

        uint32_t on_time = (led_config.blink_interval * led_config.duty_cycle) / 100;
        uint32_t off_time = led_config.blink_interval - on_time;

        if (led_on_time) {
            // 亮的时间段
            if (current_time - led_blink_timer >= on_time) {
                led_blink_timer = current_time;
                led_on_time = 0;
                LED_AMB_ON();
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // 灭
            } else {
            	LED_AMB_OFF();
            	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // 亮
            }
        } else {
            // 灭的时间段
            if (current_time - led_blink_timer >= off_time) {
                led_blink_timer = current_time;
                led_on_time = 1;
                LED_AMB_OFF();
                 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // 亮
            } else {

            	LED_AMB_ON();
            	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // 灭
            }
        }

    } else {
        // 正常：常亮
    	LED_AMB_ON();
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        led_config.mode = 0;
    }
}

// 重置函数
/**
 * @brief  重置电池显示相关状态函数
 *
 * @note   功能说明:
 *         1. 重置充电动画相关变量
 *         2. 重置显示相关变量
 *         3. 重置闪烁相关变量
 *
 * @note   使用场景:
 *         1. 系统复位时
 *         2. 需要重新初始化显示时
 *         3. 清除所有电池显示状态
 *
 * @note   重置的变量:
 *         - 充电动画百分比和方向
 *         - 显示电压和百分比
 *         - 闪烁计时器和标志
 */
void reset_battery_display(void)
{
    anim_percent = 0;
    anim_direction = 1;
    anim_last_time = 0;

    disp_voltage = 0;
    disp_percent = 0;
    disp_last_time = 0;

    flash_timer = 0;
    should_flash = 0;
}
/**************************************************************
 * draw_flow_indicator()
 * @detail
 * Draw animated drop flow indicator
 ***************************************************************/
void draw_flow_indicator()
{
static uint8_t frame_pos;
static uint8_t frame_delay;
uint8_t frame_speed;

  frame_speed = (420-G_food_rate)/10; //calculate time interval between drop movements
    								  //Higher rate -> faster drops
  if (++frame_delay >frame_speed){    //slow down the frames according frame_speed
    frame_pos++;
    frame_delay = 0;
  }

  EVE_BEGIN(EVE_BEGIN_POINTS);
  EVE_POINT_SIZE(100);

  switch (frame_pos){
    case 0:								// 0 1 2 3 4 5 0 1 2 3 4 5 0 1
      EVE_VERTEX2F(325,30);            	// * *         * *         * *
      break;                           	//   * *         * *         *
    case 1:								//     * *         * *
      EVE_VERTEX2F(325,30);				//		 * *         * *
      EVE_VERTEX2F(325,47);				//
      break;
    case 2:
      EVE_VERTEX2F(325,47);
      EVE_VERTEX2F(325,64);
      break;
    case 3:
      EVE_VERTEX2F(325,64);
      EVE_VERTEX2F(325,81);
      break;
    case 4:
	  EVE_VERTEX2F(325,81);
	  EVE_VERTEX2F(325,98);
      break;
    case 5:
      //frame_pos=1;
      break;
    case 6:
      frame_pos=0;
      break;
    default:
      frame_pos=0;
      break;
  }
}/**  End draw_flow_indicator()  **/

void draw_flow_new_indicator()
{
    static uint8_t frame_pos = 0;
    static uint8_t frame_delay = 0;
    uint8_t frame_speed;


    frame_speed = (420 - G_food_rate) / 10;
    if (frame_speed < 2) frame_speed = 2;
    if (frame_speed > 30) frame_speed = 30;

    if (++frame_delay > frame_speed) {
        frame_pos++;
        frame_delay = 0;
        if (frame_pos >= 9) {
            frame_pos = 0;
        }
    }

    // 颜色分配（3个状态循环）
    uint8_t color1, color2, color3;

    if (frame_pos < 3) {
        color1 = 2; color2 = 1; color3 = 0;   // 白 灰 深灰
    } else if (frame_pos < 6) {
        color1 = 0; color2 = 2; color3 = 1;   // 深灰 白 灰
    } else {
        color1 = 1; color2 = 0; color3 = 2;   //  灰 深灰 白
    }

    draw_arrow_with_color(ARROW1_X, ARROW_Y, color1);
    draw_arrow_with_color(ARROW2_X, ARROW_Y, color2);
    draw_arrow_with_color(ARROW3_X, ARROW_Y, color3);
}

// 辅助函数：绘制指定颜色的箭头
void draw_arrow_with_color(uint16_t x, uint16_t y, uint8_t color_type)
{
    switch(color_type) {
        case 0:  // 深灰色
            EVE_COLOR_RGB(100, 100, 100);
            break;
        case 1:  // 灰色（运行中）
            EVE_COLOR_RGB(160, 160, 160);
            break;
        case 2:  // 白色（高亮/当前）
            EVE_COLOR_RGB(255, 255, 255);
            break;
        default:
            EVE_COLOR_RGB(255, 255, 255);
            break;
    }

    EVE_LINE_WIDTH(32);  // 2像素宽

    EVE_BEGIN(EVE_BEGIN_LINE_STRIP);
    EVE_VERTEX2II(x + 2, y, 0, 0);           // 左上
    EVE_VERTEX2II(x + 14, y + 11, 0, 0);     // 箭头尖角
    EVE_VERTEX2II(x + 2, y + 22, 0, 0);      // 右下
}

/**************************************
 * screen_touched()
 * @detail
 * Check if screen touched
 */
uint16_t screen_touched(void)
{
  uint16_t touch_detected;

  if (!(HAL_MemRead16(EVE_REG_TOUCH_RAW_XY) & 0x8000)) {   //set touch_detected if screen touched
	touch_detected = 1;
  }
  else{
	touch_detected=0;
  }
  return touch_detected;
}/** End screen_touched()  **/



/**************************************
 * set_fgcolor_for()
 * @detail
 * Set different color for selected and not selected touch tags
 */
void set_fgcolor_for(uint8_t var_holder,uint8_t column)
{
	if (var_holder == column){  //var_holder has selected tag's column number
		EVE_CMD_FGCOLOR(0x484848); //color for selected tag
	}
	else{
		EVE_CMD_FGCOLOR(0x000000);  		//non selected tag color
	}
}/**  End set_fgcolor_for()  **/

/**************************************
 * toggle_dsp_brightness()
 * @detail
 * Toggle display brightness by changing back light PWM between 30 and 100%
 */
void toggle_dsp_brightness(void)
{
uint8_t dsp_br;
  dsp_br = HAL_MemRead8(EVE_REG_PWM_DUTY);
  if (dsp_br != 30){
    HAL_MemWrite8(EVE_REG_PWM_DUTY, 30);
  }
  else{
    HAL_MemWrite8(EVE_REG_PWM_DUTY, 60);
  }
}/**  End toggle_dsp_brightness()  **/

#endif /* _SCREEN_HELPER_H */
