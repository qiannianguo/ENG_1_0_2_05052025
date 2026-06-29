/*****************************************************************************
* @file			: screen_helper.h
* @short        :
*
******************************************************************************
*
* @details
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCREEN_HELPER_H
#define __SCREEN_HELPER_H

#include "screens.h"

// 常量定义（不需要放在结构体中）
#define ANIM_UPDATE_INTERVAL    200   // 200ms动画更新
#define DISPLAY_UPDATE_INTERVAL 3000  // 3秒显示更新
#define FLASH_MAX_TIMER         40
#define FLASH_THRESHOLD         20
#define LOW_VOLTAGE_THRESHOLD   LOW_BATTERY_VOLTAGE  // 使用已有的宏定义

// 几何常量
#define BATTERY_X       355
#define BATTERY_Y       116
#define BATTERY_WIDTH   19
#define BATTERY_HEIGHT  40
#define MAX_FILL_HEIGHT 33

#define TEXT_X          (BATTERY_X + 9)
#define TEXT_Y          (BATTERY_Y + 60)
#define TEXT_FONT       27
#define TEXT_OPTIONS    EVE_OPT_CENTER

#define TRIANGLE_SIDE   24
#define TRIANGLE_HEIGHT 21
#define ARC_RADIUS      (TRIANGLE_SIDE / 2 + 2)

// ====== 在配置文件中定义 ======
#define ENABLE_DISCHARGE_TEST  0  // 1=启用放电测试，0=禁用

#define ENABLE_CHARGE_TEST     0

#define ENABLE_Voltage_display     0
// ====== 简化滤波器定义 ======
#define BATTERY_AVG_POINTS 6  // 3秒数据（0.5s×6）
// 低电量LED闪烁周期
#define LED_BLINK_INTERVAL 500  // 500ms = 0.5秒周期，2Hz频率

// 开机LED闪烁周期
#define BOOT_LED_INTERVAL  150  // 150ms快速闪烁，表示启动中

#define LOW_VOLTAGE_CONFIRM_THRESHOLD 5   // 连续5次检测到低电压才锁定
#define VOLTAGE_HYSTERESIS 20             // 滞回电压30mV

// ====== 动态充电检测配置 ======
#define CHARGE_START_DETECT_VOLTAGE 1430  // 开始检测的电压
#define MAX_SAME_VALUE_COUNT 20           // 最大连续相同值计数
#define SLOW_THRESHOLD 8                  // 缓慢：连续8次以上相同值
#define STEEP_THRESHOLD 4                 // 陡峭：连续5次以下相同值

    // 箭头坐标定义
#define ARROW1_X    86
#define ARROW2_X    86 + 16 + 3   // 第一个X + 箭头宽度16 + 间距3 = 105
#define ARROW3_X    105 + 16 + 3  // 第二个X + 箭头宽度16 + 间距3 = 124
#define ARROW_Y     20


extern uint16_t G_food_rate;
extern uint8_t G_alarm_vol;
extern uint8_t G_warning_vol;
extern uint8_t G_battery_alarm_muted;


// 结构体定义（在头文件或文件顶部）
typedef struct {
    uint16_t raw_voltage;
    uint16_t filtered_voltage;
    uint8_t  percent;
    uint8_t  is_charging;
} BatteryVoltage_t;

typedef struct {
    uint8_t  animation_percent;
    uint8_t  direction;
    uint32_t last_update_time;
} AnimationState_t;

typedef struct {
    uint16_t displayed_voltage;
    uint8_t  displayed_percent;
    uint32_t last_display_update;
} DisplayState_t;

typedef struct {
    uint8_t  timer;
    uint8_t  should_flash;
} FlashEffect_t;

// ====== 新增：动态充电检测算法 ======
typedef  struct {
    uint8_t is_fully_charged;       // 是否已检测到充满
    uint16_t last_voltage;          // 上次电压值
    uint8_t same_value_count;       // 连续相同值计数
    uint8_t detection_started;      // 检测已开始
    uint16_t check_count;           // 每分钟检查计数器
    uint8_t phase;                  // 充电阶段：0=未开始，1=缓慢期，2=陡峭期
 //   uint8_t consecutive_slow_count;
    uint16_t phase_start_voltage;   // 阶段开始电压
} ChargeDetect_t ;




// 静音铃铛图标缓存结构体
typedef struct {
    Point2D bell_points[9];       // 铃铛轮廓点
    Point2D cross_lines[4];       // 交叉线端点
    Point2D bottom_dot;           // 底部实心圆点
    uint8_t  is_valid;            // 缓存是否有效
    uint16_t ref_x;               // 缓存对应的X坐标
    uint16_t ref_y;               // 缓存对应的Y坐标
    uint16_t ref_size;            // 缓存对应的大小
    uint16_t bell_height;         // 铃铛高度（用于绘制）
    uint16_t bell_bottom_width;   // 铃铛底部宽度（用于绘制）
} MuteBellCache;


// =============================================================================
// 警告图标缓存结构体
// =============================================================================
typedef struct {
    uint8_t is_valid;
    uint16_t ref_center_x;
    uint16_t ref_center_y;
    uint16_t tri_points[4][2];     // 三角形4个点
    uint16_t arc_points[31][2];    // 圆弧31个点
    uint16_t excl_points[9][2];    // 感叹号9个点
    uint8_t excl_widths[9];        // 感叹号宽度表
    uint16_t excl_dot[2];          // 感叹号底部的点
} WarningIconCache;

// 声明外部变量（在其他文件中定义）
extern uint16_t G_processed_battery_voltage;
extern uint8_t G_battery_should_flash;

// 可配置的LED闪烁控制
typedef struct {
    uint32_t blink_interval;  // 闪烁间隔（ms）
    uint8_t  duty_cycle;      // 占空比（百分比）
    uint8_t  mode;            // 模式：0=常亮，1=闪烁，2=特定模式
} LedBlinkConfig_t;


/**************************************
 * draw_batt_symbol()
 * @short		Draw battery symbol to display list
 * @param		uint16_t battery voltage
 * @retval		void
 */
void draw_batt_symbol(uint16_t);

/**************************************************************
 * draw_flow_indicator()
 * @short		Draw animated drop flow indicator
 * @param		void
 * @retval  	void
 ***************************************************************/
void draw_flow_indicator(void);

void draw_flow_new_indicator();

/**************************************
 * set_fgcolor_for()
 * @short		Set different color for selected and not selected touch tags
 * @param		uint8_t highlighted column number
 * @param       uint8_t column number to set color
 * @retval      void
 */
void set_fgcolor_for(uint8_t, uint8_t);

/**************************************
 * toggle_dsp_brightness()
 * @short		Toggle display brightness
 * @param		void
 * @retval      void
 */
void toggle_dsp_brightness(void);

#endif /* __SCREEN_HELPER_H */
