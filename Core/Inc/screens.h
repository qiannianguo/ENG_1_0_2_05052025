/*****************************************************************************
* @file			: screens.h
* @short        :
*
******************************************************************************
*
* @details
*
******************************************************************************
*/

#ifndef _SCREENS_H
#define _SCREENS_H

#include <stdint.h>
#include <FT8xx.h>
#include <stdbool.h>
#define COLUMN_0 0
#define COLUMN_1 1
#define COLUMN_2 2
#define REQUIRE_NOTOUCH 1
#define READ_ANY 0
#define TOUCH_PADDING 20
#define ARROW_BTN_SIZE 48

#define BATTERY_BITMAP_HANDLE   4
#define BATTERY_BITMAP_WIDTH           45
#define BATTERY_BITMAP_HEIGHT          25

#define PLUS_BITMAP_WIDTH        22
#define PLUS_BITMAP_HEIGHT       22
#define MINUS_BITMAP_WIDTH       22
#define MINUS_BITMAP_HEIGHT      22
#define RIGHTARROW_BITMAP_WIDTH  22
#define RIGHTARROW_BITMAP_HEIGHT 34
#define UNLOCK_BITMAP_WIDTH        49
#define UNLOCK_BITMAP_HEIGHT       42
#define LOCK_BITMAP_WIDTH        33
#define LOCK_BITMAP_HEIGHT       42

#define ALARM1_BITMAP_WIDTH        35
#define ALARM1_BITMAP_HEIGHT       28

#define ALARM3_BITMAP_WIDTH        44
#define ALARM3_BITMAP_HEIGHT       28

#define BELL_BITMAP_WIDTH        37
#define BELL_BITMAP_HEIGHT       43
#define MUTE_BITMAP_WIDTH        37
#define MUTE_BITMAP_HEIGHT       43


#define START_BITMAP_WIDTH        430
#define START_BITMAP_HEIGHT       48

#define PLUS_BITMAP_HANDLE      5
#define MINUS_BITMAP_HANDLE     6
#define RIGHTARROW_BITMAP_HANDLE 7
#define UNLOCK_BITMAP_HANDLE 8

#define START_BITMAP_HANDLE 9
#define ALARM1_BITMAP_HANDLE 10
#define BELL_BITMAP_HANDLE 11
#define MUTE_BITMAP_HANDLE 12
#define ALARM3_BITMAP_HANDLE 13


typedef struct {
    uint8_t bg_r, bg_g, bg_b;      // 背景色（24位 RGB）
    uint8_t text_r, text_g, text_b; // 文本色
    uint8_t alarm_r, alarm_g, alarm_b; // 警报色
} ScreenConfig;


// 按钮类型枚举
typedef enum {
    BUTTON_NORMAL,
    BUTTON_ARROW_RIGHT,
    BUTTON_ARROW_UP,
    BUTTON_ARROW_DOWN,
    BUTTON_TOGGLE
} ButtonType;

// 记录类型枚举
typedef enum {
    RECORD_ERROR,
    RECORD_VOLUME,
    RECORD_CALIBRATION
} RecordType;



// 点坐标结构体
typedef struct {
    uint16_t x;
    uint16_t y;
} Point2D;


// =============================================================================
// 错误消息显示专用的铃铛图标缓存结构
// =============================================================================
typedef struct {
    uint8_t is_valid;
    uint16_t ref_x;
    uint16_t ref_y;
    uint8_t ref_size;
    Point2D bell_points[9];       // 铃铛轮廓7个点
    Point2D cross_lines[4];       // 交叉线4个点
    Point2D bottom_dot;           // 底部圆点
    uint16_t bell_height;
    uint16_t bell_bottom_width;
} ErrorBellCache;

// =============================================================================
// 错误消息警告图标缓存结构
// =============================================================================
typedef struct {
    uint8_t is_valid;
    uint16_t ref_center_x;
    uint16_t ref_center_y;
    uint16_t ref_triangle_side;
    uint16_t ref_triangle_height;
//    uint32_t ref_color24;

    // 三角形顶点坐标
    uint16_t tri_top_x;
    uint16_t tri_top_y;
    uint16_t tri_left_x;
    uint16_t tri_left_y;
    uint16_t tri_right_x;
    uint16_t tri_right_y;
    uint16_t tri_bottom_y;
    uint16_t tri_center_x;

    // 圆弧中心坐标
    uint16_t arc_center_y;
    uint16_t arc_radius;

    // 感叹号 - 改为直线端点方式（删除数组，改为单个端点）
    uint16_t exclamation_x[3];     // 3个感叹号的X坐标（保留）
    uint16_t drop_top_y;           // 顶部Y坐标（新增）
    uint16_t line_bottom_y;        // 底部Y坐标（新增）
    uint16_t dot_y;               // 底部点的Y坐标（保留）

    // 圆弧点坐标
    uint16_t arc_points[31][2];


} ErrorWarningIconCache;
/**************************************
 * EVE_calibrate()
 * @short		FT810 EVE calibration
 * @param		void
 * @retval      void
 */
void EVE_calibrate(bool force_calib);

/**************************************
 * air_in_tubing_screen()
 * @short		Display AIR IN LINE message
 * @param		void
 * @retval      void
 */
void air_in_tubing_screen(void);

/**************************************
 * auto_primescreen()
 * @short		Display AUTO PRIME screen
 * @param		uint8_t set status (ready/not ready)
 * @param		uint8_t set status (auto priming in progress)
 * @retval  	void
 */
void auto_primescreen(uint8_t,uint8_t);

/**************************************
 * cal_rec_screen()
 * @short		Display calibration values stored in eeprom
 * @param		void
 * @retval  	uint8_t exit status
 */
uint8_t cal_rec_screen(void);

/**************************************
 * clear_rates_screen()
 * @short		Display clear rates screen (volume, dose limit and total volume)
 * @param		void
 * @retval  	uint8_t, return task code.
 */
void clear_rates_screen(void);

/**************************************
 * controls_locked_screen()
 * @short			Display CONNTROLS LOCKED / CONTROLS UNLOCKED screen
 * @param			uint8_t locked / unlocked
 * @retval  	    uint8_t locked / unlocked
 */
uint8_t controls_locked_screen(uint8_t);



/**************************************
 * daily_volume_screen()
 * @short			Display daily totals for delivered bolus and food volumes
 * @param			void
 * @retval  	    uint8_t exit status
 */
uint8_t daily_volume_screen(void);

/**************************************
 * display_start_screen()
 * @short			Display the start screen
 * @param			void
 * @retval  	    void
 */
void display_start_screen(void);

/**************************************
 * dose_complete_alarm_task()
 * @short		Display "DOSE COMPLETE" alarm
 * @param		void
 * @retval      uint8_t task_num
 */
uint8_t dose_complete_alarm_task(void);

/**************************************
 * empty_tubing_screen()
 * @short		Display EMPTY FORMULA or EMPTY BOLUS message
 * @param		uint8_t
 * @retval      void
 */
void empty_tubing_screen(uint8_t);

/**************************************
 * error_screen()
 * @short		Display error messages saved in eprom.
 * @param		location of last saved error message
 * @retval  	uint8_t, return task code.
 */
uint8_t error_screen(void);

/**************************************
 * eve_initialize()
 * @short		Initialize the display
 * @param		void
 * @retval		void
 */
void eve_initialize(void);

/**************************************
 * @short		Read touch tag from display
 * @param		void
 * @retval      uint8_t task_num
 */
uint8_t eve_read_tag(void);

/**************************************
 * @short		Display HOLD MODE Screen
 * @param		void
 * @retval  	void
 */
void hold_mode_screen(uint8_t);

void hold_mode_new_screen(uint8_t );

/**************************************
 * @short		Load custom graphics to graphics controller
 * @param		void
 * @retval  	void
 */
void load_RAMG(void);

/**************************************
 * manual_primescreen()
 * @short			Display MANUAL PRIME screen
 * @param			uint8_t set status (ready/not ready)
 * @retval  	    void
 */
void manual_primescreen(uint8_t);

/**************************************
 * MCU_off_bar()
 * @short			Prepare to power down
 * @param			void
 * @retval  	    void
 */
uint8_t MCU_off_bar(void);

/**************************************
 * occlusion_in_line_screen()
 * @short		Display OCCLUSION IN LINE message
 * @param		void
 * @retval      uint8_t task_num
 */
void occlusion_in_line_screen(void);

void air_occlusion_in_line_screen(void);

/**************************************
 * @short		Display RUN MODE Screen
 * @param		void
 * @retval  	void
 */
void run_mode_screen(void);

void run_mode_new_screen(void);

/**************************************
 * @short		Check if screen touched
 * @param		void
 * @retval      uint8_t touch detected
 */
uint16_t screen_touched(void);

/**************************************
 * service_page_one_screen()
 * @short			Service page 1 Screen
 * @param			uint8_t cam position
 * @retval  	    void
 */
void service_page_one_screen(uint8_t);

/**************************************
 * service_page_two_screen()
 * @short			Service page 2 Screen
 * @param			uint16_t total_formula_delivered
 * @param			uint16_t total_bolus_delivered
 * @param			uint16_t total_hours_battery
 * @retval  	    void
 */
void service_page_two_screen(uint16_t, uint16_t, uint16_t, uint16_t);


/**************************************
 * service_page_three_screen()
 * @short			Service page 3 Screen
 * @param			uint16_t total_formula_delivered
 *                  uint16_t total_bolus_delivered
 *                  uint16_t total_hours_battery
 *                  uint16_t alrm_mon_value
 * @retval  	    void
 */
void service_page_three_screen(uint16_t,uint16_t);

/**************************************
 * set_dose_limit_screen()
 * @short		Display Set dose limit screen
 * @param		void
 * @retval  	void
 */
void set_dose_limit_screen(void);

/**************************************
 * @short		Set time to the RTC
 * @param		void
 * @retval  	uint8_t, return task code.
 */
uint8_t set_time_task(void);

/**************************************
 * setting_screen_task()
 * @short		Edit program settings
 * @param		void
 * @retval  	uint8_t, return task code.
 */
uint8_t setting_screen_task();

/**************************************
 * set_water_rate_screen()
 * @short		Display set water rate screen
 * @param		void
 * @retval  	void
 */
void set_water_rate_screen(void);

/**************************************
 * show_bolus_time()
 * @short			Show bolus time screen
 * @param			uint16_t bolus time remaining in seconds
 * @retval  	    void
 */
void show_bolus_time(uint16_t);

/**************************************
 * show_empty_battery_warning
 * @short		Display empty battery warning
 * @param		void
 * @retval      uint8_t EMPTY_BATTERY or AC_CONNECTED
 */
uint8_t show_empty_battery_warning(void);

/**************************************
 * show_error_message()
 * @short		Display error message
 * @param		uint8_t display timeout in seconds
 * @param		char strline1, message line 1
 * @param       char strline2, message line 2
 * @retval      void
 */
void show_error_message(uint8_t, const char *,const char *);

void show_error_message_without_alarm(uint8_t timeout, const char *strline1, const char *strline2);

/**************************************
 * show_error_number()
 * @short		Display error message with error number
 * @param		uint8_t error number
 * @retval      void
 */
void show_error_number(uint8_t);

/**************************************
 * show_message()
 * @short		Display message
 * @param		char strline1, message line 1
 * @param       char strline2, message line 2
 * @retval      void
 */
void show_message(const char*,const char*);

/**************************************
 * show_message_yes_no()
 * @short		Display  message with feedback buttons
 * @param		char strline1, message line 1
 * @retval      void
 */
uint8_t show_message_yes_no(const char *);

/**************************************
 * show_plug_ac_message()
 * @short		Display "PLUG INTO AC" message
 * @param		void
 * @retval      void
 */
void show_plug_ac_message(void);

/**************************************
 * show_timeout_message()
 * @short		Display message
 * @param		char strline1, message line 1
 * @param       char strline2, message line 2
 * @retval      void
 */
void show_timeout_message(const char *,const char *);

/**************************************
 * show_total_volume()
 * @short			Show total volume on display
 * @param			void
 * @retval  	    void
 */
void show_total_volume(void);

/**************************************
 * special_char_screen()
 * @short		Demonstrate FT810 special character font
 * @param		void
 * @retval  	uint8_t return task code.
 */
uint8_t special_char_screen(void);

void begin_screen(ScreenConfig config);
void end_screen();
// 绘制按钮
void draw_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                const char* text, uint8_t tag, ButtonType type);
// 专门绘制文本按钮
void draw_text_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     const char* text, uint8_t tag, uint8_t font_size) ;
// 绘制文本（自动调整大小）
void draw_text(uint16_t x, uint16_t y, const char* text, uint16_t options, uint8_t large);
// 绘制数字
void draw_number(uint16_t x, uint16_t y, uint16_t value,uint8_t digits, uint8_t options);
void draw_arrow_button(uint8_t tag, uint8_t handle, uint32_t bitmap_offset,
                      uint16_t x, uint16_t y, uint8_t extend_touch);
// 增强版箭头按钮绘制函数
void draw_arrow_button_ex(uint8_t tag, uint8_t handle, uint32_t bitmap_offset,
                         uint16_t x, uint16_t y, uint8_t extend_touch,
						 uint8_t r, uint8_t g, uint8_t b);
void extend_touch_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

uint32_t get_battery_offset(uint8_t percent);

void draw_battery_simple(uint16_t x, uint16_t y, uint8_t percent);

void draw_plus_icon(uint16_t x, uint16_t y);
void draw_minus_icon(uint16_t x, uint16_t y);
void draw_rightarrow_icon(uint16_t x, uint16_t y);
void draw_bell(uint16_t x, uint16_t y);
void draw_mute(uint16_t x, uint16_t y);
void draw_alarm1(uint16_t x, uint16_t y);
void draw_alarm3(uint16_t x, uint16_t y);

void draw_right_arrow_22x34(uint16_t x, uint16_t y);

void draw_right_arrow_16x23(uint16_t x, uint16_t y);
void draw_unlock_icon(uint16_t x, uint16_t y);

#endif /* _SCREENS_H */
