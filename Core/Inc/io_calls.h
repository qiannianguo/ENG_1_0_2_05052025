/*****************************************************************************
* @file			: io_calls.h
* @short        : Header for io_calls.c file
******************************************************************************
*
* @details
*
******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_CALLS_H
#define __IO_CALLS_H

#include <stdbool.h>
// 位置状态枚举
typedef enum {
    POS_UNKNOWN = 0,   // 未知
    POS_LEFT,          // 最左端
    POS_MID,           // 中间（磁铁位置）
    POS_RIGHT          // 最右端
} PincherPosition_t;

typedef struct {
    bool magnet_detected;
    PincherPosition_t position;     // 当前位置（左/中/右）
    uint16_t step_count;
    int16_t offset_from_mid;        // 相对中间位置的偏移步数（负=左边，正=右边）
} PincherDetectionResult;

extern PincherDetectionResult g_pincher_result;


// 中优先级报警音符频率定义（喂食泵：药物/液体输送类别）
#define NOTE_C4   523   // Do
#define NOTE_D4   587   // Re
#define NOTE_G4   784   // Sol

// 脉冲时长：200ms（符合125~250ms范围）
#define PULSE_DURATION_MS  200

// 音符间的静默间隔：50ms
#define NOTE_GAP_MS        50


/**************************************
 * ac_connected()
 * @short		Test if wall dc adapter is connected
 * @param		void
 * @retval  	uint8_t
 */
uint8_t ac_connected(void);

/**************************************
 * any_key_or_touch()
 * @short		Detect if any of the keys has been pressed or screen has been touched
 * @param		void
 * @retval  	uint8_t status 0 = key presses, 1 = no key
 */
uint8_t any_key_or_touch(void);

/**************************************
 * autoprimedual()
 * @short		Auto prime dual set
 * @param		void
 * @retval  	void
 */
uint8_t autoprimedual(void);

/**************************************
 * autoprimesingle()
 * @short		Auto prime single set.
 * @param		void
 * @retval  	void
 */
uint8_t autoprimesingle(void);
/**************************************
 * beep_1khz_350ms()
 * @short		Generate 1khz audiple tone for 150ms period
 * @param		uint8_t loudness: 1=loud 0 = normal
 * @retval  	void
 */

void beep_1khz_150ms(uint8_t loud);

/**************************************
 * beep_1khz_p5sec()
 * @short		Generate 1khz audiple tone for a half of second period
 * @param		uint8_t loudness: 1=loud 0 = normal
 * @retval  	void
 */
void beep_1khz_p5sec(uint8_t loud);

/**************************************
 * beep_1khz_1sec()
 * @short		Generate 1khz audiple tone for a one second period
 * @param		uint8_t loudness: 1=loud 0 = normal
 * @retval  	void
 */
void beep_1khz_1sec(uint8_t loud);


void beep_medium_priority_feeding_pump(uint8_t loud);
/**************************************
 * beep_1khz_1sec_mon()
 * @short		Generate 1khz audiple tone and return the monitor value
 * @param		uint8_t loudness: 1=loud 0 = normal
 * @retval  	uint16_t monitor value
 */
uint16_t beep_1khz_1sec_mon(uint8_t loud);

/**************************************
 * board_io_initialize()
 * @short		Initialize board io
 * @param		void
 * @retval  	void
 */
void board_io_initialize(void);

/**************************************
 * check_err_data()
 * @short		Check if error data has been initialized
 * @param		void
 * @retval  	void
  */
void check_err_data(void);

/**************************************
 * cal_tir_sensor()
 * @short		Calibrate the tubing ir sensor
 * @param		void
 * @retval  	void
 */
void cal_tir_sensor(void);

/**************************************
 * check_nv_vars()
 * @short		Check NV variables
 * @param		void
 * @retval  	void
 */
void check_nv_vars();

/**************************************************************
 * check_pts_wake_up()
 * @short		Check if set is installed after powering up w/ TPS wake-up
 * @param		void
 * @retval  	void
 */
uint8_t check_pts_wake_up(void);

/**************************************************************
 * detect_tpa()
 * @short       Detect if TPA installed and identify the TPA type.
 * @param		void
 * @retval  	void
 */
void detect_tpa(void);

/**************************************
 * distal_tube_detected()
 * @short		Test if distal tube installed
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t distal_tube_detected (void);


/**************************************
 * distal_tube_empty()
 * @short		Test if the distal tube is empty (no fluid)
 * @param		void
 * @retval  	uint8_t
 */
uint8_t distal_tube_empty();

/**************************************
 * safe_distal_tube_empty()
 * @short		Safe test if the distal tube is empty with ultrasonic state check
 * @param		void
 * @retval  	uint8_t TUBING_EMPTY or TUBING_NOT_EMPTY
 * @note		Only returns valid result when ultrasonic is ON and stable
 *              Returns TUBING_NOT_EMPTY when ultrasonic is OFF or unstable
 **************************************/
uint8_t safe_distal_tube_empty(void);

/**************************************
 * distal_tube_removed()
 * @short       Test if distal tubing installed. Allow set removal if not installed.
 * @param		uint16_t *occl_timer
 * @retval  	uint8_t distal tube was removed YES or NO
 */
uint8_t distal_tube_removed(uint16_t *);

/**************************************
 * empty_test_w_heat()
 * @short		Short heat signature test to verify if the tubing is empty or full
 * @param		void
 * @retval  	uint8_t status 1 or 0
 */
uint8_t empty_test_w_heat(void);

/**************************************
 * end_startup_init()
 * @short		Execute remaining startup tasks
 * @param		void
 * @retval  	void
 */
void end_startup_init(void);

/**************************************
 * eve_calib_read()
 * @short		read touch screen calibration values.
 * @param		struct touchscreen_calibration (pointer)
 * @retval  	uint32_t (calc CS - stored CS)
 */
int8_t eve_calib_read(struct touchscreen_calibration*);

/**************************************
 * eve_calib_write()
 * @short		Save touch screen calibration values.
 * @param		struct touchscreen_calibration (pointer)
 * @retval  	uint32_t (CS)
 */
uint32_t eve_calib_write(struct touchscreen_calibration*);

/**************************************
 * get_cal_rec
 * @short		Read one calibration record
 * @param		uint16_t record number
 * @param 		vol_rec * record
 * @retval  	void
 */
void get_cal_rec(uint16_t,cal_rec*);

/**************************************
 * get_err_rec
 * @short		Read one error record
 * @param		uint16_t record number
 * @param 		vol_rec * record
 * @retval  	void
 */
void get_err_rec(uint16_t,err_rec*);

/**************************************
 * get_time()
 * @short		read RTC
 * @param		void
 * @retval  	void
 */
void get_time(chip_time*);

/**************************************
 * get_vol_rec()
 * @short		Read one daily volume record
 * @param		uint16_t record number
 * @param 		vol_rec * record
 * @retval  	void
 */
void get_vol_rec(uint16_t,vol_rec *);

/**************************************
 * initial_power_test
 * @short		test that there is proper power to run the pump
 * @param		void
 * @retval  	void
 */
void initial_power_test(void);

/**************************************
 * ir_heat_off()
 * @short		IR heat off
 * @param		void
 * @retval  	void
 */
void ir_heat_off(void);

/**************************************
 * ir_heat_on()
 * @short		IR heat on
 * @param		void
 * @retval  	void
 */
void ir_heat_on(void);

/**************************************
 * key_click()
 * @short		Generate audiple click. Used for touch feedback.
 * @param		void
 * @retval  	void
 */
void key_click(void);

/**************************************
 * magnet()
 * @short		Test hall sensor for magnet
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t magnet(void);

/**************************************
 * manualprime()
 * @short		Manual prime function
 * @param		void
 * @retval  	void
 */
void manualprime(void);

/**************************************
 * move_pincher_all_ccw()
 * @short		Rotate pincher all counter clockwise (Bolus tubing closed).
 * @param		void
 * @retval  	uint8_t status (pincher position = ALL_CCW))
 */
uint8_t move_pincher_all_ccw(void);

/**************************************
 * move_pincher_all_cw()
 * @short		Rotate pincher all clockwise (food tubing closed).
 * @param		void
 * @retval  	uint8_t status (pincher position = ALL_CW))
 */
uint8_t move_pincher_all_cw(void);

/**************************************
 * move_pincher_from_all_ccw_to_magnet()
 * @short		Rotate pincher from right (bolus tubing closed) position to center.
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t move_pincher_from_all_ccw_to_magnet(PincherDetectionResult *result);

/**************************************
 * move_pincher_from _all_cw_to_magnet()
 * @short		Rotate pincher from left (food tubing closed) position to center.
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t move_pincher_from_all_cw_to_magnet(void);

/**************************************
 * move_pincher_to_center()
 * @short		Rotate pincher to center position.
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t move_pincher_to_center(void);

/**************************************
 * pinch_ccw()
 * @short		Step pinch motor one step counter clockwise
 * @param		uint8_t pointer motor phase (0 to 4)
 * @retval  	void
 */
void pinch_ccw(uint8_t *);

/**************************************
 * pinch_cw()
 * @short		Step pinch motor one step clockwise
 * @param		uint8_t pointer motor phase (0 to 4)
 * @retval  	void
 */
void pinch_cw(uint8_t *);

/**************************************
 * pinch_n_steps()
 * @short		Step pinch motor n steps left (CW) or right (CCW) clockwise
 * @param		uint16_t number of steps
 * @param		uint8_t pintch direction
 * @retval  	void
 */
void pinch_n_steps(uint16_t , uint8_t );

#if (MOTOR_TYPE == 2)
void Motor_Init(void);

void Motor_SetPhase(uint8_t phase);

#endif
/**************************************
 * power_off_pump()
 * @short		Power off the pump hardware.
 * @param		void
 * @retval  	void
 */
void power_off_pump(void);

/**************************************
 * power_test()
 * @short		test that there is proper power to run the pump
 * @param		void
 * @retval  	uint8_t exit status
 */
uint8_t power_test(void);

/**************************************
 * pts_sensor()
 * @short		Read PTS sensor from AD buffer
 * @param		void
 * @retval  	uint8_t status
 */
uint8_t pts_sensor(void);

/**************************************
 * pts_sensor_read()
 * @short		Read PTS sensor from ADC
 * @param		void
 * @retval  	uint8_t
 */
uint8_t pts_sensor_read(void);

/**************************************
 * usonic_sensor_read()
 * @short		Read ultrasonic sensor from ADC
 * @param		void
 * @retval  	uint8_t
 */
uint8_t usonic_sensor_read(void);

/**************************************
 * pump_motor_off()
 * @short		Stop pump motor and turn coils off
 * @param		void
 * @retval  	void
 */
void pump_motor_off(void);

/**************************************************************
 * read_acc_bat_time()
 * @short		Read accumulated time on battery power from eeprom
 * @param		void
 * @retval		uint32_t accumulated time on battery power in hours
 * Read accumulated time on battery power from eeprom (min). Convert from minutes to hours.
 *
***************************************************************/
uint16_t read_acc_bat_time(void);

/**************************************************************
 * read_acc_bolus_volume()
 * @short		Read accumulated bolus volume from eeprom
 * @param		void
 * @retval		uint32_t Accumulated bolus volume in liters
 */
uint16_t read_acc_bolus_volume(void);

/**************************************************************
 * read_acc_formula_volume()
 * @short		Read accumulated formula volume from eeprom
 * @param       void
 * @retval		uint32_t Accumulated formula volume in liters
 */
uint16_t read_acc_formula_volume(void);

/**************************************
 * read_all_keys()
 * @short		Read keys and the touch screen
 * @param		uint16_t pointer key timer
 * @retval  	uint8_t key code
 */
uint8_t read_all_keys(uint16_t *);

/**************************************
 * read_analog_channel()
 * @short		Read 5 sample average from analog converter
 * @param		void
 * @retval  	uint16_t
 */
uint16_t read_analog_channel(void);

/**************************************
 * ReadNV8()
 * @short		read 8 bit var from EEPROM
 * @param		16-bit address
 * @retval  	8-bit data
 */
uint8_t ReadNV8(uint16_t);

/**************************************
 * ReadNV16()
 * @short		read 16 bit var from EEPROM
 * @param		16-bit address
 * @retval  	16-bit data
 */
uint16_t ReadNV16(uint16_t);

/**************************************
 * ReadNV32()
 * @short		read 32 bit var from EEPROM
 * @param		16-bit address
 * @retval  	32-bit data
 */
uint32_t ReadNV32(uint16_t);

/**************************************
 * read_nv_acc_volume_vars()
 * @short		Read accumulated volume parameters from EEPROM
 * @param		volume parameters structure
 * @retval  	void
 */
void read_nv_acc_volume_vars(void);

/**************************************
 * read_nv_setting_vars()
 * @short		read setting variables from EEPROM
 * @param		void
 * @retval  	void
 */
void read_nv_setting_vars(void);

/**************************************
 * read_nv_volume_vars()
 * @short		read volume parameters from EEPROM
 * @param		void
 * @retval  	void
 */
void read_nv_volume_vars(void);

/**************************************************************
 * read_tir_cal()
 * @short		Read TIR calibration values from eeprom
 * @param       int16_t* , pointer pd2
 * 				int16_t* , dac
 * @retval		checksum
 */
uint16_t read_tir_cal(int16_t*,int16_t*);

/**************************************
 * read_v_bat()
 * @short		Read battery voltage from ADC
 * @param		void
 * @retval  	uint16_t battery voltage in mV
 */
uint16_t read_v_bat(void);

/**************************************
 * read_battery_voltage()
 * @short		Read battery voltage from ADC buffer
 * @param		void
 * @retval  	uint16_t
  */
uint16_t read_battery_voltage(void);

/**************************************
 * @short		Read keys (RUN/STOP and PWR)
 * @param		void
 * @retval  	uint8_t code
 */
uint8_t read_keypad(void);

/**************************************
 * save_cal_rec()
 * @short		Save calibration record to next location in eeprom.
 * @param 		uint16_t DAC value, PD2 value
 * 				uint16_t dac_cal_value
 * @retval  	void
 * Save calibration record to next location in eeprom.
 * DAC value, PD2 value, date and time.
 */
void save_cal_rec(uint16_t, uint16_t);

/**************************************
 * save_daily_vol_rec()
 * @short		Save daily volume record
 * @param		16-bit record address, pointer to vol record
 * @retval  	void
 */
void save_daily_vol_rec(uint16_t,vol_rec *);

/**************************************
 * save_err_rec()
 * @short		Save error record to EEPROM
 * @param		8-bit error number
 * @retval  	void
 */
void save_err_rec(uint8_t);

/**************************************
 * SaveNV8()
 * @short		save 8 bit var to EEPROM
 * @param		8-bit data, 16-bit address
 * @retval  	void
 */
void SaveNV8(uint8_t,uint16_t);

/**************************************
 * SaveNV16()
 * @short		save 16 bit var to EEPROM
 * @param		16-bit data, 16-bit address
 * @retval  	void
 */
void SaveNV16(uint16_t,uint16_t);

/**************************************
 * SaveNV32()
 * @short		save 32 bit var to EEPROM
 * @param		32-bit data, 16-bit address
 * @retval  	void
 */
void SaveNV32(uint32_t,uint16_t);

/**************************************
 * save_nv()
 * @short		Save pumping variables to eeprom
 * @param		void
 * @retval  	void
 */
void save_nv(void);

/**************************************
 * SaveNV_Acc_Volume()
 * @short		Save accumulated volumes
 * @param		void
 * @retval  	void
 */
void SaveNV_Acc_Volume(void);

/**************************************
 * SaveNV_Settings()
 * @short		Save program settings to eeprom
 * @param		void
 * @retval  	void
 */
void SaveNV_Settings(void);

/**************************************
 * SaveNV_Volume()
 * @short		Safe current food rate and volumes to eeprom
 * @param		void
 * @retval  	void
 */
void SaveNV_Volume(void);

/**************************************
 * save_time()
 * @short		save time to RTC
 * @param		void
 * @retval  	void
 */
void save_time(chip_time *);

/**************************************************************
 * save_tir_cal()
 * @short		Save TIR calibration values and the checksum to eeprom
 * @param       int16_t pd2
 * 				int16_t dac_cal_value
 * @retval      void
 */
void save_tir_cal(uint16_t,uint16_t);


/**************************************
 * service_mode_only()
 * @short 		When the pump is started with RUN?STOP button held down,
 * 				it starts in service mode displaying the service pages only.
 * @param	    void
 * @retval      void
 */
void service_mode_only(void);

/**************************************
 * service_mode_req()
 * @short 		Return 1 if RUN/STOP pressed during power on.
 * @param	    uint8_t, 1 = service mode, 0 =  normal operation
 * @retval      void
 */
uint8_t service_mode_req(void);

/**************************************
 * select_analog_channel()
 * @short		select analog channel
 * @param		ad channel
 * @retval  	void
 */
void select_analog_channel(uint8_t );

/**************************************
 * start_pump()
 * @short		Start pump motor
 * @param		uint16_t pump rate
 * @retval  	uint16_t motor interrupt timing
 */
void start_pump(uint16_t ,uint16_t );

/**************************************
 * start_pump_w_old_rate()
 * @short		Start pump motor with existing rate
 * @param		void
 * @retval  	void
 */
void start_pump_w_old_rate(void);

/**************************************
 * stop_pump_motor()
 * @short		Stop pump motor
 * @param		void
 * @retval  	void
 */
void stop_pump_motor(void);

/**************************************
 * test_if_set_installed()
 * @short		Test if set is installed.
 * @param		uint8_t pointer count between activations
 * @param       uint8_t pointer status set_installed 1/0
 * @retval
 */
void test_if_set_installed(uint8_t *, uint8_t *);

/**************************************
 * update_daily_food_vol()
 * @short		save daily delivered food volume to eeprom
 * @param		16-bit hourly deliver food volume (food rate mL/h)
 * @retval  	void
 */
void update_daily_food_vol(uint16_t vol);

/**************************************
 * update_daily_volumes()
 * @short		save both (food/water) daily delivered volumes to eeprom
 * @param		void
 * @retval  	void
 */
void update_daily_volumes(uint16_t food_vol, uint16_t water_vol);

/**************************************
 * update_daily_water_vol()
 * @short		Save daily delivered water volume to eeprom
 * @param		16-bit hourly water rate mL/h
 * @retval  	void
 */
void update_daily_water_vol(uint16_t);



#endif /* __IO_CALLS_H */


