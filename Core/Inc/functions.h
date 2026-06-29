/*****************************************************************************
* @file			: functions.h
* @short        : miscellaneous functions
******************************************************************************
*
* @details
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTIONS_H
#define __FUNCTIONS_H

/**************************************
 * bcd_to_dec()
 * @short		convert 8 bit bcd to 8 bit decimal
 * @param		uint8_t BCD number
 * @retval  	uint8_t decimal number
 */
uint8_t bcd_to_dec(uint8_t bcd_num);

/**************************************
 * calc_occl_peaks()
 * @short		Calculate up/down peaks from OCCL data
 * @param		uint16_t* occl data buffer
 * @retval  	uint8_t no of peaks and valleys in occl data
 */
uint8_t calc_occl_peaks(int16_t*);

/**************************************************************
 * check_tir_calibration(void)
 * @short		Test if TIR sensor has been calibrated
 * @param		void
 * @retval      return calibration status
 */
uint8_t check_tir_calibration(void);

/**************************************
 * dec_to_bcd()
 * @short		convert 8 bit decimal to 8 bit bcd
 * @param		uint8_t decimal number
 * @retval  	uint8_t BCD number
 */
uint8_t dec_to_bcd(uint8_t);

/**************************************
 * gen_err_handler()
 * @short		Error handler for non HAL layer errors
 * @param		uint8_t error number
 * @retval  	void
 */
void gen_err_handler(uint8_t);

/**************************************
 * get_saved_variables()
 * @short		Get variables from NV ram
 * @param		void
 * @retval		void
 */
void get_saved_variables(void);

/**************************************************************
 * inactivity_timeout()
 * @short		Test inactivity timer
 * @param       uint16_t*ninactivity timer
 * @retval      void
 */
void inactivity_timeout(uint16_t*);


/**************************************
 * detect_air_vol()
 * @detail
 * Based on volume 1 in 3 ml
 * Air if 1ml detected in 3ml deliver
 *
 */


uint8_t detect_air(air_test_struct *air_test);

/**************************************
 * inc_bat_time()
 * @short		Increment minutes pumped with battery power
 * @param		uint32_t* minutes on battery power
 * @retval  	void
 */
void inc_bat_time(uint32_t*);

/**************************************************************
 * increment_volumes()
 * @short		Increment delivered volumes
 * @param       uint8_t air bubble size in ml
 * @param 		uint16_t bolus_run_volume water volume pumped during current bolus run
 * @retval
 */
void increment_volumes(uint8_t bolus_run, uint8_t *air_ml_count,
        uint16_t *bolus_run_volume, uint16_t *food_volume_increment,dynamic_interval_control_t *control);

/**************************************
 * occl_normalize()
 * @short		Calculate peaks from OCCL data
 * @param		int16_t* occl data buffer
 * @retval  	uint8_tNo of data point out of limits
 */
uint8_t occl_normalize(int16_t*);
/**************************************

 * @brief 带尾部校正的压力数据处理
 * @param occl_data 压力数据数组指针(长度≥120)
 * @return 异常数据点数量(0-255)
 * @note 会自动执行滑动平均/基线校正/尾部插值
 */
uint8_t occl_normalize_tail_and_compare_limits(int16_t occl_data[]);
/**************************************

 * @brief 标准化压力数据处理 (无尾部校正)
 * @param occl_data 压力数据数组(长度≥120)
 * @note 包含滑动平均/基线校正/全局归一化
 */
void occl_normalize_max(int16_t occl_data[]);

/**************************************************************
 * occl_test()
 * @short		OCCL test
 * @param       uint16_t*  timer count for OCCL test events
 * 				int16_t pointer to occlusion test data buffer
 * @retval      uint8_t test status
 */
uint8_t occl_test(uint16_t *occl_timer, int16_t occl_data[OCCL_DATA_SIZE], uint16_t *air_rate_inc,dynamic_interval_control_t *interval_control,uint8_t air_result);

/**
 * @brief 初始化动态间隔控制结构
 * @param control 动态间隔控制结构体指针
 * @detail 在每小时开始时调用，重置所有计数和时间基准
 */
void init_dynamic_interval_control(dynamic_interval_control_t *control);
/**
 * @brief 更新动态间隔控制状态（非第一次进入时调用）
 * @param control 动态间隔控制结构体指针
 * @param current_time 当前时间
 * @param last_enter_time_ptr 上次进入时间的指针（用于更新）
 */
#ifdef USE_UPDATE_DYNAMIC_CONTROL
void init_pause_related_params(dynamic_interval_control_t *control);
#endif

void init_low_flow_wait_state(LowFlowWaitState_t *wait_state);

void update_dynamic_interval_control(dynamic_interval_control_t *control,
                                          uint32_t current_time,
                                          uint32_t last_enter_time);

void check_8th_test_interval(dynamic_interval_control_t *control);

/**
 * @brief 每小时重置间隔控制
 * @param control 动态间隔控制结构体指针
 * @detail 在每小时剂量完成时调用，清零计数并更新时间基准
 */
void reset_hourly_interval_control(dynamic_interval_control_t *control);

/**************************************************************
 * pic_i2c_read()
 * @short		Read one byte from PIC I2C data buffer
 * @param		uint8_t address
 * @retval  	uint8_t Data at address
 */
uint8_t pic_i2c_read(uint8_t);

/**************************************************************
 * reset_air_test_var()
 * @short		Reset variables used for bubble and empty testing
 * @param       air_test_struct variable structure
 * @retval		void
 */
void reset_air_test_var(air_test_struct*);

/**************************************************************
 * start_analog_scan()
 * @short		Allow automatic analog readings
 * @param		void
 * @retval  	void
 */
void start_analog_scan(void);

/**************************************************************
 * stop_analog_scan()
 * @short		Stop automatic analog readings
 * @param		void
 * @retval  	void
 */
void stop_analog_scan(void);

/**************************************************************
 * test_background_ir()
 * @short		Test if the background IR is too strong
 * @param		uint16_t value of measured dark ir of PD2
 * @retval  	void
 */
void test_background_ir(uint16_t);

/**************************************************************
 * test_tir_cal_values()
 * @short		Test if TIR calibration values are within limits
 * @param		int16_t photodiode PD2
 * 				int16_t dac cal value
 * @retval      uint8_t test result CAL_OK or CAL_ERROR
 */
uint8_t test_tir_cal_values(int16_t,int16_t,uint8_t);

/**************************************************************
 * variable_decr()
 * @short		Variable speed value decrement
 * @param       uint8_t count how many loop iterations decrement button has been pressed
 * 				uint16_t* decremented variable pointer
 * @retval      void
 */
void variable_decr(uint8_t,uint16_t*);

/**************************************************************
 * variable_decr_dose()
 * @short		Variable speed decrement for dose limit
 * @param       uint8_t count how many loop iterations decrement button has been pressed
 * 				uint16_t* decremented variable pointer
 * @retval      void
 */
void variable_decr_dose(uint8_t count,uint16_t*);

/**************************************************************
 * variable_decr_rate()
 * @short		Variable speed rate decrement
 * @param       uint8_t count how many loop iterations decrement button has been pressed
 * 				uint16_t* decremented variable pointer
 * @retval      void
 */
void variable_decr_rate(uint8_t count,uint16_t*);


uint8_t detect_air(air_test_struct *air_test);

/**************************************************************
 * variable_incr()
 * @short		Variable speed value increment
 * @param       uint8_t count how many loop iterations increment button has been pressed
 * 				uint16_t* incremented variable pointer
 * @retval      void
 */
void variable_incr(uint8_t,uint16_t*);

/**************************************************************
 * variable_incr_dose()
 * @short		Variable speed increment for dose limit
 * @param       uint8_t count how many loop iterations increment button has been pressed
 * 				uint16_t* incremented variable pointer
 * @retval      void
 */
void variable_incr_dose(uint8_t count,uint16_t*);

/**
 * @brief 判断是否应该触发触摸计数事件
 * @param count 50ms为单位的触摸计数
 * @return 1-触发事件, 0-不触发
 *
 * 逻辑：前1秒不响应，1-5秒每100ms响应，5秒后每50ms响应
 */
uint8_t touch_count_event(uint16_t count);

/**************************************************************
 * variable_incr_rate()
 * @short		Variable speed rate increment
 * @param       uint8_t count how many loop iterations increment button has been pressed
 * 				uint16_t* incremented variable pointer
 * @retval      void
 */
void variable_incr_rate(uint8_t,uint16_t*);



#endif /* __FUNCTIONS_H */
