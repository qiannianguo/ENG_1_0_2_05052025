/*****************************************************************************
* @file			: main_tasks.h
* @short        : Header for main_tasks.c file.
******************************************************************************
*
* @details
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_TASKS_H
#define __MAIN_TASKS_H

/**************************************
 * MCU_off_task()
 * @short			Prepare pump for power down
 * @param			void
 * @retval  	    uint8_t, return task mode key
 */
uint8_t MCU_off_task(void);

/**************************************
 * @short			Set pump to Hold Mode
 * @param			void
 * @retval  	    uint8_t task_num
 */
uint8_t hold_mode_task(void);

/**************************************
 * power_test()
 * @short		test that there is proper power to run the pump
 * @param		void
 * @retval  	void
 */
uint8_t power_test(void);

/**************************************
 * @short		Test if ready for run mode
 * @param		void
 * @retval  	uint8_t task_num
 */
uint8_t pre_run_check_task(void);

/**************************************
 * @short		Prime function. Automatic and manual
 * @param		uint8_t, set_type (dual/single TPA)
 * @retval  	uint8_t, return the task mode key. HOLD_MODE__TASK only.
 * @global      G_prime_completed, G_one_ml_count
 */
uint8_t prime_mode_task(uint8_t set_type);

/**************************************
 * @short			RAllow set removal
 * @param			void
 * @retval  	    void
 */
uint8_t remove_set_task(void);

/**************************************
 * @short			Run Mode
 * @param			void
 * @retval  		uint8_t task_num
 */
uint8_t run_mode_task(void);

/**************************************
 * @short			Service page 1
 * @param			void
 * @retval  	    uint8_t task_num
 */
uint8_t service_page_one_task(void);

/**************************************
 * @short			Service page 2
 * @param			void
 * @retval  	    uint8_t task_num
 */
uint8_t service_page_two_task(void);

/**************************************
 * @short			Service page 3
 * @param			void
 * @retval  	    uint8_t task_num
 */
uint8_t service_page_three_task(void);

/**************************************
 * @short			Set dose limit
 * @param			void
 * @retval  		uint8_t, return task code.
  */
uint8_t set_dose_limit_task(void);

/**************************************
 * @short			Set bolus rate
 * @param			void
 * @retval  		uint8_t, return task code.
  */
uint8_t set_water_rate_task();

/**************************************
 * swap_bolus_to_formula()
 * @short			Change from water delivery to formula
 * @param			void
 * @retval  	    uint8_t return delivery mode 0 (FOOD)
 */
uint8_t swap_bolus_to_formula(void);

/************************************
 * swap_formula_to_bolus()
 * @short			Change from formula delivery to water
 * @param			void
 * @retval  	    uint8_t return delivery mode 1 (BOLUS)
 */
uint8_t swap_formula_to_bolus(void);


/**************************************
 * @short			test if set(distal tubing) installed
 * @param			void
 * @retval  		uint8_t, TASK number
 */
uint8_t test_if_set_ready_task(void);

/**************************************
 * @short			Clear rates rate
 * @param			void
 * @retval  		uint8_t, return task code.
  */
uint8_t clear_rates_task(void);

dynamic_interval_control_t* get_interval_control(void);

#endif /* __MAIN_TASKS_H */

