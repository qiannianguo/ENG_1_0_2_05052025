/**
******************************************************************************
* @file			: main.c
* @short        : Main program body
******************************************************************************
* @date       	created: 	07252023
* @date       	modified:	05132025
*
* @version      ENG 1.0.2
* Version history in separate file.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common.h"
#include "functions.h"
#include "screens.h"
#include "io_calls.h"
#include "main_tasks.h"
#include "MCU_init.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* USER CODE BEGIN PV */

uint8_t G_tick5ms_counter;
uint8_t G_tpa_type;
uint8_t G_prime_completed;
uint8_t G_cal_status;

volatile uint16_t G_one_ml_count;
volatile uint8_t G_one_ml_flag;
volatile uint8_t air_usonic_detected_flag = 0;

uint8_t G_bg_ir_test_enabled;
uint16_t G_ir_pd2_nocomp;
uint16_t G_usonic_value, G_ir_pd2_value;
uint16_t G_dac_cal_value;
uint16_t G_tp_value,G_vpts,G_v_battery;
uint8_t G_an_on, G_ir_heat;
uint16_t G_ntc_value;
uint16_t G_alrm_mon_value;
uint16_t G_motor_rate;
uint16_t G_food_rate;
uint16_t G_volume, G_hour_volume,G_water;
uint16_t G_total_volume;
uint16_t G_water_rate;
uint16_t G_dose_limit;

#if ENABLE_LOW_FLOW_MODE
extern uint16_t G_food_rate;
uint16_t G_low_flow_mode;
#endif

uint32_t G_acc_formula_ml, G_acc_bolus_ml,G_acc_bat_time_min;

uint8_t G_controls_locked,G_battery_alarm_muted;
uint8_t G_alarm_vol;
uint8_t G_warning_vol;
uint8_t G_display_brightness;
uint8_t G_dose_complete_callback_index;
uint8_t G_svc_entry_delay;

int16_t occl_comp_peak_offset;
uint8_t occl_comp_count_test,occl_peak_count_test;
volatile int16_t occl_comp_count_test_value[20];
volatile uint8_t occl_comp_count_test_position[20];

uint16_t running_volume;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  uint8_t menu_no;


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  MCU_peripheral_init();

  stop_analog_scan();					//making sure that ADC not running in tim22
 	  	  	  	  						//start later when all initial ADC checks have been completed
  eve_initialize();						//EVE (Embedded video Engine) initialization

  board_io_initialize();                //initialize important io
  initial_power_test();                 //shut off if running with low battery
  start_analog_scan();					//start analog scan (TIM22) IR sensor,Ultrasonic sensor, TP, PTS and Vbat

  //EraseNV1();
//  cal_tir_sensor();
  G_cal_status = check_tir_calibration();	//test if TIR calibration is valid.

  if (service_mode_req()){              //Display service mode screens if RUN/STOP pressed during the
	get_saved_variables();			    // power up
	end_startup_init();
	service_mode_only();
  }
  get_saved_variables();
  load_RAMG(); 						//load fonts and graphic objects
  G_tpa_type = check_pts_wake_up(); 	    //actuate pincher to detect TPA type

#ifdef TEST_MODE_volume
    running_volume = G_volume;  // 测试模式
#else
    running_volume = 0;          // 非测试模式
#endif



  end_startup_init();


  if (G_cal_status == CAL_OK){
	menu_no = TEST_IF_SET_READY__TASK;


  }
  else 	menu_no = HOLD_MODE__TASK;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	pump_motor_off();
    switch (menu_no){
      case TEST_IF_SET_READY__TASK:
        menu_no = test_if_set_ready_task();
        break;
      case HOLD_MODE__TASK:
        menu_no = hold_mode_task();
        break;
      case RUN_MODE__TASK:
        menu_no = run_mode_task();
        break;
      case DOSE_COMPLETE_ALARM__TASK:
        menu_no = dose_complete_alarm_task();
      	break;
      case REMOVE_SET__TASK:
        menu_no = remove_set_task();
        break;
      case SET_DOSE_LIMIT__TASK:
        menu_no = set_dose_limit_task();
        break;
      case SET_WATER_RATE__TASK:
        menu_no = set_water_rate_task();
        break;
      case CLEAR_RATES__TASK:
        menu_no = clear_rates_task();
        break;
      case PRE_RUN_CHECK__TASK:
    	menu_no = pre_run_check_task();
		break;
      case PRIME_MODE__TASK:
        menu_no = prime_mode_task(G_tpa_type);
        break;
      case SERVICE_PAGE_ONE__TASK:
        menu_no =service_page_one_task();
        break;
      case SERVICE_PAGE_TWO__TASK:
        menu_no =service_page_two_task();
        break;
      case SERVICE_PAGE_THREE__TASK:
        menu_no =service_page_three_task();
        break;
      case SET_TIME__TASK:
        menu_no = set_time_task();
        break;
      case MCU_OFF__TASK:
        menu_no = MCU_off_task();
        break;
      case SETTING_SCREEN__TASK:
        menu_no =setting_screen_task();
        break;
      default:
        power_off_pump();
        break;
    }
  }
}
