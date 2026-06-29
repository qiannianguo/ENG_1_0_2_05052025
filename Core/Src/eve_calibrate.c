/*****************************************************************************
* @file			: eve_calibrate.c
* @short        : Calibration routine for touch display
******************************************************************************
*
* @details
*
******************************************************************************
*/

//#include <screens.h>
#include <stdint.h>


#include "EVE.h"
#include "HAL.h"
#include "screens.h"

#include "common.h"
#include "io_calls.h"
/* CONSTANTS ***********************************************************************/

/* GLOBAL VARIABLES ****************************************************************/

/* LOCAL VARIABLES *****************************************************************/

/* MACROS **************************************************************************/

/* LOCAL FUNCTIONS / INLINES *******************************************************/


/* FUNCTIONS ***********************************************************************/

void eve_calibrate(void)
{
  struct touchscreen_calibration calib;

  // If no store of calibration or current screen touch.
  if ((eve_calib_read(&calib) != 0) || (screen_touched())) {
	// Wait for end of touch.

	beep_1khz_p5sec(NORMAL_TONE);
	while (screen_touched());

    EVE_LIB_BeginCoProList();
    EVE_CMD_DLSTART();
    EVE_CLEAR_COLOR_RGB(HIGH_VIS_BG_COLOR);           //black background
	EVE_CLEAR(1,1,1);
	EVE_COLOR_RGB(WHITE_COLOR);                    //white text
	EVE_CMD_TEXT(EVE_DISP_WIDTH/2, EVE_DISP_HEIGHT/2,28, EVE_OPT_CENTERX | EVE_OPT_CENTERY,"Please tap on the dots");
	EVE_CMD_CALIBRATE(0);
	EVE_LIB_EndCoProList();
	EVE_LIB_AwaitCoProEmpty();

	calib.transform[0] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_A);	//  33000;    //
	calib.transform[1] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_B);	//  0;        //
	calib.transform[2] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_C);	//  -1000000; //
	calib.transform[3] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_D);  //  50;       //
	calib.transform[4] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_E);	//  -19650;   //
	calib.transform[5] = HAL_MemRead32(EVE_REG_TOUCH_TRANSFORM_F);	//  18700000; //

	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_A, calib.transform[0]);
	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_B, calib.transform[1]);
	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_C, calib.transform[2]);
	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_D, calib.transform[3]);
	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_E, calib.transform[4]);
	//HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_F, calib.transform[5]);
	eve_calib_write(&calib);
  }
  else{
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_A, calib.transform[0]);
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_B, calib.transform[1]);
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_C, calib.transform[2]);
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_D, calib.transform[3]);
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_E, calib.transform[4]);
	HAL_MemWrite32(EVE_REG_TOUCH_TRANSFORM_F, calib.transform[5]);
  }
}
