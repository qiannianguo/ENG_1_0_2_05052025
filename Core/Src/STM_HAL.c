/*****************************************************************************
* @file			: STM_HAL.c
* @short        : STM32 HAL calls
******************************************************************************
*
* @details
*
*
*
******************************************************************************
**/

#include <stm32l0xx_hal.h>

extern ADC_HandleTypeDef hadc;

/**************************************
  * @short		Stop analog conversion
  * @param		void
  * @retval  	void
  * @detail
  * Stop analog conversion
  */
 void stop_analog_conversion(void)
{
  HAL_ADC_Stop(&hadc);
}
