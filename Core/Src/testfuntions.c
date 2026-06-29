/*****************************************************************************
* @file			: testfunctions.c
* @short        : misc functions for testing the code.
******************************************************************************
*
* @details
*
*
*
******************************************************************************
*/

#include <stdint.h>
#include <string.h>
#include "common.h"
#include <stm32l0xx_hal.h>

extern I2C_HandleTypeDef hi2c3;

/**************************************
 * EraseNV1()
 * @detail
 * Erase all NV data
  */
void EraseNV1(void)
{
  uint8_t buf[64];
  uint8_t i,j;
  uint16_t addr;

  for (i=0; i<64; i++){
	buf[i] = 0xff;
  }

  addr = 0;

  for (j=0; j<232; j++){
	HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);

	HAL_Delay(EEPROM_WRITE_DELAY);
	addr = addr + 64;
  }

}/** End EraseNV1() **/

/**************************************
 * EraseNV2()
 * @detail
 * Erase all NV data except error, volume and calibration data
  */
void EraseNV2(void)
{
  uint8_t buf[64];
  uint8_t i;
  uint16_t addr;

  for (i=0; i<64; i++){
	buf[1] = 0xff;
  }
  addr = 0;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 64;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 128;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 192;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 256;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 576;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);
  addr = 640;
  HAL_I2C_Mem_Write(&hi2c3,0xae, addr, 2, buf, 64, 1000);
  HAL_Delay(EEPROM_WRITE_DELAY);


}/** End EraseNV2() **/
