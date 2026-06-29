/*****************************************************************************
* @file			: EVE_MCU_STM32.c
* @short        : MCU level io for FT810
******************************************************************************
*
* @details
*
******************************************************************************
*/

#pragma message "Compiling " __FILE__ " for ST STM32"

#define bswap16(x) (((x) >> 8) | ((x) << 8))
#define bswap32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) \
                  | (((x) & 0x0000FF00) << 8) | ((x) << 24))

#include <main.h>
#include <EVE.h>
#include <EVE_config.h>
#include <FT8xx.h>
#include <HAL.h>
#include <MCU.h>
#include <string.h>
#include <stdint.h> // for Uint8/16/32 and Int8/16/32 data types

extern SPI_HandleTypeDef hspi1;


// ########################### GPIO CONTROL ####################################

// --------------------- Chip Select line low ----------------------------------
inline void MCU_CSlow(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //lo
}

// --------------------- Chip Select line high ---------------------------------
inline void MCU_CShigh(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //hi
}

// -------------------------- PD line low --------------------------------------
inline void MCU_PDlow(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //lo                                                     // PD# line low
}

// ------------------------- PD line high --------------------------------------
inline void MCU_PDhigh(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); //hi                                                      // PD# line high
}

// --------------------- SPI Send and Receive ----------------------------------
uint8_t MCU_SPIReadWrite8(uint8_t DataToWrite)
{
  uint8_t DataRead[4];
  uint8_t TxBuffer[4];
	
  TxBuffer[0] = DataToWrite;
		
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)TxBuffer, (uint8_t *)DataRead, 1, 5000);

	// Note that this call to the STM32 HAL returns a status value which can be checked as
	// shown below in orderto make the application more robust
	
	//switch(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)TxBuffer, (uint8_t *)DataRead, 1, 5000))
	//{
    //case HAL_OK:
      /* Communication is completed ___________________________________________ */
//    case HAL_TIMEOUT:
//      /* A Timeout Occur ______________________________________________________*/
//      /* Call Timeout Handler */
//      Timeout_Error_Handler();
//      break;
//      /* An Error Occur ______________________________________________________ */
//    case HAL_ERROR:
//      /* Call Timeout Handler */
//      Error_Handler();
//      break;
    //default:
      //break;
//  }
  return DataRead[0];
}

uint16_t MCU_SPIReadWrite16(uint16_t DataToWrite)
{
  uint16_t DataRead = 0;
  DataRead = MCU_SPIReadWrite8((DataToWrite) >> 8) << 8;
  DataRead |= MCU_SPIReadWrite8((DataToWrite) & 0xff);
  return MCU_be16toh(DataRead);
}

uint32_t MCU_SPIReadWrite24(uint32_t DataToWrite)
{
  uint32_t DataRead = 0;
  uint32_t temp;

  temp = (MCU_SPIReadWrite8((DataToWrite) >> 24));
  DataRead |= (temp<<24);

  temp = (MCU_SPIReadWrite8((DataToWrite) >> 16));
  DataRead |= (temp<<16);
  temp = (MCU_SPIReadWrite8((DataToWrite) >> 8));
  DataRead |= (temp<<8);

  return MCU_be32toh(DataRead);
}


uint32_t MCU_SPIReadWrite32(uint32_t DataToWrite)
{
  uint32_t DataRead = 0;
  uint32_t temp;

  temp = (MCU_SPIReadWrite8((DataToWrite) >> 24));
  DataRead |= (temp << 24);
  temp = (MCU_SPIReadWrite8((DataToWrite) >> 16));
  DataRead |= (temp << 16);
  DataRead |= (MCU_SPIReadWrite8((DataToWrite) >> 8) << 8);
  DataRead |= (MCU_SPIReadWrite8(DataToWrite) & 0xff);
 
  return MCU_be32toh(DataRead);
}

void MCU_Delay_20ms(void)
{
  HAL_Delay(20);
}

void MCU_Delay_500ms(void)
{
  uint8_t dly = 0;

  for(dly =0; dly < 100; dly++)
  {
    HAL_Delay(20);
  }
}



// --------------------- SPI Send and Receive ----------------------------------

uint8_t MCU_SPIRead8(void)
{
  uint8_t DataRead = 0;

  DataRead = MCU_SPIReadWrite8(0);
    
  return DataRead;
}

void MCU_SPIWrite8(uint8_t DataToWrite)
{
  MCU_SPIReadWrite8(DataToWrite);
}

uint16_t MCU_SPIRead16(void)
{
  uint16_t DataRead = 0;

  DataRead = MCU_SPIReadWrite16(0);

  return DataRead;
}

void MCU_SPIWrite16(uint16_t DataToWrite)
{
  MCU_SPIReadWrite16(DataToWrite);
}

uint32_t MCU_SPIRead24(void)
{
  uint32_t DataRead = 0;

  DataRead = MCU_SPIReadWrite24(0);

  return DataRead;
}

void MCU_SPIWrite24(uint32_t DataToWrite)
{
  MCU_SPIReadWrite24(DataToWrite);
}

uint32_t MCU_SPIRead32(void)
{
  uint32_t DataRead = 0;

  DataRead = MCU_SPIReadWrite32(0);

  return DataRead;
}

void MCU_SPIWrite32(uint32_t DataToWrite)
{
  MCU_SPIReadWrite32(DataToWrite);
}

void MCU_SPIWrite(const uint8_t *DataToWrite, uint32_t length)
{
  //spi_writen(SPIM, DataToWrite, length);

  uint16_t DataPointer = 0;
  DataPointer = 0;

  while(DataPointer < length)
  {
    MCU_SPIWrite8(DataToWrite[DataPointer]);      // Send data byte-by-byte from array
    DataPointer ++;
  }
}

uint16_t MCU_htobe16 (uint16_t h)
{
  return h;
}

uint32_t MCU_htobe32 (uint32_t h)
{
  return h;
}

uint16_t MCU_htole16 (uint16_t h)
{
  return bswap16(h);
}

uint32_t MCU_htole32 (uint32_t h)
{
  return bswap32(h);
}

uint16_t MCU_be16toh (uint16_t h)
{
  return h;
}

uint32_t MCU_be32toh (uint32_t h)
{
  return h;
}

uint16_t MCU_le16toh (uint16_t h)
{
  return bswap16(h);
}

uint32_t MCU_le32toh (uint32_t h)
{
  return bswap32(h);
}


