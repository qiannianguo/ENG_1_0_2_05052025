/*****************************************************************************
* @file			: MCU.h
* @short        : Header for MCU.c file.
*
******************************************************************************
*
* @details
*
******************************************************************************
*/

#ifndef MCU_HEADER_H
#define	MCU_HEADER_H
#define PLATFORM_STM32
#include <stdint.h> // for Uint8/16/32 and Int8/16/32 data types

/****************************************************
 * @brief 	MCU specific initialisation
 * @details Must contain any MCU-specific initialisation. This will typically be
 * 	 		setting up the SPI bus, GPIOs and operating environment requirements.
 */
void MCU_Init(void);

/****************************************************
 * @brief 	MCU specific chip select enable
 * @details This function will pull the chip select line to the EVE low to
 * 	 		allow data transmission on the SPI bus.
 * 	 		The EVE requires chip select to toggle frequently.
 */
void MCU_CSlow(void);

/****************************************************
 * @brief 	MCU specific chip select deassert
 * @details This function will pull the chip select line to the EVE high to
 * 	 		prevent data transmission on the SPI bus.
 * 	 		The EVE requires chip select to toggle frequently.
 */
void MCU_CShigh(void);

/****************************************************
 * @brief 	MCU specific power down enable
 * @details This function will pull the power down line to the EVE low to
 * 	 		force the device into power down mode.
 * 	 		This will be done during EVE initialisation and can be done to allow
 * 	 		deep power saving.
 */
void MCU_PDlow(void);

/****************************************************
 * @brief 	MCU specific power down disable
 * @details This function will pull the power down line to the EVE high to
 * 	 		enable normal operation of the EVE.
 * 	 		This will be done during EVE initialisation and can be done to allow
 * 	 		recovery from deep power saving.
 */
void MCU_PDhigh(void);

/****************************************************
 * @brief 	MCU specific SPI write
 * @details Performs an SPI write of the data block and discards the data
 * 	 		received in response.
 * @param 	DataToWrite - pointer to buffer to write.
 * @param 	length - number of bytes to write.
 */
void MCU_SPIWrite(const uint8_t *DataToWrite, uint32_t length);

/****************************************************
 * @brief 	MCU specific SPI 8 bit read
 * @details Performs an SPI dummy write and returns the data received in
 *    	 	response.
 * @returns Data received from EVE.
 */
uint8_t MCU_SPIRead8(void);

/****************************************************
 * @brief 	MCU specific SPI 8 bit write
 * @details Performs an SPI write and discards the data received in
 *    	 	response.
 * @param 	Data to write to EVE.
 */
void MCU_SPIWrite8(uint8_t DataToWrite);

/****************************************************
 * @brief 	MCU specific SPI 16 bit read
 * @details Performs an SPI dummy write and returns the data received in
 *    	 	response.
 * @returns Data received from EVE.
 */
uint16_t MCU_SPIRead16(void);

/****************************************************
 * @brief 	MCU specific SPI 16 bit write
 * @details Performs an SPI write and discards the data received in
 *    	 	response.
 * @param 	Data to write to EVE.
 */
void MCU_SPIWrite16(uint16_t DataToWrite);

/****************************************************
 * @brief 	MCU specific SPI 24 bit read
 * @details Performs an SPI dummy write and returns the data received in
 *    	 	response.
 * @returns Data received from EVE.
 */
uint32_t MCU_SPIRead24(void);

/****************************************************
 * @brief 	MCU specific SPI 24 bit write
 * @details Performs an SPI write and discards the data received in
 *    	 	response.
 * @param 	Data to write to EVE.
 */
void MCU_SPIWrite24(uint32_t DataToWrite);

/****************************************************
 * @brief 	MCU specific SPI 32 bit read
 * @details Performs an SPI dummy write and returns the data received in
 *    	 	response.
 * @returns Data received from EVE.
 */
uint32_t MCU_SPIRead32(void);

/****************************************************
 * @brief 	MCU specific SPI 32 bit write
 * @details Performs an SPI write and discards the data received in
 *    	 	response.
 * @param 	Data to write to EVE.
 */
void MCU_SPIWrite32(uint32_t DataToWrite);

/****************************************************
 * @brief 	MCU specific 20 ms delay
 * @details Cause the MCU to idle or otherwise delay for a minimum of
 * 	 		20 milliseconds. This is used during initialisation to perform a
 * 	 		power down of the EVE for a controlled minimum period of time.
 */
void MCU_Delay_20ms(void);

/****************************************************
 * @brief 	MCU specific 500 ms delay
 * @details Cause the MCU to idle or otherwise delay for a minimum of
 * 	 		500 milliseconds. This is used during initialisation to perform a
 * 	 		power down of the EVE for a controlled minimum period of time.
 */
void MCU_Delay_500ms(void);

/****************************************************
 * @brief 	MCU specific byte swapping routines
 * @details EVE addresses from the HAL_SetReadAddress and HAL_SetWriteAddress
 * 	 		are sent in big-endian format. However, data for registers or memory
 * 	 		mapped areas are in little-endian format.
 */
//@{
uint16_t MCU_htobe16(uint16_t h);
uint32_t MCU_htobe32(uint32_t h);
uint16_t MCU_htole16(uint16_t h);
uint32_t MCU_htole32(uint32_t h);
uint16_t MCU_be16toh(uint16_t h);
uint32_t MCU_be32toh(uint32_t h);
uint16_t MCU_le16toh(uint16_t h);
uint32_t MCU_le32toh(uint32_t h);
//@}

#endif	/* MCU_HEADER_H */
