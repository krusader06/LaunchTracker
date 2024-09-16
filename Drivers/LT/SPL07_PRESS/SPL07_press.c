/** @file SPL07_press.c
 *
 * @brief Driver library for the SPL07-003 absolute pressure sensor
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "SPL07_press.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/


/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
spl07_handle_t spl07Handle = {0};

static I2C_HandleTypeDef *spl07i2c = NULL;		// I2C Handle

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/

static lt_err_t pressReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size);
static lt_err_t pressWriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/





/* I2C Read and Write Functions ************************************************/

/**
  * @brief  Low-Level Read function
  * @param  <regAddr> Address of the target register
  * @param  <dest> Pointer to the storage location of the read data
  * @param  <size> Number of bytes to read
  * @retval <lt_err_t> Returns an error code
  */
static lt_err_t pressReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size) {
	return HAL_I2C_Mem_Read(spl07i2c, PRESS_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, dest, size, PRESS_I2C_TIMEOUT);
}

/**
  * @brief  Low-Level write function
  * @param  <regAddr> Address of the target register
  * @param  <src> Pointer to the data to write
  * @param  <size> Number of bytes to write
  * @retval <lt_err_t> Returns an error code
  */
static lt_err_t pressWriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size) {
	return HAL_I2C_Mem_Write(spl07i2c, PRESS_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, src, size, PRESS_I2C_TIMEOUT);
}

/********************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/

/**
  * @brief  One-Line Description of the Function
  * @note   Document any notes if needed
  * @param  <paramName> List and describe input parameters
  * @retval <retvalName> List and describe return value
  */
lt_err_t spl07Init(I2C_HandleTypeDef *hi2c) {
	lt_err_t retVal = LT_OK;
	spl07i2c = hi2c;




	return retVal;
}

lt_err_t spl07ReadAltitude(void) {
	lt_err_t retVal = LT_OK;

	return retVal;
}

lt_err_t spl07SetGroundAltitude(void) {
	lt_err_t retVal = LT_OK;

	return retVal;
}

/*** end of file ***/
