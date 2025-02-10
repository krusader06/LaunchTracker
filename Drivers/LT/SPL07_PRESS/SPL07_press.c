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
#include "SPL07_press_regs.h"

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
static int32_t getTwosComplement(uint32_t raw, uint8_t length);

static lt_err_t pressReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size);
static lt_err_t pressWriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/


static int32_t getTwosComplement(uint32_t raw, uint8_t length) {
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    } else {
        return raw;
    }
}


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
	uint8_t tmpVal;

	// Sensor Reset
	uint8_t data = 0x09;
	retVal = pressWriteBytes(SPL07_RESET, &data, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	HAL_Delay(40);

	retVal = pressReadBytes(SPL07_MEAS_CFG, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	spl07_meas_cfg_bitfield_t status;
	status.all = tmpVal;

	// Check if coefficients are available
	if (status.bits.coef_rdy == 0) {
		return LT_MOD_NOT_INIT;
	}

	// Check if sensor initialization is complete
	if (status.bits.sensor_rdy == 0) {
		return LT_MOD_NOT_INIT;
	}

	// 1. Read the pressure calibration coefficients (c00, c10, c20, c30, c01, c11, and c21, c31, c40) from the Calibration Coefficient register.
	//   Note: The coefficients read from the coefficient register are 2's complement numbers.
	// Do the read of the coefficients in multiple parts, as the chip will return a read failure when trying to read all at once over I2C.
	uint8_t coef[21] = {0};
	uint8_t startReg = SPL07_COEF_C0;

	for (uint8_t i = 0; i < sizeof(coef); i++) {
		uint8_t readReg = startReg + i;
		uint8_t readVal;
		pressReadBytes(readReg, &readVal, 1);
		if (retVal != LT_OK) {
			return retVal;
		}
		coef[i] = readVal;
	}

	// See section 8.11, Calibration Coefficients (COEF), of datasheet
	// 0x11 c0 [3:0] + 0x10 c0 [11:4]
	spl07Handle.calibration.tempCoef.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);
    // 0x11 c1 [11:8] + 0x12 c1 [7:0]
	spl07Handle.calibration.tempCoef.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);
    // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
	spl07Handle.calibration.pressCoef.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);
    // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
	spl07Handle.calibration.pressCoef.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);
    // 0x18 c01 [15:8] + 0x19 c01 [7:0]
	spl07Handle.calibration.pressCoef.c01 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
    // 0x1A c11 [15:8] + 0x1B c11 [7:0]
	spl07Handle.calibration.pressCoef.c11 = getTwosComplement(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);
    // 0x1C c20 [15:8] + 0x1D c20 [7:0]
	spl07Handle.calibration.pressCoef.c20 = getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
    // 0x1E c21 [15:8] + 0x1F c21 [7:0]
	spl07Handle.calibration.pressCoef.c21 = getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
    // 0x20 c30 [15:8] + 0x21 c30 [7:0]
	spl07Handle.calibration.pressCoef.c30 = getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);
	// 0x23 c31 [3:0] + 0x22 c31 [11:4]
	spl07Handle.calibration.pressCoef.c31 = getTwosComplement(((uint32_t)coef[18] << 4) | (((uint32_t)coef[19] >> 4) & 0x0F), 12);
	// 0x23 c40 [11:8] + 0x24 c40 [7:0]
	spl07Handle.calibration.pressCoef.c40 = getTwosComplement((((uint32_t)coef[19] & 0x0F) << 8) | (uint32_t)coef[20], 12);

	// PRS_CFG: pressure measurement rate (32 Hz) and oversampling (16 time standard)
	spl07_prs_cfg_bitfield_t prsCfg;
	prsCfg.bits.pm_prc = 0x04; // 16x Oversampling Rate
	prsCfg.bits.pm_rate = 0x05; // 32x Measurements/Second
	tmpVal = prsCfg.all;
	retVal = pressWriteBytes(SPL07_PRS_CFG, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// TMP_CFG: temperature measurement rate (32 Hz) and oversampling (16 time standard)
	spl07_tmp_cfg_bitfield_t tmpCfg;
	tmpCfg.bits.tmp_prc = 0x04; // 16x Oversampling Rate
	tmpCfg.bits.tmp_rate = 0x05; // 32x Measurements/Second
	tmpVal = tmpCfg.all;
	retVal = pressWriteBytes(SPL07_TMP_CFG, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// CFG_REG: set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
	spl07_cfg_reg_bitfield_t cfgRes;
	cfgRes.bits.t_shift = 1; // Temperature result bit-shift
	cfgRes.bits.p_shift = 1; // Pressure result bit-shift
	tmpVal = cfgRes.all;
	retVal = pressWriteBytes(SPL07_CFG_REG, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// MEAS_CFG: Continuous pressure and temperature measurement
	spl07_meas_cfg_bitfield_t measCfg;
	measCfg.bits.meas_ctrl = 0x07; // Temperature result bit-shift
	tmpVal = measCfg.all;
	retVal = pressWriteBytes(SPL07_MEAS_CFG, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

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
