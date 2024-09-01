/** @file M24C64_eep.c
 *
 * @brief Drivers for the M24C64 EEPROM IC
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */


/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "M24C64_eep.h"

#include <string.h>
#include <math.h>

#include "stm32_seq.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/


/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
static I2C_HandleTypeDef *eep_i2c;
static bool eepromModInit;

static lt_settings_t ltSettingsLocal;			/*!< Main Settings Holder					*/

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/
static lt_err_t alignDefaultSettings(bool factoryReset);

static lt_err_t eepromRead(uint16_t memAddr, uint8_t *data, uint16_t size);
static lt_err_t eepromWrite(uint16_t memAddr, uint8_t *data, uint16_t size);
/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/

/**
 * @brief  	Aligns the default settings.
 * @note	This will not overwrite previous settings unless a factory reset has been requested.
 * @param  	<factoryReset> Factory reset request. True = Set all settings back to default.
 * @retval 	<lt_err_t> Returns an error code
 */
static lt_err_t alignDefaultSettings(bool factoryReset) {
	lt_err_t retVal = LT_OK;
	uint8_t readBytes[2];
	uint16_t storedSize;
	uint16_t rollingSize;

	// Get the stored eeprom settings size
	retVal = eepromRead(EEPROM_SETTING_SIZE_ADDR, readBytes, 2);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Convert the returned bytes to uint16_t
	storedSize = (uint16_t)(readBytes[1] << 8) | readBytes[0];

	// Uninitialized EEPROM will return 0xFFFF
	if (storedSize == 0xFFFF) {
		storedSize = 0;
	}

	// Set all values to default if a factory reset has been requested.
	if (factoryReset == true) {
		storedSize = 0;
	}

	// Return if the stored eeprom size matches the current settings size.
	if (storedSize == sizeof(lt_settings_t)) {
		return retVal;
	}

	// Iterate through and find the first setting that needs a default value.
	// First, add the payload size
	rollingSize = sizeof(ltSettingsLocal.payload);

	// Hardware Revision
	rollingSize += sizeof(ltSettingsLocal.hardwareRev);
	if (storedSize < rollingSize) {
		memset((char*)ltSettingsLocal.hardwareRev, 0, HARDWARE_REV_LEN);
		strlcpy((char*)ltSettingsLocal.hardwareRev, HARDWARE_REV, HARDWARE_REV_LEN);
	}

	// Serial Number
	rollingSize += sizeof(ltSettingsLocal.serialNumber);
	if (storedSize < rollingSize) {
		memset((char*)ltSettingsLocal.serialNumber, 0, LT_SERIAL_LEN);
		strlcpy((char*)ltSettingsLocal.serialNumber, LT_SERIAL, LT_SERIAL_LEN);
	}

	// Rocket Name
	rollingSize += sizeof(ltSettingsLocal.rocketName);
	if (storedSize < rollingSize) {
		memset((char*)ltSettingsLocal.rocketName, 0, LT_ROCKETNAME_LEN);
		strlcpy((char*)ltSettingsLocal.rocketName, LT_ROCKETNAME, LT_ROCKETNAME_LEN);
	}

	// Unit Select
	rollingSize += sizeof(ltSettingsLocal.unitSelect);
	if (storedSize < rollingSize) {
		ltSettingsLocal.unitSelect = LT_DEFAULT_UNIT;
	}

	// State Timeout Value
	rollingSize += sizeof(ltSettingsLocal.stateTimeout);
	if (storedSize < rollingSize) {
		ltSettingsLocal.stateTimeout = LT_DEFAULT_TIMEOUT;
	}

	// Write the settings back to the EEPROM
	retVal = eepromWrite(EEPROM_SETTINGS_ADDR, &ltSettingsLocal.payload, sizeof(lt_settings_t));
	if (retVal != LT_OK) {
		return retVal;
	}

	// Write the length of the settings
	uint8_t sendBytes[2];
	sendBytes[0] = (uint8_t)(rollingSize & 0x00FF);
	sendBytes[1] = (uint8_t)(rollingSize >> 8);
	retVal = eepromWrite(EEPROM_SETTING_SIZE_ADDR, sendBytes, 2);

	return retVal;
}


/*----- Read/Write Operations -------------------------------------------------*/
/**
 * @brief  Low Level I2C read driver
 * @param  <memAddr> Register address to read from
 * @param  <data> Pointer to the read data
 * @param  <size> Number of bytes to read
 * @retval <lt_err_t> Returns an error code
 */
static lt_err_t eepromRead(uint16_t memAddr, uint8_t *data, uint16_t size){
	HAL_StatusTypeDef halStatus;
	uint8_t retryCount = 10;
	do {
		halStatus = HAL_I2C_Mem_Read(eep_i2c, EEPROM_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_16BIT, data, size, EEPROM_I2C_TIMEOUT);
		HAL_Delay(1);
		retryCount--;
	} while (halStatus != HAL_OK && retryCount > 0);

	return halStatus;
}

/**
 * @brief  Low Level I2C write driver
 * @note	The EEPROM is split up into 256x32 byte pages. The data to be written cannot pass the page boundaries.
 * 		Care must be taken to stay within the page boundaries to avoid overwriting previous data.
 * @param  <memAddr> EEPROM address to write to
 * @param  <data> Pointer to the data to write
 * @param  <size> Number of bytes to send
 * @retval <lt_err_t> Returns an error code
 */
static lt_err_t eepromWrite(uint16_t memAddr, uint8_t *data, uint16_t size){
	lt_err_t retVal = LT_OK;

	HAL_StatusTypeDef halStatus;

	uint8_t offset = (uint8_t)(memAddr%EEPROM_PAGE_SIZE);
	uint16_t eepromPage = (uint16_t)(memAddr/EEPROM_PAGE_SIZE);
	uint16_t pagesToWrite;
	uint8_t currentPage;

	uint16_t newMemoryAddr, newSize;		// These are used when we have to send multiple pages
	uint8_t *newData;

	// Check that all memory locations exist
	if ((memAddr + size) > EEPROM_MEM_SIZE) {
		return LT_BAD_PARAMETER;
	}

	// Check whether or not the data flows over page boundaries
	if ((size + offset) > EEPROM_PAGE_SIZE) {
		// The data flows over page boundaries. Make separate writes to ensure that the data loads correctly.
		// Calculate the number of pages to write. Round up to the nearest integer.
		pagesToWrite = ceilf(((float)(size + offset) / EEPROM_PAGE_SIZE));

		// Write the first page
		uint8_t retryCount = 10;
		do {
			halStatus = HAL_I2C_Mem_Write(eep_i2c, EEPROM_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_16BIT, data, (uint16_t)(EEPROM_PAGE_SIZE - offset), EEPROM_I2C_TIMEOUT);
			HAL_Delay(1);
			retryCount--;
		} while (halStatus != HAL_OK && retryCount > 0);

		retVal = halStatus;
		if (retVal != LT_OK) {
			return retVal;
		}

		currentPage = 1;
		while (currentPage < pagesToWrite) {
			// Calculate the new Memory Address
			newMemoryAddr = (eepromPage + currentPage) * EEPROM_PAGE_SIZE;
			// Calculate the pointer to the data
			newData = data + (EEPROM_PAGE_SIZE - offset) + ((currentPage - 1) * EEPROM_PAGE_SIZE);
			// Calculate the new send size
			if (size - (currentPage * EEPROM_PAGE_SIZE) > EEPROM_PAGE_SIZE) {
				newSize = EEPROM_PAGE_SIZE;
			} else {
				newSize = size - (currentPage * EEPROM_PAGE_SIZE);
			}

			// The EEPROM may be busy writing the prior page. Keep trying until the EEPROM is ready.
			uint8_t retryCount = 10;
			do {
				halStatus = HAL_I2C_Mem_Write(eep_i2c, EEPROM_I2C_ADDR, newMemoryAddr, I2C_MEMADD_SIZE_16BIT, newData , newSize, EEPROM_I2C_TIMEOUT);
				HAL_Delay(1);
				retryCount--;
			} while (halStatus != HAL_OK && retryCount > 0);

			retVal = halStatus;
			if (retVal != LT_OK) {
				return retVal;
			}
			currentPage++;
		}
	} else {
		// No overflow. Just write the data.
		uint8_t retryCount = 10;
		do {
			halStatus = HAL_I2C_Mem_Write(eep_i2c, EEPROM_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_16BIT, data, size, EEPROM_I2C_TIMEOUT);
			HAL_Delay(1);
			retryCount--;
		} while (halStatus != HAL_OK && retryCount > 0);

		retVal = halStatus;
		if (retVal != LT_OK) {
			return retVal;
		}
	}

	return retVal;
}

/********************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/
/**
 * @brief  Initializes the module and establishes communication with the eeprom
 * @param  <hi2c> Pointer to the I2C Handle
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t EEPROMInit(I2C_HandleTypeDef *hi2c) {
	lt_err_t retVal = LT_OK;

	// Bind the I2C Bus
	eep_i2c = hi2c;

	retVal = readLTSettings();
	if (retVal != LT_OK) {
		return retVal;
	}

	retVal = alignDefaultSettings(false);
	if (retVal != LT_OK) {
		return retVal;
	}

	eepromModInit = true;

	return retVal;
}

/**
 * @brief  De-Initializes the module
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t EEPROMDeInit(void) {
	lt_err_t retVal = LT_OK;
	eep_i2c = NULL;
	eepromModInit = false;
	return retVal;
}

/**
 * @brief  Returns the module initialization status
 * @param  <NONE>
 * @retval <bool> Returns the module status
 */
bool EEPROMIsModInit(void) {
	return eepromModInit;
}

/**
 * @brief  	Resets the EEPROM during a factory reset.
 * @note	This will overwrite protected values!
 * @param  	<NONE>
 * @retval 	<NONE>
 */
void EEPROMReset(void) {
	alignDefaultSettings(true);
}

/**
 * @brief  Reads the FRS Settings from the EEPROM and updates the frsSettings structure.
 * @note	This reads directly from the EEPROM, but does not provide any information to the caller.
 * 		Use the setter/getter routines below to call information.
 * @param  NONE
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t readLTSettings(void) {
	lt_err_t retVal = LT_OK;

	retVal = eepromRead(EEPROM_SETTINGS_ADDR, &ltSettingsLocal.payload, sizeof(lt_settings_t));

	return retVal;
}

/**
 * @brief  Writes the FRS Settings to the EEPROM from the frsSettings structure.
 * @note	This just writes whatever is in the frsSettings structure and does not accept any info from the caller.
 * 		Use the setter/getter routines below to call information.
 * @param  NONE
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t writeLTSettings(void) {
	lt_err_t retVal = LT_OK;

	retVal = eepromWrite(EEPROM_SETTINGS_ADDR, &ltSettingsLocal.payload, sizeof(lt_settings_t));

	return retVal;
}

/* Generic Settings Setters and Getters ********************************************************** */
/**
 * @brief  Transfers the current FRS Settings into the caller's frsSettings structure
 * @note	This copies the settings, but does not talk to the EEPROM at all.
 * @param  [OUT] <frsSettings> Pointer to the settings structure
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t getLTSettings(lt_settings_t *frsSettings) {
	lt_err_t retVal = LT_OK;

	memcpy(&frsSettings->payload, &ltSettingsLocal.payload, sizeof(lt_settings_t));

	return retVal;
}

/**
 * @brief  Updates the frs settings via the caller
 * @param  [IN] <frsSettings> Pointer to the settings structure
 * @retval <lt_err_t> Returns an error code
 */
lt_err_t setLTSettings(lt_settings_t *frsSettings) {
	lt_err_t retVal = LT_OK;

	memcpy(&ltSettingsLocal.payload, &frsSettings->payload, sizeof(lt_settings_t));

	retVal = writeLTSettings();

	return retVal;
}


/*** end of file ***/
