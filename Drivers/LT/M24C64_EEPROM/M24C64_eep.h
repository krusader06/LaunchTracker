/** @file M24C64_eep.h
 *
 * @brief Drivers for the M24C64 EEPROM IC
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EEPROM_DRIVER_H_
#define EEPROM_DRIVER_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "stm32wbxx_hal.h"
#include "LT_App.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/

#define EEPROM_I2C_ADDR				(0x50 << 1)			/*!< I2C Address												*/
#define EEPROM_I2C_TIMEOUT			500					/*!< Timeout													*/

#define EEPROM_MEM_SIZE				0x1FFF				/*!< Total EEPROM Memory Size									*/
#define EEPROM_PAGE_SIZE			32					/*!< Number of Bytes per Page									*/

/* Memory Map Locations (64Kbit EEPROM)
 * |----  Item  ----|--  Size  --|----  Address  ----|------------------------------  Description  ---------------------------------|
 * | LT Settings    | Max 1024U  | 0x0000 - 0x03FF   | This is the main storage for the LT settings structure                       |
 * | Flight Logs    | Max 6896U  | 0x0400 - 0x1EEF   | This holds any data for all flight logs									    |
 * | Unallocated    | 256U       | 0x1EF0 - 0x1FEF   | Unallocated spacer bytes used for future expansion                           |
 * | Setting Size   | 2U         | 0x1FF0 - 0x1FF1   | Holds the LT settings struct size used to detect changes                     |
 * | Unallocated    | 14U        | 0x1FF2 - 0x1FFF   | Unallocated Bytes used for future expansion                                  |
 * |----------------|------------|-------------------|------------------------------------------------------------------------------|
 */

#define EEPROM_SETTINGS_ADDR		0x0000				/*!< LT Settings Address										*/
#define EEPROM_FLIGHTLOG_ADDR		0x0400				/*!< Flight Log Address											*/
#define EEPROM_SETTING_SIZE_ADDR	0x1FF0				/*!< Value used to test the eeprom								*/

/********************************************************************************
 * TYPES
 *******************************************************************************/

/**** Configuration Types ***************************************************/

typedef	struct {
	float maxGForce;										/*!< Maximum G Force Experienced							*/
	uint16_t ejectAltitude;									/*!< Ejection altitude										*/
	uint16_t maxAltitude;									/*!< Maximum altitude										*/
	uint16_t maxVelocity;									/*!< Maximum velocity										*/
	uint16_t descentVelocity;								/*!< Descent velocity										*/
	uint16_t thrustTime;									/*!< Thrust time in ms										*/
	uint16_t descentTime;									/*!< Descent time in ms										*/
	uint16_t coastToApogeeTime;								/*!< Coast to apogee time in ms								*/
	uint16_t apogeeToEjectTime;								/*!< Apogee to eject time in ms								*/
	uint16_t totalFlightTime;								/*!< Total flight time in ms								*/
} lt_flight_log_t;

typedef union {
	struct {
		lt_flight_log_t flightLog[MAX_FLIGHT_LOG_COUNT];
	};
	uint8_t payload;
} lt_flights_t;

/**
  * @brief  This structure holds all of the configuration data
  * @note	These parameters/settings are assignable. Some parameters are hard-coded such as model number and firmware rev.
  * 		Those parameters/settings are not included within the EEPROM or this structure.
  */
typedef enum {
	UNIT_IMPERIAL,
	UNIT_METRIC,
} unit_select_t;

typedef union {
	struct {
		/* Common Settings	*********************************************************************************************** */
		uint8_t hardwareRev[HARDWARE_REV_LEN];					/*!< Hardware Revision String								*/
		uint8_t serialNumber[LT_SERIAL_LEN];					/*!< Serial Number String									*/
		uint8_t rocketName[LT_ROCKETNAME_LEN];					/*!< Rocket Name String										*/
		unit_select_t unitSelect;								/*!< Measurement Unit Selection								*/
		uint16_t stateTimeout;									/*!< State machine timeout in minutes						*/
	};
	uint8_t payload;
} lt_settings_t;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
// Module Control
lt_err_t EEPROMInit(I2C_HandleTypeDef *hi2c);
lt_err_t EEPROMDeInit(void);
bool EEPROMIsModInit(void);
lt_err_t EEPROMTest(void);
void EEPROMReset(void);

lt_err_t readLTSettings(void);
lt_err_t writeLTSettings(void);

// Getter/Setter Functions
lt_err_t getLTSettings(lt_settings_t *ltSettings);
lt_err_t setLTSettings(lt_settings_t *ltSettings);



#endif // EEPROM_DRIVER_H_

/*** end of file ***/
