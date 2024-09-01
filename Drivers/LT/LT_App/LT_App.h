/** @file LT_App.h
 *
 * @brief LaunchTracker application functions
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LT_APP_H_
#define LT_APP_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "main.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
#define FIRMWARE_MAJOR_VER			1									/*!< Firmware Revision - Encoded						*/
#define FIRMWARE_MINOR_VER			0
#define FIRMWARE_REV				0

#define HARDWARE_REV				""
#define HARDWARE_REV_LEN			16

#define LT_MANU						"LIGHT WIDOW"						/*!< Product Manufacturer - Encoded						*/
#define LT_MANU_LEN					16									/*!< Manufacturer String Length							*/

#define LT_SERIAL					""
#define LT_SERIAL_LEN				16

#define LT_ROCKETNAME				"TRACKER"
#define LT_ROCKETNAME_LEN			9


#define MAX_FLIGHT_LOG_COUNT		100

/* TIMERS ********************************************************************************************************************* */
#define TIMER_SECONDS(x)			(x) * (LSE_VALUE / CFG_RTCCLK_DIV)
#define BAT_CHECK_TIMER				TIMER_SECONDS(3600)					/* 1 hour battery check timer							*/
#define BAT_LOW_THRESHOLD			3010								/* Battery low voltage threshold (mv) to change activity*/

/* DEFAULT VALUES ************************************************************************************************************* */
#define LT_DEFAULT_UNIT				0									/* Default Unit = 0 = Imperial							*/
#define LT_DEFAULT_TIMEOUT			5									/* Default State Timeout = 5 minutes					*/

/********************************************************************************
 * TYPES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Type
  * @note   Document any notes if needed
  */
typedef enum {
	LT_OK,
	LT_ERR,
	LT_MOD_NOT_INIT,
	LT_BAD_PARAMETER,
} lt_err_t;

typedef enum {
	LT_STATE_IDLE,
	LT_STATE_READY,
	LT_STATE_FLIGHT,
	LT_STATE_IMMEDIATE_CAPTURE,
	LT_STATE_EXTENDED_CAPTURE,
} lt_state_t;

typedef struct {
	lt_state_t state;
} lt_handle_t;

/**** External Hardware References *********************************************/

extern I2C_HandleTypeDef hi2c1;

/**** External Global References ***********************************************/
extern lt_handle_t ltProcessHandle;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
void ltHWInit(void);
void ltSWInit(void);


#endif // LT_APP_H_

/*** end of file ***/
