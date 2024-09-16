/** @file LT_App.c
 *
 * @brief LaunchTracker application functions
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "LT_App.h"
#include "stm32_seq.h"

#include "adp5360.h"
#include "M24C64_eep.h"


/********************************************************************************
 * DEFINES
 *******************************************************************************/


/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
lt_handle_t ltProcessHandle = {0};

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/
void processStateIdle(void);
void processStateReady(void);
void processStateFlight(void);
void processStateImmediateCapture(void);
void processStateExtendedCapture(void);

void runLTProcess(void);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/

void processStateIdle(void) {
	// Turn off BLE
	// Put sensors into low power mode
}

void processStateReady(void) {
	// Turn on BLE
	// Prepare Flight Data Storage
	// Prepare Sensors
	// Wait for launch impulse or timeout
}

void processStateFlight(void) {
	// Gather sensor info

	// Capture flight events

	// Check state

	// Re-trigger LT Process
	UTIL_SEQ_SetTask(1<<LT_TASK_RUN_APPLICATION, CFG_SCH_PRIO_0);
}

void processStateImmediateCapture(void) {
	// Turn off sensors
	// Enable BLE fast beacon
	// Alarm Chirp Timing
	// Wait for BLE finish flight command or immediate capture timeout
}

void processStateExtendedCapture(void) {
	// Turn off sensors
	// Enable BLE low power beacon
	// Disable Alarm Chirp
	// Wait for BLE finish flight command
}

/**
  * @brief  One-Line Description of the Function
  * @note   Document any notes if needed
  * @param  <paramName> List and describe input parameters
  * @retval <retvalName> List and describe return value
  */
void runLTProcess(void) {
	switch(ltProcessHandle.state) {
		case LT_STATE_IDLE:
			processStateIdle();
			break;
		case LT_STATE_READY:
			processStateReady();
			break;
		case LT_STATE_FLIGHT:
			processStateFlight();
			break;
		case LT_STATE_IMMEDIATE_CAPTURE:
			processStateImmediateCapture();
			break;
		case LT_STATE_EXTENDED_CAPTURE:
			processStateExtendedCapture();
			break;
	}
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
void ltHWInit(void) {
	adp5360Init(&hi2c1);							/*!< Initialize the Power Module			*/
	EEPROMInit(&hi2c1);								/*!< Initialize the FRS Settings			*/
	kx134Init(&hi2c1);								/*!< Initialize the Accelerometer			*/

	// Register the LT Software Initialization Task
	UTIL_SEQ_RegTask(1<<LT_TASK_SW_INIT, UTIL_SEQ_RFU, ltSWInit);
	UTIL_SEQ_SetTask(1<<LT_TASK_SW_INIT, CFG_SCH_PRIO_0);
}

void ltSWInit(void) {
	ltProcessHandle.state = LT_STATE_IDLE;

	// Register and run the LT Application Task
	UTIL_SEQ_RegTask(1<<LT_TASK_RUN_APPLICATION, UTIL_SEQ_RFU, runLTProcess);
	UTIL_SEQ_SetTask(1<<LT_TASK_RUN_APPLICATION, CFG_SCH_PRIO_0);
}

/*** end of file ***/
