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


/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/


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

	// Register the LT Software Initialization Task
	UTIL_SEQ_RegTask(1<<LT_TASK_SW_INIT, UTIL_SEQ_RFU, ltSWInit);
	UTIL_SEQ_SetTask(1<<LT_TASK_SW_INIT, CFG_SCH_PRIO_0);
}

void ltSWInit(void) {

}

/*** end of file ***/
