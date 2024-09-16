/** @file SPL07_press.h
 *
 * @brief Driver library for the SPL07-003 absolute pressure sensor
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPL07_PRESS_H_
#define SPL07_PRESS_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "LT_App.h"
#include "SPL07_press_regs.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */
#define PRESS_I2C_ADDR				(0x76 << 1)			/*!< I2C  Address																			*/
#define PRESS_I2C_TIMEOUT			500					/*!< Timeout																				*/


/********************************************************************************
 * TYPES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Type
  * @note   Document any notes if needed
  */
typedef struct {
	uint32_t c00;
	uint32_t c10;
	uint16_t c20;
	uint16_t c30;
	uint16_t c40;
	uint16_t c01;
	uint16_t c11;
	uint16_t c21;
	uint16_t c31;
} spl07_press_coef_t;

typedef struct {
	uint16_t c0;
	uint16_t c1;
} spl07_temp_coef_t;

typedef struct {
	spl07_press_coef_t pressCoef;
	spl07_temp_coef_t tempCoef;
} spl07_calibration_t;

typedef struct {
	spl07_calibration_t calibration;
	float temperature;
	float pressure;
	float altitude;
	float groundAltitude;	// pressure sensor altitude at 0 above ground level
} spl07_handle_t;

// Instantiate the KX134 Structure
extern spl07_handle_t spl07Handle;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
lt_err_t spl07Init(I2C_HandleTypeDef *hi2c);

lt_err_t spl07ReadAltitude(void);

lt_err_t spl07SetGroundAltitude(void);


#endif // SPL07_PRESS_H_

/*** end of file ***/
