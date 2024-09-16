/** @file KX134_accel.h
 *
 * @brief Driver library for the KX134-1211 accelerometer
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KX134_ACCEL_H_
#define KX134_ACCEL_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "KX134_regs.h"
#include "LT_App.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */
#define ACCEL_I2C_ADDR_PRIM			(0x1E << 1)			/*!< I2C Primary Address																	*/
#define ACCEL_I2C_ADDR_FLIP			(0x1C << 1)			/*!< I2C Flipped Address																	*/
#define ACCEL_I2C_TIMEOUT			500					/*!< Timeout																				*/


#define DEFAULT_RANGE				KX134_RANGE8G

#define TOTAL_ACCEL_DATA_16BIT		6

/********************************************************************************
 * TYPES
 *******************************************************************************/
typedef enum {
	KX134_RANGE8G,
	KX134_RANGE16G,
	KX134_RANGE32G,
	KX134_RANGE64G,
} kx134_accel_range_t;

typedef enum {
	KX134_IRQ_INT1,
	KX134_IRQ_INT2,

	KX134_IRQ_PIN_CNT,
} kx134_irq_pins_t;

/**
  * @brief  IRQ Type Bit Definitions.
  * @note	IRQ Pin 1 = INC4 (0x25) | IRQ Pin 2 = INC6 (0x27)
  */
typedef enum {
	KX134_IRQ_NONE = 		(0x00),
	KX134_IRQ_TILT = 		(0x01 << 0),
	KX134_IRQ_WAKEUP = 		(0x01 << 1),
	KX134_IRQ_TAP = 		(0x01 << 2),
	KX134_IRQ_BACKTOSLEEP = (0x01 << 3),
	KX134_IRQ_DATAREADY = 	(0x01 << 4),
	KX134_IRQ_WATERMARK = 	(0x01 << 5),
	KX134_IRQ_BUFFERFULL = 	(0x01 << 6),
	KX134_IRQ_FREEFALL = 	(0x01 << 7),
	KX134_IRQ_ALL =			(0xFF),
} kx134_irq_type_t;


typedef struct {
	kx134_accel_range_t currentRange;
	uint8_t configuredIrqMask[KX134_IRQ_PIN_CNT];
} kx134_settings_t;

typedef struct {
    uint16_t xData;
    uint16_t yData;
    uint16_t zData;
} kx134_raw_data_t;

typedef struct {
    float xData;
    float yData;
    float zData;
} kx134_accel_data_t;

typedef struct {
	kx134_accel_data_t outputAccel;
	kx134_raw_data_t rawOutputAccel;
	kx134_settings_t settings;
} kx134_handle_t;

// Instantiate the KX134 Structure
extern kx134_handle_t kx134Handle;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
lt_err_t kx134Init(I2C_HandleTypeDef *hi2c);

lt_err_t kx134SetAccelRange(kx134_accel_range_t range);
lt_err_t kx134StartAccelOutput(void);
lt_err_t kx134StopAccelOutput(void);
lt_err_t kx134ReadAccelData(void);

void kx134IRQHandler(kx134_irq_pins_t irqPin);


#endif // KX134_ACCEL_H_

/*** end of file ***/
