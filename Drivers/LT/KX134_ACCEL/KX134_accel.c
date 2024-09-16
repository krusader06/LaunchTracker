/** @file KX134_accel.c
 *
 * @brief Driver library for the KX134-1211 accelerometer
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */


/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "KX134_accel.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 *******************************************************************************/
kx134_handle_t kx134Handle = {0};

/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
static I2C_HandleTypeDef *kx134i2c = NULL;		// I2C Handle
static uint8_t i2cAddr = ACCEL_I2C_ADDR_PRIM;

// G Conversion Values
const double convRange8G = .000244;
const double convRange16G = .000488;
const double convRange32G = .000977;
const double convRange64G = .001953;

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/
static void accelInitDefaultSettings(void);
static lt_err_t accelSoftwareReset(void);
static bool accelEnable(bool enable);
static lt_err_t accelManualSleep(void);
static lt_err_t accelEnableTapEngine(bool enable);
static lt_err_t accelEnableTiltEngine(bool enable);
static lt_err_t accelEnableDataEngine(bool enable);
static void accelConvertOutputData(void);

static uint8_t accelGetTriggeredIrq(void);
static lt_err_t accelEnablePhyIrq(bool enable, kx134_irq_pins_t irqPin);
static lt_err_t accelEnableIrq(bool enable, kx134_irq_pins_t irqPin, kx134_irq_type_t irqType);
static lt_err_t accelConfigureIrqTap(kx134_irq_pins_t irqPin);
static lt_err_t accelConfigureIrqWakeup(kx134_irq_pins_t irqPin);
static lt_err_t accelConfigureIrq(kx134_irq_pins_t irqPin, uint8_t irqTypeMask);
static void accelResetIrq(void);
static void accelHandleIrqTap(kx134_irq_pins_t irqPin);
static void accelHandleIrqWakeup(kx134_irq_pins_t irqPin);

static lt_err_t accelReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size);
static lt_err_t accelWriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/

static void accelInitDefaultSettings(void) {
	// Selected G Range
	kx134Handle.settings.currentRange = DEFAULT_RANGE;
}


static lt_err_t accelSoftwareReset(void) {
	lt_err_t retVal = LT_OK;

	i2cAddr = ACCEL_I2C_ADDR_PRIM;

	// a. Write 0x00 to register 0x7F to start the reset sequence. Based on TN027 Power On Procedure.
	uint8_t data = 0x00;
	retVal = accelWriteBytes(0x7F, &data, 1);
	if (retVal != LT_OK) {
		// Flip the I2C address and try again.
		i2cAddr = ACCEL_I2C_ADDR_FLIP;
		retVal = accelWriteBytes(0x7F, &data, 1);
		if (retVal != LT_OK) {
			// Still no good. Just return.
			return retVal;
		}
	}

	// b. Write 0x00 to Control Register 2
	data = 0x00;
	retVal = accelWriteBytes(SFE_KX13X_CNTL2, &data, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// c. Write 0x80 to Control Register 2 to initiate software reset
	data = 0x80;
	retVal = accelWriteBytes(SFE_KX13X_CNTL2, &data, 1);
	if (retVal != LT_OK) {
		return retVal;
	}
	HAL_Delay(3); // software reset time

	// Reset the I2C address back to primary if needed.
	i2cAddr = ACCEL_I2C_ADDR_PRIM;

	// d. Read Who-Am-I. The value should be 0x46 (KX134)
	uint8_t whoami;
	retVal = accelReadBytes(SFE_KX13X_WHO_AM_I, &whoami, 1);
	if (whoami != 0x46 || retVal != LT_OK) {
		return retVal;
	}

	// e. Read COTR register. The value should be 0x55
	uint8_t cotr;
	retVal = accelReadBytes(SFE_KX13X_COTR, &cotr, 1);
	if (cotr != 0x55 || retVal != LT_OK) {
		return retVal;
	}

	return retVal;
}

static bool accelEnable(bool enable) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpVal;

	retVal = accelReadBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return false;
	}

	sfe_kx13x_cntl1_bitfield_t cntl1;
	cntl1.all = tmpVal;
	cntl1.bits.pc1 = enable; // Set Operating Mode (Power Control)
	cntl1.bits.res = enable; // Set Power Mode (Resolution)
	kx134Handle.settings.currentRange = cntl1.bits.gsel; // Update G Range
	tmpVal = cntl1.all;

	retVal = accelWriteBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return false;
	}

	return true;
}

static lt_err_t accelManualSleep(void) {
	uint8_t setting = 0x01;
	return accelWriteBytes(SFE_KX13X_CNTL5, &setting, 1);
}

static lt_err_t accelEnableTapEngine(bool enable) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpVal;

	retVal = accelReadBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	sfe_kx13x_cntl1_bitfield_t cntl1;
	cntl1.all = tmpVal;
	cntl1.bits.tdte = enable; // Set tap engine
	tmpVal = cntl1.all;

	return accelWriteBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
}

static lt_err_t accelEnableTiltEngine(bool enable) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpVal;

	retVal = accelReadBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	sfe_kx13x_cntl1_bitfield_t cntl1;
	cntl1.all = tmpVal;
	cntl1.bits.tpe = enable; // Set tilt engine
	tmpVal = cntl1.all;

	return accelWriteBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
}

static lt_err_t accelEnableDataEngine(bool enable) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpVal;

	retVal = accelReadBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	sfe_kx13x_cntl1_bitfield_t cntl1;
	cntl1.all = tmpVal;
	cntl1.bits.drdye = enable; // Set data ready engine
	tmpVal = cntl1.all;

	return accelWriteBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
}

static void accelConvertOutputData(void) {
	switch (kx134Handle.settings.currentRange) {
		case KX134_RANGE8G:
			kx134Handle.outputAccel.xData = (float)kx134Handle.rawOutputAccel.xData * convRange8G;
			kx134Handle.outputAccel.yData = (float)kx134Handle.rawOutputAccel.yData * convRange8G;
			kx134Handle.outputAccel.zData = (float)kx134Handle.rawOutputAccel.zData * convRange8G;
			break;
		case KX134_RANGE16G:
			kx134Handle.outputAccel.xData = (float)kx134Handle.rawOutputAccel.xData * convRange16G;
			kx134Handle.outputAccel.yData = (float)kx134Handle.rawOutputAccel.yData * convRange16G;
			kx134Handle.outputAccel.zData = (float)kx134Handle.rawOutputAccel.zData * convRange16G;
			break;
		case KX134_RANGE32G:
			kx134Handle.outputAccel.xData = (float)kx134Handle.rawOutputAccel.xData * convRange32G;
			kx134Handle.outputAccel.yData = (float)kx134Handle.rawOutputAccel.yData * convRange32G;
			kx134Handle.outputAccel.zData = (float)kx134Handle.rawOutputAccel.zData * convRange32G;
			break;
		case KX134_RANGE64G:
			kx134Handle.outputAccel.xData = (float)kx134Handle.rawOutputAccel.xData * convRange64G;
			kx134Handle.outputAccel.yData = (float)kx134Handle.rawOutputAccel.yData * convRange64G;
			kx134Handle.outputAccel.zData = (float)kx134Handle.rawOutputAccel.zData * convRange64G;
			break;
		default:
			break;
	}
}

/* IRQ Handle Functions ************************************************/

// Determines which event triggered an interrupt
static uint8_t accelGetTriggeredIrq(void) {
	uint8_t retVal = 0;
	uint8_t irqMask;
	retVal = accelReadBytes(SFE_KX13X_INS2, &irqMask, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	sfe_kx13x_ins2_bitfield_t ins2;
	ins2.all = irqMask;

	if (ins2.bits.tps) retVal |= KX134_IRQ_TILT;
	if (ins2.bits.tdts > 0) retVal |= KX134_IRQ_TAP;
	if (ins2.bits.drdy) retVal |= KX134_IRQ_DATAREADY;
	if (ins2.bits.wmi) retVal |= KX134_IRQ_WATERMARK;
	if (ins2.bits.bfi) retVal |= KX134_IRQ_BUFFERFULL;
	if (ins2.bits.ffs) retVal |= KX134_IRQ_FREEFALL;

	retVal = accelReadBytes(SFE_KX13X_INS3, &irqMask, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	sfe_kx13x_ins3_bitfield_t ins3;
	ins3.all = irqMask;

	if (ins3.bits.wufs) retVal |= KX134_IRQ_WAKEUP;
	if (ins3.bits.bts) retVal |= KX134_IRQ_BACKTOSLEEP;

	return retVal;
}

// enables or disables the physical pin
static lt_err_t accelEnablePhyIrq(bool enable, kx134_irq_pins_t irqPin) {
	lt_err_t retVal = LT_OK;

	// Determine the selected interrupt control register
	uint8_t selectedINC;
	if (irqPin > KX134_IRQ_INT2) {
		return LT_BAD_PARAMETER;
	} else if (irqPin == KX134_IRQ_INT1) {
		selectedINC = SFE_KX13X_INC1;
	} else {
		selectedINC = SFE_KX13X_INC5;
	}

	// Read the physical pin status
	uint8_t pinStatus;
	retVal = accelReadBytes(selectedINC, &pinStatus, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	if (enable && (pinStatus != 0x30)) {
		uint8_t incData = 0x30; // Physical INT Pin Enabled, Polarity High, Latched operation
		return accelWriteBytes(selectedINC, &incData, 1);
	} else if (!enable && (pinStatus != 0x10)) {
		uint8_t incData = 0x10; // Physical INT Pin Disabled, Polarity High
		return accelWriteBytes(selectedINC, &incData, 1);
	}

	return retVal;
}

// enables or disables the irq routing to the physical pin
static lt_err_t accelEnableIrq(bool enable, kx134_irq_pins_t irqPin, kx134_irq_type_t irqType) {
	lt_err_t retVal = LT_OK;

	// Determine the selected interrupt control register
	uint8_t selectedINC;
	if (irqPin > KX134_IRQ_INT2) {
		return LT_BAD_PARAMETER;
	} else if (irqPin == KX134_IRQ_INT1) {
		selectedINC = SFE_KX13X_INC4;
	} else {
		selectedINC = SFE_KX13X_INC6;
	}

	// Read the current irq status
	uint8_t irqStatus;
	retVal = accelReadBytes(selectedINC, &irqStatus, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Determine if the IRQ is already enabled
	bool irqEnabled = (irqStatus && irqType);
	if (enable && !irqEnabled) {
		irqStatus |= irqType;
		retVal = accelWriteBytes(selectedINC, &irqStatus, 1);
	} else if (!enable && irqEnabled) {
		irqStatus &= ~(irqType);
		retVal = accelWriteBytes(selectedINC, &irqStatus, 1);
	}

	// Save the new configuration to the handle
	kx134Handle.settings.configuredIrqMask[irqPin] = irqStatus;

	return retVal;
}

// configures a Tap/Double-Tap IRQ for a desired pin
static lt_err_t accelConfigureIrqTap(kx134_irq_pins_t irqPin) {
	lt_err_t retVal = LT_OK;
	uint8_t setting;

	accelEnable(false);

	// Enable Tap/Double-Tap from positive and negative directions of all three axis
	setting = 0x3F;
	retVal = accelWriteBytes(SFE_KX13X_INC3, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set output data rate for Direction Tap function to 400Hz
	setting = 0xA8;
	retVal = accelWriteBytes(SFE_KX13X_CNTL3, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Enable Interrupt on Single-Tap AND Double-Tap
	setting = 0x03;
	retVal = accelWriteBytes(SFE_KX13X_TDTRC, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set tap counter to 0.3s
	setting = 0x78;
	retVal = accelWriteBytes(SFE_KX13X_TDTC, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Tap Threshold High
	setting = 0x33;
	retVal = accelWriteBytes(SFE_KX13X_TTH, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Tap Threshold Low
	setting = 0x07;
	retVal = accelWriteBytes(SFE_KX13X_TTL, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set First Tap Detection (FTD) counter to 5ms
	setting = 0xA2;
	retVal = accelWriteBytes(SFE_KX13X_FTD, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Second Tap Detection (STD) counter to 90ms
	setting = 0x24;
	retVal = accelWriteBytes(SFE_KX13X_STD, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Tap (TLT) counter to 100ms
	setting = 0x28;
	retVal = accelWriteBytes(SFE_KX13X_TLT, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Tap Window (TWS) counter to 400ms
	setting = 0xA0;
	retVal = accelWriteBytes(SFE_KX13X_TWS, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Output the interrupt onto the physical pin
	accelEnablePhyIrq(true, irqPin);

	// Set the Tap/Double-Tap interrupt to be reported on the selected pin
	accelEnableIrq(true, irqPin, KX134_IRQ_TAP);

	// Enable Tap Engine
	accelEnableTapEngine(true);
	accelEnable(true);

	return retVal;
}

// configures a Wakeup IRQ for a desired pin
static lt_err_t accelConfigureIrqWakeup(kx134_irq_pins_t irqPin) {
	lt_err_t retVal = LT_OK;
	uint8_t setting;

	accelEnable(false);

	// Set output data control to 50Hz
	setting = 0x06;
	retVal = accelWriteBytes(SFE_KX13X_ODCNTL, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Output the interrupt onto the physical pin
	accelEnablePhyIrq(true, irqPin);

	// Set the Tap/Double-Tap interrupt to be reported on the selected pin
	accelEnableIrq(true, irqPin, KX134_IRQ_WAKEUP);

	// Enable Wakeup for all positive and negative directions
	setting = 0x3F;
	retVal = accelWriteBytes(SFE_KX13X_INC2, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set output data rate for the wakeup engine to 50Hz
	setting = 0xAE;
	retVal = accelWriteBytes(SFE_KX13X_CNTL3, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set counter mode to clear, threshold to relative, enable wakeup, disable back to sleep, set pulse reject to standard, set output data rate to 0.781Hz
	setting = 0x60;
	retVal = accelWriteBytes(SFE_KX13X_CNTL4, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Put sensor into manual sleep mode
	accelManualSleep();

	// Set wakeup motion time to 100ms (5 counts @ 50Hz)
	setting = 0x05;
	retVal = accelWriteBytes(SFE_KX13X_WUFC, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set wakeup motion g threshold to 2g's
	setting = 0x80;
	retVal = accelWriteBytes(SFE_KX13X_WUFTH, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set back to sleep motion g threshold to 0g's
	setting = 0x00;
	retVal = accelWriteBytes(SFE_KX13X_BTSWUFTH, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	accelEnableDataEngine(true);
	accelEnable(true);

	return retVal;
}

// takes care of enabling or disabling the irq's
static lt_err_t accelConfigureIrq(kx134_irq_pins_t irqPin, uint8_t irqTypeMask) {
	lt_err_t retVal = LT_OK;

	// Determine the selected interrupt control register
	uint8_t selectedINC;
	if (irqPin > KX134_IRQ_INT2) {
		return LT_BAD_PARAMETER;
	} else if (irqPin == KX134_IRQ_INT1) {
		selectedINC = SFE_KX13X_INC4;
	} else {
		selectedINC = SFE_KX13X_INC6;
	}

	if (irqTypeMask == 0x00) {
		// Disable the physical pin
		accelEnablePhyIrq(false, irqPin);
	}

	// Read the currently activated IRQs for the selected INT pin
	uint8_t curKxIrqMask;
	retVal = accelReadBytes(selectedINC, &curKxIrqMask, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Find any differences between the desired and current IRQs
	for (uint8_t i = 0; i < 8; i++) {
		kx134_irq_type_t irqType = (0x01 << i);
		bool kxIrqActive = ((irqType && curKxIrqMask) > 0);
		bool newItqActive = ((irqType && irqTypeMask) > 0);

		// IRQ needs to be disabled
		if (kxIrqActive && !newItqActive) {
			accelEnableIrq(false, irqPin, irqType);
		}

		// IRQ needs to be enabled
		if (!kxIrqActive && newItqActive) {
			switch (irqType) {
			case KX134_IRQ_TILT:
				// Not Supported Yet.
				break;
			case KX134_IRQ_WAKEUP:
				accelConfigureIrqWakeup(irqPin);
				break;
			case KX134_IRQ_TAP:
				accelConfigureIrqTap(irqPin);
				break;
			case KX134_IRQ_BACKTOSLEEP:
				// Not Supported Yet.
				break;
			case KX134_IRQ_DATAREADY:
				// Not Supported Yet.
				break;
			case KX134_IRQ_WATERMARK:
				// Not Supported Yet.
				break;
			case KX134_IRQ_BUFFERFULL:
				// Not Supported Yet.
				break;
			case KX134_IRQ_FREEFALL:
				// Not Supported Yet.
				break;
			default:
				break;
			}
		}
	}

	return retVal;
}

// Resets an IRQ so another interrupt can be triggered
static void accelResetIrq(void) {
	uint8_t setting = 0x00;
	accelWriteBytes(SFE_KX13X_INT_REL, &setting, 1);
}

// Handles a received IRQ from a tap event
static void accelHandleIrqTap(kx134_irq_pins_t irqPin) {
	// TODO - Trigger a sequencer function that updates the LT application state machine
	// Reset the interrupt
	accelResetIrq();
}

// Handles a received IRQ from a wakeup event
static void accelHandleIrqWakeup(kx134_irq_pins_t irqPin) {
	// TODO - Trigger a sequencer function that updates the LT application state machine
	accelResetIrq();
}

/* I2C Read and Write Functions ************************************************/

/**
  * @brief  Low-Level Read function
  * @param  <regAddr> Address of the target register
  * @param  <dest> Pointer to the storage location of the read data
  * @param  <size> Number of bytes to read
  * @retval <lt_err_t> Returns an error code
  */
static lt_err_t accelReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size) {
	return HAL_I2C_Mem_Read(kx134i2c, i2cAddr, regAddr, I2C_MEMADD_SIZE_8BIT, dest, size, ACCEL_I2C_TIMEOUT);
}

/**
  * @brief  Low-Level write function
  * @param  <regAddr> Address of the target register
  * @param  <src> Pointer to the data to write
  * @param  <size> Number of bytes to write
  * @retval <lt_err_t> Returns an error code
  */
static lt_err_t accelWriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size) {
	return HAL_I2C_Mem_Write(kx134i2c, i2cAddr, regAddr, I2C_MEMADD_SIZE_8BIT, src, size, ACCEL_I2C_TIMEOUT);
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
lt_err_t kx134Init(I2C_HandleTypeDef *hi2c) {
	lt_err_t retVal = LT_OK;

	// Bind the HW Interface
	kx134i2c = hi2c;

	accelInitDefaultSettings();

	if (accelSoftwareReset() != LT_OK) {
		return retVal;
	}

	// Setup Double-Tap detection on IRQ1
	if (accelConfigureIrq(KX134_IRQ_INT1, KX134_IRQ_TAP) != LT_OK) {
		return retVal;
	}

	// Setup Wakeup detection on IRQ2 (Launch Detection)
//	if (accelConfigureIrq(KX134_IRQ_INT2, KX134_IRQ_WAKEUP) != LT_OK) {
//		return retVal;
//	}

	return retVal;
}

lt_err_t kx134SetAccelRange(kx134_accel_range_t range) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpVal;

	if (range > KX134_RANGE64G) {
		return LT_BAD_PARAMETER;
	}

	retVal = accelReadBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Update the range selection
	sfe_kx13x_cntl1_bitfield_t cntl1;
	cntl1.all = tmpVal;
	cntl1.bits.gsel = range;
	tmpVal = cntl1.all;

	// Write the new value back
	retVal = accelWriteBytes(SFE_KX13X_CNTL1, &tmpVal, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Update our range
	kx134Handle.settings.currentRange = range;
	return retVal;
}

lt_err_t kx134StartAccelOutput(void) {
	lt_err_t retVal = LT_OK;
	uint8_t setting;

	accelEnable(false);

	// Set output data rate to 50Hz
	setting = 0x06;
	retVal = accelWriteBytes(SFE_KX13X_ODCNTL, &setting, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	retVal = accelEnableDataEngine(false); // This disables the "wakeup" irq, which isn't needed for streaming.
	accelEnable(true);

	return retVal;
}

lt_err_t kx134StopAccelOutput(void) {
	accelEnable(false);
	return LT_OK;
}

lt_err_t kx134ReadAccelData(void) {
	lt_err_t retVal = LT_OK;
	uint8_t rawAccelData[6] = {0};

	retVal = accelReadBytes(SFE_KX13X_XOUT_L, rawAccelData, TOTAL_ACCEL_DATA_16BIT);
	if (retVal != LT_OK) {
		return retVal;
	}

	kx134Handle.rawOutputAccel.xData = rawAccelData[0];
	kx134Handle.rawOutputAccel.xData |= (uint16_t)rawAccelData[1] << 8;

	kx134Handle.rawOutputAccel.yData = rawAccelData[2];
	kx134Handle.rawOutputAccel.yData |= (uint16_t)rawAccelData[3] << 8;

	kx134Handle.rawOutputAccel.zData = rawAccelData[4];
	kx134Handle.rawOutputAccel.zData |= (uint16_t)rawAccelData[5] << 8;

	accelConvertOutputData();

	return retVal;
}

void kx134IRQHandler(kx134_irq_pins_t irqPin) {
	// Determine what triggered the IRQ
	uint8_t irqMask = kx134Handle.settings.configuredIrqMask[irqPin];
	uint8_t irqTrigger = accelGetTriggeredIrq();

	for (uint8_t i = 0; i < 8; i++) {
		kx134_irq_type_t irqType = (0x01 << i);
		if ((irqType & irqMask & irqTrigger) == 0) {
			continue;
		}
		switch (irqType) {
		case KX134_IRQ_TILT:
			// Not Supported Yet.
			break;
		case KX134_IRQ_WAKEUP:
			accelHandleIrqWakeup(irqPin);
			break;
		case KX134_IRQ_TAP:
			accelHandleIrqTap(irqPin);
			break;
		case KX134_IRQ_BACKTOSLEEP:
			// Not Supported Yet.
			break;
		case KX134_IRQ_DATAREADY:
			// Not Supported Yet.
			break;
		case KX134_IRQ_WATERMARK:
			// Not Supported Yet.
			break;
		case KX134_IRQ_BUFFERFULL:
			// Not Supported Yet.
			break;
		case KX134_IRQ_FREEFALL:
			// Not Supported Yet.
			break;
		default:
			break;
		}

	}
}

/*** end of file ***/
