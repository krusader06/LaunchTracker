/** @file adp5360.c
 *
 * @brief Driver library for the Analog Devices ADP5360.
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2022.  All rights reserved.
 */

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "adp5360.h"

#include "stm32_seq.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
#if OVERRIDE_FUEL
/********************************************************************************
 * BATTERY VOLTAGE TO STATE-OF-CHARGE MAP WITH FUEL GAUGE OVERRIDE
 *******************************************************************************/
// This defines the characterized battery voltages so that the SOC is scaled
// correctly throughout the use of the matte box.
// Battery Voltage (V) = (2.5 + V_SOC * 0.008) (V)
uint16_t V_SOC_MAP[10][2] = {
		{0, 3204},				/*!< V_SOC_0   = 3.204V							*/
		{5, 3572},				/*!< V_SOC_5   = 3.572V							*/
		{11, 3692},				/*!< V_SOC_11  = 3.692V							*/
		{19, 3724},				/*!< V_SOC_19  = 3.724V							*/
		{28, 3756},				/*!< V_SOC_28  = 3.756V							*/
		{41, 3780},				/*!< V_SOC_41  = 3.780V							*/
		{55, 3812},				/*!< V_SOC_55  = 3.812V							*/
		{69, 3892},				/*!< V_SOC_69  = 3.892V							*/
		{84, 3980},				/*!< V_SOC_84  = 3.980V							*/
		{100, 4132}				/*!< V_SOC_100 = 4.132V							*/
};

#endif
/********************************************************************************
 * BATTERY VOLTAGE TO STATE-OF-CHARGE TABLE
 *******************************************************************************/
// This defines the characterized battery voltages so that the SOC is scaled
// correctly throughout the use of the matte box.
// NOTE: First element is the PMIC register for the SOC value. The second element is the value.
// Battery Voltage (V) = (2.5 + V_SOC * 0.008) (V)
uint8_t V_SOC_LUT[10][2] = {
		{0x16, 0x58},			/*!< V_SOC_0   = 3.204V							*/
		{0x17, 0x86},			/*!< V_SOC_5   = 3.572V							*/
		{0x18, 0x95},			/*!< V_SOC_11  = 3.692V							*/
		{0x19, 0x99},			/*!< V_SOC_19  = 3.724V							*/
		{0x1A, 0x9D},			/*!< V_SOC_28  = 3.756V							*/
		{0x1B, 0xA0},			/*!< V_SOC_41  = 3.780V							*/
		{0x1C, 0xA4},			/*!< V_SOC_55  = 3.812V							*/
		{0x1D, 0xAE},			/*!< V_SOC_69  = 3.892V							*/
		{0x1E, 0xB9},			/*!< V_SOC_84  = 3.980V							*/
		{0x1F, 0xCC}			/*!< V_SOC_100 = 4.132V							*/
};

/********************************************************************************
 * GLOBAL VARIABLES
 *******************************************************************************/
ADP5360_Handle_t adp5360;

/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
I2C_HandleTypeDef *adp5360i2c = NULL;		// I2C Handle
bool adp5360ModInit = false;				// Initialized Flag

static uint8_t periodicBatteryCheckTimerID;

/* 	This IRQ check flag is used during the battery check function.
	Since the FRS low light mode and the battery check both have a period of 1 hour,
	a race condition is possible where the battery check function triggers a reset
	of the low light timer. Because of this, we only want to disable the LEDs if the
	battery check is from an IRQ (charger status change) and not from the periodic
	battery check timer. */
static bool irqCheck = false;

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/
lt_err_t adpSetDefaultConfig(void);
lt_err_t adpResetSOC(void);
lt_err_t adpReadStatus(void);
lt_err_t adpClearInterrupts(void);
void LTCheckBatteryPrep(void);
uint8_t calculateSOC(uint16_t batVolt);

lt_err_t adp5360ReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size);
lt_err_t adp5360WriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size);
static lt_err_t recoverI2C(void);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/

/**
  * @brief  Sets the default operating configuration
  * @note	Change the default parameters in adp5360.h
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adpSetDefaultConfig(void) {
	lt_err_t retVal = LT_OK;

	uint8_t tmpData;

	// Set charge current to 320mA
	tmpData = CHG_CURRENT_DEFAULT;
	retVal = adp5360WriteBytes(CHG_CUR_SET, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Disable the Charge Timer
	tmpData = CHG_TIMER_EN_DEFAULT;
	retVal = adp5360WriteBytes(CHG_TIM_SET, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the Thermistor Current
	tmpData = THR_CUR_DEFAULT;
	retVal = adp5360WriteBytes(BAT_THERM_CTRL, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Disable Battery Protection
	tmpData = BAT_CTRL_DEFAULT;
	retVal = adp5360WriteBytes(BAT_PROT_CTRL, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Battery Undervoltage Protection to 2.8V
	tmpData = BAT_UV_DEFAULT;
	retVal = adp5360WriteBytes(BAT_PROT_UV_SET, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set Battery Charge Over-Current Protection to 400mA
	tmpData = BAT_OC_CHG_DEFAULT;
	retVal = adp5360WriteBytes(BAT_PROT_OC_CHG_SET, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the SOC Voltage Scaling
	for (uint8_t i = 0; i < 10; i++) {
		retVal = adp5360WriteBytes(V_SOC_LUT[i][0], &V_SOC_LUT[i][1], 1);
		if (retVal != LT_OK) {
			return retVal;
		}
	}

	// Set the Battery Capacity
	tmpData = BAT_CAP_DEFAULT;
	retVal = adp5360WriteBytes(BAT_CAP, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Enable the Fuel Gauge
	tmpData = FG_EN_DEFAULT;
	retVal = adp5360WriteBytes(FUEL_GAUGE_MODE, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

//	// Set the Buck Converter Voltage. Default: 3.05V
//	tmpData = BUCK_VOUT_DEFAULT;
//	retVal = adp5360WriteBytes(BCK_OUT_VOLT_SET, &tmpData, 1);
//	if (retVal != LT_OK) {
//		return retVal;
//	}

	// Set the Buck Converter Configuration
	tmpData = BUCK_CFG_DEFAULT;
	retVal = adp5360WriteBytes(BCK_CFG, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

//	// Set the Buck-Boost Converter Voltage. Default: 5.0V
//	tmpData = BUCKBST_VOUT_DEFAULT;
//	retVal = adp5360WriteBytes(BCK_BST_OUT_VOLT_SET, &tmpData, 1);
//	if (retVal != LT_OK) {
//		return retVal;
//	}

	// Set the PGOOD1 Pin Mask. Disable all output.
	tmpData = PGOOD1_MASK_DEFAULT;
	retVal = adp5360WriteBytes(PGOOD1_MASK, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the PGOOD2 Pin Mask. Disable all output.
	tmpData = PGOOD2_MASK_DEFAULT;
	retVal = adp5360WriteBytes(PGOOD2_MASK, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

//	// Set the Buck-Boost Converter Configuration
//	tmpData = BUCKBST_CFG_DEFAULT;
//	retVal = adp5360WriteBytes(BCK_BST_CFG, &tmpData, 1);
//	if (retVal != LT_OK) {
//		return retVal;
//	}

	// Set the IRQ1 Register
	tmpData = ADP_IRQ1_DEFAULT;
	retVal = adp5360WriteBytes(IRQ_EN1, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the IRQ2 Register
	tmpData = ADP_IRQ2_DEFAULT;
	retVal = adp5360WriteBytes(IRQ_EN2, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	return retVal;
}

/**
  * @brief  Resets the SOC registers
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adpResetSOC(void) {
	lt_err_t retVal = LT_OK;

	uint8_t tmpData;

	tmpData = 0x80;
	retVal = adp5360WriteBytes(SOC_RESET, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	tmpData = 0;
	retVal = adp5360WriteBytes(SOC_RESET, &tmpData, 1);

	return retVal;
}


/**
  * @brief  Reads the status registers and updates the main handle
  * @param  <adp5360> Pointer to the ADP5360 Handle
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adpReadStatus(void) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpData;

	// Read the Charger Status1 Register
	retVal = adp5360ReadBytes(CHG_STS1, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.chargeStatus.chgStatus = (tmpData & ADP_CHG_STS1_CHARGER_STATUS) >> 0;
	adp5360.chargeStatus.vbus_ilim = (tmpData & ADP_CHG_STS1_VBUS_ILIM) >> 5;
	adp5360.chargeStatus.adpichg = (tmpData & ADP_CHG_STS1_ADPICHG) >> 6;
	adp5360.chargeStatus.vbus_ov = (tmpData & ADP_CHG_STS1_VBUS_OV) >> 7;

	// Read the Charger Status2 Register
	retVal = adp5360ReadBytes(CHG_STS2, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.chargeStatus.batChgStatus = (tmpData & ADP_CHG_STS2_BAT_CHG_STATUS) >> 0;
	adp5360.chargeStatus.batUVStatus = (tmpData & ADP_CHG_STS2_BAT_UV_STATUS) >> 3;
	adp5360.chargeStatus.batOVStatus = (tmpData & ADP_CHG_STS2_BAT_OV_STATUS) >> 4;
	adp5360.chargeStatus.thrStatus = (tmpData & ADP_CHG_STS2_THR_STATUS) >> 5;

	// Read the Power Status Register
	retVal = adp5360ReadBytes(PGOOD_STATUS, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.powerStatus.vOut1OK = (tmpData & ADP_PGOOD_STATUS_VOUT1OK) >> 0;
	adp5360.powerStatus.vOut2OK = (tmpData & ADP_PGOOD_STATUS_VOUT2OK) >> 1;
	adp5360.powerStatus.batOK = (tmpData & ADP_PGOOD_STATUS_BATOK) >> 2;
	adp5360.powerStatus.vBusOK = (tmpData & ADP_PGOOD_STATUS_VBUSOK) >> 3;
	adp5360.powerStatus.chgComplete = (tmpData & ADP_PGOOD_STATUS_CHG_CMPLT) >> 4;
	adp5360.powerStatus.mrPress = (tmpData & ADP_PGOOD_STATUS_MR_PRESS) >> 5;

	// Read the Battery Voltage
	uint8_t vbathigh;
	uint8_t vbatlow;

	retVal = adp5360ReadBytes(VBAT_READ_H, &vbathigh, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	retVal = adp5360ReadBytes(VBAT_READ_L, &vbatlow, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.batteryVoltage = (uint16_t)((vbathigh << 8) | vbatlow) >> 3;

	// Read the State of Charge
#if OVERRIDE_FUEL
	adp5360.stateOfCharge = calculateSOC(adp5360.batteryVoltage);
#else
	retVal = adp5360ReadBytes(BAT_SOC, &tmpData, 1);
		if (retVal != LT_OK) {
			return retVal;
		}
	// Store the bytes
	adp5360.stateOfCharge = (tmpData & ADP_BAT_SOC) >> 0;
#endif

	return retVal;
}

/**
  * @brief  Clears any pending interrupts
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adpClearInterrupts(void) {
	lt_err_t retVal = LT_OK;

	uint8_t tmpData[2] = {0xFF, 0xFF};

	// Clear the FLAG1 Register
	retVal = adp5360ReadBytes(IRQ_FLAG1, tmpData, 2);
	if (retVal != LT_OK) {
		return retVal;
	}

	return retVal;
}

/**
  * @brief  Prepare to check the battery status.
  * @note	This is called when the RTC wakes up for a battery check.
  * The FRSCheckBattery function requires access to spi which needs to be initialized during the PostIdle.
  * This function is called from the Interrupt vector, so hardware SPI's aren't initialized yet.
  * @param  <timerID> Timer ID
  * @retval <NONE>
  */
void LTCheckBatteryPrep(void) {
	UTIL_SEQ_SetTask(1<<LT_TASK_CHECK_BATTERY, CFG_SCH_PRIO_0);
}

#if OVERRIDE_FUEL
/**
  * @brief  Calculates SOC based on battery voltage.
  * @note	This uses the V_SOC_MAP to scale the value correctly.
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
uint8_t calculateSOC(uint16_t batVolt) {

	// Find base soc level
	uint8_t socLevel;
	uint8_t levelNum = sizeof(V_SOC_MAP) / sizeof(V_SOC_MAP[0]);

	if (batVolt <= V_SOC_MAP[0][1]) {
		// Lowest Level
		return 0;
	} else if (batVolt >= V_SOC_MAP[levelNum - 1][1]) {
		// Highest Level
		return 100;
	} else {
		// Find Level
		for (socLevel = 0; socLevel < levelNum; socLevel++) {
			if (batVolt < V_SOC_MAP[socLevel + 1][1]) {
				break;
			}
		}
	}

	uint16_t voltLow = V_SOC_MAP[socLevel][1];
	uint16_t voltHigh = V_SOC_MAP[socLevel + 1][1];
	uint8_t socLow = V_SOC_MAP[socLevel][0];
	uint8_t socHigh = V_SOC_MAP[socLevel + 1][0];
	uint8_t batSOC;

	float mapCalc = 0;
	mapCalc = (float)(batVolt - voltLow) / (float)(voltHigh - voltLow);
	mapCalc *= (socHigh - socLow);
	mapCalc += socLow;

	batSOC = (uint8_t)mapCalc;

	return batSOC;
}
#endif

/* I2C Read and Write Functions ************************************************/

/**
  * @brief  Low-Level Read function
  * @param  <regAddr> Address of the target register
  * @param  <dest> Pointer to the storage location of the read data
  * @param  <size> Number of bytes to read
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360ReadBytes(uint8_t regAddr, uint8_t *dest, uint8_t size) {
	uint8_t retryCount;
	HAL_StatusTypeDef status;
	lt_err_t retVal = LT_OK;

	retryCount = 2;

	do {
		status = HAL_I2C_Mem_Read(adp5360i2c, ADP_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, dest, size, ADP_I2C_TIMEOUT);
		if (status != HAL_OK) {
			// Attempt to Recover
			recoverI2C();
		}
		retryCount--;
	} while ((status != HAL_OK) && (retryCount > 0));

	return retVal;
}

/**
  * @brief  Low-Level write function
  * @param  <regAddr> Address of the target register
  * @param  <src> Pointer to the data to write
  * @param  <size> Number of bytes to write
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360WriteBytes(uint8_t regAddr, uint8_t *src, uint8_t size) {
	uint8_t retryCount;
	HAL_StatusTypeDef status;
	lt_err_t retVal = LT_OK;

	retryCount = 2;

	do {
		status = HAL_I2C_Mem_Write(adp5360i2c, ADP_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, src, size, ADP_I2C_TIMEOUT);
		if (status != HAL_OK) {
			// Attempt to Recover
			recoverI2C();
		}
		retryCount--;
	} while ((status != HAL_OK) && (retryCount > 0));

	return retVal;
}

/**
  * @brief  I2C Recovery routine.
  * @note	This attempts to recover from an I2C HAL Timeout error by resetting the I2C HW.
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
static lt_err_t recoverI2C(void) {
	// The bus is tied up for some reason. Try to re-initialize.
	HAL_I2C_DeInit(adp5360i2c);
	return HAL_I2C_Init(adp5360i2c);
}


/********************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/

/**
  * @brief  Initializes the ADP5360 Module
  * @param	<hi2c>	  Pointer to the hardware I2C Handler
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360Init(I2C_HandleTypeDef *hi2c) {
	lt_err_t retVal = LT_OK;

	// Bind the HW Interface
	adp5360i2c = hi2c;

	// Test the HW Interface
	retVal = adp5360Test();
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the default configuration
	retVal = adpSetDefaultConfig();
	if (retVal != LT_OK) {
		return retVal;
	}

	// Set the default configuration
	retVal = adp5360SwitchBuckMode(SELECTED_BUCK_MODE);
	if (retVal != LT_OK) {
		return retVal;
	}

	// Read the status
	retVal = adpReadStatus();
	if (retVal != LT_OK) {
		return retVal;
	}

	// Clear the interrupts
	retVal = adpClearInterrupts();
	if (retVal != LT_OK) {
		return retVal;
	}

	// Register the Check Battery Task
	UTIL_SEQ_RegTask(1<<LT_TASK_CHECK_BATTERY, UTIL_SEQ_RFU, checkBattery);

	// Register the PMIC IRQ callback with the scheduler
	UTIL_SEQ_RegTask(1<<LT_TASK_CHECK_PMIC, UTIL_SEQ_RFU, adp5360IRQHandler);

	// Create a timer to wake up the system every (hour) in order to do a quick battery check.
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &periodicBatteryCheckTimerID, hw_ts_Repeated, LTCheckBatteryPrep);
	HW_TS_Start(periodicBatteryCheckTimerID, BAT_CHECK_TIMER);

	// Module is initialized
	adp5360ModInit = true;

	return retVal;
}

/**
  * @brief  De-Initializes the Module
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360DeInit(void) {
	lt_err_t retVal = LT_OK;

	adp5360i2c = NULL;
	adp5360ModInit = false;

	return retVal;
}

/**
  * @brief  Tests the HW Interface to make sure the IC communicates.
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360Test(void) {
	lt_err_t retVal = LT_OK;

	uint8_t tmpData;
	// Read the Manufacturer and Model
	retVal = adp5360ReadBytes(MFG_MODEL_ID, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.hardwareDetails.adpModel = (tmpData & ADP_MFG_MODEL_ID_MODEL) >> 0;
	adp5360.hardwareDetails.adpManu = (tmpData & ADP_MFG_MODEL_ID_MANUF) >> 4;

	// Read the Silicon Revision
	retVal = adp5360ReadBytes(SILICON_REV, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	adp5360.hardwareDetails.adpSiRev = (tmpData & ADP_SILICON_REV) >> 0;

	return retVal;
}

/**
  * @brief  Returns the module initialization status
  * @param  <NONE>
  * @retval <bool> Returns the module initialization status
  */
bool adp5360IsModInit(void) {
	return adp5360ModInit;
}

/**
  * @brief  Switch Buck Mode
  * @param  <mode> Mode select - Hysterisis or FPWM mode
  * @note		Hysterisis mode is lower power, but may be prone to brown-outs
  * 			FPWM mode is more stable.
  * @retval <bool> Returns the module initialization status
  */
lt_err_t adp5360SwitchBuckMode(adp_buck_mode_t mode) {
	lt_err_t retVal = LT_OK;
	uint8_t tmpData;

	// Read the Manufacturer and Model
	retVal = adp5360ReadBytes(BCK_CFG, &tmpData, 1);
	if (retVal != LT_OK) {
		return retVal;
	}

	switch (mode) {
		case REG_BUCK_HYST:
			// Set the Buck Mode to FPWM
			tmpData &= ~ADP_BCK_CFG_BUCK_MODE;
			retVal = adp5360WriteBytes(BCK_CFG, &tmpData, 1);
			if (retVal != LT_OK) {
				return retVal;
			}
			break;
		case REG_BUCK_FPWM:
			// Set the Buck Mode to FPWM
			tmpData |= ADP_BCK_CFG_BUCK_MODE;
			retVal = adp5360WriteBytes(BCK_CFG, &tmpData, 1);
			if (retVal != LT_OK) {
				return retVal;
			}
			break;
	}

	return retVal;
}

/**
  * @brief  Enables the 5V Power Rail
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360Enable5V(void) {
	lt_err_t retVal = LT_OK;

	// Check that the module is initialized
	if (!adp5360ModInit) {
		return LT_MOD_NOT_INIT;
	}

	// Enable the 5V Line
#if (ADP_EN_5V_GPIO_Port != NULL)
	HAL_GPIO_WritePin(ADP_EN_5V_GPIO_Port, ADP_EN_5V_Pin, 1);
	HAL_Delay(5);
#endif

	return retVal;
}

/**
  * @brief  Disables the 5V Power Rail
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360Disable5V(void) {
	lt_err_t retVal = LT_OK;

	// Check that the module is initialized
	if (!adp5360ModInit) {
		return LT_MOD_NOT_INIT;
	}

	// Disable the 5V Line
#if (ADP_EN_5V_GPIO_Port != NULL)
	HAL_GPIO_WritePin(ADP_EN_5V_GPIO_Port, ADP_EN_5V_Pin, 0);
#endif

	return retVal;
}

/**
  * @brief  Reads and updates the charger and power status
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360ReadStatus(void) {
	return adpReadStatus();
}


/**
  * @brief  Resets and refreshes the battery state of charge
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360RefreshSOC(void) {
	lt_err_t retVal = LT_OK;

	// This function resets the SOC, then waits until the PMIC gets a new SOC value.
	retVal = adpResetSOC();
	if (retVal != LT_OK) {
		return retVal;
	}

	adpReadStatus();

	uint16_t timeout = 0;
	while ((adp5360.batteryVoltage < BAT_LOW_THRESHOLD) && (timeout < 1000)) {
		adpReadStatus();
		timeout++;
		HAL_Delay(5);
	}

	return retVal;
}

/**
  * @brief  Puts the device into Shipment Mode
  * @note	This sets the EN_SHIPMODE bit (address: 0x36)
  * @param  <NONE>
  * @retval <lt_err_t> Returns an error code
  */
lt_err_t adp5360EnableShipMode(void) {
	// Set the EN_SHIPMODE bit within SHIPMODE register, address: 0x36
	uint8_t tmpData = ADP_EN_SHIPMODE;
	return adp5360WriteBytes(SHIPMODE, &tmpData, 1);
}

/**
  * @brief  Checks the state of the battery and updates the display if needed.
  * @note	This is the periodic check so that the display always shows the correct SOC.
  * @note	This is fired once every hour or after a charger status update.
  * @param  <NONE>
  * @retval <NONE>
  */
void checkBattery(void) {
	static bool previousVBusSts = false;

	HAL_Delay(1000);	// Allow PMIC to grab the latest voltage...
	adpReadStatus();

	// Update the BLE Battery Level Characteristic
	UTIL_SEQ_SetTask(1<<LT_TASK_SEND_BAT_LEVEL, CFG_SCH_PRIO_0);

	// Handle dead battery context switching
	UTIL_SEQ_SetTask(1<<LT_TASK_HANDLE_DEAD_BAT, CFG_SCH_PRIO_0);

 	// Detect charger insert
 	if (previousVBusSts != adp5360.powerStatus.vBusOK) {
 		if (adp5360.powerStatus.vBusOK) {
 			// Charger was just inserted
// 			UTIL_SEQ_SetTask(1<<FRS_TASK_ACTIVATE_FRS, CFG_SCH_PRIO_0);
 		}
 		previousVBusSts = adp5360.powerStatus.vBusOK;
 	}
}

/**
  * @brief  ADP5360 IRQ Handler
  * @note	This gets called from the sequencer after an IRQ is triggered
  * @param  <NONE>
  * @retval <NONE>
  */
void adp5360IRQHandler(void) {
	lt_err_t retVal = LT_OK;

	uint8_t intFlag1, intFlag2;
	static adp_charger_status_t previousChargeState;

	// Read the FLAG1 interrupts
	retVal = adp5360ReadBytes(IRQ_FLAG1, &intFlag1, 1);
	if (retVal != LT_OK) {
		return;
	}

	// Read the FLAG2 interrupts
	retVal = adp5360ReadBytes(IRQ_FLAG2, &intFlag2, 1);
	if (retVal != LT_OK) {
		return;
	}

	// FLAG1 Interrupts
	if (intFlag1 & ADP_IRQ_FLAG1_VBUS_INT) {
		// Interrupt Due To VBUS Voltage Threshold
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_CHG_INT) >> 1) {
		// Interrupt Due To Charger Mode Change
		retVal = adpReadStatus();
		if (retVal != LT_OK) {
			return;
		}

		// We want to update the display only if the charger turns on or off.
		// There is a point where the charger will oscillate between constant voltage and constant current
		// We don't want to update the display during this period.
		bool updateOK = true;
		if ((previousChargeState == STS_CHG_FAST_CC) || (previousChargeState == STS_CHG_FAST_CV)) {
			if ((adp5360.chargeStatus.chgStatus == STS_CHG_FAST_CC) || (adp5360.chargeStatus.chgStatus == STS_CHG_FAST_CV)) {
				updateOK = false;
			}
		}

		if (updateOK) {
			irqCheck = true;
			UTIL_SEQ_SetTask(1<<LT_TASK_CHECK_BATTERY, CFG_SCH_PRIO_0);
		}

		previousChargeState = adp5360.chargeStatus.chgStatus;
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_BAT_INT) >> 2) {
		// Interrupt Due To Battery Voltage Threshold
		// NOTE - NOT ENABLED
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_THR_INT) >> 3) {
		// Interrupt Due To THR Temperature Threshold
		// NOTE - NOT ENABLED
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_BATPRO_INT) >> 4) {
		// Interrupt Due To Battery Fault
		// NOTE - NOT ENABLED
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_ADPICHG_INT) >> 5) {
		// Interrupt Due To VBUS Input Current Limit Adaptive Regulation
		// NOTE - NOT ENABLED
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_SOCACM_INT) >> 6) {
		// Interrupt Due To State of Charge Accumulation Overflow
		// NOTE - NOT ENABLED
	}

	if ((intFlag1 & ADP_IRQ_FLAG1_SOCLOW_INT) >> 7) {
		// Interrupt Due To Battery Low State of Charge
		// This will fire when the battery reaches 6% SOC
		retVal = adpReadStatus();
		if (retVal != LT_OK) {
			return;
		}

		UTIL_SEQ_SetTask(1<<LT_TASK_CHECK_BATTERY, CFG_SCH_PRIO_0);
	}

	// FLAG2 Interrupts
	if ((intFlag2 & ADP_IRQ_FLAG2_BUCKBSTPG_INT) >> 4) {
		// Interrupt Due To VOUT2OK Trigger
		// NOTE - NOT ENABLED
	}

	if ((intFlag2 & ADP_IRQ_FLAG2_BUCKPG_INT) >> 5) {
		// Interrupt Due To VOUT1OK Trigger
		// NOTE - NOT ENABLED
	}

	if ((intFlag2 & ADP_IRQ_FLAG2_WD_INT) >> 6) {
		// Interrupt Due To Watchdog Alarm
		// NOTE - NOT ENABLED
	}

	if ((intFlag2 & ADP_IRQ_FLAG2_MR_INT) >> 7) {
		// Interrupt Due To MR Pressed
		// NOTE - NOT ENABLED
	}
}


/*** end of file ***/
