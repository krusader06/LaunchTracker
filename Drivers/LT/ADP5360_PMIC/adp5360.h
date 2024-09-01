/** @file adp5360.h
 *
 * @brief Driver library for the Analog Devices ADP5360.
 *
 * @author Colton Crandell
 * @revision history:
 * - 1.0.0: 07-20-2021 (Crandell) Original
 *
 * COPYRIGHT NOTICE: (c) 2021.  All rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADP5360_H_
#define ADP5360_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "LT_App.h"
/********************************************************************************
 * DEFINES
 *******************************************************************************/
/* Generic Defines ******************************************************************************************************************************** */
#define ADP_I2C_ADDR							(0x46 << 1)			/*!< 0b1000110 7-bit Address													*/
#define ADP_I2C_TIMEOUT							500					/*!< 500ms I2C Timeout															*/

#define ADP_EN_5V_GPIO_Port						(NULL)
#define ADP_EN_5V_Pin							(NULL)

#define OVERRIDE_FUEL							false				/*!< Override fuel gauge with lookup table instead								*/

/* Default Configuration Values ******************************************************************************************************************* */
#define CHG_CURRENT_DEFAULT						0x38				/*!< Charger Current Setting: 320mA (ICHG[4:0] Address: 0x04)					*/
#define CHG_TIMER_EN_DEFAULT					0x00				/*!< Charger Timer Setting: Disabled (EN_CHG_TIEMR[2] Address: 0x06)			*/
#define THR_CUR_DEFAULT							0xC0				/*!< Thermistor Current: 6uA (ITHR[7:6] Address: 0x0A)							*/
#define BAT_CTRL_DEFAULT						0x03				/*!< Battery Protection: Enabled (EN_BATPRO[0] Address: 0x11)					*/
#define BAT_UV_DEFAULT							0xF0				/*!< Battery Undervoltage: 2.8V (UV_DISCH[3:0] Address: 0x12)					*/
#define BAT_OC_CHG_DEFAULT						0xC8				/*!< Battery Charge Overcurrent Protection: 300mA (OC_CHG[2:0] Address: 0x15)	*/
#define BAT_CAP_DEFAULT							0x7D				/*!< Battery Capacity: 250mAh (BAT_CAP[7:0] Address: 0x20)						*/
#define FG_EN_DEFAULT							0x13				/*!< Fuel Gauge: Enabled (EN_FG[0] Address: 0x27)								*/
#define BUCK_CFG_DEFAULT						0x61				/*!< Buck Config: 400mA, Address: 0x29											*/
//#define BUCK_VOUT_DEFAULT						0x31				/*!< Buck Voltage: 3.05V (VOUT_BUCK[5:0] Address: 0x2A)							*/
//#define BUCKBST_CFG_DEFAULT					0x18				/*!< Buck-Boost Config: 400mA, Address: 0x2B									*/
//#define BUCKBST_VOUT_DEFAULT					0x34				/*!< Buck-Boost Voltage: 0x34 (5.0V) (VOUT_BUCKBST[5:0] Address: 0x2C)			*/
#define PGOOD1_MASK_DEFAULT						0x00				/*!< PGOOD1 Pin Mask Register: 0x00 Do Not Output (Address: 0x30)				*/
#define PGOOD2_MASK_DEFAULT						0x00				/*!< PGOOD1 Pin Mask Register: 0x00 Do Not Output (Address: 0x31)				*/

#define SELECTED_BUCK_MODE						REG_BUCK_HYST		/*!< Buck-Boost Selection for the Application Startup							*/


// Default Enabled Interrupts: Charger State Change
#define ADP_IRQ1_DEFAULT						ADP_IRQ_EN1_EN_CHG_INT | ADP_IRQ_EN1_EN_VBUS_INT
#define ADP_IRQ2_DEFAULT						0


/* Charger Bit Descriptions *********************************************************************************************************************** */
// Manufacture and Model ID, Address 0x00 Bit Descriptions
#define ADP_MFG_MODEL_ID_MANUF 					(0x0F << 4)			/*!< The 4-bit manufacturer identification bus. 								*/
#define ADP_MFG_MODEL_ID_MODEL 					(0x0F << 0)			/*!< The 4-bit model identification bus. 										*/

// Silicon Revision, Address 0x01 Bit Descriptions
#define ADP_SILICON_REV							(0x0F << 0)			/*!< The 4-bit silicon revision identification bus. 							*/

// CHARGER_VBUS_ILIM, Address 0x02 Bit Descriptions
#define ADP_CHG_VBUS_ILIM_VADPICHG  			(0x07 << 5)			/*!< Adaptive Current Limit to VBUS Voltage Threshold	 						*/
#define ADP_CHG_VBUS_ILIM_VSYSTEM  				(0x01 << 3)			/*!< VSYS Voltage Programming													*/
#define ADP_CHG_VBUS_ILIM_ILIM  				(0x07 << 0)			/*!< VBUS Pin Input Current-Limit Programming Bus	 							*/

// CHARGER_TERMINATION_SETTING, Address 0x03 Bit Descriptions
#define ADP_CHG_TERM_SET_VTRM  					(0x3F << 2)			/*!< Termination Voltage Programming Bus	 									*/
#define ADP_CHG_TERM_SET_ITRK_DEAD  			(0x03 << 0)			/*!< Trickle and Weak Charge Current Programming Bus	 						*/

// CHARGER_CURRENT_SETTING, Address 0x04 Bit Descriptions
#define ADP_CHG_CUR_SET_IEND					(0x07 << 5)			/*!< Termination Current Programming Bus	 									*/
#define ADP_CHG_CUR_SET_ICHG					(0x1F << 0)			/*!< Fast Charge Current Programming Bus	 									*/

// CHARGER_VOLTAGE_THRESHOLD, Address 0x05 Bit Descriptions
#define ADP_CHG_VLT_THR_DIS_RCH					(0x01 << 7)			/*!< Recharge Function Disable				 									*/
#define ADP_CHG_VLT_THR_VRCH					(0x03 << 5)			/*!< Recharge Voltage Programming Bus		 									*/
#define ADP_CHG_VLT_THR_VTRK_DEAD				(0x03 << 3)			/*!< Trickle to Fast Charge Dead Battery Voltage Programming Bus	 			*/
#define ADP_CHG_VLT_THR_VWEAK					(0x07 << 0)			/*!< Weak Battery Voltage Rising Threshold	 									*/

// CHARGER_TIMER_SETTING, Address 0x06 Bit Descriptions
#define ADP_CHG_TIM_SET_EN_TEND					(0x01 << 3)			/*!< Low = Disable Charge Completion Timer	 									*/
#define ADP_CHG_TIM_SET_CHG_TIMER				(0x01 << 2)			/*!< High = Enable trickle and fast charge timers								*/
#define ADP_CHG_TIM_SET_CHG_TMR_PERIOD			(0x03 << 0)			/*!< tTRK and tCHG Period					 									*/

// CHARGER_FUNCTION_SETTING, Address 0x07 Bit Descriptions
#define ADP_CHG_FNC_SET_EN_JEITA				(0x01 << 7)			/*!< Low = Disable JEITA Li-Ion charging specification							*/
#define ADP_CHG_FNC_SET_ILIM_JEITA_COOL			(0x01 << 6)			/*!< Charging current while in COOL mode	 									*/
#define ADP_CHG_FNC_SET_OFF_ISOFET				(0x01 << 4)			/*!< Select ISOFET and VSYS when battery is present								*/
#define ADP_CHG_FNC_SET_EN_LDO					(0x01 << 3)			/*!< Select charge LDO						 									*/
#define ADP_CHG_FNC_SET_EN_EOC					(0x01 << 2)			/*!< Select if end of charge is enabled		 									*/
#define ADP_CHG_FNC_SET_EN_ADPICHG				(0x01 << 1)			/*!< Select VBUS adaptive current-limit											*/
#define ADP_CHG_FNC_SET_EN_CHG					(0x01 << 0)			/*!< Select whether or not charging is enabled 									*/

// CHARGET_STATUS1, Address 0x08 Bit Descriptions
#define ADP_CHG_STS1_VBUS_OV					(0x01 << 7)			/*!< VBUS over threshold of VBUS_OK			 									*/
#define ADP_CHG_STS1_ADPICHG					(0x01 << 6)			/*!< Adaptive charge current													*/
#define ADP_CHG_STS1_VBUS_ILIM					(0x01 << 5)			/*!< Current is limited by the high voltage blocking FET						*/
#define ADP_CHG_STS1_CHARGER_STATUS				(0x07 << 0)			/*!< Charger Status Bus															*/

// CHARGER_STATUS2, Address 0x09 Bit Descriptions
#define ADP_CHG_STS2_THR_STATUS					(0x07 << 5)			/*!< THR Pin Status																*/
#define ADP_CHG_STS2_BAT_OV_STATUS				(0x01 << 4)			/*!< Battery Over Voltage Status												*/
#define ADP_CHG_STS2_BAT_UV_STATUS				(0x01 << 3)			/*!< Battery Under Voltage Status												*/
#define ADP_CHG_STS2_BAT_CHG_STATUS				(0x07 << 0)			/*!< Battery Status Bus															*/

// BATTERY_THERMISTOR_CONTROL, Address 0x0A Bit Descriptions
#define ADP_BAT_THERM_CTRL_ITHR					(0x03 << 6)			/*!< Select Battery Thermistor NTC Resistance									*/
#define ADP_BAT_THERM_CTRL_EN_THR				(0x01 << 0)			/*!< High = ITHR current source is enabled										*/

// THERMISTOR_60C Threshold, Address 0x0B Bit Descriptions
#define ADP_THERM_60C							(0xFF << 0)			/*!< Thermistor Voltage Threshold for 60 deg C									*/

// THERMISTOR_45C Threshold, Address 0x0C Bit Descriptions
#define ADP_THERM_45C							(0xFF << 0)			/*!< Thermistor Voltage Threshold for 45 deg C									*/

// THERMISTOR_10C Threshold, Address 0x0D Bit Descriptions
#define ADP_THERM_10C							(0xFF << 0)			/*!< Thermistor Voltage Threshold for 10 deg C									*/

// THERMISTOR_0C Threshold, Address 0x0E Bit Descriptions
#define ADP_THERM_0C							(0xFF << 0)			/*!< Thermistor Voltage Threshold for 0 deg C									*/

// THR_VOLTAGE Low, Address 0x0F Bit Descriptions
#define ADP_THR_VOLT_LOW						(0xFF << 0)			/*!< 8-Bit Thermistor Node Voltage Low (mV)										*/

// THR_VOLTAGE High, Address 0x10 Bit Descriptions
#define ADP_THR_VOLT_HI							(0x0F << 0)			/*!< 4-Bit Thermistor Node Voltage High (mV)									*/

// Battery Protection Control, Address 0x11 Bit Descrpitions
#define ADP_BAT_PROT_CTRL_ISOFET_OVCHG			(0x01 << 4)			/*!< ISOFET polarity Selection when battery charging OV protection is triggered	*/
#define ADP_BAT_PROT_CTRL_OC_DIS_HICCUP			(0x01 << 3)			/*!< Battery Discharge Overcurrent Protection Mode Selection.					*/
#define ADP_BAT_PROT_CTRL_OC_CHG_HICCUP			(0x01 << 2)			/*!< Battery Charge Overcurrent Protection Mode Selection.						*/
#define ADP_BAT_PROT_CTRL_EN_CHGLB				(0x01 << 1)			/*!< Battery charge if undervoltage protection triggered.						*/
#define ADP_BAT_PROT_CTRL_EN_BATPRO				(0x01 << 0)			/*!< Battery Protection Enabled													*/

// Battery Protection Undervoltage Setting, Address 0x12 Bit Descriptions
#define ADP_BAT_PROT_UV_SET_UV_DISCH			(0x0F << 4)			/*!< Battery Undervoltage Protection Threshold									*/
#define ADP_BAT_PROT_UV_SET_HYS_UV_DISCH		(0x03 << 2)			/*!< Battery Undervoltage Protection for Overdischarge Hysteresis				*/
#define ADP_BAT_PROT_UV_SET_DGT_UV_DISCH		(0x03 << 0)			/*!< Battery Undervoltage Protection Deglitch Time								*/

// Battery Protection Overcharge Setting, Address 0x13 Bit Descriptions
#define ADP_BAT_PROT_OC_DISCH_SET_OC_DISCH		(0x07 << 5)			/*!< Battery Overcurrent Protection for Overdischarge Threshold					*/
#define ADP_BAT_PROT_OC_DISCH_SET_DGT_OC_DISCH	(0x07 << 1)			/*!< Battery Discharge Overcurrent Protection Deglitch Time Setting				*/

// Battery Protection Overvoltage Setting, Address 0x14 Bit Descriptions
#define ADP_BAT_PROT_OV_SET_OV_CHG				(0x1F << 3)			/*!< Battery Overvoltage Protection Threshold									*/
#define ADP_BAT_PROT_OV_SET_HYS_OV_CHG			(0x03 << 1)			/*!< Battery Overvoltage Protection for Charge Hysteresis						*/
#define ADP_BAT_PROT_OV_SET_DGT_OV_CHG			(0x01 << 0)			/*!< Battery Overvoltage Protection Deglitch Time								*/

// Battery Protection Charge Overcharge Settings, Address 0x15
#define ADP_BAT_PROT_OC_CHG_SET_OC_CHG			(0x07 << 5)			/*!< Battery Overcurrent Protection for Overdischarge Threshold					*/
#define ADP_BAT_PROT_OC_CHG_SET_DGT_OC_CHG		(0x03 << 3)			/*!< Battery Discharge Overcurrent Protection Deglitch Time Setting				*/

/* Fuel Gauge Bit Descriptions ******************************************************************************************************************** */
// V_SOC_0, Address 0x16 Bit Descriptions
#define ADP_V_SOC_0								(0xFF << 0)			/*!< Battery Voltage When State of Charge = 0%									*/

// V_SOC_0, Address 0x17 Bit Descriptions
#define ADP_V_SOC_5								(0xFF << 0)			/*!< Battery Voltage When State of Charge = 5%									*/

// V_SOC_0, Address 0x18 Bit Descriptions
#define ADP_V_SOC_11							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 11%									*/

// V_SOC_0, Address 0x19 Bit Descriptions
#define ADP_V_SOC_19							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 19%									*/

// V_SOC_0, Address 0x1A Bit Descriptions
#define ADP_V_SOC_28							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 28%									*/

// V_SOC_0, Address 0x1B Bit Descriptions
#define ADP_V_SOC_41							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 41%									*/

// V_SOC_0, Address 0x1C Bit Descriptions
#define ADP_V_SOC_55							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 55%									*/

// V_SOC_0, Address 0x1D Bit Descriptions
#define ADP_V_SOC_69							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 69%									*/

// V_SOC_0, Address 0x1E Bit Descriptions
#define ADP_V_SOC_84							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 84%									*/

// V_SOC_0, Address 0x1F Bit Descriptions
#define ADP_V_SOC_100							(0xFF << 0)			/*!< Battery Voltage When State of Charge = 100%								*/

// BAT_CAP, Address 0x20 Bit Descrpitions
#define ADP_BAT_CAP								(0xFF << 0)			/*!< Battery Capacity Input														*/

// BAT_SOC, Address 0x21 Bit Descriptions
#define ADP_BAT_SOC								(0x7F << 0)			/*!< Battery State of Charge Output												*/

// BAT_SOCACM_CTL, Address 0x22 Bit Descriptions
#define ADP_BAT_SOCACM_CTRL_BATCAP_AGE			(0x03 << 6)			/*!< Battery Capacity Reduction Percentage When BAT_SOCACM Overflows			*/
#define ADP_BAT_SOCACM_CTRL_BATCAP_TEMP			(0x03 << 4)			/*!< Battery Capacity Compensation with Temperature Coefficient					*/
#define ADP_BAT_SOCACM_CTRL_EN_BATCAP_TEMP		(0x01 << 1)			/*!< Battery Capacity Temperature Compensation Function Selection				*/
#define ADP_BAT_SOCACM_CTRL_EN_BATCAP_AGE		(0x01 << 0)			/*!< Battery Capacity Aging Compensation Function Selection						*/

// BAT_SOCACM_H, Address 0x23 Bit Descriptions
#define ADP_BAT_SOCACM_H						(0xFF << 0)			/*!< Highest Eight Bits of an 8-Bit Accumulation of the Charge State			*/

// BAT_SOCACM_L, Address 0x24 Bit Descriptions
#define ADP_BAT_SOCACM_L						(0x0F << 4)			/*!< Lowest Four Bits of a 4-Bit Accumulation of the Charge State				*/

// VBAT_READ_H, Address 0x25 Bit Descriptions
#define ADP_VBAT_READ_H							(0xFF << 0)			/*!< Battery Voltage Reading of the Highest Eight Bits (mV)						*/

// VBAT_READ_L, Address 0x26 Bit Descriptions
#define ADP_VBAT_READ_L							(0x1F << 3)			/*!< Battery Voltage Reading of the Lowest Five Bits (mV)						*/

// FUEL_GAUGE_MODE, Address 0x27 Bit Descriptions
#define ADP_FUEL_GAUGE_MODE_SOC_LOW_TH			(0x03 << 6)			/*!< Indication of Low State of Charge Threshold								*/
#define ADP_FUEL_GAUGE_MODE_SLP_CURR			(0x03 << 4)			/*!< Fuel Gauge Sleep Mode Current Threshold									*/
#define ADP_FUEL_GAUGE_MODE_SLP_TIME			(0x03 << 2)			/*!< Fuel Gauge Update Rate of Sleep Mode										*/
#define ADP_FUEL_GAUGE_MODE_FG_MODE				(0x01 << 1)			/*!< Fuel Gauge Operation Mode Selection										*/
#define ADP_FUEL_GAUGE_MODE_EN_FG				(0x01 << 0)			/*!< Fuel Gauge Function Selection												*/

// SOC_RESET, Address 0x28 Bit Descriptions
#define ADP_SOC_RESET							(0x01 << 7)			/*!< Write to refresh BAT_SOC, VBAT_READ_H, VBAT_READ_L Registers				*/

/* Switching Regulator Register Bit Descriptions ************************************************************************************************** */
// Buck Configure, Address 0x29 Bit Descriptions
#define ADP_BCK_CFG_BUCK_SS						(0x03 << 6)			/*!< Buck Regulator Output Soft Start Time										*/
#define ADP_BCK_CFG_BUCK_ILIM					(0x03 << 4)			/*!< Buck Regulator Peak Current Limit											*/
#define ADP_BCK_CFG_BUCK_MODE					(0x01 << 3)			/*!< Buck Operate Mode Selection												*/
#define ADP_BCK_CFG_STP_BUCK					(0x01 << 2)			/*!< Enable Stop Feature to Buck Regulator										*/
#define ADP_BCK_CFG_DISCHG_BUCK					(0x01 << 1)			/*!< Configure Output Discharge Functionality for Buck							*/
#define ADP_BCK_CFG_EN_BUCK						(0x01 << 0)			/*!< Buck Output Control														*/

// Buck Output Voltage Setting, Address 0x2A Bit Descriptions
#define ADP_BCK_OUT_VOLT_SET_BUCK_DLY			(0x03 << 6)			/*!< Buck Switch Delay Time in Hysteresis										*/
#define ADP_BCK_OUT_VOLT_SET_VOUT_BUCK			(0x3F << 0)			/*!< Buck Output Voltage Setting												*/

// Buck Boost Configure, Address 0x2B Bit Descriptions
#define ADP_BCK_BST_CFG_BUCKBST_SS				(0x03 << 6)			/*!< Buck Boost Regulator Output Soft Start Time								*/
#define ADP_BCK_BST_CFG_BUCKBST_ILIM			(0x07 << 3)			/*!< Buck Boost Regulator Peak Current Limit									*/
#define ADP_BCK_BST_CFG_STP_BUCKBST				(0x01 << 2)			/*!< Enable Stop Feature to Buck Boost Regulator								*/
#define ADP_BCK_BST_CFG_DISCHG_BUCKBST			(0x01 << 1)			/*!< Configure Output Discharge Functionality for Buck Boost					*/
#define ADP_BCK_BST_CFG_EN_BUCKBST				(0x01 << 0)			/*!< Buck Boost Output Control													*/

// Buck Boost Output Voltage Setting, Address 0x2C Bit Descriptions
#define ADP_BCK_BST_OUT_VOLT_SET_BUCKBST_DLY	(0x03 << 6)			/*!< Buck Boost Switch Delay Time in Hysteresis									*/
#define ADP_BCK_BST_OUT_VOLT_SET_VOUT_BUCKBST	(0x3F << 0)			/*!< Buck Boost Output Voltage Setting											*/

/* Supervisory Register Bit Descriptions ********************************************************************************************************** */
// Supervisory Setting, Address 0x2D Bit Descriptions
#define ADP_SUPERVISOR_SET_VOUT1_RST			(0x01 << 7)			/*!< Buck Output Voltage Monitor to RESET Selection								*/
#define ADP_SUPERVISOR_SET_VOUT2_RST			(0x01 << 6)			/*!< Buck Boost Output Voltage Monitor to RESET Selection						*/
#define ADP_SUPERVISOR_SET_RESET_TIME			(0x01 << 5)			/*!< RESET Timeout Period Selection												*/
#define ADP_SUPERVISOR_SET_WD_TIME				(0x03 << 3)			/*!< Watchdog Timeout Period Selection											*/
#define ADP_SUPERVISOR_SET_EN_WD				(0x01 << 2)			/*!< Watchdog Timer Select Function												*/
#define ADP_SUPERVISOR_SET_EN_MR_SD				(0x01 << 1)			/*!< Shipment Mode Entry Select Function										*/
#define ADP_SUPERVISOR_SET_RESET_SD				(0x01 << 0)			/*!< Watchdog safety timer reset select function								*/

/* Status and Fault Register Bit Descriptions ***************************************************************************************************** */
// Fault, Address 0x2E Bit Descriptions
#define ADP_FAULT_BAT_UV						(0x01 << 7)			/*!< Battery Undervoltage During Discharging									*/
#define ADP_FAULT_BAT_OC						(0x01 << 6)			/*!< Battery Overcurrent During Overdischarge									*/
#define ADP_FAULT_BAT_CHGOC						(0x01 << 5)			/*!< Battery Overcurrent During Overcharge										*/
#define ADP_FAULT_BAT_CHGOV						(0x01 << 4)			/*!< Battery Overvoltage During Overcharge										*/
#define ADP_FAULT_WD_TIMEOUT					(0x01 << 2)			/*!< Watchdog Timeout Occured													*/
#define ADP_FAULT_TSD110						(0x01 << 0)			/*!< Temperature Shutdown														*/

// PGOOD_STATUS Register, Address 0x2F Bit Descriptions
#define ADP_PGOOD_STATUS_MR_PRESS				(0x01 << 5)			/*!< MR Pin Is Pulled To Low After tDG											*/
#define ADP_PGOOD_STATUS_CHG_CMPLT				(0x01 << 4)			/*!< Battery Charge Complete													*/
#define ADP_PGOOD_STATUS_VBUSOK					(0x01 << 3)			/*!< Real Time Status of VBUS Pin Voltage										*/
#define ADP_PGOOD_STATUS_BATOK					(0x01 << 2)			/*!< Real Time Status of Battery Voltage										*/
#define ADP_PGOOD_STATUS_VOUT2OK				(0x01 << 1)			/*!< Real Time Power Good Status for the Buck Boost Regulator					*/
#define ADP_PGOOD_STATUS_VOUT1OK				(0x01 << 0)			/*!< Real Time Power Good Status for the Buck Regulator							*/

// PGOOD1_MASK Register, Address 0x30 Bit Descriptions
#define ADP_PGOOD1_MASK_PG1_REV					(0x01 << 7)			/*!< Configures Active Low Output of the PGOOD1 Pin								*/
#define ADP_PGOOD1_MASK_CHGCMPLT_MASK1			(0x01 << 4)			/*!< Bind Charger Complete Signal to PGOOD1 Pin									*/
#define ADP_PGOOD1_MASK_VBUSOK_MASK1			(0x01 << 3)			/*!< Bind VBUS OK Signal to PGOOD1 Pin											*/
#define ADP_PGOOD1_MASK_BATOK_MASK1				(0x01 << 2)			/*!< Bind Battery Voltage OK Signal to PGOOD1 Pin								*/
#define ADP_PGOOD1_MASK_VOUT2OK_MASK1			(0x01 << 1)			/*!< Bind Buck Boost Voltage OK Signal to PGOOD1 Pin							*/
#define ADP_PGOOD1_MASK_VOUT1OK_MASK1			(0x01 << 0)			/*!< Bind Buck Voltage OK Signal to PGOOD1 Pin									*/

// PGOOD2_MASK Register, Address 0x31 Bit Descriptions
#define ADP_PGOOD2_MASK_PG2_REV					(0x01 << 7)			/*!< Configures Active Low Output of the PGOOD2 Pin								*/
#define ADP_PGOOD2_MASK_CHGCMPLT_MASK2			(0x01 << 4)			/*!< Bind Charger Complete Signal to PGOOD2 Pin									*/
#define ADP_PGOOD2_MASK_VBUSOK_MASK2			(0x01 << 3)			/*!< Bind VBUS OK Signal to PGOOD2 Pin											*/
#define ADP_PGOOD2_MASK_BATOK_MASK2				(0x01 << 2)			/*!< Bind Battery Voltage OK Signal to PGOOD2 Pin								*/
#define ADP_PGOOD2_MASK_VOUT2OK_MASK2			(0x01 << 1)			/*!< Bind Buck Boost Voltage OK Signal to PGOOD2 Pin							*/
#define ADP_PGOOD2_MASK_VOUT1OK_MASK2			(0x01 << 0)			/*!< Bind Buck Voltage OK Signal to PGOOD2 Pin									*/

// INTERRUPT_ENABLE1 Register, Address 0x32 Bit Descriptions
#define ADP_IRQ_EN1_EN_SOCLOW_INT				(0x01 << 7)			/*!< Enable Battery Low State of Charge Interrupt								*/
#define ADP_IRQ_EN1_EN_SOCACM_INT				(0x01 << 6)			/*!< Enable State of Charge Accumulation Interrupt								*/
#define ADP_IRQ_EN1_EN_ADPICHG_INT				(0x01 << 5)			/*!< Enable VBUS Adaptive Charge Current Limit Interrupt						*/
#define ADP_IRQ_EN1_EN_BATPRO_INT				(0x01 << 4)			/*!< Enable Battery Protection Interrupt										*/
#define ADP_IRQ_EN1_EN_THR_INT					(0x01 << 3)			/*!< Enable THR Temperature Threshold Interrupt									*/
#define ADP_IRQ_EN1_EN_BAT_INT					(0x01 << 2)			/*!< Enable Battery Voltage Threshold Interrupt									*/
#define ADP_IRQ_EN1_EN_CHG_INT					(0x01 << 1)			/*!< Enable Charger Mode Change Interrupt										*/
#define ADP_IRQ_EN1_EN_VBUS_INT					(0x01 << 0)			/*!< Enable VBUS Pin Voltage Threshold Interrupt								*/

// INTERRUPT_ENABLE2 Register, Address 0x33 Bit Descriptions
#define ADP_IRQ_EN2_EN_MR_INT					(0x01 << 7)			/*!< Enable MR Press Interrupt													*/
#define ADP_IRQ_EN2_EN_WD_INT					(0x01 << 6)			/*!< Enable Watchdog Alarm Interrupt											*/
#define ADP_IRQ_EN2_EN_BUCKPG_INT				(0x01 << 5)			/*!< Enable VOUT1OK Change Interrupt											*/
#define ADP_IRQ_EN2_EN_BUCKBSTPG_INT			(0x01 << 4)			/*!< Enable VOUT2OK Change Interrupt											*/

// INTERRUPT_FLAG1 Register, Address 0x34 Bit Descriptions
#define ADP_IRQ_FLAG1_SOCLOW_INT				(0x01 << 7)			/*!< Interrupt Due To Battery Low State of Charge								*/
#define ADP_IRQ_FLAG1_SOCACM_INT				(0x01 << 6)			/*!< Interrupt Due To State of Charge Accumulation Overflow						*/
#define ADP_IRQ_FLAG1_ADPICHG_INT				(0x01 << 5)			/*!< Interrupt Due To VBUS Input Current Limit Adaptive Regulation				*/
#define ADP_IRQ_FLAG1_BATPRO_INT				(0x01 << 4)			/*!< Interrupt Due To Battery Fault												*/
#define ADP_IRQ_FLAG1_THR_INT					(0x01 << 3)			/*!< Interrupt Due To THR Temperature Threshold									*/
#define ADP_IRQ_FLAG1_BAT_INT					(0x01 << 2)			/*!< Interrupt Due To Battery Voltage Threshold									*/
#define ADP_IRQ_FLAG1_CHG_INT					(0x01 << 1)			/*!< Interrupt Due To Charger Mode Change										*/
#define ADP_IRQ_FLAG1_VBUS_INT					(0x01 << 0)			/*!< Interrupt Due To VBUS Voltage Threshold									*/

// INTERRUPT_FLAG2 Register, Address 0x35 Bit Descriptions
#define ADP_IRQ_FLAG2_MR_INT					(0x01 << 7)			/*!< Interrupt Due To MR Pressed												*/
#define ADP_IRQ_FLAG2_WD_INT					(0x01 << 6)			/*!< Interrupt Due To Watchdog Alarm											*/
#define ADP_IRQ_FLAG2_BUCKPG_INT				(0x01 << 5)			/*!< Interrupt Due To VOUT1OK Trigger											*/
#define ADP_IRQ_FLAG2_BUCKBSTPG_INT				(0x01 << 4)			/*!< Interrupt Due To VOUT2OK Trigger											*/

// SHIPMODE Register, Address 0x36 Bit Descriptions
#define ADP_EN_SHIPMODE							(0x01 << 0)			/*!< Enable Shipment Mode														*/


/********************************************************************************
 * TYPES
 *******************************************************************************/
// ADP5360 Register Map
typedef enum {
	MFG_MODEL_ID 				= 0x00,
	SILICON_REV					= 0x01,
	CHG_VBUS_ILIM				= 0x02,
	CHG_TERM_SET				= 0x03,
	CHG_CUR_SET					= 0x04,
	CHG_VLT_THR					= 0x05,
	CHG_TIM_SET					= 0x06,
	CHG_FNC_SET					= 0x07,
	CHG_STS1					= 0x08,
	CHG_STS2					= 0x09,
	BAT_THERM_CTRL				= 0x0A,
	THERM_60C					= 0x0B,
	THERM_45C					= 0x0C,
	THERM_10C					= 0x0D,
	THERM_0C					= 0x0E,
	THR_VOLT_LOW				= 0x0F,
	THR_VOLT_HI					= 0x10,
	BAT_PROT_CTRL				= 0x11,
	BAT_PROT_UV_SET				= 0x12,
	BAT_PROT_OC_DISCH_SET		= 0x13,
	BAT_PROT_OV_SET				= 0x14,
	BAT_PROT_OC_CHG_SET			= 0x15,
	V_SOC_0						= 0x16,
	V_SOC_5						= 0x17,
	V_SOC_11					= 0x18,
	V_SOC_19					= 0x19,
	V_SOC_28					= 0x1A,
	V_SOC_41					= 0x1B,
	V_SOC_55					= 0x1C,
	V_SOC_69					= 0x1D,
	V_SOC_84					= 0x1E,
	V_SOC_100					= 0x1F,
	BAT_CAP						= 0x20,
	BAT_SOC						= 0x21,
	BAT_SOCACM_CTRL				= 0x22,
	BAT_SOCACM_H				= 0x23,
	BAT_SOCACM_L				= 0x24,
	VBAT_READ_H					= 0x25,
	VBAT_READ_L					= 0x26,
	FUEL_GAUGE_MODE				= 0x27,
	SOC_RESET					= 0x28,
	BCK_CFG						= 0x29,
	BCK_OUT_VOLT_SET			= 0x2A,
	BCK_BST_CFG					= 0x2B,
	BCK_BST_OUT_VOLT_SET		= 0x2C,
	SUPERVISOR_SET				= 0x2D,
	FAULT						= 0x2E,
	PGOOD_STATUS				= 0x2F,
	PGOOD1_MASK					= 0x30,
	PGOOD2_MASK					= 0x31,
	IRQ_EN1						= 0x32,
	IRQ_EN2						= 0x33,
	IRQ_FLAG1					= 0x34,
	IRQ_FLAG2					= 0x35,
	SHIPMODE					= 0x36,
} ADP5360_Registers_t;


/* Battery and Charger Enumerations ***************************************************** */
// Charger Status Definition, Address 0x08, Bits [0:2]
typedef enum {
	STS_CHG_OFF = 0,
	STS_CHG_TRICKLE,
	STS_CHG_FAST_CC,
	STS_CHG_FAST_CV,
	STS_CHG_COMPLETE,
	STS_CHG_LDO,
	STS_CHG_TIMER_EXP,
	STS_CHG_BATT_DET,
} adp_charger_status_t;

// Thermistor Status Definition, Address 0x09, Bits [7:5]
typedef enum {
	STS_THR_OFF = 0,
	STS_THR_BAT_COLD,
	STS_THR_BAT_COOL,
	STS_THR_BAT_WARM,
	STS_THR_BAT_HOT,
	STS_THR_OK = 7,
} adp_thr_status_t;

// Thermistor Status Definition, Address 0x09, Bits [2:0]
typedef enum {
	STS_BAT_CHG_NORMAL = 0,
	STS_BAT_CHG_NO_BAT,
	STS_BAT_CHG_TRICKLE,
	STS_BAT_CHG_WEAK,
	STS_BAT_CHG,
} adp_battery_chg_status_t;

/* Regulator Enumerations ***************************************************** */
// Buck Regulator Mode, Address 0x29, Bit [3]
typedef enum {
	REG_BUCK_HYST = 0,
	REG_BUCK_FPWM,
} adp_buck_mode_t;

/* Structures *************************************************************************** */

typedef struct {
	uint8_t adpManu;
	uint8_t adpModel;
	uint8_t adpSiRev;
} ADP5360_hwDetails_t;

typedef struct {
	// CHARGER_STATUS1
	bool vbus_ov;
	bool adpichg;
	bool vbus_ilim;
	adp_charger_status_t chgStatus;

	// CHARGER_STATUS2
	adp_thr_status_t thrStatus;
	bool batOVStatus;
	bool batUVStatus;
	adp_battery_chg_status_t batChgStatus;
} ADP5360_chgStatus_t;

typedef struct {
	bool mrPress;
	bool chgComplete;
	bool vBusOK;
	bool batOK;
	bool vOut2OK;
	bool vOut1OK;
} ADP5360_powerStatus_t;

typedef struct {
	ADP5360_hwDetails_t hardwareDetails;
	ADP5360_chgStatus_t chargeStatus;
	ADP5360_powerStatus_t powerStatus;
	uint8_t stateOfCharge;
	uint16_t batteryVoltage;
} ADP5360_Handle_t;


// Instantiate the adp5360 Structure
extern ADP5360_Handle_t adp5360;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/

// Initialization Functions
lt_err_t adp5360Init(I2C_HandleTypeDef *hi2c);
lt_err_t adp5360DeInit(void);
lt_err_t adp5360Test(void);
bool adp5360IsModInit(void);

// Battery Checker and IRQ
void checkBattery(void);
void adp5360IRQHandler(void);

// Module Functions
lt_err_t adp5360SwitchBuckMode(adp_buck_mode_t mode);
lt_err_t adp5360Enable5V(void);
lt_err_t adp5360Disable5V(void);
lt_err_t adp5360ReadStatus(void);
lt_err_t adp5360RefreshSOC(void);
lt_err_t adp5360EnableShipMode(void);

#endif // ADP5360_H_

/*** end of file ***/
