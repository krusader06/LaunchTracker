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

#include "LT_App.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */
#define ACCEL_I2C_ADDR				(0x1E << 1)			/*!< I2C Address																			*/
#define ACCEL_I2C_TIMEOUT			500					/*!< Timeout																				*/

/* Accelerometer Bit Descriptions ***************************************************************************************************************** */
// Manufacturing ID, Address 0x00 Bit Descriptions
#define KX_MAN_ID_MANID				(0xFF << 0)			/*!< Manufacturing ID																		*/

// Part ID, Address 0x01 Bit Descriptions
#define KX_PART_ID_PARTID			(0xFF << 0)			/*!< Part ID																				*/

// X-Axis ADP, Address 0x02 Bit Descriptions
#define KX_XADP_L_XHP				(0xFF << 0)			/*!< X-axis Advanced Data Path (ADP) output least significant byte							*/

// X-Axis ADP, Address 0x03 Bit Descriptions
#define KX_XADP_H_XHP				(0xFF << 0)			/*!< X-axis Advanced Data Path (ADP) output most significant byte							*/

// Y-Axis ADP, Address 0x04 Bit Descriptions
#define KX_YADP_L_YHP				(0xFF << 0)			/*!< Y-axis Advanced Data Path (ADP) output least significant byte							*/

// Y-Axis ADP, Address 0x05 Bit Descriptions
#define KX_YADP_H_YHP				(0xFF << 0)			/*!< Y-axis Advanced Data Path (ADP) output most significant byte							*/

// Z-Axis ADP, Address 0x06 Bit Descriptions
#define KX_ZADP_L_ZHP				(0xFF << 0)			/*!< Z-axis Advanced Data Path (ADP) output least significant byte							*/

// Z-Axis ADP, Address 0x07 Bit Descriptions
#define KX_ZADP_H_ZHP				(0xFF << 0)			/*!< Z-axis Advanced Data Path (ADP) output most significant byte							*/

// X-Axis, Address 0x08 Bit Descriptions
#define KX_XOUT_L_XOUT				(0xFF << 0)			/*!< X-axis accelerometer output least significant byte										*/

// X-Axis, Address 0x09 Bit Descriptions
#define KX_XOUT_H_XOUT				(0xFF << 0)			/*!< X-axis accelerometer output most significant byte										*/

// X-Axis, Address 0x0A Bit Descriptions
#define KX_YOUT_L_YOUT				(0xFF << 0)			/*!< Y-axis accelerometer output least significant byte										*/

// X-Axis, Address 0x0B Bit Descriptions
#define KX_YOUT_H_YOUT				(0xFF << 0)			/*!< Y-axis accelerometer output most significant byte										*/

// Z-Axis, Address 0x0C Bit Descriptions
#define KX_ZOUT_L_ZOUT				(0xFF << 0)			/*!< Z-axis accelerometer output least significant byte										*/

// Z-Axis, Address 0x0D Bit Descriptions
#define KX_ZOUT_H_ZOUT				(0xFF << 0)			/*!< Z-axis accelerometer output most significant byte										*/

// COTR, Address 0x12 Bit Descriptions
#define KX_COTR_DCSTR				(0xFF << 0)			/*!< Command Test Response Register. Default value = 0x55									*/

// WHO_AM_I, Address 0x13 Bit Descriptions
#define KX_WHO_AM_I_WAI				(0xFF << 0)			/*!< Supplier Recognition. Default value = 0x3D												*/

// Current Tilt Position, Address 0x14 Bit Descriptions
#define KX_TSCP_FU					(0x01 << 0)			/*!< Current Tilt Position - Face-Up State (Z+)												*/
#define KX_TSCP_FD					(0x01 << 1)			/*!< Current Tilt Position - Face-Down State (Z-)											*/
#define KX_TSCP_UP					(0x01 << 2)			/*!< Current Tilt Position - Up State (Y+)													*/
#define KX_TSCP_DO					(0x01 << 3)			/*!< Current Tilt Position - DownState (Y-)													*/
#define KX_TSCP_RI					(0x01 << 4)			/*!< Current Tilt Position - Right State (X+)												*/
#define KX_TSCP_LE					(0x01 << 5)			/*!< Current Tilt Position - Left State (X-)												*/

// Previous Tilt Position, Address 0x15 Bit Descriptions
#define KX_TSPP_FU					(0x01 << 0)			/*!< Previous Tilt Position - Face-Up State (Z+)											*/
#define KX_TSPP_FD					(0x01 << 1)			/*!< Previous Tilt Position - Face-Down State (Z-)											*/
#define KX_TSPP_UP					(0x01 << 2)			/*!< Previous Tilt Position - Up State (Y+)													*/
#define KX_TSPP_DO					(0x01 << 3)			/*!< Previous Tilt Position - DownState (Y-)												*/
#define KX_TSPP_RI					(0x01 << 4)			/*!< Previous Tilt Position - Right State (X+)												*/
#define KX_TSPP_LE					(0x01 << 5)			/*!< Previous Tilt Position - Left State (X-)												*/

// Interrupt Source 1, Address 0x16 Bit Descriptions - contains tap and double-tap interrupts
#define KX_INS1_TFU					(0x01 << 0)			/*!< Z Positive (Z+) Reported																*/
#define KX_INS1_TFD					(0x01 << 1)			/*!< Z Negative (Z-) Reported																*/
#define KX_INS1_TUP					(0x01 << 2)			/*!< Y Positive (Y+) Reported																*/
#define KX_INS1_TDO					(0x01 << 3)			/*!< Y Negative (Y-) Reported																*/
#define KX_INS1_TRI					(0x01 << 4)			/*!< X Positive (X+) Reported																*/
#define KX_INS1_TLE					(0x01 << 5)			/*!< X Negative (X-) Reported																*/

// Interrupt Source 2, Address 0x17 Bit Descriptions - tells which function caused an interrupt
#define KX_INS2_TPS					(0x01 << 0)			/*!< Tilt position status																	*/
#define KX_INS2_TDTS				(0x03 << 2)			/*!< Tap/Double-Tap Status																	*/
#define KX_INS2_DRDY				(0x01 << 4)			/*!< Data Ready																				*/
#define KX_INS2_WMI					(0x01 << 5)			/*!< Watermark Interrupt																	*/
#define KX_INS2_BFI					(0x01 << 6)			/*!< Buffer Full Interrupt																	*/
#define KX_INS2_FFS					(0x01 << 7)			/*!< Free-fall Status																		*/

// Interrupt Source 3, Address 0x18 Bit Descriptions - Reports axis and direction of detected motion that triggered interrupt
#define KX_INS3_ZPWU				(0x01 << 0)			/*!< Z Positive (Z+) Reported																*/
#define KX_INS3_ZNWU				(0x01 << 1)			/*!< Z Negative (Z-) Reported																*/
#define KX_INS3_YPWU				(0x01 << 2)			/*!< Y Positive (Y+) Reported																*/
#define KX_INS3_YNWU				(0x01 << 3)			/*!< Y Negative (Y-) Reported																*/
#define KX_INS3_XPWU				(0x01 << 4)			/*!< X Positive (X+) Reported																*/
#define KX_INS3_XNWU				(0x01 << 5)			/*!< X Negative (X-) Reported																*/
#define KX_INS3_BTS					(0x01 << 6)			/*!< Back to Sleep Interrupt																*/
#define KX_INS3_WUFS				(0x01 << 7)			/*!< Wake Up Interrupt																		*/

// Status Register, Address 0x19 Bit Descriptions
#define KX_STATUS_REG_WAKE			(0x01 << 0)			/*!< Wake/Back to sleep state																*/
#define KX_STATUS_REG_INT			(0x01 << 4)			/*!< Combined interrupt information															*/

// Interrupt Latch Release, Address 0x1A Bit Descriptions
#define KX_INT_REL_SRC				(0xFF << 0)			/*!< Interrupt Latch Release (read to releease interrupt latch)								*/

// Control Register 1, Address 0x1B Bit Descriptions
#define KX_CNTL1_TPE				(0x01 << 0)			/*!< Tilt position engine enable bit														*/
#define KX_CNTL1_TDTE				(0x01 << 2)			/*!< Tap/Double-Tap Engine enable bit														*/
#define KX_CNTL1_GSEL				(0x03 << 3)			/*!< G-range Select bits (8g/16g/32g/64g)													*/
#define KX_CNTL1_DRDYE				(0x01 << 5)			/*!< Data Ready Engine enable bit															*/
#define KX_CNTL1_RES				(0x01 << 6)			/*!< Performance mode. 0 = Low Power, 1 = High-Performance Mode								*/
#define KX_CNTL1_PC1				(0x01 << 7)			/*!< Operating Mode. 0 = Stand-by, 1 = High-Perf or Low power mode							*/

// Control Register 2, Address 0x1C Bit Descriptions
#define KX_CNTL2_FUM				(0x01 << 0)			/*!< Face-Up State Enable (Z+)																*/
#define KX_CNTL2_FDM				(0x01 << 1)			/*!< Face-Down State Enable	(Z-)															*/
#define KX_CNTL2_UPM				(0x01 << 2)			/*!< Up State Enable (Y+)																	*/
#define KX_CNTL2_DOM				(0x01 << 3)			/*!< Down State Enable (Y-)																	*/
#define KX_CNTL2_RIM				(0x01 << 4)			/*!< Right State Enable	(X+)																*/
#define KX_CNTL2_LEM				(0x01 << 5)			/*!< Left State Enable (X-)																	*/
#define KX_CNTL2_COTC				(0x01 << 6)			/*!< Command Test Control. 0 = no action, 1 = Sets COTR to 0xAA								*/
#define KX_CNTL2_SRST				(0x01 << 7)			/*!< Software Reset bit - Write a 1 to initiate a software reset							*/

// Control Register 3, Address 0x1D Bit Descriptions
#define KX_CNTL3_OWUF				(0x07 << 0)			/*!< Motion Wake Up Function Output Data Rate												*/
#define KX_CNTL3_OTDT				(0x07 << 3)			/*!< Directional-Tap Function Output Data Rate												*/
#define KX_CNTL3_OTP				(0x03 << 6)			/*!< Tilt Position Function Output Data Rate												*/

// Control Register 4, Address 0x1E Bit Descriptions
#define KX_CNTL4_OBTS				(0x07 << 0)			/*!< Motion Back-to-Sleep Function Output Data Rate											*/
#define KX_CNTL4_PR_MODE			(0x01 << 3)			/*!< Pulse Reject Mode. 0 = Standard Operation, 1 = Reject pulse-like motion				*/
#define KX_CNTL4_BTSE				(0x01 << 4)			/*!< Back-to-Sleep Engine enable bit														*/
#define KX_CNTL4_WUFE				(0x01 << 5)			/*!< Wake-up Function Engine enable bit.													*/
#define KX_CNTL4_TH_MODE			(0x01 << 6)			/*!< Defines wake/back to sleep threshold mode												*/
#define KX_CNTL4_C_MODE				(0x01 << 7)			/*!< Defines de-bounce counter clear mode													*/

// Control Register 5, Address 0x1F Bit Descriptions
#define KX_CNTL5_MAN_SLEEP			(0x01 << 0)			/*!< Manual wake-sleep engine overwrite														*/
#define KX_CNTL5_MAN_WAKE			(0x01 << 1)			/*!< Manual wake-sleep engine overwrite 													*/
#define KX_CNTL5_ADPE				(0x01 << 4)			/*!< Advanced Data Path (ADP) enable														*/

// Output data control register, Address 0x21 Bit Descriptions
#define KX_ODCNTL_OSA				(0x0F << 0)			/*!< Output Data Rate																		*/
#define KX_ODCNTL_FSTUP				(0x01 << 5)			/*!< Fast Start Up Enable Bit																*/
#define KX_ODCNTL_LPRO				(0x01 << 6)			/*!< Low-Pass filter Roll-Off control														*/

// Interrupt Control Register 1, Address 0x22 Bit Descriptions
#define KX_INC1_SPI3E				(0x01 << 0)			/*!< Sets 3-Wire SPI Interface																*/
#define KX_INC1_STPOL				(0x01 << 1)			/*!< Sets the polarity of Self-Test															*/
#define KX_INC1_IEL1				(0x01 << 3)			/*!< Interrupt latch control for physical interrupt pin										*/
#define KX_INC1_IEA1				(0x01 << 4)			/*!< Interrupt active level control for interrupt pin										*/
#define KX_INC1_IEN1				(0x01 << 5)			/*!< Enables/disables the physical interrupt pin 1											*/
#define KX_INC1_PW1					(0x03 << 6)			/*!< Pulse INT1 pin width configuration														*/

// Interrupt Control Register 2, Address 0x23 Bit Descriptions
#define KX_INC2_ZPWUE				(0x01 << 0)			/*!< Z positive (Z+) mask for WUF and BTS													*/
#define KX_INC2_ZNWUE				(0x01 << 1)			/*!< Z negative (Z-) mask for WUF and BTS													*/
#define KX_INC2_YPWUE				(0x01 << 2)			/*!< Y Positive (Y+) mask for WUF and BTS													*/
#define KX_INC2_YNWUE				(0x01 << 3)			/*!< Y Negative (Y-) mask for WUF and BTS													*/
#define KX_INC2_XPWUE				(0x01 << 4)			/*!< X Positive (X+) mask for WUF and BTS													*/
#define KX_INC2_XNWUE				(0x01 << 5)			/*!< X Negative (X-) mask for WUF and BTS													*/
#define KX_INC2_AOI					(0x01 << 6)			/*!< AND-OR configuration on motion detection												*/

// Interrupt Control Register 3, Address 0x24 Bit Descriptions
#define KX_INC3_TFUM				(0x01 << 0)			/*!< Tap Face Up (Z+) state mask															*/
#define KX_INC3_TFDM				(0x01 << 1)			/*!< Tap Face Down (Z-) state mask															*/
#define KX_INC3_TUPM				(0x01 << 2)			/*!< Tap Up (Y+) state mask																	*/
#define KX_INC3_TDOM				(0x01 << 3)			/*!< Tap Down (Y-) state mask																*/
#define KX_INC3_TRIM				(0x01 << 4)			/*!< Tap Right (X+) state mask																*/
#define KX_INC3_TLEM				(0x01 << 5)			/*!< Tap Left (X-) state mask																*/
#define KX_INC3_TMEN				(0x01 << 6)			/*!< enables/disables alternate tap masking scheme											*/

// Interrupt Control Register 4, Address 0x25 Bit Descriptions
#define KX_INC4_TPI1				(0x01 << 0)			/*!< Tilt position interrupt reported on physical interrupt pin INT1						*/
#define KX_INC4_WUFI1				(0x01 << 1)			/*!< Wake-Up (motion detect) interrupt reported on physical interrupt pin INT1				*/
#define KX_INC4_TDTI1				(0x01 << 2)			/*!< Tap/Double Tap interrupt reported on physical interrupt pin INT1						*/
#define KX_INC4_BTSI1				(0x01 << 3)			/*!< Back to sleep interrupt reported on physical interrupt pin INT1						*/
#define KX_INC4_DRDYI1				(0x01 << 4)			/*!< Data ready interrupt reported on physical interrupt pin INT1							*/
#define KX_INC4_WMI1				(0x01 << 5)			/*!< Watermark interrupt reported on physical interrupt pin INT1							*/
#define KX_INC4_BFI1				(0x01 << 6)			/*!< Buffer full interrupt reported on physical interrupt pin INT1							*/
#define KX_INC4_FFI1				(0x01 << 7)			/*!< Free fall interrupt reported on physical interrupt pin INT1							*/

// Interrupt Control Register 5, Address 0x26 Bit Descriptions
#define KX_INC5_ACLR1				(0x01 << 0)			/*!< Clear latched interrupt source information	for Interrupt 1								*/
#define KX_INC5_ACLR2				(0x01 << 1)			/*!< Clear latched interrupt source information	for Interrupt 2								*/
#define KX_INC5_IEL2				(0x01 << 3)			/*!< Interrupt latch control for interrupt pin												*/
#define KX_INC5_IEA2				(0x01 << 4)			/*!< Interrupt active level control for interrupt pin										*/
#define KX_INC5_IEN2				(0x01 << 5)			/*!< Enables/disables the physical interrupt pin 2											*/
#define KX_INC5_PW2					(0x03 << 6)			/*!< Pulse INT2 pin width configuration														*/

// Interrupt Control Register 6, Address 0x27 Bit Descriptions
#define KX_INC6_TPI2				(0x01 << 0)			/*!< Tilt position interrupt reported on physical interrupt pin INT2						*/
#define KX_INC6_WUFI2				(0x01 << 1)			/*!< Wake-Up (motion detect) interrupt reported on physical interrupt pin INT2				*/
#define KX_INC6_TDTI2				(0x01 << 2)			/*!< Tap/Double Tap interrupt reported on physical interrupt pin INT2						*/
#define KX_INC6_BTSI2				(0x01 << 3)			/*!< Back to sleep interrupt reported on physical interrupt pin INT2						*/
#define KX_INC6_DRDYI2				(0x01 << 4)			/*!< Data ready interrupt reported on physical interrupt pin INT2							*/
#define KX_INC6_WMI2				(0x01 << 5)			/*!< Watermark interrupt reported on physical interrupt pin INT2							*/
#define KX_INC6_BFI2				(0x01 << 6)			/*!< Buffer full interrupt reported on physical interrupt pin INT2							*/
#define KX_INC6_FFI2				(0x01 << 7)			/*!< Free fall interrupt reported on physical interrupt pin INT2							*/

// Tilt Timer, Address 0x29 Bit Descriptions
#define KX_TILT_TIMER_TSC			(0xFF << 0)			/*!< Initial count register for the tilt position state time								*/

// Tap/Double-Tap Report Control, Address 0x2A Bit Descriptions
#define KX_TDTRC_STRE				(0x01 << 0)			/*!< Enables/disables single tap interrupt													*/
#define KX_TDTRC_DTRE				(0x01 << 1)			/*!< Enables/disables the double tap interrupt												*/

// Double Tap Counter, Address 0x2B Bit Descriptions
#define KX_TDTC_TDTC				(0xFF << 0)			/*!< Double tap counter period																*/

// Tap Threshold High, Address 0x2C Bit Descriptions
#define KX_TTH_TTH					(0xFF << 0)			/*!< Tap Threshold High																		*/

// Tap Threshold Low, Address 0x2D Bit Descriptions
#define KX_TTL_TTL					(0xFF << 0)			/*!< Tap Threshold Low																		*/

// Tap Counter, Address 0x2E Bit Descriptions
#define KX_FTD_FTDL					(0xFF << 0)			/*!< Tap Counter information																*/

// Double Tap Counter, Address 0x2F Bit Descriptions
#define KX_STD_STD					(0xFF << 0)			/*!< Double Tap Counter information															*/

// Tap Counter, Address 0x30 Bit Descriptions
#define KX_TLT_TLT					(0xFF << 0)			/*!< Tap Counter information																*/

// Single and Double tap Counter, Address 0x31 Bit Descriptions
#define KX_TWS_TWS					(0xFF << 0)			/*!< Single and Double tap Counter															*/

// Free Fall Threshold, Address 0x32 Bit Descriptions
#define KX_FFTH_FFTH				(0xFF << 0)			/*!< Free Fall Threshold																	*/

// Free Fall Counter, Address 0x33 Bit Descriptions
#define KX_FFC_FFC					(0xFF << 0)			/*!< Free Fall Counter																		*/

// Free Fall Control, Address 0x34 Bit Descriptions
#define KX_FFCNTL_OFFI				(0x07 << 0)			/*!< Free Fall Engine Output Data Rate														*/
#define KX_FFCNTL_DCRM				(0x01 << 3)			/*!< Debounce methodology control															*/
#define KX_FFCNTL_FFDC				(0x03 << 4)			/*!< Free fall interrupt delayed clear duration for unlatched mode							*/
#define KX_FFCNTL_ULMODE			(0x01 << 6)			/*!< Free fall interrupt latch/un-latch control												*/
#define KX_FFCNTL_FFIE				(0x01 << 7)			/*!< Free fall engine enable																*/

// Tilt Angle Low Limit, Address 0x37 Bit Descriptions
#define KX_TILT_ANGLE_LL_LL			(0xFF << 0)			/*!< Tilt Angle Low Limit																	*/

// Tilt Angle High Limit, Address 0x38 Bit Descriptions
#define KX_TILT_ANGLE_HL_HL			(0xFF << 0)			/*!< Tilt Angle High Limit																	*/

// Hysteresis Setting, Address 0x39 Bit Descriptions
#define KX_HYST_SET_HYST			(0x3F << 0)			/*!< Hysteresis Setting																		*/

// Low Power Control 1, Address 0x3A Bit Descriptions
#define KX_LP_CNTL1_AVC				(0x07 << 4)			/*!< Averaging Filter Control																*/

// Low Power Control 2, Address 0x3B Bit Descriptions
#define KX_LP_CNTL2_LPSTPSEL		(0x01 << 0)			/*!< Digital power shut-off select															*/

// Wakeup Threshold, Address 0x49 Bit Descriptions
#define KX_WUFTH_WUFTH				(0xFF << 0)			/*!< Wakeup Threshold																		*/

// Wakeup/Back to Sleep Threshold, Address 0x4A Bit Descriptions
#define KX_BTSWUFTH_WUFTH			(0x07 << 0)			/*!< Wakeup Threshold, High Bits															*/
#define KX_BTSWUFTH_BTSTH			(0x07 << 4)			/*!< Back to Sleep Threshold, High Bits														*/

// Back to Sleep Threshold, Address 0x4B Bit Descriptions
#define KX_BTSTH_BTSTH				(0xFF << 0)			/*!< Back to Sleep Threshold																*/

// Back to Sleep Debounce Counter, Address 0x4C Bit Descriptions
#define KX_BTSC_BTSC				(0xFF << 0)			/*!< Debounce counter register for the Back-to-Sleep (BTS) engine							*/

// Wakeup Debounce Counter, Address 0x4D Bit Descriptions
#define KX_WUFC_WUFC				(0xFF << 0)			/*!< Debounce counter register for the Wake-up Function (WUF) engine						*/

// Self Test, Address 0x5D Bit Descriptions
#define KX_SELF_TEST_ST				(0xFF << 0)			/*!< Self Test Enable Register (write only)													*/

// Buffer sample threshold, Address 0x5E Bit Descriptions
#define KX_BUF_CNTL1_SMP_TH			(0xFF << 0)			/*!< Sample Threshold																		*/

// Sample buffer operation, Address 0x5F Bit Descriptions
#define KX_BUF_CNTL2_BM				(0x03 << 0)			/*!< selects the operating mode of the sample buffer										*/
#define KX_BUF_CNTL2_BFIE			(0x01 << 5)			/*!< buffer full interrupt enable bit														*/
#define KX_BUF_CNTL2_BRES			(0x01 << 6)			/*!< Resolution of the acceleration data samples collected by the sample buffer				*/
#define KX_BUF_CNTL2_BUFE			(0x01 << 7)			/*!< controls activation of the sample buffer												*/

// Sample Buffer Status, Address 0x60 Bit Descriptions
#define KX_BUF_STATUS_1_SMP_LEV		(0xFF << 0)			/*!< Sample Level																			*/

// Sample Buffer Status, Address 0x61 Bit Descriptions
#define KX_BUF_STATUS_2_SMP_LEV		(0x03 << 0)			/*!< Sample Level																			*/
#define KX_BUF_STATUS_2_BUF_TRIG	(0x01 << 7)			/*!< status of the buffer’s trigger function												*/

// Clear Buffer, Address 0x62 Bit Descriptions
#define KX_BUF_CLEAR_X				(0xFF << 0)			/*!< Clear Buffer																			*/

// Read Buffer, Address 0x63 Bit Descriptions
#define KX_BUF_READ_X				(0xFF << 0)			/*!< Read Buffer																			*/

// Advanced Data Path Control Register 1, Address 0x64 Bit Descriptions
#define KX_ADP_CNTL1_OADP			(0x0F << 0)			/*!< Output Data Rate																		*/
#define KX_ADP_CNTL1_RMS_AVC		(0x07 << 4)			/*!< Number of samples used to calculate RMS output											*/

// Advanced Data Path Control Register 2, Address 0x65 Bit Descriptions
#define KX_ADP_CNTL2_ADP_F2_HP		(0x01 << 0)			/*!< Filter-2 High-pass enable																*/
#define KX_ADP_CNTL2_ADP_RMS_OSEL	(0x01 << 1)			/*!< Select data out to XADP, YADP, ZADP registers and to the output buffer					*/
#define KX_ADP_CNTL2_ADP_FLT1_BYP	(0x01 << 3)			/*!< Advanced Data Path Filter-1 bypass control												*/
#define KX_ADP_CNTL2_ADP_FLT2_BYP	(0x01 << 4)			/*!< Advanced Data Path Filter-2 bypass control												*/
#define KX_ADP_CNTL2_ADP_WB_OSEL	(0x01 << 5)			/*!< RMS select data for the Wake-up/Back-to-Sleep engines									*/
#define KX_ADP_CNTL2_ADP_WB_ISEL	(0x01 << 6)			/*!< Input select for the Wake-up/Back-to-Sleep engines										*/
#define KX_ADP_CNTL2_ADP_BUF_SEL	(0x01 << 7)			/*!< Select data to be routed to the sample buffer											*/

// Advanced Data Path Control Register 3, Address 0x66 Bit Descriptions
#define KX_ADP_CNTL3_ADP_F1_1A		(0x7F << 0)			/*!< ADP filter-1 coefficient (1/A)															*/

// Advanced Data Path Control Register 4, Address 0x67 Bit Descriptions
#define KX_ADP_CNTL4_ADP_F1_BA		(0xFF << 0)			/*!< ADP filter-1 coefficient (B/A)	Bits 0-7												*/

// Advanced Data Path Control Register 5, Address 0x68 Bit Descriptions
#define KX_ADP_CNTL5_ADP_F1_BA		(0xFF << 0)			/*!< ADP filter-1 coefficient (B/A)	Bits 8-15												*/

// Advanced Data Path Control Register 6, Address 0x69 Bit Descriptions
#define KX_ADP_CNTL6_ADP_F1_BA		(0x7F << 0)			/*!< ADP filter-1 coefficient (B/A)	Bits 16-22												*/

// Advanced Data Path Control Register 7, Address 0x6A Bit Descriptions
#define KX_ADP_CNTL7_ADP_F1_CA		(0xFF << 0)			/*!< ADP filter-1 coefficient (C/A)	Bits 0-7												*/

// Advanced Data Path Control Register 8, Address 0x6B Bit Descriptions
#define KX_ADP_CNTL8_ADP_F1_CA		(0xFF << 0)			/*!< ADP filter-1 coefficient (C/A)	Bits 8-15												*/

// Advanced Data Path Control Register 9, Address 0x6C Bit Descriptions
#define KX_ADP_CNTL9_ADP_F1_CA		(0x7F << 0)			/*!< ADP filter-1 coefficient (C/A)	Bits 16-22												*/

// Advanced Data Path Control Register 10, Address 0x6D Bit Descriptions
#define KX_ADP_CNTL10_ADP_F1_ISH	(0x1F << 0)			/*!< ADP filter-1 input scale shift value													*/

// Advanced Data Path Control Register 11, Address 0x6E Bit Descriptions
#define KX_ADP_CNTL11_ADP_F2_1A		(0x7F << 0)			/*!< ADP filter-2 coefficient (1/A)															*/
#define KX_ADP_CNTL11_ADP_F1_OSH	(0x01 << 7)			/*!< ADP filter-1 output scale shift value													*/

// Advanced Data Path Control Register 12, Address 0x6F Bit Descriptions
#define KX_ADP_CNTL12_ADP_F2_BA		(0xFF << 0)			/*!< ADP filter-2 coefficient (B/A) Bits 0-7												*/

// Advanced Data Path Control Register 13, Address 0x70 Bit Descriptions
#define KX_ADP_CNTL13_ADP_F2_BA		(0x7F << 0)			/*!< ADP filter-2 coefficient (B/A) Bits 8-14												*/

// Advanced Data Path Control Register 14, Address 0x71 Bit Descriptions
#define KX_ADP_CNTL14_0				(0xFF << 0)			/*!< Register set to 0																		*/

// Advanced Data Path Control Register 15, Address 0x72 Bit Descriptions
#define KX_ADP_CNTL15_0				(0xFF << 0)			/*!< Register set to 0																		*/

// Advanced Data Path Control Register 16, Address 0x73 Bit Descriptions
#define KX_ADP_CNTL16_0				(0xFF << 0)			/*!< Register set to 0																		*/

// Advanced Data Path Control Register 17, Address 0x74 Bit Descriptions
#define KX_ADP_CNTL17_0				(0xFF << 0)			/*!< Register set to 0																		*/

// Advanced Data Path Control Register 18, Address 0x75 Bit Descriptions
#define KX_ADP_CNTL18_ADP_F2_ISH	(0x1F << 0)			/*!< ADP filter-2 input scale shift value													*/

// Advanced Data Path Control Register 19, Address 0x76 Bit Descriptions
#define KX_ADP_CNTL19_ADP_F2_OSH	(0x1F << 0)			/*!< ADP filter-2 output scale shift value													*/



/********************************************************************************
 * TYPES
 *******************************************************************************/
/* KX134-1211 Register Map ************************************************************************************************************************ */
typedef enum {
	KX_MAN_ID = 0x00,									/*!< Manufacturing ID																		*/
	KX_PART_ID = 0x01,
	KX_XADP_L = 0x02,
	KX_XADP_H = 0x03,
	KX_YADP_L = 0x04,
	KX_YADP_H = 0x05,
	KX_ZADP_L = 0x06,
	KX_ZADP_H = 0x07,
	KX_XOUT_L = 0x08,
	KX_XOUT_H = 0x09,
	KX_YOUT_L = 0x0A,
	KX_YOUT_H = 0x0B,
	KX_ZOUT_L = 0x0C,
	KX_ZOUT_H = 0x0D,
	KX_COTR = 0x12,
	KX_WHO_AM_I = 0x13,
	KX_TSCP = 0x14,
	KX_TSPP = 0x15,
	KX_INS1 = 0x16,
	KX_INS2 = 0x17,
	KX_INS3 = 0x18,
	KX_STATUS_REG = 0x19,
	KX_INT_REL = 0x1A,
	KX_CNTL1 = 0x1B,
	KX_CNTL2 = 0x1C,
	KX_CNTL3 = 0x1D,
	KX_CNTL4 = 0x1E,
	KX_CNTL5 = 0x1F,
	KX_CNTL6 = 0x20,
	KX_ODCNTL = 0x21,
	KX_INC1 = 0x22,
	KX_INC2 = 0x23,
	KX_INC3 = 0x24,
	KX_INC4 = 0x25,
	KX_INC5 = 0x26,
	KX_INC6 = 0x27,
	KX_TILT_TIMER = 0x29,
	KX_TDTRC = 0x2A,
	KX_TDTC = 0x2B,
	KX_TTH = 0x2C,
	KX_TTL = 0x2D,
	KX_FTD = 0x2E,
	KX_STD = 0x2F,
	KX_TLT = 0x30,
	KX_TWS = 0x31,
	KX_FFTH = 0x32,
	KX_FFC = 0x33,
	KX_FFCNTL = 0x34,
	KX_TILT_ANGLE_LL = 0x37,
	KX_TILT_ANGLE_HL = 0x38,
	KX_HYST_SET = 0x39,
	KX_LP_CNTL1 = 0x3A,
	KX_LP_CNTL2 = 0x3B,
	KX_WUFTH = 0x49,
	KX_BTSWUFTH = 0x4A,
	KX_BTSTH = 0x4B,
	KX_BTSC = 0x4C,
	KX_WUFC = 0x4D,
	KX_SELF_TEST = 0x5D,
	KX_BUF_CNTL1 = 0x5E,
	KX_BUF_CNTL2 = 0x5F,
	KX_BUF_STATUS_1 = 0x60,
	KX_BUF_STATUS_2 = 0x61,
	KX_BUF_CLEAR = 0x62,
	KX_BUF_READ = 0x63,
	KX_ADP_CNTL1 = 0x64,
	KX_ADP_CNTL2 = 0x65,
	KX_ADP_CNTL3 = 0x66,
	KX_ADP_CNTL4 = 0x67,
	KX_ADP_CNTL5 = 0x68,
	KX_ADP_CNTL6 = 0x69,
	KX_ADP_CNTL7 = 0x6A,
	KX_ADP_CNTL8 = 0x6B,
	KX_ADP_CNTL9 = 0x6C,
	KX_ADP_CNTL10 = 0x6D,
	KX_ADP_CNTL11 = 0x6E,
	KX_ADP_CNTL12 = 0x6F,
	KX_ADP_CNTL13 = 0x70,
	KX_ADP_CNTL14 = 0x71,
	KX_ADP_CNTL15 = 0x72,
	KX_ADP_CNTL16 = 0x73,
	KX_ADP_CNTL17 = 0x74,
	KX_ADP_CNTL18 = 0x75,
	KX_ADP_CNTL19 = 0x76,
} KX134_Registers_t;



typedef struct {

} KX134_Handle_t;


// Instantiate the KX134 Structure
extern KX134_Handle_t kx134Handle;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
lt_err_t kx134Init(I2C_HandleTypeDef *hi2c);
lt_err_t kx134DeInit(void);
lt_err_t kx134Test(void);
bool kx134IsModInit(void);




#endif // KX134_ACCEL_H_

/*** end of file ***/
