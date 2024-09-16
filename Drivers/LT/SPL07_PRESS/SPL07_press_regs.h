/** @file SPL07_press.h
 *
 * @brief Driver library for the SPL07-003 absolute pressure sensor
 *
 * @author Colton Crandell
 *
 * COPYRIGHT NOTICE: (c) 2024.  All rights reserved.
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPL07_PRESS_REGS_H_
#define SPL07_PRESS_REGS_H_

#define SPL07_PRS_B2 0x00 		// Pressure Data, 24bit

typedef struct {
	uint8_t prs_b2 : 8; 		// Pressure Data [23:16]
} spl07_prs_b2_t;

#define SPL07_PRS_B1 0x01

typedef struct {
	uint8_t prs_b1 : 8; 		// Pressure Data [15:8]
} spl07_prs_b1_t;

#define SPL07_PRS_B0 0x02

typedef struct {
	uint8_t prs_b0 : 8; 		// Pressure Data [7:0]
} spl07_prs_b0_t;

#define SPL07_TMP_B2 0x03 		// Temperature Data, 24bit

typedef struct {
	uint8_t tmp_b2 : 8; 		// Temperature Data [23:16]
} spl07_tmp_b2_t;

#define SPL07_TMP_B1 0x04

typedef struct {
	uint8_t tmp_b1 : 8; 		// Temperature Data [15:8]
} spl07_tmp_b1_t;

#define SPL07_TMP_B0 0x05

typedef struct {
	uint8_t tmp_b0 : 8; 		// Temperature Data [7:0]
} spl07_tmp_b0_t;

#define SPL07_PRS_CFG 0x06 		// Pressure Configuration

typedef struct {
	uint8_t pm_prc : 4;			// Pressure Oversampling Rate
	uint8_t pm_rate : 4;		// Pressure Measurement Rate
} spl07_prs_cfg_t;

typedef union {
	uint8_t all;
	spl07_prs_cfg_t bits;
} spl07_prs_cfg_bitfield_t;

#define SPL07_TMP_CFG 0x07 		// Temperature Configuration

typedef struct {
	uint8_t tmp_prc : 4;		// Temperature Oversampling Rate (Precision)
	uint8_t tmp_rate : 4;		// Temperature Measurement Rate
} spl07_tmp_cfg_t;

typedef union {
	uint8_t all;
	spl07_tmp_cfg_t bits;
} spl07_tmp_cfg_bitfield_t;

#define SPL07_MEAS_CFG 0x08 	// Sensor Operating Mode and Status

typedef struct {
	uint8_t meas_ctrl : 3;		// Set measurement mode and type
	uint8_t tmp_ext : 1;		// Internal temperature sensor = 0
	uint8_t prs_rdy : 1;		// Pressure measurement ready
	uint8_t tmp_rdy : 1;		// Temperature measurement ready
	uint8_t sensor_rdy : 1;		// Sensor initialization complete
	uint8_t coef_rdy : 1;		// Coefficients are available
} spl07_meas_cfg_t;

typedef union {
	uint8_t all;
	spl07_meas_cfg_t bits;
} spl07_meas_cfg_bitfield_t;

#define SPL07_CFG_REG 0x09 		// Configuration Register

typedef struct {
	uint8_t spi_mode : 1;		// Set SPI Mode
	uint8_t fifo_en : 1;		// Enable FIFO
	uint8_t p_shift : 1;		// Pressure result bit-shift
	uint8_t t_shift : 1;		// Temperature result bit-shift
	uint8_t int_prs : 1;		// Generate interrupt when a pressure measurement is ready
	uint8_t int_tmp : 1;		// Generate interrupt when a temperature measurement is ready
	uint8_t int_fifo : 1;		// Generate interrupt when the FIFO is full
	uint8_t int_hl : 1;			// Interrupt on SDO pin active level (0 = Active Low)
} spl07_cfg_reg_t;

typedef union {
	uint8_t all;
	spl07_cfg_reg_t bits;
} spl07_cfg_reg_bitfield_t;

#define SPL07_INT_STS 0x0A 		// Interrupt Status

typedef struct {
	uint8_t int_prs : 1;		// Status of pressure measurement interrupt (0 = interrupt not active)
	uint8_t int_tmp : 1;		// Status of temperature measurement interrupt (0 = interrupt not active)
	uint8_t int_fifo_full : 1;	// Status of FIFO interrupt (0 = interrupt not active)
	uint8_t reserved : 5;
} spl07_int_sts_t;

typedef union {
	uint8_t all;
	spl07_int_sts_t bits;
} spl07_int_sts_bitfield_t;

#define SPL07_FIFO_STS 0x0B 	// FIFO Status

typedef struct {
	uint8_t fifo_empty : 1;		// 1 = FIFO empty
	uint8_t fifo_full : 1;		// 1 = FIFO full
	uint8_t reserved : 6;
} spl07_fifo_sts_t;

typedef union {
	uint8_t all;
	spl07_fifo_sts_t bits;
} spl07_fifo_sts_bitfield_t;

#define SPL07_RESET 0x0C 		// Soft reset and FIFO flush

typedef struct {
	uint8_t soft_rst : 4;		// Write '1001' to generate a soft reset
	uint8_t reserved : 3;
	uint8_t fifo_flush : 1;		// 1 = Empty FIFO
} spl07_reset_t;

typedef union {
	uint8_t all;
	spl07_reset_t bits;
} spl07_reset_bitfield_t;

#define SPL07_ID 0x0D 			// Product and Revision ID

typedef struct {
	uint8_t prod_id : 4;		// Product ID
	uint8_t rev_id : 4;			// Revision ID
} spl07_id_t;

typedef union {
	uint8_t all;
	spl07_id_t bits;
} spl07_id_bitfield_t;

#define SPL07_COEF_C0 0x10

typedef struct {
	uint8_t c0 : 8;				// c0[11:4]
} spl07_c0_t;

#define SPL07_COEF_C0_C1 0x11

typedef struct {
	uint8_t c1 : 4;				// c1[11:8]
	uint8_t c0 : 4;				// c0[3:0]
} spl07_c0_c1_t;

#define SPL07_COEF_C1 0x12

typedef struct {
	uint8_t c1 : 8;				// c1[7:0]
} spl07_c1_t;

#define SPL07_COEF_C00_1 0x13

typedef struct {
	uint8_t c00 : 8;			// c00[19:12]
} spl07_c00_1_t;

#define SPL07_COEF_C00_2 0x14

typedef struct {
	uint8_t c00 : 8;			// c00[11:4]
} spl07_c00_2_t;

#define SPL07_COEF_C00_C10 0x15

typedef struct {
	uint8_t c10 : 4;			// c10[19:16]
	uint8_t c00 : 4;			// c00[3:0]
} spl07_c00_C10_t;

#define SPL07_COEF_C10_1 0x16

typedef struct {
	uint8_t c10 : 8;			// c10[15:8]
} spl07_C10_1_t;

#define SPL07_COEF_C10_2 0x17

typedef struct {
	uint8_t c10 : 8;			// c10[7:0]
} spl07_C10_2_t;

#define SPL07_COEF_C01_1 0x18

typedef struct {
	uint8_t c01 : 8;			// c01[15:8]
} spl07_C01_1_t;

#define SPL07_COEF_C01_2 0x19

typedef struct {
	uint8_t c01 : 8;			// c01[7:0]
} spl07_C01_2_t;

#define SPL07_COEF_C11_1 0x1A

typedef struct {
	uint8_t c11 : 8;			// c11[15:8]
} spl07_C11_1_t;

#define SPL07_COEF_C11_2 0x1B

typedef struct {
	uint8_t c11 : 8;			// c11[7:0]
} spl07_C11_2_t;

#define SPL07_COEF_C20_1 0x1C

typedef struct {
	uint8_t c20 : 8;			// c20[15:8]
} spl07_C20_1_t;

#define SPL07_COEF_C20_2 0x1D

typedef struct {
	uint8_t c20 : 8;			// c20[7:0]
} spl07_C20_2_t;

#define SPL07_COEF_C21_1 0x1E

typedef struct {
	uint8_t c21 : 8;			// c21[15:8]
} spl07_C21_1_t;

#define SPL07_COEF_C21_2 0x1F

typedef struct {
	uint8_t c21 : 8;			// c21[7:0]
} spl07_C21_2_t;

#define SPL07_COEF_C30_1 0x20

typedef struct {
	uint8_t c30 : 8;			// c30[15:8]
} spl07_C30_1_t;

#define SPL07_COEF_C30_2 0x21

typedef struct {
	uint8_t c30 : 8;			// c30[7:0]
} spl07_C30_2_t;

#define SPL07_COEF_C31 0x22

typedef struct {
	uint8_t c31 : 8;			// c31[11:4]
} spl07_C31_t;

#define SPL07_COEF_C31_C40 0x23

typedef struct {
	uint8_t c40 : 4;			// c40[11:8]
	uint8_t c31 : 4;			// c31[3:0]
} spl07_C31_C40_t;

#define SPL07_COEF_C40 0x24

typedef struct {
	uint8_t c40 : 8;			// c40[7:0]
} spl07_C40_t;


#endif // SPL07_PRESS_REGS_H_

/*** end of file ***/
