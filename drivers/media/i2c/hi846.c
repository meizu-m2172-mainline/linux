// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 Purism SPC

#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SGBRG10_1X10
#define HI846_RGB_DEPTH			10

/* Frame length lines / vertical timings */
#define HI846_REG_FLL			0x0006
#define HI846_FLL_MAX			0xffff

/* Horizontal timing */
#define HI846_REG_LLP			0x0008
#define HI846_LINE_LENGTH		3800

#define HI846_REG_BINNING_MODE		0x000c

#define HI846_REG_IMAGE_ORIENTATION	0x000e

#define HI846_REG_UNKNOWN_0022		0x0022

#define HI846_REG_Y_ADDR_START_VACT_H	0x0026
#define HI846_REG_Y_ADDR_START_VACT_L	0x0027
#define HI846_REG_UNKNOWN_0028		0x0028

#define HI846_REG_Y_ADDR_END_VACT_H	0x002c
#define HI846_REG_Y_ADDR_END_VACT_L	0x002d

#define HI846_REG_Y_ODD_INC_FOBP	0x002e
#define HI846_REG_Y_EVEN_INC_FOBP	0x002f

#define HI846_REG_Y_ODD_INC_VACT	0x0032
#define HI846_REG_Y_EVEN_INC_VACT	0x0033

#define HI846_REG_GROUPED_PARA_HOLD	0x0046

#define HI846_REG_TG_ENABLE		0x004c

#define HI846_REG_UNKNOWN_005C		0x005c

#define HI846_REG_UNKNOWN_006A		0x006a

/*
 * Long exposure time. Actually, exposure is a 20 bit value that
 * includes the lower 4 bits of 0x0073 too. Only 16 bits are used
 * right now
 */
#define HI846_REG_EXPOSURE		0x0074
#define HI846_EXPOSURE_MIN		6
#define HI846_EXPOSURE_MAX_MARGIN	2
#define HI846_EXPOSURE_STEP		1

/* Analog gain controls from sensor */
#define HI846_REG_ANALOG_GAIN		0x0077
#define HI846_ANAL_GAIN_MIN		0
#define HI846_ANAL_GAIN_MAX		240
#define HI846_ANAL_GAIN_STEP		8

/* Digital gain controls from sensor */
#define HI846_REG_MWB_GR_GAIN_H		0x0078
#define HI846_REG_MWB_GR_GAIN_L		0x0079
#define HI846_REG_MWB_GB_GAIN_H		0x007a
#define HI846_REG_MWB_GB_GAIN_L		0x007b
#define HI846_REG_MWB_R_GAIN_H		0x007c
#define HI846_REG_MWB_R_GAIN_L		0x007d
#define HI846_REG_MWB_B_GAIN_H		0x007e
#define HI846_REG_MWB_B_GAIN_L		0x007f
#define HI846_DGTL_GAIN_MIN		512
#define HI846_DGTL_GAIN_MAX		8191
#define HI846_DGTL_GAIN_STEP		1
#define HI846_DGTL_GAIN_DEFAULT		512

#define HI846_REG_X_ADDR_START_HACT_H	0x0120
#define HI846_REG_X_ADDR_END_HACT_H	0x0122

#define HI846_REG_UNKNOWN_012A		0x012a

#define HI846_REG_UNKNOWN_0200		0x0200

#define HI846_REG_UNKNOWN_021C		0x021c
#define HI846_REG_UNKNOWN_021E		0x021e

#define HI846_REG_UNKNOWN_0402		0x0402
#define HI846_REG_UNKNOWN_0404		0x0404
#define HI846_REG_UNKNOWN_0408		0x0408
#define HI846_REG_UNKNOWN_0410		0x0410
#define HI846_REG_UNKNOWN_0412		0x0412
#define HI846_REG_UNKNOWN_0414		0x0414

#define HI846_REG_UNKNOWN_0418		0x0418

#define HI846_REG_UNKNOWN_051E		0x051e

/* Formatter */
#define HI846_REG_X_START_H		0x0804
#define HI846_REG_X_START_L		0x0805

/* MIPI */
#define HI846_REG_UNKNOWN_0900		0x0900
#define HI846_REG_MIPI_TX_OP_EN		0x0901
#define HI846_REG_MIPI_TX_OP_MODE	0x0902
#define HI846_RAW8			BIT(5)

#define HI846_REG_UNKNOWN_090C		0x090c
#define HI846_REG_UNKNOWN_090E		0x090e

#define HI846_REG_UNKNOWN_0914		0x0914
#define HI846_REG_TLPX			0x0915
#define HI846_REG_TCLK_PREPARE		0x0916
#define HI846_REG_TCLK_ZERO		0x0917
#define HI846_REG_UNKNOWN_0918		0x0918
#define HI846_REG_THS_PREPARE		0x0919
#define HI846_REG_THS_ZERO		0x091a
#define HI846_REG_THS_TRAIL		0x091b
#define HI846_REG_TCLK_POST		0x091c
#define HI846_REG_TCLK_TRAIL_MIN	0x091d
#define HI846_REG_UNKNOWN_091E		0x091e

#define HI846_REG_UNKNOWN_0954		0x0954
#define HI846_REG_UNKNOWN_0956		0x0956
#define HI846_REG_UNKNOWN_0958		0x0958
#define HI846_REG_UNKNOWN_095A		0x095a

/* ISP Common */
#define HI846_REG_MODE_SELECT		0x0a00
#define HI846_MODE_STANDBY		0x0000
#define HI846_MODE_STREAMING		0x0100
#define HI846_REG_FAST_STANDBY_MODE	0x0a02
#define HI846_REG_ISP_EN_H		0x0a04

/* Test Pattern Control */
#define HI846_REG_ISP			0x0a05
#define HI846_REG_ISP_TPG_EN		0x01
#define HI846_REG_TEST_PATTERN		0x020a /* 1-9 */

#define HI846_REG_UNKNOWN_0A0C		0x0a0c

/* Windowing */
#define HI846_REG_X_OUTPUT_SIZE_H	0x0a12
#define HI846_REG_X_OUTPUT_SIZE_L	0x0a13
#define HI846_REG_Y_OUTPUT_SIZE_H	0x0a14
#define HI846_REG_Y_OUTPUT_SIZE_L	0x0a15

/* ISP Common */
#define HI846_REG_PEDESTAL_EN		0x0a1a

#define HI846_REG_UNKNOWN_0A1E		0x0a1e

/* Horizontal Binning Mode */
#define HI846_REG_HBIN_MODE		0x0a22

#define HI846_REG_UNKNOWN_0A24		0x0a24
#define HI846_REG_UNKNOWN_0B02		0x0b02
#define HI846_REG_UNKNOWN_0B10		0x0b10
#define HI846_REG_UNKNOWN_0B12		0x0b12
#define HI846_REG_UNKNOWN_0B14		0x0b14

/* BLC (Black Level Calibration) */
#define HI846_REG_BLC_CTL0		0x0c00

#define HI846_REG_UNKNOWN_0C06		0x0c06
#define HI846_REG_UNKNOWN_0C10		0x0c10
#define HI846_REG_UNKNOWN_0C12		0x0c12
#define HI846_REG_UNKNOWN_0C14		0x0c14
#define HI846_REG_UNKNOWN_0C16		0x0c16

#define HI846_REG_UNKNOWN_0E04		0x0e04

#define HI846_REG_CHIP_ID_L		0x0f16
#define HI846_REG_CHIP_ID_H		0x0f17
#define HI846_CHIP_ID_L			0x46
#define HI846_CHIP_ID_H			0x08

#define HI846_REG_UNKNOWN_0F04		0x0f04
#define HI846_REG_UNKNOWN_0F08		0x0f08

/* PLL */
#define HI846_REG_PLL_CFG_MIPI2_H	0x0f2a
#define HI846_REG_PLL_CFG_MIPI2_L	0x0f2b

#define HI846_REG_UNKNOWN_0F30		0x0f30
#define HI846_REG_PLL_CFG_RAMP1_H	0x0f32
#define HI846_REG_UNKNOWN_0F36		0x0f36
#define HI846_REG_PLL_CFG_MIPI1_H	0x0f38

#define HI846_REG_UNKNOWN_2008		0x2008
#define HI846_REG_UNKNOWN_326E		0x326e

struct hi846_reg {
	u16 address;
	u16 val;
};

struct hi846_reg_list {
	u32 num_of_regs;
	const struct hi846_reg *regs;
};

struct hi846_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timing size */
	u32 llp;

	/* Link frequency needed for this resolution */
	u8 link_freq_index;

	u16 fps;

	/* Vertical timining size */
	u16 frame_len;

	const struct hi846_reg_list reg_list_config;
	const struct hi846_reg_list reg_list_2lane;
	const struct hi846_reg_list reg_list_4lane;

	/* Position inside of the 3264x2448 pixel array */
	struct v4l2_rect crop;
};

static const struct hi846_reg hi846_init_2lane[] = {
	/* Android hi846_wide_txd initSettings from QTI sensormodule. */
	{0x0066, 0x0101},
	{0x2000, 0x98e8},
	{0x2002, 0x00ff},
	{0x2004, 0x0006},
	{0x2008, 0x3fff},
	{0x200a, 0xc314},
	{0x2022, 0x4130},
	{0x2034, 0x1292},
	{0x2036, 0xc02e},
	{0x2038, 0x4130},
	{0x206e, 0xf0b2},
	{0x2070, 0xffbf},
	{0x2072, 0x2004},
	{0x2074, 0x43c2},
	{0x2076, 0x82fa},
	{0x2078, 0x12b0},
	{0x207a, 0xcab0},
	{0x207c, 0x42a2},
	{0x207e, 0x7324},
	{0x2080, 0x4130},
	{0x2082, 0x120b},
	{0x2084, 0x425b},
	{0x2086, 0x008c},
	{0x2088, 0x4292},
	{0x208a, 0x7300},
	{0x208c, 0x82f2},
	{0x208e, 0x4292},
	{0x2090, 0x7302},
	{0x2092, 0x82f4},
	{0x2094, 0x1292},
	{0x2096, 0xc006},
	{0x2098, 0x421f},
	{0x209a, 0x0710},
	{0x209c, 0x523f},
	{0x209e, 0x4f82},
	{0x20a0, 0x82e4},
	{0x20a2, 0x93c2},
	{0x20a4, 0x829f},
	{0x20a6, 0x241e},
	{0x20a8, 0x403e},
	{0x20aa, 0xfffe},
	{0x20ac, 0x40b2},
	{0x20ae, 0xec78},
	{0x20b0, 0x82ec},
	{0x20b2, 0x40b2},
	{0x20b4, 0xec78},
	{0x20b6, 0x82ee},
	{0x20b8, 0x40b2},
	{0x20ba, 0xec78},
	{0x20bc, 0x82f0},
	{0x20be, 0x934b},
	{0x20c0, 0x2405},
	{0x20c2, 0x4e0f},
	{0x20c4, 0x503f},
	{0x20c6, 0xffd8},
	{0x20c8, 0x4f82},
	{0x20ca, 0x82ec},
	{0x20cc, 0x907b},
	{0x20ce, 0x0003},
	{0x20d0, 0x200b},
	{0x20d2, 0x421f},
	{0x20d4, 0x82ec},
	{0x20d6, 0x5e0f},
	{0x20d8, 0x4f82},
	{0x20da, 0x82ee},
	{0x20dc, 0x5e0f},
	{0x20de, 0x4f82},
	{0x20e0, 0x82f0},
	{0x20e2, 0x3c02},
	{0x20e4, 0x432e},
	{0x20e6, 0x3fe2},
	{0x20e8, 0x413b},
	{0x20ea, 0x4130},
	{0x20ec, 0x421f},
	{0x20ee, 0x7100},
	{0x20f0, 0x4f0e},
	{0x20f2, 0x503e},
	{0x20f4, 0xffd8},
	{0x20f6, 0x4e82},
	{0x20f8, 0x7a04},
	{0x20fa, 0x421e},
	{0x20fc, 0x82ec},
	{0x20fe, 0x5f0e},
	{0x2100, 0x4e82},
	{0x2102, 0x7a06},
	{0x2104, 0x0b00},
	{0x2106, 0x7304},
	{0x2108, 0x0050},
	{0x210a, 0x40b2},
	{0x210c, 0xd081},
	{0x210e, 0x0b88},
	{0x2110, 0x421e},
	{0x2112, 0x82ee},
	{0x2114, 0x5f0e},
	{0x2116, 0x4e82},
	{0x2118, 0x7a0e},
	{0x211a, 0x521f},
	{0x211c, 0x82f0},
	{0x211e, 0x4f82},
	{0x2120, 0x7a10},
	{0x2122, 0x0b00},
	{0x2124, 0x7304},
	{0x2126, 0x007a},
	{0x2128, 0x40b2},
	{0x212a, 0x0081},
	{0x212c, 0x0b88},
	{0x212e, 0x4392},
	{0x2130, 0x7a0a},
	{0x2132, 0x0800},
	{0x2134, 0x7a0c},
	{0x2136, 0x0b00},
	{0x2138, 0x7304},
	{0x213a, 0x022b},
	{0x213c, 0x40b2},
	{0x213e, 0xd081},
	{0x2140, 0x0b88},
	{0x2142, 0x0b00},
	{0x2144, 0x7304},
	{0x2146, 0x0255},
	{0x2148, 0x40b2},
	{0x214a, 0x0081},
	{0x214c, 0x0b88},
	{0x214e, 0x9382},
	{0x2150, 0x7112},
	{0x2152, 0x2402},
	{0x2154, 0x4392},
	{0x2156, 0x760e},
	{0x2158, 0x4130},
	{0x215a, 0x120b},
	{0x215c, 0x120a},
	{0x215e, 0x4e0a},
	{0x2160, 0x4f0b},
	{0x2162, 0x4c0e},
	{0x2164, 0x4d0f},
	{0x2166, 0x8a0e},
	{0x2168, 0x7b0f},
	{0x216a, 0x2c02},
	{0x216c, 0x4a0c},
	{0x216e, 0x4b0d},
	{0x2170, 0x4c0e},
	{0x2172, 0x4d0f},
	{0x2174, 0x413a},
	{0x2176, 0x413b},
	{0x2178, 0x4130},
	{0x217a, 0x120b},
	{0x217c, 0x120a},
	{0x217e, 0x1209},
	{0x2180, 0x1208},
	{0x2182, 0x1207},
	{0x2184, 0x1206},
	{0x2186, 0x1205},
	{0x2188, 0x42d2},
	{0x218a, 0x82fa},
	{0x218c, 0x82a0},
	{0x218e, 0x403b},
	{0x2190, 0x00c1},
	{0x2192, 0x4b6f},
	{0x2194, 0x4fc2},
	{0x2196, 0x82d4},
	{0x2198, 0x43c2},
	{0x219a, 0x82d5},
	{0x219c, 0x1292},
	{0x219e, 0xc046},
	{0x21a0, 0x4292},
	{0x21a2, 0x7560},
	{0x21a4, 0x82f6},
	{0x21a6, 0x4292},
	{0x21a8, 0x7562},
	{0x21aa, 0x82f8},
	{0x21ac, 0x93cb},
	{0x21ae, 0x0000},
	{0x21b0, 0x2452},
	{0x21b2, 0x4215},
	{0x21b4, 0x7316},
	{0x21b6, 0x4216},
	{0x21b8, 0x7318},
	{0x21ba, 0x421f},
	{0x21bc, 0x0710},
	{0x21be, 0x4f0e},
	{0x21c0, 0x430f},
	{0x21c2, 0x4507},
	{0x21c4, 0x4608},
	{0x21c6, 0x8e07},
	{0x21c8, 0x7f08},
	{0x21ca, 0x421f},
	{0x21cc, 0x82e2},
	{0x21ce, 0x522f},
	{0x21d0, 0x4f09},
	{0x21d2, 0x430a},
	{0x21d4, 0x470d},
	{0x21d6, 0x480e},
	{0x21d8, 0x490b},
	{0x21da, 0x4a0c},
	{0x21dc, 0x870b},
	{0x21de, 0x780c},
	{0x21e0, 0x2c02},
	{0x21e2, 0x490d},
	{0x21e4, 0x4a0e},
	{0x21e6, 0x4d0f},
	{0x21e8, 0x43d2},
	{0x21ea, 0x01b3},
	{0x21ec, 0x4d82},
	{0x21ee, 0x7324},
	{0x21f0, 0x4292},
	{0x21f2, 0x7540},
	{0x21f4, 0x82e8},
	{0x21f6, 0x4292},
	{0x21f8, 0x7542},
	{0x21fa, 0x82ea},
	{0x21fc, 0x434b},
	{0x21fe, 0x823f},
	{0x2200, 0x4f0c},
	{0x2202, 0x430d},
	{0x2204, 0x421e},
	{0x2206, 0x82e8},
	{0x2208, 0x421f},
	{0x220a, 0x82ea},
	{0x220c, 0x5e0c},
	{0x220e, 0x6f0d},
	{0x2210, 0x870c},
	{0x2212, 0x780d},
	{0x2214, 0x2801},
	{0x2216, 0x435b},
	{0x2218, 0x4bc2},
	{0x221a, 0x82fa},
	{0x221c, 0x93c2},
	{0x221e, 0x829a},
	{0x2220, 0x201a},
	{0x2222, 0x93c2},
	{0x2224, 0x82a0},
	{0x2226, 0x2404},
	{0x2228, 0x43b2},
	{0x222a, 0x7540},
	{0x222c, 0x43b2},
	{0x222e, 0x7542},
	{0x2230, 0x93c2},
	{0x2232, 0x82fa},
	{0x2234, 0x2410},
	{0x2236, 0x503e},
	{0x2238, 0x0003},
	{0x223a, 0x630f},
	{0x223c, 0x4e82},
	{0x223e, 0x82e8},
	{0x2240, 0x4f82},
	{0x2242, 0x82ea},
	{0x2244, 0x450c},
	{0x2246, 0x460d},
	{0x2248, 0x8e0c},
	{0x224a, 0x7f0d},
	{0x224c, 0x2c04},
	{0x224e, 0x4582},
	{0x2250, 0x82e8},
	{0x2252, 0x4682},
	{0x2254, 0x82ea},
	{0x2256, 0x4135},
	{0x2258, 0x4136},
	{0x225a, 0x4137},
	{0x225c, 0x4138},
	{0x225e, 0x4139},
	{0x2260, 0x413a},
	{0x2262, 0x413b},
	{0x2264, 0x4130},
	{0x2266, 0x403e},
	{0x2268, 0x00c2},
	{0x226a, 0x421f},
	{0x226c, 0x7314},
	{0x226e, 0xf07f},
	{0x2270, 0x000c},
	{0x2272, 0x5f4f},
	{0x2274, 0x5f4f},
	{0x2276, 0xdfce},
	{0x2278, 0x0000},
	{0x227a, 0xf0fe},
	{0x227c, 0x000f},
	{0x227e, 0x0000},
	{0x2280, 0x4130},
	{0x2282, 0x120b},
	{0x2284, 0x120a},
	{0x2286, 0x1209},
	{0x2288, 0x1208},
	{0x228a, 0x1207},
	{0x228c, 0x1206},
	{0x228e, 0x93c2},
	{0x2290, 0x00c1},
	{0x2292, 0x249f},
	{0x2294, 0x425e},
	{0x2296, 0x00c2},
	{0x2298, 0xc35e},
	{0x229a, 0x425f},
	{0x229c, 0x82a0},
	{0x229e, 0xdf4e},
	{0x22a0, 0x4ec2},
	{0x22a2, 0x00c2},
	{0x22a4, 0x934f},
	{0x22a6, 0x248f},
	{0x22a8, 0x4217},
	{0x22aa, 0x7316},
	{0x22ac, 0x4218},
	{0x22ae, 0x7318},
	{0x22b0, 0x4326},
	{0x22b2, 0xb3e2},
	{0x22b4, 0x00c2},
	{0x22b6, 0x2482},
	{0x22b8, 0x0900},
	{0x22ba, 0x731c},
	{0x22bc, 0x0800},
	{0x22be, 0x731c},
	{0x22c0, 0x421a},
	{0x22c2, 0x7300},
	{0x22c4, 0x421b},
	{0x22c6, 0x7302},
	{0x22c8, 0x421f},
	{0x22ca, 0x7304},
	{0x22cc, 0x9f82},
	{0x22ce, 0x829c},
	{0x22d0, 0x2c02},
	{0x22d2, 0x531a},
	{0x22d4, 0x630b},
	{0x22d6, 0x4a0e},
	{0x22d8, 0x4b0f},
	{0x22da, 0x821e},
	{0x22dc, 0x82f2},
	{0x22de, 0x721f},
	{0x22e0, 0x82f4},
	{0x22e2, 0x2c68},
	{0x22e4, 0x4a09},
	{0x22e6, 0x9339},
	{0x22e8, 0x3460},
	{0x22ea, 0x0b00},
	{0x22ec, 0x7304},
	{0x22ee, 0x0320},
	{0x22f0, 0x421e},
	{0x22f2, 0x7300},
	{0x22f4, 0x421f},
	{0x22f6, 0x7302},
	{0x22f8, 0x531e},
	{0x22fa, 0x630f},
	{0x22fc, 0x4e0c},
	{0x22fe, 0x4f0d},
	{0x2300, 0x821c},
	{0x2302, 0x82f6},
	{0x2304, 0x721d},
	{0x2306, 0x82f8},
	{0x2308, 0x2c0e},
	{0x230a, 0x93b2},
	{0x230c, 0x7560},
	{0x230e, 0x2003},
	{0x2310, 0x93b2},
	{0x2312, 0x7562},
	{0x2314, 0x2408},
	{0x2316, 0x4e82},
	{0x2318, 0x7540},
	{0x231a, 0x4f82},
	{0x231c, 0x7542},
	{0x231e, 0x4e82},
	{0x2320, 0x82f6},
	{0x2322, 0x4f82},
	{0x2324, 0x82f8},
	{0x2326, 0x4e82},
	{0x2328, 0x7316},
	{0x232a, 0x12b0},
	{0x232c, 0xfe66},
	{0x232e, 0x0900},
	{0x2330, 0x730e},
	{0x2332, 0x403f},
	{0x2334, 0x7316},
	{0x2336, 0x4a09},
	{0x2338, 0x8f29},
	{0x233a, 0x478f},
	{0x233c, 0x0000},
	{0x233e, 0x460c},
	{0x2340, 0x430d},
	{0x2342, 0x421e},
	{0x2344, 0x7300},
	{0x2346, 0x421f},
	{0x2348, 0x7302},
	{0x234a, 0x9c0e},
	{0x234c, 0x23f8},
	{0x234e, 0x9d0f},
	{0x2350, 0x23f6},
	{0x2352, 0x0b00},
	{0x2354, 0x7304},
	{0x2356, 0x01f4},
	{0x2358, 0x5036},
	{0x235a, 0x0006},
	{0x235c, 0x460c},
	{0x235e, 0x430d},
	{0x2360, 0x490e},
	{0x2362, 0x4e0f},
	{0x2364, 0x5f0f},
	{0x2366, 0x7f0f},
	{0x2368, 0xe33f},
	{0x236a, 0x521e},
	{0x236c, 0x82e8},
	{0x236e, 0x621f},
	{0x2370, 0x82ea},
	{0x2372, 0x12b0},
	{0x2374, 0xfd5a},
	{0x2376, 0x4e82},
	{0x2378, 0x7540},
	{0x237a, 0x4f82},
	{0x237c, 0x7542},
	{0x237e, 0x403b},
	{0x2380, 0x7316},
	{0x2382, 0x421c},
	{0x2384, 0x82e4},
	{0x2386, 0x430d},
	{0x2388, 0x4b2f},
	{0x238a, 0x590f},
	{0x238c, 0x4f0e},
	{0x238e, 0x430f},
	{0x2390, 0x12b0},
	{0x2392, 0xfd5a},
	{0x2394, 0x4e8b},
	{0x2396, 0x0000},
	{0x2398, 0x4ba2},
	{0x239a, 0x82ce},
	{0x239c, 0x4382},
	{0x239e, 0x82d0},
	{0x23a0, 0x12b0},
	{0x23a2, 0xfe66},
	{0x23a4, 0xd3d2},
	{0x23a6, 0x00c2},
	{0x23a8, 0x3c16},
	{0x23aa, 0x9329},
	{0x23ac, 0x3bc8},
	{0x23ae, 0x4906},
	{0x23b0, 0x5326},
	{0x23b2, 0x3fc5},
	{0x23b4, 0x4a09},
	{0x23b6, 0x8219},
	{0x23b8, 0x82ce},
	{0x23ba, 0x3f95},
	{0x23bc, 0x0800},
	{0x23be, 0x731c},
	{0x23c0, 0x0900},
	{0x23c2, 0x731c},
	{0x23c4, 0x3f7d},
	{0x23c6, 0x0900},
	{0x23c8, 0x730c},
	{0x23ca, 0x0b00},
	{0x23cc, 0x7304},
	{0x23ce, 0x01f4},
	{0x23d0, 0x3fe9},
	{0x23d2, 0x0900},
	{0x23d4, 0x732c},
	{0x23d6, 0x425f},
	{0x23d8, 0x0788},
	{0x23da, 0x4136},
	{0x23dc, 0x4137},
	{0x23de, 0x4138},
	{0x23e0, 0x4139},
	{0x23e2, 0x413a},
	{0x23e4, 0x413b},
	{0x23e6, 0x4130},
	{0x23fe, 0xc056},
	{0x3236, 0xfc22},
	{0x323a, 0xfcec},
	{0x323c, 0xfc82},
	{0x323e, 0xfd7a},
	{0x3246, 0xfe82},
	{0x3248, 0xfc34},
	{0x324e, 0xfc6e},
	{0x326a, 0xc374},
	{0x326c, 0xc37c},
	{0x326e, 0x0000},
	{0x3270, 0xc378},
	{0x32e2, 0x0020},
	{0x0a00, 0x0000},
	{0x0e04, 0x0012},
	{0x002e, 0x1111},
	{0x0032, 0x1111},
	{0x0022, 0x0008},
	{0x0026, 0x0040},
	{0x0028, 0x0017},
	{0x002c, 0x09cf},
	{0x005c, 0x2101},
	{0x0006, 0x09de},
	{0x0008, 0x0ed8},
	{0x000e, 0x0100},
	{0x000c, 0x0022},
	{0x0a22, 0x0000},
	{0x0a24, 0x0000},
	{0x0804, 0x0000},
	{0x0a12, 0x0cc0},
	{0x0a14, 0x0990},
	{0x0710, 0x09b0},
	{0x0074, 0x09d8},
	{0x0076, 0x0000},
	{0x051e, 0x0000},
	{0x0200, 0x0400},
	{0x0a1a, 0x0c00},
	{0x0a0c, 0x0010},
	{0x0a1e, 0x0ccf},
	{0x0402, 0x0110},
	{0x0404, 0x00f4},
	{0x0408, 0x0000},
	{0x0410, 0x008d},
	{0x0412, 0x011a},
	{0x0414, 0x864c},
	{0x021c, 0x0001},
	{0x0c00, 0x9950},
	{0x0c06, 0x0021},
	{0x0c10, 0x0040},
	{0x0c12, 0x0040},
	{0x0c14, 0x0040},
	{0x0c16, 0x0040},
	{0x0a02, 0x0100},
	{0x0a04, 0x014a},
	{0x0418, 0x0000},
	{0x0128, 0x0028},
	{0x012a, 0xffff},
	{0x0120, 0x0046},
	{0x0122, 0x0376},
	{0x012c, 0x0020},
	{0x012e, 0xffff},
	{0x0124, 0x0040},
	{0x0126, 0x0378},
	{0x0746, 0x0050},
	{0x0748, 0x01d5},
	{0x074a, 0x022b},
	{0x074c, 0x03b0},
	{0x0756, 0x043f},
	{0x0758, 0x3f1d},
	{0x0b02, 0xe04d},
	{0x0b10, 0x6821},
	{0x0b12, 0x0120},
	{0x0b14, 0x0001},
	{0x2008, 0x38fd},
	{0x326e, 0x0000},
	{0x0900, 0x0300},
	{0x0902, 0xc319},
	{0x0914, 0xc109},
	{0x0916, 0x061a},
	{0x0918, 0x0407},
	{0x091a, 0x0a0b},
	{0x091c, 0x0e08},
	{0x091e, 0x0a00},
	{0x090c, 0x0427},
	{0x090e, 0x0059},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xca80},
	{0x095a, 0x9240},
	{0x0040, 0x0200},
	{0x0042, 0x0100},
	{0x0d04, 0x0000},
	{0x0f08, 0x2f04},
	{0x0f30, 0x001f},
	{0x0f36, 0x001f},
	{0x0f04, 0x3a00},
	{0x0f32, 0x025a},
	{0x0f38, 0x025a},
	{0x0f2a, 0x4124},
	{0x006a, 0x0100},
	{0x004c, 0x0100},
	{0x0044, 0x0001},
};

static const struct hi846_reg hi846_init_4lane[] = {
	{0x2000, 0x987a},
	{0x2002, 0x00ff},
	{0x2004, 0x0047},
	{0x2006, 0x3fff},
	{0x2008, 0x3fff},
	{0x200a, 0xc216},
	{0x200c, 0x1292},
	{0x200e, 0xc01a},
	{0x2010, 0x403d},
	{0x2012, 0x000e},
	{0x2014, 0x403e},
	{0x2016, 0x0b80},
	{0x2018, 0x403f},
	{0x201a, 0x82ae},
	{0x201c, 0x1292},
	{0x201e, 0xc00c},
	{0x2020, 0x4130},
	{0x2022, 0x43e2},
	{0x2024, 0x0180},
	{0x2026, 0x4130},
	{0x2028, 0x7400},
	{0x202a, 0x5000},
	{0x202c, 0x0253},
	{0x202e, 0x0ad1},
	{0x2030, 0x2360},
	{0x2032, 0x0009},
	{0x2034, 0x5020},
	{0x2036, 0x000b},
	{0x2038, 0x0002},
	{0x203a, 0x0044},
	{0x203c, 0x0016},
	{0x203e, 0x1792},
	{0x2040, 0x7002},
	{0x2042, 0x154f},
	{0x2044, 0x00d5},
	{0x2046, 0x000b},
	{0x2048, 0x0019},
	{0x204a, 0x1698},
	{0x204c, 0x000e},
	{0x204e, 0x099a},
	{0x2050, 0x0058},
	{0x2052, 0x7000},
	{0x2054, 0x1799},
	{0x2056, 0x0310},
	{0x2058, 0x03c3},
	{0x205a, 0x004c},
	{0x205c, 0x064a},
	{0x205e, 0x0001},
	{0x2060, 0x0007},
	{0x2062, 0x0bc7},
	{0x2064, 0x0055},
	{0x2066, 0x7000},
	{0x2068, 0x1550},
	{0x206a, 0x158a},
	{0x206c, 0x0004},
	{0x206e, 0x1488},
	{0x2070, 0x7010},
	{0x2072, 0x1508},
	{0x2074, 0x0004},
	{0x2076, 0x0016},
	{0x2078, 0x03d5},
	{0x207a, 0x0055},
	{0x207c, 0x08ca},
	{0x207e, 0x2019},
	{0x2080, 0x0007},
	{0x2082, 0x7057},
	{0x2084, 0x0fc7},
	{0x2086, 0x5041},
	{0x2088, 0x12c8},
	{0x208a, 0x5060},
	{0x208c, 0x5080},
	{0x208e, 0x2084},
	{0x2090, 0x12c8},
	{0x2092, 0x7800},
	{0x2094, 0x0802},
	{0x2096, 0x040f},
	{0x2098, 0x1007},
	{0x209a, 0x0803},
	{0x209c, 0x080b},
	{0x209e, 0x3803},
	{0x20a0, 0x0807},
	{0x20a2, 0x0404},
	{0x20a4, 0x0400},
	{0x20a6, 0xffff},
	{0x20a8, 0xf0b2},
	{0x20aa, 0xffef},
	{0x20ac, 0x0a84},
	{0x20ae, 0x1292},
	{0x20b0, 0xc02e},
	{0x20b2, 0x4130},
	{0x20b4, 0xf0b2},
	{0x20b6, 0xffbf},
	{0x20b8, 0x2004},
	{0x20ba, 0x403f},
	{0x20bc, 0x00c3},
	{0x20be, 0x4fe2},
	{0x20c0, 0x8318},
	{0x20c2, 0x43cf},
	{0x20c4, 0x0000},
	{0x20c6, 0x9382},
	{0x20c8, 0xc314},
	{0x20ca, 0x2003},
	{0x20cc, 0x12b0},
	{0x20ce, 0xcab0},
	{0x20d0, 0x4130},
	{0x20d2, 0x12b0},
	{0x20d4, 0xc90a},
	{0x20d6, 0x4130},
	{0x20d8, 0x42d2},
	{0x20da, 0x8318},
	{0x20dc, 0x00c3},
	{0x20de, 0x9382},
	{0x20e0, 0xc314},
	{0x20e2, 0x2009},
	{0x20e4, 0x120b},
	{0x20e6, 0x120a},
	{0x20e8, 0x1209},
	{0x20ea, 0x1208},
	{0x20ec, 0x1207},
	{0x20ee, 0x1206},
	{0x20f0, 0x4030},
	{0x20f2, 0xc15e},
	{0x20f4, 0x4130},
	{0x20f6, 0x1292},
	{0x20f8, 0xc008},
	{0x20fa, 0x4130},
	{0x20fc, 0x42d2},
	{0x20fe, 0x82a1},
	{0x2100, 0x00c2},
	{0x2102, 0x1292},
	{0x2104, 0xc040},
	{0x2106, 0x4130},
	{0x2108, 0x1292},
	{0x210a, 0xc006},
	{0x210c, 0x42a2},
	{0x210e, 0x7324},
	{0x2110, 0x9382},
	{0x2112, 0xc314},
	{0x2114, 0x2011},
	{0x2116, 0x425f},
	{0x2118, 0x82a1},
	{0x211a, 0xf25f},
	{0x211c, 0x00c1},
	{0x211e, 0xf35f},
	{0x2120, 0x2406},
	{0x2122, 0x425f},
	{0x2124, 0x00c0},
	{0x2126, 0xf37f},
	{0x2128, 0x522f},
	{0x212a, 0x4f82},
	{0x212c, 0x7324},
	{0x212e, 0x425f},
	{0x2130, 0x82d4},
	{0x2132, 0xf35f},
	{0x2134, 0x4fc2},
	{0x2136, 0x01b3},
	{0x2138, 0x93c2},
	{0x213a, 0x829f},
	{0x213c, 0x2421},
	{0x213e, 0x403e},
	{0x2140, 0xfffe},
	{0x2142, 0x40b2},
	{0x2144, 0xec78},
	{0x2146, 0x831c},
	{0x2148, 0x40b2},
	{0x214a, 0xec78},
	{0x214c, 0x831e},
	{0x214e, 0x40b2},
	{0x2150, 0xec78},
	{0x2152, 0x8320},
	{0x2154, 0xb3d2},
	{0x2156, 0x008c},
	{0x2158, 0x2405},
	{0x215a, 0x4e0f},
	{0x215c, 0x503f},
	{0x215e, 0xffd8},
	{0x2160, 0x4f82},
	{0x2162, 0x831c},
	{0x2164, 0x90f2},
	{0x2166, 0x0003},
	{0x2168, 0x008c},
	{0x216a, 0x2401},
	{0x216c, 0x4130},
	{0x216e, 0x421f},
	{0x2170, 0x831c},
	{0x2172, 0x5e0f},
	{0x2174, 0x4f82},
	{0x2176, 0x831e},
	{0x2178, 0x5e0f},
	{0x217a, 0x4f82},
	{0x217c, 0x8320},
	{0x217e, 0x3ff6},
	{0x2180, 0x432e},
	{0x2182, 0x3fdf},
	{0x2184, 0x421f},
	{0x2186, 0x7100},
	{0x2188, 0x4f0e},
	{0x218a, 0x503e},
	{0x218c, 0xffd8},
	{0x218e, 0x4e82},
	{0x2190, 0x7a04},
	{0x2192, 0x421e},
	{0x2194, 0x831c},
	{0x2196, 0x5f0e},
	{0x2198, 0x4e82},
	{0x219a, 0x7a06},
	{0x219c, 0x0b00},
	{0x219e, 0x7304},
	{0x21a0, 0x0050},
	{0x21a2, 0x40b2},
	{0x21a4, 0xd081},
	{0x21a6, 0x0b88},
	{0x21a8, 0x421e},
	{0x21aa, 0x831e},
	{0x21ac, 0x5f0e},
	{0x21ae, 0x4e82},
	{0x21b0, 0x7a0e},
	{0x21b2, 0x521f},
	{0x21b4, 0x8320},
	{0x21b6, 0x4f82},
	{0x21b8, 0x7a10},
	{0x21ba, 0x0b00},
	{0x21bc, 0x7304},
	{0x21be, 0x007a},
	{0x21c0, 0x40b2},
	{0x21c2, 0x0081},
	{0x21c4, 0x0b88},
	{0x21c6, 0x4392},
	{0x21c8, 0x7a0a},
	{0x21ca, 0x0800},
	{0x21cc, 0x7a0c},
	{0x21ce, 0x0b00},
	{0x21d0, 0x7304},
	{0x21d2, 0x022b},
	{0x21d4, 0x40b2},
	{0x21d6, 0xd081},
	{0x21d8, 0x0b88},
	{0x21da, 0x0b00},
	{0x21dc, 0x7304},
	{0x21de, 0x0255},
	{0x21e0, 0x40b2},
	{0x21e2, 0x0081},
	{0x21e4, 0x0b88},
	{0x21e6, 0x4130},
	{0x23fe, 0xc056},
	{0x3232, 0xfc0c},
	{0x3236, 0xfc22},
	{0x3238, 0xfcfc},
	{0x323a, 0xfd84},
	{0x323c, 0xfd08},
	{0x3246, 0xfcd8},
	{0x3248, 0xfca8},
	{0x324e, 0xfcb4},
	{0x326a, 0x8302},
	{0x326c, 0x830a},
	{0x326e, 0x0000},
	{0x32ca, 0xfc28},
	{0x32cc, 0xc3bc},
	{0x32ce, 0xc34c},
	{0x32d0, 0xc35a},
	{0x32d2, 0xc368},
	{0x32d4, 0xc376},
	{0x32d6, 0xc3c2},
	{0x32d8, 0xc3e6},
	{0x32da, 0x0003},
	{0x32dc, 0x0003},
	{0x32de, 0x00c7},
	{0x32e0, 0x0031},
	{0x32e2, 0x0031},
	{0x32e4, 0x0031},
	{0x32e6, 0xfc28},
	{0x32e8, 0xc3bc},
	{0x32ea, 0xc384},
	{0x32ec, 0xc392},
	{0x32ee, 0xc3a0},
	{0x32f0, 0xc3ae},
	{0x32f2, 0xc3c4},
	{0x32f4, 0xc3e6},
	{0x32f6, 0x0003},
	{0x32f8, 0x0003},
	{0x32fa, 0x00c7},
	{0x32fc, 0x0031},
	{0x32fe, 0x0031},
	{0x3300, 0x0031},
	{0x3302, 0x82ca},
	{0x3304, 0xc164},
	{0x3306, 0x82e6},
	{0x3308, 0xc19c},
	{0x330a, 0x001f},
	{0x330c, 0x001a},
	{0x330e, 0x0034},
	{0x3310, 0x0000},
	{0x3312, 0x0000},
	{0x3314, 0xfc94},
	{0x3316, 0xc3d8},

	{0x0a00, 0x0000},
	{0x0e04, 0x0012},
	{0x002e, 0x1111},
	{0x0032, 0x1111},
	{0x0022, 0x0008},
	{0x0026, 0x0040},
	{0x0028, 0x0017},
	{0x002c, 0x09cf},
	{0x005c, 0x2101},
	{0x0006, 0x09de},
	{0x0008, 0x0ed8},
	{0x000e, 0x0100},
	{0x000c, 0x0022},
	{0x0a22, 0x0000},
	{0x0a24, 0x0000},
	{0x0804, 0x0000},
	{0x0a12, 0x0cc0},
	{0x0a14, 0x0990},
	{0x0074, 0x09d8},
	{0x0076, 0x0000},
	{0x051e, 0x0000},
	{0x0200, 0x0400},
	{0x0a1a, 0x0c00},
	{0x0a0c, 0x0010},
	{0x0a1e, 0x0ccf},
	{0x0402, 0x0110},
	{0x0404, 0x00f4},
	{0x0408, 0x0000},
	{0x0410, 0x008d},
	{0x0412, 0x011a},
	{0x0414, 0x864c},
	/* for OTP */
	{0x021c, 0x0003},
	{0x021e, 0x0235},
	/* for OTP */
	{0x0c00, 0x9950},
	{0x0c06, 0x0021},
	{0x0c10, 0x0040},
	{0x0c12, 0x0040},
	{0x0c14, 0x0040},
	{0x0c16, 0x0040},
	{0x0a02, 0x0100},
	{0x0a04, 0x015a},
	{0x0418, 0x0000},
	{0x0128, 0x0028},
	{0x012a, 0xffff},
	{0x0120, 0x0046},
	{0x0122, 0x0376},
	{0x012c, 0x0020},
	{0x012e, 0xffff},
	{0x0124, 0x0040},
	{0x0126, 0x0378},
	{0x0746, 0x0050},
	{0x0748, 0x01d5},
	{0x074a, 0x022b},
	{0x074c, 0x03b0},
	{0x0756, 0x043f},
	{0x0758, 0x3f1d},
	{0x0b02, 0xe04d},
	{0x0b10, 0x6821},
	{0x0b12, 0x0120},
	{0x0b14, 0x0001},
	{0x2008, 0x38fd},
	{0x326e, 0x0000},
	{0x0900, 0x0300},
	{0x0902, 0xc319},
	{0x0914, 0xc109},
	{0x0916, 0x061a},
	{0x0918, 0x0407},
	{0x091a, 0x0a0b},
	{0x091c, 0x0e08},
	{0x091e, 0x0a00},
	{0x090c, 0x0427},
	{0x090e, 0x0059},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xca80},
	{0x095a, 0x9240},
	{0x0f08, 0x2f04},
	{0x0f30, 0x001f},
	{0x0f36, 0x001f},
	{0x0f04, 0x3a00},
	{0x0f32, 0x025a},
	{0x0f38, 0x025a},
	{0x0f2a, 0x4124},
	{0x006a, 0x0100},
	{0x004c, 0x0100},
	{0x0044, 0x0001},
};

static const struct hi846_reg mode_640x480_config[] = {
	{HI846_REG_MODE_SELECT,			0x0000},
	{HI846_REG_Y_ODD_INC_FOBP,		0x3311},
	{HI846_REG_Y_ODD_INC_VACT,		0x3311},
	{HI846_REG_Y_ADDR_START_VACT_H,		0x0348},
	{HI846_REG_Y_ADDR_END_VACT_H,		0x0707},
	{HI846_REG_UNKNOWN_005C,		0x4202},
	{HI846_REG_FLL,				0x09de},
	{HI846_REG_LLP,				0x0ed8},
	{HI846_REG_BINNING_MODE,		0x0122},
	{HI846_REG_HBIN_MODE,			0x0100},
	{HI846_REG_UNKNOWN_0A24,		0x0000},
	{HI846_REG_X_START_H,			0x03e0},
	{HI846_REG_X_OUTPUT_SIZE_H,		0x0280},
	{HI846_REG_Y_OUTPUT_SIZE_H,		0x01e0},
	{HI846_REG_EXPOSURE,			0x09d8},

	/* For OTP */
	{HI846_REG_UNKNOWN_021C,		0x0001},
	{HI846_REG_UNKNOWN_021E,		0x0235},

	{HI846_REG_ISP_EN_H,			0x016a},
	{HI846_REG_UNKNOWN_0418,		0x0410},
	{HI846_REG_UNKNOWN_0B02,		0xe04d},
	{HI846_REG_UNKNOWN_0B10,		0x6c21},
	{HI846_REG_UNKNOWN_0B12,		0x0120},
	{HI846_REG_UNKNOWN_0B14,		0x0005},
	{HI846_REG_UNKNOWN_2008,		0x38fd},
	{HI846_REG_UNKNOWN_326E,		0x0000},
};

static const struct hi846_reg mode_640x480_mipi_2lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0x4319},
	{HI846_REG_UNKNOWN_0914,		0xc105},
	{HI846_REG_TCLK_PREPARE,		0x030c},
	{HI846_REG_UNKNOWN_0918,		0x0304},
	{HI846_REG_THS_ZERO,			0x0708},
	{HI846_REG_TCLK_POST,			0x0b04},
	{HI846_REG_UNKNOWN_091E,		0x0500},
	{HI846_REG_UNKNOWN_090C,		0x0208},
	{HI846_REG_UNKNOWN_090E,		0x009a},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4924},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_640x480_mipi_4lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0xc319},
	{HI846_REG_UNKNOWN_0914,		0xc105},
	{HI846_REG_TCLK_PREPARE,		0x030c},
	{HI846_REG_UNKNOWN_0918,		0x0304},
	{HI846_REG_THS_ZERO,			0x0708},
	{HI846_REG_TCLK_POST,			0x0b04},
	{HI846_REG_UNKNOWN_091E,		0x0500},
	{HI846_REG_UNKNOWN_090C,		0x0208},
	{HI846_REG_UNKNOWN_090E,		0x009a},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4924},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_1280x720_config[] = {
	{HI846_REG_MODE_SELECT,			0x0000},
	{HI846_REG_Y_ODD_INC_FOBP,		0x3311},
	{HI846_REG_Y_ODD_INC_VACT,		0x3311},
	{HI846_REG_Y_ADDR_START_VACT_H,		0x0238},
	{HI846_REG_Y_ADDR_END_VACT_H,		0x07d7},
	{HI846_REG_UNKNOWN_005C,		0x4202},
	{HI846_REG_FLL,				0x04ef},
	{HI846_REG_LLP,				0x0ed8},
	{HI846_REG_BINNING_MODE,		0x0122},
	{HI846_REG_HBIN_MODE,			0x0100},
	{HI846_REG_UNKNOWN_0A24,		0x0000},
	{HI846_REG_X_START_H,			0x00b0},
	{HI846_REG_X_OUTPUT_SIZE_H,		0x0500},
	{HI846_REG_Y_OUTPUT_SIZE_H,		0x02d0},
	{HI846_REG_EXPOSURE,			0x04e9},

	/* For OTP */
	{HI846_REG_UNKNOWN_021C,		0x0001},
	{HI846_REG_UNKNOWN_021E,		0x0235},

	{HI846_REG_ISP_EN_H,			0x016a},
	{HI846_REG_UNKNOWN_0418,		0x0410},
	{HI846_REG_UNKNOWN_0B02,		0xe04d},
	{HI846_REG_UNKNOWN_0B10,		0x6c21},
	{HI846_REG_UNKNOWN_0B12,		0x0120},
	{HI846_REG_UNKNOWN_0B14,		0x0005},
	{HI846_REG_UNKNOWN_2008,		0x38fd},
	{HI846_REG_UNKNOWN_326E,		0x0000},
};

static const struct hi846_reg mode_1280x720_mipi_2lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0xc319},
	{HI846_REG_UNKNOWN_0914,		0xc105},
	{HI846_REG_TCLK_PREPARE,		0x030c},
	{HI846_REG_UNKNOWN_0918,		0x0304},
	{HI846_REG_THS_ZERO,			0x0708},
	{HI846_REG_TCLK_POST,			0x0b04},
	{HI846_REG_UNKNOWN_091E,		0x0500},
	{HI846_REG_UNKNOWN_090C,		0x0208},
	{HI846_REG_UNKNOWN_090E,		0x008a},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4924},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_1280x720_mipi_4lane[] = {
	/* 360Mbps */
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0xc319},
	{HI846_REG_UNKNOWN_0914,		0xc105},
	{HI846_REG_TCLK_PREPARE,		0x030c},
	{HI846_REG_UNKNOWN_0918,		0x0304},
	{HI846_REG_THS_ZERO,			0x0708},
	{HI846_REG_TCLK_POST,			0x0b04},
	{HI846_REG_UNKNOWN_091E,		0x0500},
	{HI846_REG_UNKNOWN_090C,		0x0208},
	{HI846_REG_UNKNOWN_090E,		0x008a},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4924},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_1632x1224_config[] = {
	{HI846_REG_MODE_SELECT,			0x0000},
	{HI846_REG_Y_ODD_INC_FOBP,		0x3311},
	{HI846_REG_Y_ODD_INC_VACT,		0x3311},
	{HI846_REG_Y_ADDR_START_VACT_H,		0x0040},
	{HI846_REG_Y_ADDR_END_VACT_H,		0x09cf},
	{HI846_REG_UNKNOWN_005C,		0x4202},
	{HI846_REG_FLL,				0x09de},
	{HI846_REG_LLP,				0x0ed8},
	{HI846_REG_BINNING_MODE,		0x0122},
	{HI846_REG_HBIN_MODE,			0x0100},
	{HI846_REG_UNKNOWN_0A24,		0x0000},
	{HI846_REG_X_START_H,			0x0000},
	{HI846_REG_X_OUTPUT_SIZE_H,		0x0660},
	{HI846_REG_Y_OUTPUT_SIZE_H,		0x04c8},
	{HI846_REG_EXPOSURE,			0x09d8},

	/* For OTP */
	{HI846_REG_UNKNOWN_021C,		0x0003},
	{HI846_REG_UNKNOWN_021E,		0x0235},

	{HI846_REG_ISP_EN_H,			0x016a},
	{HI846_REG_UNKNOWN_0418,		0x0000},
	{HI846_REG_UNKNOWN_0B02,		0xe04d},
	{HI846_REG_UNKNOWN_0B10,		0x6c21},
	{HI846_REG_UNKNOWN_0B12,		0x0120},
	{HI846_REG_UNKNOWN_0B14,		0x0005},
	{HI846_REG_UNKNOWN_2008,		0x38fd},
	{HI846_REG_UNKNOWN_326E,		0x0000},
};

static const struct hi846_reg mode_1632x1224_mipi_2lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0x4319},
	{HI846_REG_UNKNOWN_0914,		0xc109},
	{HI846_REG_TCLK_PREPARE,		0x061a},
	{HI846_REG_UNKNOWN_0918,		0x0407},
	{HI846_REG_THS_ZERO,			0x0a0b},
	{HI846_REG_TCLK_POST,			0x0e08},
	{HI846_REG_UNKNOWN_091E,		0x0a00},
	{HI846_REG_UNKNOWN_090C,		0x0427},
	{HI846_REG_UNKNOWN_090E,		0x0069},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4124},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_1632x1224_mipi_4lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0xc319},
	{HI846_REG_UNKNOWN_0914,		0xc105},
	{HI846_REG_TCLK_PREPARE,		0x030c},
	{HI846_REG_UNKNOWN_0918,		0x0304},
	{HI846_REG_THS_ZERO,			0x0708},
	{HI846_REG_TCLK_POST,			0x0b04},
	{HI846_REG_UNKNOWN_091E,		0x0500},
	{HI846_REG_UNKNOWN_090C,		0x0208},
	{HI846_REG_UNKNOWN_090E,		0x001c},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4924},
	{HI846_REG_TG_ENABLE,			0x0100},
};

static const struct hi846_reg mode_3264x2448_config[] = {
	{HI846_REG_MODE_SELECT,			0x0000},
	{HI846_REG_UNKNOWN_0E04,		0x0012},
	{HI846_REG_Y_ODD_INC_FOBP,		0x1111},
	{HI846_REG_Y_ODD_INC_VACT,		0x1111},
	{HI846_REG_UNKNOWN_0022,		0x0008},
	{HI846_REG_Y_ADDR_START_VACT_H,		0x0040},
	{HI846_REG_UNKNOWN_0028,		0x0017},
	{HI846_REG_Y_ADDR_END_VACT_H,		0x09cf},
	{HI846_REG_UNKNOWN_005C,		0x2101},
	{HI846_REG_FLL,				0x09de},
	{HI846_REG_LLP,				0x0ed8},
	{HI846_REG_IMAGE_ORIENTATION,		0x0100},
	{HI846_REG_BINNING_MODE,		0x0022},
	{HI846_REG_HBIN_MODE,			0x0000},
	{HI846_REG_UNKNOWN_0A24,		0x0000},
	{HI846_REG_X_START_H,			0x0000},
	{HI846_REG_X_OUTPUT_SIZE_H,		0x0cc0},
	{HI846_REG_Y_OUTPUT_SIZE_H,		0x0990},
	{HI846_REG_EXPOSURE,			0x09d8},
	{0x0076,				0x0000},
	{HI846_REG_UNKNOWN_051E,		0x0000},
	{HI846_REG_UNKNOWN_0200,		0x0400},
	{HI846_REG_PEDESTAL_EN,			0x0c00},
	{HI846_REG_UNKNOWN_0A0C,		0x0010},
	{HI846_REG_UNKNOWN_0A1E,		0x0ccf},
	{HI846_REG_UNKNOWN_0402,		0x0110},
	{HI846_REG_UNKNOWN_0404,		0x00f4},
	{HI846_REG_UNKNOWN_0408,		0x0000},
	{HI846_REG_UNKNOWN_0410,		0x008d},
	{HI846_REG_UNKNOWN_0412,		0x011a},
	{HI846_REG_UNKNOWN_0414,		0x864c},
	{HI846_REG_UNKNOWN_021C,		0x0003},
	{HI846_REG_UNKNOWN_021E,		0x0235},
	{HI846_REG_BLC_CTL0,			0x9950},
	{HI846_REG_UNKNOWN_0C06,		0x0021},
	{HI846_REG_UNKNOWN_0C10,		0x0040},
	{HI846_REG_UNKNOWN_0C12,		0x0040},
	{HI846_REG_UNKNOWN_0C14,		0x0040},
	{HI846_REG_UNKNOWN_0C16,		0x0040},
	{HI846_REG_FAST_STANDBY_MODE,		0x0100},
	{HI846_REG_ISP_EN_H,			0x016a},
	{HI846_REG_UNKNOWN_0418,		0x0000},
	{0x0128,				0x0028},
	{HI846_REG_UNKNOWN_012A,		0xffff},
	{HI846_REG_X_ADDR_START_HACT_H,		0x0046},
	{HI846_REG_X_ADDR_END_HACT_H,		0x0376},
	{0x012c,				0x0020},
	{0x012e,				0xffff},
	{0x0124,				0x0040},
	{0x0126,				0x0378},
	{0x0746,				0x0050},
	{0x0748,				0x01d5},
	{0x074a,				0x022b},
	{0x074c,				0x03b0},
	{0x0756,				0x043f},
	{0x0758,				0x3f1d},
	{HI846_REG_UNKNOWN_0B02,		0xe04d},
	{HI846_REG_UNKNOWN_0B10,		0x6821},
	{HI846_REG_UNKNOWN_0B12,		0x0120},
	{HI846_REG_UNKNOWN_0B14,		0x0001},
	{HI846_REG_UNKNOWN_2008,		0x38fd},
	{HI846_REG_UNKNOWN_326E,		0x0000},
};

static const struct hi846_reg mode_3264x2448_mipi_4lane[] = {
	{HI846_REG_UNKNOWN_0900,		0x0300},
	{HI846_REG_MIPI_TX_OP_MODE,		0xc319},
	{HI846_REG_UNKNOWN_0914,		0xc109},
	{HI846_REG_TCLK_PREPARE,		0x061a},
	{HI846_REG_UNKNOWN_0918,		0x0407},
	{HI846_REG_THS_ZERO,			0x0a0b},
	{HI846_REG_TCLK_POST,			0x0e08},
	{HI846_REG_UNKNOWN_091E,		0x0a00},
	{HI846_REG_UNKNOWN_090C,		0x0427},
	{HI846_REG_UNKNOWN_090E,		0x0059},
	{HI846_REG_UNKNOWN_0954,		0x0089},
	{HI846_REG_UNKNOWN_0956,		0x0000},
	{HI846_REG_UNKNOWN_0958,		0xca80},
	{HI846_REG_UNKNOWN_095A,		0x9240},
	{HI846_REG_UNKNOWN_0F08,		0x2f04},
	{HI846_REG_UNKNOWN_0F30,		0x001f},
	{HI846_REG_UNKNOWN_0F36,		0x001f},
	{HI846_REG_UNKNOWN_0F04,		0x3a00},
	{HI846_REG_PLL_CFG_RAMP1_H,		0x025a},
	{HI846_REG_PLL_CFG_MIPI1_H,		0x025a},
	{HI846_REG_PLL_CFG_MIPI2_H,		0x4124},
	{HI846_REG_UNKNOWN_006A,		0x0100},
	{HI846_REG_TG_ENABLE,			0x0100},
	{0x0044,				0x0001},
};

static const char * const hi846_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade To Grey Colour Bars",
	"PN9",
	"Gradient Horizontal",
	"Gradient Vertical",
	"Check Board",
	"Slant Pattern",
	"Resolution Pattern",
};

#define FREQ_INDEX_640	0
#define FREQ_INDEX_1280	1
static const s64 hi846_link_freqs[] = {
	[FREQ_INDEX_640] = 80000000,
	[FREQ_INDEX_1280] = 180000000,
};

static const struct hi846_reg_list hi846_init_regs_list_2lane = {
	.num_of_regs = ARRAY_SIZE(hi846_init_2lane),
	.regs = hi846_init_2lane,
};

static const struct hi846_reg_list hi846_init_regs_list_4lane = {
	.num_of_regs = ARRAY_SIZE(hi846_init_2lane),
	.regs = hi846_init_2lane,
};

static const struct hi846_mode supported_modes[] = {
	{
		.width = 640,
		.height = 480,
		.link_freq_index = FREQ_INDEX_640,
		.fps = 30,
		.frame_len = 2526,
		.llp = HI846_LINE_LENGTH,
		.reg_list_config = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_config),
			.regs = mode_640x480_config,
		},
		.reg_list_2lane = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_mipi_2lane),
			.regs = mode_640x480_mipi_2lane,
		},
		.reg_list_4lane = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_mipi_4lane),
			.regs = mode_640x480_mipi_4lane,
		},
		.crop = {
			.left = 0x3e0,
			.top = 0x348,
			.width = 640 * 2,
			.height = 480 * 2,
		},
	},
	{
		.width = 1280,
		.height = 720,
		.link_freq_index = FREQ_INDEX_1280,
		.fps = 90,
		.frame_len = 1263,
		.llp = HI846_LINE_LENGTH,
		.reg_list_config = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_config),
			.regs = mode_1280x720_config,
		},
		.reg_list_2lane = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_mipi_2lane),
			.regs = mode_1280x720_mipi_2lane,
		},
		.reg_list_4lane = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_mipi_4lane),
			.regs = mode_1280x720_mipi_4lane,
		},
		.crop = {
			.left = 0xb0,
			.top = 0x238,
			.width = 1280 * 2,
			.height = 720 * 2,
		},
	},
	{
		.width = 1632,
		.height = 1224,
		.link_freq_index = FREQ_INDEX_1280,
		.fps = 30,
		.frame_len = 2526,
		.llp = HI846_LINE_LENGTH,
		.reg_list_config = {
			.num_of_regs = ARRAY_SIZE(mode_1632x1224_config),
			.regs = mode_1632x1224_config,
		},
		.reg_list_2lane = {
			.num_of_regs = ARRAY_SIZE(mode_1632x1224_mipi_2lane),
			.regs = mode_1632x1224_mipi_2lane,
		},
		.reg_list_4lane = {
			.num_of_regs = ARRAY_SIZE(mode_1632x1224_mipi_4lane),
			.regs = mode_1632x1224_mipi_4lane,
		},
		.crop = {
			.left = 0x0,
			.top = 0x0,
			.width = 1632 * 2,
			.height = 1224 * 2,
		},
	},
	{
		.width = 3264,
		.height = 2448,
		.link_freq_index = FREQ_INDEX_1280,
		.fps = 30,
		.frame_len = 2526,
		.llp = HI846_LINE_LENGTH,
		.reg_list_config = {
			.num_of_regs = ARRAY_SIZE(mode_3264x2448_config),
			.regs = mode_3264x2448_config,
		},
		.reg_list_2lane = {
			.num_of_regs = 0,
		},
		.reg_list_4lane = {
			.num_of_regs = ARRAY_SIZE(mode_3264x2448_mipi_4lane),
			.regs = mode_3264x2448_mipi_4lane,
		},
		.crop = {
			.left = 0x0,
			.top = 0x0,
			.width = 3264,
			.height = 2448,
		},
	}
};

struct hi846_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

static const char * const hi846_supply_names[] = {
	"vddio", /* Digital I/O (1.8V or 2.8V) */
	"vdda", /* Analog (2.8V) */
	"vddd", /* Digital Core (1.2V) */
};

#define HI846_NUM_SUPPLIES ARRAY_SIZE(hi846_supply_names)

struct hi846 {
	struct gpio_desc *rst_gpio;
	struct gpio_desc *shutdown_gpio;
	struct regulator_bulk_data supplies[HI846_NUM_SUPPLIES];
	struct clk *clock;
	const struct hi846_datafmt *fmt;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	u8 nr_lanes;

	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	struct mutex mutex; /* protect cur_mode, streaming and chip access */
	const struct hi846_mode *cur_mode;
	bool powered;
	bool streaming;
};

static inline struct hi846 *to_hi846(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hi846, sd);
}

static const struct hi846_datafmt hi846_colour_fmts[] = {
	{ HI846_MEDIA_BUS_FORMAT, V4L2_COLORSPACE_RAW },
};

static const struct hi846_datafmt *hi846_find_datafmt(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hi846_colour_fmts); i++)
		if (hi846_colour_fmts[i].code == code)
			return &hi846_colour_fmts[i];

	return NULL;
}

static inline u8 hi846_get_link_freq_index(struct hi846 *hi846)
{
	return hi846->cur_mode->link_freq_index;
}

static u64 hi846_get_link_freq(struct hi846 *hi846)
{
	u8 index = hi846_get_link_freq_index(hi846);

	return hi846_link_freqs[index];
}

static u64 hi846_calc_pixel_rate(struct hi846 *hi846)
{
	u64 link_freq = hi846_get_link_freq(hi846);
	u64 pixel_rate = link_freq * 2 * hi846->nr_lanes;

	do_div(pixel_rate, HI846_RGB_DEPTH);

	return pixel_rate;
}

static int hi846_read_reg(struct hi846 *hi846, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[1] = {0};
	int ret;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "i2c read error: %d\n", ret);
		return -EIO;
	}

	*val = data_buf[0];

	return 0;
}

static int hi846_write_reg(struct hi846 *hi846, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg[] = {
		{ .addr = client->addr, .flags = 0,
		  .len = ARRAY_SIZE(buf), .buf = buf },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}

	return 0;
}

static void hi846_write_reg_16(struct hi846 *hi846, u16 reg, u16 val, int *err)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	u8 buf[4];
	int ret;

	if (*err < 0)
		return;

	put_unaligned_be16(reg, buf);
	put_unaligned_be16(val, buf + 2);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "i2c_master_send != %zu: %d\n",
			sizeof(buf), ret);
		*err = -EIO;
	}
}

static int hi846_write_reg_list(struct hi846 *hi846,
				const struct hi846_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < r_list->num_of_regs; i++) {
		hi846_write_reg_16(hi846, r_list->regs[i].address,
				   r_list->regs[i].val, &ret);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "failed to write reg 0x%4.4x: %d",
					    r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int hi846_update_digital_gain(struct hi846 *hi846, u16 d_gain)
{
	int ret = 0;

	hi846_write_reg_16(hi846, HI846_REG_MWB_GR_GAIN_H, d_gain, &ret);
	hi846_write_reg_16(hi846, HI846_REG_MWB_GB_GAIN_H, d_gain, &ret);
	hi846_write_reg_16(hi846, HI846_REG_MWB_R_GAIN_H, d_gain, &ret);
	hi846_write_reg_16(hi846, HI846_REG_MWB_B_GAIN_H, d_gain, &ret);

	return ret;
}

static int hi846_test_pattern(struct hi846 *hi846, u32 pattern)
{
	int ret;
	u8 val;

	if (pattern) {
		ret = hi846_read_reg(hi846, HI846_REG_ISP, &val);
		if (ret)
			return ret;

		ret = hi846_write_reg(hi846, HI846_REG_ISP,
				      val | HI846_REG_ISP_TPG_EN);
		if (ret)
			return ret;
	}

	return hi846_write_reg(hi846, HI846_REG_TEST_PATTERN, pattern);
}

static int hi846_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hi846 *hi846 = container_of(ctrl->handler,
					     struct hi846, ctrl_handler);
	s64 exposure_max;
	int ret = 0;
	u32 shutter, frame_len;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = hi846->cur_mode->height + ctrl->val -
			       HI846_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(hi846->exposure,
					 hi846->exposure->minimum,
					 exposure_max, hi846->exposure->step,
					 exposure_max);
	}

	if (!hi846->powered)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = hi846_write_reg(hi846, HI846_REG_ANALOG_GAIN, ctrl->val);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = hi846_update_digital_gain(hi846, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE:
		shutter = ctrl->val;
		frame_len = hi846->cur_mode->frame_len;

		if (shutter > frame_len - 6) { /* margin */
			frame_len = shutter + 6;
			if (frame_len > 0xffff) { /* max frame len */
				frame_len = 0xffff;
			}
		}

		if (shutter < 6)
			shutter = 6;
		if (shutter > (0xffff - 6))
			shutter = 0xffff - 6;

		hi846_write_reg_16(hi846, HI846_REG_FLL, frame_len, &ret);
		hi846_write_reg_16(hi846, HI846_REG_EXPOSURE, shutter, &ret);
		break;

	case V4L2_CID_VBLANK:
		/* Update FLL that meets expected vertical blanking */
		hi846_write_reg_16(hi846, HI846_REG_FLL,
				   hi846->cur_mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = hi846_test_pattern(hi846, ctrl->val);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops hi846_ctrl_ops = {
	.s_ctrl = hi846_set_ctrl,
};

static int hi846_init_controls(struct hi846 *hi846)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max, h_blank;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	struct v4l2_fwnode_device_properties props;

	ctrl_hdlr = &hi846->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &hi846->mutex;

	hi846->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &hi846_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(hi846_link_freqs) - 1,
				       0, hi846_link_freqs);
	if (hi846->link_freq)
		hi846->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hi846->pixel_rate =
		v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, 0,
				  hi846_calc_pixel_rate(hi846), 1,
				  hi846_calc_pixel_rate(hi846));
	hi846->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops,
					  V4L2_CID_VBLANK,
					  hi846->cur_mode->frame_len -
					  hi846->cur_mode->height,
					  HI846_FLL_MAX -
					  hi846->cur_mode->height, 1,
					  hi846->cur_mode->frame_len -
					  hi846->cur_mode->height);

	h_blank = hi846->cur_mode->llp - hi846->cur_mode->width;

	hi846->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops,
					  V4L2_CID_HBLANK, h_blank, h_blank, 1,
					  h_blank);
	if (hi846->hblank)
		hi846->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  HI846_ANAL_GAIN_MIN, HI846_ANAL_GAIN_MAX,
			  HI846_ANAL_GAIN_STEP, HI846_ANAL_GAIN_MIN);
	v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  HI846_DGTL_GAIN_MIN, HI846_DGTL_GAIN_MAX,
			  HI846_DGTL_GAIN_STEP, HI846_DGTL_GAIN_DEFAULT);
	exposure_max = hi846->cur_mode->frame_len - HI846_EXPOSURE_MAX_MARGIN;
	hi846->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops,
					    V4L2_CID_EXPOSURE,
					    HI846_EXPOSURE_MIN, exposure_max,
					    HI846_EXPOSURE_STEP,
					    exposure_max);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &hi846_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(hi846_test_pattern_menu) - 1,
				     0, 0, hi846_test_pattern_menu);
	if (ctrl_hdlr->error) {
		dev_err(&client->dev, "v4l ctrl handler error: %d\n",
			ctrl_hdlr->error);
		ret = ctrl_hdlr->error;
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &hi846_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	hi846->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

static int hi846_set_video_mode(struct hi846 *hi846, int fps)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	u64 frame_length;
	int ret = 0;
	int dummy_lines;
	u64 link_freq = hi846_get_link_freq(hi846);

	dev_dbg(&client->dev, "%s: link freq: %llu\n", __func__,
		hi846_get_link_freq(hi846));

	do_div(link_freq, fps);
	frame_length = link_freq;
	do_div(frame_length, HI846_LINE_LENGTH);

	dummy_lines = (frame_length > hi846->cur_mode->frame_len) ?
			(frame_length - hi846->cur_mode->frame_len) : 0;

	frame_length = hi846->cur_mode->frame_len + dummy_lines;

	dev_dbg(&client->dev, "%s: frame length calculated: %llu\n", __func__,
		frame_length);

	hi846_write_reg_16(hi846, HI846_REG_FLL, frame_length & 0xFFFF, &ret);
	hi846_write_reg_16(hi846, HI846_REG_LLP,
			   HI846_LINE_LENGTH & 0xFFFF, &ret);

	return ret;
}

static int hi846_start_streaming(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	int ret = 0;
	u8 val;

	if (hi846->nr_lanes == 2)
		ret = hi846_write_reg_list(hi846, &hi846_init_regs_list_2lane);
	else
		ret = hi846_write_reg_list(hi846, &hi846_init_regs_list_4lane);
	if (ret) {
		dev_err(&client->dev, "failed to set plls: %d\n", ret);
		return ret;
	}

	ret = hi846_write_reg_list(hi846, &hi846->cur_mode->reg_list_config);
	if (ret) {
		dev_err(&client->dev, "failed to set mode: %d\n", ret);
		return ret;
	}

	if (hi846->nr_lanes == 2)
		ret = hi846_write_reg_list(hi846,
					   &hi846->cur_mode->reg_list_2lane);
	else
		ret = hi846_write_reg_list(hi846,
					   &hi846->cur_mode->reg_list_4lane);
	if (ret) {
		dev_err(&client->dev, "failed to set mipi mode: %d\n", ret);
		return ret;
	}

	hi846_set_video_mode(hi846, hi846->cur_mode->fps);

	ret = __v4l2_ctrl_handler_setup(hi846->sd.ctrl_handler);
	if (ret)
		return ret;

	/*
	 * Reading 0x0034 is purely done for debugging reasons: It is not
	 * documented in the DS but only mentioned once:
	 * "If 0x0034[2] bit is disabled , Visible pixel width and height is 0."
	 * So even though that sounds like we won't see anything, we don't
	 * know more about this, so in that case only inform the user but do
	 * nothing more.
	 */
	ret = hi846_read_reg(hi846, 0x0034, &val);
	if (ret)
		return ret;
	if (!(val & BIT(2)))
		dev_info(&client->dev, "visible pixel width and height is 0\n");

	hi846_write_reg_16(hi846, HI846_REG_MODE_SELECT,
			   HI846_MODE_STREAMING, &ret);
	if (ret) {
		dev_err(&client->dev, "failed to start stream");
		return ret;
	}

	hi846->streaming = 1;

	dev_dbg(&client->dev, "%s: started streaming successfully\n", __func__);

	return ret;
}

static void hi846_stop_streaming(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	int ret = 0;

	hi846_write_reg_16(hi846, HI846_REG_MODE_SELECT,
			   HI846_MODE_STANDBY, &ret);
	if (ret)
		dev_err(&client->dev, "failed to stop stream");

	hi846->streaming = 0;
}

static int hi846_power_on(struct hi846 *hi846);
static int hi846_power_off(struct hi846 *hi846);

static int hi846_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct hi846 *hi846 = to_hi846(sd);
	int ret = 0;

	mutex_lock(&hi846->mutex);

	if (enable) {
		ret = hi846_power_on(hi846);
		if (ret)
			goto out;

		ret = hi846_start_streaming(hi846);
	}

	if (!enable || ret) {
		hi846_stop_streaming(hi846);
		hi846_power_off(hi846);
	}

out:
	mutex_unlock(&hi846->mutex);

	return ret;
}

static int hi846_power_on(struct hi846 *hi846)
{
	int ret;

	if (hi846->powered)
		return 0;

	ret = regulator_bulk_enable(HI846_NUM_SUPPLIES, hi846->supplies);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(hi846->clock);
	if (ret < 0)
		goto err_reg;

	if (hi846->shutdown_gpio)
		gpiod_set_value_cansleep(hi846->shutdown_gpio, 0);

	/* 30us = 2400 cycles at 80Mhz */
	usleep_range(30, 60);
	if (hi846->rst_gpio)
		gpiod_set_value_cansleep(hi846->rst_gpio, 0);
	usleep_range(30, 60);

	hi846->powered = true;

	return 0;

err_reg:
	regulator_bulk_disable(HI846_NUM_SUPPLIES, hi846->supplies);

	return ret;
}

static int hi846_power_off(struct hi846 *hi846)
{
	if (!hi846->powered)
		return 0;

	if (hi846->rst_gpio)
		gpiod_set_value_cansleep(hi846->rst_gpio, 1);

	if (hi846->shutdown_gpio)
		gpiod_set_value_cansleep(hi846->shutdown_gpio, 1);

	clk_disable_unprepare(hi846->clock);
	hi846->powered = false;

	return regulator_bulk_disable(HI846_NUM_SUPPLIES, hi846->supplies);
}

static int __maybe_unused hi846_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	return hi846_power_off(hi846);
}

static int __maybe_unused hi846_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	return hi846_power_on(hi846);
}

static int hi846_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *format)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct hi846_datafmt *fmt = hi846_find_datafmt(mf->code);
	const struct hi846_mode *mode;
	u32 tgt_fps;
	s32 vblank_def, h_blank;

	if (!fmt) {
		mf->code = hi846_colour_fmts[0].code;
		mf->colorspace = hi846_colour_fmts[0].colorspace;
		fmt = &hi846_colour_fmts[0];
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, format->pad) = *mf;
		return 0;
	}

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height, mf->width, mf->height);

	if (hi846->nr_lanes == 2) {
		if (!mode->reg_list_2lane.num_of_regs) {
			dev_err(&client->dev,
				"this mode is not supported for 2 lanes\n");
			return -EINVAL;
		}
	} else {
		if (!mode->reg_list_4lane.num_of_regs) {
			dev_err(&client->dev,
				"this mode is not supported for 4 lanes\n");
			return -EINVAL;
		}
	}

	mutex_lock(&hi846->mutex);

	if (hi846->streaming) {
		mutex_unlock(&hi846->mutex);
		return -EBUSY;
	}

	hi846->fmt = fmt;
	hi846->cur_mode = mode;
	dev_dbg(&client->dev, "%s: found mode: %dx%d\n", __func__,
		hi846->cur_mode->width, hi846->cur_mode->height);

	tgt_fps = hi846->cur_mode->fps;
	dev_dbg(&client->dev, "%s: target fps: %d\n", __func__, tgt_fps);

	mf->width = hi846->cur_mode->width;
	mf->height = hi846->cur_mode->height;
	mf->code = HI846_MEDIA_BUS_FORMAT;
	mf->field = V4L2_FIELD_NONE;

	__v4l2_ctrl_s_ctrl(hi846->link_freq, hi846_get_link_freq_index(hi846));
	__v4l2_ctrl_s_ctrl_int64(hi846->pixel_rate,
				 hi846_calc_pixel_rate(hi846));

	/* Update limits and set FPS to default */
	vblank_def = hi846->cur_mode->frame_len - hi846->cur_mode->height;
	__v4l2_ctrl_modify_range(hi846->vblank,
				 hi846->cur_mode->frame_len -
					hi846->cur_mode->height,
				 HI846_FLL_MAX - hi846->cur_mode->height, 1,
				 vblank_def);
	__v4l2_ctrl_s_ctrl(hi846->vblank, vblank_def);

	h_blank = hi846->cur_mode->llp - hi846->cur_mode->width;

	__v4l2_ctrl_modify_range(hi846->hblank, h_blank, h_blank, 1,
				 h_blank);

	dev_dbg(&client->dev, "Set fmt w=%d h=%d code=0x%x colorspace=0x%x\n",
		mf->width, mf->height,
		fmt->code, fmt->colorspace);

	mutex_unlock(&hi846->mutex);

	return 0;
}

static int hi846_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *format)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format = *v4l2_subdev_state_get_format(sd_state,
							       format->pad);
		return 0;
	}

	mutex_lock(&hi846->mutex);
	mf->code        = HI846_MEDIA_BUS_FORMAT;
	mf->colorspace  = V4L2_COLORSPACE_RAW;
	mf->field       = V4L2_FIELD_NONE;
	mf->width       = hi846->cur_mode->width;
	mf->height      = hi846->cur_mode->height;
	mutex_unlock(&hi846->mutex);
	dev_dbg(&client->dev,
		"Get format w=%d h=%d code=0x%x colorspace=0x%x\n",
		mf->width, mf->height, mf->code, mf->colorspace);

	return 0;
}

static int hi846_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = HI846_MEDIA_BUS_FORMAT;

	return 0;
}

static int hi846_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (fse->pad || fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != HI846_MEDIA_BUS_FORMAT) {
		dev_err(&client->dev, "frame size enum not matching\n");
		return -EINVAL;
	}

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].height;

	dev_dbg(&client->dev, "%s: max width: %d max height: %d\n", __func__,
		fse->max_width, fse->max_height);

	return 0;
}

static int hi846_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_selection *sel)
{
	struct hi846 *hi846 = to_hi846(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		mutex_lock(&hi846->mutex);
		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
			break;
		case V4L2_SUBDEV_FORMAT_ACTIVE:
			sel->r = hi846->cur_mode->crop;
			break;
		}
		mutex_unlock(&hi846->mutex);
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = 3264;
		sel->r.height = 2448;
		return 0;
	default:
		return -EINVAL;
	}
}

static int hi846_init_state(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *mf;

	mf = v4l2_subdev_state_get_format(sd_state, 0);

	mutex_lock(&hi846->mutex);
	mf->code        = HI846_MEDIA_BUS_FORMAT;
	mf->colorspace  = V4L2_COLORSPACE_RAW;
	mf->field       = V4L2_FIELD_NONE;
	mf->width       = hi846->cur_mode->width;
	mf->height      = hi846->cur_mode->height;
	mutex_unlock(&hi846->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops hi846_video_ops = {
	.s_stream = hi846_set_stream,
};

static const struct v4l2_subdev_pad_ops hi846_pad_ops = {
	.enum_frame_size = hi846_enum_frame_size,
	.enum_mbus_code = hi846_enum_mbus_code,
	.set_fmt = hi846_set_format,
	.get_fmt = hi846_get_format,
	.get_selection = hi846_get_selection,
};

static const struct v4l2_subdev_ops hi846_subdev_ops = {
	.video = &hi846_video_ops,
	.pad = &hi846_pad_ops,
};

static const struct v4l2_subdev_internal_ops hi846_internal_ops = {
	.init_state = hi846_init_state,
};

static const struct media_entity_operations hi846_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int hi846_identify_module(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	int ret;
	u8 hi, lo;

	ret = hi846_read_reg(hi846, HI846_REG_CHIP_ID_L, &lo);
	if (ret)
		return ret;

	if (lo != HI846_CHIP_ID_L) {
		dev_err(&client->dev, "wrong chip id low byte: %x", lo);
		return -ENXIO;
	}

	ret = hi846_read_reg(hi846, HI846_REG_CHIP_ID_H, &hi);
	if (ret)
		return ret;

	if (hi != HI846_CHIP_ID_H) {
		dev_err(&client->dev, "wrong chip id high byte: %x", hi);
		return -ENXIO;
	}

	dev_info(&client->dev, "chip id %02X %02X using %d mipi lanes\n",
		 hi, lo, hi846->nr_lanes);

	return 0;
}

static s64 hi846_check_link_freqs(struct hi846 *hi846,
				  struct v4l2_fwnode_endpoint *ep)
{
	const s64 *freqs = hi846_link_freqs;
	int freqs_count = ARRAY_SIZE(hi846_link_freqs);
	int i, j;

	for (i = 0; i < freqs_count; i++) {
		for (j = 0; j < ep->nr_of_link_frequencies; j++)
			if (freqs[i] == ep->link_frequencies[j])
				break;
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}

	return 0;
}

static int hi846_parse_dt(struct hi846 *hi846, struct device *dev)
{
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;
	s64 fq;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep) {
		dev_err(dev, "unable to find endpoint node\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(dev, "failed to parse endpoint node: %d\n", ret);
		return ret;
	}

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

	hi846->nr_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

	/* Check that link frequences for all the modes are in device tree */
	fq = hi846_check_link_freqs(hi846, &bus_cfg);
	if (fq) {
		dev_err(dev, "Link frequency of %lld is not supported\n", fq);
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

	v4l2_fwnode_endpoint_free(&bus_cfg);

	hi846->rst_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(hi846->rst_gpio)) {
		dev_err(dev, "failed to get reset gpio: %pe\n",
			hi846->rst_gpio);
		return PTR_ERR(hi846->rst_gpio);
	}

	hi846->shutdown_gpio = devm_gpiod_get_optional(dev, "shutdown",
						       GPIOD_OUT_LOW);
	if (IS_ERR(hi846->shutdown_gpio)) {
		dev_err(dev, "failed to get shutdown gpio: %pe\n",
			hi846->shutdown_gpio);
		return PTR_ERR(hi846->shutdown_gpio);
	}

	return 0;

check_hwcfg_error:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int hi846_probe(struct i2c_client *client)
{
	struct hi846 *hi846;
	int ret;
	int i;
	u32 mclk_freq;

	hi846 = devm_kzalloc(&client->dev, sizeof(*hi846), GFP_KERNEL);
	if (!hi846)
		return -ENOMEM;

	ret = hi846_parse_dt(hi846, &client->dev);
	if (ret) {
		dev_err(&client->dev, "failed to check HW configuration: %d",
			ret);
		return ret;
	}

	hi846->clock = devm_v4l2_sensor_clk_get(&client->dev, NULL);
	if (IS_ERR(hi846->clock))
		return dev_err_probe(&client->dev, PTR_ERR(hi846->clock),
				     "failed to get clock: %pe\n",
				     hi846->clock);

	mclk_freq = clk_get_rate(hi846->clock);
	if (mclk_freq != 24000000 && mclk_freq != 25000000)
		dev_warn(&client->dev,
			 "External clock freq should be 24000000 or 25000000, not %u.\n",
			 mclk_freq);

	for (i = 0; i < HI846_NUM_SUPPLIES; i++)
		hi846->supplies[i].supply = hi846_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev, HI846_NUM_SUPPLIES,
				      hi846->supplies);
	if (ret < 0)
		return ret;

	v4l2_i2c_subdev_init(&hi846->sd, client, &hi846_subdev_ops);
	hi846->sd.internal_ops = &hi846_internal_ops;

	mutex_init(&hi846->mutex);

	ret = hi846_power_on(hi846);
	if (ret)
		goto err_mutex;

	ret = hi846_identify_module(hi846);
	if (ret)
		goto err_power_off;

	hi846->cur_mode = &supported_modes[0];

	ret = hi846_init_controls(hi846);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto err_power_off;
	}

	hi846->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	hi846->sd.entity.ops = &hi846_subdev_entity_ops;
	hi846->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	hi846->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&hi846->sd.entity, 1, &hi846->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto err_v4l2_ctrl_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&hi846->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto err_media_entity_cleanup;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_media_entity_cleanup:
	media_entity_cleanup(&hi846->sd.entity);

err_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(hi846->sd.ctrl_handler);

err_power_off:
	hi846_power_off(hi846);

err_mutex:
	mutex_destroy(&hi846->mutex);

	return ret;
}

static void hi846_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		hi846_suspend(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&hi846->mutex);
}

static const struct dev_pm_ops hi846_pm_ops = {
	SET_RUNTIME_PM_OPS(hi846_suspend, hi846_resume, NULL)
};

static const struct of_device_id hi846_of_match[] = {
	{ .compatible = "hynix,hi846", },
	{},
};
MODULE_DEVICE_TABLE(of, hi846_of_match);

static struct i2c_driver hi846_i2c_driver = {
	.driver = {
		.name = "hi846",
		.pm = &hi846_pm_ops,
		.of_match_table = hi846_of_match,
	},
	.probe = hi846_probe,
	.remove = hi846_remove,
};

module_i2c_driver(hi846_i2c_driver);

MODULE_AUTHOR("Angus Ainslie <angus@akkea.ca>");
MODULE_AUTHOR("Martin Kepplinger <martin.kepplinger@puri.sm>");
MODULE_DESCRIPTION("Hynix HI846 sensor driver");
MODULE_LICENSE("GPL v2");
