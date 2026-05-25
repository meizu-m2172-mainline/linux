// SPDX-License-Identifier: GPL-2.0-only
/*
 * SK Hynix Hi-1336 camera sensor driver
 *
 * Register tables are derived from the Meizu M2172 Android vendor camera
 * module com.qti.sensormodule.hi1336_front_txd.bin.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define HI1336_LINK_FREQ_360MHZ		(360ULL * HZ_PER_MHZ)
#define HI1336_LINK_FREQ_720MHZ		(720ULL * HZ_PER_MHZ)
#define HI1336_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define HI1336_DATA_LANES		4
#define HI1336_NATIVE_WIDTH		4208
#define HI1336_NATIVE_HEIGHT		3120
#define HI1336_DEFAULT_MODE		2
#define HI1336_EXPOSURE_MIN		4
#define HI1336_FRAME_LENGTH_MAX		0xffff
#define HI1336_DIGITAL_GAIN_MIN		0x0100
#define HI1336_DIGITAL_GAIN_MAX		0x0800
#define HI1336_DIGITAL_GAIN_DEFAULT	0x0200

#define HI1336_REG_CHIP_ID		CCI_REG16(0x0716)
#define HI1336_CHIP_ID			0x1336
#define HI1336_REG_STREAM		CCI_REG16(0x0b00)
#define HI1336_STREAM_ON		0x0100
#define HI1336_STREAM_OFF		0x0000
#define HI1336_REG_GROUP_HOLD		CCI_REG16(0x0208)
#define HI1336_GROUP_HOLD_ON		0x0102
#define HI1336_GROUP_HOLD_OFF		0x0002
#define HI1336_REG_EXPOSURE		CCI_REG16(0x020a)
#define HI1336_REG_FRAME_LENGTH		CCI_REG16(0x020e)
#define HI1336_REG_DIGITAL_GAIN_GR	CCI_REG16(0x0214)
#define HI1336_REG_DIGITAL_GAIN_GB	CCI_REG16(0x0216)
#define HI1336_REG_DIGITAL_GAIN_R	CCI_REG16(0x0218)
#define HI1336_REG_DIGITAL_GAIN_B	CCI_REG16(0x021a)
#define HI1336_REG_TEST_PATTERN_EN	CCI_REG16(0x0b04)
#define HI1336_REG_TEST_PATTERN		CCI_REG16(0x0c0a)

#define to_hi1336(_sd)			container_of(_sd, struct hi1336, sd)

enum hi1336_link_freq_index {
	HI1336_LINK_FREQ_360MHZ_INDEX,
	HI1336_LINK_FREQ_720MHZ_INDEX,
};

static const s64 hi1336_link_freq_menu[] = {
	HI1336_LINK_FREQ_360MHZ,
	HI1336_LINK_FREQ_720MHZ,
};

static const char * const hi1336_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Fade to gray color bars",
	"PN9",
};

struct hi1336_reg {
	u16 address;
	u16 val;
};

struct hi1336_reg_list {
	const struct hi1336_reg *regs;
	unsigned int num_regs;
};

struct hi1336_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 fps_milli;
	u32 exposure_margin;
	u32 exposure_default;
	u32 link_freq_index;
	u64 pixel_rate;
	struct v4l2_rect crop;
	struct hi1336_reg_list reg_list;
};

static const char * const hi1336_supply_names[] = {
	"vddio",
	"vdda",
	"vddd",
};

struct hi1336 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(hi1336_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *digital_gain;

	const struct hi1336_mode *mode;
};

static const struct hi1336_reg hi1336_init_regs[] = {
	{ 0x2000, 0x0021 },
	{ 0x2002, 0x04a5 },
	{ 0x2004, 0xb124 },
	{ 0x2006, 0xc09c },
	{ 0x2008, 0x0064 },
	{ 0x200a, 0x088e },
	{ 0x200c, 0x01c2 },
	{ 0x200e, 0x00b4 },
	{ 0x2010, 0x4020 },
	{ 0x2012, 0x4292 },
	{ 0x2014, 0xf00a },
	{ 0x2016, 0x0310 },
	{ 0x2018, 0x12b0 },
	{ 0x201a, 0xc3f2 },
	{ 0x201c, 0x425f },
	{ 0x201e, 0x0282 },
	{ 0x2020, 0xf35f },
	{ 0x2022, 0xf37f },
	{ 0x2024, 0x5f0f },
	{ 0x2026, 0x4f92 },
	{ 0x2028, 0xf692 },
	{ 0x202a, 0x0402 },
	{ 0x202c, 0x93c2 },
	{ 0x202e, 0x82cc },
	{ 0x2030, 0x2403 },
	{ 0x2032, 0xf0f2 },
	{ 0x2034, 0xffe7 },
	{ 0x2036, 0x0254 },
	{ 0x2038, 0x4130 },
	{ 0x203a, 0x120b },
	{ 0x203c, 0x120a },
	{ 0x203e, 0x1209 },
	{ 0x2040, 0x425f },
	{ 0x2042, 0x0600 },
	{ 0x2044, 0xf35f },
	{ 0x2046, 0x4f4b },
	{ 0x2048, 0x12b0 },
	{ 0x204a, 0xc6ae },
	{ 0x204c, 0x403d },
	{ 0x204e, 0x0100 },
	{ 0x2050, 0x403e },
	{ 0x2052, 0x2bfc },
	{ 0x2054, 0x403f },
	{ 0x2056, 0x8020 },
	{ 0x2058, 0x12b0 },
	{ 0x205a, 0xc476 },
	{ 0x205c, 0x930b },
	{ 0x205e, 0x2009 },
	{ 0x2060, 0x93c2 },
	{ 0x2062, 0x0c0a },
	{ 0x2064, 0x2403 },
	{ 0x2066, 0x43d2 },
	{ 0x2068, 0x0e1f },
	{ 0x206a, 0x3c13 },
	{ 0x206c, 0x43c2 },
	{ 0x206e, 0x0e1f },
	{ 0x2070, 0x3c10 },
	{ 0x2072, 0x4039 },
	{ 0x2074, 0x0e08 },
	{ 0x2076, 0x492a },
	{ 0x2078, 0x421c },
	{ 0x207a, 0xf010 },
	{ 0x207c, 0x430b },
	{ 0x207e, 0x430d },
	{ 0x2080, 0x12b0 },
	{ 0x2082, 0xdfb6 },
	{ 0x2084, 0x403d },
	{ 0x2086, 0x000e },
	{ 0x2088, 0x12b0 },
	{ 0x208a, 0xc62c },
	{ 0x208c, 0x4e89 },
	{ 0x208e, 0x0000 },
	{ 0x2090, 0x3fe7 },
	{ 0x2092, 0x4139 },
	{ 0x2094, 0x413a },
	{ 0x2096, 0x413b },
	{ 0x2098, 0x4130 },
	{ 0x209a, 0xb0b2 },
	{ 0x209c, 0x0020 },
	{ 0x209e, 0xf002 },
	{ 0x20a0, 0x2429 },
	{ 0x20a2, 0x421e },
	{ 0x20a4, 0x0256 },
	{ 0x20a6, 0x532e },
	{ 0x20a8, 0x421f },
	{ 0x20aa, 0xf008 },
	{ 0x20ac, 0x9e0f },
	{ 0x20ae, 0x2c01 },
	{ 0x20b0, 0x4e0f },
	{ 0x20b2, 0x4f0c },
	{ 0x20b4, 0x430d },
	{ 0x20b6, 0x421e },
	{ 0x20b8, 0x7300 },
	{ 0x20ba, 0x421f },
	{ 0x20bc, 0x7302 },
	{ 0x20be, 0x5c0e },
	{ 0x20c0, 0x6d0f },
	{ 0x20c2, 0x821e },
	{ 0x20c4, 0x830c },
	{ 0x20c6, 0x721f },
	{ 0x20c8, 0x830e },
	{ 0x20ca, 0x2c0d },
	{ 0x20cc, 0x0900 },
	{ 0x20ce, 0x7312 },
	{ 0x20d0, 0x421e },
	{ 0x20d2, 0x7300 },
	{ 0x20d4, 0x421f },
	{ 0x20d6, 0x7302 },
	{ 0x20d8, 0x5c0e },
	{ 0x20da, 0x6d0f },
	{ 0x20dc, 0x821e },
	{ 0x20de, 0x830c },
	{ 0x20e0, 0x721f },
	{ 0x20e2, 0x830e },
	{ 0x20e4, 0x2bf3 },
	{ 0x20e6, 0x4292 },
	{ 0x20e8, 0x8248 },
	{ 0x20ea, 0x0a08 },
	{ 0x20ec, 0x0c10 },
	{ 0x20ee, 0x4292 },
	{ 0x20f0, 0x8252 },
	{ 0x20f2, 0x0a12 },
	{ 0x20f4, 0x12b0 },
	{ 0x20f6, 0xdc9c },
	{ 0x20f8, 0xd0f2 },
	{ 0x20fa, 0x0018 },
	{ 0x20fc, 0x0254 },
	{ 0x20fe, 0x4130 },
	{ 0x2100, 0x120b },
	{ 0x2102, 0x12b0 },
	{ 0x2104, 0xcfc8 },
	{ 0x2106, 0x4f4b },
	{ 0x2108, 0x12b0 },
	{ 0x210a, 0xcfc8 },
	{ 0x210c, 0xf37f },
	{ 0x210e, 0x108f },
	{ 0x2110, 0xdb0f },
	{ 0x2112, 0x413b },
	{ 0x2114, 0x4130 },
	{ 0x2116, 0x120b },
	{ 0x2118, 0x12b0 },
	{ 0x211a, 0xcfc8 },
	{ 0x211c, 0x4f4b },
	{ 0x211e, 0x108b },
	{ 0x2120, 0x12b0 },
	{ 0x2122, 0xcfc8 },
	{ 0x2124, 0xf37f },
	{ 0x2126, 0xdb0f },
	{ 0x2128, 0x413b },
	{ 0x212a, 0x4130 },
	{ 0x212c, 0x120b },
	{ 0x212e, 0x120a },
	{ 0x2130, 0x1209 },
	{ 0x2132, 0x1208 },
	{ 0x2134, 0x4338 },
	{ 0x2136, 0x40b2 },
	{ 0x2138, 0x17fb },
	{ 0x213a, 0x83be },
	{ 0x213c, 0x12b0 },
	{ 0x213e, 0xcfc8 },
	{ 0x2140, 0xf37f },
	{ 0x2142, 0x903f },
	{ 0x2144, 0x0013 },
	{ 0x2146, 0x244c },
	{ 0x2148, 0x12b0 },
	{ 0x214a, 0xf100 },
	{ 0x214c, 0x4f82 },
	{ 0x214e, 0x82a4 },
	{ 0x2150, 0xb3e2 },
	{ 0x2152, 0x0282 },
	{ 0x2154, 0x240a },
	{ 0x2156, 0x5f0f },
	{ 0x2158, 0x5f0f },
	{ 0x215a, 0x521f },
	{ 0x215c, 0x83be },
	{ 0x215e, 0x533f },
	{ 0x2160, 0x4f82 },
	{ 0x2162, 0x83be },
	{ 0x2164, 0x43f2 },
	{ 0x2166, 0x83c0 },
	{ 0x2168, 0x4308 },
	{ 0x216a, 0x4309 },
	{ 0x216c, 0x9219 },
	{ 0x216e, 0x82a4 },
	{ 0x2170, 0x2c34 },
	{ 0x2172, 0xb3e2 },
	{ 0x2174, 0x0282 },
	{ 0x2176, 0x242a },
	{ 0x2178, 0x12b0 },
	{ 0x217a, 0xf116 },
	{ 0x217c, 0x4f0b },
	{ 0x217e, 0x12b0 },
	{ 0x2180, 0xf116 },
	{ 0x2182, 0x4f0a },
	{ 0x2184, 0x490f },
	{ 0x2186, 0x5f0f },
	{ 0x2188, 0x5f0f },
	{ 0x218a, 0x4b8f },
	{ 0x218c, 0x2bfc },
	{ 0x218e, 0x4a8f },
	{ 0x2190, 0x2bfe },
	{ 0x2192, 0x5319 },
	{ 0x2194, 0x9039 },
	{ 0x2196, 0x0100 },
	{ 0x2198, 0x2be9 },
	{ 0x219a, 0x43d2 },
	{ 0x219c, 0x83c0 },
	{ 0x219e, 0x421e },
	{ 0x21a0, 0x82a4 },
	{ 0x21a2, 0x903e },
	{ 0x21a4, 0x0080 },
	{ 0x21a6, 0x2810 },
	{ 0x21a8, 0x421f },
	{ 0x21aa, 0x2d28 },
	{ 0x21ac, 0x503f },
	{ 0x21ae, 0x0014 },
	{ 0x21b0, 0x4f82 },
	{ 0x21b2, 0x82a0 },
	{ 0x21b4, 0x903e },
	{ 0x21b6, 0x00c0 },
	{ 0x21b8, 0x2805 },
	{ 0x21ba, 0x421f },
	{ 0x21bc, 0x2e28 },
	{ 0x21be, 0x503f },
	{ 0x21c0, 0x0014 },
	{ 0x21c2, 0x3c12 },
	{ 0x21c4, 0x480f },
	{ 0x21c6, 0x3c10 },
	{ 0x21c8, 0x480f },
	{ 0x21ca, 0x3ff2 },
	{ 0x21cc, 0x12b0 },
	{ 0x21ce, 0xf100 },
	{ 0x21d0, 0x4f0a },
	{ 0x21d2, 0x12b0 },
	{ 0x21d4, 0xf100 },
	{ 0x21d6, 0x4f0b },
	{ 0x21d8, 0x3fd5 },
	{ 0x21da, 0x430a },
	{ 0x21dc, 0x430b },
	{ 0x21de, 0x3fd2 },
	{ 0x21e0, 0x40b2 },
	{ 0x21e2, 0x1bfe },
	{ 0x21e4, 0x83be },
	{ 0x21e6, 0x3fb0 },
	{ 0x21e8, 0x4f82 },
	{ 0x21ea, 0x82a2 },
	{ 0x21ec, 0x4138 },
	{ 0x21ee, 0x4139 },
	{ 0x21f0, 0x413a },
	{ 0x21f2, 0x413b },
	{ 0x21f4, 0x4130 },
	{ 0x21f6, 0x43d2 },
	{ 0x21f8, 0x0300 },
	{ 0x21fa, 0x12b0 },
	{ 0x21fc, 0xcf6a },
	{ 0x21fe, 0x12b0 },
	{ 0x2200, 0xcf0a },
	{ 0x2202, 0xb3d2 },
	{ 0x2204, 0x0267 },
	{ 0x2206, 0x2404 },
	{ 0x2208, 0x12b0 },
	{ 0x220a, 0xf12c },
	{ 0x220c, 0xc3d2 },
	{ 0x220e, 0x0267 },
	{ 0x2210, 0x12b0 },
	{ 0x2212, 0xd0d4 },
	{ 0x2214, 0x0261 },
	{ 0x2216, 0x0000 },
	{ 0x2218, 0x43c2 },
	{ 0x221a, 0x0300 },
	{ 0x221c, 0x4392 },
	{ 0x221e, 0x732a },
	{ 0x2220, 0x4130 },
	{ 0x2222, 0x90f2 },
	{ 0x2224, 0x0010 },
	{ 0x2226, 0x0260 },
	{ 0x2228, 0x2002 },
	{ 0x222a, 0x12b0 },
	{ 0x222c, 0xd4aa },
	{ 0x222e, 0x12b0 },
	{ 0x2230, 0xd5fa },
	{ 0x2232, 0x4392 },
	{ 0x2234, 0x732a },
	{ 0x2236, 0x12b0 },
	{ 0x2238, 0xf1f6 },
	{ 0x223a, 0x4130 },
	{ 0x223c, 0x120b },
	{ 0x223e, 0x120a },
	{ 0x2240, 0x1209 },
	{ 0x2242, 0x1208 },
	{ 0x2244, 0x1207 },
	{ 0x2246, 0x1206 },
	{ 0x2248, 0x1205 },
	{ 0x224a, 0x1204 },
	{ 0x224c, 0x8031 },
	{ 0x224e, 0x000a },
	{ 0x2250, 0x4291 },
	{ 0x2252, 0x82d8 },
	{ 0x2254, 0x0004 },
	{ 0x2256, 0x411f },
	{ 0x2258, 0x0004 },
	{ 0x225a, 0x4fa1 },
	{ 0x225c, 0x0006 },
	{ 0x225e, 0x4257 },
	{ 0x2260, 0x82e5 },
	{ 0x2262, 0x4708 },
	{ 0x2264, 0xd038 },
	{ 0x2266, 0xff00 },
	{ 0x2268, 0x4349 },
	{ 0x226a, 0x4346 },
	{ 0x226c, 0x90b2 },
	{ 0x226e, 0x07d1 },
	{ 0x2270, 0x0b94 },
	{ 0x2272, 0x2806 },
	{ 0x2274, 0x40b2 },
	{ 0x2276, 0x0246 },
	{ 0x2278, 0x0228 },
	{ 0x227a, 0x40b2 },
	{ 0x227c, 0x09fb },
	{ 0x227e, 0x0232 },
	{ 0x2280, 0x4291 },
	{ 0x2282, 0x0422 },
	{ 0x2284, 0x0000 },
	{ 0x2286, 0x421f },
	{ 0x2288, 0x0424 },
	{ 0x228a, 0x812f },
	{ 0x228c, 0x4f81 },
	{ 0x228e, 0x0002 },
	{ 0x2290, 0x4291 },
	{ 0x2292, 0x8248 },
	{ 0x2294, 0x0008 },
	{ 0x2296, 0x4214 },
	{ 0x2298, 0x0310 },
	{ 0x229a, 0x421a },
	{ 0x229c, 0x82a0 },
	{ 0x229e, 0xf80a },
	{ 0x22a0, 0x421b },
	{ 0x22a2, 0x82a2 },
	{ 0x22a4, 0xf80b },
	{ 0x22a6, 0x4382 },
	{ 0x22a8, 0x7334 },
	{ 0x22aa, 0x0f00 },
	{ 0x22ac, 0x7304 },
	{ 0x22ae, 0x4192 },
	{ 0x22b0, 0x0008 },
	{ 0x22b2, 0x0a08 },
	{ 0x22b4, 0x4382 },
	{ 0x22b6, 0x040c },
	{ 0x22b8, 0x4305 },
	{ 0x22ba, 0x9382 },
	{ 0x22bc, 0x7112 },
	{ 0x22be, 0x2001 },
	{ 0x22c0, 0x4315 },
	{ 0x22c2, 0x421e },
	{ 0x22c4, 0x7100 },
	{ 0x22c6, 0xb2f2 },
	{ 0x22c8, 0x0261 },
	{ 0x22ca, 0x2406 },
	{ 0x22cc, 0xb3d2 },
	{ 0x22ce, 0x0b02 },
	{ 0x22d0, 0x2403 },
	{ 0x22d2, 0x42d2 },
	{ 0x22d4, 0x0809 },
	{ 0x22d6, 0x0b00 },
	{ 0x22d8, 0x40b2 },
	{ 0x22da, 0x00b6 },
	{ 0x22dc, 0x7334 },
	{ 0x22de, 0x0f00 },
	{ 0x22e0, 0x7304 },
	{ 0x22e2, 0x4482 },
	{ 0x22e4, 0x0a08 },
	{ 0x22e6, 0xb2e2 },
	{ 0x22e8, 0x0b05 },
	{ 0x22ea, 0x2404 },
	{ 0x22ec, 0x4392 },
	{ 0x22ee, 0x7a0e },
	{ 0x22f0, 0x0800 },
	{ 0x22f2, 0x7a10 },
	{ 0x22f4, 0xf80e },
	{ 0x22f6, 0x93c2 },
	{ 0x22f8, 0x82de },
	{ 0x22fa, 0x2468 },
	{ 0x22fc, 0x9e0a },
	{ 0x22fe, 0x2803 },
	{ 0x2300, 0x9349 },
	{ 0x2302, 0x2001 },
	{ 0x2304, 0x4359 },
	{ 0x2306, 0x9e0b },
	{ 0x2308, 0x2802 },
	{ 0x230a, 0x9369 },
	{ 0x230c, 0x245c },
	{ 0x230e, 0x421f },
	{ 0x2310, 0x731a },
	{ 0x2312, 0xc312 },
	{ 0x2314, 0x100f },
	{ 0x2316, 0x4f82 },
	{ 0x2318, 0x7334 },
	{ 0x231a, 0x0f00 },
	{ 0x231c, 0x7304 },
	{ 0x231e, 0x4192 },
	{ 0x2320, 0x0008 },
	{ 0x2322, 0x0a08 },
	{ 0x2324, 0x421e },
	{ 0x2326, 0x7100 },
	{ 0x2328, 0x812e },
	{ 0x232a, 0x425c },
	{ 0x232c, 0x0419 },
	{ 0x232e, 0x537c },
	{ 0x2330, 0xfe4c },
	{ 0x2332, 0x9305 },
	{ 0x2334, 0x2003 },
	{ 0x2336, 0x40b2 },
	{ 0x2338, 0x0c78 },
	{ 0x233a, 0x7100 },
	{ 0x233c, 0x421f },
	{ 0x233e, 0x731a },
	{ 0x2340, 0xc312 },
	{ 0x2342, 0x100f },
	{ 0x2344, 0x503f },
	{ 0x2346, 0x00b6 },
	{ 0x2348, 0x4f82 },
	{ 0x234a, 0x7334 },
	{ 0x234c, 0x0f00 },
	{ 0x234e, 0x7304 },
	{ 0x2350, 0x4482 },
	{ 0x2352, 0x0a08 },
	{ 0x2354, 0x9e81 },
	{ 0x2356, 0x0002 },
	{ 0x2358, 0x2814 },
	{ 0x235a, 0xf74c },
	{ 0x235c, 0x434d },
	{ 0x235e, 0x411f },
	{ 0x2360, 0x0004 },
	{ 0x2362, 0x4f1e },
	{ 0x2364, 0x0002 },
	{ 0x2366, 0x9381 },
	{ 0x2368, 0x0006 },
	{ 0x236a, 0x240b },
	{ 0x236c, 0x4e6f },
	{ 0x236e, 0xf74f },
	{ 0x2370, 0x9c4f },
	{ 0x2372, 0x2423 },
	{ 0x2374, 0x535d },
	{ 0x2376, 0x503e },
	{ 0x2378, 0x0006 },
	{ 0x237a, 0x4d4f },
	{ 0x237c, 0x911f },
	{ 0x237e, 0x0006 },
	{ 0x2380, 0x2bf5 },
	{ 0x2382, 0x9359 },
	{ 0x2384, 0x2403 },
	{ 0x2386, 0x9079 },
	{ 0x2388, 0x0003 },
	{ 0x238a, 0x2028 },
	{ 0x238c, 0x434d },
	{ 0x238e, 0x464f },
	{ 0x2390, 0x5f0f },
	{ 0x2392, 0x5f0f },
	{ 0x2394, 0x4f9f },
	{ 0x2396, 0x2dfc },
	{ 0x2398, 0x8020 },
	{ 0x239a, 0x4f9f },
	{ 0x239c, 0x2dfe },
	{ 0x239e, 0x8022 },
	{ 0x23a0, 0x5356 },
	{ 0x23a2, 0x9076 },
	{ 0x23a4, 0x0040 },
	{ 0x23a6, 0x2407 },
	{ 0x23a8, 0x9076 },
	{ 0x23aa, 0xff80 },
	{ 0x23ac, 0x2404 },
	{ 0x23ae, 0x535d },
	{ 0x23b0, 0x926d },
	{ 0x23b2, 0x2bed },
	{ 0x23b4, 0x3c13 },
	{ 0x23b6, 0x5359 },
	{ 0x23b8, 0x3c11 },
	{ 0x23ba, 0x4ea2 },
	{ 0x23bc, 0x040c },
	{ 0x23be, 0x4e92 },
	{ 0x23c0, 0x0002 },
	{ 0x23c2, 0x040e },
	{ 0x23c4, 0x3fde },
	{ 0x23c6, 0x4079 },
	{ 0x23c8, 0x0003 },
	{ 0x23ca, 0x3fa1 },
	{ 0x23cc, 0x9a0e },
	{ 0x23ce, 0x2803 },
	{ 0x23d0, 0x9349 },
	{ 0x23d2, 0x2001 },
	{ 0x23d4, 0x4359 },
	{ 0x23d6, 0x9b0e },
	{ 0x23d8, 0x2b9a },
	{ 0x23da, 0x3f97 },
	{ 0x23dc, 0x9305 },
	{ 0x23de, 0x2363 },
	{ 0x23e0, 0x5031 },
	{ 0x23e2, 0x000a },
	{ 0x23e4, 0x4134 },
	{ 0x23e6, 0x4135 },
	{ 0x23e8, 0x4136 },
	{ 0x23ea, 0x4137 },
	{ 0x23ec, 0x4138 },
	{ 0x23ee, 0x4139 },
	{ 0x23f0, 0x413a },
	{ 0x23f2, 0x413b },
	{ 0x23f4, 0x4130 },
	{ 0x23f6, 0x120b },
	{ 0x23f8, 0x120a },
	{ 0x23fa, 0x1209 },
	{ 0x23fc, 0x1208 },
	{ 0x23fe, 0x1207 },
	{ 0x2400, 0x1206 },
	{ 0x2402, 0x1205 },
	{ 0x2404, 0x1204 },
	{ 0x2406, 0x8221 },
	{ 0x2408, 0x425f },
	{ 0x240a, 0x0600 },
	{ 0x240c, 0xf35f },
	{ 0x240e, 0x4fc1 },
	{ 0x2410, 0x0002 },
	{ 0x2412, 0x43c1 },
	{ 0x2414, 0x0003 },
	{ 0x2416, 0x403f },
	{ 0x2418, 0x0603 },
	{ 0x241a, 0x4fe1 },
	{ 0x241c, 0x0000 },
	{ 0x241e, 0xb3ef },
	{ 0x2420, 0x0000 },
	{ 0x2422, 0x2431 },
	{ 0x2424, 0x4344 },
	{ 0x2426, 0x4445 },
	{ 0x2428, 0x450f },
	{ 0x242a, 0x5f0f },
	{ 0x242c, 0x5f0f },
	{ 0x242e, 0x403d },
	{ 0x2430, 0x000e },
	{ 0x2432, 0x4f1e },
	{ 0x2434, 0x0632 },
	{ 0x2436, 0x4f1f },
	{ 0x2438, 0x0634 },
	{ 0x243a, 0x12b0 },
	{ 0x243c, 0xc62c },
	{ 0x243e, 0x4e08 },
	{ 0x2440, 0x4f09 },
	{ 0x2442, 0x421e },
	{ 0x2444, 0xf00c },
	{ 0x2446, 0x430f },
	{ 0x2448, 0x480a },
	{ 0x244a, 0x490b },
	{ 0x244c, 0x4e0c },
	{ 0x244e, 0x4f0d },
	{ 0x2450, 0x12b0 },
	{ 0x2452, 0xdf96 },
	{ 0x2454, 0x421a },
	{ 0x2456, 0xf00e },
	{ 0x2458, 0x430b },
	{ 0x245a, 0x403d },
	{ 0x245c, 0x0009 },
	{ 0x245e, 0x12b0 },
	{ 0x2460, 0xc62c },
	{ 0x2462, 0x4e06 },
	{ 0x2464, 0x4f07 },
	{ 0x2466, 0x5a06 },
	{ 0x2468, 0x6b07 },
	{ 0x246a, 0x425f },
	{ 0x246c, 0x0668 },
	{ 0x246e, 0xf37f },
	{ 0x2470, 0x9f08 },
	{ 0x2472, 0x2c6b },
	{ 0x2474, 0x4216 },
	{ 0x2476, 0x06ca },
	{ 0x2478, 0x4307 },
	{ 0x247a, 0x5505 },
	{ 0x247c, 0x4685 },
	{ 0x247e, 0x065e },
	{ 0x2480, 0x5354 },
	{ 0x2482, 0x9264 },
	{ 0x2484, 0x2bd0 },
	{ 0x2486, 0x403b },
	{ 0x2488, 0x0603 },
	{ 0x248a, 0x416f },
	{ 0x248c, 0xc36f },
	{ 0x248e, 0x4fcb },
	{ 0x2490, 0x0000 },
	{ 0x2492, 0x12b0 },
	{ 0x2494, 0xcd42 },
	{ 0x2496, 0x41eb },
	{ 0x2498, 0x0000 },
	{ 0x249a, 0x421f },
	{ 0x249c, 0x0256 },
	{ 0x249e, 0x522f },
	{ 0x24a0, 0x421b },
	{ 0x24a2, 0xf008 },
	{ 0x24a4, 0x532b },
	{ 0x24a6, 0x9f0b },
	{ 0x24a8, 0x2c01 },
	{ 0x24aa, 0x4f0b },
	{ 0x24ac, 0x9381 },
	{ 0x24ae, 0x0002 },
	{ 0x24b0, 0x2409 },
	{ 0x24b2, 0x430a },
	{ 0x24b4, 0x421e },
	{ 0x24b6, 0x0614 },
	{ 0x24b8, 0x503e },
	{ 0x24ba, 0x000a },
	{ 0x24bc, 0x421f },
	{ 0x24be, 0x0680 },
	{ 0x24c0, 0x9f0e },
	{ 0x24c2, 0x2801 },
	{ 0x24c4, 0x431a },
	{ 0x24c6, 0xb0b2 },
	{ 0x24c8, 0x0020 },
	{ 0x24ca, 0xf002 },
	{ 0x24cc, 0x241f },
	{ 0x24ce, 0x93c2 },
	{ 0x24d0, 0x82cc },
	{ 0x24d2, 0x201c },
	{ 0x24d4, 0x4b0e },
	{ 0x24d6, 0x430f },
	{ 0x24d8, 0x521e },
	{ 0x24da, 0x7300 },
	{ 0x24dc, 0x621f },
	{ 0x24de, 0x7302 },
	{ 0x24e0, 0x421c },
	{ 0x24e2, 0x7316 },
	{ 0x24e4, 0x421d },
	{ 0x24e6, 0x7318 },
	{ 0x24e8, 0x8c0e },
	{ 0x24ea, 0x7d0f },
	{ 0x24ec, 0x2c0f },
	{ 0x24ee, 0x930a },
	{ 0x24f0, 0x240d },
	{ 0x24f2, 0x421f },
	{ 0x24f4, 0x8248 },
	{ 0x24f6, 0xf03f },
	{ 0x24f8, 0xf7ff },
	{ 0x24fa, 0x4f82 },
	{ 0x24fc, 0x0a08 },
	{ 0x24fe, 0x0c10 },
	{ 0x2500, 0x421f },
	{ 0x2502, 0x8252 },
	{ 0x2504, 0xd03f },
	{ 0x2506, 0x00c0 },
	{ 0x2508, 0x4f82 },
	{ 0x250a, 0x0a12 },
	{ 0x250c, 0x4b0a },
	{ 0x250e, 0x430b },
	{ 0x2510, 0x421e },
	{ 0x2512, 0x7300 },
	{ 0x2514, 0x421f },
	{ 0x2516, 0x7302 },
	{ 0x2518, 0x5a0e },
	{ 0x251a, 0x6b0f },
	{ 0x251c, 0x421c },
	{ 0x251e, 0x7316 },
	{ 0x2520, 0x421d },
	{ 0x2522, 0x7318 },
	{ 0x2524, 0x8c0e },
	{ 0x2526, 0x7d0f },
	{ 0x2528, 0x2c1a },
	{ 0x252a, 0x0900 },
	{ 0x252c, 0x7312 },
	{ 0x252e, 0x421e },
	{ 0x2530, 0x7300 },
	{ 0x2532, 0x421f },
	{ 0x2534, 0x7302 },
	{ 0x2536, 0x5a0e },
	{ 0x2538, 0x6b0f },
	{ 0x253a, 0x421c },
	{ 0x253c, 0x7316 },
	{ 0x253e, 0x421d },
	{ 0x2540, 0x7318 },
	{ 0x2542, 0x8c0e },
	{ 0x2544, 0x7d0f },
	{ 0x2546, 0x2bf1 },
	{ 0x2548, 0x3c0a },
	{ 0x254a, 0x460e },
	{ 0x254c, 0x470f },
	{ 0x254e, 0x803e },
	{ 0x2550, 0x0800 },
	{ 0x2552, 0x730f },
	{ 0x2554, 0x2b92 },
	{ 0x2556, 0x4036 },
	{ 0x2558, 0x07ff },
	{ 0x255a, 0x4307 },
	{ 0x255c, 0x3f8e },
	{ 0x255e, 0x5221 },
	{ 0x2560, 0x4134 },
	{ 0x2562, 0x4135 },
	{ 0x2564, 0x4136 },
	{ 0x2566, 0x4137 },
	{ 0x2568, 0x4138 },
	{ 0x256a, 0x4139 },
	{ 0x256c, 0x413a },
	{ 0x256e, 0x413b },
	{ 0x2570, 0x4130 },
	{ 0x2572, 0x7400 },
	{ 0x2574, 0x2003 },
	{ 0x2576, 0x72a1 },
	{ 0x2578, 0x2f00 },
	{ 0x257a, 0x7020 },
	{ 0x257c, 0x2f21 },
	{ 0x257e, 0x7800 },
	{ 0x2580, 0x0040 },
	{ 0x2582, 0x7400 },
	{ 0x2584, 0x2005 },
	{ 0x2586, 0x72a1 },
	{ 0x2588, 0x2f00 },
	{ 0x258a, 0x7020 },
	{ 0x258c, 0x2f22 },
	{ 0x258e, 0x7800 },
	{ 0x2590, 0x7400 },
	{ 0x2592, 0x2011 },
	{ 0x2594, 0x72a1 },
	{ 0x2596, 0x2f00 },
	{ 0x2598, 0x7020 },
	{ 0x259a, 0x2f21 },
	{ 0x259c, 0x7800 },
	{ 0x259e, 0x7400 },
	{ 0x25a0, 0x2009 },
	{ 0x25a2, 0x72a1 },
	{ 0x25a4, 0x2f1f },
	{ 0x25a6, 0x7021 },
	{ 0x25a8, 0x3f40 },
	{ 0x25aa, 0x7800 },
	{ 0x25ac, 0x7400 },
	{ 0x25ae, 0x2005 },
	{ 0x25b0, 0x72a1 },
	{ 0x25b2, 0x2f1f },
	{ 0x25b4, 0x7021 },
	{ 0x25b6, 0x3f40 },
	{ 0x25b8, 0x7800 },
	{ 0x25ba, 0x7400 },
	{ 0x25bc, 0x2009 },
	{ 0x25be, 0x72a1 },
	{ 0x25c0, 0x2f00 },
	{ 0x25c2, 0x7020 },
	{ 0x25c4, 0x2f22 },
	{ 0x25c6, 0x7800 },
	{ 0x25c8, 0x0009 },
	{ 0x25ca, 0xf572 },
	{ 0x25cc, 0x0009 },
	{ 0x25ce, 0xf582 },
	{ 0x25d0, 0x0009 },
	{ 0x25d2, 0xf590 },
	{ 0x25d4, 0x0009 },
	{ 0x25d6, 0xf59e },
	{ 0x25d8, 0xf580 },
	{ 0x25da, 0x0004 },
	{ 0x25dc, 0x0009 },
	{ 0x25de, 0xf590 },
	{ 0x25e0, 0x0009 },
	{ 0x25e2, 0xf5ba },
	{ 0x25e4, 0x0009 },
	{ 0x25e6, 0xf572 },
	{ 0x25e8, 0x0009 },
	{ 0x25ea, 0xf5ac },
	{ 0x25ec, 0xf580 },
	{ 0x25ee, 0x0004 },
	{ 0x25f0, 0x0009 },
	{ 0x25f2, 0xf572 },
	{ 0x25f4, 0x0009 },
	{ 0x25f6, 0xf5ac },
	{ 0x25f8, 0x0009 },
	{ 0x25fa, 0xf590 },
	{ 0x25fc, 0x0009 },
	{ 0x25fe, 0xf59e },
	{ 0x2600, 0xf580 },
	{ 0x2602, 0x0004 },
	{ 0x2604, 0x0009 },
	{ 0x2606, 0xf590 },
	{ 0x2608, 0x0009 },
	{ 0x260a, 0xf59e },
	{ 0x260c, 0x0009 },
	{ 0x260e, 0xf572 },
	{ 0x2610, 0x0009 },
	{ 0x2612, 0xf5ac },
	{ 0x2614, 0xf580 },
	{ 0x2616, 0x0004 },
	{ 0x2618, 0x0212 },
	{ 0x261a, 0x0217 },
	{ 0x261c, 0x041f },
	{ 0x261e, 0x1017 },
	{ 0x2620, 0x0413 },
	{ 0x2622, 0x0103 },
	{ 0x2624, 0x010b },
	{ 0x2626, 0x1c0a },
	{ 0x2628, 0x0202 },
	{ 0x262a, 0x0407 },
	{ 0x262c, 0x0205 },
	{ 0x262e, 0x0204 },
	{ 0x2630, 0x0114 },
	{ 0x2632, 0x0110 },
	{ 0x2634, 0xffff },
	{ 0x2636, 0x0048 },
	{ 0x2638, 0x0090 },
	{ 0x263a, 0x0000 },
	{ 0x263c, 0x0000 },
	{ 0x263e, 0xf618 },
	{ 0x2640, 0x0000 },
	{ 0x2642, 0x0000 },
	{ 0x2644, 0x0060 },
	{ 0x2646, 0x0078 },
	{ 0x2648, 0x0060 },
	{ 0x264a, 0x0078 },
	{ 0x264c, 0x004f },
	{ 0x264e, 0x0037 },
	{ 0x2650, 0x0048 },
	{ 0x2652, 0x0090 },
	{ 0x2654, 0x0000 },
	{ 0x2656, 0x0000 },
	{ 0x2658, 0xf618 },
	{ 0x265a, 0x0000 },
	{ 0x265c, 0x0000 },
	{ 0x265e, 0x0180 },
	{ 0x2660, 0x0780 },
	{ 0x2662, 0x0180 },
	{ 0x2664, 0x0780 },
	{ 0x2666, 0x04cf },
	{ 0x2668, 0x0337 },
	{ 0x266a, 0xf636 },
	{ 0x266c, 0xf650 },
	{ 0x266e, 0xf5c8 },
	{ 0x2670, 0xf5dc },
	{ 0x2672, 0xf5f0 },
	{ 0x2674, 0xf604 },
	{ 0x2676, 0x0100 },
	{ 0x2678, 0xff8a },
	{ 0x267a, 0xffff },
	{ 0x267c, 0x0104 },
	{ 0x267e, 0xff0a },
	{ 0x2680, 0xffff },
	{ 0x2682, 0x0108 },
	{ 0x2684, 0xff02 },
	{ 0x2686, 0xffff },
	{ 0x2688, 0x010c },
	{ 0x268a, 0xff82 },
	{ 0x268c, 0xffff },
	{ 0x268e, 0x0004 },
	{ 0x2690, 0xf676 },
	{ 0x2692, 0xe4e4 },
	{ 0x2694, 0x4e4e },
	{ 0x2ffe, 0xc114 },
	{ 0x3224, 0xf222 },
	{ 0x322a, 0xf23c },
	{ 0x3230, 0xf03a },
	{ 0x3238, 0xf09a },
	{ 0x323a, 0xf012 },
	{ 0x323e, 0xf3f6 },
	{ 0x32a0, 0x0000 },
	{ 0x32a2, 0x0000 },
	{ 0x32a4, 0x0000 },
	{ 0x32b0, 0x0000 },
	{ 0x32c0, 0xf66a },
	{ 0x32c2, 0xf66e },
	{ 0x32c4, 0x0000 },
	{ 0x32c6, 0xf66e },
	{ 0x32c8, 0x0000 },
	{ 0x32ca, 0xf68e },
	{ 0x0a7e, 0x219c },
	{ 0x3244, 0x8400 },
	{ 0x3246, 0xe400 },
	{ 0x3248, 0xc88e },
	{ 0x324e, 0xfcd8 },
	{ 0x3250, 0xa060 },
	{ 0x325a, 0x7a37 },
	{ 0x0734, 0x4b0b },
	{ 0x0736, 0xd8b0 },
	{ 0x0600, 0x1190 },
	{ 0x0602, 0x0052 },
	{ 0x0604, 0x1008 },
	{ 0x0606, 0x0200 },
	{ 0x0616, 0x0040 },
	{ 0x0614, 0x0040 },
	{ 0x0612, 0x0040 },
	{ 0x0610, 0x0040 },
	{ 0x06b2, 0x0500 },
	{ 0x06b4, 0x3ff0 },
	{ 0x0618, 0x0a80 },
	{ 0x0668, 0x4303 },
	{ 0x06ca, 0x02cc },
	{ 0x066e, 0x0050 },
	{ 0x0670, 0x0050 },
	{ 0x113c, 0x0001 },
	{ 0x11c4, 0x1080 },
	{ 0x11c6, 0x0c34 },
	{ 0x1104, 0x0160 },
	{ 0x1106, 0x0138 },
	{ 0x110a, 0x010e },
	{ 0x110c, 0x021d },
	{ 0x110e, 0xba2e },
	{ 0x1110, 0x0056 },
	{ 0x1112, 0x00ac },
	{ 0x1114, 0x6907 },
	{ 0x1122, 0x0011 },
	{ 0x1124, 0x0022 },
	{ 0x1126, 0x2e8c },
	{ 0x1128, 0x0016 },
	{ 0x112a, 0x002b },
	{ 0x112c, 0x3483 },
	{ 0x1130, 0x0200 },
	{ 0x1132, 0x0200 },
	{ 0x1102, 0x0028 },
	{ 0x113e, 0x0200 },
	{ 0x0d00, 0x4000 },
	{ 0x0d02, 0x8004 },
	{ 0x120a, 0x0a00 },
	{ 0x0214, 0x0200 },
	{ 0x0216, 0x0200 },
	{ 0x0218, 0x0200 },
	{ 0x021a, 0x0200 },
	{ 0x1000, 0x0300 },
	{ 0x1002, 0xc319 },
	{ 0x105a, 0x0091 },
	{ 0x105c, 0x0f08 },
	{ 0x105e, 0x0000 },
	{ 0x1060, 0x3008 },
	{ 0x1062, 0x0000 },
	{ 0x0202, 0x0100 },
	{ 0x0b10, 0x400c },
	{ 0x0212, 0x0000 },
	{ 0x035e, 0x0701 },
	{ 0x040a, 0x0000 },
	{ 0x0420, 0x0003 },
	{ 0x0424, 0x0c47 },
	{ 0x0418, 0x1010 },
	{ 0x0740, 0x004f },
	{ 0x0354, 0x1000 },
	{ 0x035c, 0x0303 },
	{ 0x050e, 0x0000 },
	{ 0x0510, 0x0058 },
	{ 0x0512, 0x0058 },
	{ 0x0514, 0x0050 },
	{ 0x0516, 0x0050 },
	{ 0x0260, 0x0003 },
	{ 0x0262, 0x0700 },
	{ 0x0266, 0x0007 },
	{ 0x0250, 0x0000 },
	{ 0x0258, 0x0002 },
	{ 0x025c, 0x0002 },
	{ 0x025a, 0x03e8 },
	{ 0x0256, 0x0100 },
	{ 0x0254, 0x0001 },
	{ 0x0440, 0x000c },
	{ 0x0908, 0x0003 },
	{ 0x0708, 0x2f00 },
	{ 0x027e, 0x0100 },
};

static const struct hi1336_reg hi1336_mode0_4208x3120_30fps_regs[] = {
	{ 0x3250, 0xa060 },
	{ 0x0730, 0x770f },
	{ 0x0732, 0xe0b0 },
	{ 0x1118, 0x0006 },
	{ 0x1200, 0x011f },
	{ 0x1204, 0x1c01 },
	{ 0x1240, 0x0100 },
	{ 0x0b20, 0x8100 },
	{ 0x0f00, 0x0000 },
	{ 0x1002, 0xc319 },
	{ 0x1004, 0x2bab },
	{ 0x103e, 0x0000 },
	{ 0x1020, 0xc10b },
	{ 0x1022, 0x0a31 },
	{ 0x1024, 0x030b },
	{ 0x1026, 0x0d0f },
	{ 0x1028, 0x1a0e },
	{ 0x102a, 0x1311 },
	{ 0x102c, 0x2400 },
	{ 0x1010, 0x07d0 },
	{ 0x1012, 0x017d },
	{ 0x1014, 0x006a },
	{ 0x1016, 0x006a },
	{ 0x1018, 0x0040 },
	{ 0x101a, 0x006a },
	{ 0x1038, 0x4100 },
	{ 0x1042, 0x1108 },
	{ 0x1048, 0x0080 },
	{ 0x1044, 0x0100 },
	{ 0x1046, 0x0004 },
	{ 0x104c, 0x0000 },
	{ 0x0404, 0x0008 },
	{ 0x0406, 0x1087 },
	{ 0x0220, 0x0008 },
	{ 0x022a, 0x0017 },
	{ 0x0222, 0x0c80 },
	{ 0x022c, 0x0c89 },
	{ 0x0224, 0x002e },
	{ 0x022e, 0x0c61 },
	{ 0x0f04, 0x0008 },
	{ 0x0f06, 0x0000 },
	{ 0x023a, 0x1111 },
	{ 0x0234, 0x1111 },
	{ 0x0238, 0x1111 },
	{ 0x0246, 0x0020 },
	{ 0x020a, 0x0cfe },
	{ 0x021c, 0x0008 },
	{ 0x0206, 0x05dd },
	{ 0x020e, 0x0d02 },
	{ 0x0b12, 0x1070 },
	{ 0x0b14, 0x0c30 },
	{ 0x0204, 0x0000 },
	{ 0x041c, 0x0048 },
	{ 0x041e, 0x1047 },
	{ 0x0b04, 0x037c },
};

static const struct hi1336_reg hi1336_mode1_2400x1080_30fps_regs[] = {
	{ 0x3250, 0xa060 },
	{ 0x0730, 0x770f },
	{ 0x0732, 0xe0b0 },
	{ 0x1118, 0x0402 },
	{ 0x1200, 0x011f },
	{ 0x1204, 0x1c01 },
	{ 0x1240, 0x0100 },
	{ 0x0b20, 0x8100 },
	{ 0x0f00, 0x0000 },
	{ 0x1002, 0xc319 },
	{ 0x103e, 0x0000 },
	{ 0x1020, 0xc10b },
	{ 0x1022, 0x0a31 },
	{ 0x1024, 0x030b },
	{ 0x1026, 0x0d0f },
	{ 0x1028, 0x1a0e },
	{ 0x102a, 0x1311 },
	{ 0x102c, 0x2400 },
	{ 0x1010, 0x07d0 },
	{ 0x1012, 0x03b2 },
	{ 0x1014, 0x0196 },
	{ 0x1016, 0x0196 },
	{ 0x101a, 0x0196 },
	{ 0x0404, 0x0008 },
	{ 0x0406, 0x1087 },
	{ 0x0220, 0x0008 },
	{ 0x022a, 0x0017 },
	{ 0x0222, 0x0c80 },
	{ 0x022c, 0x0c89 },
	{ 0x0224, 0x042a },
	{ 0x022e, 0x0865 },
	{ 0x0f04, 0x0390 },
	{ 0x0f06, 0x0000 },
	{ 0x023a, 0x1111 },
	{ 0x0234, 0x1111 },
	{ 0x0238, 0x1111 },
	{ 0x0246, 0x0020 },
	{ 0x020a, 0x0cfe },
	{ 0x021c, 0x0008 },
	{ 0x0206, 0x05dd },
	{ 0x020e, 0x0d02 },
	{ 0x0b12, 0x0960 },
	{ 0x0b14, 0x0438 },
	{ 0x0204, 0x0000 },
	{ 0x041c, 0x0048 },
	{ 0x041e, 0x1047 },
	{ 0x0b04, 0x037c },
};

static const struct hi1336_reg hi1336_mode2_2104x1560_30fps_regs[] = {
	{ 0x3250, 0xa470 },
	{ 0x0730, 0x770f },
	{ 0x0732, 0xe1b0 },
	{ 0x1118, 0x0004 },
	{ 0x1200, 0x011f },
	{ 0x1204, 0x1c01 },
	{ 0x1240, 0x0100 },
	{ 0x0b20, 0x8200 },
	{ 0x0f00, 0x0400 },
	{ 0x1002, 0xc319 },
	{ 0x1004, 0x2bab },
	{ 0x103e, 0x0100 },
	{ 0x1020, 0xc106 },
	{ 0x1022, 0x0617 },
	{ 0x1024, 0x0306 },
	{ 0x1026, 0x0609 },
	{ 0x1028, 0x1207 },
	{ 0x102a, 0x090a },
	{ 0x102c, 0x1400 },
	{ 0x1010, 0x07d0 },
	{ 0x1012, 0x00ba },
	{ 0x1014, 0x001b },
	{ 0x1016, 0x001b },
	{ 0x1018, 0x0040 },
	{ 0x101a, 0x001b },
	{ 0x1038, 0x0000 },
	{ 0x1042, 0x0008 },
	{ 0x1048, 0x0080 },
	{ 0x1044, 0x0100 },
	{ 0x1046, 0x0004 },
	{ 0x104c, 0x0000 },
	{ 0x0404, 0x0008 },
	{ 0x0406, 0x1087 },
	{ 0x0220, 0x0008 },
	{ 0x022a, 0x0015 },
	{ 0x0222, 0x0c80 },
	{ 0x022c, 0x0c89 },
	{ 0x0224, 0x002c },
	{ 0x022e, 0x0c61 },
	{ 0x0f04, 0x0004 },
	{ 0x0f06, 0x0000 },
	{ 0x023a, 0x2222 },
	{ 0x0234, 0x3311 },
	{ 0x0238, 0x3311 },
	{ 0x0246, 0x0020 },
	{ 0x020a, 0x0cfe },
	{ 0x021c, 0x0008 },
	{ 0x0206, 0x05dd },
	{ 0x020e, 0x0d02 },
	{ 0x0b12, 0x0838 },
	{ 0x0b14, 0x0618 },
	{ 0x0204, 0x0200 },
	{ 0x041c, 0x0048 },
	{ 0x041e, 0x1047 },
	{ 0x0b04, 0x037c },
};

static const struct hi1336_reg hi1336_mode3_1920x1080_60fps_regs[] = {
	{ 0x3250, 0xa470 },
	{ 0x0730, 0x770f },
	{ 0x0732, 0xe1b0 },
	{ 0x1118, 0x01aa },
	{ 0x1200, 0x011f },
	{ 0x1204, 0x1c01 },
	{ 0x1240, 0x00f0 },
	{ 0x0b20, 0x8200 },
	{ 0x0f00, 0x0400 },
	{ 0x1002, 0xc319 },
	{ 0x103e, 0x0100 },
	{ 0x1020, 0xc106 },
	{ 0x1022, 0x0617 },
	{ 0x1024, 0x0306 },
	{ 0x1026, 0x0609 },
	{ 0x1028, 0x1207 },
	{ 0x102a, 0x090a },
	{ 0x102c, 0x1400 },
	{ 0x1010, 0x07d0 },
	{ 0x1012, 0x00f4 },
	{ 0x1014, 0x0038 },
	{ 0x1016, 0x0038 },
	{ 0x101a, 0x0038 },
	{ 0x0404, 0x0008 },
	{ 0x0406, 0x1087 },
	{ 0x0220, 0x0008 },
	{ 0x022a, 0x0015 },
	{ 0x0222, 0x0c80 },
	{ 0x022c, 0x0c89 },
	{ 0x0224, 0x020c },
	{ 0x022e, 0x0a81 },
	{ 0x0f04, 0x0060 },
	{ 0x0f06, 0x0000 },
	{ 0x023a, 0x2222 },
	{ 0x0234, 0x3311 },
	{ 0x0238, 0x3311 },
	{ 0x0246, 0x0020 },
	{ 0x020a, 0x067c },
	{ 0x021c, 0x0008 },
	{ 0x0206, 0x05dd },
	{ 0x020e, 0x0681 },
	{ 0x0b12, 0x0780 },
	{ 0x0b14, 0x0438 },
	{ 0x0204, 0x0200 },
	{ 0x041c, 0x0048 },
	{ 0x041e, 0x1047 },
	{ 0x0b04, 0x037c },
};

static const struct hi1336_mode hi1336_supported_modes[] = {
	{
		.width = 4208,
		.height = 3120,
		.hts = 6004,
		.vts = 3330,
		.fps_milli = 30000,
		.exposure_margin = 4,
		.exposure_default = 0x0cfe,
		.link_freq_index = HI1336_LINK_FREQ_720MHZ_INDEX,
		.pixel_rate = 571200000,
		.crop = { 0, 0, 4208, 3120 },
		.reg_list = {
			.regs = hi1336_mode0_4208x3120_30fps_regs,
			.num_regs = ARRAY_SIZE(hi1336_mode0_4208x3120_30fps_regs),
		},
	}, {
		.width = 2400,
		.height = 1080,
		.hts = 6004,
		.vts = 3330,
		.fps_milli = 30000,
		.exposure_margin = 4,
		.exposure_default = 0x0cfe,
		.link_freq_index = HI1336_LINK_FREQ_720MHZ_INDEX,
		.pixel_rate = 571200000,
		.crop = { 0, 0, 2400, 1080 },
		.reg_list = {
			.regs = hi1336_mode1_2400x1080_30fps_regs,
			.num_regs = ARRAY_SIZE(hi1336_mode1_2400x1080_30fps_regs),
		},
	}, {
		.width = 2104,
		.height = 1560,
		.hts = 6004,
		.vts = 3330,
		.fps_milli = 30000,
		.exposure_margin = 4,
		.exposure_default = 0x0cfe,
		.link_freq_index = HI1336_LINK_FREQ_360MHZ_INDEX,
		.pixel_rate = 285600000,
		.crop = { 0, 0, 2104, 1560 },
		.reg_list = {
			.regs = hi1336_mode2_2104x1560_30fps_regs,
			.num_regs = ARRAY_SIZE(hi1336_mode2_2104x1560_30fps_regs),
		},
	}, {
		.width = 1920,
		.height = 1080,
		.hts = 6004,
		.vts = 1665,
		.fps_milli = 60000,
		.exposure_margin = 5,
		.exposure_default = 0x067c,
		.link_freq_index = HI1336_LINK_FREQ_360MHZ_INDEX,
		.pixel_rate = 285600000,
		.crop = { 0, 0, 1920, 1080 },
		.reg_list = {
			.regs = hi1336_mode3_1920x1080_60fps_regs,
			.num_regs = ARRAY_SIZE(hi1336_mode3_1920x1080_60fps_regs),
		},
	},
};

static int hi1336_write_regs(struct hi1336 *hi1336,
			     const struct hi1336_reg *regs, unsigned int num_regs)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < num_regs; i++) {
		cci_write(hi1336->regmap, CCI_REG16(regs[i].address), regs[i].val, &ret);
		if (ret)
			return ret;
	}

	return 0;
}

static int hi1336_write_grouped_reg(struct hi1336 *hi1336, u32 reg, u32 val)
{
	int ret = 0;

	cci_write(hi1336->regmap, HI1336_REG_GROUP_HOLD, HI1336_GROUP_HOLD_ON, &ret);
	cci_write(hi1336->regmap, reg, val, &ret);
	cci_write(hi1336->regmap, HI1336_REG_GROUP_HOLD, HI1336_GROUP_HOLD_OFF, &ret);

	return ret;
}

static int hi1336_set_test_pattern(struct hi1336 *hi1336, int pattern)
{
	int ret = 0;

	cci_write(hi1336->regmap, HI1336_REG_TEST_PATTERN_EN,
		  pattern ? 0x037f : 0x037c, &ret);
	cci_write(hi1336->regmap, HI1336_REG_TEST_PATTERN,
		  pattern ? pattern << 8 : 0, &ret);

	return ret;
}

static int hi1336_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hi1336 *hi1336 = container_of(ctrl->handler,
					     struct hi1336, ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 exposure_max = hi1336->mode->height + ctrl->val -
				   hi1336->mode->exposure_margin;
		u32 exposure_def = min_t(u32, exposure_max, hi1336->exposure->val);

		ret = __v4l2_ctrl_modify_range(hi1336->exposure,
					      HI1336_EXPOSURE_MIN, exposure_max,
					      1, exposure_def);
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(hi1336->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = hi1336_write_grouped_reg(hi1336, HI1336_REG_FRAME_LENGTH,
						 hi1336->mode->height + ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = hi1336_write_grouped_reg(hi1336, HI1336_REG_EXPOSURE, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(hi1336->regmap, HI1336_REG_GROUP_HOLD,
			  HI1336_GROUP_HOLD_ON, &ret);
		cci_write(hi1336->regmap, HI1336_REG_DIGITAL_GAIN_GR, ctrl->val, &ret);
		cci_write(hi1336->regmap, HI1336_REG_DIGITAL_GAIN_GB, ctrl->val, &ret);
		cci_write(hi1336->regmap, HI1336_REG_DIGITAL_GAIN_R, ctrl->val, &ret);
		cci_write(hi1336->regmap, HI1336_REG_DIGITAL_GAIN_B, ctrl->val, &ret);
		cci_write(hi1336->regmap, HI1336_REG_GROUP_HOLD,
			  HI1336_GROUP_HOLD_OFF, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = hi1336_set_test_pattern(hi1336, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(hi1336->dev);

	return ret;
}

static const struct v4l2_ctrl_ops hi1336_ctrl_ops = {
	.s_ctrl = hi1336_set_ctrl,
};

static u32 hi1336_exposure_default(const struct hi1336_mode *mode)
{
	return mode->exposure_default;
}

static int hi1336_update_controls(struct hi1336 *hi1336,
				  const struct hi1336_mode *mode)
{
	s64 hblank = mode->hts - mode->width;
	s64 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - mode->exposure_margin;
	u32 exposure_def = hi1336_exposure_default(mode);
	int ret;

	ret = __v4l2_ctrl_s_ctrl(hi1336->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(hi1336->pixel_rate,
					      mode->pixel_rate, mode->pixel_rate,
					      1, mode->pixel_rate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl_int64(hi1336->pixel_rate, mode->pixel_rate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(hi1336->hblank, hblank, hblank, 1,
					      hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(hi1336->hblank, hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(hi1336->vblank, vblank,
					      HI1336_FRAME_LENGTH_MAX - mode->height,
					      1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(hi1336->vblank, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(hi1336->exposure,
					      HI1336_EXPOSURE_MIN, exposure_max,
					      1, exposure_def);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(hi1336->exposure, exposure_def);
}

static int hi1336_set_active_mode(struct hi1336 *hi1336,
				  const struct hi1336_mode *mode)
{
	const struct hi1336_mode *old_mode = hi1336->mode;
	int ret;

	if (old_mode == mode)
		return 0;

	hi1336->mode = mode;

	ret = hi1336_update_controls(hi1336, mode);
	if (ret)
		hi1336->mode = old_mode;

	return ret;
}

static int hi1336_init_controls(struct hi1336 *hi1336)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &hi1336->ctrl_handler;
	const struct hi1336_mode *mode = hi1336->mode;
	struct v4l2_fwnode_device_properties props;
	s64 hblank = mode->hts - mode->width;
	s64 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - mode->exposure_margin;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 7);

	hi1336->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(hi1336_link_freq_menu) - 1,
						   mode->link_freq_index,
						   hi1336_link_freq_menu);
	if (hi1336->link_freq)
		hi1336->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hi1336->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
					       V4L2_CID_PIXEL_RATE,
					       mode->pixel_rate, mode->pixel_rate,
					       1, mode->pixel_rate);
	if (hi1336->pixel_rate)
		hi1336->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hi1336->hblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_HBLANK,
					    hblank, hblank, 1, hblank);
	if (hi1336->hblank)
		hi1336->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hi1336->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &hi1336_ctrl_ops,
					    V4L2_CID_VBLANK, vblank,
					    HI1336_FRAME_LENGTH_MAX - mode->height,
					    1, vblank);
	hi1336->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &hi1336_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      HI1336_EXPOSURE_MIN, exposure_max, 1,
					      hi1336_exposure_default(mode));
	hi1336->digital_gain = v4l2_ctrl_new_std(ctrl_hdlr, &hi1336_ctrl_ops,
						  V4L2_CID_DIGITAL_GAIN,
						  HI1336_DIGITAL_GAIN_MIN,
						  HI1336_DIGITAL_GAIN_MAX, 1,
						  HI1336_DIGITAL_GAIN_DEFAULT);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &hi1336_ctrl_ops,
				       V4L2_CID_TEST_PATTERN,
				       ARRAY_SIZE(hi1336_test_pattern_menu) - 1,
				       0, 0, hi1336_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(hi1336->dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &hi1336_ctrl_ops, &props);
	if (ret)
		goto err_free_handler;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto err_free_handler;
	}

	hi1336->sd.ctrl_handler = ctrl_hdlr;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int hi1336_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	const struct hi1336_mode *mode = hi1336->mode;
	int ret;

	ret = pm_runtime_resume_and_get(hi1336->dev);
	if (ret)
		return ret;

	ret = hi1336_write_regs(hi1336, hi1336_init_regs,
				  ARRAY_SIZE(hi1336_init_regs));
	if (ret)
		goto err_pm_put;

	ret = hi1336_write_regs(hi1336, mode->reg_list.regs,
				  mode->reg_list.num_regs);
	if (ret)
		goto err_pm_put;

	ret = __v4l2_ctrl_handler_setup(hi1336->sd.ctrl_handler);
	if (ret)
		goto err_pm_put;

	ret = cci_write(hi1336->regmap, HI1336_REG_STREAM, HI1336_STREAM_ON, NULL);
	if (ret)
		goto err_pm_put;

	return 0;

err_pm_put:
	dev_err(hi1336->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_sync(hi1336->dev);

	return ret;
}

static int hi1336_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	int ret;

	ret = cci_write(hi1336->regmap, HI1336_REG_STREAM, HI1336_STREAM_OFF, NULL);
	if (ret)
		dev_err(hi1336->dev, "failed to stop streaming: %d\n", ret);

	/*
	 * Hi-1336 is not reliable when the next full init/mode table is written
	 * while the sensor remains powered after a previous stream.
	 */
	pm_runtime_put_sync(hi1336->dev);

	return ret;
}

static void hi1336_update_pad_format(const struct hi1336_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SGBRG10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static void hi1336_set_frame_interval(struct v4l2_fract *interval,
				       const struct hi1336_mode *mode)
{
	interval->numerator = 1000;
	interval->denominator = mode->fps_milli;
}

static u32 hi1336_fps_diff(u32 a, u32 b)
{
	return a > b ? a - b : b - a;
}

static u32 hi1336_interval_to_fps_milli(const struct v4l2_fract *interval,
					       u32 fallback)
{
	if (!interval || !interval->numerator || !interval->denominator)
		return fallback;

	return DIV_ROUND_CLOSEST_ULL((u64)interval->denominator * 1000,
				     interval->numerator);
}

static const struct hi1336_mode *
hi1336_find_nearest_mode(const struct hi1336 *hi1336, u32 width, u32 height,
			 const struct v4l2_fract *interval)
{
	const struct hi1336_mode *nearest, *best;
	u32 fps_milli;
	u32 best_diff = U32_MAX;
	unsigned int i;

	nearest = v4l2_find_nearest_size(hi1336_supported_modes,
					       ARRAY_SIZE(hi1336_supported_modes),
					       width, height, width, height);
	best = nearest;
	fps_milli = hi1336_interval_to_fps_milli(interval,
						  hi1336->mode->fps_milli);

	for (i = 0; i < ARRAY_SIZE(hi1336_supported_modes); i++) {
		const struct hi1336_mode *mode = &hi1336_supported_modes[i];
		u32 diff;

		if (mode->width != nearest->width || mode->height != nearest->height)
			continue;

		diff = hi1336_fps_diff(mode->fps_milli, fps_milli);
		if (diff < best_diff) {
			best = mode;
			best_diff = diff;
		}
	}

	return best;
}

static int hi1336_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	struct v4l2_fract interval;
	const struct hi1336_mode *mode;
	int ret;

	if (fmt->pad)
		return -EINVAL;

	hi1336_set_frame_interval(&interval, hi1336->mode);
	mode = hi1336_find_nearest_mode(hi1336, fmt->format.width,
					       fmt->format.height, &interval);

	hi1336_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = hi1336_set_active_mode(hi1336, mode);
		if (ret)
			return ret;
	}

	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	*v4l2_subdev_state_get_crop(state, fmt->pad) = mode->crop;

	return 0;
}

static int hi1336_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGBRG10_1X10;

	return 0;
}

static int hi1336_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct hi1336_mode *mode;

	if (fse->pad || fse->index >= ARRAY_SIZE(hi1336_supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGBRG10_1X10)
		return -EINVAL;

	mode = &hi1336_supported_modes[fse->index];
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static int hi1336_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	unsigned int i, index = 0;

	if (fie->pad || fie->code != MEDIA_BUS_FMT_SGBRG10_1X10)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(hi1336_supported_modes); i++) {
		const struct hi1336_mode *mode = &hi1336_supported_modes[i];

		if (mode->width != fie->width || mode->height != fie->height)
			continue;

		if (index++ != fie->index)
			continue;

		hi1336_set_frame_interval(&fie->interval, mode);
		return 0;
	}

	return -EINVAL;
}

static int hi1336_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *interval)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	const struct hi1336_mode *mode = hi1336->mode;

	if (interval->pad)
		return -EINVAL;

	if (interval->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_state_get_format(state, interval->pad);
		mode = hi1336_find_nearest_mode(hi1336, fmt->width, fmt->height,
						     NULL);
	}

	hi1336_set_frame_interval(&interval->interval, mode);

	return 0;
}

static int hi1336_set_frame_interval_op(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_frame_interval *interval)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	struct v4l2_mbus_framefmt *fmt;
	const struct hi1336_mode *mode;
	int ret;

	if (interval->pad)
		return -EINVAL;

	if (interval->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_state_get_format(state, interval->pad);
		mode = hi1336_find_nearest_mode(hi1336, fmt->width, fmt->height,
						     &interval->interval);
		*v4l2_subdev_state_get_crop(state, interval->pad) = mode->crop;
	} else {
		mode = hi1336_find_nearest_mode(hi1336, hi1336->mode->width,
						     hi1336->mode->height,
						     &interval->interval);
		ret = hi1336_set_active_mode(hi1336, mode);
		if (ret)
			return ret;
	}

	hi1336_set_frame_interval(&interval->interval, mode);

	return 0;
}

static int hi1336_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct hi1336 *hi1336 = to_hi1336(sd);

	if (sel->pad)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
			sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		else
			sel->r = hi1336->mode->crop;
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = HI1336_NATIVE_WIDTH;
		sel->r.height = HI1336_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int hi1336_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct hi1336 *hi1336 = to_hi1336(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = hi1336->mode->width,
			.height = hi1336->mode->height,
		},
	};

	hi1336_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops hi1336_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops hi1336_pad_ops = {
	.set_fmt = hi1336_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = hi1336_get_selection,
	.enum_mbus_code = hi1336_enum_mbus_code,
	.enum_frame_size = hi1336_enum_frame_size,
	.enum_frame_interval = hi1336_enum_frame_interval,
	.get_frame_interval = hi1336_get_frame_interval,
	.set_frame_interval = hi1336_set_frame_interval_op,
	.enable_streams = hi1336_enable_streams,
	.disable_streams = hi1336_disable_streams,
};

static const struct v4l2_subdev_ops hi1336_subdev_ops = {
	.video = &hi1336_video_ops,
	.pad = &hi1336_pad_ops,
};

static const struct v4l2_subdev_internal_ops hi1336_internal_ops = {
	.init_state = hi1336_init_state,
};

static const struct media_entity_operations hi1336_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int hi1336_identify_sensor(struct hi1336 *hi1336)
{
	u64 val;
	int ret;

	ret = cci_read(hi1336->regmap, HI1336_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(hi1336->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != HI1336_CHIP_ID) {
		dev_err(hi1336->dev, "chip id mismatch: 0x%x != 0x%llx\n",
			HI1336_CHIP_ID, val);
		return -ENODEV;
	}

	dev_info(hi1336->dev, "chip id 0x%04llx using %u mipi lanes\n", val,
		 HI1336_DATA_LANES);

	return 0;
}

static int hi1336_check_hwcfg(struct hi1336 *hi1336)
{
	struct fwnode_handle *fwnode = dev_fwnode(hi1336->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = HI1336_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != HI1336_DATA_LANES) {
		dev_err(hi1336->dev, "invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto out_free_bus_cfg;
	}

	ret = v4l2_link_freq_to_bitmap(hi1336->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       hi1336_link_freq_menu,
				       ARRAY_SIZE(hi1336_link_freq_menu),
				       &freq_bitmap);

out_free_bus_cfg:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int hi1336_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct hi1336 *hi1336 = to_hi1336(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(hi1336_supply_names),
				    hi1336->supplies);
	if (ret)
		return ret;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	ret = clk_prepare_enable(hi1336->mclk);
	if (ret)
		goto err_disable_regulators;

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(hi1336->reset_gpio, 0);
	usleep_range(30 * USEC_PER_MSEC, 31 * USEC_PER_MSEC);

	return 0;

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(hi1336_supply_names),
			       hi1336->supplies);

	return ret;
}

static int hi1336_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct hi1336 *hi1336 = to_hi1336(sd);

	gpiod_set_value_cansleep(hi1336->reset_gpio, 1);
	usleep_range(1000, 1500);
	clk_disable_unprepare(hi1336->mclk);
	regulator_bulk_disable(ARRAY_SIZE(hi1336_supply_names),
			       hi1336->supplies);

	return 0;
}

static int hi1336_probe(struct i2c_client *client)
{
	struct hi1336 *hi1336;
	unsigned int i;
	int ret;

	hi1336 = devm_kzalloc(&client->dev, sizeof(*hi1336), GFP_KERNEL);
	if (!hi1336)
		return -ENOMEM;

	hi1336->dev = &client->dev;
	hi1336->mode = &hi1336_supported_modes[HI1336_DEFAULT_MODE];

	hi1336->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(hi1336->regmap))
		return dev_err_probe(hi1336->dev, PTR_ERR(hi1336->regmap),
				     "failed to initialize CCI regmap\n");

	hi1336->mclk = devm_clk_get(hi1336->dev, NULL);
	if (IS_ERR(hi1336->mclk))
		return dev_err_probe(hi1336->dev, PTR_ERR(hi1336->mclk),
				     "failed to get mclk\n");

	if (clk_get_rate(hi1336->mclk) != HI1336_MCLK_FREQ_24MHZ)
		return dev_err_probe(hi1336->dev, -EINVAL,
				     "mclk frequency must be 24 MHz\n");

	for (i = 0; i < ARRAY_SIZE(hi1336_supply_names); i++)
		hi1336->supplies[i].supply = hi1336_supply_names[i];

	ret = devm_regulator_bulk_get(hi1336->dev, ARRAY_SIZE(hi1336_supply_names),
				      hi1336->supplies);
	if (ret)
		return dev_err_probe(hi1336->dev, ret,
				     "failed to get regulators\n");

	hi1336->reset_gpio = devm_gpiod_get_optional(hi1336->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(hi1336->reset_gpio))
		return dev_err_probe(hi1336->dev, PTR_ERR(hi1336->reset_gpio),
				     "failed to get reset gpio\n");

	v4l2_i2c_subdev_init(&hi1336->sd, client, &hi1336_subdev_ops);
	hi1336->sd.internal_ops = &hi1336_internal_ops;
	hi1336->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			     V4L2_SUBDEV_FL_HAS_EVENTS;

	ret = hi1336_check_hwcfg(hi1336);
	if (ret)
		return dev_err_probe(hi1336->dev, ret,
				     "failed to check hardware configuration\n");

	ret = hi1336_power_on(hi1336->dev);
	if (ret)
		return dev_err_probe(hi1336->dev, ret, "failed to power on\n");

	ret = hi1336_identify_sensor(hi1336);
	if (ret)
		goto err_power_off;

	ret = hi1336_init_controls(hi1336);
	if (ret)
		goto err_power_off;

	hi1336->pad.flags = MEDIA_PAD_FL_SOURCE;
	hi1336->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	hi1336->sd.entity.ops = &hi1336_subdev_entity_ops;
	ret = media_entity_pads_init(&hi1336->sd.entity, 1, &hi1336->pad);
	if (ret)
		goto err_free_ctrls;

	ret = v4l2_subdev_init_finalize(&hi1336->sd);
	if (ret)
		goto err_media_cleanup;

	pm_runtime_set_active(hi1336->dev);
	pm_runtime_enable(hi1336->dev);
	pm_runtime_idle(hi1336->dev);

	ret = v4l2_async_register_subdev_sensor(&hi1336->sd);
	if (ret)
		goto err_pm_runtime;

	return 0;

err_pm_runtime:
	pm_runtime_disable(hi1336->dev);
	pm_runtime_set_suspended(hi1336->dev);
	v4l2_subdev_cleanup(&hi1336->sd);
err_media_cleanup:
	media_entity_cleanup(&hi1336->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(hi1336->sd.ctrl_handler);
err_power_off:
	hi1336_power_off(hi1336->dev);

	return ret;
}

static void hi1336_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi1336 *hi1336 = to_hi1336(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(hi1336->dev);
	if (!pm_runtime_status_suspended(hi1336->dev)) {
		hi1336_power_off(hi1336->dev);
		pm_runtime_set_suspended(hi1336->dev);
	}
}

static const struct dev_pm_ops hi1336_pm_ops = {
	SET_RUNTIME_PM_OPS(hi1336_power_off, hi1336_power_on, NULL)
};

static const struct of_device_id hi1336_of_match[] = {
	{ .compatible = "hynix,hi1336" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hi1336_of_match);

static struct i2c_driver hi1336_i2c_driver = {
	.driver = {
		.name = "hi1336",
		.pm = &hi1336_pm_ops,
		.of_match_table = hi1336_of_match,
	},
	.probe = hi1336_probe,
	.remove = hi1336_remove,
};
module_i2c_driver(hi1336_i2c_driver);

MODULE_DESCRIPTION("SK Hynix Hi-1336 camera sensor driver");
MODULE_LICENSE("GPL");
