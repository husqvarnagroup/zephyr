/*
 * Copyright (c) 2021 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_si4467_hw.h"

#include <ztest.h>
#include <ztest_error_hook.h>

#include <drivers/transceiver/si4467.h>

static struct si4467_context ctx;

struct device dev = {
	.data = &ctx,
};

static int
test_si4467_state_dispatch_nirq_events(struct si4467_context *const ctx_si4467)
{
	return 0;
}

void test_si4467_hw_init(void)
{
	zassert_ok(si4467_init(&dev, test_si4467_state_dispatch_nirq_events),
		   "Successful startup expected");
}

void test_si4467_hw_setup(void)
{
	zassert_ok(si4467_start(&dev), "Successful startup expected");
}

void test_si4467_hw_properties_get_many(void)
{
	uint8_t rx_hop_bytes[66];

	zassert_true(si4467_command_get_properties(
			     &ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			     SI4467_PROPERTY_RX_HOP_CONTROL, rx_hop_bytes,
			     17) < 0,
		     "No more than 16 property bytes can be fetched at once");

	zassert_ok(si4467_command_get_properties(
			   &ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			   SI4467_PROPERTY_RX_HOP_CONTROL, rx_hop_bytes, 16),
		   "Getting control, table_size and first 14 table entries");
	zassert_equal(rx_hop_bytes[0], 0x04,
		      "Default: Hopping disabled, 4 nibbles RSSI timeout");
	zassert_equal(rx_hop_bytes[1], 1, "Default: Table size of one");
	zassert_equal(rx_hop_bytes[2], 0, "Default: Entry 0 has value 0");
	zassert_equal(rx_hop_bytes[3], 1, "Default: Entry 1 has value 1");
	/* ...many more... */
	zassert_equal(rx_hop_bytes[15], 13, "Default: Entry 13 has value 13");
}

void test_si4467_hw_restart(void)
{
	zassert_ok(si4467_start(&dev), "Successful startup expected");
	zassert_ok(si4467_start(&dev), "Successful restart expected");
	zassert_ok(si4467_start(&dev), "Successful restart expected");
}

void test_si4467_hw_properties_set_and_get_one(void)
{
	uint8_t rx_hop_byte;

	zassert_ok(si4467_command_set_property(
			   &ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			   SI4467_PROPERTY_RX_HOP_TABLE_SIZE, 2),
		   "Setting RX hopping table size to two");

	zassert_ok(si4467_command_get_properties(
			   &ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			   SI4467_PROPERTY_RX_HOP_TABLE_SIZE, &rx_hop_byte, 1),
		   "Expecting hopping table size to be actually two");
}

void test_si4467_hw_configure_rx_hopping(void)
{
	const uint8_t channels[5] = { 10, 11, 12, 13, 55 };
	const struct si4467_property_group_rx_hop_control config = {
		.rssi_timeout = 0xF,
		.enable = SI4467_PROPERTY_RX_HOP_CONTROL_EN_PM_RSSI_PM_SYNC_TO,
	};
	uint8_t property_bytes[8];

	zassert_equal(si4467_configure_rx_hopping(&ctx, &config, NULL, 65),
		      -EINVAL, "Refusing invalid table size");

	zassert_ok(si4467_configure_rx_hopping(&ctx, &config, channels,
					       ARRAY_SIZE(channels)),
		   "Applying must be Successful");

	zassert_ok(si4467_command_get_properties(
			   &ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			   SI4467_PROPERTY_RX_HOP_CONTROL, property_bytes,
			   ARRAY_SIZE(property_bytes)),
		   "Fetching property bytes must work");

	zassert_equal(property_bytes[0], 0x4F,
		      "HOP_RSSI_PM_SYNC_TO and max RSSI timeout set");
	zassert_equal(property_bytes[1], 5, "Table size of 5");
	zassert_equal(property_bytes[2], 10, "Channel entry 0 is 10");
	zassert_equal(property_bytes[3], 11, "Channel entry 1 is 11");
	zassert_equal(property_bytes[4], 12, "Channel entry 2 is 12");
	zassert_equal(property_bytes[5], 13, "Channel entry 3 is 13");
	zassert_equal(property_bytes[6], 55, "Channel entry 4 is 55");
	zassert_equal(property_bytes[7], 0xFF, "Unused channel entry 5 = 0xFF");
}

void test_si4467_hw_get_property_clkgen_band(void)
{
	struct si4467_property_modem_clkgen_band band;

	zassert_ok(si4467_get_property_clkgen_band(&ctx, &band),
		   "No failure expected");

	zassert_equal(band.band,
		      SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_4,
		      "Default is 900 MHz band");
	zassert_equal(band.sy_sel,
		      SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_HIGH_PERFORMANCE,
		      "Default is high performance mode");
	zassert_equal(band.force_sy_recal,
		      SI4467_PROPERTY_MODEM_CLKGEN_BAND_FORCE_SY_RECAL_FORCE,
		      "Force recalibration is enabled by default");
}

void test_si4467_hw_get_property_freq_control_inte(void)
{
	uint8_t inte;

	zassert_ok(si4467_get_property_freq_control_inte(&ctx, &inte),
		   "No failure expected");

	zassert_equal(inte, 0x3C, "Default value is 0x3c");
}

void test_si4467_hw_get_property_freq_control_frac(void)
{
	uint32_t frac;

	zassert_ok(si4467_get_property_freq_control_frac(&ctx, &frac),
		   "No failure expected");

	zassert_equal(frac, 0x080000, "Default value is 0x080000");
}

void test_si4467_hw_get_property_freq_control_channel_ss(void)
{
	uint16_t step_size;

	zassert_ok(si4467_get_property_freq_control_channel_step_size(
			   &ctx, &step_size),
		   "No failure expected");

	zassert_equal(step_size, 0x0000, "Default value is 0x0000");
}

void test_si4467_hw_get_property_rx_hop_control(void)
{
	struct si4467_property_group_rx_hop_control control;

	zassert_ok(si4467_get_property_rx_hop_control(&ctx, &control),
		   "No failure expected");

	zassert_equal(control.enable, SI4467_PROPERTY_RX_HOP_CONTROL_EN_DISABLE,
		      "Hopping should be disabled");
	zassert_equal(control.rssi_timeout, 0x4, "Default RSSI timeout is 0x4");
}

void test_si4467_hw_get_property_pwr_lvl(void)
{
	uint8_t power_level;

	zassert_ok(si4467_get_property_pwr_lvl(&ctx, &power_level),
		   "No failure expected");

	/*
	 * note: we get the chip default here, not what we have in WDS config
	 * (as WDS config is not loaded for tests)
	 */
	zassert_equal(power_level, 0x7f, "Default power level expected");
}

void test_si4467_hw_set_property_pwr_lvl(void)
{
	zassert_ok(si4467_set_property_pwr_lvl(&ctx, 0), "No failure expected");

	zassert_ok(si4467_set_property_pwr_lvl(&ctx, 0x7f),
		   "No failure expected");

	uint8_t power_level;

	zassert_ok(si4467_get_property_pwr_lvl(&ctx, &power_level),
		   "No failure expected");
	zassert_equal(power_level, 0x7f, "Updated power level expected");

	zassert_equal(si4467_set_property_pwr_lvl(&ctx, 0x8f), -EINVAL,
		      "-EINVAL expected");

	zassert_ok(si4467_get_property_pwr_lvl(&ctx, &power_level),
		   "No failure expected");
	zassert_equal(power_level, 0x7f, "Unchanged power level expected");
}

void test_si4467_hw_calibration(void)
{
	uint8_t tune_value;

	zassert_equal(si4467_command_set_xo_tune_value(&ctx, 130), -EINVAL,
		      "tune value outside valid range");

	zassert_ok(si4467_command_set_xo_tune_value(&ctx, 10),
		      "set valid tune value");

	zassert_ok(si4467_save_calibration(&ctx), "store tune value");

	zassert_ok(si4467_command_set_xo_tune_value(&ctx, 20),
		      "set valid tune value");

	zassert_ok(si4467_command_get_xo_tune_value(&ctx, &tune_value),
		   "tune value readout OK");
	zassert_equal(tune_value, 20, "correct tune value read");

	zassert_ok(si4467_restore_calibration(&ctx),
		   "restore previous tune value");

	zassert_ok(si4467_command_get_xo_tune_value(&ctx, &tune_value),
		   "tune value readout OK");
	zassert_equal(tune_value, 10, "previous tune value correctly restored");
}

void test_si4467_hw_patch(void)
{
	struct func_information info;

	zassert_ok(si4467_command_func_info(&ctx, &info), "get func_info");

	zassert_equal(info.patch, 0xD046, "correct patch applied");
}
