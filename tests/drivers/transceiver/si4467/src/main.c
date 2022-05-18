/*
 * Copyright (c) 2020 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_si4467_hw.h"

#include <drivers/transceiver/si4467.h>

#include <ztest.h>
#include <ztest_error_hook.h>

static void test_si4467_chip_interrupt_to_byte(void)
{
	zassert_equal(0x7F,
		      si4467_chip_interrupt_to_byte((struct chip_interrupt){
			      .cal = 1,
			      .fifo_underflow_overflow_error = 1,
			      .state_change = 1,
			      .cmd_error = 1,
			      .chip_ready = 1,
			      .low_batt = 1,
			      .wut = 1,
		      }),
		      "All possible fields set");
	zassert_equal(0,
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .unused = 1 }),
		      "Unused field has no effect");
	zassert_equal(BIT(6),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .cal = 1 }),
		      "cal bit");
	zassert_equal(BIT(5),
		      si4467_chip_interrupt_to_byte((struct chip_interrupt){
			      .fifo_underflow_overflow_error = 1 }),
		      "fifo_underflow_overflow_error bit");
	zassert_equal(BIT(4),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .state_change = 1 }),
		      "state_change bit");
	zassert_equal(BIT(3),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .cmd_error = 1 }),
		      "cmd_error bit");
	zassert_equal(BIT(2),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .chip_ready = 1 }),
		      "chip_ready bit");
	zassert_equal(BIT(1),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .low_batt = 1 }),
		      "low_batt bit");
	zassert_equal(BIT(0),
		      si4467_chip_interrupt_to_byte(
			      (struct chip_interrupt){ .wut = 1 }),
		      "wut bit");
	zassert_equal(
		0x00,
		si4467_chip_interrupt_to_byte((struct chip_interrupt){ 0 }),
		"No fields set");
}

static void test_si4467_modem_interrupt_to_byte(void)
{
	zassert_equal(0xFF,
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .rssi_latch = 1,
							.postamble_detect = 1,
							.invalid_sync = 1,
							.rssi_jump = 1,
							.rssi = 1,
							.invalid_preamble = 1,
							.preamble_detect = 1,
							.sync_detect = 1 }),
		      "All fields set");
	zassert_equal(BIT(7),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .rssi_latch = 1 }),
		      "rssi_latch bit");
	zassert_equal(BIT(6),
		      si4467_modem_interrupt_to_byte((struct modem_interrupt){
			      .postamble_detect = 1 }),
		      "postamble_detect bit");
	zassert_equal(BIT(5),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .invalid_sync = 1 }),
		      "invalid_sync bit");
	zassert_equal(BIT(4),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .rssi_jump = 1 }),
		      "rssi_jump bit");
	zassert_equal(BIT(3),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .rssi = 1 }),
		      "rssi bit");
	zassert_equal(BIT(2),
		      si4467_modem_interrupt_to_byte((struct modem_interrupt){
			      .invalid_preamble = 1 }),
		      "invalid_preamble bit");
	zassert_equal(BIT(1),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .preamble_detect = 1 }),
		      "preamble_detect bit");
	zassert_equal(BIT(0),
		      si4467_modem_interrupt_to_byte(
			      (struct modem_interrupt){ .sync_detect = 1 }),
		      "sync_detect bit");
	zassert_equal(
		0x00,
		si4467_modem_interrupt_to_byte((struct modem_interrupt){ 0 }),
		"No fields set");
}

static void test_si4467_packet_handler_interrupt_to_byte(void)
{
	zassert_equal(0xFF,
		      si4467_packet_handler_interrupt_to_byte(
			      (struct packet_handler_interrupt){
				      .filter_match = 1,
				      .filter_miss = 1,
				      .packet_sent = 1,
				      .packet_rx = 1,
				      .crc_error = 1,
				      .alt_crc_error = 1,
				      .tx_fifo_almost_empty = 1,
				      .rx_fifo_almost_full = 1 }),
		      "All fields set");
	zassert_equal(
		BIT(7),
		si4467_packet_handler_interrupt_to_byte(
			(struct packet_handler_interrupt){ .filter_match = 1 }),
		"filter_match bit");
	zassert_equal(
		BIT(6),
		si4467_packet_handler_interrupt_to_byte(
			(struct packet_handler_interrupt){ .filter_miss = 1 }),
		"filter_miss bit");
	zassert_equal(
		BIT(5),
		si4467_packet_handler_interrupt_to_byte(
			(struct packet_handler_interrupt){ .packet_sent = 1 }),
		"packet_sent bit");
	zassert_equal(
		BIT(4),
		si4467_packet_handler_interrupt_to_byte(
			(struct packet_handler_interrupt){ .packet_rx = 1 }),
		"packet_rx bit");
	zassert_equal(
		BIT(3),
		si4467_packet_handler_interrupt_to_byte(
			(struct packet_handler_interrupt){ .crc_error = 1 }),
		"crc_error bit");
	zassert_equal(
		BIT(2),
		si4467_packet_handler_interrupt_to_byte((
			struct packet_handler_interrupt){ .alt_crc_error = 1 }),
		"alt_crc_error bit");
	zassert_equal(BIT(1),
		      si4467_packet_handler_interrupt_to_byte(
			      (struct packet_handler_interrupt){
				      .tx_fifo_almost_empty = 1 }),
		      "tx_fifo_almost_empty bit");
	zassert_equal(BIT(0),
		      si4467_packet_handler_interrupt_to_byte(
			      (struct packet_handler_interrupt){
				      .rx_fifo_almost_full = 1 }),
		      "rx_fifo_almost_full bit");
	zassert_equal(0x00,
		      si4467_packet_handler_interrupt_to_byte(
			      (struct packet_handler_interrupt){ 0 }),
		      "No fields set");
}

static void test_si4467_is_valid_next_operating_state(void)
{
	zassert_true(si4467_is_valid_next_operating_state(0), "No change");
	zassert_true(si4467_is_valid_next_operating_state(1), "SLEEP");
	zassert_true(si4467_is_valid_next_operating_state(2), "SPI_ACTIVE");
	zassert_true(si4467_is_valid_next_operating_state(3), "READY");
	zassert_false(si4467_is_valid_next_operating_state(4), "Invalid");
	zassert_true(si4467_is_valid_next_operating_state(5), "TX_TUNE");
	zassert_true(si4467_is_valid_next_operating_state(6), "RX_TUNE");
	zassert_true(si4467_is_valid_next_operating_state(7), "TX");
	zassert_true(si4467_is_valid_next_operating_state(8), "RX");
	zassert_false(si4467_is_valid_next_operating_state(9), "Invalid");
	/* [10,254] also invalid */
	zassert_false(si4467_is_valid_next_operating_state(255), "Invalid");
}

static void test_si4467_modem_interrupt_present(void)
{
	struct modem_interrupt i = {};

	/* No flags */
	zassert_false(si4467_modem_interrupt_present(i), "No flags");

	/* Single flags */
	i = (struct modem_interrupt){ .rssi_latch = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "rssi_latch set");

	i = (struct modem_interrupt){ .postamble_detect = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "postamble_detect set");

	i = (struct modem_interrupt){ .invalid_sync = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "invalid_sync set");

	i = (struct modem_interrupt){ .rssi_jump = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "rssi_jump set");

	i = (struct modem_interrupt){ .rssi = 1 };
	zassert_true(si4467_modem_interrupt_present(i), " rssi set");

	i = (struct modem_interrupt){ .invalid_preamble = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "invalid_preamble set");

	i = (struct modem_interrupt){ .preamble_detect = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "preamble_detect set");

	i = (struct modem_interrupt){ .sync_detect = 1 };
	zassert_true(si4467_modem_interrupt_present(i), "sync_detect set");

	/* Multiple flags */
	i = (struct modem_interrupt){ .rssi_latch = 1, .sync_detect = 1 };
	zassert_true(si4467_modem_interrupt_present(i),
		     "rssi_latch and sync_detect set");
}

static void test_si4467_chip_interrupt_present(void)
{
	struct chip_interrupt i = {};

	/* No flags */
	zassert_false(si4467_chip_interrupt_present(i), "No flags");

	i.unused = 1;
	zassert_false(si4467_chip_interrupt_present(i), "Unused flag");

	/* Single flags */
	i = (struct chip_interrupt){ .cal = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "cal set");

	i = (struct chip_interrupt){ .fifo_underflow_overflow_error = 1 };
	zassert_true(si4467_chip_interrupt_present(i),
		     "fifo_underflow_overflow_error set");

	i = (struct chip_interrupt){ .state_change = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "state_change set");

	i = (struct chip_interrupt){ .cmd_error = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "cmd_error set");

	i = (struct chip_interrupt){ .chip_ready = 1 };
	zassert_true(si4467_chip_interrupt_present(i), " chip_ready set");

	i = (struct chip_interrupt){ .low_batt = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "low_batt set");

	i = (struct chip_interrupt){ .wut = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "wut set");

	/* Multiple flags */
	i = (struct chip_interrupt){ .cal = 1, .wut = 1 };
	zassert_true(si4467_chip_interrupt_present(i), "cal and wut set");
}

static void test_si4467_packet_handler_interrupt_present(void)
{
	struct packet_handler_interrupt i = {};

	/* No flags */
	zassert_false(si4467_packet_handler_interrupt_present(i), "No flags");

	/* Single flags */
	i = (struct packet_handler_interrupt){ .filter_match = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "filter_match set");

	i = (struct packet_handler_interrupt){ .filter_miss = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "filter_miss set");

	i = (struct packet_handler_interrupt){ .packet_sent = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "packet_sent set");

	i = (struct packet_handler_interrupt){ .packet_rx = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "packet_rx set");

	i = (struct packet_handler_interrupt){ .crc_error = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     " crc_error set");

	i = (struct packet_handler_interrupt){ .alt_crc_error = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "alt_crc_error set");

	i = (struct packet_handler_interrupt){ .tx_fifo_almost_empty = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "tx_fifo_almost_empty set");

	i = (struct packet_handler_interrupt){ .rx_fifo_almost_full = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "rx_fifo_almost_full set");

	/* Multiple flags */
	i = (struct packet_handler_interrupt){ .filter_match = 1,
					       .rx_fifo_almost_full = 1 };
	zassert_true(si4467_packet_handler_interrupt_present(i),
		     "filter_match and rx_fifo_almost_full set");
}

static void test_si4467_si4467_property_group_size(void)
{
	ztest_set_assert_valid(true);
	zassert_equal(
		si4467_property_group_size((enum si4467_property_group)0xFF),
		-EINVAL, "0xFF is not a valid property group");

	ztest_set_assert_valid(false);
	zassert_equal(si4467_property_group_size(SI4467_PROPERTY_GROUP_GLOBAL),
		      11, "Property group global has 11 bytes");
}

static void test_si4467_get_xtal_frequency(void)
{
	zassert_equal(si4467_get_xtal_frequency(), 26000000,
		      "Code has only been tested with 26 MHz crystal.");
}

static void test_si4467_property_group_str(void)
{
	zassert_equal(
		strcmp(si4467_property_group_str(SI4467_PROPERTY_GROUP_RX_HOP),
		       "RX_HOP"),
		0, NULL);
}

static void test_si4467_property_modem_clkgen_band_sy_sel_str(void)
{
	const enum si4467_property_modem_clkgen_band_sy_sel p =
		SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_LOW_POWER;

	zassert_equal(strcmp(si4467_property_modem_clkgen_band_sy_sel_str(p),
			     "Low power"),
		      0, NULL);
}

static void test_si4467_property_modem_clkgen_band_band_to_outdiv(void)
{
	zassert_equal(
		si4467_property_modem_clkgen_band_band_to_outdiv(
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_4),
		4, 0, "Output is FVCO/4");

	zassert_equal(
		si4467_property_modem_clkgen_band_band_to_outdiv(
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24),
		24, 0, "Output is FVCO/24");
	zassert_equal(
		si4467_property_modem_clkgen_band_band_to_outdiv(
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_2),
		24, 0, "Output is FVCO/24");
	zassert_equal(
		si4467_property_modem_clkgen_band_band_to_outdiv(
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_3),
		24, 0, "Output is FVCO/24");
}

static void test_si4467_pa_dbm_to_pwr_lvl(void)
{
	zassert_equal(si4467_pa_dbm_to_pwr_lvl(0), 9,
		      "correct calculation of LUT low value");
	zassert_equal(si4467_pa_dbm_to_pwr_lvl(11), 60,
		      "correct calculation of LUT high value");
	zassert_equal(si4467_pa_dbm_to_pwr_lvl(12), 0x4f,
		      "correct calculation of value above LUT");
	zassert_equal(si4467_pa_dbm_to_pwr_lvl(100), 0x4f,
		      "correct calculation of value above LUT");
	zassert_equal(si4467_pa_dbm_to_pwr_lvl(255), 0x4f,
		      "correct calculation of value above LUT");
}

void test_main(void)
{
	ztest_test_suite(
		sw_tests, ztest_unit_test(test_si4467_chip_interrupt_to_byte),
		ztest_unit_test(test_si4467_modem_interrupt_to_byte),
		ztest_unit_test(test_si4467_packet_handler_interrupt_to_byte),
		ztest_unit_test(test_si4467_is_valid_next_operating_state),
		ztest_unit_test(test_si4467_modem_interrupt_present),
		ztest_unit_test(test_si4467_chip_interrupt_present),
		ztest_unit_test(test_si4467_packet_handler_interrupt_present),
		ztest_unit_test(test_si4467_si4467_property_group_size),
		ztest_unit_test(test_si4467_property_group_str),
		ztest_unit_test(test_si4467_get_xtal_frequency),
		ztest_unit_test(
			test_si4467_property_modem_clkgen_band_sy_sel_str),
		ztest_unit_test(
			test_si4467_property_modem_clkgen_band_band_to_outdiv),
		ztest_unit_test(test_si4467_pa_dbm_to_pwr_lvl));

	ztest_run_test_suite(sw_tests);

	test_si4467_hw_init();
	ztest_test_suite(
		hw_tests, ztest_unit_test(test_si4467_hw_restart),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_properties_set_and_get_one,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_properties_get_many,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_configure_rx_hopping,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_clkgen_band,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_freq_control_inte,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_freq_control_frac,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_freq_control_channel_ss,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_rx_hop_control,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_get_property_pwr_lvl,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_set_property_pwr_lvl,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_calibration,
			test_si4467_hw_setup, unit_test_noop),
		ztest_unit_test_setup_teardown(
			test_si4467_hw_patch,
			test_si4467_hw_setup, unit_test_noop));

	ztest_run_test_suite(hw_tests);
}
