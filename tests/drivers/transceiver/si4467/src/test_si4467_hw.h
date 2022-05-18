/*
 * Copyright (c) 2021 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TESTS_DRIVERS_TRANSCEIVERS_SI4467_H
#define ZEPHYR_TESTS_DRIVERS_TRANSCEIVERS_SI4467_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize hardware - call once before running any of the hardware dependent
 * tests.
 */
void test_si4467_hw_init(void);

/** Setup up function - call for each tests. */
void test_si4467_hw_setup(void);

/**
 * All tests rely on the device being able be reset during test setup phase -
 * test this first.
 */
void test_si4467_hw_restart(void);

void test_si4467_hw_properties_set_and_get_one(void);
void test_si4467_hw_properties_get_many(void);
void test_si4467_hw_configure_rx_hopping(void);
void test_si4467_hw_get_property_clkgen_band(void);

void test_si4467_hw_get_property_freq_control_inte(void);
void test_si4467_hw_get_property_freq_control_frac(void);
/* ss = step_size abbreviation */
void test_si4467_hw_get_property_freq_control_channel_ss(void);
void test_si4467_hw_get_property_rx_hop_control(void);
void test_si4467_hw_get_property_pwr_lvl(void);
void test_si4467_hw_set_property_pwr_lvl(void);
void test_si4467_hw_calibration(void);
void test_si4467_hw_patch(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_TESTS_DRIVERS_TRANSCEIVERS_SI4467_H_ */
