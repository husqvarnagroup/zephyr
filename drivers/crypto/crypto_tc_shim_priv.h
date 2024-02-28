/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief tinycrypt driver context info
 *
 * The file defines the structure which is used to store per session context
 * by the driver. Placed in common location so that crypto applications
 * can allocate memory for the required number of sessions, to free driver
 * from dynamic memory allocation.
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_TC_SHIM_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_TC_SHIM_PRIV_H_

#include <stdbool.h>

#include <tinycrypt/aes.h>

struct tc_shim_drv_state {
	bool in_use;

	/**
	 * True if counter argument got initialized using the initialization vector.
	 *
	 * Warning: Counters can not be preserved across crypto session!
	 */
	bool ctr_initialized;

	struct tc_aes_key_sched_struct session_key;
};

#endif  /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_TC_SHIM_PRIV_H_ */
