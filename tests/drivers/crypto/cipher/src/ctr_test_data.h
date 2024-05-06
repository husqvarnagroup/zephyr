/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Used seed: 0
 */

#pragma once

#include <stdint.h>

enum {
	CTR_TEST_DATA_LEN = 4096,
};

struct ctr_test_data {
	uint8_t plaintext[CTR_TEST_DATA_LEN];
	uint8_t ciphertext[CTR_TEST_DATA_LEN];
	uint8_t nonce[12];
	uint8_t key[16];
	uint32_t ctr_init;
};

extern const struct ctr_test_data ctr_test_data;
