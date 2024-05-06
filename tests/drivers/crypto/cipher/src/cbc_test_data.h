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
	CBC_TEST_DATA_LEN = 4096,
};

struct cbc_test_data {
	uint8_t plaintext[CBC_TEST_DATA_LEN];
	uint8_t ciphertext[CBC_TEST_DATA_LEN];
	uint8_t iv[16];
	uint8_t key[16];
};

extern const struct cbc_test_data cbc_test_data;
