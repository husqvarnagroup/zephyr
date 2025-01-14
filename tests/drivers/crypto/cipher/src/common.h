/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Used seed: 0
 */

#pragma once

#ifdef CONFIG_CRYPTO_SI32
#define CRYPTO_DEV_COMPAT silabs_si32_aes
#elif CONFIG_CRYPTO_NRF52_CC310_AES
#define CRYPTO_DEV_COMPAT nordic_nrf52_cc310_aes
#elif CONFIG_CRYPTO_TINYCRYPT_SHIM
#define CRYPTO_DRV_NAME CONFIG_CRYPTO_TINYCRYPT_SHIM_DRV_NAME
#elif CONFIG_CRYPTO_MBEDTLS_SHIM
#define CRYPTO_DRV_NAME CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#else
#error "You need to enable one crypto device"
#endif
