/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si32_aes

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aes_silabs_si32);

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>

#include <errno.h>
#include <zephyr/sys/atomic.h>

#define ECB_AES_KEY_SIZE 16

struct crypto_si32_data {
	struct k_sem device_sem;
	struct k_sem session_sem;
};

static atomic_t crypto_si32_session_in_use;

static int crypto_si32_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS);
}

static int crypto_si32_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int crypto_si32_aes_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	return -ENOTSUP;
}

static int crypto_si32_aes_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	return -ENOTSUP;
}

static int crypto_si32_begin_session(const struct device *dev, struct cipher_ctx *ctx,
				     const enum cipher_algo algo, const enum cipher_mode mode,
				     const enum cipher_op op_type)
{
	ARG_UNUSED(dev);

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		LOG_ERR("This driver supports only AES");
		return -ENOTSUP;
	}

	if (!(ctx->flags & CAP_SYNC_OPS)) {
		LOG_ERR("This driver supports only synchronous mode");
		return -ENOTSUP;
	}

	if (ctx->keylen != ECB_AES_KEY_SIZE) {
		LOG_ERR("Invalid key len: %" PRIu16, ctx->keylen);
		return -EINVAL;
	}

	if (ctx->keylen != ECB_AES_KEY_SIZE) {
		LOG_ERR("Invalid key len: %" PRIu16, ctx->keylen);
		return -EINVAL;
	}

	if (mode != CRYPTO_CIPHER_MODE_ECB) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}

	if (ctx->key.bit_stream == NULL) {
		LOG_ERR("No key provided");
		return -EINVAL;
	}

	if (!atomic_cas(&crypto_si32_session_in_use, 0, 1)) {
		LOG_ERR("All session(s) in use");
		return -EBUSY;
	}

	switch (op_type) {
	case CRYPTO_CIPHER_OP_ENCRYPT:
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = crypto_si32_aes_ecb_encrypt;
			return 0;
		default:
			break;
		}
	case CRYPTO_CIPHER_OP_DECRYPT:
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = crypto_si32_aes_ecb_decrypt;
			return 0;
		default:
			break;
		}
	default:
		break;
	}

	return -ENOTSUP;
}

static int crypto_si32_free_session(const struct device *dev, struct cipher_ctx *ctx)
{
	ARG_UNUSED(dev);

	if (!atomic_cas(&crypto_si32_session_in_use, 1, 0)) {
		LOG_ERR("Session not in use");
		return -EINVAL;
	}

	return 0;
}

/* AES only, no support for hashing */
static const struct crypto_driver_api crypto_si32_api = {
	.cipher_begin_session = crypto_si32_begin_session,
	.cipher_free_session = crypto_si32_free_session,
	.query_hw_caps = crypto_si32_query_hw_caps,
};

DEVICE_DT_INST_DEFINE(0, crypto_si32_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_CRYPTO_INIT_PRIORITY, &crypto_si32_api);
