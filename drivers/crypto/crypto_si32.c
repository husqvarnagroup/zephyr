/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Design decisions:
 *  - For DMA operations, the hardware requires 4 word aligned starting addresses of the input
 *    buffers. As this module has no control over its inputs, and copying data into aligned memory
 *    would require (potentially too) big buffers, this drivers uses the software mode (see
 *    chapter 12.10 in the reference manual).
 */

#define DT_DRV_COMPAT silabs_si32_aes

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aes_silabs_si32);

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <SI32_AES_A_Type.h>
#include <SI32_CLKCTRL_A_Type.h>
#include <si32_device.h>

#include <errno.h>
#include <zephyr/sys/atomic.h>

#define ECB_AES_KEY_SIZE 16

struct crypto_si32_config {
	SI32_AES_A_Type *base;
};

struct crypto_si32_data {
	atomic_t session_in_use;
};

static const struct crypto_si32_config crypto_si32_config_0 = {
	.base = SI32_AES_0,
};

static struct crypto_si32_data crypto_si32_data_0;

static int crypto_si32_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS);
}

static void crypto_si32_irq_handler(const struct device *dev)
{
	LOG_ERR("ISR FIRED!");
}

/* For simplicity, the AES HW does not get turned of when not in use. */
static int crypto_si32_init(const struct device *dev)
{
	const struct crypto_si32_config *config = dev->config;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), crypto_si32_irq_handler,
		    DEVICE_DT_INST_GET(0), 0);

	/* Enable clock for AES HW */
	SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_AES0);

	/* Reset to ensure a clean state */
	SI32_AES_A_reset_module(config->base);

	/* Configure some settings valid for all (supported) operation modes */
	/* 1) We support only AES-128 */
	SI32_AES_A_select_key_size_128(config->base);
	/* 2) XXX */
	SI32_AES_A_exit_bypass_hardware_mode(config->base);

	/* We use software mode */
	SI32_AES_A_select_software_mode(config->base);

	/* XXX */
	SI32_AES_A_select_xor_path_none(config->base);

	SI32_AES_A_enable_error_interrupt(config->base);

	return 0;
}
static int crypto_si32_aes_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	const struct crypto_si32_config *config = ctx->device->config;

	if (!pkt->in_len) {
		LOG_WRN("Zero sized data");
		return 0;
	}

	if (pkt->in_len % 16) {
		LOG_ERR("Data size must be 4-word aligned");
		return -EINVAL;
	}

	/* For security reasons, ECB mode should not be used to encrypt more than one block. Use CBC
	 * mode instead.
	 */
	if (pkt->in_len > 16) {
		LOG_ERR("Can't encrypt more than 1 block");
		return -EINVAL;
	}

	if (pkt->out_buf_max < 16) {
		LOG_ERR("Output buf too small");
		return -EINVAL;
	}

	__ASSERT(config->base->CONTROL.ERRIEN == 1, "Error interrupt enabled");

	/* As per "12.10. Using the AES0 Module in Software Mode" */

	/* 1. The RESET bit must be cleared to access the AES registers. */
	__ASSERT(config->base->CONTROL.RESET == 0, "Reset done during init, completed by now");
	/* 2. Configure the operation, including setting SWMDEN to 1. */
	__ASSERT(config->base->CONTROL.SWMDEN, "SW mode enabled during init");
	__ASSERT(config->base->CONTROL.KEYSIZE == 00, "Keysize configured during init");
	SI32_AES_A_select_encryption_mode(config->base);
	SI32_AES_A_write_xfrsize(config->base, 3);
	config->base->HWKEY0.U32 = *((uint32_t *)ctx->key.bit_stream);
	config->base->HWKEY1.U32 = *((uint32_t *)ctx->key.bit_stream + 1);
	config->base->HWKEY2.U32 = *((uint32_t *)ctx->key.bit_stream + 2);
	config->base->HWKEY3.U32 = *((uint32_t *)ctx->key.bit_stream + 3);
	SI32_AES_A_exit_cipher_block_chaining_mode(config->base);
	SI32_AES_A_exit_counter_mode(config->base);
	SI32_AES_A_select_xor_path_none(config->base);
	SI32_AES_A_exit_bypass_hardware_mode(config->base);
	/* 3. Load the input/output data FIFO (DATAFIFO) with four words. */
	__ASSERT(config->base->STATUS.DFIFOLVL == 0, "Data FIFO empty");
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 1);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 2);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 3);
	__ASSERT(config->base->STATUS.DFIFOLVL == 4, "Data FIFO full");
	/* 4. Load the XOR data FIFO (XORFIFO) with four words if XOREN is set to 01b or 10b. */
	__ASSERT(config->base->CONTROL.XOREN == 0b00, "XOR disabled during init");
	/* 5. Set KEYCPEN to 1 if key capture is required (EDMD must also be set to 1 for the key
	 *    capture to occur)
	 */
	SI32_AES_A_enable_key_capture(config->base);
	/* 6. Enable the operation complete interrupt by setting OCIEN to 1. Alternatively, firmware
	 *    can poll XFRSTA or BUSYF. */
	SI32_AES_A_disable_operation_complete_interrupt(config->base);
	/* 7. Set XFRSTA to 1 to start the AES operation on the 4-word block. */
	SI32_AES_A_start_operation(config->base);
	/* 8. Wait for the completion interrupt or poll until the operation completes. */
	while (config->base->CONTROL.XFRSTA) {
		/* This bit is automatically cleared when the XFRSIZE register is 0 and the current
		 * operation completes.
		 */
	}
	/* 9. Read the input/output data FIFO (DATAFIFO) with four word reads to obtain the
	 * resulting cipher text output.
	 */
	*((uint32_t *)pkt->out_buf) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 1) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 2) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 3) = config->base->DATAFIFO.U32;
	pkt->out_len = 16;

	__ASSERT(config->base->STATUS.DFIFOLVL == 0, "Data FIFO empty again");

	return 0;
}

static int crypto_si32_aes_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	const struct crypto_si32_config *config = ctx->device->config;

	if (!pkt->in_len) {
		LOG_WRN("Zero sized data");
		return 0;
	}

	if (pkt->in_len % 16) {
		LOG_ERR("Data size must be 4-word aligned");
		return -EINVAL;
	}

	/* For security reasons, ECB mode should not be used to decrypt more than one block. Use CBC
	 * mode instead.
	 */
	if (pkt->in_len > 16) {
		LOG_ERR("Can't decrypt more than 1 block");
		return -EINVAL;
	}

	if (pkt->out_buf_max < 16) {
		LOG_ERR("Output buf too small");
		return -EINVAL;
	}

	__ASSERT(config->base->CONTROL.ERRIEN == 1, "Error interrupt enabled");

	/* As per "12.10. Using the AES0 Module in Software Mode" */

	/* 1. The RESET bit must be cleared to access the AES registers. */
	__ASSERT(config->base->CONTROL.RESET == 0, "Reset done during init, completed by now");
	/* 2. Configure the operation, including setting SWMDEN to 1. */
	__ASSERT(config->base->CONTROL.SWMDEN, "SW mode enabled during init");
	__ASSERT(config->base->CONTROL.KEYSIZE == 00, "Keysize configured during init");
	SI32_AES_A_select_decryption_mode(config->base);
	SI32_AES_A_write_xfrsize(config->base, 3);
	// config->base->HWKEY0.U32 = *((uint32_t *)ctx->key.bit_stream);
	// config->base->HWKEY1.U32 = *((uint32_t *)ctx->key.bit_stream + 1);
	// config->base->HWKEY2.U32 = *((uint32_t *)ctx->key.bit_stream + 2);
	// config->base->HWKEY3.U32 = *((uint32_t *)ctx->key.bit_stream + 3);
	SI32_AES_A_exit_cipher_block_chaining_mode(config->base);
	SI32_AES_A_exit_counter_mode(config->base);
	SI32_AES_A_select_xor_path_none(config->base);
	SI32_AES_A_exit_bypass_hardware_mode(config->base);
	/* 3. Load the input/output data FIFO (DATAFIFO) with four words. */
	__ASSERT(config->base->STATUS.DFIFOLVL == 0, "Data FIFO empty");
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 1);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 2);
	config->base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 3);
	__ASSERT(config->base->STATUS.DFIFOLVL == 4, "Data FIFO full");
	/* 4. Load the XOR data FIFO (XORFIFO) with four words if XOREN is set to 01b or 10b. */
	__ASSERT(config->base->CONTROL.XOREN == 0b00, "XOR disabled during init");
	/* 5. Set KEYCPEN to 1 if key capture is required (EDMD must also be set to 1 for the key
	 *    capture to occur)
	 */
	SI32_AES_A_disable_key_capture(config->base);
	/* 6. Enable the operation complete interrupt by setting OCIEN to 1. Alternatively, firmware
	 *    can poll XFRSTA or BUSYF. */
	SI32_AES_A_disable_operation_complete_interrupt(config->base);
	/* 7. Set XFRSTA to 1 to start the AES operation on the 4-word block. */
	SI32_AES_A_start_operation(config->base);
	/* 8. Wait for the completion interrupt or poll until the operation completes. */
	while (config->base->CONTROL.XFRSTA) {
		/* This bit is automatically cleared when the XFRSIZE register is 0 and the current
		 * operation completes.
		 */
	}
	/* 9. Read the input/output data FIFO (DATAFIFO) with four word reads to obtain the
	 * resulting cipher text output.
	 */
	*((uint32_t *)pkt->out_buf) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 1) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 2) = config->base->DATAFIFO.U32;
	*((uint32_t *)pkt->out_buf + 3) = config->base->DATAFIFO.U32;
	pkt->out_len = 16;

	__ASSERT(config->base->STATUS.DFIFOLVL == 0, "Data FIFO empty again");

	return 0;
}

static int crypto_si32_begin_session(const struct device *dev, struct cipher_ctx *ctx,
				     const enum cipher_algo algo, const enum cipher_mode mode,
				     const enum cipher_op op_type)
{
	struct crypto_si32_data *data = dev->data;

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

	if (!atomic_cas(&data->session_in_use, 0, 1)) {
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
	ARG_UNUSED(ctx);

	struct crypto_si32_data *data = dev->data;

	if (!atomic_cas(&data->session_in_use, 1, 0)) {
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

DEVICE_DT_INST_DEFINE(0, crypto_si32_init, NULL, &crypto_si32_data_0, &crypto_si32_config_0,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &crypto_si32_api);
