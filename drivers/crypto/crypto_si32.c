/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Design decisions:
 *  - XXX
 * Notes:
 *  - If not noted otherwise, chaper numbers refer to the SiM3U1XX/SiM3C1XX reference manual
 *    (SiM3U1xx-SiM3C1xx-RM.pdf, revision 1.0)
 */

#define DT_DRV_COMPAT silabs_si32_aes

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aes_silabs_si32);

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <SI32_AES_A_Type.h>
#include <SI32_CLKCTRL_A_Type.h>
#include <SI32_DMACTRL_A_Type.h>
#include "SI32_DMAXBAR_A_Type.h"
#include <si32_device.h>

#include <errno.h>

#define AES_KEY_SIZE   16
#define AES_BLOCK_SIZE 16

#define DMA_CHANNEL_COUNT DT_PROP(DT_INST(0, silabs_si32_dma), dma_channels)

#define DMA_CHANNEL_ID_INPUT  DT_INST_DMAS_CELL_BY_NAME(0, input, channel)
#define DMA_CHANNEL_ID_OUTPUT DT_INST_DMAS_CELL_BY_NAME(0, output, channel)
#define DMA_CHANNEL_ID_XOR    DT_INST_DMAS_CELL_BY_NAME(0, xor, channel)

BUILD_ASSERT(DMA_CHANNEL_ID_INPUT < DMA_CHANNEL_COUNT, "Too few DMA channels");
BUILD_ASSERT(DMA_CHANNEL_ID_OUTPUT < DMA_CHANNEL_COUNT, "Too few DMA channels");
BUILD_ASSERT(DMA_CHANNEL_ID_XOR < DMA_CHANNEL_COUNT, "Too few DMA channels");

struct crypto_si32_config {
	SI32_AES_A_Type *base;
	unsigned int irq;
};

struct crypto_si32_data {
	atomic_t session_in_use;
};

static const struct crypto_si32_config crypto_si32_config = {
	.base = SI32_AES_0,
};

static struct crypto_si32_data crypto_si32_data;

static void crypto_si32_dma_completed(const struct device *dev, void *user_data, uint32_t channel,
				      int status)
{
	LOG_ERR("AES0 finished work");

	return;
}

static int crypto_si32_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS);
}

static void crypto_si32_irq_error_handler(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* 12.3 Interrupts: An AES0 error interrupt can be generated whenever an input/output data
	 * FIFO overrun (DORF = 1) or underrun (DURF = 1) error occurs, or when an XOR data FIFO
	 * overrun (XORF = 1) occurs.
	 */
	if (crypto_si32_config.base->STATUS.ERRI) {
		LOG_ERR("ISR: FIFO overrun (%u), underrun (%u), XOR FIF0 overrun (%u)",
			crypto_si32_config.base->STATUS.DORF, crypto_si32_config.base->STATUS.DURF,
			crypto_si32_config.base->STATUS.XORF);
		SI32_AES_A_clear_error_interrupt(SI32_AES_0);
	}
}

/* For simplicity, the AES HW does not get turned of when not in use. */
static int crypto_si32_init(const struct device *dev)
{
	const struct crypto_si32_config *config = dev->config;

	__ASSERT(config->base == SI32_AES_0, "There is only one instance");
	(void)config;

	/* Enable clock for AES HW */
	SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_AES0);

	/* To use the AES0 module, firmware must first clear the RESET bit before initializing the
	 * registers.
	 */
	SI32_AES_A_reset_module(crypto_si32_config.base);

	__ASSERT(crypto_si32_config.base->CONTROL.RESET == 0, "Reset done");

	/* 12.3. Interrupts: The completion interrupt should only be used in conjunction
	 * with software mode (SWMDEN bit is set to 1) and not with DMA operations, where the DMA
	 * completion interrupt should be used.
	 */
	SI32_AES_A_disable_operation_complete_interrupt(crypto_si32_config.base); /* default */

	/* 12.3. Interrupts: The error interrupt should always be enabled (ERRIEN = 1), even when
	 * using the DMA with the AES module.
	 */
	SI32_AES_A_enable_error_interrupt(crypto_si32_config.base);

	/* Install error handler */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), crypto_si32_irq_error_handler,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	/* Halt AES0 module on debug breakpoint */
	SI32_AES_A_enable_stall_in_debug_mode(crypto_si32_config.base);

	/* For peripheral transfers, firmware should configure the peripheral for the DMA transfer
	 * and set the device’s DMA crossbar (DMAXBAR) to map a DMA channel to the peripheral.
	 */
	SI32_DMAXBAR_A_select_channel_peripheral(SI32_DMAXBAR_0, SI32_DMAXBAR_CHAN5_AES0_TX);
	SI32_DMAXBAR_A_select_channel_peripheral(SI32_DMAXBAR_0, SI32_DMAXBAR_CHAN6_AES0_RX);

	return 0;
}

static void assert_dma_settings_common(struct SI32_DMADESC_A_Struct *channel_descriptor)
{
	__ASSERT(channel_descriptor->CONFIG.SRCSIZE == 2,
		 "Source size (SRCSIZE) and destination size (DSTSIZE) are 2 for a word transfer.");
	__ASSERT(channel_descriptor->CONFIG.DSTSIZE == 2,
		 "Source size (SRCSIZE) and destination size (DSTSIZE) are 2 for a word transfer.");
	__ASSERT(channel_descriptor->CONFIG.RPOWER == 2,
		 "RPOWER = 2 (4 data transfers per transaction).");
}

static void assert_dma_settings_channel_input(struct SI32_DMADESC_A_Struct *channel_descriptor,
					      struct cipher_pkt *pkt)
{
	assert_dma_settings_common(channel_descriptor);

	__ASSERT(channel_descriptor->CONFIG.SRCSIZE == 2,
		 "Source size (SRCSIZE) and destination size (DSTSIZE) are 2 for a word transfer.");
	__ASSERT(channel_descriptor->CONFIG.DSTSIZE == 2,
		 "Source size (SRCSIZE) and destination size (DSTSIZE) are 2 for a word transfer.");
	__ASSERT(channel_descriptor->CONFIG.RPOWER == 2,
		 "RPOWER = 2 (4 data transfers per transaction).");
	__ASSERT(channel_descriptor->DSTEND.U32 == (uintptr_t)&crypto_si32_config.base->DATAFIFO,
		 "Destination end pointer set to the DATAFIFO register.");
	__ASSERT(channel_descriptor->SRCEND.U32 == (uintptr_t)pkt->in_buf + pkt->in_len - 4,
		 "Source end pointer set to the plain or cipher text input buffer address "
		 "location + 16 x N – 4, where N is the number of blocks.");
	__ASSERT(channel_descriptor->CONFIG.DSTAIMD == 0b11,
		 "The DSTAIMD field should be set to 011b for no increment.");
	__ASSERT(channel_descriptor->CONFIG.SRCAIMD == 0b10,
		 "The SRCAIMD field should be set to 010b for word increments.");
}

static void assert_dma_settings_channel_output(struct SI32_DMADESC_A_Struct *channel_descriptor,
					       struct cipher_pkt *pkt)
{
	assert_dma_settings_common(channel_descriptor);

	__ASSERT(channel_descriptor->DSTEND.U32 == (uintptr_t)pkt->out_buf + pkt->in_len - 4,
		 "Destination end pointer set to the plain or cipher text output buffer address "
		 "location + 16 x N – 4, where N is the number of blocks.");
	__ASSERT(channel_descriptor->SRCEND.U32 == (uintptr_t)&crypto_si32_config.base->DATAFIFO,
		 "Source end pointer set to the DATAFIFO register.");
	__ASSERT(channel_descriptor->CONFIG.DSTAIMD == 0b10,
		 "The DSTAIMD field should be set to 010b for word increments.");
	__ASSERT(channel_descriptor->CONFIG.SRCAIMD == 0b11,
		 "The SRCAIMD field should be set to 011b for no increment.");
}

static int crypto_si32_aes_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct dma_block_config dma_block_input = {0};
	struct dma_block_config dma_block_output = {0};
	const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma));
	int ret;

	if (!pkt->in_len) {
		LOG_WRN("Zero sized data");
		return 0;
	}

	if (pkt->in_len % 16) {
		LOG_ERR("Data size must be 4-word aligned");
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("Output buf too small");
		return -EINVAL;
	}

	if (ctx->keylen != 16) {
		LOG_ERR("Only AES-128 implemented");
		return -ENOSYS;
	}

	/* Set up input DMA channel */
	dma_block_input.block_size = pkt->in_len;
	dma_block_input.source_address = (uintptr_t)pkt->in_buf;
	dma_block_input.source_addr_adj = 0b00; /* increment */
	dma_block_input.dest_address = (uintptr_t)&crypto_si32_config.base->DATAFIFO;
	dma_block_input.dest_addr_adj = 0b10; /* no change */

	struct dma_config dma_config_input = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 4, /* SiM3x1xx limitation: must match dest_data_size */
		.dest_data_size = 4,   /* DATAFIFO must be written to in word chunks (4 bytes) */
		.source_burst_length = 16,
		.dest_burst_length = 16,
		.block_count = 1,
		.head_block = &dma_block_input,
		.dma_callback = crypto_si32_dma_completed,
	};

	ret = dma_config(dma, DMA_CHANNEL_ID_INPUT, &dma_config_input);
	if (ret) {
		LOG_ERR("Input DMA channel setup failed: %d", ret);
		return ret;
	}

	/* Set up output DMA channel */
	dma_block_output.block_size = pkt->in_len;
	dma_block_output.source_address = (uintptr_t)&crypto_si32_config.base->DATAFIFO;
	dma_block_output.source_addr_adj = 0b10; /* no change */
	dma_block_output.dest_address = (uintptr_t)pkt->out_buf;
	dma_block_output.dest_addr_adj = 0b00; /* increment */

	struct dma_config dma_config_output = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 4, /* DATAFIFO must be read from in word chunks (4 bytes) */
		.dest_data_size = 4,   /* SiM3x1xx limitation: must match source_data_size */
		.source_burst_length = 16,
		.dest_burst_length = 16,
		.block_count = 1,
		.head_block = &dma_block_output,
		.dma_callback = crypto_si32_dma_completed,
	};

	ret = dma_config(dma, DMA_CHANNEL_ID_OUTPUT, &dma_config_output);
	if (ret) {
		LOG_ERR("Output DMA channel setup failed: %d", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_ASSERT)) {
		struct SI32_DMADESC_A_Struct *d =
			(struct SI32_DMADESC_A_Struct *)SI32_DMACTRL_0->BASEPTR.U32;

		/* As per 12.5.2. General DMA Transfer Setup, check input and output channel
		 * programming */
		assert_dma_settings_channel_input(d + DMA_CHANNEL_ID_INPUT, pkt);
		assert_dma_settings_channel_output(d + DMA_CHANNEL_ID_OUTPUT, pkt);

		/* Other checks */
		__ASSERT(SI32_DMACTRL_A_is_channel_enabled(SI32_DMACTRL_0, DMA_CHANNEL_ID_INPUT),
			 "The channel request mask (CHREQMCLR) must be cleared for the channel to "
			 "use "
			 "peripheral transfers.");

		__ASSERT(SI32_DMACTRL_A_is_channel_enabled(SI32_DMACTRL_0, DMA_CHANNEL_ID_OUTPUT),
			 "The channel request mask (CHREQMCLR) must be cleared for the channel to "
			 "use "
			 "peripheral transfers.");

		__ASSERT(SI32_DMAXBAR_0->DMAXBAR0.CH5SEL == 0b0001,
			 "0001: Service AES0 TX data requests.");
		__ASSERT(SI32_DMAXBAR_0->DMAXBAR0.CH6SEL == 0b0001,
			 "0001: Service AES0 RX data requests.");

		__ASSERT(crypto_si32_config.base->CONTROL.RESET == 0,
			 "Reset done during init, completed by now");
	}

	ret = dma_start(dma, DMA_CHANNEL_ID_INPUT);
	if (ret) {
		LOG_ERR("Input DMA channel start failed: %d", ret);
		return ret;
	}

	ret = dma_start(dma, DMA_CHANNEL_ID_OUTPUT);
	if (ret) {
		LOG_ERR("Output DMA channel start failed: %d", ret);
		return ret;
	}

	/* 1. The XFRSIZE register should be set to N-1, where N is the number of 4-word blocks. */
	SI32_AES_A_write_xfrsize(crypto_si32_config.base, pkt->in_len / AES_BLOCK_SIZE - 1);

	/* 2. The HWKEYx registers should be written with the desired key in little endian format.
	 */
	crypto_si32_config.base->HWKEY0.U32 = *((uint32_t *)ctx->key.bit_stream);
	crypto_si32_config.base->HWKEY1.U32 = *((uint32_t *)ctx->key.bit_stream + 1);
	crypto_si32_config.base->HWKEY2.U32 = *((uint32_t *)ctx->key.bit_stream + 2);
	crypto_si32_config.base->HWKEY3.U32 = *((uint32_t *)ctx->key.bit_stream + 3);

	/* 3. The CONTROL register should be set as follows: */
	{
		__ASSERT(crypto_si32_config.base->CONTROL.ERRIEN == 1, "a. ERRIEN set to 1.");
		/* b. KEYSIZE set to the appropriate number of bits for the key. */
		SI32_AES_A_select_key_size_128(crypto_si32_config.base);
		/* c. EDMD set to 1 for encryption. */
		SI32_AES_A_select_encryption_mode(crypto_si32_config.base);
		/* d. KEYCPEN set to 1 to enable key capture at the end of the transaction. */
		SI32_AES_A_enable_key_capture(crypto_si32_config.base);
		/* e. The HCBCEN, HCTREN, XOREN, BEN, SWMDEN bits should all be cleared to 0. */
		SI32_AES_A_exit_cipher_block_chaining_mode(crypto_si32_config.base);
		SI32_AES_A_exit_counter_mode(crypto_si32_config.base);
		SI32_AES_A_select_xor_path_none(crypto_si32_config.base);
		SI32_AES_A_exit_bypass_hardware_mode(crypto_si32_config.base);
		SI32_AES_A_select_dma_mode(crypto_si32_config.base);
	}

	/* Once the DMA and AES settings have been set, the transfer should be started by writing 1
	 * to the XFRSTA bit.
	 */
	SI32_AES_A_clear_operation_complete_interrupt(crypto_si32_config.base);
	SI32_AES_A_start_operation(crypto_si32_config.base);

	return 0;
}

static int crypto_si32_aes_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
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
		return -ENOSYS;
	}

	if (pkt->out_buf_max < 16) {
		LOG_ERR("Output buf too small");
		return -EINVAL;
	}

	// __ASSERT(crypto_si32_config.base->CONTROL.ERRIEN == 1, "Error interrupt enabled");

	// /* As per "12.10. Using the AES0 Module in Software Mode" */

	// /* 1. The RESET bit must be cleared to access the AES registers. */
	// __ASSERT(crypto_si32_config.base->CONTROL.RESET == 0,
	// 	 "Reset done during init, completed by now");
	// /* 2. Configure the operation, including setting SWMDEN to 1. */
	// __ASSERT(crypto_si32_config.base->CONTROL.SWMDEN, "SW mode enabled during init");
	// __ASSERT(crypto_si32_config.base->CONTROL.KEYSIZE == 00, "Keysize configured during
	// init"); SI32_AES_A_select_decryption_mode(crypto_si32_config.base);
	// SI32_AES_A_write_xfrsize(crypto_si32_config.base, 3);
	// // crypto_si32_config.base->HWKEY0.U32 = *((uint32_t *)ctx->key.bit_stream);
	// // crypto_si32_config.base->HWKEY1.U32 = *((uint32_t *)ctx->key.bit_stream + 1);
	// // crypto_si32_config.base->HWKEY2.U32 = *((uint32_t *)ctx->key.bit_stream + 2);
	// // crypto_si32_config.base->HWKEY3.U32 = *((uint32_t *)ctx->key.bit_stream + 3);
	// SI32_AES_A_exit_cipher_block_chaining_mode(crypto_si32_config.base);
	// SI32_AES_A_exit_counter_mode(crypto_si32_config.base);
	// SI32_AES_A_select_xor_path_none(crypto_si32_config.base);
	// SI32_AES_A_exit_bypass_hardware_mode(crypto_si32_config.base);
	// /* 3. Load the input/output data FIFO (DATAFIFO) with four words. */
	// __ASSERT(crypto_si32_config.base->STATUS.DFIFOLVL == 0, "Data FIFO empty");
	// crypto_si32_config.base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf);
	// crypto_si32_config.base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 1);
	// crypto_si32_config.base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 2);
	// crypto_si32_config.base->DATAFIFO.U32 = *((uint32_t *)pkt->in_buf + 3);
	// __ASSERT(crypto_si32_config.base->STATUS.DFIFOLVL == 4, "Data FIFO full");
	// /* 4. Load the XOR data FIFO (XORFIFO) with four words if XOREN is set to 01b or 10b. */
	// __ASSERT(crypto_si32_config.base->CONTROL.XOREN == 0b00, "XOR disabled during init");
	// /* 5. Set KEYCPEN to 1 if key capture is required (EDMD must also be set to 1 for the key
	//  *    capture to occur)
	//  */
	// SI32_AES_A_disable_key_capture(crypto_si32_config.base);
	// /* 6. Enable the operation complete interrupt by setting OCIEN to 1. Alternatively,
	// firmware
	//  *    can poll XFRSTA or BUSYF. */
	// SI32_AES_A_disable_operation_complete_interrupt(crypto_si32_config.base);
	// /* 7. Set XFRSTA to 1 to start the AES operation on the 4-word block. */
	// SI32_AES_A_start_operation(crypto_si32_config.base);

	// /* 9. Read the input/output data FIFO (DATAFIFO) with four word reads to obtain the
	//  * resulting cipher text output.
	//  */
	// *((uint32_t *)pkt->out_buf) = crypto_si32_config.base->DATAFIFO.U32;
	// *((uint32_t *)pkt->out_buf + 1) = crypto_si32_config.base->DATAFIFO.U32;
	// *((uint32_t *)pkt->out_buf + 2) = crypto_si32_config.base->DATAFIFO.U32;
	// *((uint32_t *)pkt->out_buf + 3) = crypto_si32_config.base->DATAFIFO.U32;
	// pkt->out_len = 16;

	// __ASSERT(crypto_si32_config.base->STATUS.DFIFOLVL == 0, "Data FIFO empty again");

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

	if (ctx->keylen != AES_KEY_SIZE) {
		LOG_ERR("Invalid key len: %" PRIu16, ctx->keylen);
		return -EINVAL;
	}

	if (ctx->keylen != AES_KEY_SIZE) {
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

DEVICE_DT_INST_DEFINE(0, crypto_si32_init, NULL, &crypto_si32_data, &crypto_si32_config,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &crypto_si32_api);
