/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si32_dma

#include <SI32_CLKCTRL_A_Type.h>
#include <SI32_DMACTRL_A_Type.h>
#include <SI32_DMADESC_A_Type.h>
#include <SI32_SCONFIG_A_Type.h>
#include <si32_device.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_si32, CONFIG_DMA_LOG_LEVEL);

BUILD_ASSERT((uintptr_t)SI32_DMACTRL_0 == (uintptr_t)DT_INST_REG_ADDR(0),
	     "There is just one DMA controller");

#define CHANNEL_COUNT DT_INST_PROP(0, dma_channels)

struct dma_si32_channel_data {
	dma_callback_t callback;
	void *callback_user_data;
	unsigned int TMD: 3;
};

struct dma_si32_data {
	struct dma_context ctx; /* MUST BE FIRST!  See API docs */
	struct dma_si32_channel_data channel_data[CHANNEL_COUNT];
};

ATOMIC_DEFINE(dma_si32_atomic, CHANNEL_COUNT);
static struct dma_si32_data dam_si32_data = {.ctx = {
						     .magic = DMA_MAGIC,
						     .atomic = dma_si32_atomic,
						     .dma_channels = CHANNEL_COUNT,
					     }};

__aligned(SI32_DMADESC_PRI_ALIGN) struct SI32_DMADESC_A_Struct channel_descriptors[CHANNEL_COUNT];

static void dma_si32_isr_handler(uint8_t channel)
{
	const struct SI32_DMADESC_A_Struct *channel_descriptor = &channel_descriptors[channel];
	const dma_callback_t cb = dam_si32_data.channel_data[channel].callback;
	void *user_data = dam_si32_data.channel_data[channel].callback_user_data;
	int result;

	LOG_DBG("Channel %" PRIu8 " ISR fired", channel);

	if (SI32_DMACTRL_A_is_bus_error_set(SI32_DMACTRL_0)) {
		LOG_ERR("Bus error on channel %" PRIu8, channel);
		result = -EIO;
	} else {
		result = DMA_STATUS_COMPLETE;
		__ASSERT(channel_descriptor->CONFIG.TMD == 0, "Result of success: TMD set to zero");
		__ASSERT(channel_descriptor->CONFIG.NCOUNT == 0,
			 "Result of success: All blocks processed");
		(void)channel_descriptor;
		__ASSERT((SI32_DMACTRL_0->CHENSET.U32 & BIT(channel)) == 0,
			 "Result of success: Channel disabled");
	}

	if (cb) {
		cb(DEVICE_DT_INST_GET(0), user_data, channel, result);
	}
}

#define DMA_SI32_IRQ_CONNECT(channel)                                                              \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, channel, irq),                                   \
			    DT_INST_IRQ_BY_IDX(0, channel, priority), dma_si32_isr_handler,        \
			    channel, 0);                                                           \
		irq_enable(DT_INST_IRQ_BY_IDX(0, channel, irq));                                   \
	} while (false)

#define DMA_SI32_IRQ_CONNECT_GEN(i, _) DMA_SI32_IRQ_CONNECT(i);

static int dma_si32_init(const struct device *dev)
{
	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	__ASSERT(SI32_DMACTRL_0 == SI32_DMACTRL_0, "There is only one DMA controller");
	__ASSERT(SI32_DMACTRL_A_get_number_of_channels(SI32_DMACTRL_0) >= CHANNEL_COUNT,
		 "Invalid channel count");

	/* Route clock to the DMA controller */
	SI32_CLKCTRL_A_enable_ahb_to_dma_controller(SI32_CLKCTRL_0);

	/* TODO: Peripheral clock, needed when doring more than just memory transfers */

	/* Configure base address of the DMA channel descriptors */
	SI32_DMACTRL_A_write_baseptr(SI32_DMACTRL_0, (uintptr_t)channel_descriptors);

	/* Enable the DMA interface */
	SI32_DMACTRL_A_enable_module(SI32_DMACTRL_0);

	/* Primary descriptors only. This driver does do not support the more complex cases yet. */
	SI32_DMACTRL_A_write_chalt(SI32_DMACTRL_0, 0);

	/* AN666.pdf: The SCONFIG module contains a bit (FDMAEN) that enables faster DMA transfers
	 * when set to 1. It is recommended that all applications using the DMA set this bit to 1.
	 */
	SI32_SCONFIG_A_enter_fast_dma_mode(SI32_SCONFIG_0);

	/* Install handlers for all channels */
	LISTIFY(DT_NUM_IRQS(DT_DRV_INST(0)), DMA_SI32_IRQ_CONNECT_GEN, (;));

	return 0;
}

static int dma_si32_config(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	const struct dma_block_config *block;
	struct SI32_DMADESC_A_Struct *channel_descriptor;
	uint32_t ncount;

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	channel_descriptor = &channel_descriptors[channel];

	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	if (cfg == NULL) {
		LOG_ERR("Missing config");
		return -EINVAL;
	}

	if (cfg->complete_callback_en > 1) {
		LOG_ERR("Callback on each block not implemented");
		return -ENOTSUP;
	}

	if (cfg->error_callback_dis > 1) {
		LOG_ERR("Error callback disabling not implemented");
		return -ENOTSUP;
	}

	if (cfg->source_handshake > 1 || cfg->dest_handshake > 1) {
		LOG_ERR("Handshake not implemented");
		return -ENOTSUP;
	}

	if (cfg->channel_priority > 1) {
		LOG_ERR("Channel priority not implemented");
		return -ENOTSUP;
	}

	if (cfg->source_chaining_en > 1 || cfg->dest_chaining_en > 1) {
		LOG_ERR("Chaining not implemented");
		return -ENOTSUP;
	}

	if (cfg->linked_channel > 1) {
		LOG_ERR("Linked channel not implemented");
		return -ENOTSUP;
	}

	if (cfg->cyclic > 1) {
		LOG_ERR("Cyclic transfer not implemented");
		return -ENOTSUP;
	}

	if (cfg->source_data_size != 1 && cfg->source_data_size != 2 &&
	    cfg->source_data_size != 4) {
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")", cfg->source_data_size);
		return -ENOTSUP;
	}

	if (cfg->dest_data_size != 1 && cfg->dest_data_size != 2 && cfg->dest_data_size != 4) {
		LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")", cfg->dest_data_size);
		return -ENOTSUP;
	}

	__ASSERT(cfg->source_data_size == cfg->dest_data_size,
		 "The destination size (DSTSIZE) must equal the source size (SRCSIZE).");

	if (cfg->source_burst_length != cfg->dest_burst_length) {
		LOG_ERR("Individual burst modes not supported");
		return -ENOTSUP;
	}

	if (POPCOUNT(cfg->source_burst_length) > 1) {
		LOG_ERR("Burst lengths must be power of two");
		return -ENOTSUP;
	}

	if (cfg->block_count > 1) {
		LOG_ERR("Scatter-Gather not implemented");
		return -ENOTSUP;
	}

	/* Config is sane, start using it */
	dam_si32_data.channel_data[channel].callback = cfg->dma_callback;
	dam_si32_data.channel_data[channel].callback_user_data = cfg->user_data;

	channel_descriptor->CONFIG.RPOWER =
		cfg->source_burst_length ? find_msb_set(cfg->source_burst_length) - 1 : 0;

	switch (cfg->source_data_size) {
	case 4:
		channel_descriptor->CONFIG.SRCSIZE = 0b10;
		channel_descriptor->CONFIG.DSTSIZE = 0b10;
		break;
	case 2:
		channel_descriptor->CONFIG.SRCSIZE = 0b01;
		channel_descriptor->CONFIG.DSTSIZE = 0b01;
		break;
	case 1:
		channel_descriptor->CONFIG.SRCSIZE = 0b00;
		channel_descriptor->CONFIG.DSTSIZE = 0b00;
		break;
	default:
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")", cfg->source_data_size);
		return -EINVAL;
	}

	/* Configuration evaluated and extracted, except for its (first) block. Do this now. */
	if (!cfg->head_block || cfg->block_count == 0) {
		LOG_ERR("Missing head block");
		return -EINVAL;
	}

	block = cfg->head_block;

	if (block->block_size % cfg->source_data_size != 0) {
		LOG_ERR("Block size not aliged with data size");
		return -EINVAL;
	}

	ncount = block->block_size / cfg->source_data_size - 1;

	/* NCOUNT works only for values up to 1023 (1024 transfers) */
	if (ncount >= 1024) {
		LOG_ERR("Transfer size exceeded");
		return -EINVAL;
	}

	channel_descriptor->CONFIG.NCOUNT = ncount;

	/* Copy data to own location so that cfg must not exist during all of the channels usage */
	switch (cfg->channel_direction) {
	case 0b000: /* memory to memory */
		/* SiM3U1xx-SiM3C1xx-RM.pdf, 16.6.2. Auto-Request Transfers:
		 * This transfer type is recommended for memory to memory transfers.
		 */
		dam_si32_data.channel_data[channel].TMD =
			SI32_DMADESC_A_CONFIG_TMD_AUTO_REQUEST_VALUE;
		break;
	default: /* everything else is not (yet) supported */
		LOG_ERR("Channel direction not implemented: %d", cfg->channel_direction);
		return -ENOTSUP;
	}

	switch (block->source_addr_adj) {
	case 0b00: /* increment */
		channel_descriptor->SRCEND.U32 =
			block->source_address + ncount * cfg->source_data_size;
		channel_descriptor->CONFIG.SRCAIMD = channel_descriptor->CONFIG.SRCSIZE;
		break;
	case 0b01: /* decrement */
		LOG_ERR("source_addr_adj value not supported by HW");
		return -ENOTSUP;
	case 0b10: /* no change */
		channel_descriptor->SRCEND.U32 = block->source_address;
		channel_descriptor->CONFIG.SRCAIMD = 0b11;
		break;
	default:
		LOG_ERR("Unknown source_addr_adj value");
		return -EINVAL;
	}

	switch (block->dest_addr_adj) {
	case 0b00: /* increment */
		channel_descriptor->DSTEND.U32 = block->dest_address + ncount * cfg->dest_data_size;
		channel_descriptor->CONFIG.DSTAIMD = channel_descriptor->CONFIG.DSTSIZE;
		break;
	case 0b01: /* decrement */
		LOG_ERR("dest_addr_adj value not supported by HW");
		return -ENOTSUP;
	case 0b10: /* no change */
		channel_descriptor->DSTEND.U32 = block->dest_address;
		channel_descriptor->CONFIG.DSTAIMD = 0b11;
		break;
	default:
		LOG_ERR("Unknown dest_addr_adj value");
		return -EINVAL;
	}

	SI32_DMACTRL_A_disable_data_request(SI32_DMACTRL_0, channel);
	SI32_DMACTRL_A_enable_channel(SI32_DMACTRL_0, channel);

	return 0;
}

static int dma_si32_start(const struct device *dev, uint32_t channel)
{
	struct SI32_DMADESC_A_Struct *channel_desc = &channel_descriptors[channel];

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	/* As per "5. Using the DMA for a Memory-to-Memory Transfer" */
	__ASSERT(SI32_CLKCTRL_0->AHBCLKG.DMACEN,
		 "1. Enable the AHB and APB clocks to the DMA controller.");
	__ASSERT(SI32_DMACTRL_A_is_enabled(SI32_DMACTRL_0),
		 "2. Enable the DMA module (DMAEN = 1).");
	__ASSERT(SI32_DMACTRL_0->BASEPTR.U32 == (uintptr_t)channel_descriptors,
		 "3. Set the address location of the channel transfer descriptors (BASEPTR)");
	__ASSERT(SI32_DMACTRL_A_is_primary_selected(SI32_DMACTRL_0, channel),
		 "4. Use the CHALTCLR register to set the channel to use the primary descriptor");
	/* 5. Create the primary descriptor in memory for the desired transfer ... */
	__ASSERT(channel_desc->SRCEND.U32,
		 "a. Set the SRCEND field to the last address of the source data.");
	__ASSERT(channel_desc->DSTEND.U32,
		 "b. Set the DSTEND field to the last address of the destination memory.");
	__ASSERT(channel_desc->CONFIG.DSTAIMD == channel_desc->CONFIG.SRCAIMD,
		 "c. Set the destination and source address increment modes (DSTAIMD and "
		 "SRCAIMD). In most cases, these values should be the same");
	// __ASSERT(channel_desc->CONFIG.DSTSIZE == 0,
	// 	 "d. Set the destination and source data size (DSTSIZE and SRCSIZE) to the "
	// 	 "same value.");
	// __ASSERT(channel_desc->CONFIG.SRCSIZE == 0,
	// 	 "d. Set the destination and source data size (DSTSIZE and SRCSIZE) to the "
	// 	 "same value.");
	// __ASSERT(channel_desc->CONFIG.RPOWER == 0,
	// 	 "e. Set the RPOWER to the desired number of data transfers between "
	// 	 "rearbitration. In most cases, this value can be 0.");
	// __ASSERT(channel_desc->CONFIG.NCOUNT == 47,
	// 	 "f. Set the NCOUNT field to the total number of transfers minus 1.");
	channel_desc->CONFIG.TMD = dam_si32_data.channel_data[channel].TMD;
	__ASSERT(channel_desc->CONFIG.TMD == 2,
		 "g. Set the transfer mode to the auto-request type (TMD = 2).");
	__ASSERT(SI32_DMACTRL_0->CHREQMSET.U32 & BIT(channel),
		 "6. Disable data requests for the channel using the CHREQMSET register.");
	__ASSERT(SI32_SCONFIG_0->CONFIG.FDMAEN,
		 "7. Set the DMA to fast mode using the FDMAEN bit in the SCONFIG module.");
	__ASSERT(SI32_DMACTRL_0->CHENSET.U32 & BIT(channel),
		 "8. Enable the DMA channel using the CHENSET register.");
	/* 9. (Optional) Enable the DMA channel interrupt. */
	irq_enable(DMACH0_IRQn + channel);

	/* 10. Submit a request to start the transfer. */
	LOG_DBG("Generate SW request for channel %" PRIu32, channel);
	SI32_DMACTRL_A_generate_software_request(SI32_DMACTRL_0, channel);

	return 0;
}

static int dma_si32_stop(const struct device *dev, uint32_t channel)
{
	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	irq_disable(DMACH0_IRQn + channel);

	channel_descriptors[channel].CONFIG.TMD = 0;

	return 0;
}

static const struct dma_driver_api dma_si32_driver_api = {
	.config = dma_si32_config,
	.start = dma_si32_start,
	.stop = dma_si32_stop,
};

DEVICE_DT_INST_DEFINE(0, &dma_si32_init, NULL, &dam_si32_data, NULL, POST_KERNEL,
		      CONFIG_DMA_INIT_PRIORITY, &dma_si32_driver_api);
