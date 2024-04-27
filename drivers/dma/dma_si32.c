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

__aligned(SI32_DMADESC_PRI_ALIGN) struct SI32_DMADESC_A_Struct channels[CHANNEL_COUNT];

static void dma_si32_isr_handler(uint8_t channel)
{
	struct SI32_DMADESC_A_Struct *channel_descriptor = &channels[channel];

	LOG_DBG("Channel %" PRIu8 " ISR fired", channel);

	if (SI32_DMACTRL_A_is_bus_error_set(SI32_DMACTRL_0)) {
		LOG_ERR("Bus error on channel %" PRIu8, channel);
		return;
	}

	__ASSERT(channel_descriptor->CONFIG.TMD == 0, "Result of success: TMD set to zero");
	__ASSERT(channel_descriptor->CONFIG.NCOUNT == 0, "Result of success: All blocks processed");
	__ASSERT(SI32_DMACTRL_0->CHENSET.CH0 == 0, "Result of success: Channel disabled");
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
	SI32_DMACTRL_A_write_baseptr(SI32_DMACTRL_0, (uintptr_t)channels);

	/* Enable the DMA interface */
	SI32_DMACTRL_A_enable_module(SI32_DMACTRL_0);

	/* Use primary descriptors only */
	SI32_DMACTRL_A_write_chalt(SI32_DMACTRL_0, 0);

	/* AN666.pdf: The SCONFIG module contains a bit (FDMAEN) that enables faster DMA transfers
	 * when set to 1. It is recommended that all applications using the DMA set this bit to 1.
	 */
	SI32_SCONFIG_A_enter_fast_dma_mode(SI32_SCONFIG_0);

	LISTIFY(DT_NUM_IRQS(DT_DRV_INST(0)), DMA_SI32_IRQ_CONNECT_GEN, (;));

	return 0;
}

int dma_si32_config(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	const struct dma_block_config *block;

	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	if (cfg == NULL) {
		LOG_ERR("Missing config");
		return -EINVAL;
	}
	block = cfg->head_block;

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	if (!cfg->head_block) {
		LOG_ERR("Missing head block");
		return -EINVAL;
	}

	switch (cfg->channel_direction) {
	case 0b000: /* memory to memory */
		break;
	default: /* everything else is not supported */
		return -ENOTSUP;
	}

	LOG_INF("Configuring channel %" PRIu32 " to transfer %d bytes", channel, block->block_size);

	SI32_DMADESC_A_configure(&channels[channel], (void *)block->source_address,
				 (void *)block->dest_address, block->block_size,
				 SI32_DMADESC_A_CONFIG_BYTE_MOVE_AUTO);

	SI32_DMACTRL_A_disable_data_request(SI32_DMACTRL_0, channel);
	SI32_DMACTRL_A_enable_channel(SI32_DMACTRL_0, channel);

	return 0;
}

int dma_si32_start(const struct device *dev, uint32_t channel)
{
	struct SI32_DMADESC_A_Struct *channel_desc = &channels[channel];

	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	/* As per "5. Using the DMA for a Memory-to-Memory Transfer" */
	__ASSERT(SI32_CLKCTRL_0->AHBCLKG.DMACEN,
		 "1. Enable the AHB and APB clocks to the DMA controller.");
	__ASSERT(SI32_DMACTRL_A_is_enabled(SI32_DMACTRL_0),
		 "2. Enable the DMA module (DMAEN = 1).");
	__ASSERT(SI32_DMACTRL_0->BASEPTR.U32 == (uintptr_t)channels,
		 "3. Set the address location of the channel transfer descriptors (BASEPTR)");
	__ASSERT(SI32_DMACTRL_A_is_primary_selected(SI32_DMACTRL_0, channel),
		 "4. Use the CHALTCLR register to set the channel to use the primary descriptor");
	/* 5. Create the primary descriptor in memory for the desired transfer ... */
	__ASSERT(channel_desc->SRCEND.U32,
		 "a. Set the SRCEND field to the last address of the source data.");
	__ASSERT(channel_desc->DSTEND.U32,
		 "b. Set the DSTEND field to the last address of the destination memory.");
	__ASSERT(channel_desc->CONFIG.DSTAIMD == 0 && channel_desc->CONFIG.SRCAIMD == 0,
		 "c. Set the destination and source address increment modes (DSTAIMD and "
		 "SRCAIMD). In most cases, these values should be the same");
	__ASSERT(channel_desc->CONFIG.DSTSIZE == 0,
		 "d. Set the destination and source data size (DSTSIZE and SRCSIZE) to the "
		 "same value.");
	__ASSERT(channel_desc->CONFIG.SRCSIZE == 0,
		 "d. Set the destination and source data size (DSTSIZE and SRCSIZE) to the "
		 "same value.");
	__ASSERT(channel_desc->CONFIG.RPOWER == 0,
		 "e. Set the RPOWER to the desired number of data transfers between "
		 "rearbitration. In most cases, this value can be 0.");
	__ASSERT(channel_desc->CONFIG.NCOUNT == 47,
		 "f. Set the NCOUNT field to the total number of transfers minus 1.");
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

	/* Own stuff */
	__ASSERT(!SI32_DMACTRL_A_is_bus_error_set(SI32_DMACTRL_0), "No error allowed");
	LOG_DBG("State machine state: %d", SI32_DMACTRL_0->STATUS.STATE);

	return 0;
}

int dma_si32_stop(const struct device *dev, uint32_t channel)
{
	if (dev == NULL) {
		LOG_ERR("Missing device");
		return -EINVAL;
	}

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel (id %" PRIu32 ", have %d)", channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	irq_disable(DMACH0_IRQn + channel);

	channels[channel].CONFIG.TMD = 0;

	return 0;
}

static const struct dma_driver_api dma_si32_driver_api = {
	.config = dma_si32_config,
	.start = dma_si32_start,
	.stop = dma_si32_stop,
};

DEVICE_DT_INST_DEFINE(0, &dma_si32_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,
		      &dma_si32_driver_api);
