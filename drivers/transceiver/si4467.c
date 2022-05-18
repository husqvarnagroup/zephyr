/* si4467.c - Silicon Labs Si4467 driver */

#define DT_DRV_COMPAT silabs_si4467

/*
 * Copyright (c) 2020 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME transceiver_si4467
#define LOG_LEVEL CONFIG_TRX_DRIVER_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <kernel.h>
#include <arch/cpu.h>
#include <debug/stack.h>

#include <device.h>
#include <init.h>
#include <logging/log.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <string.h>
#include <random/rand32.h>

#include <drivers/spi.h>
#include <drivers/gpio.h>

#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE
#include <settings/settings.h>
#endif

#if CONFIG_TRX_SI4467_APPLY_PATCH
#include "drivers/transceiver/si4467_patch.h"
#endif

#include "drivers/transceiver/si4467.h"

/*
 * Using Z_LOG directly leads to build errors when logs are globally disabled
 * (CONFIG_LOG=n). The following is a workaround:
 */
#ifdef CONFIG_LOG
#define Z_LOG_WRAPPER(_level, ...) Z_LOG(_level, __VA_ARGS__)
#else
#define Z_LOG_WRAPPER(_level, ...)
#endif

static struct spi_cs_control cs_ctrl;

/* Any chance this code really does not exist in Zephyr?! */
static bool bit_is_set(uint8_t byte, uint8_t bit_index)
{
	return (bool)(byte & BIT(bit_index));
}

static struct chip_interrupt si4467_chip_interrupt_from_byte(const uint8_t b)
{
	struct chip_interrupt i = {};

	i.cal = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_CAL);
	i.fifo_underflow_overflow_error = bit_is_set(
		b, SI4467_CHIP_INTERRUPT_POS_FIFO_UNDERFLOW_OVERFLOW_ERROR);
	i.state_change = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_STATE_CHANGE);
	i.cmd_error = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_CMD_ERROR);
	i.chip_ready = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_CHIP_READY);
	i.low_batt = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_LOW_BATT);
	i.wut = bit_is_set(b, SI4467_CHIP_INTERRUPT_POS_WUT);

	return i;
}

/* Code is valid in the reply context of GET_MODEM_STATUS and GET_INT_STATUS */
static enum sync_trigger si4467_sync_trigger_from_byte(const uint8_t b)
{
	return (b & BIT(0)) ? SI4467_SYNC_TRIGGER_2 : SI4467_SYNC_TRIGGER_1;
}

uint8_t si4467_chip_interrupt_to_byte(struct chip_interrupt s)
{
	uint8_t ret = 0;

	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_CAL, s.cal);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_FIFO_UNDERFLOW_OVERFLOW_ERROR,
		  s.fifo_underflow_overflow_error);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_FIFO_UNDERFLOW_OVERFLOW_ERROR,
		  s.fifo_underflow_overflow_error);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_STATE_CHANGE, s.state_change);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_CMD_ERROR, s.cmd_error);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_CHIP_READY, s.chip_ready);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_LOW_BATT, s.low_batt);
	WRITE_BIT(ret, SI4467_CHIP_INTERRUPT_POS_WUT, s.wut);

	return ret;
}

/* Code is valid in the reply context of GET_CHIP_STATUS and GET_INT_STATUS */
static enum calibration_type si4467_calibration_type_from_byte(const uint8_t b)
{
	return (b & BIT(1)) ? SI4467_CALIBRATION_TYPE_OFFLINE2 :
				    SI4467_CALIBRATION_TYPE_OFFLINE;
}

int si4467_get_and_log_device_state(struct si4467_context *const ctx)
{
	struct request_device_state_reply reply;

	if (si4467_command_request_device_state(ctx, &reply)) {
		LOG_ERR("failed to get device state");
		return -EIO;
	}

	LOG_DBG("Device state: %d (%s)", reply.curr_state,
		request_device_state_reply_to_string(reply.curr_state));

	return 0;
}

struct status_state {
	const char *name; /* e.g. " state_change" (with trailing space!) */
	const char *status; /* e.g. "=1/1" */
};

/**
 * @param[in, out] status_state_it pointer to first unmodified \ref
 * status_state instance.
 * @param[in] name Interrupt name (just the name, no C string/parentheses!)
 */
#define status_state_set_if_interrupt_active(status_state_it, field_name)      \
	do {                                                                   \
		const char *const int_status_str = interrupt_status_to_string( \
			p.field_name, s.field_name);                           \
		if (int_status_str != interrupt_statuses[0]) {                 \
			status_state_it->name = " " #field_name;               \
			status_state_it->status = int_status_str;              \
			status_state_it++;                                     \
		}                                                              \
	} while (0)

static const char *const interrupt_statuses[4] = {
	"=0/0", /* neither pending nor state is set */
	"=0/1", /* state is set */
	"=1/0", /* pending is set */
	"=1/1", /* state and pending is set */
};

static const char *interrupt_status_to_string(const bool pending,
					      const bool status)
{
	return interrupt_statuses[2 * pending + 1 * status];
}

enum { PRINT_LOG_STATUS_COMPACT_MAX = 6 };

void si4467_log_chip_status(const struct chip_status_reply *const status,
			    const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	struct chip_interrupt p = status->interrupt_pending;
	struct chip_interrupt s = status->interrupt_status;
	const uint8_t active_interrupts = si4467_chip_interrupt_to_byte(s) |
					  si4467_chip_interrupt_to_byte(p);
	const uint8_t active_interrupts_count = popcount(active_interrupts);

	/* Do not print anything when non interrupt states are set */
	if (active_interrupts_count == 0) {
		return;
	}

	/*
	 * According to commit 62d011549ad6b151bb59065831a9afcf8fad0d80 in the
	 * Zephyr repository, LOG_* macros have a hard limit of 15 arguments.
	 * Therefore, falling back on unconditional printing when too many
	 * interrupts are active.
	 */
	if (active_interrupts_count > PRINT_LOG_STATUS_COMPACT_MAX) {
		return si4467_log_chip_status_full(status, log_level);
	}

	/* Initialize fields with zero-output-generating data */
	struct status_state result[PRINT_LOG_STATUS_COMPACT_MAX] = {
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
	};
	struct status_state *rit = result;

	/* Extract representation for all active interrupts. */
	status_state_set_if_interrupt_active(rit, cal);
	status_state_set_if_interrupt_active(rit,
					     fifo_underflow_overflow_error);
	status_state_set_if_interrupt_active(rit, state_change);
	status_state_set_if_interrupt_active(rit, cmd_error);
	status_state_set_if_interrupt_active(rit, chip_ready);
	status_state_set_if_interrupt_active(rit, low_batt);
	status_state_set_if_interrupt_active(rit, wut);

	__ASSERT_NO_MSG(rit - result == active_interrupts_count);

	Z_LOG_WRAPPER(log_level,
	      "Chip interrupts pending/status:%s%s%s%s%s%s%s%s%s%s%s%s",
	      result[0].name, result[0].status, result[1].name,
	      result[1].status, result[2].name, result[2].status,
	      result[3].name, result[3].status, result[4].name,
	      result[4].status, result[5].name, result[5].status);
}

void si4467_log_chip_status_full(const struct chip_status_reply *status,
				 const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	Z_LOG_WRAPPER(log_level,
	      "Chip interrupts pending/status:"
	      " cal=%s"
	      " fifo_underflow_overflow_error=%s"
	      " state_change=%s"
	      " cmd_error=%s"
	      " chip_ready=%s"
	      " low_batt=%s"
	      " wut=%s",
	      interrupt_status_to_string(status->interrupt_pending.cal,
					 status->interrupt_status.cal),
	      interrupt_status_to_string(
		      status->interrupt_pending.fifo_underflow_overflow_error,
		      status->interrupt_status.fifo_underflow_overflow_error),
	      interrupt_status_to_string(status->interrupt_pending.state_change,
					 status->interrupt_status.state_change),
	      interrupt_status_to_string(status->interrupt_pending.cmd_error,
					 status->interrupt_status.cmd_error),
	      interrupt_status_to_string(status->interrupt_pending.chip_ready,
					 status->interrupt_status.chip_ready),
	      interrupt_status_to_string(status->interrupt_pending.low_batt,
					 status->interrupt_status.low_batt),
	      interrupt_status_to_string(status->interrupt_pending.wut,
					 status->interrupt_status.wut));
}

void si4467_log_unhandled_chip_interrupts(
	const struct chip_status_reply *const status, const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	const uint8_t unhandled =
		si4467_chip_interrupt_to_byte(status->interrupt_pending);
	if (unhandled) {
		Z_LOG_WRAPPER(log_level, "Unhandled chip interrupt(s): 0x%02x",
		      unhandled);
	}
}

static struct modem_interrupt si4467_modem_interrupt_from_byte(const uint8_t b)
{
	return (struct modem_interrupt){
		.rssi_latch =
			bit_is_set(b, SI4467_MODEM_INTERRUPT_POS_RSSI_LATCH),
		.postamble_detect = bit_is_set(
			b, SI4467_MODEM_INTERRUPT_POS_POSTAMBLE_DETECT),
		.invalid_sync =
			bit_is_set(b, SI4467_MODEM_INTERRUPT_POS_INVALID_SYNC),
		.rssi_jump =
			bit_is_set(b, SI4467_MODEM_INTERRUPT_POS_RSSI_JUMP),
		.rssi = bit_is_set(b, SI4467_MODEM_INTERRUPT_POS_RSSI),
		.invalid_preamble = bit_is_set(
			b, SI4467_MODEM_INTERRUPT_POS_INVALID_PREAMBLE),
		.preamble_detect = bit_is_set(
			b, SI4467_MODEM_INTERRUPT_POS_PREAMBLE_DETECT),
		.sync_detect =
			bit_is_set(b, SI4467_MODEM_INTERRUPT_POS_SYNC_DETECT),
	};
}

uint8_t si4467_modem_interrupt_to_byte(struct modem_interrupt s)
{
	uint8_t ret = 0;

	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_RSSI_LATCH, s.rssi_latch);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_POSTAMBLE_DETECT,
		  s.postamble_detect);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_INVALID_SYNC, s.invalid_sync);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_RSSI_JUMP, s.rssi_jump);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_RSSI, s.rssi);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_INVALID_PREAMBLE,
		  s.invalid_preamble);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_PREAMBLE_DETECT,
		  s.preamble_detect);
	WRITE_BIT(ret, SI4467_MODEM_INTERRUPT_POS_SYNC_DETECT, s.sync_detect);

	return ret;
}

/*
 * Very similar to si4467_log_chip_status(). Accepting code duplication to
 * minimize macro code.
 */
void si4467_log_modem_status(const struct modem_status_reply *status,
			     const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	struct modem_interrupt p = status->interrupt_pending;
	struct modem_interrupt s = status->interrupt_status;
	const uint8_t active_interrupts = si4467_modem_interrupt_to_byte(s) |
					  si4467_modem_interrupt_to_byte(p);
	const uint8_t active_interrupts_count = popcount(active_interrupts);

	if (active_interrupts_count == 0) {
		return;
	}

	if (active_interrupts_count > PRINT_LOG_STATUS_COMPACT_MAX) {
		return si4467_log_modem_status_full(status, log_level);
	}

	struct status_state result[PRINT_LOG_STATUS_COMPACT_MAX] = {
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
	};
	struct status_state *rit = result;

	status_state_set_if_interrupt_active(rit, rssi_latch);
	status_state_set_if_interrupt_active(rit, postamble_detect);
	status_state_set_if_interrupt_active(rit, invalid_sync);
	status_state_set_if_interrupt_active(rit, rssi_jump);
	status_state_set_if_interrupt_active(rit, rssi);
	status_state_set_if_interrupt_active(rit, invalid_preamble);
	status_state_set_if_interrupt_active(rit, preamble_detect);
	status_state_set_if_interrupt_active(rit, sync_detect);

	__ASSERT_NO_MSG(rit - result == active_interrupts_count);

	Z_LOG_WRAPPER(log_level,
	      "Modem interrupts pending/status:%s%s%s%s%s%s%s%s%s%s%s%s",
	      result[0].name, result[0].status, result[1].name,
	      result[1].status, result[2].name, result[2].status,
	      result[3].name, result[3].status, result[4].name,
	      result[4].status, result[5].name, result[5].status);
}

void si4467_log_modem_status_full(const struct modem_status_reply *status,
				  const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	Z_LOG_WRAPPER(log_level,
	      "Modem interrupts pending/status:"
	      " rssi_latch=%s"
	      " postamble_detect=%s"
	      " invalid_sync=%s"
	      " rssi_jump=%s"
	      " rssi=%s"
	      " invalid_preamble=%s"
	      " preamble_detect=%s"
	      " sync_detect=%s",
	      interrupt_status_to_string(status->interrupt_pending.rssi_latch,
					 status->interrupt_status.rssi_latch),
	      interrupt_status_to_string(
		      status->interrupt_pending.postamble_detect,
		      status->interrupt_status.postamble_detect),
	      interrupt_status_to_string(status->interrupt_pending.invalid_sync,
					 status->interrupt_status.invalid_sync),
	      interrupt_status_to_string(status->interrupt_pending.rssi_jump,
					 status->interrupt_status.rssi_jump),
	      interrupt_status_to_string(status->interrupt_pending.rssi,
					 status->interrupt_status.rssi),
	      interrupt_status_to_string(
		      status->interrupt_pending.invalid_preamble,
		      status->interrupt_status.invalid_preamble),
	      interrupt_status_to_string(
		      status->interrupt_pending.preamble_detect,
		      status->interrupt_status.preamble_detect),
	      interrupt_status_to_string(status->interrupt_pending.sync_detect,
					 status->interrupt_status.sync_detect));
}

void si4467_log_unhandled_modem_interrupts(
	const struct modem_status_reply *const status, const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	const uint8_t unhandled =
		si4467_modem_interrupt_to_byte(status->interrupt_pending);
	if (unhandled) {
		Z_LOG_WRAPPER(log_level, "Unhandled modem interrupt(s): 0x%02x",
		      unhandled);
	}
}

struct packet_handler_interrupt
si4467_packet_handler_interrupt_from_byte(const uint8_t b)
{
	struct packet_handler_interrupt i;

	i.filter_match =
		bit_is_set(b, SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MATCH);
	i.filter_miss =
		bit_is_set(b, SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MISS);
	i.packet_sent =
		bit_is_set(b, SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_SENT);
	i.packet_rx =
		bit_is_set(b, SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_RX);
	i.crc_error =
		bit_is_set(b, SI4467_PACKET_HANDLER_INTERRUPT_POS_CRC_ERROR);
	i.alt_crc_error = bit_is_set(
		b, SI4467_PACKET_HANDLER_INTERRUPT_POS_ALT_CRC_ERROR);
	i.tx_fifo_almost_empty = bit_is_set(
		b, SI4467_PACKET_HANDLER_INTERRUPT_POS_TX_FIFO_ALMOST_EMPTY);
	i.rx_fifo_almost_full = bit_is_set(
		b, SI4467_PACKET_HANDLER_INTERRUPT_POS_RX_FIFO_ALMOST_FULL);

	return i;
}

uint8_t
si4467_packet_handler_interrupt_to_byte(struct packet_handler_interrupt s)
{
	uint8_t ret = 0;

	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_RX_FIFO_ALMOST_FULL,
		  s.rx_fifo_almost_full);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_TX_FIFO_ALMOST_EMPTY,
		  s.tx_fifo_almost_empty);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_ALT_CRC_ERROR,
		  s.alt_crc_error);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_CRC_ERROR,
		  s.crc_error);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_RX,
		  s.packet_rx);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_SENT,
		  s.packet_sent);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MISS,
		  s.filter_miss);
	WRITE_BIT(ret, SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MATCH,
		  s.filter_match);

	return ret;
}

static struct int_interrupt si4467_int_interrupt_from_byte(const uint8_t b)
{
	return (struct int_interrupt){
		.chip_int = bit_is_set(b, SI4467_INT_INTERRUPT_POS_CHIP),
		.modem_int = bit_is_set(b, SI4467_INT_INTERRUPT_POS_MODEM),
		.packet_handler =
			bit_is_set(b, SI4467_INT_INTERRUPT_POS_PACKET_HANDLER),
	};
}

/*
 * Very similar to si4467_log_chip_status(). Accepting code duplication to
 * minimize macro code.
 */
void si4467_log_packet_handler_status(
	const struct packet_handler_status_reply *status,
	const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	struct packet_handler_interrupt p = status->interrupt_pending;
	struct packet_handler_interrupt s = status->interrupt_status;
	const uint8_t active_interrupts =
		si4467_packet_handler_interrupt_to_byte(s) |
		si4467_packet_handler_interrupt_to_byte(p);
	const uint8_t active_interrupts_count = popcount(active_interrupts);

	if (active_interrupts_count == 0) {
		return;
	}

	if (active_interrupts_count > PRINT_LOG_STATUS_COMPACT_MAX) {
		return si4467_log_packet_handler_status_full(status, log_level);
	}

	struct status_state result[PRINT_LOG_STATUS_COMPACT_MAX] = {
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
		{ .name = "", .status = "" }, { .name = "", .status = "" },
	};
	struct status_state *rit = result;

	status_state_set_if_interrupt_active(rit, filter_match);
	status_state_set_if_interrupt_active(rit, filter_miss);
	status_state_set_if_interrupt_active(rit, packet_sent);
	status_state_set_if_interrupt_active(rit, packet_rx);
	status_state_set_if_interrupt_active(rit, crc_error);
	status_state_set_if_interrupt_active(rit, alt_crc_error);
	status_state_set_if_interrupt_active(rit, tx_fifo_almost_empty);
	status_state_set_if_interrupt_active(rit, rx_fifo_almost_full);

	__ASSERT_NO_MSG(rit - result == active_interrupts_count);

	Z_LOG_WRAPPER(log_level,
	      "Packet handler interrupts pending/status:%s%s%s%s%s%s%s%s%s%s%s%s",
	      result[0].name, result[0].status, result[1].name,
	      result[1].status, result[2].name, result[2].status,
	      result[3].name, result[3].status, result[4].name,
	      result[4].status, result[5].name, result[5].status);
}

void si4467_log_packet_handler_status_full(
	const struct packet_handler_status_reply *status,
	const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	Z_LOG_WRAPPER(log_level,
	      "Packet handler interrupts pending/status:"
	      " filter_match=%s"
	      " filter_miss=%s"
	      " packet_sent=%s"
	      " packet_rx=%s"
	      " crc_error=%s"
	      " alt_crc_error=%s"
	      " tx_fifo_almost_empty=%s"
	      " rx_fifo_almost_full=%s",
	      interrupt_status_to_string(status->interrupt_pending.filter_match,
					 status->interrupt_status.filter_match),
	      interrupt_status_to_string(status->interrupt_pending.filter_miss,
					 status->interrupt_status.filter_miss),
	      interrupt_status_to_string(status->interrupt_pending.packet_sent,
					 status->interrupt_status.packet_sent),
	      interrupt_status_to_string(status->interrupt_pending.packet_rx,
					 status->interrupt_status.packet_rx),
	      interrupt_status_to_string(status->interrupt_pending.crc_error,
					 status->interrupt_status.crc_error),
	      interrupt_status_to_string(
		      status->interrupt_pending.alt_crc_error,
		      status->interrupt_status.alt_crc_error),
	      interrupt_status_to_string(
		      status->interrupt_pending.tx_fifo_almost_empty,
		      status->interrupt_status.tx_fifo_almost_empty),
	      interrupt_status_to_string(
		      status->interrupt_pending.rx_fifo_almost_full,
		      status->interrupt_status.rx_fifo_almost_full));
}

void si4467_log_unhandled_packet_handler_interrupts(
	const struct packet_handler_status_reply *const status,
	const uint8_t log_level)
{
	__ASSERT(log_level >= LOG_LEVEL_ERR && log_level <= LOG_LEVEL_DBG,
		 "Invalid log level");

	const uint8_t unhandled = si4467_packet_handler_interrupt_to_byte(
		status->interrupt_pending);
	if (unhandled) {
		Z_LOG_WRAPPER(log_level,
		      "Unhandled packet handler interrupt(s): 0x%02x",
		      unhandled);
	}
}

/**
 * @brief Poll for CTS via SPI
 *
 * @param ctx Pointer to the device structure for the driver instance
 *
 * @return 0 If successful, negative errno code otherwise.
 */
static int si4467_command_await_cts_spi(struct si4467_context *const ctx)
{
	uint8_t rx_bytes;
	const struct spi_buf rx_buf = {
		.buf = &rx_bytes,
		.len = sizeof(rx_bytes),
	};
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };
	uint8_t tx_bytes = COMMAND_READ_CMD_BUFF;
	const struct spi_buf tx_buf = {
		.buf = &tx_bytes,
		.len = sizeof(tx_bytes),
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	/* Retry up to 20 times with 1 ms breaks in between */
	for (int i = 0; i < 20; i++) {
		const int ret_tx = spi_write(ctx->spi, &ctx->spi_cfg, &tx);

		if (ret_tx) {
			LOG_ERR("Cannot write READ_CMD_BUFF command");
			return -EIO;
		}

		const int ret_rx = spi_read(ctx->spi, &ctx->spi_cfg, &rx);

		if (ret_rx) {
			LOG_ERR("Cannot read CTS value");
			return -EIO;
		}

		if (rx_bytes == 0xFF) {
			return 0;
		}

		/* nSEL needs to be released for next try */
		spi_release(ctx->spi, &ctx->spi_cfg);

		k_sleep(K_MSEC(1));
	};
	return -ETIME;
}

/**
 * @brief Poll for CTS via GPIO
 *
 * @param ctx Pointer to the device structure for the driver instance
 *
 * @return 0 If successful, negative errno code otherwise.
 */
static int si4467_await_cts_gpio(struct si4467_context *const ctx)
{
	struct si4467_gpio_configuration gpio1 =
		ctx->gpios[SI4467_GPIO_IDX_GPIO1];
	int i = 0;

	while (gpio_pin_get(gpio1.dev, gpio1.pin) == 0) {
		k_usleep(100);
		i++;
		if (i > 100) {
			LOG_ERR("timeout while waiting for CTS GPIO");
			return -ETIME;
		}
	}

	return 0;
}

int si4467_command_send(struct si4467_context *const ctx,
			const enum si4467_command cmd, const uint8_t *tx_bytes,
			const size_t tx_length, uint8_t *rx_bytes,
			const size_t rx_length, const bool await_cts)
{
	uint8_t cmd_buf = cmd; /* non-const because of usage in spi_buf */
	const struct spi_buf tx_buf[2] = {
		{ .buf = &cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = (void *)tx_bytes, .len = tx_length }
	};
	struct spi_buf_set tx = { .buffers = tx_buf,
				  .count = ARRAY_SIZE(tx_buf) };
	int ret = 0;

	if (cmd == COMMAND_READ_CMD_BUFF) {
		return -EINVAL;
	}

	if (rx_length > SI4467_MAX_COMMAND_RESPONSE_SIZE) {
		LOG_ERR("Can not read more than 16 bytes. Requested: %u",
			rx_length);
		return -EINVAL;
	}

	/* Not strictly necessarily, but trying to prevent mistakes here */
	if (!await_cts &&
	    !(cmd == COMMAND_READ_RX_FIFO || cmd == COMMAND_WRITE_TX_FIFO)) {
		LOG_ERR("CTS must be awaited when reading data back!");
		return -EINVAL;
	}

	/* We need exclusive access while we do the send-command-transaction */
	k_mutex_lock(&ctx->dev_lock, K_FOREVER);

	if (!await_cts) {
		ctx->spi_cfg.operation |= SPI_HOLD_ON_CS | SPI_LOCK_ON;
	}

	const int ret_tx = spi_write(ctx->spi, &ctx->spi_cfg, &tx);

	if (ret_tx) {
		LOG_ERR("Cannot write command 0x%02x with data %p and length"
			" %u: %d",
			cmd, tx_bytes, tx_length, ret_tx);
		ret = -EIO;
		goto exit_release_spi;
	}

	if (await_cts) {
		ctx->spi_cfg.operation |= SPI_HOLD_ON_CS | SPI_LOCK_ON;
		const int ret_cts = si4467_command_await_cts_spi(ctx);

		if (ret_cts) {
			LOG_ERR("Cannot read CTS after sending command 0x%02x"
				" with data %p and length %u: %d",
				cmd, tx_bytes, tx_length, ret_cts);
			ret = -EIO;
			goto exit_release_spi;
		}
	}

	if (rx_length) {
		const struct spi_buf rx_buf = {
			.buf = rx_bytes,
			.len = rx_length,
		};
		const struct spi_buf_set rx = { .buffers = &rx_buf,
						.count = 1 };
		const int ret_rx = spi_read(ctx->spi, &ctx->spi_cfg, &rx);

		if (ret_rx) {
			LOG_ERR("Cannot read command 0x%02x with data %p and"
				" length %u: %d",
				cmd, rx_bytes, rx_length, ret_rx);
			ret = -EIO;
			goto exit_release_spi;
		}
	}

exit_release_spi:
	spi_release(ctx->spi, &ctx->spi_cfg);
	ctx->spi_cfg.operation &= ~(SPI_HOLD_ON_CS | SPI_LOCK_ON);
	k_mutex_unlock(&ctx->dev_lock);

	return ret;
}

int si4467_command_read_fast_response_register(
	struct si4467_context *const ctx, const enum fast_response_register frr,
	uint8_t *register_value)
{
	int ret;
	uint8_t cmd;

	uint8_t rx_bytes[2];

	switch (frr) {
	case SI4467_FRR_A:
		cmd = COMMAND_FRR_A_READ;
		break;
	case SI4467_FRR_B:
		cmd = COMMAND_FRR_B_READ;
		break;
	case SI4467_FRR_C:
		cmd = COMMAND_FRR_C_READ;
		break;
	case SI4467_FRR_D:
		cmd = COMMAND_FRR_D_READ;
		break;
	default:
		LOG_ERR("invalid FRR: %d", frr);
		return -EINVAL;
	}

	/* We directly use spi_transceive() here, as we do not want to wait for
	 * CTS to read the FRR, and NSEL must not be deasserted between
	 * writing/reading. As read/write happens synchronously, we read two
	 * bytes. The FRR value is in the second byte.
	 */
	const struct spi_buf tx_buf[1] = { {
		.buf = &cmd,
		.len = 1,
	} };
	struct spi_buf_set tx = { .buffers = tx_buf,
				  .count = ARRAY_SIZE(tx_buf) };

	const struct spi_buf rx_buf = {
		.buf = rx_bytes,
		.len = sizeof(rx_bytes),
	};
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	/* get exclusive access for transaction */
	k_mutex_lock(&ctx->dev_lock, K_FOREVER);

	ret = spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI transaction failed:: %d", ret);
	} else {
		*register_value = rx_bytes[1];
	}

	spi_release(ctx->spi, &ctx->spi_cfg);
	ctx->spi_cfg.operation &= ~(SPI_HOLD_ON_CS | SPI_LOCK_ON);
	k_mutex_unlock(&ctx->dev_lock);

	return ret;
}

/**
 * @brief Set TX/RX switch state if applicable
 * @param ctx Pointer to the device structure for the driver instance
 * @param tx Set switch to TX position (true) or RX (false).
 * @return Negative errno code on failure, 0 on success.
 */
static int si4467_set_rx_tx_switch(struct si4467_context *const ctx, bool tx)
{
#if CONFIG_TRX_SI4467_RF_CONTROL_RX_HANDLING
	struct si4467_gpio_configuration rx_enable_gpio =
		ctx->gpios[SI4467_GPIO_IDX_RX_ENABLE];

	if (gpio_pin_set(rx_enable_gpio.dev, rx_enable_gpio.pin, tx ? 0 : 1)) {
		LOG_ERR("Cannot set RX/TX switch: failed to set RX GPIO pin");
		return -EIO;
	}
#endif

#if CONFIG_TRX_SI4467_RF_CONTROL_TX_HANDLING
	struct si4467_gpio_configuration tx_enable_gpio =
		ctx->gpios[SI4467_GPIO_IDX_TX_ENABLE];

	/* set correct direction of RX/TX control switch */
	/* general confusion note: SKY13351 INPUT is connected to OUTPUT1 when
	 * VCTL1 is LOW (and VCTL2 is HIGH), but RX is connected to OUTPUT2,
	 * while RX_ENABLE is connected to VCTL1, so we set RX_ENABLE to high
	 * for RX (and TX_ENABLE to low) and vice versa.
	 */
	if (gpio_pin_set(tx_enable_gpio.dev, tx_enable_gpio.pin, tx ? 1 : 0)) {
		LOG_ERR("Cannot set RX/TX switch: failed to set TX GPIO pin");
		return -EIO;
	}

#endif

	return 0;
}

int si4467_command_start_rx(struct si4467_context *const ctx,
			    const struct start_rx_condition condition,
			    const uint16_t rx_len,
			    const uint8_t rx_channel,
			    const enum operating_state next_timeout,
			    const enum operating_state next_valid,
			    const enum operating_state next_invalid)
{
	/* set RF switch to RX */
	si4467_set_rx_tx_switch(ctx, false);

	/* start RX */
	const uint8_t tx_bytes[] = {
		rx_channel, /* CHANNEL */
		condition.start | (condition.update << 3), /* CONDITION */
		(rx_len >> 8) & 0x1F, /* RX_LEN[12:8] */
		rx_len & 0xFF, /* RX_LEN[7:0] */
		next_timeout,
		next_valid,
		next_invalid,
	};

	/* Refuse unsupported RX lengths */
	if (rx_len > 0x1FFF) {
		return -EINVAL;
	}

	/* Warning: CTS won't turn high until chip has entered the RX state! */
	const int ret = si4467_command_send(ctx, COMMAND_START_RX, tx_bytes,
					    sizeof(tx_bytes), NULL, 0, true);
	if (ret) {
		LOG_ERR("Cannot execute command START_RX");
		return -EIO;
	}

	return 0;
}

static int si4467_command_read_rx_fifo(struct si4467_context *const ctx,
				       uint8_t *rx, size_t rx_size)
{
	if (si4467_command_send(ctx, COMMAND_READ_RX_FIFO, NULL, 0, rx, rx_size,
				false)) {
		LOG_ERR("Cannot execute command READ_RX_FIFO");
		return -EIO;
	}

	return 0;
}

int si4467_command_start_tx(struct si4467_context *ctx,
						const uint16_t txlen,
						const uint8_t tx_channel)
{
	/* set RF switch to TX */
	si4467_set_rx_tx_switch(ctx, true);

	/* start TX */
	const uint8_t tx_bytes[] = {
		tx_channel,
		(uint8_t)(SI4467_START_TRX_IMMEDIATE << 0) |
			(uint8_t)(SI4467_START_TX_SEND << 1) |
			(uint8_t)(SI4467_START_TRX_UPDATE_USE << 3) |
			(uint8_t)(SI4467_OPERATING_STATE_RX << 4),
		(uint8_t)(txlen >> 8), /* TX length [12:8] */
		(uint8_t)(txlen), /* TX length [7:0] */
		0x00, /* TX delay */
		0x00, /* don't repeat transmission */
	};

	const int ret = si4467_command_send(ctx, COMMAND_START_TX, tx_bytes,
					    sizeof(tx_bytes), NULL, 0, true);
	if (ret) {
		LOG_ERR("Cannot execute command START_TX");
		return -EIO;
	}

	return 0;
}

#if CONFIG_TRX_SI4467_APPLY_PATCH
static const uint8_t si4467_patch[][8] = SI4467_PATCH_DATA_ARRAY;
#endif

static int si4467_command_boot_up(struct si4467_context *const ctx)
{
	const uint32_t xtal_frequency_hz = si4467_get_xtal_frequency();
	uint8_t tx_bytes[] = {
		0x01, /* BOOT_OPTIONS */
		0x00, /* XTAL_OPTIONS: TCXO = 0 */
		(xtal_frequency_hz >> 24) & 0xFF, /* XO_FREQ[31:24] */
		(xtal_frequency_hz >> 16) & 0xFF, /* XO_FREQ[23:16] */
		(xtal_frequency_hz >> 8) & 0xFF, /* XO_FREQ[15:8 */
		(xtal_frequency_hz >> 0) & 0xFF, /* XO_FREQ[7:0] */
	};

#if CONFIG_TRX_SI4467_APPLY_PATCH
	LOG_INF("applying patch: %zu bytes", sizeof(si4467_patch));

	for (uint8_t i = 0; i < ARRAY_SIZE(si4467_patch); i++) {
		const int ret = si4467_command_send(ctx, si4467_patch[i][0],
						    &si4467_patch[i][1], 7,
						    NULL, 0, true);
		if (ret) {
			LOG_ERR("Cannot patch TRX firmware");
			return -EIO;
		}
	}

	tx_bytes[0] |= BIT(7); /* enable application of patch on power-up */
#endif

	const int ret = si4467_command_send(ctx, COMMAND_POWER_UP, tx_bytes,
					    sizeof(tx_bytes), NULL, 0, true);
	if (ret) {
		LOG_ERR("Cannot execute command power up");
		return -EIO;
	}

	return 0;
}

/**
 * @brief Fetch details about transceiver.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param info Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
static int si4467_command_part_info(struct si4467_context *const ctx,
				    struct part_information *info)
{
	uint8_t rx_bytes[8];

	const int ret = si4467_command_send(ctx, COMMAND_PART_INFO, NULL, 0,
					    rx_bytes, sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command part info: %d", ret);
		return -EIO;
	}

	info->chiprev = rx_bytes[0];
	info->part = ((uint16_t)rx_bytes[1] << 8) | rx_bytes[2];
	info->pbuild = rx_bytes[3];
	info->id = ((uint16_t)rx_bytes[4] << 8) | rx_bytes[5];
	info->customer = rx_bytes[6];
	info->romid = rx_bytes[7];

	return 0;
}

int si4467_command_func_info(struct si4467_context *const ctx,
			     struct func_information *info)
{
	uint8_t rx_bytes[6];

	const int ret = si4467_command_send(ctx, COMMAND_FUNC_INFO, NULL, 0,
					    rx_bytes, sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command part info: %d", ret);
		return -EIO;
	}

	info->revext = rx_bytes[0];
	info->revbranch = rx_bytes[1];
	info->revint = rx_bytes[2];
	info->patch = ((uint16_t)rx_bytes[3] << 8) | rx_bytes[4];
	info->func = rx_bytes[5];

	return 0;
}

int si4467_command_gpio_pin_config(struct si4467_context *const ctx,
				   const uint8_t gpio[4], uint8_t nirq,
				   uint8_t sdo, uint8_t gen_config)
{
	uint8_t config_bytes[7];

	for (uint8_t i = 0; i < 4; i++) {
		config_bytes[i] = gpio[i];
	}
	config_bytes[4] = nirq;
	config_bytes[5] = sdo;
	config_bytes[6] = gen_config;

	const int ret = si4467_command_send(ctx, COMMAND_GPIO_PIN_CFG,
					    config_bytes, sizeof(config_bytes),
					    config_bytes, sizeof(config_bytes),
					    true);
	if (ret) {
		LOG_ERR("Cannot execute command GPIO_PIN_CFG: %d", ret);
		return -EIO;
	}

	return 0;
}

/**
 * @brief Set the value of one or more properties.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param arg_len Argument stream length.
 * @param arg_stream Pointer to argument stream. Index zero points to group
 * value.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
static int si4467_command_set_property_raw(struct si4467_context *const ctx,
					   const uint8_t arg_len,
					   const uint8_t *arg_stream)
{
	return si4467_command_send(ctx, COMMAND_SET_PROPERTY, arg_stream,
				   arg_len, NULL, 0, true);
}

int si4467_command_set_property(struct si4467_context *const ctx,
				const enum si4467_property_group group,
				const uint8_t property, const uint8_t data)
{
	const uint8_t tx_bytes[4] = { group, 1, property, data };

	return si4467_command_send(ctx, COMMAND_SET_PROPERTY, tx_bytes,
				   sizeof(tx_bytes), NULL, 0, true);
}

int si4467_command_get_properties(struct si4467_context *const ctx,
				  const enum si4467_property_group group,
				  const uint8_t property, uint8_t *data,
				  const uint8_t len)
{
	const uint8_t tx_bytes[3] = { group, len, property };

	/* No more than 16 properties can be fetched at once */
	if (len > 16) {
		return -EIO;
	}
	return si4467_command_send(ctx, COMMAND_GET_PROPERTY, tx_bytes,
				   sizeof(tx_bytes), data, len, true);
}

ssize_t si4467_property_group_size(const enum si4467_property_group group)
{
	switch (group) {
	case SI4467_PROPERTY_GROUP_GLOBAL:
		return SI4467_PROPERTY_GROUP_GLOBAL_SIZE;
	case SI4467_PROPERTY_GROUP_INT_CTL:
		return SI4467_PROPERTY_GROUP_INT_CTL_SIZE;
	case SI4467_PROPERTY_GROUP_FRR_CTL:
		return SI4467_PROPERTY_GROUP_FRR_CTL_SIZE;
	case SI4467_PROPERTY_GROUP_PREAMBLE:
		return SI4467_PROPERTY_GROUP_PREAMBLE_SIZE;
	case SI4467_PROPERTY_GROUP_SYNC:
		return SI4467_PROPERTY_GROUP_SYNC_SIZE;
	case SI4467_PROPERTY_GROUP_PKT:
		return SI4467_PROPERTY_GROUP_PKT_SIZE;
	case SI4467_PROPERTY_GROUP_MODEM:
		return SI4467_PROPERTY_GROUP_MODEM_SIZE;
	case SI4467_PROPERTY_GROUP_MODEM_CHFLT:
		return SI4467_PROPERTY_GROUP_MODEM_CHFLT_SIZE;
	case SI4467_PROPERTY_GROUP_PA:
		return SI4467_PROPERTY_GROUP_PA_SIZE;
	case SI4467_PROPERTY_GROUP_SYNTH:
		return SI4467_PROPERTY_GROUP_SYNTH_SIZE;
	case SI4467_PROPERTY_GROUP_MATCH:
		return SI4467_PROPERTY_GROUP_MATCH_SIZE;
	case SI4467_PROPERTY_GROUP_FREQ_CONTROL:
		return SI4467_PROPERTY_GROUP_FREQ_CONTROL_SIZE;
	case SI4467_PROPERTY_GROUP_RX_HOP:
		return SI4467_PROPERTY_GROUP_RX_HOP_SIZE;
	case SI4467_PROPERTY_GROUP_PTI:
		return SI4467_PROPERTY_GROUP_PTI_SIZE;
	}

	__ASSERT_NO_MSG(false);
	return -EINVAL;
}

const char *si4467_property_group_str(const enum si4467_property_group group)
{
	switch (group) {
	case SI4467_PROPERTY_GROUP_GLOBAL:
		return "GLOBAL";
	case SI4467_PROPERTY_GROUP_INT_CTL:
		return "INT_CTL";
	case SI4467_PROPERTY_GROUP_FRR_CTL:
		return "FRR_CTL";
	case SI4467_PROPERTY_GROUP_PREAMBLE:
		return "PREAMBLE";
	case SI4467_PROPERTY_GROUP_SYNC:
		return "SYNC";
	case SI4467_PROPERTY_GROUP_PKT:
		return "PKT";
	case SI4467_PROPERTY_GROUP_MODEM:
		return "MODEM";
	case SI4467_PROPERTY_GROUP_MODEM_CHFLT:
		return "MODEM_CHFLT";
	case SI4467_PROPERTY_GROUP_PA:
		return "PA";
	case SI4467_PROPERTY_GROUP_SYNTH:
		return "SYNTH";
	case SI4467_PROPERTY_GROUP_MATCH:
		return "MATCH";
	case SI4467_PROPERTY_GROUP_FREQ_CONTROL:
		return "FREQ_CONTROL";
	case SI4467_PROPERTY_GROUP_RX_HOP:
		return "RX_HOP";
	case SI4467_PROPERTY_GROUP_PTI:
		return "PTI";
	}

	__ASSERT_NO_MSG(false);
	return "???";
}

int si4467_get_property_clkgen_band(
	struct si4467_context *const ctx,
	struct si4467_property_modem_clkgen_band *modem_clkgen_band)
{
	uint8_t rx_byte;
	int ret;

	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_MODEM,
					    SI4467_PROPERTY_MODEM_CLKGEN_BAND,
					    &rx_byte, sizeof(rx_byte));
	if (ret) {
		return -EIO;
	}

	/* Enforce valididty of known 0 values match */
	if (rx_byte & (BIT_MASK(3) << 5)) {
		return -EIO;
	}

	modem_clkgen_band->band = rx_byte & BIT_MASK(3);
	modem_clkgen_band->sy_sel =
		SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_HIGH_PERFORMANCE;
	if (!(rx_byte & BIT(3))) {
		__ASSERT(false,
			 "Contact Silicon Labs for assistance with frequency "
			 "calculations when clearing this bit.");
		modem_clkgen_band->sy_sel =
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_LOW_POWER;
	}
	if (rx_byte & BIT(4)) {
		modem_clkgen_band->force_sy_recal =
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_FORCE_SY_RECAL_SKIP;
	} else {
		modem_clkgen_band->force_sy_recal =
			SI4467_PROPERTY_MODEM_CLKGEN_BAND_FORCE_SY_RECAL_FORCE;
	}

	return 0;
}

const char *si4467_property_modem_clkgen_band_sy_sel_str(
	const enum si4467_property_modem_clkgen_band_sy_sel sy_sel)
{
	switch (sy_sel) {
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_LOW_POWER:
		return "Low power";
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_HIGH_PERFORMANCE:
		return "High performance";
	}

	__ASSERT(false, "Unreachable");

	return "???";
}

int si4467_get_property_freq_control_inte(struct si4467_context *const ctx,
					  uint8_t *inte)
{
	uint8_t rx_byte;
	int ret;

	ret = si4467_command_get_properties(ctx,
					    SI4467_PROPERTY_GROUP_FREQ_CONTROL,
					    SI4467_PROPERTY_FREQ_CONTROL_INTE,
					    &rx_byte, sizeof(rx_byte));
	if (ret) {
		return -EIO;
	}

	/* Enforce valididty of known 0 values match */
	if (rx_byte & BIT(7)) {
		return -EIO;
	}

	*inte = rx_byte;

	return 0;
}

int si4467_get_property_freq_control_frac(struct si4467_context *const ctx,
					  uint32_t *frac)
{
	uint8_t rx_bytes[3];
	int ret;

	ret = si4467_command_get_properties(ctx,
					    SI4467_PROPERTY_GROUP_FREQ_CONTROL,
					    SI4467_PROPERTY_FREQ_CONTROL_FRAC,
					    rx_bytes, sizeof(rx_bytes));
	if (ret) {
		return -EIO;
	}

	/* Enforce validity of known 0 values match */
	if (rx_bytes[0] & (BIT_MASK(4) << 4)) {
		return -EIO;
	}

	*frac = sys_get_be24(rx_bytes);

	return 0;
}

int si4467_get_property_freq_control_channel_step_size(
	struct si4467_context *const ctx, uint16_t *channel_step_size_hz)
{
	uint8_t rx_bytes[2];
	int ret;

	ret = si4467_command_get_properties(
		ctx, SI4467_PROPERTY_GROUP_FREQ_CONTROL,
		SI4467_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE, rx_bytes,
		sizeof(rx_bytes));
	if (ret) {
		return -EIO;
	}

	*channel_step_size_hz = sys_get_be16(rx_bytes);

	return 0;
}

int si4467_get_property_rx_hop_control(
	struct si4467_context *const ctx,
	struct si4467_property_group_rx_hop_control *control)
{
	uint8_t rx_byte;
	int ret;

	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_RX_HOP,
					    SI4467_PROPERTY_RX_HOP_CONTROL,
					    &rx_byte, sizeof(rx_byte));
	if (ret) {
		return -EIO;
	}

	/* Enforce valididty of known 0 values match */
	if (rx_byte & BIT(7)) {
		return -EIO;
	}

	if (((rx_byte >> 4) & BIT_MASK(3)) > 4) {
		return -EIO;
	}
	control->enable =
		(enum si4467_property_group_rx_hop_control_en)((rx_byte >> 4) &
							       BIT_MASK(3));
	control->rssi_timeout = rx_byte & BIT_MASK(4);

	return 0;
}

int si4467_get_property_pwr_lvl(struct si4467_context *const ctx,
				uint8_t *power_level)
{
	int ret;

	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_PA,
					    SI4467_PROPERTY_PA_PWR_LVL,
					    power_level, sizeof(*power_level));
	if (ret) {
		return -EIO;
	}

	return 0;
}

int si4467_get_property_pa_bias_clkduty(struct si4467_context *const ctx,
					uint8_t *bias, uint8_t *clk_duty)
{
	uint8_t reg_value;
	int ret;

	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_PA,
					    SI4467_PROPERTY_PA_BIAS_CLKDUTY,
					    &reg_value, sizeof(reg_value));
	if (ret) {
		return -EIO;
	}

	*bias = (reg_value & SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_MASK) >>
		SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_POS;
	*clk_duty = (reg_value &
		     SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_MASK) >>
		    SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS;

	return 0;
}

int si4467_set_property_pa_bias_clkduty(struct si4467_context *const ctx,
					const uint8_t bias,
					const uint8_t clk_duty)
{
	uint8_t reg_value;

	if ((bias << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_POS) >
		    SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_MASK ||
	    (clk_duty << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS) >
		    SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_MASK) {
		return -EIO;
	}
	reg_value =
		(bias << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_POS) |
		(clk_duty << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS);
	return si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_PA,
					   SI4467_PROPERTY_PA_BIAS_CLKDUTY,
					   reg_value);
}

uint8_t si4467_pa_dbm_to_pwr_lvl(const uint8_t tx_power_dbm)
{
	/*
	 * LUT to convert dBm values to TX power levels for the Si4467. These
	 * values are based on power measurements on the Si4467 DK and may need
	 * to be adapted in case modulation settings are changed. The values
	 * should be reusable across device types except for a constant offset
	 * (which may be corrected by setting an appropriate value for
	 * SI4467_MODEM_RSSI_COMP).
	 *
	 * Note: PA level could be calculated with a polynomial, but we would
	 * need at least degree 4, so since we have only 12 relevant values,
	 * look-up table is more efficient both CPU and RAM-wise (12x uint8 vs
	 * 5x float).
	 */
	const uint8_t LUT[] = { 9, 11, 12, 14, 17, 19, 21, 25, 29, 36, 45, 60 };

	if (tx_power_dbm >= 12) {
		return 0x4f;
	}

	return LUT[tx_power_dbm];
}

int si4467_set_property_pwr_lvl(struct si4467_context *const ctx,
				const uint8_t power_level)
{
	int ret;

	if (power_level > SI4467_PA_POWER_LEVEL_MAX) {
		LOG_ERR("power level exceeds max: %d > %d", power_level,
			SI4467_PA_POWER_LEVEL_MAX);
		return -EINVAL;
	}

	ret = si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_PA,
					  SI4467_PROPERTY_PA_PWR_LVL,
					  power_level);
	if (ret) {
		return -EIO;
	}

	return 0;
}

uint8_t si4467_property_modem_clkgen_band_band_to_outdiv(
	const enum si4467_property_modem_clkgen_band_band band)
{
	switch (band) {
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_4:
		return 4;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_6:
		return 6;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_8:
		return 8;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_12:
		return 12;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_16:
		return 16;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24:
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_2:
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_3:
		return 24;
	}
	__ASSERT(false, "Unreachable code");
	return 4; /* Dummy value */
}

const char *
request_device_state_reply_to_string(const enum operating_state state)
{
	static const char *const device_state_str[] = {
		"Invalid", "SLEEP",   "SPI_ACTIVE", "READY", "READY2",
		"TX_TUNE", "RX_TUNE", "TX",	    "RX"
	};

	/* Better safe than sorry. This should never ever happen. */
	__ASSERT(state >= 1 && state <= 8,
		 "Invalid request device state reply value: %d", state);
	if (state < 1 || state > 8) {
		return device_state_str[0];
	}

	return device_state_str[state];
}

int si4467_command_request_device_state(struct si4467_context *const ctx,
					struct request_device_state_reply *out)
{
	uint8_t rx_bytes[2];

	const int ret = si4467_command_send(ctx, COMMAND_REQUEST_DEVICE_STATE,
					    NULL, 0, rx_bytes, sizeof(rx_bytes),
					    true);
	if (ret) {
		LOG_ERR("Cannot execute command REQUEST_DEVICE_STATE: %d", ret);
		return -EIO;
	}

	if (rx_bytes[0] < 1 || rx_bytes[0] > 8) {
		LOG_ERR("invalid device state: %d", rx_bytes[0]);
		return -EIO;
	}
	out->curr_state = (enum operating_state)rx_bytes[0];
	out->curr_channel = rx_bytes[1];

	LOG_DBG("device state: 0x%02x", rx_bytes[0]);
	LOG_DBG("channel: %d", rx_bytes[1]);

	/* Normalize returned value */
	if (out->curr_state == SI4467_OPERATING_STATE_READY2) {
		LOG_INF("Normalized device state READY2 to READY");
		out->curr_state = SI4467_OPERATING_STATE_READY;
	}
	return 0;
}

bool si4467_is_valid_next_operating_state(const uint8_t state)
{
	if (state == 4 || state > 8) {
		return false;
	}
	return true;
}

int si4467_command_change_state(struct si4467_context *const ctx,
				const enum operating_state next_state)
{
	/* Better safe than sorry. This should never ever happen. */
	__ASSERT(si4467_is_valid_next_operating_state(next_state),
		 "Invalid operating_state: %d", next_state);

	const uint8_t tx = next_state;

	const int ret = si4467_command_send(ctx, COMMAND_CHANGE_STATE, &tx,
					    sizeof(tx), NULL, 0, true);
	if (ret) {
		LOG_ERR("Cannot execute command CHANGE_STATE: %d", ret);
		return -EIO;
	}

	return 0;
}

int si4467_command_get_chip_status(struct si4467_context *const ctx,
				   const struct chip_interrupt preserve,
				   struct chip_status_reply_detailed *out)
{
	const uint8_t tx = si4467_chip_interrupt_to_byte(preserve);
	uint8_t rx_bytes[5];

	const int ret = si4467_command_send(ctx, COMMAND_GET_CHIP_STATUS, &tx,
					    sizeof(tx), rx_bytes,
					    sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command get chip status: %d", ret);
		return -EIO;
	}

	LOG_DBG("Chip status: 0x%02x%02x%02x%02x%02x", rx_bytes[0], rx_bytes[1],
		rx_bytes[2], rx_bytes[3], rx_bytes[4]);

	out->chip_status.interrupt_pending =
		si4467_chip_interrupt_from_byte(rx_bytes[0]);
	out->chip_status.interrupt_status =
		si4467_chip_interrupt_from_byte(rx_bytes[1]);
	out->command_error_status = rx_bytes[2];
	out->command_error_command_id = rx_bytes[3];
	out->calibration = si4467_calibration_type_from_byte(rx_bytes[4]);

	return 0;
}

int si4467_command_get_packet_handler_status(
	struct si4467_context *const ctx,
	const struct packet_handler_interrupt preserve,
	struct packet_handler_status_reply *out)
{
	const uint8_t tx = si4467_packet_handler_interrupt_to_byte(preserve);
	uint8_t rx_bytes[2];

	const int ret = si4467_command_send(ctx, COMMAND_GET_PH_STATUS, &tx,
					    sizeof(tx), rx_bytes,
					    sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command get packet_handler status: %d",
			ret);
		return -EIO;
	}

	LOG_DBG("Packet handler status: 0x%02x%02x", rx_bytes[0], rx_bytes[1]);

	out->interrupt_pending =
		si4467_packet_handler_interrupt_from_byte(rx_bytes[0]);
	out->interrupt_status =
		si4467_packet_handler_interrupt_from_byte(rx_bytes[1]);

	return 0;
}

int si4467_command_get_modem_status(struct si4467_context *const ctx,
				    const struct modem_interrupt preserve,
				    struct modem_status_reply_detailed *out)
{
	const uint8_t tx = si4467_modem_interrupt_to_byte(preserve);
	uint8_t rx_bytes[9];

	const int ret = si4467_command_send(ctx, COMMAND_GET_MODEM_STATUS, &tx,
					    sizeof(tx), rx_bytes,
					    sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command get modem status: %d", ret);
		return -EIO;
	}

	LOG_DBG("Modem status: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		rx_bytes[0], rx_bytes[1], rx_bytes[2], rx_bytes[3], rx_bytes[4],
		rx_bytes[5], rx_bytes[6], rx_bytes[7], rx_bytes[8]);

	out->modem_status.interrupt_pending =
		si4467_modem_interrupt_from_byte(rx_bytes[0]);
	out->modem_status.interrupt_status =
		si4467_modem_interrupt_from_byte(rx_bytes[1]);
	out->curr_rssi = rx_bytes[2];
	out->latch_rssi = rx_bytes[3];
	out->ant1_rssi = rx_bytes[4];
	out->ant2_rssi = rx_bytes[5];
	out->afc_freq_offset = ((uint16_t)rx_bytes[7]) << 8 | rx_bytes[6];
	out->info_flags = si4467_sync_trigger_from_byte(rx_bytes[8]);

	return 0;
}

int si4467_command_get_int_status(struct si4467_context *const ctx,
				  const struct int_status_interrupt *preserve,
				  struct int_status_reply *int_status)
{
	const uint8_t tx[3] = {
		preserve ? si4467_packet_handler_interrupt_to_byte(
				   preserve->packet) :
				 0,
		preserve ? si4467_modem_interrupt_to_byte(preserve->modem) : 0,
		preserve ? si4467_chip_interrupt_to_byte(preserve->chip) : 0,
	};
	const uint8_t tx_size = preserve ? sizeof(tx) : 0;
	uint8_t rx_bytes[9];
	const uint8_t rx_size = int_status ? sizeof(rx_bytes) : 0;

	const int ret = si4467_command_send(ctx, COMMAND_GET_INT_STATUS, tx,
					    tx_size, rx_bytes, rx_size, true);
	if (ret) {
		LOG_ERR("Cannot execute command get modem status: %d", ret);
		return -EIO;
	}

	if (!int_status) {
		return 0;
	}

	LOG_DBG("Interrupt status: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		rx_bytes[0], rx_bytes[1], rx_bytes[2], rx_bytes[3], rx_bytes[4],
		rx_bytes[5], rx_bytes[6], rx_bytes[7], rx_bytes[8]);

	int_status->pending = si4467_int_interrupt_from_byte(rx_bytes[0]);
	int_status->status = si4467_int_interrupt_from_byte(rx_bytes[1]);
	int_status->packet_handler.interrupt_pending =
		si4467_packet_handler_interrupt_from_byte(rx_bytes[2]);
	int_status->packet_handler.interrupt_status =
		si4467_packet_handler_interrupt_from_byte(rx_bytes[3]);
	int_status->modem.interrupt_pending =
		si4467_modem_interrupt_from_byte(rx_bytes[4]);
	int_status->modem.interrupt_status =
		si4467_modem_interrupt_from_byte(rx_bytes[5]);
	int_status->chip.interrupt_pending =
		si4467_chip_interrupt_from_byte(rx_bytes[6]);
	int_status->chip.interrupt_status =
		si4467_chip_interrupt_from_byte(rx_bytes[7]);
	int_status->info_flags.sync =
		si4467_sync_trigger_from_byte(rx_bytes[8]);
	int_status->info_flags.calibration =
		si4467_calibration_type_from_byte(rx_bytes[8]);

	return 0;
}

int si4467_get_chip_status(struct si4467_context *const ctx,
			   const struct chip_interrupt preserve_interrupts,
			   struct chip_status_reply_detailed *chip_status)
{
	if (si4467_command_get_chip_status(ctx, preserve_interrupts,
					   chip_status)) {
		LOG_ERR("Cannot clear pending chip interrupts");
		return -EIO;
	}

	return 0;
}

int si4467_get_packet_handler_status(
	struct si4467_context *const ctx,
	const struct packet_handler_interrupt preserve_interrupts,
	struct packet_handler_status_reply *packet_handler_status)
{
	if (si4467_command_get_packet_handler_status(ctx, preserve_interrupts,
						     packet_handler_status)) {
		LOG_ERR("Cannot clear pending packet handler interrupts");
		return -EIO;
	}

	return 0;
}

int si4467_command_fifo_info(struct si4467_context *const ctx,
			     const bool reset_rx_fifo, const bool reset_tx_fifo,
			     struct fifo_information *info)
{
	const uint8_t tx_buf[1] = {
		(reset_rx_fifo ? SI4467_FIFO_INFO_RESET_RX_FIFO : 0) |
		(reset_tx_fifo ? SI4467_FIFO_INFO_RESET_TX_FIFO : 0)
	};
	uint8_t rx_bytes[2];

	const int ret = si4467_command_send(ctx, COMMAND_FIFO_INFO, tx_buf,
					    sizeof(tx_buf), rx_bytes,
					    sizeof(rx_bytes), true);
	if (ret) {
		LOG_ERR("Cannot execute command FIFO_INFO: %d", ret);
		return -EIO;
	}

	info->rx_fifo_count = rx_bytes[0];
	info->tx_fifo_space = rx_bytes[1];

	if (info->rx_fifo_count > SI4467_FIFO_MAX_SIZE ||
	    info->tx_fifo_space > SI4467_FIFO_MAX_SIZE) {
		/*
		 * This should never happen; however for the CBTL, we get 255
		 * for RX FIFO count in very rare cases (less than 1 in 1000).
		 * The problem seems to occur only on CBTL and only happens when
		 * using the Zephyr shell via USB. Potential cause could be
		 * interrupts from USB driver or noise from USB bus affecting
		 * the SPI signal (lower SPI frequency mitigates the issue).
		 * See SG-17344 for details.
		 */
		LOG_ERR("invalid FIFO info values: %d / %d",
			info->rx_fifo_count, info->tx_fifo_space);
		return -EIO;
	}

	return 0;
}

int si4467_command_packet_info(struct si4467_context *const ctx,
			       const enum packet_info_field_num_override f,
			       const uint16_t rx_len_new,
			       uint16_t *const rx_len_currently)
{
	const uint8_t tx_buf[3] = {
		f, (uint8_t)(rx_len_new >> 8), /* TX length [12:8] */
		(uint8_t)(rx_len_new), /* TX length [7:0] */
	};
	uint8_t rx_bytes[2];

	const int ret = si4467_command_send(
		ctx, COMMAND_PACKET_INFO, tx_buf,
		f != SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_NONE ?
			      sizeof(tx_buf) :
			      0,
		rx_bytes, rx_len_currently ? sizeof(rx_bytes) : 0, true);
	if (ret) {
		LOG_ERR("Cannot execute command COMMAND_PACKET_INFO: %d", ret);
		return -EIO;
	}

	if (rx_len_currently) {
		/*
		 * If no previous variable-length packet has been received, the
		 * return value will be 0.
		 */
		if (rx_bytes[0] == 0 && rx_bytes[1] == 0) {
			LOG_WRN("Variable-length packet not yet received");
			return -EBUSY; /* is there a better code? */
		}

		*rx_len_currently = rx_bytes[1];
		*rx_len_currently <<= 8;
		*rx_len_currently += rx_bytes[0];
	}

	return 0;
}

int si4467_get_modem_status(struct si4467_context *const ctx,
			    const struct modem_interrupt preserve_interrupts,
			    struct modem_status_reply_detailed *modem_status)
{
	if (si4467_command_get_modem_status(ctx, preserve_interrupts,
					    modem_status)) {
		LOG_ERR("Cannot clear pending modem interrupts");
		return -EIO;
	}

	return 0;
}

static inline void si4467_nirq_int_handler(const struct device *port,
					   struct gpio_callback *cb,
					   uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct si4467_context *const ctx =
		CONTAINER_OF(cb, struct si4467_context, nirq_cb);
	k_sem_give(&ctx->isr_nirq_sem);
}

/**
 * @brief Enable (or disable) the interrupt for nIRQ.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param enable Whether to enable or disable the interrupt.
 *
 * @return 0 if successful, negative errno code on failure.
 */
static int si4467_enable_nirq_interrupt(struct si4467_context *const ctx,
					const bool enable)
{
	return gpio_pin_interrupt_configure(
		ctx->gpios[SI4467_GPIO_IDX_NIRQ].dev,
		ctx->gpios[SI4467_GPIO_IDX_NIRQ].pin,
		enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
}

/**
 * @brief Initialize and install callbacks for all GPIOs
 *
 * @param ctx Pointer to the device structure for the driver instance
 *
 * @return 0 if successful, negative errno code on failure.
 */
static int si4467_setup_gpio_callback(struct si4467_context *const ctx)
{
	gpio_init_callback(&ctx->nirq_cb, si4467_nirq_int_handler,
			   BIT(ctx->gpios[SI4467_GPIO_IDX_NIRQ].pin));
	return gpio_add_callback(ctx->gpios[SI4467_GPIO_IDX_NIRQ].dev,
				 &ctx->nirq_cb);
}

uint8_t *si4467_get_mac(struct si4467_context *const ctx)
{
#if defined(CONFIG_TRX_SI4467_RANDOM_MAC)
	uint32_t *ptr = (uint32_t *)(ctx->mac_addr + 4);

	UNALIGNED_PUT(sys_rand32_get(), ptr);

	ctx->mac_addr[7] = (ctx->mac_addr[7] & ~0x01) | 0x02;
#else
	si4467->mac_addr[4] = CONFIG_TRX_SI4467_MAC4;
	si4467->mac_addr[5] = CONFIG_TRX_SI4467_MAC5;
	si4467->mac_addr[6] = CONFIG_TRX_SI4467_MAC6;
	si4467->mac_addr[7] = CONFIG_TRX_SI4467_MAC7;
#endif

	ctx->mac_addr[0] = 0x00;
	ctx->mac_addr[1] = 0x12;
	ctx->mac_addr[2] = 0x4b;
	ctx->mac_addr[3] = 0x00;

	return ctx->mac_addr;
}

int si4467_configure_rx_hopping(
	struct si4467_context *ctx,
	const struct si4467_property_group_rx_hop_control *config,
	const uint8_t *const channels, uint8_t count)
{
	int ret;

	if (count > 0x40) {
		LOG_ERR("Exceeding valid table size: %u", count);
		return -EINVAL;
	}

	/* Disable hopping */
	ret = si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_RX_HOP,
					  SI4467_PROPERTY_RX_HOP_CONTROL, 0);
	if (ret) {
		return -EIO;
	}

	/* Configure hopping channel count */
	if (channels) {
		ret = si4467_command_set_property(
			ctx, SI4467_PROPERTY_GROUP_RX_HOP,
			SI4467_PROPERTY_RX_HOP_TABLE_SIZE, count);
		if (ret) {
			return -EIO;
		}

		for (uint_fast8_t offset = 0;
		     offset < SI4467_PROPERTY_RX_HOP_TABLE_SIZE_MAX;
		     offset += SI4467_PROPERTY_MAX_SET_COUNT) {
			/* Last batch must contain less than 12 elements */
			const size_t batch_size =
				MIN((SI4467_PROPERTY_RX_HOP_TABLE_SIZE_MAX -
				     offset),
				    SI4467_PROPERTY_MAX_SET_COUNT);

			uint8_t tx_bytes[3 + SI4467_PROPERTY_MAX_SET_COUNT] = {
				SI4467_PROPERTY_GROUP_RX_HOP,
				batch_size,
				SI4467_PROPERTY_RX_HOP_TABLE_ENTRY_0 + offset,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
				0xFF,
			};

			/* If any left, set channels be configured. */
			if (offset < count) {
				memcpy(tx_bytes + 3, channels + offset,
				       count - offset);
			}
			ret = si4467_command_send(ctx, COMMAND_SET_PROPERTY,
						  tx_bytes, 3 + batch_size,
						  NULL, 0, true);
			if (ret) {
				return ret;
			}
		}
	}

	/* Enable, reconfigure hopping */
	return si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_RX_HOP,
					   SI4467_PROPERTY_RX_HOP_CONTROL,
					   (((uint8_t)config->enable) << 4) |
						   config->rssi_timeout);
}

int si4467_setup_gpio_pin_config(struct si4467_context *const ctx)
{
	uint8_t gpio_config[] = {
		/* used by CCA function */
		SI4467_GPIO_PIN_CFG_PULL_CTL_EN |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_CCA,
		SI4467_GPIO_PIN_CFG_PULL_CTL_EN |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_CTS,
#if CONFIG_TRX_SI4467_RX_TX_GPIO
		SI4467_GPIO_PIN_CFG_PULL_CTL_EN |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_STATE,
		SI4467_GPIO_PIN_CFG_PULL_CTL_EN |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_STATE,
#else
		SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
		SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
			SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
#endif
	};

	if (si4467_command_gpio_pin_config(
		    ctx, gpio_config,
		    SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
			    SI4467_GPIO_PIN_CFG_GPIO_MODE_DONOTHING,
		    SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
			    SI4467_GPIO_PIN_CFG_GPIO_MODE_DONOTHING,
		    SI4467_GPIO_PIN_CFG_DRV_STRENGTH_MED_LOW)) {
		LOG_ERR("Cannot configure GPIOs.");
		return -EIO;
	}

	return 0;
}

/**
 * @brief Install initial settings as produced by Silicon Labs' Wireless
 * Development Suite (WDS).
 *
 * @param dev
 * @param config Pointer to a RADIO_CONFIGURATION_DATA_ARRAY as produced by WDS.
 * Last element must be zero terminated.
 * @param configuration_bytes Size in bytes for config. While not strictly
 * necessary, this value gets used for validation and prevention of buffer
 * overruns.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_rf_install_settings(struct si4467_context *const ctx,
			       const uint8_t *configuration,
			       const size_t configuration_bytes)
{
	size_t command_size_offset = 0;

	if (configuration_bytes < 5) {
		LOG_WRN("Too small configuration: %d bytes",
			configuration_bytes);
		return 0;
	}

	/* Loop until we hit the zero byte end marker */
	while (configuration[command_size_offset]) {
		/* Size of command including arguments */
		const uint8_t command_size = configuration[command_size_offset];
		const uint8_t command = configuration[command_size_offset + 1];

		switch (command) {
		case COMMAND_SET_PROPERTY: {
			enum {
				GROUP_SUB_OFFSET = 2,
				NUM_PROPS_SUB_OFFSET = 3,
				START_PROP_SUB_OFFSET = 4
			};
			const uint8_t payload_size =
				configuration[command_size_offset +
					      NUM_PROPS_SUB_OFFSET];
			if (payload_size + START_PROP_SUB_OFFSET !=
			    command_size) {
				LOG_ERR("invalid set_property size: %d",
					payload_size);
				return -EINVAL;
			}
			if (si4467_command_set_property_raw(
				    ctx, command_size - sizeof(command),
				    &configuration[command_size_offset +
						   GROUP_SUB_OFFSET])) {
				LOG_ERR("failed to set property");
				return -EIO;
			}
			break;
		}
		default:
			LOG_WRN("unhandled command: 0x%02x", command);
			break;
		}

		command_size_offset += command_size + 1;
		if (command_size_offset >= configuration_bytes) {
			LOG_ERR("Invalid configuration");
			return -EINVAL;
		}
	}

	return 0;
}

/* Timing information taken from AN633: 8.2 Radio Chip Waking Up */
static int si4467_reset_and_boot(struct si4467_context *const ctx)
{
	struct si4467_gpio_configuration sdn_gpio =
		ctx->gpios[SI4467_GPIO_IDX_SDN];

	/* reset transceiver by pulling SDN high */
	if (gpio_pin_set(sdn_gpio.dev, sdn_gpio.pin, 1)) {
		LOG_ERR("Cannot enter shut down");
		return -EIO;
	}

	k_busy_wait(10);

	if (gpio_pin_set(sdn_gpio.dev, sdn_gpio.pin, 0)) {
		LOG_ERR("Cannot exit shut down");
		return -EIO;
	}

	/* wait for power on reset to complete (see section 8.2 of ANN633; this
	 * should typically take 900 us, but can take up to 6 ms)
	 */
	if (si4467_await_cts_gpio(ctx)) {
		LOG_ERR("Cannot await power on reset");
		return -EIO;
	}

	/* send POWER_UP command */
	if (si4467_command_boot_up(ctx)) {
		LOG_ERR("Cannot boot up");
		return -EIO;
	}

	/* wait for boot to complete (see section 8.2 of ANN633; this
	 * should typically take 15 ms)
	 */
	if (si4467_await_cts_gpio(ctx)) {
		LOG_ERR("Cannot await boot complete");
		return -EIO;
	}

	return 0;
}

static int si4467_verify_part_info(struct si4467_context *const ctx)
{
	struct part_information info;

	const int ret = si4467_command_part_info(ctx, &info);

	if (ret) {
		LOG_ERR("Cannot get part info: %d", ret);
		return -EIO;
	}

	LOG_INF("Part information: chiprev=0x%02x, part=0x%04x, pbuild=0x%02x,"
		" id=0x%04x, customer=0x%02x, romid=0x%02x",
		info.chiprev, info.part, info.pbuild, info.id, info.customer,
		info.romid);

	if (info.chiprev != 0x02) {
		LOG_ERR("Unknown chip revision: 0x%02x", info.chiprev);
		return -ENXIO; /* Wanted: Better return value */
	}

	if (info.part != 0x4467) {
		LOG_ERR("Unknown part: 0x%04x", info.part);
		return -ENXIO; /* Wanted: Better return value */
	}

	if (info.romid != 0x6) {
		LOG_ERR("Unknown romid: 0x%02x", info.romid);
		return -ENXIO; /* Wanted: Better return value */
	}

	return 0;
}

/**
 * @brief Initialize and install all GPIOs
 *
 * @param ctx Pointer to the device structure for the driver instance.
 *
 * @return 0 if successful, negative errno code on failure.
 */
static int si4467_configure_gpios(struct si4467_context *const ctx)
{
	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL_BY_IDX(0, gpio_gpios, 0));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_GPIO0].pin =
			DT_INST_GPIO_PIN_BY_IDX(0, gpio_gpios, 0);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_GPIO0].pin,
				   GPIO_INPUT | DT_INST_GPIO_FLAGS_BY_IDX(
							0, gpio_gpios, 0));
		ctx->gpios[SI4467_GPIO_IDX_GPIO0].dev = gpio;
	}

	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL_BY_IDX(0, gpio_gpios, 1));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_GPIO1].pin =
			DT_INST_GPIO_PIN_BY_IDX(0, gpio_gpios, 1);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_GPIO1].pin,
				   GPIO_INPUT | DT_INST_GPIO_FLAGS_BY_IDX(
							0, gpio_gpios, 1));
		ctx->gpios[SI4467_GPIO_IDX_GPIO1].dev = gpio;
	}

	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL_BY_IDX(0, gpio_gpios, 2));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_GPIO2].pin =
			DT_INST_GPIO_PIN_BY_IDX(0, gpio_gpios, 2);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_GPIO2].pin,
				   GPIO_INPUT | DT_INST_GPIO_FLAGS_BY_IDX(
							0, gpio_gpios, 2));
		ctx->gpios[SI4467_GPIO_IDX_GPIO2].dev = gpio;
	}

	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL_BY_IDX(0, gpio_gpios, 3));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_GPIO3].pin =
			DT_INST_GPIO_PIN_BY_IDX(0, gpio_gpios, 3);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_GPIO3].pin,
				   GPIO_INPUT | DT_INST_GPIO_FLAGS_BY_IDX(
							0, gpio_gpios, 3));
		ctx->gpios[SI4467_GPIO_IDX_GPIO3].dev = gpio;
	}

	{
		const struct device *gpio =
			device_get_binding(DT_INST_GPIO_LABEL(0, nirq_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_NIRQ].pin =
			DT_INST_GPIO_PIN(0, nirq_gpios);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_NIRQ].pin,
				   GPIO_INPUT |
					   DT_INST_GPIO_FLAGS(0, nirq_gpios));
		ctx->gpios[SI4467_GPIO_IDX_NIRQ].dev = gpio;
	}

	{
		const struct device *gpio =
			device_get_binding(DT_INST_GPIO_LABEL(0, sdn_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_SDN].pin =
			DT_INST_GPIO_PIN(0, sdn_gpios);
		gpio_pin_configure(gpio, ctx->gpios[SI4467_GPIO_IDX_SDN].pin,
				   GPIO_OUTPUT |
					   DT_INST_GPIO_FLAGS(0, sdn_gpios));
		ctx->gpios[SI4467_GPIO_IDX_SDN].dev = gpio;
	}

#if CONFIG_TRX_SI4467_NIRQ_DEBUG
	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL(0, nirq_handling_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_NIRQ_HANDLING].pin =
			DT_INST_GPIO_PIN(0, nirq_handling_gpios);
		gpio_pin_configure(
			gpio, ctx->gpios[SI4467_GPIO_IDX_NIRQ_HANDLING].pin,
			GPIO_OUTPUT |
				DT_INST_GPIO_FLAGS(0, nirq_handling_gpios));
		ctx->gpios[SI4467_GPIO_IDX_NIRQ_HANDLING].dev = gpio;
	}
#endif

#if CONFIG_TRX_SI4467_RF_CONTROL_RX_HANDLING
	/* RX control pin */
	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL(0, rx_enable_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_RX_ENABLE].pin =
			DT_INST_GPIO_PIN(0, rx_enable_gpios);
		gpio_pin_configure(
			gpio, ctx->gpios[SI4467_GPIO_IDX_RX_ENABLE].pin,
			GPIO_OUTPUT | DT_INST_GPIO_FLAGS(0, rx_enable_gpios));
		ctx->gpios[SI4467_GPIO_IDX_RX_ENABLE].dev = gpio;
	}
#endif

#if CONFIG_TRX_SI4467_RF_CONTROL_TX_HANDLING
	/* TX control pin */
	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL(0, tx_enable_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_TX_ENABLE].pin =
			DT_INST_GPIO_PIN(0, tx_enable_gpios);
		gpio_pin_configure(
			gpio, ctx->gpios[SI4467_GPIO_IDX_TX_ENABLE].pin,
			GPIO_OUTPUT | DT_INST_GPIO_FLAGS(0, tx_enable_gpios));
		ctx->gpios[SI4467_GPIO_IDX_TX_ENABLE].dev = gpio;
	}

#endif

#if CONFIG_TRX_SI4467_ANTENNA_DIVERSITY_HANDLING
	{
		const struct device *gpio = device_get_binding(
			DT_INST_GPIO_LABEL(0, ant_diversity_gpios));

		if (!gpio) {
			return -ENODEV;
		}

		ctx->gpios[SI4467_GPIO_IDX_ANT_DIVERSITY].pin =
			DT_INST_GPIO_PIN(0, ant_diversity_gpios);
		gpio_pin_configure(
			gpio, ctx->gpios[SI4467_GPIO_IDX_ANT_DIVERSITY].pin,
			GPIO_OUTPUT |
				DT_INST_GPIO_FLAGS(0, ant_diversity_gpios));
		ctx->gpios[SI4467_GPIO_IDX_ANT_DIVERSITY].dev = gpio;
	}

#endif

	return si4467_setup_gpio_callback(ctx);
}

static int si4467_configure_spi(struct si4467_context *const ctx)
{
	ctx->spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!ctx->spi) {
		LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

	cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	if (!cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	cs_ctrl.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0);
	cs_ctrl.delay = 0U; /* At least 20 ns -> 0 microseconds */

	ctx->spi_cfg.cs = &cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
		DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
		DT_INST_SPI_DEV_CS_GPIOS_PIN(0));

	ctx->spi_cfg.operation = SPI_WORD_SET(8);
	ctx->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	ctx->spi_cfg.slave = DT_INST_REG_ADDR(0);

	return 0;
}

int si4467_command_get_xo_tune_value(struct si4467_context *const ctx,
				     uint8_t *tune_value)
{
	return si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_GLOBAL,
					     SI4467_PROPERTY_GLOBAL_XO_TUNE,
					     tune_value, sizeof(*tune_value));
}

int si4467_command_set_xo_tune_value(struct si4467_context *const ctx,
				     const uint8_t tune_value)
{
	if (tune_value > 127) {
		LOG_ERR("invalid tune value: %u", tune_value);
		return -EINVAL;
	}

	if (si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_GLOBAL,
					SI4467_PROPERTY_GLOBAL_XO_TUNE,
					tune_value)) {
		LOG_ERR("failed to set tune value");
		return -EIO;
	}

	return 0;
}

#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE
int si4467_save_calibration(struct si4467_context *const ctx)
{
	int ret;
	uint8_t tune_value;

	if (si4467_command_get_xo_tune_value(ctx, &tune_value)) {
		LOG_ERR("failed to get tune value");
		return -EIO;
	}

	ret = settings_subsys_init();
	if (ret) {
		LOG_ERR("settings subsys initialization: %d", ret);
		return -EIO;
	}

	ret = settings_save_one(CONFIG_TRX_SI4467_CALIBRATION_SETTING_NAME,
				&tune_value, sizeof(uint8_t));
	if (ret) {
		LOG_ERR("failed to save settings: %d", ret);
		return -EIO;
	}

	return 0;
}

static int clock_calibration_settings_load_direct_cb(const char *key,
						     size_t len,
						     settings_read_cb read_cb,
						     void *cb_arg, void *param)
{
	uint8_t tune_val;
	int ret;
	struct si4467_context *const ctx = (struct si4467_context *)param;

	/*
	 * full name used in restore, so should be empty
	 *
	 * note: in ztest, we get NULL here, rather than an empty string (not
	 * sure why; possibly due to ztest using NVS rather than FS)
	 */
	if (key != NULL && strlen(key) != 0) {
		LOG_WRN("unknown key");
		return 0; /* return 0 to continue processing */
	}

	if (len != 1) {
		LOG_ERR("unexpected setting data size");
		return -EINVAL;
	}

	ret = read_cb(cb_arg, &tune_val, sizeof(uint8_t));
	if (ret < 0) {
		LOG_ERR("read_cb failed: %d", ret);
		return -EIO;
	}

	if (ret != 1 || tune_val > 127) {
		LOG_ERR("invalid length or calibration setting value: %d/%d",
			ret, tune_val);
		return -EINVAL;
	}

	LOG_INF("applying calibration tune value: %u", tune_val);

	if (si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_GLOBAL,
					SI4467_PROPERTY_GLOBAL_XO_TUNE,
					tune_val)) {
		LOG_ERR("failed to set tune value");
		return -EIO;
	}

	return 0;
}

int si4467_restore_calibration(struct si4467_context *const ctx)
{
	int ret;

	ret = settings_subsys_init(); /* OK if done multiple times */
	if (ret) {
		LOG_ERR("settings subsys initialization: %d", ret);
		return -EIO;
	}

	ret = settings_load_subtree_direct(
		CONFIG_TRX_SI4467_CALIBRATION_SETTING_NAME,
		clock_calibration_settings_load_direct_cb, (void *)ctx);
	if (ret) {
		LOG_ERR("failed to load settings: %d", ret);
		return -EIO;
	}

	return 0;
}
#endif

/* TODO experimentally determined; reevaluate this, maybe make it dynamic */
const uint8_t rssi_threshold = 85;
int si4467_cca(const struct device *dev)
{
	int ret;
	uint8_t frr_value;
	struct si4467_context *const ctx = dev->data;

	/* make sure transceiver is not in RX state (otherwise, no new RSSI
	 * value will be latched)
	 */
	ret = si4467_command_read_fast_response_register(ctx, SI4467_FRR_B,
							 &frr_value);
	if (ret) {
		return ret;
	}

	/* XXX this should be synchronized rather than just setting another */
	if (frr_value == SI4467_OPERATING_STATE_RX) {
		si4467_command_change_state(ctx, SI4467_OPERATING_STATE_READY);
	}

	/* switch transceiver to RX state */
	si4467_command_change_state(ctx, SI4467_OPERATING_STATE_RX);

	/* wait for at least 7*Tb to allow latching of a new value */
	k_usleep(100);

	/* get new RSSI value & check if over threshold */
	ret = si4467_command_read_fast_response_register(ctx, SI4467_FRR_A,
							 &frr_value);
	if (ret) {
		return ret;
	}

	LOG_DBG("%s: RSSI is %u", __func__, frr_value);

	return frr_value >= rssi_threshold;
}

int si4467_start(const struct device *dev)
{
	struct si4467_context *const ctx = dev->data;
	int ret = 0;

	k_mutex_lock(&ctx->dev_lock, K_FOREVER);
	if (si4467_reset_and_boot(ctx)) {
		LOG_ERR("Configuring Si4467 failed");
		goto error;
	}

	if (si4467_verify_part_info(ctx)) {
		LOG_ERR("Verifying part information failed");
		goto error;
	}

	if (si4467_setup_gpio_pin_config(ctx)) {
		LOG_ERR("Configuring Si4467 GPIO pins failed");
		goto error;
	}

	if (si4467_rf_install_settings(ctx, ctx->radio_configuration_data,
				       ctx->radio_configuration_data_size)) {
		LOG_ERR("Configuring properties failed");
		goto error;
	}
	k_mutex_unlock(&ctx->dev_lock);

	ret = si4467_enable_nirq_interrupt(ctx, true);
	if (ret) {
		LOG_ERR("Enabling nIRQ handler failed: %d", ret);
		goto error;
	}

	LOG_INF("Transceiver started");

	return 0;
error:
	k_mutex_unlock(&ctx->dev_lock);
	return -EIO;
}

bool si4467_chip_interrupt_present(const struct chip_interrupt i)
{
	/* For obvious reasons, field "unused" not checked here */
	return i.cal || i.fifo_underflow_overflow_error || i.state_change ||
	       i.cmd_error || i.chip_ready || i.low_batt || i.wut;
}

bool si4467_modem_interrupt_present(const struct modem_interrupt i)
{
	return i.rssi_latch || i.postamble_detect || i.invalid_sync ||
	       i.rssi_jump || i.rssi || i.invalid_preamble ||
	       i.preamble_detect || i.sync_detect;
}

bool si4467_packet_handler_interrupt_present(
	const struct packet_handler_interrupt i)
{
	return i.filter_match || i.filter_miss || i.packet_sent ||
	       i.packet_rx || i.crc_error || i.alt_crc_error ||
	       i.tx_fifo_almost_empty || i.rx_fifo_almost_full;
}

int si4467_fill_tx_fifo_buffer(struct si4467_context *const ctx,
			       struct net_pkt *pkt)
{
	uint8_t sent = 0;
	uint8_t chunk;
	int ret;
	struct fifo_information fifo_info;
	const size_t remaining = net_pkt_remaining_data(pkt);

	/* Exit early if nothing to do */
	if (remaining == 0) {
		return 0;
	}

	/*
	 * Enabling overwrite prevents buffers len from being extended
	 * when skipping data. Feels wrong to do so but after reading existing
	 * code, this seems the way to go, even when just reading from net_pkt.
	 */
	const bool overwrite_backup = net_pkt_is_being_overwritten(pkt);

	net_pkt_set_overwrite(pkt, true);

	/* Determine number of bytes available in TX FIFO */
	ret = si4467_command_fifo_info(ctx, 0, 0, &fifo_info);
	if (ret) {
		ret = -EIO;
		goto out_error;
	}

	/* Loop to allow crossing pkt fragments */
	for (uint8_t left = MIN(fifo_info.tx_fifo_space, remaining); left > 0;
	     left -= chunk, sent += chunk) {
		/*
		 * Determine maximum number of bytes accessible contiguously.
		 * Using a separate variable to prevent superfluous calls during
		 * macro expansion.
		 */
		const size_t contiguous = net_pkt_get_contiguous_len(pkt);

		if (contiguous == 0) {
			__ASSERT(false, "Need contiguous data to operate on");
			ret = -EINVAL;
			goto out_error;
		}

		/* The lower of both values dictates the maximum chunk size */
		chunk = MIN(left, contiguous);

		LOG_HEXDUMP_DBG(pkt->cursor.buf, chunk, "TX FIFO write");

		/*
		 * Not using si4467_fill_tx_fifo_buffer_raw() to avoid the
		 * overhead of querying the available FIFO space again.
		 */
		ret = si4467_command_send(ctx, COMMAND_WRITE_TX_FIFO,
					  net_pkt_cursor_get_pos(pkt), chunk,
					  NULL, 0, false);
		if (ret) {
			goto out_error;
		}

		/*
		 * Move cursor by number of bytes sent to the TX FIFO.
		 *
		 * Enabling overwrite prevents buffers len from potentially
		 * being extended when skipping data. Feels wrong to do so but
		 * seems needed, even when just reading from the packet.
		 */
		ret = net_pkt_skip(pkt, chunk);
		if (ret < 0) {
			LOG_ERR("Failed to skip %zu bytes", chunk);
			return ret;
		}
	}

	LOG_DBG("Sent %zu bytes to TX FIFO", sent);

	ret = sent;

out_error:
	net_pkt_set_overwrite(pkt, overwrite_backup);

	return ret;
}

int si4467_fetch_rx_fifo_buffer(struct si4467_context *const ctx,
				struct net_pkt *pkt)
{
	int ret;
	struct fifo_information fifo_info;
	size_t read = 0;

	ret = si4467_command_fifo_info(ctx, 0, 0, &fifo_info);
	if (ret) {
		LOG_ERR("Cannot execute command FIFO_INFO");
		return -EIO;
	}

	const uint8_t fifo_bytes_available = fifo_info.rx_fifo_count;
	const uint16_t buffer_available = net_pkt_available_buffer(pkt);
	uint8_t chunk;

	/* Loop to allow crossing pkt fragments, command size read limits */
	for (uint8_t left = MIN(fifo_bytes_available, buffer_available);
	     left > 0; left -= chunk, read += chunk) {
		/*
		 * Determine maximum number of bytes accessible contiguously.
		 * Using a separate variable to prevent superfluous calls during
		 * macro expansion.
		 */
		const size_t contiguous = net_pkt_get_contiguous_len(pkt);

		if (contiguous == 0) {
			__ASSERT(false, "Need contiguous data to operate on");
			return -EINVAL;
		}

		/* Limited by the length of contiguous data in the RX buffer */
		chunk = MIN(left, contiguous);

		/* Limited by the max command RX size of the Si4467 */
		chunk = MIN(chunk, SI4467_MAX_COMMAND_RESPONSE_SIZE);

		ret = si4467_command_read_rx_fifo(
			ctx, net_pkt_cursor_get_pos(pkt), chunk);
		if (ret) {
			return ret;
		}

		ret = net_pkt_skip(pkt, chunk);
		if (ret < 0) {
			LOG_ERR("Failed to skip %zu bytes", chunk);
			return ret;
		}
	}

	return read;
}

ssize_t si4467_drain_rx_fifo_data(struct si4467_context *const ctx)
{
	int ret;
	struct fifo_information fifo_info;

	ret = si4467_command_fifo_info(ctx, 0, 0, &fifo_info);
	if (ret) {
		LOG_ERR("Cannot execute command FIFO_INFO");
		return ret;
	}
	LOG_DBG("RX FIFO count: %d", fifo_info.rx_fifo_count);
	LOG_DBG("TX FIFO space: %d", fifo_info.tx_fifo_space);

	if (fifo_info.rx_fifo_count == 0) {
		return 0;
	}

	uint8_t rx_buff[fifo_info.rx_fifo_count];
	uint8_t bytes_left = fifo_info.rx_fifo_count;
	uint8_t bytes_read = 0;

	memset(rx_buff, bytes_read, sizeof(rx_buff));
	do {
		const uint8_t bytes_to_read =
			MIN(bytes_left, SI4467_MAX_COMMAND_RESPONSE_SIZE);
		bytes_left -= bytes_to_read;
		ret = si4467_command_read_rx_fifo(ctx, rx_buff + bytes_read,
						  bytes_to_read);
		if (ret) {
			return ret;
		}
		bytes_read += bytes_to_read;
	} while (bytes_left);

	LOG_HEXDUMP_INF(rx_buff, fifo_info.rx_fifo_count, "RX FIFO data");

	return fifo_info.rx_fifo_count;
}

static void si4467_thread_nirq(struct si4467_context *const ctx,
			       si4467_thread_nirq_callback_t nirq_handler)
{
	while (true) {
		k_sem_take(&ctx->isr_nirq_sem, K_FOREVER);

#if CONFIG_TRX_SI4467_NIRQ_DEBUG
		struct si4467_gpio_configuration nirq_handling_gpio =
			ctx->gpios[SI4467_GPIO_IDX_NIRQ_HANDLING];

		if (gpio_pin_set(nirq_handling_gpio.dev, nirq_handling_gpio.pin,
				 1)) {
			LOG_ERR("Could not set nIRQ handling pin");
		}
#endif

		nirq_handler(ctx);

#if CONFIG_TRX_SI4467_NIRQ_DEBUG
		if (gpio_pin_set(nirq_handling_gpio.dev, nirq_handling_gpio.pin,
				 0)) {
			LOG_ERR("Could not release nIRQ handling pin");
		}
#endif
	}
}

uint32_t si4467_get_xtal_frequency(void)
{
	return DT_INST_PROP(0, xtal_frequency);
}

int si4467_init(const struct device *dev,
		si4467_thread_nirq_callback_t si4467_thread_nirq_fn)
{
	struct si4467_context *const ctx = dev->data;

	k_sem_init(&ctx->isr_nirq_sem, 0, 1);
	k_mutex_init(&ctx->dev_lock);

	if (si4467_configure_gpios(ctx)) {
		LOG_ERR("Configuring GPIOS failed");
		return -EIO;
	}
	LOG_DBG("GPIO configured");

	if (si4467_configure_spi(ctx) != 0) {
		LOG_ERR("Configuring SPI failed");
		return -EIO;
	}
	LOG_DBG("SPI configured");

	k_thread_create(&ctx->thread_nirq, ctx->rx_stack,
			CONFIG_TRX_SI4467_RX_STACK_SIZE,
			(k_thread_entry_t)si4467_thread_nirq, ctx,
			si4467_thread_nirq_fn, NULL, K_PRIO_COOP(10), 0,
			K_NO_WAIT);
	k_thread_name_set(&ctx->thread_nirq, "si4467_nirq");

	LOG_INF("Si4467 initialized");

	return 0;
}

/*
 * Network device initialization happens before the file system is initialized;
 * as calibration data comes from settings, which may be stored on a file
 * system, we can't put the following in si4467_init() or si4467_start().
 *
 * This is somewhat ugly & assumption that si4467 is the first interface is not
 * necessarily correct; a better solution would be welcome (if we switch to NVS
 * rather than using FS for settings, si4467_restore_calibration() can be called
 * from si4467_start()).
 */
#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE && !CONFIG_ZTEST
static int si4467_restore_calibration_init(const struct device *init_dev)
{
	ARG_UNUSED(init_dev);

	struct net_if *iface = net_if_get_by_index(1);
	const struct device *dev = net_if_get_device(iface);
	struct si4467_context *const ctx = dev->data;

	return si4467_restore_calibration(ctx);
}

/*
 * littlefs init happens on level POST_KERNEL with prio 99 (see littlefs_fs.c),
 * therefore we should be good with APPLICATION level.
 */
SYS_INIT(si4467_restore_calibration_init, APPLICATION,
	 CONFIG_APPLICATION_INIT_PRIORITY);
#endif
