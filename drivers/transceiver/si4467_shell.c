/*
 * Copyright (c) 2020 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file
 * @brief Si4467 shell module
 */

#define LOG_MODULE_NAME si4467_shell
#define LOG_LEVEL CONFIG_TRX_DRIVER_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <string.h>
#include <drivers/gpio.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
#include <net/buf.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <sys/printk.h>
#include <sys/crc.h>

#include "drivers/transceiver/si4467.h"

#define ARRAY_AND_LENGTH(var) var, ARRAY_SIZE(var)

struct property_field_definition {
	const char *const name;
	const char *const *const options;
	const uint8_t num_options;
	const uint8_t bit_index; /* LSB */
	const uint8_t length; /* bits */
};

#if CONFIG_MINIMAL_LIBC
#define strerror(...) "strerror() not available in MINIMAL_LIBC"
#endif /* CONFIG_MINIMAL_LIBC */

static const struct device *get_device(void)
{
	struct net_if *iface = net_if_get_by_index(1);
	const struct device *dev = net_if_get_device(iface);

	return dev;
}

static struct si4467_context *get_context(void)
{
	return get_device()->data;
}

static void print_property(const struct shell *shell, const uint32_t value,
			   const uint8_t num_fields,
			   const struct property_field_definition fields[])
{
	for (uint8_t i = 0; i < num_fields; i++) {
		uint32_t const field_val = (value >> fields[i].bit_index) &
					   ((1 << fields[i].length) - 1);
		if (field_val < fields[i].num_options) {
			shell_print(shell, "- %s: %s (%u)", fields[i].name,
				    fields[i].options[field_val], field_val);
		} else {
			shell_print(shell, "- %s: invalid value: %u",
				    fields[i].name, field_val);
		}
	}
}

/**
 * @brief Update property value from shell arguments.
 *
 * @param shell pointer to shell
 * @param property pointer to property
 * @param num_fields number of fields in property
 * @param fields property field definitions
 * @param argc command argument count
 * @param argv command arguments
 *
 * @return 0 if there was no change, 1 if the property was updated, negative
 * value in case of error
 */
static int update_property(const struct shell *shell, uint8_t *property,
			   const uint8_t num_fields,
			   const struct property_field_definition fields[],
			   const int argc, const char *argv[])
{
	uint8_t new_value = *property;

	for (uint8_t argidx = 1; argidx < argc; argidx++) {
		bool found = false;

		for (uint8_t i = 0; i < num_fields && !found; i++) {
			for (uint8_t j = 0; j < fields[i].num_options && !found;
			     j++) {
				if (strcmp(argv[argidx],
					   fields[i].options[j]) == 0) {
					found = true;
					new_value &=
						~(((1 << fields[i].length) - 1)
						  << fields[i].bit_index);
					new_value |= j << fields[i].bit_index;
				}
			}
		}

		if (!found) {
			shell_error(shell, "invalid argument: %s",
				    argv[argidx]);
			return -EINVAL;
		}
	}

	if (new_value != *property) {
		*property = new_value;
		return 1;
	}

	return 0;
}

static int cmd_si4467_fast_response_register(const struct shell *shell,
					     size_t argc, char *argv[])
{
	enum fast_response_register frr;
	uint8_t register_value;

	struct si4467_context *ctx = get_context();

	if (strlen(argv[1]) > 1) {
		shell_error(shell, "invalid register");
		return -EINVAL;
	}

	switch (argv[1][0]) {
	case 'a':
	case 'A':
		frr = SI4467_FRR_A;
		break;
	case 'b':
	case 'B':
		frr = SI4467_FRR_B;
		break;
	case 'c':
	case 'C':
		frr = SI4467_FRR_C;
		break;
	case 'd':
	case 'D':
		frr = SI4467_FRR_D;
		break;
	default:
		shell_error(shell, "invalid register");
		return -EINVAL;
	}

	if (si4467_command_read_fast_response_register(ctx, frr,
						       &register_value)) {
		shell_error(shell, "failed to read FRR");
		return -EIO;
	}

	shell_print(shell, "FRR value: 0x%02x", register_value);

	return 0;
}

static void shell_print_modem_status_reply(const struct shell *shell,
					   struct modem_status_reply status)
{
	const struct modem_interrupt interrupt_pending =
		status.interrupt_pending;
	const struct modem_interrupt interrupt_status = status.interrupt_status;

	if (!si4467_modem_interrupt_present(interrupt_pending) &&
	    !si4467_modem_interrupt_present(interrupt_status)) {
		return;
	}

	shell_fprintf(shell, SHELL_NORMAL, "Modem interrupts pending/status:");
	if (interrupt_pending.rssi_latch || interrupt_status.rssi_latch) {
		shell_fprintf(shell, SHELL_NORMAL, " rssi_latch=%u/%u",
			      interrupt_pending.rssi_latch,
			      interrupt_status.rssi_latch);
	}
	if (interrupt_pending.postamble_detect ||
	    interrupt_status.postamble_detect) {
		shell_fprintf(shell, SHELL_NORMAL, " postamble_detect=%u/%u",
			      interrupt_pending.postamble_detect,
			      interrupt_status.postamble_detect);
	}
	if (interrupt_pending.invalid_sync || interrupt_status.invalid_sync) {
		shell_fprintf(shell, SHELL_NORMAL, " invalid_sync=%u/%u",
			      interrupt_pending.invalid_sync,
			      interrupt_status.invalid_sync);
	}
	if (interrupt_pending.rssi_jump || interrupt_status.rssi_jump) {
		shell_fprintf(shell, SHELL_NORMAL, " rssi_jump=%u/%u",
			      interrupt_pending.rssi_jump,
			      interrupt_status.rssi_jump);
	}
	if (interrupt_pending.rssi || interrupt_status.rssi) {
		shell_fprintf(shell, SHELL_NORMAL, " rssi=%u/%u",
			      interrupt_pending.rssi, interrupt_status.rssi);
	}
	if (interrupt_pending.invalid_preamble ||
	    interrupt_status.invalid_preamble) {
		shell_fprintf(shell, SHELL_NORMAL, " invalid_preamble=%u/%u",
			      interrupt_pending.invalid_preamble,
			      interrupt_status.invalid_preamble);
	}
	if (interrupt_pending.preamble_detect ||
	    interrupt_status.preamble_detect) {
		shell_fprintf(shell, SHELL_NORMAL, " preamble_detect=%u/%u",
			      interrupt_pending.preamble_detect,
			      interrupt_status.preamble_detect);
	}
	if (interrupt_pending.sync_detect || interrupt_status.sync_detect) {
		shell_fprintf(shell, SHELL_NORMAL, " sync_detect=%u/%u",
			      interrupt_pending.sync_detect,
			      interrupt_status.sync_detect);
	}
	shell_fprintf(shell, SHELL_NORMAL, "\n");
}

static void shell_print_modem_status_reply_detailed(
	const struct shell *shell, struct modem_status_reply_detailed status)
{
	shell_print_modem_status_reply(shell, status.modem_status);

	shell_print(shell, "curr_rssi: %u", status.curr_rssi);
	shell_print(shell, "latch_rssi: %u", status.latch_rssi);
	shell_print(shell, "afc_freq_offset: %u", status.afc_freq_offset);

	if (status.modem_status.interrupt_status.sync_detect) {
		shell_print(shell, "ant1_rssi: %u", status.ant1_rssi);
		shell_print(shell, "ant2_rssi: %u", status.ant2_rssi);
		shell_print(shell, "info_flags: 0x%02x", status.info_flags);
	}
}

static int cmd_si4467_modem_status(const struct shell *shell, size_t argc,
				   char *argv[])
{
	struct si4467_context *ctx = get_context();
	/* Per default, do preserve the status */
	const bool preserve = argv[1][0] != '1';
	const struct modem_interrupt preserve_what = { preserve, preserve,
						       preserve, preserve,
						       preserve, preserve,
						       preserve, preserve };
	struct modem_status_reply_detailed status;

	if (si4467_command_get_modem_status(ctx, preserve_what, &status)) {
		shell_error(shell, "Getting modem status failed");
		return -EIO;
	}

	shell_print_modem_status_reply_detailed(shell, status);

	return 0;
}

static void shell_print_chip_status_reply(const struct shell *shell,
					  const struct chip_status_reply status)
{
	const struct chip_interrupt interrupt_pending =
		status.interrupt_pending;
	const struct chip_interrupt interrupt_status = status.interrupt_status;

	if (!si4467_chip_interrupt_present(interrupt_pending) &&
	    !si4467_chip_interrupt_present(interrupt_status)) {
		return;
	}

	shell_fprintf(shell, SHELL_NORMAL, "Chip interrupts pending/status:");
	if (interrupt_pending.cal || interrupt_status.cal) {
		shell_fprintf(shell, SHELL_NORMAL, " cal=%u/%u",
			      interrupt_pending.cal, interrupt_status.cal);
	}
	if (interrupt_pending.fifo_underflow_overflow_error ||
	    interrupt_status.fifo_underflow_overflow_error) {
		shell_fprintf(shell, SHELL_NORMAL,
			      " fifo_underflow_overflow_error=%u/%u",
			      interrupt_pending.fifo_underflow_overflow_error,
			      interrupt_status.fifo_underflow_overflow_error);
	}
	if (interrupt_pending.state_change || interrupt_status.state_change) {
		shell_fprintf(shell, SHELL_NORMAL, " state_change=%u/%u",
			      interrupt_pending.state_change,
			      interrupt_status.state_change);
	}
	if (interrupt_pending.cmd_error || interrupt_status.cmd_error) {
		shell_fprintf(shell, SHELL_NORMAL, " cmd_error=%u/%u",
			      interrupt_pending.cmd_error,
			      interrupt_status.cmd_error);
	}
	if (interrupt_pending.chip_ready || interrupt_status.chip_ready) {
		shell_fprintf(shell, SHELL_NORMAL, " chip_ready=%u/%u",
			      interrupt_pending.chip_ready,
			      interrupt_status.chip_ready);
	}
	if (interrupt_pending.low_batt || interrupt_status.low_batt) {
		shell_fprintf(shell, SHELL_NORMAL, " low_batt=%u/%u",
			      interrupt_pending.low_batt,
			      interrupt_status.low_batt);
	}
	if (interrupt_pending.wut || interrupt_status.wut) {
		shell_fprintf(shell, SHELL_NORMAL, " wut=%u/%u",
			      interrupt_pending.wut, interrupt_status.wut);
	}
	shell_fprintf(shell, SHELL_NORMAL, "\n");
}

static void shell_print_chip_status_reply_detailed(
	const struct shell *shell,
	const struct chip_status_reply_detailed status)
{
	shell_print_chip_status_reply(shell, status.chip_status);

	if (status.chip_status.interrupt_status.cmd_error) {
		shell_print(shell, "Command error status: 0x%02x",
			    status.command_error_status);
		shell_print(shell, "Command error id: 0x%02x",
			    status.command_error_command_id);
	}

	if (status.chip_status.interrupt_status.cal) {
		shell_print(shell, "Calibration type: %u", status.calibration);
	}
}

static void shell_print_chip_status_device_state(
	const struct shell *shell,
	const struct request_device_state_reply state)
{
	shell_print(shell, "Device state: %d (%s)", state.curr_state,
		    request_device_state_reply_to_string(state.curr_state));
	shell_print(shell, "Current channel: %u", state.curr_channel);
}

static int cmd_si4467_chip_status(const struct shell *shell, size_t argc,
				  char *argv[])
{
	struct si4467_context *ctx = get_context();
	/* Per default, do preserve the status */
	const bool preserve = argv[1][0] != '1';
	const struct chip_interrupt preserve_what = { preserve, preserve,
						      preserve, preserve,
						      preserve, preserve,
						      preserve, preserve };
	struct chip_status_reply_detailed status;

	if (si4467_command_get_chip_status(ctx, preserve_what, &status)) {
		shell_error(shell, "Getting chip status failed");
		return -EIO;
	}

	shell_print_chip_status_reply_detailed(shell, status);

	if (status.chip_status.interrupt_status.state_change) {
		struct request_device_state_reply reply;

		if (si4467_command_request_device_state(ctx, &reply)) {
			shell_error(shell, "failed to get device state");
			return -EIO;
		}
		shell_print_chip_status_device_state(shell, reply);
	}

	return 0;
}

static void shell_print_packet_handler_status_reply(
	const struct shell *shell,
	const struct packet_handler_status_reply status)
{
	if (!si4467_packet_handler_interrupt_present(
		    status.interrupt_pending) &&
	    !si4467_packet_handler_interrupt_present(status.interrupt_status)) {
		return;
	}

	shell_fprintf(shell, SHELL_NORMAL,
		      "Packet handler interrupts pending/status:");
	if (status.interrupt_pending.filter_match ||
	    status.interrupt_status.filter_match) {
		shell_fprintf(shell, SHELL_NORMAL, " filter_match=%u/%u",
			      status.interrupt_pending.filter_match,
			      status.interrupt_status.filter_match);
	}
	if (status.interrupt_pending.filter_miss ||
	    status.interrupt_status.filter_miss) {
		shell_fprintf(shell, SHELL_NORMAL, " filter_miss=%u/%u",
			      status.interrupt_pending.filter_miss,
			      status.interrupt_status.filter_miss);
	}
	if (status.interrupt_pending.packet_sent ||
	    status.interrupt_status.packet_sent) {
		shell_fprintf(shell, SHELL_NORMAL, " packet_sent=%u/%u",
			      status.interrupt_pending.packet_sent,
			      status.interrupt_status.packet_sent);
	}
	if (status.interrupt_pending.packet_rx ||
	    status.interrupt_status.packet_rx) {
		shell_fprintf(shell, SHELL_NORMAL, " packet_rx=%u/%u",
			      status.interrupt_pending.packet_rx,
			      status.interrupt_status.packet_rx);
	}
	if (status.interrupt_pending.crc_error ||
	    status.interrupt_status.crc_error) {
		shell_fprintf(shell, SHELL_NORMAL, " crc_error=%u/%u",
			      status.interrupt_pending.crc_error,
			      status.interrupt_status.crc_error);
	}
	if (status.interrupt_pending.alt_crc_error ||
	    status.interrupt_status.alt_crc_error) {
		shell_fprintf(shell, SHELL_NORMAL, " alt_crc_error=%u/%u",
			      status.interrupt_pending.alt_crc_error,
			      status.interrupt_status.alt_crc_error);
	}
	if (status.interrupt_pending.tx_fifo_almost_empty ||
	    status.interrupt_status.tx_fifo_almost_empty) {
		shell_fprintf(shell, SHELL_NORMAL,
			      " tx_fifo_almost_empty=%u/%u",
			      status.interrupt_pending.tx_fifo_almost_empty,
			      status.interrupt_status.tx_fifo_almost_empty);
	}
	if (status.interrupt_pending.rx_fifo_almost_full ||
	    status.interrupt_status.rx_fifo_almost_full) {
		shell_fprintf(shell, SHELL_NORMAL, " rx_fifo_almost_full=%u/%u",
			      status.interrupt_pending.rx_fifo_almost_full,
			      status.interrupt_status.rx_fifo_almost_full);
	}
	shell_fprintf(shell, SHELL_NORMAL, "\n");
}

static int cmd_si4467_packet_handler_status(const struct shell *shell,
					    size_t argc, char *argv[])
{
	struct si4467_context *ctx = get_context();
	/* Per default, do preserve the status */
	const bool preserve = argv[1][0] != '1';
	const struct packet_handler_interrupt preserve_what = {
		preserve, preserve, preserve, preserve,
		preserve, preserve, preserve, preserve
	};
	struct packet_handler_status_reply status;

	if (si4467_command_get_packet_handler_status(ctx, preserve_what,
						     &status)) {
		shell_error(shell, "Getting packet handler status failed");
		return -EIO;
	}

	shell_print_packet_handler_status_reply(shell, status);

	return 0;
}

static int cmd_si4467_rx_fifo_data(const struct shell *shell, size_t argc,
				   char *argv[])
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct si4467_context *ctx = get_context();

	return si4467_drain_rx_fifo_data(ctx);
}

#define MODULATION_HELP                                                     \
	("[TX direct mode] [TX direct mode GPIO] [mod source] [mod type]\n" \
	 "Print [and configure] modulation settings\n"                      \
	 "\n"                                                               \
	 "TX direct mode: data processing mode\n"                           \
	 "- SYNC: synchronous mode\n"                                       \
	 "- ASYNC: asynchronous mode\n"                                     \
	 "TX direct mode GPIO: GPIO for direct mode [0-3]\n"                \
	 "mod source: TX data source\n"                                     \
	 "- PACKET: data from TX FIFO\n"                                    \
	 "- DIRECT: data from GPIO pin\n"                                   \
	 "- PSEUDO: data from pseudo-random generator\n"                    \
	 "mod type: modulation format\n"                                    \
	 "- CW: continuous wave (TX only)\n"                                \
	 "- OOK: on-off keying\n"                                           \
	 "- 2FSK: 2-symbol frequency shift keying\n"                        \
	 "- 2GFSK: 2-symbol Gaussian FSK\n"                                 \
	 "- 4FSK: 4-symbol FSK\n"                                           \
	 "- 4GFSK: 4-symbol Gaussian FSK\n")
static int cmd_si4467_modulation(const struct shell *shell, size_t argc,
				 char *argv[])
{
	const static char *const field_options_mode_type[] = { "ASYNC",
							       "SYNC" };
	const static char *const field_options_mode_gpio[] = { "GPIO0", "GPIO1",
							       "GPIO2",
							       "GPIO3" };
	const static char *const field_options_mod_source[] = { "PACKET",
								"DIRECT",
								"PSEUDO" };
	const static char *const field_options_mod_type[] = { "CW",   "OOK",
							      "2FSK", "2GFSK",
							      "4FSK", "4GFSK" };

	const struct property_field_definition fields[] = {
		{ "TX direct mode", ARRAY_AND_LENGTH(field_options_mode_type),
		  7, 1 },
		{ "TX direct mode GPIO",
		  ARRAY_AND_LENGTH(field_options_mode_gpio), 5, 2 },
		{ "mod source", ARRAY_AND_LENGTH(field_options_mod_source), 3,
		  2 },
		{ "mod type", ARRAY_AND_LENGTH(field_options_mod_type), 0, 3 }
	};
	enum { num_fields = ARRAY_SIZE(fields) };

	int ret;
	uint8_t modem_mod_type;

	struct si4467_context *ctx = get_context();

	/* get current settings */
	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_MODEM,
					    SI4467_PROPERTY_MODEM_MOD_TYPE,
					    &modem_mod_type, 1);
	if (ret) {
		return -EIO;
	}

	/* print current settings */
	shell_print(shell, "current MODEM_MOD_TYPE = 0x%02x", modem_mod_type);
	print_property(shell, modem_mod_type, num_fields, fields);

	/* scan arguments & determine new settings */
	ret = update_property(shell, &modem_mod_type, num_fields, fields, argc,
			      (const char **)argv);
	if (ret < 0) {
		return ret;
	}

	/* changed? */
	if (ret > 0) {
		/* set new settings */
		si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_MODEM,
					    SI4467_PROPERTY_MODEM_MOD_TYPE,
					    modem_mod_type);

		/* print new settings */
		shell_print(shell, "new MODEM_MOD_TYPE = 0x%02x",
			    modem_mod_type);
		print_property(shell, modem_mod_type, num_fields, fields);
	} else {
		shell_print(shell, "MODEM_MOD_TYPE unchanged");
	}

	return 0;
}

#define RX_HOPPING_HELP                                                        \
	("[<condition> <rssi timeout> [<channel #1> [... <channel #64>]]\n"    \
	 "Print [and configure] RX hopping channel\n"                          \
	 "- condition: 0 Disable hopping\n"                                    \
	 "             1 Hop on preamble timeout\n"                            \
	 "             2 Hop on preamble or RSSI timeout\n"                    \
	 "             3 Hop on preamble timeout or invalid sync word\n"       \
	 "             4 Hop on preamble or RSSI timeout, invalid sync word\n" \
	 "- rssi timeout: RSSI timeout period (0-15) in nibbles (4 bits)\n"    \
	 "- channels: 1 to 64 Si4467 channels (maybe limited by shell)\n")

static int cmd_si4467_rx_hopping(const struct shell *shell, size_t argc,
				 char *argv[])
{
	struct si4467_context *ctx = get_context();
	struct si4467_property_group_rx_hop_control hop_control;
	uint8_t channel_table[0x40];
	uint8_t channel_table_size = 0;
	int ret;
	struct si4467_property_group_rx_hop_control config = {};
	char *position;

	if (argc > 1) {
		const int enable = strtoul(argv[1], &position, 10);

		if (enable < 0 || enable > 4 || position == argv[1]) {
			shell_error(shell, "Invalid HOP_EN: %d", enable);
			return -EINVAL;
		}
		config.enable = enable;
	}

	if (argc == 2) {
		/* Only disabling can be done with a single argument */
		if (config.enable !=
		    SI4467_PROPERTY_RX_HOP_CONTROL_EN_DISABLE) {
			shell_error(shell, "RSSI timeout missing");
			return -EINVAL;
		}

		ret = si4467_configure_rx_hopping(ctx, &config, NULL, 0);
		if (ret) {
			shell_error(shell, "Failed to disable RX hopping: %d",
				    ret);
			return -EIO;
		}
	}

	if (argc >= 3) {
		const int rssi_timeout = strtoul(argv[2], &position, 10);

		/* Max allowed value is 15 */
		if (rssi_timeout < 0 || rssi_timeout > 0x0F ||
		    position == argv[2]) {
			shell_error(shell, "Invalid RSSI_TIMEOUT: %d",
				    rssi_timeout);
			return -EINVAL;
		}
		config.rssi_timeout = rssi_timeout;
	}

	/* Only (re-)configure settings, not channels */
	if (argc == 3) {
		ret = si4467_configure_rx_hopping(ctx, &config, NULL, 0);
		if (ret) {
			shell_error(shell,
				    "Failed to reconfigure RX hopping: %d",
				    ret);
			return -EIO;
		}
	}

	/* Configure RX hopping settings and channels */
	if (argc > 3) {
		channel_table_size = argc - 3;

		for (uint8_t i = 0; i < SI4467_PROPERTY_RX_HOP_TABLE_SIZE_MAX;
		     i++) {
			/* Disable channels not requested to be active */
			if (channel_table_size <= i) {
				channel_table[i] = 0xFF;
				continue;
			}

			const int channel = strtoul(argv[3 + i], &position, 10);

			if (channel > 0xFF || position == argv[3 + i]) {
				shell_error(shell, "Invalid channel: %s",
					    argv[3 + i]);
				return -EINVAL;
			}
			channel_table[i] = channel;
		}

		ret = si4467_configure_rx_hopping(ctx, &config, channel_table,
						  channel_table_size);
		if (ret) {
			shell_error(shell,
				    "Failed to reconfigure RX hopping settings "
				    "and channels: %d",
				    ret);
			return -EIO;
		}
	}

	/* Print currently active settings */
	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_RX_HOP,
					    SI4467_PROPERTY_RX_HOP_TABLE_SIZE,
					    &channel_table_size, 1);
	if (ret) {
		return -EIO;
	}

	if (channel_table_size > sizeof(channel_table)) {
		shell_error(shell, "Invalid RX hop table size: %u",
			    channel_table_size);
		return -EIO;
	}

	ret = si4467_command_get_properties(
		ctx, SI4467_PROPERTY_GROUP_RX_HOP,
		SI4467_PROPERTY_RX_HOP_TABLE_ENTRY_0, channel_table,
		channel_table_size);
	if (ret) {
		return -EIO;
	}

	shell_fprintf(shell, SHELL_NORMAL, "Configured channels:");
	for (uint_fast8_t i = 0; i < channel_table_size; i++) {
		shell_fprintf(shell, SHELL_NORMAL, " %u", channel_table[i]);
	}
	shell_fprintf(shell, SHELL_NORMAL, "\n");

	ret = si4467_get_property_rx_hop_control(ctx, &hop_control);
	if (ret) {
		return -EIO;
	}

	shell_print(shell, "Enable settings: %u", hop_control.enable);
	shell_print(shell, "RSSI timeout: %u nibbles",
		    hop_control.rssi_timeout);

	return 0;
}

static int cmd_si4467_properties(const struct shell *shell, size_t argc,
				 char *argv[])
{
	struct si4467_context *ctx = get_context();
	const uint8_t property_groups[] = {
		SI4467_PROPERTY_GROUP_GLOBAL,
		SI4467_PROPERTY_GROUP_INT_CTL,
		SI4467_PROPERTY_GROUP_FRR_CTL,
		SI4467_PROPERTY_GROUP_PREAMBLE,
		SI4467_PROPERTY_GROUP_SYNC,
		SI4467_PROPERTY_GROUP_PKT,
		SI4467_PROPERTY_GROUP_MODEM,
		SI4467_PROPERTY_GROUP_MODEM_CHFLT,
		SI4467_PROPERTY_GROUP_PA,
		SI4467_PROPERTY_GROUP_SYNTH,
		SI4467_PROPERTY_GROUP_MATCH,
		SI4467_PROPERTY_GROUP_FREQ_CONTROL,
		SI4467_PROPERTY_GROUP_RX_HOP,
		SI4467_PROPERTY_GROUP_PTI,
	};

	for (uint_fast8_t i = 0; i < ARRAY_SIZE(property_groups); i++) {
		uint8_t group = property_groups[i];
		const ssize_t total = si4467_property_group_size(group);
		uint_fast8_t done = 0;

		shell_print(shell, "%s (0x%02X):", si4467_property_group_str(group), group);

		while (done < total) {
			uint8_t values[16];
			const uint8_t chunk = MIN(sizeof(values), total - done);
			const int ret = si4467_command_get_properties(
				ctx, group, done, values, chunk);

			if (ret) {
				shell_error(
					shell,
					"Failed to get %u properties in group %d, starting "
					"at offset %u",
					chunk, group, done);
				return -EIO;
			}
			shell_hexdump_line(shell, done, values, chunk);
			done += chunk;
		}
	}

	return 0;
}

static int cmd_si4467_channel_settings(const struct shell *shell, size_t argc,
				       char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct si4467_context *ctx = get_context();
	uint8_t fc_inte;
	uint32_t fc_frac;
	struct si4467_property_modem_clkgen_band clkgen_band;
	uint16_t channel_step_size;
	int ret;

	ret = si4467_get_property_clkgen_band(ctx, &clkgen_band);
	if (ret) {
		return -EIO;
	}

	ret = si4467_get_property_freq_control_inte(ctx, &fc_inte);
	if (ret) {
		return -EIO;
	}

	ret = si4467_get_property_freq_control_frac(ctx, &fc_frac);
	if (ret) {
		return -EIO;
	}

	ret = si4467_get_property_freq_control_channel_step_size(
		ctx, &channel_step_size);
	if (ret) {
		return -EIO;
	}

	uint8_t npresc = 2; /* Inhibit maybe-uninitialized warning */

	switch (clkgen_band.sy_sel) {
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_LOW_POWER:
		npresc = 4;
		break;
	case SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_HIGH_PERFORMANCE:
		npresc = 2;
		break;
	}
	const uint32_t freq_xo = si4467_get_xtal_frequency();
	const uint8_t outdiv = si4467_property_modem_clkgen_band_band_to_outdiv(
		clkgen_band.band);
	const float left = fc_inte + (float)fc_frac / (float)(1U << 19);
	const float right = (npresc * freq_xo / outdiv);
	const uint32_t base_channel_frequency = left * right;

	shell_print(shell, "Frequency control, integer part: %u", fc_inte);
	shell_print(shell, "Frequency control, fractional part: %u", fc_frac);
	shell_print(shell, "Prescaler divider: %u (%s)", npresc,
		    si4467_property_modem_clkgen_band_sy_sel_str(
			    clkgen_band.sy_sel));
	shell_print(shell, "Crystal frequency: %u Hz", freq_xo);
	shell_print(shell, "Output divider: %u", outdiv);
	shell_print(shell, "Base channel frequency: %u Hz",
		    base_channel_frequency);

	const uint32_t channel_step_size_hz = (float)channel_step_size /
					      (float)(1U << 19) / outdiv *
					      npresc * freq_xo;
	shell_print(shell, "Channel step size: %u Hz", channel_step_size_hz);

	return 0;
}

static int cmd_si4467_int_status(const struct shell *shell, size_t argc,
				 char *argv[])
{
	struct si4467_context *ctx = get_context();
	/* Per default, do preserve the status */
	const bool preserve = argv[1][0] != '1';
	const struct int_status_interrupt preserve_what = {
		{ preserve, preserve, preserve, preserve, preserve, preserve,
		  preserve, preserve },
		{ preserve, preserve, preserve, preserve, preserve, preserve,
		  preserve, preserve },
		{ preserve, preserve, preserve, preserve, preserve, preserve,
		  preserve, preserve }
	};
	struct int_status_reply status;

	if (si4467_command_get_int_status(ctx, &preserve_what, &status)) {
		shell_error(shell, "Getting int status failed");
		return -EIO;
	}

	shell_print_packet_handler_status_reply(shell, status.packet_handler);
	shell_print_modem_status_reply(shell, status.modem);
	shell_print_chip_status_reply(shell, status.chip);
	if (status.chip.interrupt_status.state_change) {
		struct request_device_state_reply reply;

		if (si4467_command_request_device_state(ctx, &reply)) {
			shell_error(shell, "failed to get device state");
			return -EIO;
		}
		shell_print_chip_status_device_state(shell, reply);
	}

	return 0;
}

static int cmd_si4467_device_state(const struct shell *shell, size_t argc,
				   char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct si4467_context *ctx = get_context();
	struct request_device_state_reply reply;

	if (si4467_command_request_device_state(ctx, &reply)) {
		shell_error(shell, "failed to get device state");
		return -EIO;
	}

	shell_print_chip_status_device_state(shell, reply);

	if (argc == 1) {
		return 0;
	}

	const uint8_t next_state = strtoul(argv[1], NULL, 10);

	if (!si4467_is_valid_next_operating_state(next_state)) {
		shell_error(shell, "invalid device state: %u", next_state);
		return -EINVAL;
	}

	if (si4467_command_change_state(ctx,
					(enum operating_state)next_state)) {
		shell_error(shell, "failed to set device state");
		return -EIO;
	}

	shell_print(shell, "Switched to device state %s",
		    request_device_state_reply_to_string(next_state));

	return 0;
}

static int cmd_si4467_fifo_state(const struct shell *shell, size_t argc,
				 char *argv[])
{
	struct si4467_context *ctx = get_context();
	/* Per default, do not reset the FIFOs */
	const bool reset_rx = (argc >= 2 && argv[1][0] == '1');
	const bool reset_tx = (argc == 3 && argv[1][2] == '1');
	struct fifo_information fifo_info;

	if (si4467_command_fifo_info(ctx, reset_rx, reset_tx, &fifo_info)) {
		shell_error(shell, "Cannot execute command FIFO_INFO");
		return -EIO;
	}
	shell_print(shell, "RX FIFO count: %u", fifo_info.rx_fifo_count);
	shell_print(shell, "TX FIFO space: %u", fifo_info.tx_fifo_space);

	return 0;
}

static int cmd_si4467_start_tx(const struct shell *shell, size_t argc,
			       char *argv[])
{
	struct si4467_context *ctx = get_context();
	uint16_t tx_channel = 0;

	if (argc == 2) {
		tx_channel = strtoul(argv[1], NULL, 10);
	}

	shell_print(shell, "TX start on channel %" PRIu8 "", tx_channel);

	/* start TX */
	const int ret = si4467_command_start_tx(ctx, 0, tx_channel);

	if (ret) {
		shell_error(shell, "Cannot start test data transmission");
		return ret;
	}

	return 0;
}

static int cmd_si4467_tx_sweep(const struct shell *shell, size_t argc,
			       char *argv[])
{
	struct si4467_context *ctx = get_context();

	uint16_t cycles = 1;
	uint16_t dwell_ms = 100;
	uint8_t channel_start = 0;
	uint8_t channel_stop = 67;

	if (argc > 1) {
		cycles = strtoul(argv[1], NULL, 10);
	}

	if (argc > 2) {
		dwell_ms = strtoul(argv[2], NULL, 10);
	}

	if (argc > 3) {
		channel_start = strtoul(argv[3], NULL, 10);
	}

	if (argc > 4) {
		channel_stop = strtoul(argv[4], NULL, 10);
	}

	if (channel_stop < channel_start) {
		shell_error(shell, "last channel must be >= first channel");
		return -EINVAL;
	}

	shell_print(shell,
		    "cycles: %" PRIu16 ", dwell time [ms]: %" PRIu16 ", "
		    "first channel: %" PRIu8 ", last channel: %" PRIu8 "",
		    cycles, dwell_ms, channel_start, channel_stop);
	for (uint16_t cycle = 0; cycle < cycles || cycles == 0; cycle++) {
		shell_print(shell, "cycle %" PRIu16 " / %" PRIu16 "", cycle + 1,
			    cycles);
		for (uint8_t channel = channel_start; channel <= channel_stop;
		     channel++) {
			if (si4467_command_start_tx(
				    ctx, 0, channel)) {
				return -EIO;
			}
			k_msleep(dwell_ms);
		}
	}

	/* stop TX */
	if (si4467_command_change_state(ctx, SI4467_OPERATING_STATE_READY)) {
		shell_error(shell, "failed to set device state to READY");
		return -EIO;
	}

	return 0;
}

static int cmd_si4467_receive(const struct shell *shell, size_t argc,
			      char *argv[])
{
	int ret;
	uint8_t channel = 0;

	const uint16_t num_bytes = strtol(argv[1], NULL, 10);

	if (argc >= 3) {
		channel = strtoul(argv[2], NULL, 10);
	}

	/* Reset RX FIFO */
	struct si4467_context *ctx = get_context();
	struct fifo_information fifo_info;

	ret = si4467_command_fifo_info(ctx, true, true, &fifo_info);
	if (ret) {
		shell_error(shell, "Failed to reset RX FIFO: %d", ret);
		return ret;
	}

	const struct int_status_interrupt preserve_noting = {};
	struct int_status_reply status;

	if (si4467_command_get_int_status(ctx, &preserve_noting, &status)) {
		shell_error(shell, "Resetting int status failed");
		return -EIO;
	}

	const struct start_rx_condition condition = {
		.start = SI4467_START_TRX_IMMEDIATE,
		.update = SI4467_START_TRX_UPDATE_USE,
	};

	ret = si4467_command_start_rx(ctx, condition, num_bytes, channel,
				      SI4467_OPERATING_STATE_NOCHANGE,
				      SI4467_OPERATING_STATE_READY,
				      SI4467_OPERATING_STATE_RX);
	if (ret) {
		shell_error(shell, "Failed to start RX: %d", ret);
		return ret;
	}

	if (num_bytes) {
		shell_print(shell, "receiving %u packets", num_bytes);
	} else {
		shell_print(shell, "receiving an infinite number of bytes");
	}

	return 0;
}

#define PA_MODE_HELP                                                   \
	("[external PA ramp] [digital power sequencing] "              \
	 "[PA selection] [PA mode]\n"                                  \
	 "Print [and configure] PA (power amplifier) mode settings.\n" \
	 "\n"                                                          \
	 "External PA ramp (EXT_PA_RAMP):\n"                           \
	 "- DISABLE_EXT_PA_RAMP: disabled\n"                           \
	 "- ENABLE_EXT_PA_RAMP: enabled\n"                             \
	 "Digital power sequencing (DIG_PWR_SEQ):\n"                   \
	 "- DISABLE_DIG_PWR_SEQ: disabled\n"                           \
	 "- ENABLE_DIG_PWR_SEQ: enabled\n"                             \
	 "PA selection (PA_SEL):\n"                                    \
	 "- HP_FINE: finer step size (~2x)\n"                          \
	 "- HP_COARSE: coarser step size\n"                            \
	 "- LP: Si4460: lower-power applications\n"                    \
	 "- MP: Si4461: medium-power applications\n"                   \
	 "PA mode (PA_MODE):\n"                                        \
	 "- CLE: switching-amplifier mode\n"                           \
	 "- SWC: switched current mode\n")
static int cmd_pa_mode(const struct shell *shell, size_t argc, char *argv[])
{
	const static char *field_options_ext_pa_ramp[] = {
		"DISABLE_EXT_PA_RAMP", "ENABLE_EXT_PA_RAMP"
	};
	const static char *field_options_dig_pwr_seq[] = {
		"DISABLE_DIG_PWR_SEQ", "ENABLE_DIG_PWR_SEQ"
	};
	const static char *field_options_pa_sel[] = {
		"PA_SEL_INV0", "HP_FINE",     "HP_COARSE",
		"PA_SEL_INV3", "PA_SEL_INV4", "PA_SEL_INV5",
		"LP",	       "PA_SEL_INV7", "MP"
	};
	const static char *field_options_pa_mode[] = { "CLE", "SWC" };

	const struct property_field_definition fields[] = {
		{ "External PA ramp",
		  ARRAY_AND_LENGTH(field_options_ext_pa_ramp), 7, 1 },
		{ "digital power sequencing",
		  ARRAY_AND_LENGTH(field_options_dig_pwr_seq), 6, 1 },
		{ "PA select", ARRAY_AND_LENGTH(field_options_pa_sel), 2, 4 },
		{ "PA mode", ARRAY_AND_LENGTH(field_options_pa_mode), 0, 1 }
	};
	enum { num_fields = ARRAY_SIZE(fields) };

	int ret;
	uint8_t pa_mode;

	struct si4467_context *ctx = get_context();

	/* get current settings */
	ret = si4467_command_get_properties(ctx, SI4467_PROPERTY_GROUP_PA,
					    SI4467_PROPERTY_PA_MODE, &pa_mode,
					    1);
	if (ret) {
		return -EIO;
	}

	/* print current settings */
	shell_print(shell, "current PA_MODE = 0x%02x", pa_mode);
	print_property(shell, pa_mode, num_fields, fields);

	if (argc <= 1) {
		/* no command - we're done */
		return 0;
	}

	/* scan arguments & determine new settings */
	ret = update_property(shell, &pa_mode, num_fields, fields, argc,
			      (const char **)argv);
	if (ret < 0) {
		return ret;
	}

	if (ret == 0) {
		shell_print(shell, "PA_MODE unchanged");
		return 0;
	}

	/* set new settings */
	si4467_command_set_property(ctx, SI4467_PROPERTY_GROUP_PA,
				    SI4467_PROPERTY_PA_MODE, pa_mode);

	/* print new settings */
	shell_print(shell, "new PA_MODE = 0x%02x", pa_mode);
	print_property(shell, pa_mode, num_fields, fields);

	return 0;
}

static int cmd_pa_power_level(const struct shell *shell, size_t argc,
			      char *argv[])
{
	struct si4467_context *ctx = get_context();
	uint8_t power_level;

	if (argc == 2) {
		power_level = strtoul(argv[1], NULL, 16);

		if (si4467_set_property_pwr_lvl(ctx, power_level)) {
			return -EIO;
		}
	}

	if (si4467_get_property_pwr_lvl(ctx, &power_level)) {
		return -EIO;
	}

	shell_print(shell, "PA_PWR_LVL = 0x%02x", power_level);

	return 0;
}

static int cmd_pa_bias_clkduty(const struct shell *shell, size_t argc,
			       char *argv[])
{
	struct si4467_context *ctx = get_context();
	uint8_t bias;
	uint8_t clkduty;

	if (si4467_get_property_pa_bias_clkduty(ctx, &bias, &clkduty)) {
		return -EIO;
	}
	if (argc >= 2) {
		bias = strtoul(argv[1], NULL, 16);
		if (argc == 3) {
			clkduty = strtoul(argv[2], NULL, 16);
		}
		if (si4467_set_property_pa_bias_clkduty(ctx, bias, clkduty)) {
			shell_error(shell, "invalid parameter");
			return -EIO;
		}
	}

	shell_print(
		shell, "PA_BIAS_CLKDUTY = 0x%02x",
		(bias << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_POS |
		 clkduty << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS));
	shell_print(shell, "PA_BIAS = 0x%02x", bias);
	shell_print(shell, "PA_CLKDUTY = 0x%02x", clkduty);
	return 0;
}

static int packet_trx_send(const struct shell *shell, uint8_t *buf,
			   uint8_t length, uint8_t channel)
{
	struct si4467_context *ctx = get_context();
	struct fifo_information fifo_info;
	const uint16_t bitlength = (length + 2) * 8;
	const uint8_t bitlength_msb[2] = { bitlength >> 8, bitlength & 0xff };

	/* reset TX/RX FIFO */
	if (si4467_command_fifo_info(ctx, true, true, &fifo_info)) {
		shell_error(shell, "Cannot execute command FIFO_INFO");
		return -EIO;
	}

	/*
	 * send length to TX FIFO
	 *
	 * Note: using bit count is not correct for current radio configuration,
	 * but consistent with how we use this field elsewhere. The length is
	 * ultimately irrelevant as we received a fixed number of bytes.
	 */
	if (si4467_command_send(ctx, COMMAND_WRITE_TX_FIFO, bitlength_msb,
				sizeof(bitlength), NULL, 0, true)) {
		shell_error(shell, "cannot execute command WRITE_TX_FIFO");
		return -EIO;
	}

	/* send data to TX FIFO */
	if (si4467_command_send(ctx, COMMAND_WRITE_TX_FIFO, buf, length, NULL,
				0, true)) {
		shell_error(shell, "cannot execute command WRITE_TX_FIFO");
		return -EIO;
	}

	/* start TX (data + 2 length bytes) */
	si4467_command_start_tx(ctx, length + 2, channel);

	return 0;
}

static int packet_trx_receive(const struct shell *shell, uint8_t *buf,
			      uint8_t length, uint16_t timeout_ms,
			      int16_t *rssi_dbm, uint8_t channel)
{
	int ret;
	struct si4467_context *ctx = get_context();
	struct fifo_information fifo_info;

	/* reset TX/RX FIFO */
	ret = si4467_command_fifo_info(ctx, true, true, &fifo_info);
	if (ret) {
		shell_error(shell, "Cannot execute command FIFO_INFO");
		return ret;
	}

	/* reset interrupt status */
	const struct int_status_interrupt preserve_noting = {};
	struct int_status_reply status;

	if (si4467_command_get_int_status(ctx, &preserve_noting, &status)) {
		return -EIO;
	}

	/* disable RX hopping */
	const struct si4467_property_group_rx_hop_control config = {
		.rssi_timeout = 0,
		.enable = SI4467_PROPERTY_RX_HOP_CONTROL_EN_DISABLE
	};
	ret = si4467_configure_rx_hopping(ctx, &config, NULL, 0);
	if (ret) {
		return -EIO;
	}

	/* start RX (data + 2 length bytes) */
	const struct start_rx_condition condition = {
		.start = SI4467_START_TRX_IMMEDIATE,
		.update = SI4467_START_TRX_UPDATE_USE,
	};

	ret = si4467_command_start_rx(ctx, condition, length + 2, channel,
				      SI4467_OPERATING_STATE_NOCHANGE,
				      SI4467_OPERATING_STATE_READY,
				      SI4467_OPERATING_STATE_RX);
	if (ret) {
		return ret;
	}

	/* await data */
	int64_t wait_time_start_ms = k_uptime_get();
	while (true) {
		ret = si4467_command_fifo_info(ctx, 0, 0, &fifo_info);
		if (ret) {
			return ret;
		}

		/* wait to receive all data (length bytes will not be in FIFO) */
		if (fifo_info.rx_fifo_count > length) {
			return -EMSGSIZE;
		}
		if (fifo_info.rx_fifo_count == length) {
			break;
		}

		if (k_uptime_get() - wait_time_start_ms >= timeout_ms &&
		    timeout_ms != 0) {
			return -ETIMEDOUT;
		}

		k_msleep(1);
	}

	/* get data */
	uint8_t bytes_left = fifo_info.rx_fifo_count;
	uint8_t bytes_read = 0;

	memset(buf, 0, length);
	do {
		const uint8_t bytes_to_read =
			MIN(bytes_left, SI4467_MAX_COMMAND_RESPONSE_SIZE);
		bytes_left -= bytes_to_read;
		ret = si4467_command_send(ctx, COMMAND_READ_RX_FIFO, NULL, 0,
					  buf + bytes_read, bytes_to_read,
					  false);
		if (ret) {
			return ret;
		}
		bytes_read += bytes_to_read;
	} while (bytes_left);

	/* get & store RSSI value */
	uint8_t rssi;
	ret = si4467_command_read_fast_response_register(ctx, SI4467_FRR_A,
							 &rssi);
	if (ret) {
		shell_error(shell, "failed to read RSSI from FRR A: %d", ret);
		/* OK for testing, though it shouldn't happen */
		rssi = 0;
	}

	/* calculate RSSI in dBm as per section 5.2.5 Si4467 datasheet
	 * (this assumes a fixed MODEM_RSSI_COMP of 40)
	 */
	*rssi_dbm = rssi / 2 - 40 - 70;

	return 0;
}

#define PACKET_TRX_MAX_PACKET_LENGTH (SI4467_FIFO_MAX_SIZE - 2)
#define PACKET_TRX_DEFAULT_PACKETS 10
#define PACKET_TRX_DEFAULT_LENGTH 50
#define PACKET_TRX_DEFAULT_DELAY_MS 500
#define PACKET_TRX_DEFAULT_CHANNEL 30
#define PACKET_TRX_HELP                                                       \
	("<tx|rx> "                                                           \
	 "[<packets> (number of packets) [<length> (size in bytes) "          \
	 "[TX delay/RX timeout (ms) [channel]]]]\n"                           \
	 "Send/receive pseudo-random packets for testing. "                   \
	 "For correct operation, TX and RX packet length must be identical "  \
	 "and RX timeout must be >= TX delay.\n"                              \
	 "Special options:\n"                                                 \
	 "- packets = 0: loop infinitely\n"                                   \
	 "- rx timeout = 0: no timeout\n"                                     \
	 "Default values: 10 packets of length 50 bytes with 500ms delay on " \
	 "channel 30.")
static int cmd_packet_trx(const struct shell *shell, size_t argc, char *argv[])
{
	const uint8_t seed =
		4; /* chosen by a fair dice roll. guaranteed to be random. */
	uint32_t num_packets = PACKET_TRX_DEFAULT_PACKETS;
	uint16_t packet_length = PACKET_TRX_DEFAULT_LENGTH;
	uint16_t delay_ms = PACKET_TRX_DEFAULT_DELAY_MS;
	uint8_t ref_buf[PACKET_TRX_MAX_PACKET_LENGTH];
	uint8_t rx_buf[PACKET_TRX_MAX_PACKET_LENGTH];
	uint8_t channel = PACKET_TRX_DEFAULT_CHANNEL;
	bool tx;

	/* parse arguments */
	if (strcmp(argv[1], "tx") == 0) {
		tx = true;
	} else if (strcmp(argv[1], "rx") == 0) {
		tx = false;
	} else {
		shell_error(shell, "invalid command: %s (use 'tx' or 'rx')",
			    argv[1]);
		return -EINVAL;
	}

	if (argc >= 3) {
		num_packets = strtoul(argv[2], NULL, 10);
	}

	if (argc >= 4) {
		packet_length = strtoul(argv[3], NULL, 10);

		if (packet_length > PACKET_TRX_MAX_PACKET_LENGTH) {
			shell_warn(shell,
				   "limiting packet length to max length %d",
				   PACKET_TRX_MAX_PACKET_LENGTH);
			packet_length = PACKET_TRX_MAX_PACKET_LENGTH;
		}
	}

	if (argc >= 5) {
		delay_ms = strtoul(argv[4], NULL, 10);
		if (tx && delay_ms < 20) {
			shell_warn(shell,
				   "packet loss occurs with TX delay < 20ms");
		}
	}

	if (argc >= 6) {
		channel = strtoul(argv[5], NULL, 10);
	}

	/*
	 * Remove interrupt handler callback to prevent packet handling by L2
	 * drivers; L2 driver will no longer work (as with most commands).
	 */
	struct si4467_context *ctx = get_context();
	if (gpio_remove_callback(ctx->gpios[SI4467_GPIO_IDX_NIRQ].dev,
				 &ctx->nirq_cb)) {
		shell_error(shell, "failed to remove nirq callback");
	}

	/* initialize packet */
	ref_buf[0] = crc8_ccitt(0, &seed, 1);
	for (uint16_t i = 1; i < packet_length; i++) {
		ref_buf[i] = crc8_ccitt(0, &ref_buf[i - 1], 1);
	}

	/* send / receive packets */
	if (tx) {
		shell_print(
			shell,
			"sending %d packets of size %d bytes with %d ms delay "
			"on channel %d",
			num_packets, packet_length, delay_ms, channel);
		for (uint32_t i = 1; i <= num_packets || num_packets == 0;
		     i++) {
			shell_print(shell, "sending packet %d/%d", i,
				    num_packets);
			packet_trx_send(shell, ref_buf, packet_length, channel);
			if (delay_ms > 0) {
				k_msleep(delay_ms);
			}
		}
	} else {
		shell_print(
			shell,
			"receiving %d packets of size %d bytes on channel %d",
			num_packets, packet_length, channel);
		uint32_t i = 0;
		uint32_t rx_packets_ok = 0;
		while (i < num_packets || num_packets == 0) {
			i++;

			int16_t rssi_dbm = 0;
			int ret = packet_trx_receive(shell, rx_buf,
						     packet_length, delay_ms,
						     &rssi_dbm, channel);
			if (ret) {
				shell_error(shell, "E packet %d/%d: error: %s",
					    i, num_packets, strerror(-ret));
				continue;
			}

			/* count bit errors */
			uint32_t bit_errors = 0;
			for (uint16_t j = 0; j < packet_length; j++) {
				bit_errors += popcount(ref_buf[j] ^ rx_buf[j]);
			}

			if (bit_errors) {
				shell_print(
					shell,
					"X packet %d/%d: %d bit error(s); RSSI: %d dBm",
					i, num_packets, bit_errors, rssi_dbm);
			} else {
				shell_print(shell,
					    "+ packet %d/%d: OK; RSSI: %d dBm",
					    i, num_packets, rssi_dbm);
				rx_packets_ok += 1;
			}
		}

		shell_print(shell, "%d/%d packets received successfully",
			    rx_packets_ok, num_packets);
	}

	return 0;
}

static int cmd_fifo_test(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	struct fifo_information fifo_info;
	struct si4467_context *ctx = get_context();
	int cycles = strtol(argv[1], NULL, 10);

	ret = si4467_command_fifo_info(ctx, true, true, &fifo_info);
	if (ret) {
		shell_error(shell, "failed to reset FIFO");
		return ret;
	}

	for (int i = 0; i < cycles; i++) {
		ret = si4467_command_fifo_info(ctx, 0, 0, &fifo_info);
		if (ret) {
			shell_error(shell,
				    "fifo info failed at cycle %d/%d: %d (%s)",
				    i, cycles, ret, strerror(-ret));
		}
	}

	shell_print(shell, "%d cycles done", cycles);

	return 0;
}

static int cmd_gpio_sdn(const struct shell *shell, size_t argc, char *argv[])
{
	struct si4467_context *ctx = get_context();
	const struct si4467_gpio_configuration sdn_gpio =
		ctx->gpios[SI4467_GPIO_IDX_SDN];

	if (strlen(argv[1]) != 1 || (argv[1][0] != '0' && argv[1][0] != '1')) {
		shell_error(shell, "invalid argument: %s", argv[1]);
		return -EINVAL;
	}

	const uint8_t new_gpio_value = argv[1][0] - '0';

	if (gpio_pin_set(sdn_gpio.dev, sdn_gpio.pin, new_gpio_value)) {
		shell_error(shell, "Failed to set SDN GPIO pin");
		return -EIO;
	}

	return 0;
}

static int cmd_clock_output(const struct shell *shell, size_t argc,
			    char *argv[])
{
	struct si4467_context *ctx = get_context();

	if (strlen(argv[1]) != 1 || (argv[1][0] != '0' && argv[1][0] != '1')) {
		shell_error(shell, "invalid argument: %s", argv[1]);
		return -EINVAL;
	}

	if (argv[1][0] == '1') { /* enable clock output */
		const uint8_t gpio_config[] = {
			SI4467_GPIO_PIN_CFG_PULL_CTL_EN |
				SI4467_GPIO_PIN_CFG_GPIO_MODE_DIV_CLK,
			SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
				SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
			SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
				SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
			SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
				SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
		};
		if (si4467_command_gpio_pin_config(
			    ctx, gpio_config,
			    SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
				    SI4467_GPIO_PIN_CFG_GPIO_MODE_DONOTHING,
			    SI4467_GPIO_PIN_CFG_PULL_CTL_DIS |
				    SI4467_GPIO_PIN_CFG_GPIO_MODE_DONOTHING,
			    SI4467_GPIO_PIN_CFG_DRV_STRENGTH_MED_LOW)) {
			shell_error(shell, "Cannot configure GPIOs.");
			return -EIO;
		}
		if (si4467_command_set_property(
			    ctx, SI4467_PROPERTY_GROUP_GLOBAL,
			    SI4467_PROPERTY_GLOBAL_CLK_CFG,
			    SI4467_PROPERTY_GLOBAL_CLK_CFG_DIVIDED_CLOCK_EN)) {
			shell_error(shell, "failed to enable clock output");
			return -EIO;
		}
	} else { /* disable clock output */
		if (si4467_command_set_property(
			    ctx, SI4467_PROPERTY_GROUP_GLOBAL,
			    SI4467_PROPERTY_GLOBAL_CLK_CFG, 0)) {
			shell_error(shell, "failed to disable clock output");
			return -EIO;
		}

		/* restore default GPIO pin configuration */
		if (si4467_setup_gpio_pin_config(ctx)) {
			shell_error(shell,
				    "failed to restore default GPIO config");
			return -EIO;
		}
	}

	return 0;
}

#define CLOCK_CAL_HELP                                                 \
	("[tune value] Get [or set] oscillator calibration value.\n\n" \
	 "0: lowest capacitance (i.e. highest frequency)\n"            \
	 "127: highest capacitance (i.e. lowest frequency)")
static int cmd_clock_calibration(const struct shell *shell, size_t argc,
				 char *argv[])
{
	struct si4467_context *ctx = get_context();
	uint8_t tune_val;

	if (argc == 1) { /* print current value */
		if (si4467_command_get_xo_tune_value(ctx, &tune_val)) {
			shell_error(shell, "failed to get tune value");
			return -EIO;
		}

		shell_print(shell, "tune value: %d", tune_val);
		return 0;
	}

	/* set value */
	tune_val = strtoul(argv[1], NULL, 10);

	return si4467_command_set_xo_tune_value(ctx, tune_val);
}

#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE
static int cmd_clock_calibration_save(const struct shell *shell, size_t argc,
				      char *argv[])
{
	struct si4467_context *ctx = get_context();

	if (si4467_save_calibration(ctx)) {
		shell_error(shell, "failed to save tune value");
		return -EIO;
	}

	return 0;
}

static int cmd_clock_calibration_restore(const struct shell *shell, size_t argc,
					 char *argv[])
{
	struct si4467_context *ctx = get_context();

	return si4467_restore_calibration(ctx);
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
	si4467_commands,
	SHELL_CMD_ARG(channel_settings, NULL, "Print channel settings",
		      cmd_si4467_channel_settings, 1, 0),
	SHELL_CMD_ARG(chip_status, NULL, "<0|1> Print [and reset] chip status",
		      cmd_si4467_chip_status, 2, 0),
	SHELL_CMD_ARG(device_state, NULL,
		      "[state] Print current device state."
		      " Optionally, switch to <state> (1-3, 5-8)",
		      cmd_si4467_device_state, 1, 1),
	SHELL_CMD_ARG(fifo_state, NULL,
		      "[0|1 [0|1]] Print [and reset RX [TX]] FIFO state",
		      cmd_si4467_fifo_state, 1, 2),
	SHELL_CMD_ARG(frr, NULL, "<reg> Print fast response register A-D",
		      cmd_si4467_fast_response_register, 2, 0),
	SHELL_CMD_ARG(int_state, NULL,
		      "<0|1> Print [and reset] all interrupt states",
		      cmd_si4467_int_status, 2, 0),
	SHELL_CMD_ARG(modem_status, NULL,
		      "<0|1> Print [and reset] modem status",
		      cmd_si4467_modem_status, 2, 0),
	SHELL_CMD_ARG(packet_handler_status, NULL,
		      "<0|1> Print [and reset] packet handler status",
		      cmd_si4467_packet_handler_status, 2, 0),
	SHELL_CMD_ARG(properties, NULL, "Print all properties",
		      cmd_si4467_properties, 1, 0),
	SHELL_CMD_ARG(receive, NULL,
		      "<len> [channel] Receive <len> bytes on channel [channel]",
		      cmd_si4467_receive, 2, 1),
	SHELL_CMD_ARG(rx_fifo_data, NULL, "Print RX FIFO data",
		      cmd_si4467_rx_fifo_data, 1, 0),
	SHELL_CMD_ARG(modulation, NULL, MODULATION_HELP, cmd_si4467_modulation,
		      1, 4),
	SHELL_CMD_ARG(rx_hopping, NULL, RX_HOPPING_HELP, cmd_si4467_rx_hopping,
		      1, 67),
	SHELL_CMD_ARG(start_tx, NULL,
		      "Start TX [on Si4467 channel N]\n"
		      "Note: to stop TX, use command 'si4467 device_state 3'.",
		      cmd_si4467_start_tx, 1, 1),
	SHELL_CMD_ARG(tx_sweep, NULL,
		      "[cycles] [dwell time ms] [first channel] [last channel]\n"
		      "Sweep through TX frequencies "
		      "(cycles = 0: loop infinitely)",
		      cmd_si4467_tx_sweep, 1, 4),
	SHELL_CMD_ARG(pa_mode, NULL, PA_MODE_HELP, cmd_pa_mode, 1, 4),
	SHELL_CMD_ARG(pa_power_level, NULL,
		      "[power_level (hex)] Get [and set] PA power level",
		      cmd_pa_power_level, 1, 1),
	SHELL_CMD_ARG(
		pa_bias_clkduty, NULL,
		"[bias (hex)] [clk duty] Get [and set] PA bias and clock duty",
		cmd_pa_bias_clkduty, 1, 2),
	SHELL_CMD_ARG(packet_trx, NULL, PACKET_TRX_HELP, cmd_packet_trx, 2, 4),
	SHELL_CMD_ARG(fifo_test, NULL, "FIFO testing (for SG-17344)",
		      cmd_fifo_test, 2, 0),
	SHELL_CMD_ARG(gpio_sdn, NULL,
		      "<0|1> Set SDN (shutdown) GPIO pin.\n\n"
		      "1 = shutdown: current typ. 30 nA / max 1.3 μA.",
		      cmd_gpio_sdn, 2, 0),
	SHELL_CMD_ARG(clock_output, NULL,
		      "Enable(1)/disable(0) divided system clock output",
		      cmd_clock_output, 2, 0),
	SHELL_CMD_ARG(clock_calibration, NULL, CLOCK_CAL_HELP,
		      cmd_clock_calibration, 1, 1),
#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE
	SHELL_CMD_ARG(
		clock_calibration_save, NULL,
		"Save current clock calibration value to persistent storage.",
		cmd_clock_calibration_save, 1, 0),
	SHELL_CMD_ARG(
		clock_calibration_restore, NULL,
		"Restore and apply calibration value from persistent storage.",
		cmd_clock_calibration_restore, 1, 0),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(si4467, &si4467_commands, "Si4467 commands", NULL);
