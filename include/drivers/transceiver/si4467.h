/* si4467.h - Generic functionality for Silicon Labs Si4467 */

/*
 * Copyright (c) 2020 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_H_
#define ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_H_

#include "drivers/transceiver/si4467_regs.h"

#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <linker/sections.h>
#include <net/net_pkt.h>
#include <sys/atomic.h>
#include <sys/types.h>
#include <sys/mutex.h>

#define SI4467_PA_POWER_LEVEL_MAX 0x7f

/**
 * @brief Named indexes into the si4467_context::gpios array.
 *
 * @warning The number of entries must not change, regardless from which file
 *          this header gets included. Otherwise, the ABI between the modules is
 *          in disagreement, causing all kind of hard-to-debug issues.
 */
enum si4467_gpio_index {
	SI4467_GPIO_IDX_GPIO0,
	SI4467_GPIO_IDX_GPIO1,
	SI4467_GPIO_IDX_GPIO2,
	SI4467_GPIO_IDX_GPIO3,
	SI4467_GPIO_IDX_NIRQ,
#if CONFIG_TRX_SI4467_NIRQ_DEBUG
	SI4467_GPIO_IDX_NIRQ_HANDLING,
#endif
	SI4467_GPIO_IDX_SDN,
#if CONFIG_TRX_SI4467_RF_CONTROL_RX_HANDLING
	SI4467_GPIO_IDX_RX_ENABLE,
#endif
#if CONFIG_TRX_SI4467_RF_CONTROL_TX_HANDLING
	SI4467_GPIO_IDX_TX_ENABLE,
#endif
#if CONFIG_TRX_SI4467_ANTENNA_DIVERSITY_HANDLING
	SI4467_GPIO_IDX_ANT_DIVERSITY,
#endif
	SI4467_GPIO_IDX_COUNT,
};

struct si4467_gpio_configuration {
	const struct device *dev;
	uint32_t pin;
};

struct si4467_context {
	struct net_if *iface;
	/**************************/
	struct si4467_gpio_configuration gpios[SI4467_GPIO_IDX_COUNT];
	struct gpio_callback nirq_cb;
	const struct device *spi;
	struct spi_config spi_cfg;
	uint8_t mac_addr[8];
	const uint8_t *radio_configuration_data;
	const size_t radio_configuration_data_size;
	/************RX************/
	K_THREAD_STACK_MEMBER(rx_stack, CONFIG_TRX_SI4467_RX_STACK_SIZE);
	/***TRX locking****/
	struct k_mutex dev_lock; /**< Exclusive access to transceiver */
	/***Work synchronisation***/
	struct k_thread thread_nirq;
	struct k_sem isr_nirq_sem;
};

enum fast_response_register {
	SI4467_FRR_A = 0,
	SI4467_FRR_B = 1,
	SI4467_FRR_C = 2,
	SI4467_FRR_D = 3,
};

/*
 * Setting a field to 1 causes the interrupt to not be erased when passing the
 * struct to si4467_command_get_modem_status().
 */
struct modem_interrupt {
	uint8_t rssi_latch : 1;
	uint8_t postamble_detect : 1;
	uint8_t invalid_sync : 1;
	uint8_t rssi_jump : 1;
	uint8_t rssi : 1;
	uint8_t invalid_preamble : 1;
	uint8_t preamble_detect : 1;
	uint8_t sync_detect : 1;
};
BUILD_ASSERT(sizeof(struct modem_interrupt) == 1, "");

enum sync_trigger {
	SI4467_SYNC_TRIGGER_1 = 0,
	SI4467_SYNC_TRIGGER_2 = 1,
};

struct modem_status_reply {
	struct modem_interrupt interrupt_pending;
	struct modem_interrupt interrupt_status;
};

struct modem_status_reply_detailed {
	struct modem_status_reply modem_status;
	uint8_t curr_rssi;
	uint8_t latch_rssi;
	uint8_t ant1_rssi;
	uint8_t ant2_rssi;
	uint16_t afc_freq_offset;
	enum sync_trigger info_flags;
};

/*
 * Setting a field to 1 causes the interrupt to not be erased when passing the
 * struct to si4467_command_get_chip_status().
 */
struct chip_interrupt {
	uint8_t unused : 1;
	uint8_t cal : 1;
	uint8_t fifo_underflow_overflow_error : 1;
	uint8_t state_change : 1;
	uint8_t cmd_error : 1;
	uint8_t chip_ready : 1;
	uint8_t low_batt : 1;
	uint8_t wut : 1;
};
BUILD_ASSERT(sizeof(struct chip_interrupt) == 1, "");

enum {
	SI4467_CHIP_INTERRUPT_POS_WUT = 0,
	SI4467_CHIP_INTERRUPT_POS_LOW_BATT = 1,
	SI4467_CHIP_INTERRUPT_POS_CHIP_READY = 2,
	SI4467_CHIP_INTERRUPT_POS_CMD_ERROR = 3,
	SI4467_CHIP_INTERRUPT_POS_STATE_CHANGE = 4,
	SI4467_CHIP_INTERRUPT_POS_FIFO_UNDERFLOW_OVERFLOW_ERROR = 5,
	SI4467_CHIP_INTERRUPT_POS_CAL = 6,
};

enum {
	SI4467_GPIO_PIN_CFG_PULL_CTL_EN = (1 << 6),
	SI4467_GPIO_PIN_CFG_PULL_CTL_DIS = (0 << 6)
};

enum {
	SI4467_GPIO_PIN_CFG_GPIO_MODE_DONOTHING = 0,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TRISTATE = 1,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE0 = 2,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_DRIVE1 = 3,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_INPUT = 4,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_32K_CLK = 5,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_BOOT_CLK = 6,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_DIV_CLK = 7,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_CTS = 8,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_INV_CTS = 9,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_CMD_OVERLAP = 10,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_SDO = 11,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_POR = 12,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_CAL_WUT = 13,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_WUT = 14,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_EN_PA = 15,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_DATA_CLK = 16,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_DATA_CLK = 17,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_EN_LNA = 18,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_DATA = 19,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_DATA = 20,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_RAW_DATA = 21,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_ANTENNA_1_SW = 22,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_ANTENNA_2_SW = 23,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_VALID_PREAMBLE = 24,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_INVALID_PREAMBLE = 25,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_SYNC_WORD_DETECT = 26,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_CCA = 27,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_IN_SLEEP = 28,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_PKT_TRACE = 29,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_BUFDIV_CLK = 30,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_RX_DATA_CLK = 31,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_STATE = 32,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_STATE = 33,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_RX_FIFO_FULL = 34,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_TX_FIFO_EMPTY = 35,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_LOW_BATT = 36,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_CCA_LATCH = 37,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_HOPPED = 38,
	SI4467_GPIO_PIN_CFG_GPIO_MODE_HOP_TABLE_WRAP = 39,
};

enum {
	SI4467_GPIO_PIN_CFG_DRV_STRENGTH_HIGH = (0 << 5),
	SI4467_GPIO_PIN_CFG_DRV_STRENGTH_MED_HIGH = (1 << 5),
	SI4467_GPIO_PIN_CFG_DRV_STRENGTH_MED_LOW = (2 << 5),
	SI4467_GPIO_PIN_CFG_DRV_STRENGTH_LOW = (3 << 5),
};

enum calibration_type {
	SI4467_CALIBRATION_TYPE_OFFLINE = 0,
	SI4467_CALIBRATION_TYPE_OFFLINE2 = 1,
};

struct chip_status_reply {
	struct chip_interrupt interrupt_pending;
	struct chip_interrupt interrupt_status;
};

struct chip_status_reply_detailed {
	struct chip_status_reply chip_status;
	uint8_t command_error_status;
	uint8_t command_error_command_id;
	enum calibration_type calibration;
};

/*
 * Setting a field to 1 causes the interrupt to not be erased when passing the
 * struct to si4467_command_get_packet_handler_status().
 */
struct packet_handler_interrupt {
	uint8_t filter_match : 1;
	uint8_t filter_miss : 1;
	uint8_t packet_sent : 1;
	uint8_t packet_rx : 1;
	uint8_t crc_error : 1;
	uint8_t alt_crc_error : 1;
	uint8_t tx_fifo_almost_empty : 1;
	uint8_t rx_fifo_almost_full : 1;
};
BUILD_ASSERT(sizeof(struct packet_handler_interrupt) == 1, "");

enum {
	SI4467_PACKET_HANDLER_INTERRUPT_POS_RX_FIFO_ALMOST_FULL = 0,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_TX_FIFO_ALMOST_EMPTY = 1,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_ALT_CRC_ERROR = 2,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_CRC_ERROR = 3,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_RX = 4,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_PACKET_SENT = 5,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MISS = 6,
	SI4467_PACKET_HANDLER_INTERRUPT_POS_FILTER_MATCH = 7,
};

struct packet_handler_status_reply {
	struct packet_handler_interrupt interrupt_pending;
	struct packet_handler_interrupt interrupt_status;
};

struct int_interrupt {
	uint8_t packet_handler : 1;
	uint8_t modem_int : 1;
	uint8_t chip_int : 1;
};
BUILD_ASSERT(sizeof(struct int_interrupt) == 1, "");

enum {
	SI4467_INT_INTERRUPT_POS_PACKET_HANDLER = 0,
	SI4467_INT_INTERRUPT_POS_MODEM = 1,
	SI4467_INT_INTERRUPT_POS_CHIP = 2,
};

struct int_status_reply {
	struct int_interrupt pending;
	struct int_interrupt status;
	struct packet_handler_status_reply packet_handler;
	struct modem_status_reply modem;
	struct chip_status_reply chip;
	struct {
		enum sync_trigger sync;
		enum calibration_type calibration;
	} info_flags;
};

struct int_status_interrupt {
	struct chip_interrupt chip;
	struct packet_handler_interrupt packet;
	struct modem_interrupt modem;
};

enum {
	SI4467_MODEM_INTERRUPT_POS_SYNC_DETECT = 0,
	SI4467_MODEM_INTERRUPT_POS_PREAMBLE_DETECT = 1,
	SI4467_MODEM_INTERRUPT_POS_INVALID_PREAMBLE = 2,
	SI4467_MODEM_INTERRUPT_POS_RSSI = 3,
	SI4467_MODEM_INTERRUPT_POS_RSSI_JUMP = 4,
	SI4467_MODEM_INTERRUPT_POS_INVALID_SYNC = 5,
	SI4467_MODEM_INTERRUPT_POS_POSTAMBLE_DETECT = 6,
	SI4467_MODEM_INTERRUPT_POS_RSSI_LATCH = 7,
};

enum {
	SI4467_FIFO_INFO_RESET_TX_FIFO = 1,
	SI4467_FIFO_INFO_RESET_RX_FIFO = 2,
};

enum { SI4467_FIFO_MAX_SIZE = 129 };

struct part_information {
	uint8_t chiprev;
	uint16_t part;
	uint8_t pbuild;
	uint16_t id;
	uint8_t customer;
	uint8_t romid;
};

struct func_information {
	uint8_t revext;
	uint8_t revbranch;
	uint8_t revint;
	uint16_t patch;
	uint8_t func;
};

struct fifo_information {
	uint8_t rx_fifo_count;
	uint8_t tx_fifo_space;
};

enum operating_state {
	SI4467_OPERATING_STATE_NOCHANGE = 0,
	SI4467_OPERATING_STATE_REMAIN = SI4467_OPERATING_STATE_NOCHANGE,
	SI4467_OPERATING_STATE_SLEEP = 1,
	SI4467_OPERATING_STATE_SPI_ACTIVE = 2,
	SI4467_OPERATING_STATE_READY = 3,
	SI4467_OPERATING_STATE_READY2 = 4, /**< Another enumeration for READY
					     * state. Only valid as value
					     * returned by REQUEST_DEVICE_STATE
					     */
	SI4467_OPERATING_STATE_TX_TUNE = 5,
	SI4467_OPERATING_STATE_RX_TUNE = 6,
	SI4467_OPERATING_STATE_TX = 7,
	SI4467_OPERATING_STATE_RX = 8,
};

struct request_device_state_reply {
	enum operating_state curr_state;
	uint8_t curr_channel;
};

enum start_trx_start {
	SI4467_START_TRX_IMMEDIATE = 0, /**< Start TX/RX immediately. */
	SI4467_START_TRX_WUT = 1, /**< Start TX/RX when wake up timer expires.
				    */
};

enum start_trx_update {
	SI4467_START_TRX_UPDATE_USE = 0, /**< Use TX/RX parameters to enter RX
					   * mode.
					   */
	SI4467_START_TRX_UPDATE_UPDATE = 1, /**< Update TX/RX parameters (to be
					      * used by a subsequent packet) but
					      * do not enter TX/RX mode.
					      */
};

enum start_tx_retransmit {
	SI4467_START_TX_SEND = 0, /**< Send data that has been written to the TX
				    * FIFO. If the TX FIFO is empty, a FIFO
				    * underflow interrupt will occur.
				    */
	SI4467_START_TX_RETRANSMIT = 1, /**< Send last packet again. */
};

/**< Conditions upon which reception will start. */
struct start_rx_condition {
	enum start_trx_start start;
	enum start_trx_update update;
};

enum packet_info_field_num_override {
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_NONE = 0x00,
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_1 = 0x01,
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_2 = 0x02,
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_3 = 0x04,
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_4 = 0x08,
	SI4467_PACKET_INFO_FIELD_NUM_OVERRIDE_5 = 0x10,
};

enum { SI4467_MAX_COMMAND_RESPONSE_SIZE = 16 };

/**
 * @typedef si4467_thread_nirq_signature
 * @brief Executed on nIRQ request
 * @return 0 If successful, negative errno code otherwise.
 * @param ctx Pointer to the device structure for the driver instance.
 */
typedef int (*si4467_thread_nirq_callback_t)(struct si4467_context *ctx);

/**
 * @brief Send command to transceiver, wait for CTS, read back answer.
 *
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param cmd Command as specified in "Si4467 revA2A Command/Property API
 * Documentation". All except READ_CMD_BUFF can be used.
 * @param tx_bytes Array where data to be sent originates from.
 * @param tx_length Size in bytes of \p tx_bytes
 * @param rx_bytes Array where data to be read will be written to.
 * @param rx_length Size in bytes of \p rx_bytes
 * @param await_cts If true, function awaits CTS to assert before returning.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_send(struct si4467_context *ctx, enum si4467_command cmd,
			const uint8_t *tx_bytes, size_t tx_length,
			uint8_t *rx_bytes, size_t rx_length, bool await_cts);

/**
 * @brief Read the specified fast response register.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param frr FRR specification
 * @param register_value Pointer to be updated with fetched byte.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_read_fast_response_register(struct si4467_context *ctx,
					       enum fast_response_register frr,
					       uint8_t *register_value);

/**
 * @brief Fetch details about function state of transceiver.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param info Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_func_info(struct si4467_context *const ctx,
			     struct func_information *info);

/**
 * @brief Send command to configure GPIO pins.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param gpio Configuration (pull-up & function) for GPIO0..3.
 * @param nirq Configuration (pull-up & function) for NIRQ.
 * @param sdo Configuration (pull-up & function) for SDO.
 * @param gen_config Drive strength configuration.
 *
 * @return 0 if successful, negative errno code otherwise.
 */
int si4467_command_gpio_pin_config(struct si4467_context *const ctx,
				   const uint8_t gpio[4], uint8_t nirq,
				   uint8_t sdo, uint8_t gen_config);

/**
 * @brief Set the value of exactly one property.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param group Group of the property
 * @param property Offset of the property within the group. Equals one of the
 *                 si4467_property_group_* values.
 * @param data Desired value of the property.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_set_property(struct si4467_context *const ctx,
				const enum si4467_property_group group,
				const uint8_t property, const uint8_t data);

/**
 * @brief Get the value of one or more properties.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param group Group of the properties to query
 * @param property Offset of the property within the group. Equals one of the
 *                 si4467_property_group_* values.
 * @param values Array to fill with property values. Must hold at least \p len
 *               bytes.
 * @param len Number of properties to fetch. Must be no more than 16.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_get_properties(struct si4467_context *const ctx,
				  const enum si4467_property_group group,
				  const uint8_t property, uint8_t *data,
				  const uint8_t len);

/**
 * @brief Query currently active PLL Synthesizer configuration.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param band Pointer to fill with the active settings.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_clkgen_band(
	struct si4467_context *const ctx,
	struct si4467_property_modem_clkgen_band *band);

/**
 * @brief Query currently set frac-N PLL synthesizer integer divide number.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param inte Pointer to fill with the active divide number.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_freq_control_inte(struct si4467_context *const ctx,
					  uint8_t *inte);

/**
 * @brief Query currently set frac-N PLL synthesizer fraction number.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param frac Pointer to fill with the active fraction number.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_freq_control_frac(struct si4467_context *const ctx,
					  uint32_t *frac);

/**
 * @brief Query currently set channel step size.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param channel_step_size_hz Pointer to fill with the active channel step size
 *                             in Herz.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_freq_control_channel_step_size(
	struct si4467_context *const ctx, uint16_t *channel_step_size_hz);

/**
 * @brief Query PLL Synthesizer output divider ratio for \p band.
 *
 * @param band Band to query
 *
 * @return VCO divisor for \p band.
 */
uint8_t si4467_property_modem_clkgen_band_band_to_outdiv(
	const enum si4467_property_modem_clkgen_band_band band);

/**
 * @brief Get a human readably string for a device state.
 */
const char *request_device_state_reply_to_string(enum operating_state state);

/**
 * @brief Request current device state from Si4467.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param out Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_request_device_state(struct si4467_context *ctx,
					struct request_device_state_reply *out);

/**
 * @brief Check if \p state is a valid next operation state value.
 *
 * @param state Potential integer representation of an operating state.
 *
 * @return True if \p state is a valid operation state, false otherwise.
 */
bool si4467_is_valid_next_operating_state(uint8_t state);

/**
 * @brief Manually switch the chip to a desired operating state.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param next_state Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_change_state(struct si4467_context *ctx,
				enum operating_state next_state);
/**
 * @brief Return the interrupt status of the Chip Interrupt Group (both STATUS
 * and PENDING). Optionally, may be used to clear latched (PENDING) interrupt
 * events.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve Interrupts to be preserved (1) or cleared (0) after reading
 * them out.
 * @param out Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_get_chip_status(struct si4467_context *ctx,
				   struct chip_interrupt preserve,
				   struct chip_status_reply_detailed *out);

/**
 * @brief Return the interrupt status of the Packet Handler Interrupt Group
 * (both STATUS and PENDING). Optionally, may be used to clear latched (PENDING)
 * interrupt events.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve Interrupts to be preserved (1) or cleared (0) after reading
 * them out.
 * @param out Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_get_packet_handler_status(
	struct si4467_context *ctx, struct packet_handler_interrupt preserve,
	struct packet_handler_status_reply *out);

/**
 * @brief Reset TX and/or RX FIFO and get current FIFO sizes.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param reset_rx_fifo If true, RX FIFO gets reset.
 * @param reset_tx_fifo If true, TX FIFO gets reset.
 * @param info Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_fifo_info(struct si4467_context *ctx, bool reset_rx_fifo,
			     bool reset_tx_fifo, struct fifo_information *info);

/**
 * @brief Set and/or get the current RX length.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param field Filed to be updated
 * @param rx_len_new New RX length (only applicable if \p f is not equal to
 * PACKET_INFO_FIELD_NUM_OVERRIDE_NONE)
 * @param rx_len_currently [out] If non-NULL, update with the current variable
 * length field value.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_packet_info(struct si4467_context *ctx,
			       enum packet_info_field_num_override field,
			       uint16_t rx_len_new, uint16_t *rx_len_currently);

/**
 * @brief Return the interrupt status of the Modem Interrupt events (both STATUS
 * and PENDING). Optionally, may be used to clear latched (PENDING) interrupt
 * events.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve Interrupts to be preserved (1) or cleared (0) after reading
 * them out.
 * @param out Pointer to be updated with fetched information.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_get_modem_status(struct si4467_context *ctx,
				    struct modem_interrupt preserve,
				    struct modem_status_reply_detailed *out);

/**
 * @brief Extract current chip status and optionally reset pending interrupts.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve_interrupts Interrupts which should be *not* reset.
 * @param chip_status Pointer be updated with chip status details.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_get_chip_status(struct si4467_context *ctx,
			   struct chip_interrupt preserve_interrupts,
			   struct chip_status_reply_detailed *chip_status);

/**
 * @brief Extract interrupt status, both, status and pending, of all possible
 *	  interrupt events.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve Interrupts which should *not* be reset. NULL to reset all.
 * @param int_status Pointer be updated with chip status details. NULL to ignore
 * then.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_command_get_int_status(struct si4467_context *ctx,
				  const struct int_status_interrupt *preserve,
				  struct int_status_reply *int_status);

/**
 * @brief Switch to RX state, receive up to \rx_len bytes.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param condition
 * @param rx_len Bytes to receive using the settings of field #1 [1, 8191]
 *		 If set to 0, the number of bytes to received is determined by
 *		 the values(s) of the PKT_FIELD_X_LENGTH properties.
 * @param rx_channel Si4467 channel to receive on
 * @param next_timeout Next state upon timeout of preamble detection.
 * @param next_valid Next state upon reception of a valid packet.
 * @param next_invalid Next state upon reception of an invalid packet.
 *
 * @note For \p next_timeout, \p next_valid, \p next_invalid, the difference
 *	 between OPERATING_STATE_REMAIN and OPERATING_STATE_RX is that only the
 *	 later does re-arm and aquire another packet.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_start_rx(struct si4467_context *const ctx,
			    const struct start_rx_condition condition,
			    const uint16_t rx_len,
			    const uint8_t rx_channel,
			    const enum operating_state next_timeout,
			    const enum operating_state next_valid,
			    const enum operating_state next_invalid);

/**
 * @brief Switch to TX state, send data previously submitted to TX FIFO queue.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param txlen If 0, number of data bytes is determined by PKT_FIELD_X_LENGTH
 *		and payload length is limited to 5 x 8191 = 40955 bytes.
 *		If [1, 8191], txlen numbers of bytes will be sent, respecting
 *		the field configuration options like CRC, data whitening,
 *		Manchester coding, etc. of only PKT_FIELD_1_X.
 * @param tx_channel Channel number on which to transmit.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_command_start_tx(struct si4467_context *ctx,
						uint16_t txlen,
						const uint8_t tx_channel);

/**
 * @brief Extract current modem status and optionally reset pending interrupts.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve_interrupts Interrupts which should be *not* reset.
 * @param modem_status Pointer be updated with modem status details.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_get_modem_status(struct si4467_context *ctx,
			    struct modem_interrupt preserve_interrupts,
			    struct modem_status_reply_detailed *modem_status);

/**
 * @brief Extract current packet_handler status and optionally reset pending
 * interrupts.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param preserve_interrupts Interrupts which should be *not* reset.
 * @param packet_handler_status Pointer be updated with packet_handler status
 * details.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_get_packet_handler_status(
	struct si4467_context *ctx,
	struct packet_handler_interrupt preserve_interrupts,
	struct packet_handler_status_reply *packet_handler_status);

/**
 * @brief Receive as many bytes as possible from the RX FIFO and print it to the
 * console.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 *
 * @return Number of read-out bytes, negative errno code on failure.
 */
ssize_t si4467_drain_rx_fifo_data(struct si4467_context *ctx);

/**
 * Put as much data as possible into the TX FIFO.
 *
 * Depending on the FIFO configuration, the maximum number of bytes being sent
 * is either 129 or 64 bytes.
 *
 * @pre The packet cursor must start with the first byte to send.
 * @post The packet cursor points behind the last sent byte.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param pkt Data to be sent to the TX FIFO. Cursor gets updated accordingly.
 *
 * @return Negative errno code on failure, number of submitted bytes otherwise.
 */
int si4467_fill_tx_fifo_buffer(struct si4467_context *ctx, struct net_pkt *pkt);

/**
 * Receive all data available (max of either 64 or 129 bytes) in RX FIFO.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param pkt Packet to append data to.
 *
 * @return Negative errno code on failure, number of read bytes otherwise.
 */
int si4467_fetch_rx_fifo_buffer(struct si4467_context *ctx,
				struct net_pkt *pkt);

/**
 * @brief Update the MAC address in \p ctx
 *
 * @param ctx Pointer to the device structure for the driver instance.
 *
 * @return Pointer to si4467_context::mac_addr
 */
uint8_t *si4467_get_mac(struct si4467_context *ctx);

/**
 * @brief Configure, enable or disable RX frequency hopping.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param config Settings to apply
 * @param channels \p count channels to tune to cyclically.
 * @param count Number of elements in \p channels.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_configure_rx_hopping(
	struct si4467_context *ctx,
	const struct si4467_property_group_rx_hop_control *config,
	const uint8_t *channels, uint8_t count);

/**
 * @brief Query currently set RX hopping settings.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param control Pointer to fill with the active RX hopping settings.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_rx_hop_control(
	struct si4467_context *const ctx,
	struct si4467_property_group_rx_hop_control *control);

/**
 * @brief Query current power level setting.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param power_level Pointer to fill with power level.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_pwr_lvl(struct si4467_context *const ctx,
				uint8_t *power_level);

/**
 * @brief Get power level setting for a given dBm power level.
 *
 * @param tx_power_dbm Desired power level in dBm
 * @return closest power amplifier setting for given dBm power level
 */
uint8_t si4467_pa_dbm_to_pwr_lvl(uint8_t tx_power_dbm);

/**
 * @brief Set current power level.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param power_level Power level to set (valid range: 0..0x7f).
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_set_property_pwr_lvl(struct si4467_context *const ctx,
				uint8_t power_level);

/**
 * @brief Query the frequency of the crystal oscillator.
 *
 * @return Frequency of the crystal oscillator in Hz.
 */
uint32_t si4467_get_xtal_frequency(void);

/**
 * @brief Initialize the generic Si4467 parts.
 *
 * @param dev Pointer to the device's private data.
 * @param si4467_thread_nirq_fn Pointer to si4467_thread_nirq_signature
 * function.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_init(const struct device *dev,
		si4467_thread_nirq_callback_t si4467_thread_nirq_fn);

/**
 * @brief Configure default GPIO pin functionality.
 *
 * @return 0 if successful, negative errno code otherwise.
 */
int si4467_setup_gpio_pin_config(struct si4467_context *const ctx);

/**
 * @brief Get current crystal tune value.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param tune_value Pointer be updated with current tune value.
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_command_get_xo_tune_value(struct si4467_context *const ctx,
				     uint8_t *tune_value);

/**
 * @brief Set crystal oscillator tune value.

 * @param ctx Pointer to the device structure for the driver instance.
 * @param tune_value Oscillator capacitance tune value to set (range: 0 <=
 * tune_value <= 127, with 0 being the lowest capacitance / highest oscillator
 * frequency).
 *
 * @return 0 if successful, negative errno code on failure.
 */
int si4467_command_set_xo_tune_value(struct si4467_context *const ctx,
				     const uint8_t tune_value);

#if CONFIG_TRX_SI4467_CALIBRATION_PERSISTENT_STORAGE
/**
 * @brief Save current calibration value for crystal capacitance to persistent
 * settings.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @return Negative errno code on failure, 0 on success.
 */
int si4467_save_calibration(struct si4467_context *const ctx);

/**
 * @brief Restore and apply crystal calibration tune value from settings.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 *
 * @return Negative errno code on failure, 0 on success.
 */
int si4467_restore_calibration(struct si4467_context *const ctx);

#endif

/**
 * @brief Clear channel assessment
 *
 * @return 0 when channel is free
 *         -EBUSY when channel is occupied
 *         Negative errno code on failure
 */
int si4467_cca(const struct device *dev);

/**
 * @brief Reset and boot transceiver.
 *
 * @param dev Pointer to the device's private data.
 *
 * @return Negative errno code on failure, 0 on success.
 */
int si4467_start(const struct device *dev);

/**
 * @brief Fetch current device state and print it to the log.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 *
 * @return Negative errno code on failure, 0 on success.
 */
int si4467_get_and_log_device_state(struct si4467_context *ctx);

/**
 * @brief Log an easy-to-understand representation of active interrupts.
 *
 * In this context, an interrupt is denoted as active when either the status or
 * the pending bit is set. Nothing gets logged when no interrupt is active.
 *
 * @param status Status to log
 * @param log_level One of the LOG_LEVEL_* values.
 */
void si4467_log_chip_status(const struct chip_status_reply *status,
			    const uint8_t log_level);

/**
 * @brief Like \p si4467_log_chip_status(), but also prints inactive interrupts.
 */
void si4467_log_chip_status_full(const struct chip_status_reply *status,
				 const uint8_t log_level);

/**
 * @copydoc si4467_log_chip_status()
 */
void si4467_log_modem_status(const struct modem_status_reply *status,
			     const uint8_t log_level);

/**
 * @brief Like \p si4467_log_modem_status(), but also prints inactive
 * interrupts.
 */
void si4467_log_modem_status_full(const struct modem_status_reply *status,
				  const uint8_t log_level);

/**
 * @copydoc si4467_log_chip_status()
 */
void si4467_log_packet_handler_status(
	const struct packet_handler_status_reply *status,
	const uint8_t log_level);

/**
 * @brief Like \p si4467_log_packet_handler_status(), but also prints inactive
 * interrupts.
 */
void si4467_log_packet_handler_status_full(
	const struct packet_handler_status_reply *status,
	const uint8_t log_level);

/**
 * @brief Log a hexadecimal representation of the set chip interrupts/status
 *
 * @param status chip status
 * @param log_level One of the LOG_LEVEL_* values.
 */
void si4467_log_unhandled_chip_interrupts(
	const struct chip_status_reply *status, uint8_t log_level);

/**
 * @brief Log a hexadecimal representation of the set modem interrupts/status
 *
 * @param status modem status
 * @param log_level One of the LOG_LEVEL_* values.
 */
void si4467_log_unhandled_modem_interrupts(
	const struct modem_status_reply *status, uint8_t log_level);

/**
 * @brief Log a hexadecimal representation of the set packet handler
 * interrupts/status
 *
 * @param status packet handler status
 * @param log_level One of the LOG_LEVEL_* values.
 */
void si4467_log_unhandled_packet_handler_interrupts(
	const struct packet_handler_status_reply *status, uint8_t log_level);

/**
 * @brief Evaluate if at least one packet handler interrupt sources is set.
 *
 * @param i packet handler interrupt
 */
bool si4467_packet_handler_interrupt_present(struct packet_handler_interrupt i);
/**
 * @brief Evaluate if at least one of the modem interrupt sources is set.
 *
 * @param i modem interrupt
 */
bool si4467_modem_interrupt_present(struct modem_interrupt i);
/**
 * @brief Evaluate if at least one interrupt source is set
 *
 * @param i chip interrupt
 */
bool si4467_chip_interrupt_present(struct chip_interrupt i);

/**
 * @brief Represent a chip_interrupt object as single byte.
 *
 * @param i chip interrupt
 */
uint8_t si4467_chip_interrupt_to_byte(struct chip_interrupt i);
/**
 * @brief Represent a modem_interrupt object as single byte.
 *
 * @param i modem interrupt
 */
uint8_t si4467_modem_interrupt_to_byte(struct modem_interrupt i);
/**
 * @brief Represent a packet_handler_interrupt object as single byte.
 *
 * @param i packet handler interrupt
 */
uint8_t
si4467_packet_handler_interrupt_to_byte(struct packet_handler_interrupt i);

/**
 * @brief Query the number of properties in \p group.
 *
 * @param group Property group to query
 *
 * @return Positive number on success, -EINVAL on error.
 */
ssize_t si4467_property_group_size(const enum si4467_property_group group);

/**
 * @brief Get a textual representation of \p group.
 *
 * @param group Property group to query
 *
 * @return C string representation
 */
const char *si4467_property_group_str(const enum si4467_property_group group);

/**
 * @brief Get a textual representation of \p sy_sel.
 *
 * @param sy_sel Band selector to query
 *
 * @return C string representation
 */
const char *si4467_property_modem_clkgen_band_sy_sel_str(
	const enum si4467_property_modem_clkgen_band_sy_sel sy_sel);

/**
 * @brief Get current PA bias and clock duty cycle values.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param bias Return pointer of the bias value.
 * @param clk_duty Return pointer of the clk duty cycle value.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_get_property_pa_bias_clkduty(struct si4467_context *const ctx,
					uint8_t *bias, uint8_t *clk_duty);

/**
 * @brief Set PA bias and clk duty cycle.
 *
 * @param ctx Pointer to the device structure for the driver instance.
 * @param bias Bias value.
 * @param clk_duty Clock duty cycle value.
 *
 * @return 0 If successful, negative errno code otherwise.
 */
int si4467_set_property_pa_bias_clkduty(struct si4467_context *const ctx,
					const uint8_t bias,
					const uint8_t clk_duty);

#endif /* ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_H_ */
