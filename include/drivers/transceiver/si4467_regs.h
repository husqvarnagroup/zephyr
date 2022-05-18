/* si4467_regs.h - Registers definition for Silicon Labs Si4467 */

/*
 * Copyright (c) 2020 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_REGS_H_
#define ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_REGS_H_

#include <stdint.h>

enum si4467_command {
	/* BOOT_COMMANDS - available only in bootloader */
	/* Command to power-up the device and select the operational mode and
	 * functionality.
	 */
	COMMAND_POWER_UP = 0x02,

	/* COMMON_COMMANDS */
	/* No Operation command. */
	COMMAND_NOP = 0x00, /* Also available in bootloader. */
	/* Basic device information */
	COMMAND_PART_INFO = 0x01, /* Also available in bootloader. */
	/* Function revision information */
	COMMAND_FUNC_INFO = 0x10, /* Also available in bootloader. */
	/* Set the value of one or more properties. */
	COMMAND_SET_PROPERTY = 0x11,
	/* Retrieves the value of one or more properties */
	COMMAND_GET_PROPERTY = 0x12,
	/* Configures the GPIO pins. */
	COMMAND_GPIO_PIN_CFG = 0x13,
	/* Access the current byte counts in the TX and RX FIFOs, and provide
	 * for resetting the FIFOs.
	 */
	COMMAND_FIFO_INFO = 0x15,
	/* Returns the interrupt status of ALL the possible interrupt events
	 * (both STATUS and PENDING). Optionally, it may be used to clear
	 * latched (PENDING) interrupt events.
	 */
	COMMAND_GET_INT_STATUS = 0x20,
	/* Request current device state and channel. */
	COMMAND_REQUEST_DEVICE_STATE = 0x33,
	/* Manually switch the chip to a desired operating state. */
	COMMAND_CHANGE_STATE = 0x34,
	/* Switches to high temp or low temp settings and recalibrate blocks. */
	COMMAND_OFFLINE_RECAL = 0x38,
	/* Read CTS and the command response. */
	COMMAND_READ_CMD_BUFF = 0x44, /* Also available in bootloader. */
	/* Reads the fast response registers (FRR) starting with FRR_A. */
	COMMAND_FRR_A_READ = 0x50,
	/* Reads the fast response registers (FRR) starting with FRR_B. */
	COMMAND_FRR_B_READ = 0x51,
	/* Reads the fast response registers (FRR) starting with FRR_C. */
	COMMAND_FRR_C_READ = 0x53,
	/* Reads the fast response registers (FRR) starting with FRR_D. */
	COMMAND_FRR_D_READ = 0x57,

	/* IR_CAL_COMMANDS */
	/* Image rejection calibration. */
	COMMAND_IRCAL = 0x17,
	/* Image rejection calibration. Again?! */
	COMMAND_IRCAL_MANUAL = 0x1A,

	/* TX_COMMANDS */
	/* Switches to TX state and starts transmission of a packet. */
	COMMAND_START_TX = 0x31,
	/* Hop to a new frequency while in TX. */
	COMMAND_TX_HOP = 0x37,
	/* Writes data byte(s) to the TX FIFO. */
	COMMAND_WRITE_TX_FIFO = 0x66,

	/* RX_COMMANDS */
	/* Returns information about the length of the variable field in the
	 * last packet received, and (optionally) overrides field length.
	 */
	COMMAND_PACKET_INFO = 0x16,
	/* Returns the interrupt status of the Modem Interrupt Group (both
	 * STATUS and PENDING). Optionally, it may be used to clear latched
	 * (PENDING) interrupt events.
	 */
	COMMAND_GET_MODEM_STATUS = 0x22,
	/* Switches to RX state and starts reception of a packet. */
	COMMAND_START_RX = 0x32,
	/* Manually hop to a new frequency while in RX mode. */
	COMMAND_RX_HOP = 0x36,
	/* Reads data byte(s) from the RX FIFO. */
	COMMAND_READ_RX_FIFO = 0x77,

	/* ADVANCED_COMMANDS */
	/* Performs conversions using the Auxiliary ADC and returns the results
	 * of those conversions.
	 */
	COMMAND_GET_ADC_READING = 0x14,
	/* Returns the interrupt status of the Packet Handler Interrupt Group
	 * (both STATUS and PENDING). Optionally, it may be used to clear
	 * latched (PENDING) interrupt events.
	 */
	COMMAND_GET_PH_STATUS = 0x21,
	/* Interrupt status of the Chip Interrupt Group (STATUS and PENDING).
	 * May be used to clear latched (PENDING) interrupt events.
	 */

	COMMAND_GET_CHIP_STATUS = 0x23, /* Also available in bootloader. */
};

enum si4467_property_group {
	SI4467_PROPERTY_GROUP_GLOBAL = 0x00,
	SI4467_PROPERTY_GROUP_INT_CTL = 0x01,
	SI4467_PROPERTY_GROUP_FRR_CTL = 0x02,
	SI4467_PROPERTY_GROUP_PREAMBLE = 0x10,
	SI4467_PROPERTY_GROUP_SYNC = 0x11,
	SI4467_PROPERTY_GROUP_PKT = 0x12,
	SI4467_PROPERTY_GROUP_MODEM = 0x20,
	SI4467_PROPERTY_GROUP_MODEM_CHFLT = 0x21,
	SI4467_PROPERTY_GROUP_PA = 0x22,
	SI4467_PROPERTY_GROUP_SYNTH = 0x23,
	SI4467_PROPERTY_GROUP_MATCH = 0x30,
	SI4467_PROPERTY_GROUP_FREQ_CONTROL = 0x40,
	SI4467_PROPERTY_GROUP_RX_HOP = 0x50,
	SI4467_PROPERTY_GROUP_PTI = 0xF0,
};

enum {
	/** No more than 12 properties can be set in the same SPI transaction */
	SI4467_PROPERTY_MAX_SET_COUNT = 12,
};

enum {
	SI4467_PROPERTY_GROUP_GLOBAL_SIZE = 0x0B,
	SI4467_PROPERTY_GROUP_INT_CTL_SIZE = 0x04,
	SI4467_PROPERTY_GROUP_FRR_CTL_SIZE = 0x04,
	SI4467_PROPERTY_GROUP_PREAMBLE_SIZE = 0x0E,
	SI4467_PROPERTY_GROUP_SYNC_SIZE = 0x0A,
	SI4467_PROPERTY_GROUP_PKT_SIZE = 0x3A,
	SI4467_PROPERTY_GROUP_MODEM_SIZE = 0x60,
	SI4467_PROPERTY_GROUP_MODEM_CHFLT_SIZE = 0x24,
	SI4467_PROPERTY_GROUP_PA_SIZE = 0x07,
	SI4467_PROPERTY_GROUP_SYNTH_SIZE = 0x08,
	SI4467_PROPERTY_GROUP_MATCH_SIZE = 0x0C,
	SI4467_PROPERTY_GROUP_FREQ_CONTROL_SIZE = 0x08,
	SI4467_PROPERTY_GROUP_RX_HOP_SIZE = 0x42,
	SI4467_PROPERTY_GROUP_PTI_SIZE = 0x04,
};

enum si4467_property_group_global {
	/* Configure the internal capacitor frequency tuning bank for the
	 * crystal oscillator.
	 */
	SI4467_PROPERTY_GLOBAL_XO_TUNE = 0x00,
	SI4467_PROPERTY_GLOBAL_CLK_CFG = 0x01,
	SI4467_PROPERTY_GLOBAL_LOW_BAT_THRESH = 0x02,
	SI4467_PROPERTY_GLOBAL_CONFIG = 0x03,
	SI4467_PROPERTY_GLOBAL_WUT_CONFIG = 0x04,
	SI4467_PROPERTY_GLOBAL_WUT_M = 0x05,
	SI4467_PROPERTY_GLOBAL_WUT_R = 0x07,
	SI4467_PROPERTY_GLOBAL_WUT_LDC = 0x08,
	SI4467_PROPERTY_GLOBAL_WUT_CAL = 0x09,
	SI4467_PROPERTY_GLOBAL_BUFCLK_CFG = 0x0A,
};

enum si4467_property_global_clk_cfg {
	SI4467_PROPERTY_GLOBAL_CLK_CFG_DIVIDED_CLOCK_EN = (1<<6),
};

enum si4467_property_group_int_ctl {
	/* This property provides for global enabling of the three interrupt
	 * groups (Chip, Modem and Packet Handler) in order to generate HW
	 * interrupts at the NIRQ pin.
	 */
	SI4467_PROPERTY_INT_CTL_ENABLE = 0x01,
	/* TODO: Rest */
};

enum si4467_property_group_frr_ctl {
	/* Fast Response Register A Configuration. */
	SI4467_PROPERTY_FRR_CTL_A_MODE = 0x00,
	/* Fast Response Register B Configuration. */
	SI4467_PROPERTY_FRR_CTL_B_MODE = 0x01,
	/* Fast Response Register C Configuration. */
	SI4467_PROPERTY_FRR_CTL_C_MODE = 0x02,
	/* Fast Response Register D Configuration. */
	SI4467_PROPERTY_FRR_CTL_D_MODE = 0x03,
};

enum si4467_property_group_preamble {
	/* Configure length of TX Preamble. */
	SI4467_PROPERTY_PREAMBLE_TX_LENGTH = 0x00,
	/* TODO: Rest */
};

enum si4467_property_group_sync {
	/* Sync Word configuration bits. */
	SI4467_PROPERTY_SYNC_CONFIG = 0x00,
	/* TODO: Rest */
};

enum si4467_property_group_pkt {
	/* Select a CRC polynomial and seed. */
	SI4467_PROPERTY_PKT_CRC_CONFIG = 0x00,
	/* TODO: Rest */
};

enum si4467_property_group_modem {
	/**
	 * Selects the type of modulation. In TX mode, additionally selects the
	 * source of the modulation.
	 */
	SI4467_PROPERTY_MODEM_MOD_TYPE = 0x00,
	/** Controls polarity and mapping of transmit and receive bits. */
	SI4467_PROPERTY_MODEM_MAP_CONTROL = 0x01,
	/**
	 * Miscellaneous control bits for the Delta-Sigma Modulator (DSM) in
	 * the PLL Synthesizer.
	 **/
	SI4467_PROPERTY_MODEM_DSM_CTRL = 0x02,
	/**
	 * Unsigned 24-bit value used to determine the TX data rate.
	 */
	SI4467_PROPERTY_MODEM_DATA_RATE = 0x03,
	/**
	 * TX Gaussian filter oversampling ratio and Byte 3 of unsigned 26-bit
	 * TX Numerically Controlled Oscillator (NCO) modulus.
	 */
	SI4467_PROPERTY_MODEM_TX_NCO_MODE = 0x06,
	/** 17-bit unsigned TX frequency deviation word. */
	SI4467_PROPERTY_MODEM_FREQ_DEV = 0x0A,
	/** Frequency offset adjustment (a 16-bit signed number). */
	SI4467_PROPERTY_MODEM_FREQ_OFFSET = 0x0D,
	/** The 8th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_8 = 0x0F,
	/** The 7th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_7 = 0x10,
	/** The 6th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_6 = 0x11,
	/** The 5th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_5 = 0x12,
	/** The 4th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_4 = 0x13,
	/** The 3rd coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_3 = 0x14,
	/** The 2nd coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_2 = 0x15,
	/** The 1th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_1 = 0x16,
	/** The 0th coefficient of TX spectral shaping filter. */
	SI4467_PROPERTY_MODEM_TX_FILTER_COEFF_0 = 0x17,
	/** TX ramp-down delay setting. */
	SI4467_PROPERTY_MODEM_TX_RAMP_DELAY = 0x18,
	/** MDM control. */
	SI4467_PROPERTY_MODEM_MDM_CTRL = 0x19,
	/**
	 * Selects Fixed-IF, Scaled-IF, or Zero-IF mode of RX Modem operation.
	 */
	SI4467_PROPERTY_MODEM_IF_CONTROL = 0x1A,
	/** The IF frequency setting (an 18-bit signed number) */
	SI4467_PROPERTY_MODEM_IF_FREQ = 0x1B,
	/**
	 * Specifies three decimator ratios for the Cascaded Integrator Comb
	 * (CIC) filter.
	 */
	SI4467_PROPERTY_MODEM_DECIMATION_CFG1 = 0x1E,
	/**
	 * Specifies miscellaneous parameters and decimator ratios for the
	 * Cascaded Integrator Comb (CIC) filter.
	 */
	SI4467_PROPERTY_MODEM_DECIMATION_CFG0 = 0x1F,
	/** Specifies miscellaneous decimator filter selections. */
	SI4467_PROPERTY_MODEM_DECIMATION_CFG2 = 0x20,
	/** Not documented */
	SI4467_PROPERTY_MODEM_IFPKD_THRESHOLDS = 0x21,
	/** RX BCR/Slicer oversampling rate (12-bit unsigned number). */
	SI4467_PROPERTY_MODEM_BCR_OSR = 0x22,
	/** RX BCR NCO offset value (an unsigned 22-bit number). */
	SI4467_PROPERTY_MODEM_BCR_NCO_OFFSET = 0x24,
	/** The unsigned 11-bit RX BCR loop gain value. */
	SI4467_PROPERTY_MODEM_BCR_GAIN = 0x27,
	/** RX BCR loop gear control. */
	SI4467_PROPERTY_MODEM_BCR_GEAR = 0x29,
	/** Miscellaneous control bits for the RX BCR loop. */
	SI4467_PROPERTY_MODEM_BCR_MISC1 = 0x2A,
	/** Miscellaneous RX BCR loop controls. */
	SI4467_PROPERTY_MODEM_BCR_MISC0 = 0x2B,
	/** RX AFC loop gear control. */
	SI4467_PROPERTY_MODEM_AFC_GEAR = 0x2C,
	/** RX AFC loop wait time control. */
	SI4467_PROPERTY_MODEM_AFC_WAIT = 0x2D,
	/**
	 * Sets the gain of the PLL-based AFC acquisition loop, and provides
	 * miscellaneous control bits for AFC functionality.
	 */
	SI4467_PROPERTY_MODEM_AFC_GAIN = 0x2E,
	/** Set the AFC limiter value. */
	SI4467_PROPERTY_MODEM_AFC_LIMITER = 0x30,
	/** Specifies miscellaneous AFC control bits. */
	SI4467_PROPERTY_MODEM_AFC_MISC = 0x32,
	/** AFC fixed frequency offset in zero IF mode. */
	SI4467_PROPERTY_MODEM_AFC_ZIFOFF = 0x33,
	/** Sigma Delta ADC controls. */
	SI4467_PROPERTY_MODEM_ADC_CTRL = 0x34,
	/**
	 * Miscellaneous control bits for the Automatic Gain Control (AGC)
	 * function in the RX Chain.
	 */
	SI4467_PROPERTY_MODEM_AGC_CONTROL = 0x35,
	/**
	 * Specifies the size of the measurement and settling windows for the
	 * AGC algorithm.
	 */
	SI4467_PROPERTY_MODEM_AGC_WINDOW_SIZE = 0x38,
	/** Sets the decay time of the RF peak detectors. */
	SI4467_PROPERTY_MODEM_AGC_RFPD_DECAY = 0x39,
	/** Sets the decay time of the IF peak detectors. */
	SI4467_PROPERTY_MODEM_AGC_IFPD_DECAY = 0x3A,
	/**
	 * Specifies the gain factor of the secondary branch in 4(G)FSK ISI-
	 * suppression.
	 */
	SI4467_PROPERTY_MODEM_FSK4_GAIN1 = 0x3B,
	/**
	 * Specifies the gain factor of the primary branch in 4(G)FSK ISI-
	 * suppression.
	 */
	SI4467_PROPERTY_MODEM_FSK4_GAIN0 = 0x3C,
	/** 16 bit 4(G)FSK slicer threshold. */
	SI4467_PROPERTY_MODEM_FSK4_TH = 0x3D,
	/** 4(G)FSK symbol mapping code. */
	SI4467_PROPERTY_MODEM_FSK4_MAP = 0x3F,
	/** Configures the attack and decay times of the OOK Peak Detector.*/
	SI4467_PROPERTY_MODEM_OOK_PDTC = 0x40,
	/** Configures the slicing reference level of the OOK Peak Detector. */
	SI4467_PROPERTY_MODEM_OOK_BLOPK = 0x41,
	/** OOK control. */
	SI4467_PROPERTY_MODEM_OOK_CNT1 = 0x42,
	/**
	 * Selects the detector(s) used for demodulation of an OOK signal, or
	 * for demodulation of a (G)FSK signal when using the asynchronous
	 * demodulator.
	 */
	SI4467_PROPERTY_MODEM_OOK_MISC = 0x43,
	/** Defines gain and enable controls for raw / nonstandard mode. */
	SI4467_PROPERTY_MODEM_RAW_CONTROL = 0x45,
	/** 11 bit eye-open detector threshold. */
	SI4467_PROPERTY_MODEM_RAW_EYE = 0x46,
	/** Antenna diversity mode settings. */
	SI4467_PROPERTY_MODEM_ANT_DIV_MODE = 0x48,
	/** Specifies controls for the Antenna Diversity algorithm. */
	SI4467_PROPERTY_MODEM_ANT_DIV_CONTROL = 0x49,
	/** Configures the RSSI threshold. */
	SI4467_PROPERTY_MODEM_RSSI_THRESH = 0x4A,
	/** Configures the RSSI Jump Detection threshold. */
	SI4467_PROPERTY_MODEM_RSSI_JUMP_THRESH = 0x4B,
	/**
	 * Control of the averaging modes and latching time for reporting RSSI
	 * value(s).
	 */
	SI4467_PROPERTY_MODEM_RSSI_CONTROL = 0x4C,
	/** RSSI Jump Detection control. */
	SI4467_PROPERTY_MODEM_RSSI_CONTROL2 = 0x4D,
	/** RSSI compensation value. */
	SI4467_PROPERTY_MODEM_RSSI_COMP = 0x4E,
	/**
	 * Defines and controls the search period length for the Moving Average
	 * and Min-Max detectors.
	 */
	SI4467_PROPERTY_MODEM_RAW_SEARCH2 = 0x50,
	/**
	 * Select PLL Synthesizer output divider ratio as a function of
	 * frequency band.
	 */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND = 0x51,
	/** Configures the threshold for (G)FSK Spike Detection. */
	SI4467_PROPERTY_MODEM_SPIKE_DET = 0x54,
	/**
	 * Configures parameters for th e One Shot AFC function and for BCR
	 * timing/acquisition.
	 */
	SI4467_PROPERTY_MODEM_ONE_SHOT_AFC = 0x55,
	/**
	 * Configures the amount of hysteresis on the RSSI threshold detection
	 * function.
	 */
	SI4467_PROPERTY_MODEM_RSSI_HYSTERESIS = 0x56,
	/** Configures muting of the RSSI to avoid false RSSI interrupts. */
	SI4467_PROPERTY_MODEM_RSSI_MUTE = 0x57,
	/** Configures the delay for fast RSSI Latching mode. */
	SI4467_PROPERTY_MODEM_FAST_RSSI_DELAY = 0x58,
	/** Configures the Preamble Sense Mode feature */
	SI4467_PROPERTY_MODEM_PSM = 0x59,
	/**
	 * Configures parameters for the Signal Arrival Detection circuit block
	 * and algorithm.
	 */
	SI4467_PROPERTY_MODEM_DSA_CTRL1 = 0x5B,
	/** @see SI4467_PROPERTY_MODEM_DSA_CTRL1 */
	SI4467_PROPERTY_MODEM_DSA_CTRL2 = 0x5C,
	/**
	 * Configures parameters for the Eye Opening qualification m ethod of
	 * the Signal Arrival Detection algorithm.
	 */
	SI4467_PROPERTY_MODEM_DSA_QUAL = 0x5D,
	/** Signal Arrival Detect RSSI Qualifier Config */
	SI4467_PROPERTY_MODEM_DSA_RSSI = 0x5E,
	/** Miscellaneous detection of signal arrival bits. */
	SI4467_PROPERTY_MODEM_DSA_MISC = 0x5F,
};

enum si4467_property_group_modem_chflt {
	/* Filter coefficients for the first set of RX filter coefficients. */
	SI4467_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE = 0x00,
	/* TODO: Rest */
};

enum si4467_property_modem_clkgen_band_force_sy_recal {
	/** Force Recalibration. */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_FORCE_SY_RECAL_FORCE = 0x00,
	/** Skip recalibration if frequency is not changed. */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_FORCE_SY_RECAL_SKIP = 0x01,
};

enum si4467_property_modem_clkgen_band_sy_sel {
	/**
	 * Low-power mode (fixed prescaler = Div-by-4). Approximately 200 uA
	 * less current than High Performance mode, but with coarser tuning
	 * resolution of the PLL Synthesizer.
	 */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_LOW_POWER = 0x00,
	/** High Performance mode (fixed prescaler = Div-by-2). */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_SY_SEL_HIGH_PERFORMANCE = 0x01,
};

/**
 * @brief Selects the PLL Synthesizer output divider ratio appropriate for the
 *        desired frequency band.
 *
 * The VCO in the PLL Synthesizer operates at approximately 3.6 GHz; operation
 * across a wide range of frequency bands is obtained by switching in a
 * configurable divider (OUTDIV) at the output of the PLL Synthesizer, e.g.,
 * OUTDIV = Div-by-4 for 900 MHz band.
 */
enum si4467_property_modem_clkgen_band_band {
	/** 3.6 GHz / 4 = 900 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_4 = 0x00,
	/** 3.6 GHz / 6 = 600 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_6 = 0x01,
	/** 3.6 GHz / 8 = 450 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_8 = 0x02,
	/** 3.6 GHz / 12 = 300 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_12 = 0x03,
	/** 3.6 GHz / 16 = 225 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_16 = 0x04,
	/** 3.6 GHz / 24 = 150 MHz */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24 = 0x05,
	/** 3.6 GHz / 24 = 150 MHz (?) */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_2 = 0x06,
	/** 3.6 GHz / 24 = 150 MHz (?) */
	SI4467_PROPERTY_MODEM_CLKGEN_BAND_BAND_FVCO_DIV_24_3 = 0x07,
};

struct si4467_property_modem_clkgen_band {
	enum si4467_property_modem_clkgen_band_force_sy_recal force_sy_recal;
	enum si4467_property_modem_clkgen_band_sy_sel sy_sel;
	enum si4467_property_modem_clkgen_band_band band;
};

enum si4467_property_group_pa {
	/**
	 * Selects the PA operating mode, and selects resolution of PA power
	 * adjustment (i.e., step size).
	 */
	SI4467_PROPERTY_PA_MODE = 0x00,
	/** Configuration of PA output power level. */
	SI4467_PROPERTY_PA_PWR_LVL = 0x01,
	/**
	 * Configuration of the PA Bias and duty cycle of the TX clock source.
	 */
	SI4467_PROPERTY_PA_BIAS_CLKDUTY = 0x02,
	/** Configuration of PA ramping parameters. */
	SI4467_PROPERTY_PA_TC = 0x03,
	/** Select the time constant of the external PA ramp signal. */
	SI4467_PROPERTY_PA_RAMP_EX = 0x04,
	/**
	 * Delay from the start of the PA ramp down to disabling of the PA
	 * output.
	 */
	SI4467_PROPERTY_PA_RAMP_DOWN_DELAY = 0x05,
	/** Configuration for digital power sequencing. */
	SI4467_PROPERTY_PA_DIG_PWR_SEQ_CONFIG = 0x06,
};

enum si4467_property_reg_pa_bias_clkduty {
	/* Position of bias value in register */
	SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_POS = 0,
	/* Bit mask of bias */
	SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_BIAS_MASK = 0x3F,
	/* Position of clock duty value in register */
	SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS = 6,
	/* Bit mask of clock duty */
	SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_MASK =
		(3 << SI4467_PROPERTY_REG_PA_BIAS_CLKDUTY_CLKDUTY_POS),
};

enum si4467_property_group_synth {
	/* Feed forward charge pump current selection. */
	SI4467_PROPERTY_SYNTH_PFDCP_CPFF = 0x00,
	/* TODO: Rest */
};

enum si4467_property_group_match {
	/* Match value to be compared with the result of logically AND-ing (bit-
	 * wise) the Mask 1 value with the received Match 1 byte.
	 */
	SI4467_PROPERTY_MATCH_VALUE_1 = 0x00,
	/* TODO: Rest */
};

enum si4467_property_group_freq_control {
	/** Frac-N PLL Synthesizer integer divide number. */
	SI4467_PROPERTY_FREQ_CONTROL_INTE = 0x00,
	/** Frac-N PLL fraction number. */
	SI4467_PROPERTY_FREQ_CONTROL_FRAC = 0x01,
	/** EZ Frequency Programming channel step size. */
	SI4467_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE = 0x04,
	/**
	 * Set window gating period (in number of crystal reference clock
	 * cycles) for counting VCO frequency during calibration.
	 */
	SI4467_PROPERTY_FREQ_CONTROL_W_SIZE = 0x06,
	/** Adjust target count for VCO calibration in RX mode. */
	SI4467_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ = 0x07,
};

enum si4467_property_group_rx_hop {
	/* Configuration options for the automatic RX Hop functionality. */
	SI4467_PROPERTY_RX_HOP_CONTROL = 0x00,
	/* Specifies the number of entries (channels) in the RX Hop table. */
	SI4467_PROPERTY_RX_HOP_TABLE_SIZE = 0x01,
	/* Defines entries in the RX Hopping Table (N=0 to 63) */
	SI4467_PROPERTY_RX_HOP_TABLE_ENTRY_0 = 0x02,
	SI4467_PROPERTY_RX_HOP_TABLE_ENTRY_63 = 0x41,
};

enum si4467_property_group_rx_hop_control_en {
	/** Disable hopping */
	SI4467_PROPERTY_RX_HOP_CONTROL_EN_DISABLE = 0x00,
	/** Hop on INVALID_PREAMBLE event (i.e., RX Preamble timeout) */
	SI4467_PROPERTY_RX_HOP_CONTROL_EN_PM_TO = 0x01,
	/** Hop on INVALID_PREAMBLE or RSSI Timeout event(s). */
	SI4467_PROPERTY_RX_HOP_CONTROL_EN_PM_RSSI_PM_TO = 0x02,
	/** Hop on INVALID_PREAMBLE or invalid Sync Word event(s). */
	SI4467_PROPERTY_RX_HOP_CONTROL_EN_PM_SYNC_TO = 0x03,
	/**
	 * Hop on INVALID_PREAMBLE or RSSI Timeout or invalid Sync Word event(s)
	 */
	SI4467_PROPERTY_RX_HOP_CONTROL_EN_PM_RSSI_PM_SYNC_TO = 0x04,
};

struct si4467_property_group_rx_hop_control {
	/** RSSI timeout in nibbles */
	uint8_t rssi_timeout : 4;
	/** Disable or configure how to enable hopping */
	enum si4467_property_group_rx_hop_control_en enable;
};

enum {
	/** Maximum number of channels the RX hopping table can store */
	SI4467_PROPERTY_RX_HOP_TABLE_SIZE_MAX = 64,
};

enum si4467_property_group_pti {
	/* Packet Trace Interface control fields. */
	SI4467_PROPERTY_PTI_CTL = 0x00,
	/* Desired baud rate for the PTI interface. */
	SI4467_PROPERTY_PTI_BAUD_0 = 0x01,
	/* Desired baud rate for the PTI interface. */
	SI4467_PROPERTY_PTI_BAUD_1 = 0x02,
	/* Enables what the PTI logs. */
	SI4467_PROPERTY_PTI_LOG_EN = 0x03,
};

#endif /* ZEPHYR_DRIVERS_TRANSCEIVER_SI4467_REGS_H_ */
