#!/usr/bin/env python3

import sys
import pickle
import argparse
import os
import enum

sys.path.insert(
    0, os.path.join(os.environ["ZEPHYR_BASE"], "scripts/dts/python-devicetree/src")
)

from devicetree import edtlib


class Signal(enum.Enum):
    USART0_TX = 0
    USART0_RX = 1
    USART0_RTS = 2
    USART0_CTS = 3
    USART0_UCLK = 4

    USART1_TX = 9
    USART1_RX = 10
    USART1_RTS = 11
    USART1_CTS = 12
    USART1_UCLK = 13

    SPI2_SCK = 50
    SPI2_MISO = 51
    SPI2_MOSI = 52
    SPI2_NSS = 53


class PinMode(enum.Enum):
    ANALOG = 0
    DIGITAL_INPUT = 1
    PUSH_PULL_OUTPUT = 2


class Pinmux:
    def __init__(self, value, props):
        self.pin = (value & 0x7, (value >> 3) & 0xFF)
        self.signal = Signal((value >> 22) & 0x7F)

        output_low = props["output-low"].val
        output_high = props["output-high"].val
        output_enable = props["output-enable"].val or output_low or output_high

        input_enable = props["input-enable"].val

        if output_enable and input_enable:
            raise Exception("can't enable both output and input")
        if output_low and output_high:
            raise Exception("can't define output as both low and high")

        if input_enable:
            self.pinmode = PinMode.DIGITAL_INPUT
        elif output_enable:
            self.pinmode = PinMode.PUSH_PULL_OUTPUT

            if output_low:
                self.output_value = True
            elif output_high:
                self.output_value = False
            else:
                self.output_value = None
        else:
            self.pinmode = None

    def __repr__(self):
        return self.__dict__.__repr__()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "edt_pickle", help="path to read the pickled edtlib.EDT object from"
    )
    parser.add_argument("out", help="path to write the header file")

    return parser.parse_args()


class CrossbarBit:
    def __init__(self, bit, signals, pin_first, pin_last):
        assert pin_first <= pin_last

        self.bit = bit
        self.signals = signals
        self.pin_first = pin_first
        self.pin_last = pin_last

    def __repr__(self):
        return self.__dict__.__repr__()


class Crossbar:
    def __init__(self, id, bits, portbanks):
        self.id = id
        self.value = 0
        self.bits = bits
        self.first_configurable = (0, 0)
        self.portbanks = portbanks

    def enable_bit(self, bit, pins):
        if len(pins) != len(bit.signals):
            raise Exception(
                f"pins({pins}) and signals({bit.signals}) must be the same length"
            )

        for pin in pins:
            if pin < self.first_configurable:
                raise Exception(
                    "can't enable crossbar pin anymore", pin, self.first_configurable
                )

            self.portbanks[pin[0]].unskip(pin[1])

            self.first_configurable = (pin[0], pin[1] + 1)

        self.value |= 1 << bit.bit

    def mux(self, muxs):
        for bit in self.bits:
            # collect the signals that are enabled by this bit
            signal_muxs = {}
            for mux in muxs:
                if self.id == 0 and mux.pin[0] != 0 and mux.pin[0] != 1:
                    continue
                if self.id == 1 and mux.pin[0] != 2 and mux.pin[0] != 3:
                    continue

                if mux.signal not in bit.signals:
                    continue
                if mux.signal in signal_muxs:
                    raise Exception("duplicate signal", mux)
                if mux.pin < bit.pin_first or mux.pin > bit.pin_last:
                    raise Exception("can't mux signal to pin", mux, bit)

                signal_muxs[mux.signal] = mux

            # this bit is disabled
            if len(signal_muxs.keys()) == 0:
                continue
            # we have to enable all of the signals
            if len(signal_muxs.keys()) != len(bit.signals):
                raise Exception("missing signals for bit", bit, signal_muxs)

            # build pin list for this bit in the required order
            pins = []
            for index, signal in enumerate(bit.signals):
                mux = signal_muxs[signal]
                pins.append(mux.pin)

            self.enable_bit(bit, pins)

            for mux in signal_muxs.values():
                self.portbanks[mux.pin[0]].apply_mux(mux.pin[1], mux)

    def __repr__(self):
        return self.__dict__.__repr__()


class Portbank:
    def __init__(self):
        self.pins_high = 0
        self.pins_low = 0
        self.pins_push_pull_output = 0
        self.pins_digital_input = 0
        self.pins_analog = 0

        # skip all pins by default
        self.skip_enable = 0xFFFF

    def unskip(self, pin):
        self.skip_enable &= ~(1 << pin)

    def apply_mux(self, pin, mux):
        if mux.pinmode == PinMode.ANALOG:
            self.pins_analog |= 1 << pin
        elif mux.pinmode == PinMode.DIGITAL_INPUT:
            self.pins_digital_input |= 1 << pin
        elif mux.pinmode == PinMode.PUSH_PULL_OUTPUT:
            self.pins_push_pull_output |= 1 << pin

            if mux.output_value == True:
                self.pins_high |= 1 << pin
            elif mux.output_value == False:
                self.pins_low |= 1 << pin
            elif mux.output_value == None:
                pass
            else:
                raise Exception("unsupported output value", mux.output_value)
        elif mux.pinmode == None:
            pass
        else:
            raise Exception("unsupported pinmode", mux.pinmode)

    def __repr__(self):
        return self.__dict__.__repr__()


def main():
    args = parse_args()

    with open(args.edt_pickle, "rb") as f:
        edt = pickle.load(f)

    pinmux_table = []
    for node in edt.nodes:
        if node.status != "okay":
            continue
        if not node.pinctrls:
            continue

        pinctrl = None
        for p in node.pinctrls:
            if p.name != "default":
                continue

            if pinctrl is not None:
                raise Exception("multiple default nodes", node)
            pinctrl = p

        if pinctrl is None:
            raise Exception("no default node", node)

        for conf_node in pinctrl.conf_nodes:
            for child in conf_node.children.values():
                for pin in child.props["pinmux"].val:
                    pinmux_table.append(Pinmux(pin, child.props))

    crossbar0_bits = [
        CrossbarBit(0, [Signal.USART0_TX, Signal.USART0_RX], (0, 0), (1, 15)),
        CrossbarBit(1, [Signal.USART0_RTS, Signal.USART0_CTS], (0, 0), (1, 15)),
        CrossbarBit(2, [Signal.USART0_UCLK], (0, 0), (1, 15)),
        CrossbarBit(5, [Signal.USART1_TX, Signal.USART1_RX], (0, 0), (1, 15)),
        CrossbarBit(6, [Signal.USART1_RTS, Signal.USART1_CTS], (0, 0), (1, 15)),
        CrossbarBit(7, [Signal.USART1_UCLK], (0, 0), (1, 15)),
        CrossbarBit(
            32 + 3,
            [Signal.SPI2_SCK, Signal.SPI2_MISO, Signal.SPI2_MOSI],
            (0, 0),
            (1, 15),
        ),
        CrossbarBit(32 + 4, [Signal.SPI2_NSS], (0, 0), (1, 15)),
    ]
    crossbar1_bits = [
        CrossbarBit(
            7, [Signal.SPI2_SCK, Signal.SPI2_MISO, Signal.SPI2_MOSI], (2, 6), (3, 11)
        ),
        CrossbarBit(8, [Signal.SPI2_NSS], (2, 6), (3, 11)),
    ]

    portbanks = [Portbank(), Portbank(), Portbank(), Portbank()]
    crossbars = [
        Crossbar(0, crossbar0_bits, portbanks),
        Crossbar(1, crossbar1_bits, portbanks),
    ]

    for crossbar in crossbars:
        crossbar.mux(pinmux_table)

    with open(args.out, "w", encoding="utf-8") as f:
        for index, crossbar in enumerate(crossbars):
            print(f"#define CROSSBAR_{index}_CONFIG 0x{crossbar.value:08X}ULL", file=f)

        for index, portbank in enumerate(portbanks):
            print(
                f"#define PORTBANK_{index}_SKIPEN_VALUE 0x{portbank.skip_enable:04X}",
                file=f,
            )

            print(
                f"#define PORTBANK_{index}_PINS_HIGH 0x{portbank.pins_high:04X}", file=f
            )
            print(
                f"#define PORTBANK_{index}_PINS_LOW 0x{portbank.pins_low:04X}", file=f
            )

            print(
                f"#define PORTBANK_{index}_PINS_DIGITAL_INPUT 0x{portbank.pins_digital_input:04X}",
                file=f,
            )
            print(
                f"#define PORTBANK_{index}_PINS_PUSH_PULL_OUTPUT 0x{portbank.pins_push_pull_output:04X}",
                file=f,
            )
            print(
                f"#define PORTBANK_{index}_PINS_ANALOG 0x{portbank.pins_analog:04X}",
                file=f,
            )


if __name__ == "__main__":
    main()
