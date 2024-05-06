#!/usr/bin/env python3
#
# Copyright (c) 2024 GARDENA GmbH
#
# SPDX-License-Identifier: Apache-2.0
#
# This script generates AES test data for AES-CBC and AES-CTR with a key length of 128 bit.
#
# The output is formatted to match the clang-format settings of the Zephyr project.
#
# Dependencies:
# - PyCryptdome: `pip install pycryptodome`

import argparse
import os.path
import random
import subprocess
from argparse import ArgumentParser
from datetime import datetime

from Crypto.Cipher import AES


def write_c_file(file_name: str, content: str) -> None:
    with (open(f"{file_name}", "wt") as f):
        f.write(content)

    try:
        subprocess.run(["clang-format", "--Werror", "-i", "-style=file", f.name], check=True)
    except FileNotFoundError:
        print(f"Failed to execute clang-format on {f.name}")


def existing_dir(directory_name: str) -> str:
    if not os.path.isdir(directory_name):
        raise argparse.ArgumentTypeError(f"Not a directory: {directory_name}")
    return directory_name


def generate_cbc(args):
    plaintext = bytes([x % 255 for x in range(0, args.data_size)])
    random.seed(args.seed)
    key = random.randbytes(16)
    iv = random.randbytes(16)
    cipher = AES.new(key=key, mode=AES.MODE_CBC, iv=iv)
    ciphertext = cipher.encrypt(plaintext)
    content_source = f"""/*
     * {args.copyright}
     *
     * {args.license}
     *
     * Used seed: {args.seed}
     */

    #include "cbc_test_data.h"

    #include <zephyr/toolchain.h>

    const __aligned(4) struct cbc_test_data cbc_test_data = {{
    .plaintext = {{{', '.join(f'0x{byte:02x}' for byte in plaintext)}}},
    .ciphertext = {{{', '.join(f'0x{byte:02x}' for byte in ciphertext)}}},
    .iv = {{{', '.join(f'0x{byte:02x}' for byte in iv)}}},
    .key = {{{', '.join(f'0x{byte:02x}' for byte in key)}}},
    }};
    """.replace('    ', '')
    content_header = f"""/*
     * {args.copyright}
     *
     * {args.license}
     *
     * Used seed: {args.seed}
     */

    #pragma once

    #include <stdint.h>

    enum {{
    CBC_TEST_DATA_LEN = {args.data_size},
    }};

    struct cbc_test_data {{
    uint8_t plaintext[CBC_TEST_DATA_LEN];
    uint8_t ciphertext[CBC_TEST_DATA_LEN];
    uint8_t iv[16];
    uint8_t key[16];
    }};

    extern const struct cbc_test_data cbc_test_data;
    """.replace('    ', '')
    write_c_file(f"{args.out_dir}/cbc_test_data.h", content_header)
    write_c_file(f"{args.out_dir}/cbc_test_data.c", content_source)


def generate_ctr(args):
    plaintext = bytes([x % 255 for x in range(0, args.data_size)])
    random.seed(args.seed)
    key = random.randbytes(16)
    nonce = random.randbytes(12)
    cipher = AES.new(key=key, mode=AES.MODE_CTR, nonce=nonce, initial_value=args.ctr_init)
    ciphertext = cipher.encrypt(plaintext)
    content_source = f"""/*
     * {args.copyright}
     *
     * {args.license}
     *
     * Used seed: {args.seed}
     */

    #include "ctr_test_data.h"

    #include <zephyr/toolchain.h>

    const __aligned(4) struct ctr_test_data ctr_test_data = {{
    .plaintext = {{{', '.join(f'0x{byte:02x}' for byte in plaintext)}}},
    .ciphertext = {{{', '.join(f'0x{byte:02x}' for byte in ciphertext)}}},
    .nonce = {{{', '.join(f'0x{byte:02x}' for byte in nonce)}}},
    .key = {{{', '.join(f'0x{byte:02x}' for byte in key)}}},
    .ctr_init = {args.ctr_init},
    }};
    """.replace('    ', '')
    content_header = f"""/*
     * {args.copyright}
     *
     * {args.license}
     *
     * Used seed: {args.seed}
     */

    #pragma once

    #include <stdint.h>

    enum {{
    CTR_TEST_DATA_LEN = {args.data_size},
    }};

    struct ctr_test_data {{
    uint8_t plaintext[CTR_TEST_DATA_LEN];
    uint8_t ciphertext[CTR_TEST_DATA_LEN];
    uint8_t nonce[{len(nonce)}];
    uint8_t key[16];
    uint32_t ctr_init;
    }};

    extern const struct ctr_test_data ctr_test_data;
    """.replace('    ', '')
    write_c_file(f"{args.out_dir}/ctr_test_data.h", content_header)
    write_c_file(f"{args.out_dir}/ctr_test_data.c", content_source)


def is_aes_block_size(size: str) -> int:
    try:
        size = int(size)
    except ValueError:
        raise argparse.ArgumentTypeError(f"Not an integer: {size}")
    if size % 16 != 0:
        raise argparse.ArgumentTypeError(f"Data size not divisible by block size: {size}")
    return size


def main():
    parser = ArgumentParser(description="Generate C arrays with AES test data", allow_abbrev=False)
    parser.add_argument("--seed", help="PRNG seed", type=int, default=0)
    parser.add_argument("--data-size", help="Data size [bytes]", default=4096,
                        type=is_aes_block_size)
    parser.add_argument("--ctr-init", help="Initial CTR value (default %(default)s",
                        default=0, type=int)
    parser.add_argument("--out-dir",
                        help="Directory to place the resulting code (default: %(default)s)",
                        type=existing_dir, default="./src")
    parser.add_argument("--copyright", help="Copyright of generated sources",
                        default=f"Copyright (c) {datetime.now().year} GARDENA GmbH")
    parser.add_argument("--license", help="License of generated sources",
                        default=f"SPDX-License-Identifier: Apache-2.0")
    args = parser.parse_args()

    generate_cbc(args)
    generate_ctr(args)


if __name__ == '__main__':
    main()
