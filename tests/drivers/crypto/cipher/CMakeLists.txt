# Copyright (c) 2024 GARDENA GmbH
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(crypto-cipher)

target_sources(app PRIVATE
	src/cbc_test_data.c
	src/cbc.c
	src/ctr_test_data.c
	src/ctr.c
	src/ecb.c
	src/main.c
)
