# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(si4467)

FILE(GLOB app_sources src/main.c src/test_si4467_hw.c)
target_sources(app PRIVATE ${app_sources})
