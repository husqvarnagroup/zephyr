# Copyright (c) 2023 GARDENA GmbH
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(sim3u)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
