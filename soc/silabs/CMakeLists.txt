# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2017 Christian Taedcke

if(CONFIG_SOC_FAMILY_SILABS_S0 OR CONFIG_SOC_FAMILY_SILABS_S1 OR CONFIG_SOC_FAMILY_SILABS_S2)
  add_subdirectory(common_gecko)
endif()

zephyr_include_directories(${SOC_FAMILY}/${SOC_SERIES})
