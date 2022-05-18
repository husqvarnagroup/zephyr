# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_TRX_SI4467 si4467.c)
zephyr_sources_ifdef(CONFIG_TRX_SI4467_SHELL si4467_shell.c)
