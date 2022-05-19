/*
 * Copyright (c) 2022 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "factory_data_common.h"

#include <zephyr/toolchain.h>

#include <errno.h>
#include <stddef.h>
#include <string.h>

/** Len of record without write-block-size alignment. Same logic as in settings_line_len_calc(). */
int factory_data_line_len_calc(const char *const name, const size_t val_len)
{
	/* <name>\0<value> */
	return strlen(name) + 1 + val_len;
}

#if !defined(CONFIG_FACTORY_DATA_WRITE)
int factory_data_save_one(const char *const name, const void *const value, const size_t val_len)
{
	ARG_UNUSED(name);
	ARG_UNUSED(value);
	ARG_UNUSED(val_len);

	return -ENOSYS;
}

int factory_data_erase(void)
{
	return -ENOSYS;
}

#endif /* CONFIG_FACTORY_DATA_WRITE */
