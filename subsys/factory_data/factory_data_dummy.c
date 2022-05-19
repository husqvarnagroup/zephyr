/**
 * Copyright (c) 2022 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/factory_data/factory_data.h>
#include <zephyr/toolchain.h>

#include <errno.h>
#include <stddef.h>

int factory_data_init(void)
{
	return -ENOSYS;
}

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

int factory_data_load(const factory_data_load_direct_cb cb, const void *const param)
{
	return -ENOSYS;
}

ssize_t factory_data_load_one(const char *const name, uint8_t *const value, const size_t val_len)
{
	return -ENOSYS;
}
