/*
 * Copyright (c) 2022 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/factory_data/factory_data.h>
#include <zephyr/toolchain.h>
#include <zephyr/ztest.h>

static int factory_data_pre_init_errors_data_load_dummy(const char *name, const uint8_t *value,
							size_t len, const void *param)
{
	ARG_UNUSED(name);
	ARG_UNUSED(value);
	ARG_UNUSED(len);
	ARG_UNUSED(param);

	return 0;
};

ZTEST(factory_data_uninitialized, test_factory_data_save_one)
{
	/* This should actually be a separate test, in a separate project */
	const char *const value = "value";
	uint8_t buf[16];

	zassert_equal(-ECANCELED, factory_data_save_one("name", value, strlen(value)),
		      "Failing because not initialized");
	zassert_equal(-ECANCELED,
		      factory_data_load(factory_data_pre_init_errors_data_load_dummy, NULL),
		      "Failing because not initialized");
	zassert_equal(-ECANCELED, factory_data_load_one("name", buf, sizeof(buf)),
		      "Failing because not initialized");
}

ZTEST(factory_data_uninitialized, test_factory_data_load)
{
	zassert_equal(-ECANCELED,
		      factory_data_load(factory_data_pre_init_errors_data_load_dummy, NULL),
		      "Failing because not initialized");
}

ZTEST(factory_data_uninitialized, test_factory_data_load_one)
{
	uint8_t buf[16];

	zassert_equal(-ECANCELED, factory_data_load_one("name", buf, sizeof(buf)),
		      "Failing because not initialized");
}

ZTEST(factory_data_uninitialized, test_factory_data_erase)
{
	zassert_ok(factory_data_erase(), "Must work even when not initialized");
}

ZTEST_SUITE(factory_data_uninitialized, NULL, NULL, NULL, NULL, NULL);
