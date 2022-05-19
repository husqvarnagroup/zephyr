/*
 * Copyright (c) 2022 GARDENA GmbH
 *
 * Warning: Running this tests on real hardware wears off the  storage partition!
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/factory_data/factory_data.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/ztest.h>

static void *setup(void)
{
	zassert_ok(factory_data_erase(),
		   "Starting from scratch, wipe all pre-existing data must work");

	zassert_ok(factory_data_init(), "Initializing subsystem for all tests must work");

	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
}

ZTEST(factory_data, test_factory_data_init)
{
	/* 1st initializization done and checked by setup function */
	zassert_ok(factory_data_init(), "2nd initialization must work too");
	zassert_ok(factory_data_init(), "Actually, every initialization must work");
}

ZTEST(factory_data, test_factory_data_save_one_invalid_name)
{
	const char *const value_to_set = "value";
	uint8_t value_read_back[strlen(value_to_set)];

	zassert_equal(-ENOENT, factory_data_load_one("", value_read_back, sizeof(value_read_back)),
		      "Must not exist");

	/* Explicity set, then ensure still not existing */
	zassert_equal(-EINVAL, factory_data_save_one("", value_to_set, strlen(value_to_set)),
		      "Empty name is not allowed");
	zassert_equal(-ENOENT, factory_data_load_one("", value_read_back, sizeof(value_read_back)),
		      "Must not exist");
}

ZTEST(factory_data, test_factory_data_save_one_name_smallest)
{
	const char *const value_to_set = "1char";
	uint8_t value_read_back[strlen(value_to_set)];

	zassert_ok(factory_data_save_one("s", value_to_set, strlen(value_to_set)),
		   "Single char name");
	zassert_equal((ssize_t)strlen(value_to_set),
		      factory_data_load_one("s", value_read_back, sizeof(value_read_back)),
		      "Must exist");
	zassert_mem_equal(value_to_set, value_read_back, strlen(value_to_set),
			  "Expecting proper restore");
}

ZTEST(factory_data, test_factory_data_save_one_name_max_size)
{
	const char *const value_to_set = "longest";
	uint8_t value_read_back[strlen(value_to_set)];
	char name[CONFIG_FACTORY_DATA_NAME_LEN_MAX + 1] = {0};

	memset(name, 'M', CONFIG_FACTORY_DATA_NAME_LEN_MAX);

	zassert_ok(factory_data_save_one(name, value_to_set, strlen(value_to_set)),
		   "Max sized name must be allowed");
	zassert_equal((ssize_t)strlen(value_to_set),
		      factory_data_load_one(name, value_read_back, sizeof(value_read_back)),
		      "Must exist");
	zassert_mem_equal(value_to_set, value_read_back, strlen(value_to_set),
			  "Expecting proper restore");
}

ZTEST(factory_data, test_factory_data_save_one_name_oversize)
{
	const char *const value_to_set = "value";
	char name[CONFIG_FACTORY_DATA_NAME_LEN_MAX + 2] = {0};

	memset(name, 'N', CONFIG_FACTORY_DATA_NAME_LEN_MAX + 1);
	zassert_equal(-ENAMETOOLONG,
		      factory_data_save_one(name, value_to_set, strlen(value_to_set)),
		      "Name exceeding max name length must be rejected");
}

/* Can not actually be set from shell, but still nice to have */
ZTEST(factory_data, test_factory_data_save_one_name_with_spaces)
{
	const char *const value_to_set = "value";

	zassert_ok(factory_data_save_one("name with spaces", value_to_set, strlen(value_to_set)),
		   "name with spaces");
}

ZTEST(factory_data, test_factory_data_save_one_value_empty)
{
	const char *const value_to_set = "";
	uint8_t value_read_back[10];

	zassert_ok(factory_data_save_one("value_empty", value_to_set, strlen(value_to_set)),
		   "Simple save must work");
	zassert_equal(
		0, factory_data_load_one("value_empty", value_read_back, sizeof(value_read_back)),
		"Must exist and be of size 0");
}

ZTEST(factory_data, test_factory_data_save_one_value_regular)
{
	const char *const value_to_set = "value";
	uint8_t value_read_back[strlen(value_to_set)];

	zassert_ok(factory_data_save_one("value_regular", value_to_set, strlen(value_to_set)),
		   "Simple save must work");
	zassert_equal(
		(ssize_t)strlen(value_to_set),
		factory_data_load_one("value_regular", value_read_back, sizeof(value_read_back)),
		"Must exist");
	zassert_mem_equal(value_to_set, value_read_back, strlen(value_to_set),
			  "Expecting proper restore");
}

ZTEST(factory_data, test_factory_data_save_one_value_max_length)
{
	char value_to_set[CONFIG_FACTORY_DATA_VALUE_LEN_MAX];
	uint8_t value_read_back[CONFIG_FACTORY_DATA_VALUE_LEN_MAX + 10];

	memset(value_to_set, 'X', sizeof(value_to_set));

	zassert_ok(factory_data_save_one("value_huge", value_to_set, sizeof(value_to_set)),
		   "Huge values must be persistable");
	zassert_equal((ssize_t)sizeof(value_to_set),
		      factory_data_load_one("value_huge", value_read_back, sizeof(value_read_back)),
		      "Must exist");
	zassert_mem_equal(value_to_set, value_read_back, sizeof(value_to_set),
			  "Expecting proper restore");
}

ZTEST(factory_data, test_factory_data_save_one_value_oversize)
{
	char value_to_set[CONFIG_FACTORY_DATA_VALUE_LEN_MAX + 1];
	uint8_t value_read_back[sizeof(value_to_set)];

	memset(value_to_set, 0xAA, sizeof(value_to_set));

	zassert_equal(-EFBIG,
		      factory_data_save_one("value_too_big", value_to_set, sizeof(value_to_set)),
		      "Values exceeding max size must be rejected");
	zassert_equal(
		-ENOENT,
		factory_data_load_one("value_too_big", value_read_back, sizeof(value_read_back)),
		"Must not exist");
}

ZTEST(factory_data, test_factory_data_save_one_twice)
{
	const char *const value_to_set = "value";

	zassert_ok(factory_data_save_one("unique_only_once", value_to_set, strlen(value_to_set)),
		   "First write allowed");
	zassert_equal(-EEXIST,
		      factory_data_save_one("unique_only_once", value_to_set, strlen(value_to_set)),
		      "2nd write to same variable not allowed");
}

ZTEST(factory_data, test_factory_data_erase)
{
	const char *const value_to_set = "value";
	uint8_t value_read_back[strlen(value_to_set)];

	zassert_ok(factory_data_save_one("erase-me", value_to_set, strlen(value_to_set)),
		   "Saving must work");
	zassert_equal((ssize_t)strlen(value_to_set),
		      factory_data_load_one("erase-me", value_read_back, sizeof(value_read_back)),
		      "Read back to prove proper storing");
	zassert_ok(factory_data_erase(), "Erase must succeed");
	zassert_equal(-ENOENT,
		      factory_data_load_one("erase-me", value_read_back, sizeof(value_read_back)),
		      "Entry must be gone");
}

/* Erase without using the factory data subsystem! -> separate test */
ZTEST(factory_data, test_erase_flash)
{
	int ret;
	const struct flash_area *fap;

	ret = flash_area_open(FACTORY_DATA_FLASH_PARTITION, &fap);
	zassert_ok(ret, "Flash area open must work");

	ret = flash_area_erase(fap, 0, fap->fa_size);
	zassert_ok(ret, "Flash area erase must work");
}

ZTEST_SUITE(factory_data, NULL, setup, before, NULL, NULL);
