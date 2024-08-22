/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Similar to the crypto sample, but with a larger data sets and a broader feature usage.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(crypto, LOG_LEVEL_DBG);

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

#include "cbc_test_data.h"
#include "common.h"

struct crypto_cbc_fixture {
	const struct device *dev;

	uint8_t *const scratchpad;
	const size_t scratchpad_len; /* in bytes */

	struct cipher_ctx ctx;
	bool ctx_free_in_teardown; /* Set this to true when test not freeing the session itself */

	/* Test data */
	struct {
		const uint8_t *const plaintext;
		const size_t plaintext_len; /* in bytes */
		const uint8_t *const ciphertext;
		const size_t ciphertext_len; /* in bytes */
		const uint8_t *const key;
		const uint8_t key_len;

		/* Not const because of crypto API asking for non-const pointers */
		uint8_t iv[sizeof(cbc_test_data.iv)];
	};
};

void *crypto_cbc_setup(void)
{
	static uint8_t scratchpad[16 /* for when CAP_NO_IV_PREFIX is not used */ +
				  sizeof(cbc_test_data.plaintext)];
	static struct crypto_cbc_fixture fixture = {
		.plaintext = cbc_test_data.plaintext,
		.plaintext_len = sizeof(cbc_test_data.plaintext),
		.ciphertext = cbc_test_data.ciphertext,
		.ciphertext_len = sizeof(cbc_test_data.ciphertext),
		.key = cbc_test_data.key,
		.key_len = sizeof(cbc_test_data.key),
		.scratchpad = scratchpad,
		.scratchpad_len = sizeof(scratchpad),
	};

#if defined(CRYPTO_DRV_NAME)
	fixture.dev = device_get_binding(CRYPTO_DRV_NAME);
#else
	fixture.dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
#endif

	zassert_true(device_is_ready(fixture.dev), "Crypto device must be ready");

	LOG_DBG("device: %s", fixture.dev->name);

	return &fixture;
}

void crypto_cbc_before(void *fixture)
{
	struct crypto_cbc_fixture *f = (struct crypto_cbc_fixture *)fixture;

	zassert_false(f->ctx_free_in_teardown, "Last test did not free crypto sesion");

	/* Members not const, might have been modified */
	memcpy(f->iv, cbc_test_data.iv, sizeof(cbc_test_data.iv));
	memset(f->scratchpad, 0, f->scratchpad_len);

	/* Reset context to sensible defaults */
	f->ctx = (struct cipher_ctx){
		.keylen = f->key_len,
		.key.bit_stream = f->key,
		.flags = CAP_RAW_KEY,
	};
}

void crypto_cbc_after(void *fixture)
{
	struct crypto_cbc_fixture *f = (struct crypto_cbc_fixture *)fixture;

	if (f->ctx_free_in_teardown) {
		zassert_ok(cipher_free_session(f->dev, &f->ctx),
			   "Clean up session (must always happen!)");
		f->ctx_free_in_teardown = false;
	}
}

ZTEST_F(crypto_cbc, test_encrypt_separate_io_bufs)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)fixture->plaintext, /* Out in dedicated memory, won't modify */
		.in_len = fixture->plaintext_len,
		.out_buf_max = fixture->scratchpad_len,
		.out_buf = fixture->scratchpad,
	};

	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_mem_equal(fixture->scratchpad + 16, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_F(crypto_cbc, test_encrypt_inplace_ops)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_INPLACE_OPS) == 0U) {
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_INPLACE_OPS | CAP_NO_IV_PREFIX;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = fixture->scratchpad,
		.in_len = fixture->plaintext_len,
		.out_buf = NULL,
	};

	memcpy(fixture->scratchpad, fixture->plaintext, fixture->scratchpad_len);
	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_F(crypto_cbc, test_decrypt_inplace_ops)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_INPLACE_OPS) == 0U) {
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_INPLACE_OPS | CAP_NO_IV_PREFIX;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_DECRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = fixture->scratchpad,
		.in_len = fixture->plaintext_len,
		.out_buf = NULL,
	};

	memcpy(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_mem_equal(fixture->scratchpad, fixture->plaintext, fixture->plaintext_len);
}

ZTEST_F(crypto_cbc, test_encrypt_partial_buf)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)fixture->plaintext, /* Out in dedicated memory, won't modify */
		.in_len = fixture->plaintext_len - 1,
		.out_buf_max = fixture->scratchpad_len,
		.out_buf = fixture->scratchpad,
	};

	zassert_not_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must fail");
}

ZTEST_F(crypto_cbc, test_decrypt_partial_buf)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_DECRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf =
			(uint8_t *)fixture->ciphertext, /* Out in dedicated memory, won't modify */
		.in_len = fixture->ciphertext_len - 1,
		.out_buf_max = fixture->scratchpad_len,
		.out_buf = fixture->scratchpad,
	};

	zassert_not_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must fail");
}

ZTEST_F(crypto_cbc, test_fragmented_calculation)
{
	if(IS_ENABLED(CONFIG_CRYPTO_TINYCRYPT_SHIM)) {
		/* TC prefixes IV. Supporting this would make this test harder to read */
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS | CAP_NO_IV_PREFIX;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CBC, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)fixture->plaintext, /* Out in dedicated memory, won't modify */
		.in_len = 1 * 16,                        /* Just one block */
		.out_buf = fixture->scratchpad,
		.out_buf_max = fixture->scratchpad_len,
	};

	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, 1 * 16, "Just one block done");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, 1 * 16);

	pkt = (struct cipher_pkt){
		.in_buf = (uint8_t *)fixture->plaintext +
			  1 * 16, /* Out in dedicated memory, won't modify */
		.in_len = 2 * 16, /* Do two blocks */
		.out_buf = fixture->scratchpad + 1 * 16,
		.out_buf_max = 2 * 16,
	};

	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, 2 * 16, "Two blocks done");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, 3 * 16);

	pkt = (struct cipher_pkt){
		.in_buf = (uint8_t *)fixture->plaintext +
			  3 * 16, /* Out in dedicated memory, won't modify */
		.in_len = fixture->plaintext_len - 3 * 16, /* Do remaining blocks */
		.out_buf = fixture->scratchpad + 3 * 16,
		.out_buf_max = fixture->scratchpad_len - 3 * 16,
	};

	zassert_ok(cipher_cbc_op(&fixture->ctx, &pkt, fixture->iv), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, fixture->plaintext_len - 3 * 16,
		      "Remaining blocks done");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_SUITE(crypto_cbc, NULL, crypto_cbc_setup, crypto_cbc_before, crypto_cbc_after, NULL);
