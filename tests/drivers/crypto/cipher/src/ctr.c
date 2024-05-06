/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Similar to the crypto sample, but with much larger data sets and a broader features usage.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(crypto, LOG_LEVEL_DBG);

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

#include "common.h"
#include "ctr_test_data.h"

struct crypto_ctr_fixture {
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
		uint8_t nonce[sizeof(ctr_test_data.nonce)];
	};
};

static void *crypto_ctr_setup(void)
{
	static uint8_t scratchpad[16 /* for when CAP_NO_IV_PREFIX is not used */ +
				  sizeof(ctr_test_data.plaintext)];
	static struct crypto_ctr_fixture fixture = {
		.plaintext = ctr_test_data.plaintext,
		.plaintext_len = sizeof(ctr_test_data.plaintext),
		.ciphertext = ctr_test_data.ciphertext,
		.ciphertext_len = sizeof(ctr_test_data.ciphertext),
		.key = ctr_test_data.key,
		.key_len = sizeof(ctr_test_data.key),
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

static void crypto_ctr_before(void *fixture)
{
	struct crypto_ctr_fixture *f = (struct crypto_ctr_fixture *)fixture;

	zassert_false(f->ctx_free_in_teardown, "Last test did not free crypto sesion");

	/* Members not const, might have been modified */
	memcpy(f->nonce, ctr_test_data.nonce, sizeof(ctr_test_data.nonce));
	memset(f->scratchpad, 0, f->scratchpad_len);

	/* Reset context to sensible defaults */
	f->ctx = (struct cipher_ctx){
		.key.bit_stream = f->key,
		.keylen = f->key_len,
		.mode_params.ctr_info.ctr_len = 128 - sizeof(f->nonce) * 8,
		.flags = CAP_RAW_KEY,
	};
}

static void crypto_ctr_after(void *fixture)
{
	struct crypto_ctr_fixture *f = (struct crypto_ctr_fixture *)fixture;

	if (f->ctx_free_in_teardown) {
		zassert_ok(cipher_free_session(f->dev, &f->ctx),
			   "Clean up session (must always succeed!)");
		f->ctx_free_in_teardown = false;
	}
}

ZTEST_F(crypto_ctr, test_separate_io_bufs)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CTR, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)fixture->plaintext, /* out in different memory, won't modify */
		.in_len = fixture->plaintext_len,
		.out_buf = fixture->scratchpad,
		.out_buf_max = fixture->plaintext_len, /* TC insists on 'len in' = 'len out' */
	};

	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, fixture->nonce), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, fixture->plaintext_len, "Output has same size as input");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_F(crypto_ctr, test_inplace_ops)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_INPLACE_OPS) == 0U) {
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_INPLACE_OPS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CTR, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = fixture->scratchpad,
		.in_len = fixture->plaintext_len,
		.out_buf = NULL,
	};

	memcpy(fixture->scratchpad, fixture->plaintext, fixture->scratchpad_len);

	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, fixture->nonce), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, fixture->plaintext_len, "Output has same size as input");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_F(crypto_ctr, test_custom_initial_counter_value)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_AES_CTR_CUSTOM_COUNTER_INIT) == 0U) {
		ztest_test_skip();
	}

	/* Based on output of `./generate-test-data.py --data-size 64 --ctr-init 1` */
	struct ctr_test_data {
		uint8_t plaintext[64];
		uint8_t ciphertext[64];
		uint8_t nonce[12];
		uint8_t key[16];
		uint32_t ctr_init;
	};
	static const __aligned(4) struct ctr_test_data test_data = {
		.plaintext = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
			      0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
			      0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
			      0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,
			      0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
			      0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f},
		.ciphertext = {0xb8, 0xda, 0x8b, 0x0a, 0x47, 0xbb, 0x5c, 0x73, 0x1a, 0xd7, 0x0d,
			       0x34, 0x57, 0x9d, 0x08, 0x0b, 0xa0, 0x4e, 0x5e, 0x5e, 0x7f, 0xd7,
			       0x78, 0x13, 0x5f, 0xbe, 0xd4, 0x87, 0x07, 0xaf, 0x5b, 0xbe, 0xc4,
			       0xe4, 0x6e, 0xab, 0xd2, 0x64, 0x22, 0x46, 0xf6, 0x01, 0xad, 0x51,
			       0x5a, 0x92, 0x79, 0xea, 0x16, 0x47, 0x6a, 0x76, 0x4c, 0x73, 0x24,
			       0x4d, 0x44, 0xbd, 0xe0, 0x01, 0x03, 0x57, 0x29, 0xa1},
		.nonce = {0x55, 0x94, 0xaa, 0x6b, 0x34, 0x2f, 0x5d, 0x0a, 0x3a, 0x5e, 0x48, 0x42},
		.key = {0xcd, 0x07, 0x2c, 0xd8, 0xbe, 0x6f, 0x9f, 0x62, 0xac, 0x4c, 0x09, 0xc2,
			0x82, 0x06, 0xe7, 0xe3},
		.ctr_init = 1,
	};
	uint8_t nonce_rw[sizeof(test_data.nonce)];

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	zassert_equal(1, test_data.ctr_init, "For this test, the test data counter must be 1");
	fixture->ctx.mode_params.ctr_info.ctr_initial_value = test_data.ctr_init;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CTR, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf =
			(uint8_t *)test_data.plaintext, /* out in different memory, won't modify */
		.in_len = sizeof(test_data.plaintext),
		.out_buf = fixture->scratchpad,
		.out_buf_max = fixture->scratchpad_len,
	};

	memcpy(nonce_rw, test_data.nonce, sizeof(nonce_rw));
	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, nonce_rw), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, sizeof(test_data.plaintext),
		      "Output has same size as input");
	zassert_mem_equal(fixture->scratchpad, test_data.ciphertext, sizeof(test_data.ciphertext));
}

ZTEST_F(crypto_ctr, test_fragmented_calculation)
{
	if(IS_ENABLED(CONFIG_CRYPTO_TINYCRYPT_SHIM)) {
		/* TC prefixes IV. Supporting this would make this test harder to read */
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CTR, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)fixture->plaintext, /* out in different memory, won't modify */
		.in_len = 1 * 16,                        /* Just one block */
		.out_buf = fixture->scratchpad,
		.out_buf_max = fixture->scratchpad_len,
	};

	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, fixture->nonce), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, 16, "Just one block done");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, 16);

	pkt = (struct cipher_pkt){
		.in_buf = (uint8_t *)fixture->plaintext +
			  1 * 16, /* out in different memory, won't modify */
		.in_len = 2 * 16, /* Do two blocks */
		.out_buf = fixture->scratchpad + 16,
		.out_buf_max = 2 * 16,
	};

	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, fixture->nonce), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, 2 * 16, "Two blocks done");
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, 3 * 16);

	pkt = (struct cipher_pkt){
		.in_buf = (uint8_t *)fixture->plaintext +
			  3 * 16, /* out in different memory, won't modify */
		.in_len = fixture->plaintext_len - 3 * 16, /* Do remaining blocks */
		.out_buf = fixture->scratchpad + 3 * 16,
		.out_buf_max = fixture->scratchpad_len - 3 * 16,
	};

	zassert_ok(cipher_ctr_op(&fixture->ctx, &pkt, fixture->nonce), "Encryption must succeed");
	zassert_equal((size_t)pkt.out_len, fixture->plaintext_len - 3 * 16,
		      "Remaining blocks done");

	/* End result must be the same as if done in one go (tested in another test) */
	zassert_mem_equal(fixture->scratchpad, fixture->ciphertext, fixture->ciphertext_len);
}

ZTEST_SUITE(crypto_ctr, NULL, crypto_ctr_setup, crypto_ctr_before, crypto_ctr_after, NULL);
