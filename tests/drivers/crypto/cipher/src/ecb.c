/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Similar to the crypto sample, but testing more features of the driver.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(crypto, LOG_LEVEL_DBG);

#include "common.h"

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

enum {
	AES_128_KEY_SIZE_BYTES = 16,
	AES_BLOCK_SIZE_BYTES = 16,
};

struct crypto_ecb_fixture {
	const struct device *dev;

	struct cipher_ctx ctx;
	bool ctx_free_in_teardown; /* Set this to true when test not freeing the session itself */

	/* Test data */
	struct {
		const uint8_t plaintext[AES_BLOCK_SIZE_BYTES];
		const uint8_t ciphertext[AES_BLOCK_SIZE_BYTES];
		const uint8_t key[AES_128_KEY_SIZE_BYTES];
	};
};

void *crypto_ecb_setup(void)
{
	/* from FIPS-197 test vectors */
	static struct crypto_ecb_fixture fixture = {
		.plaintext = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA,
			      0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
		.ciphertext = {0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30, 0xD8, 0xCD, 0xB7,
			       0x80, 0x70, 0xB4, 0xC5, 0x5A},
		.key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
			0x0C, 0x0D, 0x0E, 0x0F},
	};

#if defined(CRYPTO_DRV_NAME)
	fixture.dev = device_get_binding(CRYPTO_DRV_NAME);
#else
	fixture.dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
#endif

	zassert(device_is_ready(fixture.dev), "Crypto device must be ready");

	LOG_DBG("device: %s", fixture.dev->name);

	return &fixture;
}

void crypto_ecb_before(void *fixture)
{
	struct crypto_ecb_fixture *f = (struct crypto_ecb_fixture *)fixture;

	zassert_false(f->ctx_free_in_teardown, "Last test did not free crypto sesion");

	/* Reset context to sensible defaults */
	f->ctx = (struct cipher_ctx){
		.keylen = sizeof(f->key),
		.key.bit_stream = f->key,
		.flags = CAP_RAW_KEY,
	};
}

void crypto_ecb_after(void *fixture)
{
	struct crypto_ecb_fixture *f = (struct crypto_ecb_fixture *)fixture;

	if (f->ctx_free_in_teardown) {
		zassert_ok(cipher_free_session(f->dev, &f->ctx),
			   "Clean up session (must always happen!)");
		f->ctx_free_in_teardown = false;
	}
}

ZTEST_F(crypto_ecb, test_encrypt_separate_io_bufs)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	uint8_t in_buf[AES_BLOCK_SIZE_BYTES];
	uint8_t out_buf[AES_BLOCK_SIZE_BYTES];
	struct cipher_pkt pkt = {
		.in_buf = in_buf,
		.in_len = AES_BLOCK_SIZE_BYTES,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	memcpy(in_buf, fixture->plaintext, sizeof(in_buf));
	zassert_ok(cipher_block_op(&fixture->ctx, &pkt), "Encryption must succeed");
	zassert_mem_equal(out_buf, fixture->ciphertext, AES_BLOCK_SIZE_BYTES);
}

ZTEST_F(crypto_ecb, test_decrypt_separate_io_bufs)
{
	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_DECRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	uint8_t in_buf[AES_BLOCK_SIZE_BYTES];
	uint8_t out_buf[AES_BLOCK_SIZE_BYTES];
	struct cipher_pkt pkt = {
		.in_buf = in_buf,
		.in_len = AES_BLOCK_SIZE_BYTES,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	memcpy(in_buf, fixture->ciphertext, sizeof(in_buf));
	zassert_ok(cipher_block_op(&fixture->ctx, &pkt), "Decryption must succeed");
	zassert_mem_equal(out_buf, fixture->plaintext, AES_BLOCK_SIZE_BYTES);
}

ZTEST_F(crypto_ecb, test_encrypt_inplace_ops)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_INPLACE_OPS) == 0U) {
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_INPLACE_OPS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	uint8_t buf[AES_BLOCK_SIZE_BYTES];
	struct cipher_pkt pkt = {
		.in_buf = buf,
		.in_len = AES_BLOCK_SIZE_BYTES,
		.out_buf = NULL,
	};
	memcpy(buf, fixture->plaintext, sizeof(buf));
	zassert_ok(cipher_block_op(&fixture->ctx, &pkt), "Encryption must succeed");
	zassert_mem_equal(buf, fixture->ciphertext, AES_BLOCK_SIZE_BYTES);
}

ZTEST_F(crypto_ecb, test_decrypt_inplace_ops)
{
	const int flags = crypto_query_hwcaps(fixture->dev);

	if ((flags & CAP_INPLACE_OPS) == 0U) {
		ztest_test_skip();
	}

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_INPLACE_OPS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_DECRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	uint8_t buf[AES_BLOCK_SIZE_BYTES];
	struct cipher_pkt pkt = {
		.in_buf = buf,
		.in_len = AES_BLOCK_SIZE_BYTES,
		.out_buf = NULL,
	};
	memcpy(buf, fixture->ciphertext, sizeof(buf));
	zassert_ok(cipher_block_op(&fixture->ctx, &pkt), "Decryption must succeed");
	zassert_mem_equal(buf, fixture->plaintext, AES_BLOCK_SIZE_BYTES);
}

ZTEST_F(crypto_ecb, test_encrypt_zero_sized_data)
{
	int res;
	uint8_t in_buf[AES_BLOCK_SIZE_BYTES] = {};
	uint8_t out_buf[AES_BLOCK_SIZE_BYTES] = {};

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = in_buf,
		.in_len = 0,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	res = cipher_block_op(&fixture->ctx, &pkt);

	zassert_true(res == 0 || res == -EINVAL,
		     "Refusing or ignoring is fine. Just do not crash!");
}

ZTEST_F(crypto_ecb, test_encrypt_partial_buf)
{
	uint8_t in_buf[AES_BLOCK_SIZE_BYTES] = {};
	uint8_t out_buf[AES_BLOCK_SIZE_BYTES] = {};

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = in_buf,
		.in_len = 15,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	zassert_not_ok(cipher_block_op(&fixture->ctx, &pkt),
		       "Refusing to encrypt partial AES block");
}

ZTEST_F(crypto_ecb, test_encrypt_multiple_blocks)
{
	uint8_t in_buf[2 * AES_BLOCK_SIZE_BYTES] = {};
	uint8_t out_buf[2 * AES_BLOCK_SIZE_BYTES] = {};

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	struct cipher_pkt pkt = {
		.in_buf = in_buf,
		.in_len = 32,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	zassert_not_ok(cipher_block_op(&fixture->ctx, &pkt),
		       "Refusing to encrypt multiple AES blocks");
}

ZTEST_F(crypto_ecb, test_encrypt_alignment_1)
{
	enum {
		ALIGMENT_OFFSET = 1,
	};
	int ret;

	fixture->ctx.flags |= CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	zassert_ok(cipher_begin_session(fixture->dev, &fixture->ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "Session initialization must succeed");
	fixture->ctx_free_in_teardown = true;

	__aligned(AES_BLOCK_SIZE_BYTES) uint8_t in_buf[AES_BLOCK_SIZE_BYTES + ALIGMENT_OFFSET];
	__aligned(AES_BLOCK_SIZE_BYTES) uint8_t out_buf[AES_BLOCK_SIZE_BYTES + ALIGMENT_OFFSET];
	struct cipher_pkt pkt = {
		.in_buf = in_buf + ALIGMENT_OFFSET,
		.in_len = AES_BLOCK_SIZE_BYTES,
		.out_buf_max = sizeof(out_buf),
		.out_buf = out_buf,
	};
	memcpy(in_buf + ALIGMENT_OFFSET, fixture->plaintext, sizeof(in_buf) - ALIGMENT_OFFSET);
	ret = cipher_block_op(&fixture->ctx, &pkt); /* might or might now succeed */
	if (ret) {
		ztest_test_skip();
	}

	/* If signaling no error, then ensure that the result is actually correct */
	zassert_mem_equal(out_buf, fixture->ciphertext, AES_BLOCK_SIZE_BYTES);
}

ZTEST_SUITE(crypto_ecb, NULL, crypto_ecb_setup, crypto_ecb_before, crypto_ecb_after, NULL);
