/*
 * # Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <SI32_DEVICEID_A_Type.h>
#include <si32_device.h>

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>

#include <stddef.h>
#include <stdint.h>
#include <memory.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *const buffer, const size_t requested_len)
{
	const size_t resulting_len = MIN(requested_len, 16);

	/* According to the Silabs forums, the documentation is wrong and the UUID is actually
	 * stored from 0x00040380 to 0x0004038F.
	 *
	 * Source: https://community.silabs.com/s/question/0D51M00007xeFTdSAM/about-uuid
	 *
	 * Also, while the documentation says UUID, the randomness seems fairly limited:
	 *  SoC #1: 0xe8fe2b1929bde111b25ef74dd966642f
	 *  SoC #2: 0xa446cef72bbde111b25ef74dd966642f
	 *  SoC #3: 0xc8bc5e2545e6e711838b2bf8e78da6bc
	 */
	memcpy(buffer, (void *)0x00040380, resulting_len);

	return resulting_len;
}
