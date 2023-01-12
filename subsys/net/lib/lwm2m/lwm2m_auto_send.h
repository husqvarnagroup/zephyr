/*
 * Copyright (c) 2022 Garden GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LWM2M_AUTO_SEND_H
#define LWM2M_AUTO_SEND_H

#include "lwm2m_observation.h"

/**
 * @brief Add a path as hint for each auto send.
 *
 * Use this if you want to always send certain object instances with all its containing resources
 * or a multiple instance resource with all of its instances.
 *
 * @see lwm2m_engine_auto_send_add_next_path_hint
 *
 * @param[in] path
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_add_static_path_hint(const struct lwm2m_obj_path *path);

/**
 * @see lwm2m_engine_auto_send_add_static_path_hint
 *
 * @param path_str
 * @return
 */
int lwm2m_engine_auto_send_add_static_path_hint_str(const char *path_str);

/**
 * @brief Add a path as hint for the next auto send.
 *
 * If a resource is modified below that path and will be automatically sent,
 * then this path will be used instead. This allows to send resources with the next auto
 * send that were not modified but are desired to be transmitted together with the modified
 * resources. It also allows to reduce the number of paths that need to be processed internally
 * (which are limited to CONFIG_LWM2M_COMPOSITE_PATH_LIST_SIZE) when modifying (almost) all
 * resources of one or multiple objects.
 *
 * @param[in] path
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_add_next_path_hint(const struct lwm2m_obj_path *path);

/**
 * @brief Add a path to be ignored for auto send.
 *
 * Modification of resources at (or below) the provided path will never trigger an
 * automatic send after they have been modified. The resources at the path could still be
 * sent if a path above the provided path was modified.
 *
 * @param[in] path
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_ignore_path(const struct lwm2m_obj_path *path);

/**
 * @brief Enable auto send for modified resources modified after certain point in time
 *
 * @see k_uptime_get()
 *
 * @param after Uptime in milliseconds after which automatic sends should start
 * @param cooldown Number of milliseconds to wait between automatic sends
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_enable(int64_t after, uint32_t cooldown);

/**
 * @brief Disable auto send for modified resources modified after certain point in time
 *
 * @see k_uptime_get()
 *
 * @param after Uptime in milliseconds after which automatic sends should start
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_disable(int64_t after);

void check_automatic_lwm2m_sends(struct lwm2m_ctx *ctx, int64_t timestamp);

#endif /* LWM2M_AUTO_SEND_H */
