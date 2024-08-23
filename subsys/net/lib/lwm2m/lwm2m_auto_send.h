/*
 * Copyright (c) 2022 Garden GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LWM2M_AUTO_SEND_H
#define LWM2M_AUTO_SEND_H

#include "lwm2m_observation.h"

/**
 * Callback when all syncing ipso objects are sucessfully sent to the gateway.
 *
 * @param success true if the syncing was successful, false if a timeout occurred
 *
 * @note This is a weak function. You can overwrite the default implementation.
 */
__weak void lwm2m_client_device_synced(bool success);

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
 * @brief Send an object instance with all its contained resources at once
 *
 * @param[in] path
 * @return 0 for success or negative in case of error
 */
int lwm2m_engine_auto_send_send_obj_inst(const struct lwm2m_obj_path *path);

/**
 * @brief Set all objs dirty to be resent
 */
void lwm2m_engine_auto_send_send_all_objs(void);

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
 * @brief Register syncing objects to be sent first during the inclusion/connection process.
 *
 * Call this function on inclusion/connection to register ipso objects.
 * Auto send initially only sends these objects until they are all successfully
 * synchronised or a timeout occurs before the remaining objects are sent.
 * Overwrite \lwm2m_client_device_synced to get a callback when all required
 * objects are synced.
 *
 * @param path_list array with paths
 * @param count number of paths to register as syncing objects
 * @param timeout max time in ms to wait until switching to normal mode
 *                and sending all objects
 */
void lwm2m_engine_auto_send_set_syncing_objects(const struct lwm2m_obj_path path_list[],
						uint8_t count, int64_t timeout);

/**
 * @brief Enable or disable auto send for modified resources
 *
 * @param enable true to enable auto send
 *               false to disable auto send
 */
void lwm2m_engine_auto_send_set(bool enable);

/**
 * @brief Run auto send which checks all objects and sends changed resources
 *
 * @param ctx       lwm2m context
 * @param timestamp current timestamp
 */
void lwm2m_enginge_auto_send_run(struct lwm2m_ctx *ctx, int64_t timestamp);

#endif /* LWM2M_AUTO_SEND_H */
