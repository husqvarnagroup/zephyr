#define LOG_MODULE_NAME net_lwm2m_engine_auto_send
#define LOG_LEVEL       CONFIG_LWM2M_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "lwm2m_auto_send.h"
#include "lwm2m_object.h"
#include "lwm2m_observation.h"
#include "lwm2m_registry.h"
#include "lwm2m_util.h"

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEND_RESPONSE_TIMEOUT  CONFIG_LWM2M_ENGINE_AUTO_SEND_RESPONSE_TIMEOUT
#define MAX_MODIFIED_RESOURCE  CONFIG_LWM2M_ENGINE_AUTO_SEND_MODIFIED_RESOURCES_MAX
#define MAX_STATIC_SEND_HINTS  CONFIG_LWM2M_ENGINE_AUTO_SEND_STATIC_PATH_HINTS_MAX
#define MAX_SEND_PATHS_OVERALL (MAX_MODIFIED_RESOURCE + MAX_STATIC_SEND_HINTS)

#ifdef CONFIG_LWM2M_LOG_LEVEL_DBG
#define LWM2M_AUTO_SEND_DEBUG 1
#endif

struct lwm2m_auto_send_object_inst_ref {
	/* instance list */
	sys_snode_t node;

	struct lwm2m_engine_obj_inst *obj_inst;
};

typedef bool (*for_each_res_inst_callback_t)(const struct lwm2m_engine_res *obj_res,
					     struct lwm2m_engine_res_inst *obj_res_inst,
					     void *callback_ctx);

static bool auto_send_enabled;
static int64_t last_auto_send_check = -1;
static bool auto_send_recover = false;
#if CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX > 0
static bool ignore_max_frequency_for_next_check = false;
#endif

static int static_send_hints_count;
static struct lwm2m_obj_path static_send_hints[MAX_STATIC_SEND_HINTS];

static struct lwm2m_obj_path_list next_lwm2m_send_path_list_buf[MAX_SEND_PATHS_OVERALL];
static struct lwm2m_obj_path_list *entry, *tmp;
static sys_slist_t next_lwm2m_send_path_list;
static sys_slist_t next_lwm2m_send_path_free_list;

static struct lwm2m_obj_path next_lwm2m_send_paths[MAX_SEND_PATHS_OVERALL];
static struct lwm2m_obj_path modified_paths[MAX_MODIFIED_RESOURCE];
static struct lwm2m_auto_send_object_inst_ref partial_update_list_buf[MAX_MODIFIED_RESOURCE];
static sys_slist_t partial_update_list;

static time_t last_sent_time = -1;

_Static_assert((time_t)(-1) < 0, "last_sent_time must be a signed type!");

static void for_each_res_inst(const struct lwm2m_engine_obj_inst *obj_inst,
			      for_each_res_inst_callback_t callback, void *callback_ctx)
{
	struct lwm2m_engine_res *obj_res;
	struct lwm2m_engine_res_inst *obj_res_inst;
	struct lwm2m_engine_obj_field *obj_field;

	for (uint_fast16_t res_idx = 0; res_idx < obj_inst->resource_count; res_idx++) {
		obj_res = &obj_inst->resources[res_idx];
		obj_field = lwm2m_get_engine_obj_field(obj_inst->obj, obj_res->res_id);
		if (!LWM2M_HAS_PERM(obj_field, LWM2M_PERM_R)) {
			continue;
		}
		for (uint_fast8_t i = 0; i < obj_res->res_inst_count; i++) {
			obj_res_inst = &obj_res->res_instances[i];
			if (obj_res_inst->res_inst_id == RES_INSTANCE_NOT_CREATED) {
				continue;
			}

			if (callback(obj_res, obj_res_inst, callback_ctx)) {
				break;
			}
		}
	}
}

static bool unmark_resources_cb(const struct lwm2m_engine_res *obj_res,
				struct lwm2m_engine_res_inst *obj_res_inst, void *callback_data)
{
	obj_res_inst->dirty = false;
	obj_res_inst->sending = false;

	return false;
}

static bool mark_resources_as_dirty_cb(const struct lwm2m_engine_res *obj_res,
				       struct lwm2m_engine_res_inst *obj_res_inst,
				       void *callback_data)
{
	obj_res_inst->dirty = true;
	return false;
}

static bool mark_resources_as_sending_cb(const struct lwm2m_engine_res *obj_res,
					 struct lwm2m_engine_res_inst *obj_res_inst,
					 void *callback_data)
{
	obj_res_inst->sending = true;
	obj_res_inst->dirty = false;
	return false;
}

static bool mark_sending_resources_as_dirty_cb(const struct lwm2m_engine_res *obj_res,
					       struct lwm2m_engine_res_inst *obj_res_inst,
					       void *callback_data)
{
	if (obj_res_inst->sending) {
		obj_res_inst->sending = false;
		obj_res_inst->dirty = true;
	}
	return false;
}

static bool mark_resources_as_sent_cb(const struct lwm2m_engine_res *obj_res,
				      struct lwm2m_engine_res_inst *obj_res_inst,
				      void *callback_data)
{
	if (obj_res_inst->sending) {
		obj_res_inst->sending = false;
	}
	return false;
}

struct find_obj_inst_resource_changes_ctx {
	uint8_t modified_count;
	bool has_unmodified;
};

static bool mark_resources_as_ignored_cb(const struct lwm2m_engine_res *obj_res,
					 struct lwm2m_engine_res_inst *obj_res_inst,
					 void *callback_data)
{
	struct find_obj_inst_resource_changes_ctx *ctx = callback_data;
	ctx->modified_count++;

	obj_res_inst->ignore = true;
	return false;
}

static bool find_obj_inst_resource_changes_cb(const struct lwm2m_engine_res *obj_res,
					      struct lwm2m_engine_res_inst *obj_res_inst,
					      void *callback_data)
{
	struct find_obj_inst_resource_changes_ctx *ctx = callback_data;

	if (obj_res_inst->dirty && !obj_res_inst->ignore) {
		ctx->modified_count++;
	} else {
		ctx->has_unmodified = true;
	}

	return 0;
}

static void find_obj_inst_resource_changes(const struct lwm2m_engine_obj_inst *obj_inst,
					   uint8_t *modified_count, bool *has_unmodified)
{
	struct find_obj_inst_resource_changes_ctx ctx = {.modified_count = 0,
							 .has_unmodified = false};

	for_each_res_inst(obj_inst, find_obj_inst_resource_changes_cb, (void *)&ctx);

	*modified_count = ctx.modified_count;
	*has_unmodified = ctx.has_unmodified;
}

static inline struct lwm2m_obj_path *get_modified_path_ptr(struct lwm2m_obj_path *paths,
							   int *paths_count)
{
	if (*paths_count > MAX_MODIFIED_RESOURCE) {
		LOG_WRN("No remaining paths in buffer");
		return NULL;
	}

	const int path_num = (*paths_count)++;

	return &paths[path_num];
}

static inline int add_modified_path_obj_inst(uint16_t obj_id, uint16_t obj_inst_id,
					     struct lwm2m_obj_path *paths, int *paths_count)
{
	struct lwm2m_obj_path *path = get_modified_path_ptr(paths, paths_count);

	if (path == NULL) {
		return -ENOMEM;
	}

	path->obj_id = obj_id;
	path->obj_inst_id = obj_inst_id;
	path->level = LWM2M_PATH_LEVEL_OBJECT_INST;

	return 0;
}

static inline int add_modified_path_res(uint16_t obj_id, uint16_t obj_inst_id, uint16_t res_id,
					struct lwm2m_obj_path *paths, int *paths_count)
{
	struct lwm2m_obj_path *path = get_modified_path_ptr(paths, paths_count);

	if (path == NULL) {
		return -ENOMEM;
	}

	path->obj_id = obj_id;
	path->obj_inst_id = obj_inst_id;
	path->res_id = res_id;
	path->level = LWM2M_PATH_LEVEL_RESOURCE;

	return 0;
}

static inline int add_modified_path_res_inst(uint16_t obj_id, uint16_t obj_inst_id, uint16_t res_id,
					     uint16_t res_inst_id, struct lwm2m_obj_path *paths,
					     int *paths_count)
{
	struct lwm2m_obj_path *path = get_modified_path_ptr(paths, paths_count);

	if (path == NULL) {
		return -ENOMEM;
	}

	path->obj_id = obj_id;
	path->obj_inst_id = obj_inst_id;
	path->res_id = res_id;
	path->res_inst_id = res_inst_id;
	path->level = LWM2M_PATH_LEVEL_RESOURCE_INST;

	return 0;
}

static inline int
remember_partially_modified_object_inst(struct lwm2m_engine_obj_inst *obj_inst, sys_slist_t *list,
					struct lwm2m_auto_send_object_inst_ref *obj_inst_ref_buffer,
					int *obj_inst_ref_buffer_count)
{
	if (*obj_inst_ref_buffer_count > MAX_MODIFIED_RESOURCE) {
		LOG_WRN("No remaining obj inst refs in buffer");
		return -ENOMEM;
	}

	const int ref_num = (*obj_inst_ref_buffer_count)++;
	struct lwm2m_auto_send_object_inst_ref *obj_inst_ref = &obj_inst_ref_buffer[ref_num];

	obj_inst_ref->obj_inst = obj_inst;
	sys_slist_append(list, &obj_inst_ref->node);

	return 0;
}

struct add_modified_path_for_resource_changes_ctx {
	struct lwm2m_engine_obj_inst *obj_inst;
	struct lwm2m_obj_path *modified_paths;
	int *modified_paths_count;
};

static bool add_modified_path_for_resource_changes_cb(const struct lwm2m_engine_res *obj_res,
						      struct lwm2m_engine_res_inst *obj_res_inst,
						      void *callback_data)
{
	struct add_modified_path_for_resource_changes_ctx *ctx =
		(struct add_modified_path_for_resource_changes_ctx *)callback_data;

	if (obj_res_inst->dirty && !obj_res_inst->ignore) {
		if (obj_res->multi_res_inst) {
			add_modified_path_res_inst(ctx->obj_inst->obj->obj_id,
						   ctx->obj_inst->obj_inst_id, obj_res->res_id,
						   obj_res_inst->res_inst_id, ctx->modified_paths,
						   ctx->modified_paths_count);
		} else {
			add_modified_path_res(ctx->obj_inst->obj->obj_id,
					      ctx->obj_inst->obj_inst_id, obj_res->res_id,
					      ctx->modified_paths, ctx->modified_paths_count);
		}
		obj_res_inst->sending = true;
	}
	obj_res_inst->dirty = false;

	/* process all resources */
	return false;
}

static void add_modified_path_for_resource_changes(struct lwm2m_engine_obj_inst *obj_inst,
						   struct lwm2m_obj_path *paths, int *paths_count)
{
	struct add_modified_path_for_resource_changes_ctx ctx = {.obj_inst = obj_inst,
								 .modified_paths = paths,
								 .modified_paths_count =
									 paths_count};

	for_each_res_inst(obj_inst, add_modified_path_for_resource_changes_cb, (void *)&ctx);
}

int lwm2m_engine_auto_send_add_static_path_hint(const struct lwm2m_obj_path *path)
{
	if (static_send_hints_count >= MAX_STATIC_SEND_HINTS) {
		LOG_WRN("Static hint limit exceeded");
		return -ENOMEM;
	}

	if (path->level == LWM2M_PATH_LEVEL_NONE) {
		LOG_ERR("Hint must have level");
		return -EINVAL;
	}

	if (path->level == LWM2M_PATH_LEVEL_RESOURCE_INST) {
		LOG_ERR("Hint can't have resource instance level");
		return -EINVAL;
	}

	memcpy(&static_send_hints[static_send_hints_count], path, sizeof(struct lwm2m_obj_path));
	static_send_hints_count++;

	return 0;
}

static int get_obj_from_path(const struct lwm2m_obj_path *path, struct lwm2m_engine_obj **obj,
			     struct lwm2m_engine_obj_inst **obj_inst, struct lwm2m_engine_res **res)
{
	switch (path->level) {
	case LWM2M_PATH_LEVEL_OBJECT:
		*obj = get_engine_obj(path->obj_id);
		if (*obj == NULL) {
			return -EINVAL;
		}
		return 0;
	case LWM2M_PATH_LEVEL_OBJECT_INST:
		*obj_inst = get_engine_obj_inst(path->obj_id, path->obj_inst_id);
		if (*obj_inst == NULL) {
			return -EINVAL;
		}
		return 0;
	case LWM2M_PATH_LEVEL_RESOURCE:
	case LWM2M_PATH_LEVEL_RESOURCE_INST: {
		struct lwm2m_engine_obj_field *obj_field;
		struct lwm2m_engine_res_inst *res_inst = NULL;
		int ret = path_to_objs(path, obj_inst, &obj_field, res, &res_inst);
		if (ret < 0) {
			return ret;
		}
		return 0;
	}
	case LWM2M_PATH_LEVEL_NONE:
	default:
		return -EINVAL;
	}
}

int lwm2m_engine_auto_send_ignore_path(const struct lwm2m_obj_path *path)
{
	struct lwm2m_engine_obj *obj = NULL;
	struct lwm2m_engine_obj_inst *obj_inst = NULL;
	struct lwm2m_engine_res *res = NULL;
	struct find_obj_inst_resource_changes_ctx ctx = {.modified_count = 0,
							 .has_unmodified = false};
	if (get_obj_from_path(path, &obj, &obj_inst, &res) == 0) {
		lwm2m_registry_lock();

		switch (path->level) {
		case LWM2M_PATH_LEVEL_OBJECT:
			SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
				if (obj_inst->obj->obj_id == path->obj_id) {
					for_each_res_inst(obj_inst, mark_resources_as_ignored_cb,
							  &ctx);
				}
			}
			break;
		case LWM2M_PATH_LEVEL_OBJECT_INST:
			for_each_res_inst(obj_inst, mark_resources_as_ignored_cb, &ctx);
			break;
		case LWM2M_PATH_LEVEL_RESOURCE:
			for (int i = 0; i < res->res_inst_count; ++i) {
				res->res_instances[i].ignore = true;
				ctx.modified_count++;
			}
			break;
		case LWM2M_PATH_LEVEL_RESOURCE_INST:
			if (path->res_inst_id < res->res_inst_count) {
				res->res_instances[path->res_inst_id].ignore = true;
				ctx.modified_count++;
			}
			break;
		case LWM2M_PATH_LEVEL_NONE:
		default:
			break;
		}

		lwm2m_registry_unlock();
	}

	if (ctx.modified_count == 0) {
		char log_path_str_buf[LWM2M_MAX_PATH_STR_SIZE];
		LOG_ERR("Path to ignore is invalid or not found: %s",
			lwm2m_path_log_buf(log_path_str_buf, (struct lwm2m_obj_path *)path));
		return -ENXIO;
	}
	return 0;
}

static int add_matching_hints(struct lwm2m_obj_path paths[], int paths_count,
			      struct lwm2m_obj_path hints[], int hints_count,
			      sys_slist_t *lwm2m_path_list, sys_slist_t *lwm2m_free_list)
{
	struct lwm2m_obj_path *temp;
#ifdef LWM2M_AUTO_SEND_DEBUG
	char log_path_str_buf[LWM2M_MAX_PATH_STR_SIZE];
#endif
	int matches = 0;

	for (int i = 0; i < hints_count; ++i) {
		bool has_match = false;
		struct lwm2m_obj_path *hint = &hints[i];

		for (int j = 0; j < paths_count; ++j) {
			temp = &paths[j];
			if (hint->level > temp->level) {
				continue;
			}

			switch (hint->level) {
			case LWM2M_PATH_LEVEL_NONE:
				/* Invalid */
				break;
			case LWM2M_PATH_LEVEL_OBJECT:
				has_match = hint->obj_id == temp->obj_id;
				break;
			case LWM2M_PATH_LEVEL_OBJECT_INST:
				has_match = hint->obj_id == temp->obj_id &&
					    hint->obj_inst_id == temp->obj_inst_id;
				break;
			case LWM2M_PATH_LEVEL_RESOURCE:
				has_match = hint->obj_id == temp->obj_id &&
					    hint->obj_inst_id == temp->obj_inst_id &&
					    hint->res_id == temp->res_id;
				break;
			case LWM2M_PATH_LEVEL_RESOURCE_INST:
				/* not assuming match */
				break;
			}

			if (has_match) {
				break;
			}
		}

#ifdef LWM2M_AUTO_SEND_DEBUG
		lwm2m_path_log_buf(log_path_str_buf, &hints[i]);
#endif
		if (has_match) {
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("Adding hint: %s", log_path_str_buf);
#endif
			matches++;
			if (lwm2m_engine_add_path_to_list(lwm2m_path_list,
							  lwm2m_free_list,
							  hint) != 0) {
				LOG_WRN("Could not add hint path to list");
				return -ENOMEM;
			}
		} else {
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("Ignoring hint: %s - does not match modified path", log_path_str_buf);
#endif
		}
	}

	return matches;
}

static void reset_pending_resources(void)
{
	lwm2m_registry_lock();
	struct lwm2m_engine_obj_inst *obj_inst;
	SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}
		for_each_res_inst(obj_inst, unmark_resources_cb, NULL);
	}
	lwm2m_registry_unlock();
}

static void mark_resources_as_send_fail(void)
{
	lwm2m_registry_lock();
	struct lwm2m_engine_obj_inst *obj_inst;
	SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}
		for_each_res_inst(obj_inst, mark_sending_resources_as_dirty_cb, NULL);
	}
	lwm2m_registry_unlock();
}

static void mark_resources_as_send_pass(void)
{
	lwm2m_registry_lock();
	struct lwm2m_engine_obj_inst *obj_inst;
	SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}
		for_each_res_inst(obj_inst, mark_resources_as_sent_cb, NULL);
	}
	lwm2m_registry_unlock();
}

void lwm2m_engine_auto_send_set(bool enable)
{
	if (enable) {
		if (!auto_send_enabled) {
			reset_pending_resources();
		}
#if CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX > 0
		/* allow for immediate check after enabling even if min frequency is enabled */
		ignore_max_frequency_for_next_check = true;
#endif
	}
	auto_send_enabled = enable;
}

int lwm2m_engine_auto_send_send_obj_inst(const struct lwm2m_obj_path *path)
{
	lwm2m_registry_lock();

	struct lwm2m_engine_obj_inst *obj_inst = lwm2m_engine_get_obj_inst(path);
	if (obj_inst == NULL) {
		lwm2m_registry_unlock();
		return -ENOENT;
	}
	for_each_res_inst(obj_inst, mark_resources_as_dirty_cb, NULL);

	lwm2m_registry_unlock();
	return 0;
}

static void send_reply_cb(enum lwm2m_send_status status)
{
	time_t duration = k_uptime_get() - last_sent_time;
	last_sent_time = -1;

	switch (status) {
	case LWM2M_SEND_STATUS_SUCCESS:
		LOG_INF("Response SUCCESS after %" PRIi64 "ms", (int64_t)duration);
		mark_resources_as_send_pass();
		break;
	case LWM2M_SEND_STATUS_FAILURE:
		LOG_ERR("Response FAILURE after %" PRIi64 "ms", (int64_t)duration);
		mark_resources_as_send_fail();
		break;
	case LWM2M_SEND_STATUS_TIMEOUT:
		LOG_ERR("Response TIMEOUT after %" PRIi64 "ms", (int64_t)duration);
		mark_resources_as_send_fail();
		break;
	}
}

void lwm2m_enginge_auto_send_run(struct lwm2m_ctx *ctx, const int64_t timestamp)
{
	int modified_paths_count = 0;
	int resources_to_send_count = 0;
	int partial_update_object_inst_count = 0;
	int rc;
	struct lwm2m_engine_obj_inst *obj_inst;
	struct lwm2m_auto_send_object_inst_ref *obj_inst_ref;

	sys_slist_init(&partial_update_list);
#ifdef LWM2M_AUTO_SEND_DEBUG
	char log_path_str_buf[LWM2M_MAX_PATH_STR_SIZE];
#endif

	if (last_sent_time > 0) {
		if (k_uptime_get() - last_sent_time < SEND_RESPONSE_TIMEOUT) {
			return;
		}
		LOG_ERR("Auto send still waiting for response - abort after %" PRIi64 "ms",
			(int64_t)(k_uptime_get() - last_sent_time));
		mark_resources_as_send_fail();
		last_sent_time = -1;
	}

	if (!auto_send_enabled) {
		return;
	}

#if CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX > 0
	if (ignore_max_frequency_for_next_check) {
		ignore_max_frequency_for_next_check = false;
	} else if ((timestamp - last_auto_send_check) <=
		   CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX) {
		return;
	}
#endif

	lwm2m_registry_lock();

	SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}

		uint8_t modified_count = 0;
		bool has_unmodified;
		find_obj_inst_resource_changes(obj_inst, &modified_count, &has_unmodified);

		if (modified_count > MAX_MODIFIED_RESOURCE) {
			LOG_ERR("Can't send object: %d/%d, it has too many resources (%i). "
				"Update CONFIG_LWM2M_ENGINE_AUTO_SEND_MODIFIED_RESOURCES_MAX",
				obj_inst->obj->obj_id, obj_inst->obj_inst_id, modified_count);
		}

		if (modified_count > 0 && !has_unmodified) {
			if (resources_to_send_count + modified_count > MAX_MODIFIED_RESOURCE) {
				continue;
			}
			resources_to_send_count += modified_count;

			LOG_DBG("Fully modified object: %d/%d", obj_inst->obj->obj_id,
				obj_inst->obj_inst_id);

			rc = add_modified_path_obj_inst(obj_inst->obj->obj_id,
							obj_inst->obj_inst_id, modified_paths,
							&modified_paths_count);
			if (rc < 0) {
				LOG_ERR("Can't process more than %d modified resource paths",
					MAX_MODIFIED_RESOURCE);
				goto cleanup;
			}

			for_each_res_inst(obj_inst, mark_resources_as_sending_cb, NULL);
		} else if (modified_count > 0 && has_unmodified &&
			   resources_to_send_count + modified_count <= MAX_MODIFIED_RESOURCE) {
			resources_to_send_count += modified_count;

			LOG_DBG("Partially modified object: %d/%d", obj_inst->obj->obj_id,
				obj_inst->obj_inst_id);

			rc = remember_partially_modified_object_inst(
				obj_inst, &partial_update_list, partial_update_list_buf,
				&partial_update_object_inst_count);
			if (rc < 0) {
				LOG_ERR("Can't process more than %d partially modified resources",
					MAX_MODIFIED_RESOURCE);
				goto cleanup;
			}
		}

		if (resources_to_send_count && auto_send_recover) {
			auto_send_recover = false;
			LOG_INF("Recover from previous send error by reducing the amount of paths "
				"to send at once.");
			break;
		}
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&partial_update_list, obj_inst_ref, node) {
		obj_inst = obj_inst_ref->obj_inst;

		add_modified_path_for_resource_changes(obj_inst, modified_paths,
						       &modified_paths_count);
	}

	last_auto_send_check = timestamp;

	if (modified_paths_count > 0) {
		if (modified_paths_count > CONFIG_LWM2M_COMPOSITE_PATH_LIST_SIZE) {
			LOG_WRN("More than %d modified paths: %d - use send hints to reduce "
				"paths",
				CONFIG_LWM2M_COMPOSITE_PATH_LIST_SIZE, modified_paths_count);
		} else {
			LOG_DBG("Modified paths: %d", modified_paths_count);
		}

		lwm2m_engine_path_list_init(&next_lwm2m_send_path_list,
					    &next_lwm2m_send_path_free_list,
					    next_lwm2m_send_path_list_buf, MAX_SEND_PATHS_OVERALL);
		for (int i = 0; i < modified_paths_count; ++i) {
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("Adding path for resource: %s",
				lwm2m_path_log_buf(log_path_str_buf, &modified_paths[i]));
#endif
			if (lwm2m_engine_add_path_to_list(&next_lwm2m_send_path_list,
							  &next_lwm2m_send_path_free_list,
							  &modified_paths[i]) != 0) {
				LOG_ERR("Could not add resource path to send list");
				rc = -ENOMEM;
				goto cleanup;
			}
		}

		rc = add_matching_hints(modified_paths, modified_paths_count, static_send_hints,
					static_send_hints_count, &next_lwm2m_send_path_list,
					&next_lwm2m_send_path_free_list);
		if (rc < 0) {
			LOG_ERR("Could not add static send hints: %d", rc);
			goto cleanup;
		}

#ifdef LWM2M_AUTO_SEND_DEBUG
		LOG_DBG("Before duplicate path removal:");

		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&next_lwm2m_send_path_list, entry, tmp, node) {
			LOG_DBG("- %s", lwm2m_path_log_buf(log_path_str_buf, &entry->path));
		}
#endif
		lwm2m_engine_clear_duplicate_path(&next_lwm2m_send_path_list,
						  &next_lwm2m_send_path_free_list);

		int send_path_count = 0;

#ifdef LWM2M_AUTO_SEND_DEBUG
		LOG_DBG("Final sending paths:");
#endif
		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&next_lwm2m_send_path_list, entry, tmp, node) {
			memcpy(&next_lwm2m_send_paths[send_path_count], &entry->path,
			       sizeof(entry->path));
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("- %s", lwm2m_path_log_buf(log_path_str_buf, &entry->path));
#endif
			send_path_count++;
		}

		if (send_path_count > 0) {
			rc = lwm2m_send_cb(ctx, next_lwm2m_send_paths, send_path_count,
					   send_reply_cb);
			if (rc < 0) {
				LOG_ERR("Automatic lwm2m send failed "
					"- retry on next send with reduced amount of paths");
				mark_resources_as_send_fail();
				auto_send_recover = true;
				goto cleanup;
			}
			last_sent_time = k_uptime_get();
			LOG_INF("Send total %i paths and %i resources", send_path_count,
				resources_to_send_count);

		} else {
			LOG_DBG("Nothing to send");
		}
	}

cleanup:
	lwm2m_registry_unlock();
}

void lwm2m_engine_auto_send_send_all_objs(void)
{
	struct lwm2m_engine_obj_inst *obj_inst;

	lwm2m_registry_lock();
	SYS_SLIST_FOR_EACH_CONTAINER(lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}

		for_each_res_inst(obj_inst, mark_resources_as_dirty_cb, NULL);
	}
	lwm2m_registry_unlock();
}
