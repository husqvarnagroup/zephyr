#define LOG_MODULE_NAME net_lwm2m_engine_auto_send
#define LOG_LEVEL	CONFIG_LWM2M_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "lwm2m_auto_send.h"
#include "lwm2m_engine.h"
#include "lwm2m_object.h"
#include "lwm2m_observation.h"
#include "lwm2m_util.h"

#include <ctype.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_MODIFIED_RESOURCE  CONFIG_LWM2M_ENGINE_AUTO_SEND_MODIFIED_RESOURCES_MAX
#define MAX_NEXT_SEND_HINTS    CONFIG_LWM2M_ENGINE_AUTO_SEND_NEXT_PATH_HINTS_MAX
#define MAX_STATIC_SEND_HINTS  CONFIG_LWM2M_ENGINE_AUTO_SEND_STATIC_PATH_HINTS_MAX
#define MAX_IGNORED_PATHS      CONFIG_LWM2M_ENGINE_AUTO_SEND_IGNORED_PATHS_MAX
#define MAX_SEND_PATHS_OVERALL (MAX_MODIFIED_RESOURCE + MAX_NEXT_SEND_HINTS + MAX_STATIC_SEND_HINTS)

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

static int64_t auto_send_enabled_after = -1;
static int64_t auto_send_disabled_after = -1;
static uint32_t auto_send_cooldown = 0;
static int64_t last_auto_send_check = -1;
static int64_t last_auto_send_transmission = -1;
#if CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX > 0
static bool ignore_max_frequency_for_next_check = false;
#endif

static int next_send_hints_count = 0;
static struct lwm2m_obj_path next_send_hints[MAX_NEXT_SEND_HINTS] = {};

static int static_send_hints_count = 0;
static struct lwm2m_obj_path static_send_hints[MAX_STATIC_SEND_HINTS] = {};

static int ignored_paths_count = 0;
static struct lwm2m_obj_path ignored_paths[MAX_IGNORED_PATHS] = {};

static struct lwm2m_obj_path_list next_lwm2m_send_path_list_buf[MAX_SEND_PATHS_OVERALL];
static struct lwm2m_obj_path_list *entry, *tmp;
static sys_slist_t next_lwm2m_send_path_list;
static sys_slist_t next_lwm2m_send_path_free_list;

static char next_lwm2m_send_path_strings_buffer[MAX_SEND_PATHS_OVERALL][LWM2M_MAX_PATH_STR_SIZE];
static const char *next_lwm2m_send_paths[MAX_SEND_PATHS_OVERALL];

static struct lwm2m_obj_path modified_paths[MAX_MODIFIED_RESOURCE];
static struct lwm2m_auto_send_object_inst_ref partial_update_list_buf[MAX_MODIFIED_RESOURCE];
static sys_slist_t partial_update_list;

static void for_each_res_inst(const struct lwm2m_engine_obj_inst *obj_inst,
			      for_each_res_inst_callback_t callback, void *callback_ctx)
{
	struct lwm2m_engine_res *obj_res;
	struct lwm2m_engine_res_inst *obj_res_inst;
	struct lwm2m_engine_obj_field *obj_field;

	for (int res_idx = 0; res_idx < obj_inst->resource_count; res_idx++) {
		obj_res = &obj_inst->resources[res_idx];
		obj_field = lwm2m_get_engine_obj_field(obj_inst->obj, obj_res->res_id);
		if (!LWM2M_HAS_PERM(obj_field, LWM2M_PERM_R)) {
			continue;
		}
		for (int i = 0; i < obj_res->res_inst_count; i++) {
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

static bool mark_all_resources_as_sent_cb(const struct lwm2m_engine_res *obj_res,
					  struct lwm2m_engine_res_inst *obj_res_inst,
					  void *callback_data)
{
	int64_t *timestamp = (int64_t *)callback_data;
	obj_res_inst->last_sent = *timestamp;
	return false;
}

static void mark_all_resources_as_sent(struct lwm2m_engine_obj_inst *obj_inst,
				       const int64_t timestamp)
{
	for_each_res_inst(obj_inst, mark_all_resources_as_sent_cb, (void *)&timestamp);
}

struct find_obj_inst_resource_changes_ctx {
	int64_t time_window_begin;
	int64_t time_window_end_inclusive;
	bool has_modified;
	bool has_unmodified;
};

static bool find_obj_inst_resource_changes_cb(const struct lwm2m_engine_res *obj_res,
					      struct lwm2m_engine_res_inst *obj_res_inst,
					      void *callback_data)
{
	struct find_obj_inst_resource_changes_ctx *ctx =
		(struct find_obj_inst_resource_changes_ctx *)callback_data;

	// skip resources modified after current range in question
	if (obj_res_inst->last_modified > ctx->time_window_end_inclusive) {
		return false;
	}

	if (obj_res_inst->last_modified <= ctx->time_window_begin) {
		ctx->has_unmodified = true;
	}

	if (obj_res_inst->last_modified > ctx->time_window_begin) {
		ctx->has_modified = true;
	}

	// abort if found changes, we don't need to process all resources
	return ctx->has_unmodified && ctx->has_modified;
}

static void find_obj_inst_resource_changes(const struct lwm2m_engine_obj_inst *obj_inst,
					   const int64_t timestamp, bool *has_modified,
					   bool *has_unmodified)
{
	struct find_obj_inst_resource_changes_ctx ctx = {.time_window_begin = last_auto_send_check,
							 .time_window_end_inclusive = timestamp,
							 .has_modified = false,
							 .has_unmodified = false};

	for_each_res_inst(obj_inst, find_obj_inst_resource_changes_cb, (void *)&ctx);

	*has_modified = ctx.has_modified;
	*has_unmodified = ctx.has_unmodified;
}

static inline struct lwm2m_obj_path *get_modified_path_ptr(struct lwm2m_obj_path *modified_paths,
							   int *modified_paths_count)
{
	if (*modified_paths_count > MAX_MODIFIED_RESOURCE) {
		LOG_WRN("No remaining paths in buffer");
		return NULL;
	}

	int path_num = (*modified_paths_count)++;
	return &modified_paths[path_num];
}

static inline int add_modified_path_obj_inst(uint16_t obj_id, uint16_t obj_inst_id,
					     struct lwm2m_obj_path *modified_paths,
					     int *modified_paths_count)
{

	struct lwm2m_obj_path *path = get_modified_path_ptr(modified_paths, modified_paths_count);
	if (path == NULL) {
		return -ENOMEM;
	}

	path->obj_id = obj_id;
	path->obj_inst_id = obj_inst_id;
	path->level = LWM2M_PATH_LEVEL_OBJECT_INST;

	return 0;
}

static inline int add_modified_path_res(uint16_t obj_id, uint16_t obj_inst_id, uint16_t res_id,
					struct lwm2m_obj_path *modified_paths,
					int *modified_paths_count)
{

	struct lwm2m_obj_path *path = get_modified_path_ptr(modified_paths, modified_paths_count);
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
					     uint16_t res_inst_id,
					     struct lwm2m_obj_path *modified_paths,
					     int *modified_paths_count)
{

	struct lwm2m_obj_path *path = get_modified_path_ptr(modified_paths, modified_paths_count);
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

	int ref_num = (*obj_inst_ref_buffer_count)++;

	struct lwm2m_auto_send_object_inst_ref *obj_inst_ref = &obj_inst_ref_buffer[ref_num];
	obj_inst_ref->obj_inst = obj_inst;
	sys_slist_append(list, &obj_inst_ref->node);

	return 0;
}

struct add_modified_path_for_resource_changes_ctx {
	int64_t time_window_begin;
	int64_t time_window_end_inclusive;
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

	// skip resources modified after current range in question
	if (obj_res_inst->last_modified > ctx->time_window_end_inclusive) {
		return false;
	}

	if (obj_res_inst->last_modified > ctx->time_window_begin) {
		if (obj_res->multi_res_inst) {
			add_modified_path_res_inst(ctx->obj_inst->obj->obj_id,
						   ctx->obj_inst->obj_inst_id, obj_res->res_id,
						   obj_res_inst->res_inst_id, ctx->modified_paths,
						   ctx->modified_paths_count);

			//
		} else {
			add_modified_path_res(ctx->obj_inst->obj->obj_id,
					      ctx->obj_inst->obj_inst_id, obj_res->res_id,
					      ctx->modified_paths, ctx->modified_paths_count);
		}
	}

	// process all resources
	return false;
}

static void add_modified_path_for_resource_changes(struct lwm2m_engine_obj_inst *obj_inst,
						   const int64_t timestamp,
						   struct lwm2m_obj_path *modified_paths,
						   int *modified_paths_count)
{
	struct add_modified_path_for_resource_changes_ctx ctx = {
		.time_window_begin = last_auto_send_check,
		.time_window_end_inclusive = timestamp,
		.obj_inst = obj_inst,
		.modified_paths = modified_paths,
		.modified_paths_count = modified_paths_count};

	for_each_res_inst(obj_inst, add_modified_path_for_resource_changes_cb, (void *)&ctx);
}

int lwm2m_engine_auto_send_add_static_path_hint_str(const char *path_str)
{
	struct lwm2m_obj_path path;
	int rc;

	rc = lwm2m_string_to_path(path_str, &path, '/');
	if (rc < 0) {
		return rc;
	}

	return lwm2m_engine_auto_send_add_static_path_hint(&path);
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

int lwm2m_engine_auto_send_add_next_path_hint(const struct lwm2m_obj_path *path)
{
	if (next_send_hints_count >= MAX_NEXT_SEND_HINTS) {
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

	memcpy(&next_send_hints[next_send_hints_count], path, sizeof(struct lwm2m_obj_path));
	next_send_hints_count++;

	return 0;
}

int lwm2m_engine_auto_send_ignore_path(const struct lwm2m_obj_path *path)
{
	if (ignored_paths_count >= MAX_IGNORED_PATHS) {
		return -ENOMEM;
	}

	if (path->level == LWM2M_PATH_LEVEL_NONE) {
		LOG_ERR("Path must have level");
		return -EINVAL;
	}

	memcpy(&ignored_paths[ignored_paths_count], path, sizeof(struct lwm2m_obj_path));
	ignored_paths_count++;

	return 0;
}

static int add_matching_hints(struct lwm2m_obj_path modified_paths[], int modified_paths_count,
			      struct lwm2m_obj_path hints[], int hints_count,
			      sys_slist_t *next_lwm2m_send_path_list,
			      sys_slist_t *next_lwm2m_send_path_free_list)
{
	struct lwm2m_obj_path *temp;
#ifdef LWM2M_AUTO_SEND_DEBUG
	char path_str_buf[LWM2M_MAX_PATH_STR_SIZE];
#endif
	int matches = 0;
	for (int i = 0; i < hints_count; ++i) {
		bool has_match = false;
		struct lwm2m_obj_path *hint = &hints[i];
		for (int j = 0; j < modified_paths_count; ++j) {
			temp = &modified_paths[j];
			if (hint->level > temp->level) {
				continue;
			}

			switch (hint->level) {
			case LWM2M_PATH_LEVEL_NONE:
				// Invalid
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
				// not assuming match
				break;
			}

			if (has_match) {
				break;
			}
		}

#ifdef LWM2M_AUTO_SEND_DEBUG
		lwm2m_path_to_string(path_str_buf, sizeof(path_str_buf), &hints[i], hints[i].level);
#endif
		if (has_match) {
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("Adding hint: %s", path_str_buf);
#endif
			matches++;
			if (lwm2m_engine_add_path_to_list(next_lwm2m_send_path_list,
							  next_lwm2m_send_path_free_list,
							  hint) != 0) {
				LOG_WRN("Could not add hint path to list");
				return -ENOMEM;
			}
		} else {
#ifdef LWM2M_AUTO_SEND_DEBUG
			LOG_DBG("Ignoring hint: %s - does not match modified path", path_str_buf);
#endif
		}
	}

	return matches;
}

int lwm2m_engine_auto_send_enable(int64_t after, uint32_t cooldown)
{
	if (after <= auto_send_disabled_after) {
		LOG_ERR("Can't enable auto send before disabled time point");
		return -EINVAL;
	}
	if (after <= auto_send_enabled_after) {
		LOG_ERR("Can't enable auto send before previous enabled time point");
		return -EINVAL;
	}

	if (after <= auto_send_enabled_after) {
		LOG_ERR("Can't enable auto send before previous enabled time point");
		return -EINVAL;
	}

	auto_send_disabled_after = -1;
	auto_send_enabled_after = after;
	auto_send_cooldown = cooldown;

	// set last auto send so that we ignore resources modified before enabling
	last_auto_send_check = auto_send_enabled_after;
#if CONFIG_LWM2M_ENGINE_AUTO_SEND_CHECK_FREQUENCY_MAX > 0
	// allow for immediate check after enabling even if min frequency is enabled
	ignore_max_frequency_for_next_check = true;
#endif
	// ensure that we don't run into cooldown
	last_auto_send_transmission = last_auto_send_check - cooldown;

	return 0;
}

int lwm2m_engine_auto_send_disable(int64_t after)
{
	if (auto_send_enabled_after < 0) {
		LOG_ERR("Can't disable auto send because it is not enabled");
		return -EINVAL;
	}

	if (after < auto_send_disabled_after) {
		LOG_ERR("Can't disable auto send before previous disable time point");
		return -EINVAL;
	}

	if (after < auto_send_enabled_after) {
		LOG_ERR("Can't disable auto send before enabled time point");
		return -EINVAL;
	}

	auto_send_disabled_after = after;

	return 0;
}

bool is_ignored_path(struct lwm2m_obj_path *path)
{
	bool is_ignored = false;
	struct lwm2m_obj_path *ignored_path;

	for (int i = 0; i < ignored_paths_count; i++) {
		ignored_path = &ignored_paths[i];
		if (ignored_path->level > path->level) {
			continue;
		}

		switch (ignored_path->level) {
		case LWM2M_PATH_LEVEL_OBJECT:
			is_ignored = ignored_path->obj_id == path->obj_id;
			break;

		case LWM2M_PATH_LEVEL_OBJECT_INST:
			is_ignored = ignored_path->obj_id == path->obj_id &&
				     ignored_path->obj_inst_id == path->obj_inst_id;
			break;

		case LWM2M_PATH_LEVEL_RESOURCE:
			is_ignored = ignored_path->obj_id == path->obj_id &&
				     ignored_path->obj_inst_id == path->obj_inst_id &&
				     ignored_path->res_id == path->res_id;
			break;
		case LWM2M_PATH_LEVEL_RESOURCE_INST:
			is_ignored = ignored_path->obj_id == path->obj_id &&
				     ignored_path->obj_inst_id == path->obj_inst_id &&
				     ignored_path->res_id == path->res_id &&
				     ignored_path->res_inst_id == path->res_inst_id;
			break;
		}
		if (is_ignored) {
			return true;
		}
	}

	return false;
}

void check_automatic_lwm2m_sends(struct lwm2m_ctx *ctx, const int64_t timestamp)
{
	int modified_paths_count = 0;
	int partial_update_object_inst_count = 0;
	int rc;
	struct lwm2m_engine_obj_inst *obj_inst;
	struct lwm2m_auto_send_object_inst_ref *obj_inst_ref;

	sys_slist_init(&partial_update_list);
#ifdef LWM2M_AUTO_SEND_DEBUG
	char path_str_buf[LWM2M_MAX_PATH_STR_SIZE];
#endif

	if (auto_send_enabled_after < 0 || timestamp <= auto_send_enabled_after ||
	    (auto_send_disabled_after >= 0 && timestamp > auto_send_disabled_after) ||
	    timestamp <= last_auto_send_check) {
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

	if ((auto_send_cooldown > 0 &&
	     (timestamp - last_auto_send_transmission) <= auto_send_cooldown)) {
		LOG_DBG("Cooldown active. remaining: %" PRId64 " ms",
			auto_send_cooldown - (timestamp - last_auto_send_transmission));
		return;
	}

	lwm2m_registry_lock();

	SYS_SLIST_FOR_EACH_CONTAINER (lwm2m_engine_obj_inst_list(), obj_inst, node) {
		if (!obj_inst->resources || obj_inst->resource_count == 0U) {
			continue;
		}

		bool has_modified, has_unmodified;
		find_obj_inst_resource_changes(obj_inst, timestamp, &has_modified, &has_unmodified);

		if (has_modified && !has_unmodified) {
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

			mark_all_resources_as_sent(obj_inst, timestamp);
		} else if (has_modified && has_unmodified) {
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
	}

	SYS_SLIST_FOR_EACH_CONTAINER (&partial_update_list, obj_inst_ref, node) {
		obj_inst = obj_inst_ref->obj_inst;

		add_modified_path_for_resource_changes(obj_inst, timestamp, modified_paths,
						       &modified_paths_count);
	}

	last_auto_send_check = timestamp;

	if (modified_paths_count > 0) {
		if (modified_paths_count > CONFIG_LWM2M_COMPOSITE_PATH_LIST_SIZE &&
		    next_send_hints_count == 0) {
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
			lwm2m_path_to_string(path_str_buf, sizeof(path_str_buf), &modified_paths[i],
					     modified_paths[i].level);
			LOG_DBG("Adding path for resource: %s", path_str_buf);
#endif
			if (lwm2m_engine_add_path_to_list(&next_lwm2m_send_path_list,
							  &next_lwm2m_send_path_free_list,
							  &modified_paths[i]) != 0) {
				LOG_ERR("Could not add resource path to send list");
				rc = -ENOMEM;
				goto cleanup;
			}
		}

		rc = add_matching_hints(modified_paths, modified_paths_count, next_send_hints,
					next_send_hints_count, &next_lwm2m_send_path_list,
					&next_lwm2m_send_path_free_list);
		if (rc < 0) {
			LOG_ERR("Could not add next send hints: %d", rc);
			goto cleanup;
		}
		// clear next send hints
		next_send_hints_count = 0;

		rc = add_matching_hints(modified_paths, modified_paths_count, static_send_hints,
					static_send_hints_count, &next_lwm2m_send_path_list,
					&next_lwm2m_send_path_free_list);
		if (rc < 0) {
			LOG_ERR("Could not add static send hints: %d", rc);
			goto cleanup;
		}

#ifdef LWM2M_AUTO_SEND_DEBUG
		LOG_DBG("Before duplicate path removal:");

		SYS_SLIST_FOR_EACH_CONTAINER_SAFE (&next_lwm2m_send_path_list, entry, tmp, node) {
			lwm2m_path_to_string(path_str_buf, sizeof(path_str_buf), &entry->path,
					     entry->path.level);
			LOG_DBG("- %s", path_str_buf);
		}
#endif
		lwm2m_engine_clear_duplicate_path(&next_lwm2m_send_path_list,
						  &next_lwm2m_send_path_free_list);
#ifdef LWM2M_AUTO_SEND_DEBUG
		LOG_DBG("After duplicate path removal:");
		SYS_SLIST_FOR_EACH_CONTAINER_SAFE (&next_lwm2m_send_path_list, entry, tmp, node) {
			lwm2m_path_to_string(path_str_buf, sizeof(path_str_buf), &entry->path,
					     entry->path.level);
			LOG_DBG("- %s", path_str_buf);
		}
#endif

		int send_path_count = 0;
#ifdef LWM2M_AUTO_SEND_DEBUG
		LOG_DBG("Final sending paths:");
#endif
		SYS_SLIST_FOR_EACH_CONTAINER_SAFE (&next_lwm2m_send_path_list, entry, tmp, node) {
			lwm2m_path_to_string(next_lwm2m_send_path_strings_buffer[send_path_count],
					     LWM2M_MAX_PATH_STR_SIZE, &entry->path,
					     entry->path.level);

			if (!is_ignored_path(&entry->path)) {
				next_lwm2m_send_paths[send_path_count] =
					next_lwm2m_send_path_strings_buffer[send_path_count];
#ifdef LWM2M_AUTO_SEND_DEBUG
				LOG_DBG("- %s",
					next_lwm2m_send_path_strings_buffer[send_path_count]);
#endif
				send_path_count++;
			} else {
#ifdef LWM2M_AUTO_SEND_DEBUG
				LOG_DBG("- ignored %s",
					next_lwm2m_send_path_strings_buffer[send_path_count]);
#endif
			}
		}

		if (send_path_count > 0) {
			rc = lwm2m_engine_send(ctx, next_lwm2m_send_paths, send_path_count, true);
			if (rc < 0) {
				LOG_ERR("Automatic lwm2m send failed");
				goto cleanup;
			}
		} else {
			LOG_DBG("Nothing to send");
		}
		last_auto_send_transmission = timestamp;
	}

cleanup:
	lwm2m_registry_unlock();
}
