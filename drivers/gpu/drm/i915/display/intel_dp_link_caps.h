/* SPDX-License-Identifier: MIT */
/* Copyright © 2026 Intel Corporation */

#ifndef __INTEL_DP_LINK_CAPS_H__
#define __INTEL_DP_LINK_CAPS_H__

#include <linux/bitops.h>
#include <linux/types.h>

struct intel_connector;
struct intel_dp;
struct intel_dp_link_caps;
struct intel_dp_link_config;

/**
 * enum intel_dp_link_caps_order_key - key used to order configurations
 * @INTEL_DP_LINK_CAPS_ORDER_KEY_BW:
 *   Order configurations by bandwidth, then by link rate.
 * @INTEL_DP_LINK_CAPS_ORDER_KEY_RATE_LANE:
 *   Order configurations by link rate, then by lane count.
 * @INTEL_DP_LINK_CAPS_ORDER_KEY_LANE_RATE:
 *   Order configurations by lane count, then by link rate.
 * @INTEL_DP_LINK_CAPS_ORDER_KEY_NUM:
 *   Number of ordering keys.
 *
 * Selects how a caller wants the configuration table to be ordered,
 * together with an &enum intel_dp_link_caps_order_direction, for
 * iteration queries.
 *
 * See also:
 *  - &struct intel_dp_link_caps_order
 *  - intel_dp_link_caps_get_max_config()
 */
enum intel_dp_link_caps_order_key {
	INTEL_DP_LINK_CAPS_ORDER_KEY_BW,
	INTEL_DP_LINK_CAPS_ORDER_KEY_RATE_LANE,
	INTEL_DP_LINK_CAPS_ORDER_KEY_LANE_RATE,

	INTEL_DP_LINK_CAPS_ORDER_KEY_NUM
};

/**
 * enum intel_dp_link_caps_order_direction - iteration direction
 * @INTEL_DP_LINK_CAPS_ORDER_DIR_ASC:
 *   Iterate in ascending order according to the selected ordering key.
 * @INTEL_DP_LINK_CAPS_ORDER_DIR_DESC:
 *   Iterate in descending order according to the selected ordering key.
 * @INTEL_DP_LINK_CAPS_ORDER_DIR_NUM:
 *   Number of ordering directions.
 *
 * Selects the direction associated with an
 * &enum intel_dp_link_caps_order_key for iteration queries.
 *
 * See also:
 *  - &struct intel_dp_link_caps_order
 */
enum intel_dp_link_caps_order_direction {
	INTEL_DP_LINK_CAPS_ORDER_DIR_ASC,
	INTEL_DP_LINK_CAPS_ORDER_DIR_DESC,

	INTEL_DP_LINK_CAPS_ORDER_DIR_NUM
};

/**
 * struct intel_dp_link_caps_order - configuration ordering
 * @key:
 *   Key used to order configurations.
 * @dir:
 *   Direction of the selected ordering.
 *
 * Describes an iteration order for link configurations.
 *
 * See also:
 *  - for_each_dp_link_config()
 */
struct intel_dp_link_caps_order {
	enum intel_dp_link_caps_order_key key;
	enum intel_dp_link_caps_order_direction dir;
};

struct intel_dp_link_caps_filter {
	u32 config_mask;
};

#define INTEL_DP_LINK_CAPS_FILTER_NONE	\
	((struct intel_dp_link_caps_filter){ .config_mask = 0 })
#define INTEL_DP_LINK_CAPS_FILTER_ALL	\
	((struct intel_dp_link_caps_filter){ .config_mask = (u32)-1 })

struct intel_dp_link_caps_iter {
	struct intel_dp_link_caps *link_caps;
	int pos;
	struct intel_dp_link_caps_order order;
	struct intel_dp_link_caps_filter filter;

	bool (*get_next_config)(struct intel_dp_link_caps_iter *iter,
				struct intel_dp_link_config *config);
};

/**
 * for_each_dp_link_config - iterate allowed link configurations
 * @__iter:
 *   &struct intel_dp_link_caps_iter being iterated
 * @__config:
 *   pointer to &struct intel_dp_link_config filled for each match
 */
#define for_each_dp_link_config(__iter, __config) \
	while ((__iter)->get_next_config((__iter), (__config)))

void intel_dp_link_caps_iter_start(struct intel_dp_link_caps_iter *iter,
				   struct intel_dp_link_caps *link_caps,
				   struct intel_dp_link_caps_order order,
				   struct intel_dp_link_caps_filter filter);

void intel_dp_link_caps_iter_end(struct intel_dp_link_caps_iter *iter);

struct intel_dp_link_caps_order
intel_dp_link_caps_connector_compute_order(struct intel_connector *connector);
struct intel_dp_link_caps_order
intel_dp_link_caps_connector_fallback_order(bool is_mst);

int intel_dp_common_len_rate_limit(struct intel_dp_link_caps *link_caps,
				   int max_rate);
int intel_dp_common_rate(struct intel_dp_link_caps *link_caps, int index);
int intel_dp_link_caps_common_rate_idx(struct intel_dp_link_caps *link_caps, int rate);
int intel_dp_max_common_rate(struct intel_dp_link_caps *link_caps);
int intel_dp_link_caps_num_common_rates(struct intel_dp_link_caps *link_caps);
int intel_dp_link_caps_max_common_lane_count(struct intel_dp_link_caps *link_caps);

void intel_dp_link_caps_print_common_rates(struct intel_dp_link_caps *link_caps);

void intel_dp_link_caps_get_forced_params(struct intel_dp_link_caps *link_caps,
					  struct intel_dp_link_config *forced_params);

int intel_dp_link_config_index(struct intel_dp_link_caps *link_caps,
			       int link_rate, int lane_count);
void intel_dp_link_config_get(struct intel_dp_link_caps *link_caps,
			      int idx, int *link_rate, int *lane_count);

bool intel_dp_link_caps_filter_add(struct intel_dp_link_caps *link_caps,
				   struct intel_dp_link_caps_filter *filter,
				   const struct intel_dp_link_config *config);

bool intel_dp_link_caps_get_max_config(struct intel_dp_link_caps *link_caps,
				       enum intel_dp_link_caps_order_key order_key,
				       struct intel_dp_link_caps_filter filter,
				       struct intel_dp_link_config *max_config);

void intel_dp_link_caps_get_max_limits(struct intel_dp_link_caps *link_caps,
				       struct intel_dp_link_config *max_link_limits);
bool intel_dp_link_caps_set_max_limits(struct intel_dp_link_caps *link_caps,
				       const struct intel_dp_link_config *max_link_limits);
void intel_dp_link_caps_reset_max_limits(struct intel_dp_link_caps *link_caps);

bool intel_dp_link_caps_update(struct intel_dp_link_caps *link_caps,
			       const int *rates, int num_rates, int max_lane_count,
			       bool reset);
void intel_dp_link_caps_reset(struct intel_dp_link_caps *link_caps);

void intel_dp_link_caps_debugfs_add(struct intel_connector *connector);

struct intel_dp_link_caps *intel_dp_link_caps_init(struct intel_dp *intel_dp);
void intel_dp_link_caps_cleanup(struct intel_dp_link_caps *link_caps);

#endif /* __INTEL_DP_LINK_CAPS_H__ */
