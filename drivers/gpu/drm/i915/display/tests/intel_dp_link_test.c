// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <kunit/test.h>

#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/log2.h>
#include <linux/prandom.h>
#include <linux/random.h>

#include <drm/display/drm_dp_helper.h>

#include <drm/intel/display_member.h>

#include "intel_connector.h"
#include "intel_display_core.h"
#include "intel_display_types.h"
#include "intel_dp_link_caps.h"
#include "intel_dp_link_training.h"

#define LINK_TEST_NUM_LANE_CONFIGS(__max_lane_count) \
	(ilog2(__max_lane_count) + 1)

#define LINK_TEST_NUM_CONFIGS(__num_rates, __max_lane_count) \
	((__num_rates) * LINK_TEST_NUM_LANE_CONFIGS(__max_lane_count))

#define LINK_TEST_MAX_LANE_COUNT		((u32)4)
#define LINK_TEST_MAX_CONFIGS			LINK_TEST_NUM_CONFIGS(DP_MAX_SUPPORTED_RATES, \
								      LINK_TEST_MAX_LANE_COUNT)

#define LINK_TEST_NUM_RANDOM_ITERATIONS		50

struct test_ctx {
	struct {
		struct intel_display display;
		struct device device;
		struct __intel_generic_device generic_device;

		struct intel_connector connector;
		struct intel_digital_port dig_port;

		struct intel_crtc_state crtc_state;
	} dev;

	const struct intel_dp_link_caps_test_ops *link_caps_ops;
	const struct intel_dp_link_training_test_ops *link_training_ops;

	struct rnd_state rnd;
};

struct link_rate_set {
	const int *entries;
	int size;
};

struct link_config_set {
	struct intel_dp_link_config entries[LINK_TEST_MAX_CONFIGS];
	int size;
};

static const int standard_dp_link_rates[] = {
	162000, 270000, 540000, 810000, 1000000, 1350000, 2000000
};

#define LINK_TEST_NUM_STANDARD_RATES (ARRAY_SIZE(standard_dp_link_rates))

static const struct link_config_set standard_dp_link_configs[] = {
	[INTEL_DP_LINK_CAPS_ORDER_KEY_BW] = {                        /* MBps    PBN    */
		.entries = {
			{ .rate =  162000, .lane_count = 1 }, /*  162.0    3.00 */
			{ .rate =  270000, .lane_count = 1 }, /*  270.0    5.00 */
			{ .rate =  162000, .lane_count = 2 }, /*  324.0    6.00 */
			{ .rate =  270000, .lane_count = 2 }, /*  540.0   10.00 */
			{ .rate =  540000, .lane_count = 1 }, /*  540.0   10.00 */
			{ .rate =  162000, .lane_count = 4 }, /*  648.0   12.00 */
			{ .rate =  810000, .lane_count = 1 }, /*  810.0   15.00 */
			{ .rate =  270000, .lane_count = 4 }, /* 1080.0   20.00 */
			{ .rate =  540000, .lane_count = 2 }, /* 1080.0   20.00 */
			{ .rate = 1000000, .lane_count = 1 }, /* 1208.9   22.39 */
			{ .rate =  810000, .lane_count = 2 }, /* 1620.0   30.00 */
			{ .rate = 1350000, .lane_count = 1 }, /* 1632.0   30.22 */
			{ .rate =  540000, .lane_count = 4 }, /* 2160.0   40.00 */
			{ .rate = 1000000, .lane_count = 2 }, /* 2417.8   44.77 */
			{ .rate = 2000000, .lane_count = 1 }, /* 2417.8   44.77 */
			{ .rate =  810000, .lane_count = 4 }, /* 3240.0   60.00 */
			{ .rate = 1350000, .lane_count = 2 }, /* 3264.0   60.44 */
			{ .rate = 1000000, .lane_count = 4 }, /* 4835.6   89.55 */
			{ .rate = 2000000, .lane_count = 2 }, /* 4835.6   89.55 */
			{ .rate = 1350000, .lane_count = 4 }, /* 6527.9  120.89 */
			{ .rate = 2000000, .lane_count = 4 }, /* 9671.1  179.09 */
		},
		.size = LINK_TEST_NUM_CONFIGS(ARRAY_SIZE(standard_dp_link_rates),
					      LINK_TEST_MAX_LANE_COUNT),
	},
	[INTEL_DP_LINK_CAPS_ORDER_KEY_RATE_LANE] = {
		.entries = {
			{ .rate = 162000,  .lane_count = 1 },
			{ .rate = 162000,  .lane_count = 2 },
			{ .rate = 162000,  .lane_count = 4 },

			{ .rate = 270000,  .lane_count = 1 },
			{ .rate = 270000,  .lane_count = 2 },
			{ .rate = 270000,  .lane_count = 4 },

			{ .rate = 540000,  .lane_count = 1 },
			{ .rate = 540000,  .lane_count = 2 },
			{ .rate = 540000,  .lane_count = 4 },

			{ .rate = 810000,  .lane_count = 1 },
			{ .rate = 810000,  .lane_count = 2 },
			{ .rate = 810000,  .lane_count = 4 },

			{ .rate = 1000000, .lane_count = 1 },
			{ .rate = 1000000, .lane_count = 2 },
			{ .rate = 1000000, .lane_count = 4 },

			{ .rate = 1350000, .lane_count = 1 },
			{ .rate = 1350000, .lane_count = 2 },
			{ .rate = 1350000, .lane_count = 4 },

			{ .rate = 2000000, .lane_count = 1 },
			{ .rate = 2000000, .lane_count = 2 },
			{ .rate = 2000000, .lane_count = 4 },
		},
		.size = LINK_TEST_NUM_CONFIGS(ARRAY_SIZE(standard_dp_link_rates),
					      LINK_TEST_MAX_LANE_COUNT),
	},
	[INTEL_DP_LINK_CAPS_ORDER_KEY_LANE_RATE] = {
		.entries = {
			{ .rate = 162000,  .lane_count = 1 },
			{ .rate = 270000,  .lane_count = 1 },
			{ .rate = 540000,  .lane_count = 1 },
			{ .rate = 810000,  .lane_count = 1 },
			{ .rate = 1000000, .lane_count = 1 },
			{ .rate = 1350000, .lane_count = 1 },
			{ .rate = 2000000, .lane_count = 1 },

			{ .rate = 162000,  .lane_count = 2 },
			{ .rate = 270000,  .lane_count = 2 },
			{ .rate = 540000,  .lane_count = 2 },
			{ .rate = 810000,  .lane_count = 2 },
			{ .rate = 1000000, .lane_count = 2 },
			{ .rate = 1350000, .lane_count = 2 },
			{ .rate = 2000000, .lane_count = 2 },

			{ .rate = 162000,  .lane_count = 4 },
			{ .rate = 270000,  .lane_count = 4 },
			{ .rate = 540000,  .lane_count = 4 },
			{ .rate = 810000,  .lane_count = 4 },
			{ .rate = 1000000, .lane_count = 4 },
			{ .rate = 1350000, .lane_count = 4 },
			{ .rate = 2000000, .lane_count = 4 },
		},
		.size = LINK_TEST_NUM_CONFIGS(ARRAY_SIZE(standard_dp_link_rates),
					      LINK_TEST_MAX_LANE_COUNT),
	},
};

static bool link_configs_match(const struct intel_dp_link_config *a,
			       const struct intel_dp_link_config *b)
{
	return a->rate == b->rate && a->lane_count == b->lane_count;
}

static const struct intel_dp_link_caps_order config_orders[] = {
	{
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_BW,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_ASC,
	}, {
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_BW,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_DESC,
	}, {
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_RATE_LANE,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_ASC,
	}, {
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_RATE_LANE,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_DESC,
	}, {
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_LANE_RATE,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_ASC,
	}, {
		.key = INTEL_DP_LINK_CAPS_ORDER_KEY_LANE_RATE,
		.dir = INTEL_DP_LINK_CAPS_ORDER_DIR_DESC,
	}
};

static const struct link_config_set *
link_caps_config_order_key_to_set(struct kunit *test, enum intel_dp_link_caps_order_key key)
{
	return &standard_dp_link_configs[key];
}

/*
 * TEST: Baseline with fixed reference table
 * -----------------------------------------
 * Verify the link_caps config iterator using fixed standard DP config tables.
 */
static void baseline_test_for_order(struct kunit *test,
				    struct intel_dp_link_caps *link_caps,
				    struct intel_dp_link_caps_order config_order)
{
	struct test_ctx *ctx = test->priv;
	const struct link_config_set *config_set =
		link_caps_config_order_key_to_set(test, config_order.key);
	const struct intel_dp_link_caps_test_ops *ops = ctx->link_caps_ops;
	struct intel_dp_link_config iter_config;
	struct intel_dp_link_caps_iter iter;
	int pos = 0;

	ops->iter_start(&iter, link_caps, config_order, INTEL_DP_LINK_CAPS_FILTER_ALL);
	for_each_dp_link_config(&iter, &iter_config) {
		int idx = pos;

		if (config_order.dir == INTEL_DP_LINK_CAPS_ORDER_DIR_DESC)
			idx = config_set->size - idx - 1;

		KUNIT_EXPECT_TRUE(test, link_configs_match(&iter_config,
							   &config_set->entries[idx]));

		pos++;
	}
	ops->iter_end(&iter);
}

static void intel_dp_link_caps_test_baseline(struct kunit *test)
{
	struct test_ctx *ctx = test->priv;
	struct intel_dp_link_caps *link_caps = ctx->dev.dig_port.dp.link.caps;
	const struct intel_dp_link_caps_test_ops *ops =
		ctx->link_caps_ops;
	int i;

	ops->update(link_caps,
		    standard_dp_link_rates, LINK_TEST_NUM_STANDARD_RATES,
		    LINK_TEST_MAX_LANE_COUNT,
		    true);

	for (i = 0; i < ARRAY_SIZE(config_orders); i++)
		baseline_test_for_order(test, link_caps, config_orders[i]);
}

static struct kunit_case intel_dp_link_test_cases[] = {
	KUNIT_CASE(intel_dp_link_caps_test_baseline),

	{}
};

static struct test_ctx test_ctx;

static int intel_dp_link_test_init(struct kunit *test)
{
	struct intel_digital_port *dig_port;
	struct intel_encoder *encoder;
	struct intel_dp *intel_dp;

	/* Reset the dev state for each test. */
	memset(&test_ctx.dev, 0, sizeof(test_ctx.dev));

	test_ctx.dev.generic_device.drm.dev = &test_ctx.dev.device;

	test_ctx.dev.display.drm = &test_ctx.dev.generic_device.drm;
	test_ctx.dev.generic_device.display = &test_ctx.dev.display;

	encoder = &test_ctx.dev.dig_port.base;
	encoder->base.dev = &test_ctx.dev.generic_device.drm;

	dig_port = &test_ctx.dev.dig_port;
	dig_port->base.type = INTEL_OUTPUT_DP;

	test_ctx.dev.connector.encoder = encoder;

	intel_dp = &dig_port->dp;
	intel_dp->attached_connector = &test_ctx.dev.connector;

	intel_dp->link.caps = test_ctx.link_caps_ops->init(intel_dp);

	test->priv = &test_ctx;

	return 0;
}

static void intel_dp_link_test_exit(struct kunit *test)
{
	struct test_ctx *ctx = test->priv;

	ctx->link_caps_ops->cleanup(ctx->dev.dig_port.dp.link.caps);
}

static int intel_dp_link_test_suite_init(struct kunit_suite *test_suite)
{
#ifdef I915
	test_ctx.link_caps_ops = &i915_display_dp_link_caps_test_ops;
	test_ctx.link_training_ops = &i915_display_dp_link_training_test_ops;
#else
	test_ctx.link_caps_ops = &intel_display_dp_link_caps_test_ops;
	test_ctx.link_training_ops = &intel_display_dp_link_training_test_ops;
#endif
	prandom_seed_state(&test_ctx.rnd, 0);

	return 0;
}

static struct kunit_suite intel_dp_link_test_suite = {
	.name = "intel_dp_link",
	.suite_init = intel_dp_link_test_suite_init,
	.init = intel_dp_link_test_init,
	.exit = intel_dp_link_test_exit,
	.test_cases = intel_dp_link_test_cases,
};

kunit_test_suites(&intel_dp_link_test_suite);

MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL and additional rights");
MODULE_DESCRIPTION("Intel DP link KUnit tests");
