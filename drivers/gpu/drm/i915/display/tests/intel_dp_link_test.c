// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <kunit/test.h>

#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/prandom.h>
#include <linux/random.h>

#include <drm/display/drm_dp_helper.h>

#include <drm/intel/display_member.h>

#include "intel_connector.h"
#include "intel_display_core.h"
#include "intel_display_types.h"

struct test_ctx {
	struct {
		struct intel_display display;
		struct device device;
		struct __intel_generic_device generic_device;

		struct intel_connector connector;
		struct intel_digital_port dig_port;

		struct intel_crtc_state crtc_state;
	} dev;

	struct rnd_state rnd;
};

static struct kunit_case intel_dp_link_test_cases[] = {
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

	test->priv = &test_ctx;

	return 0;
}

static void intel_dp_link_test_exit(struct kunit *test)
{
}

static int intel_dp_link_test_suite_init(struct kunit_suite *test_suite)
{
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
