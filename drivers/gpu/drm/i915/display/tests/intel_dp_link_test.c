// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <kunit/test.h>

struct test_ctx {
};

static struct kunit_case intel_dp_link_test_cases[] = {
	{}
};

static struct test_ctx test_ctx;

static int intel_dp_link_test_init(struct kunit *test)
{
	test->priv = &test_ctx;

	return 0;
}

static void intel_dp_link_test_exit(struct kunit *test)
{
}

static int intel_dp_link_test_suite_init(struct kunit_suite *test_suite)
{
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
