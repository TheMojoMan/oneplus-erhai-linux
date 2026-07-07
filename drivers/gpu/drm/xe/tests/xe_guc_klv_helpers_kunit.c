// SPDX-License-Identifier: GPL-2.0 AND MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <kunit/test.h>
#include <kunit/test-bug.h>

#define TEST_KEY	(GUC_KLV_RESERVED_RANGE_START + 0x3de)
#define TEST_PAD	0xdeadbeef

static void test_count(struct kunit *test)
{
	u32 value = 0x12345678;
	u16 key = TEST_KEY;
	u32 klvs[] = {
		PREP_GUC_KLV(key + 0, 0),
		PREP_GUC_KLV(key + 1, 1), value,
		PREP_GUC_KLV(key + 2, 2), value, value,
		PREP_GUC_KLV(key + 3, 0),
		0, /* padding */
	};

	KUNIT_EXPECT_EQ(test, 0, xe_guc_klv_count(klvs, 0));
	KUNIT_EXPECT_EQ(test, 1, xe_guc_klv_count(klvs, 1));
	KUNIT_EXPECT_GT(test, 0, xe_guc_klv_count(klvs, 2));
	KUNIT_EXPECT_EQ(test, 2, xe_guc_klv_count(klvs, 3));
	KUNIT_EXPECT_GT(test, 0, xe_guc_klv_count(klvs, 4));
	KUNIT_EXPECT_GT(test, 0, xe_guc_klv_count(klvs, 5));
	KUNIT_EXPECT_EQ(test, 3, xe_guc_klv_count(klvs, 6));
	KUNIT_EXPECT_EQ(test, 4, xe_guc_klv_count(klvs, 7));

	/* 0 is treated as reserved KLV { KEY=0, LEN=0 } */
	KUNIT_EXPECT_EQ(test, 5, xe_guc_klv_count(klvs, 8));
}

static void test_encode_u32(struct kunit *test)
{
	u32 *fail = ERR_PTR(-ENOMEM);
	u32 value = 0x12345678;
	u16 key = TEST_KEY;
	u32 klvs[16];

	memset32(klvs, TEST_PAD, ARRAY_SIZE(klvs));

	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC), xe_guc_klv_encode_u32(klvs, 0, key, value));
	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC), xe_guc_klv_encode_u32(klvs, 1, key, value));

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, xe_guc_klv_encode_u32(klvs, 2, key, value));
	KUNIT_EXPECT_EQ(test, klvs[0], PREP_GUC_KLV(key, 1));
	KUNIT_EXPECT_EQ(test, klvs[1], value);
	KUNIT_EXPECT_EQ(test, klvs[2], TEST_PAD);
	KUNIT_EXPECT_PTR_EQ(test, &klvs[2], xe_guc_klv_encode_u32(klvs, 2, key, value));
	KUNIT_EXPECT_PTR_EQ(test,
			    xe_guc_klv_encode_u32(klvs, 2, key, value),
			    xe_guc_klv_encode_u32(klvs, ARRAY_SIZE(klvs), key, value));

	KUNIT_ASSERT_PTR_EQ(test, fail, xe_guc_klv_encode_u32(fail, ARRAY_SIZE(klvs), key, value));
}

static void test_encode_u64(struct kunit *test)
{
	u64 value = 0x123456789abcdef0;
	u32 *fail = ERR_PTR(-ENOMEM);
	u16 key = TEST_KEY;
	u32 klvs[16];

	memset32(klvs, TEST_PAD, ARRAY_SIZE(klvs));

	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC), xe_guc_klv_encode_u64(klvs, 0, key, value));
	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC), xe_guc_klv_encode_u64(klvs, 1, key, value));
	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC), xe_guc_klv_encode_u64(klvs, 2, key, value));

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, xe_guc_klv_encode_u64(klvs, 3, key, value));
	KUNIT_EXPECT_EQ(test, klvs[0], PREP_GUC_KLV(key, 2));
	KUNIT_EXPECT_EQ(test, klvs[1], lower_32_bits(value));
	KUNIT_EXPECT_EQ(test, klvs[2], upper_32_bits(value));
	KUNIT_EXPECT_EQ(test, klvs[3], TEST_PAD);
	KUNIT_EXPECT_PTR_EQ(test, &klvs[3], xe_guc_klv_encode_u64(klvs, 3, key, value));
	KUNIT_EXPECT_PTR_EQ(test,
			    xe_guc_klv_encode_u64(klvs, 3, key, value),
			    xe_guc_klv_encode_u64(klvs, ARRAY_SIZE(klvs), key, value));

	KUNIT_ASSERT_PTR_EQ(test, fail, xe_guc_klv_encode_u64(fail, ARRAY_SIZE(klvs), key, value));
}

static struct kunit_case guc_klv_helpers_test_cases[] = {
	KUNIT_CASE(test_count),
	KUNIT_CASE(test_encode_u32),
	KUNIT_CASE(test_encode_u64),
	{}
};

static struct kunit_suite guc_klv_helpers_suite = {
	.name = "guc_klv_helpers",
	.test_cases = guc_klv_helpers_test_cases,
};

kunit_test_suite(guc_klv_helpers_suite);
