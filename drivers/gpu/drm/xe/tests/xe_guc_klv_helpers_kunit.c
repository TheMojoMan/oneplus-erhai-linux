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

static u32 str_klv_size(const char *string)
{
	return GUC_KLV_LEN_MIN + to_num_dwords(strlen(string) + 1);
}

static void test_encode_string(struct kunit *test)
{
	size_t longest_str = to_num_bytes(FIELD_MAX(GUC_KLV_0_LEN)) - 1;
	u32 avail = GUC_KLV_LEN_MIN + FIELD_MAX(GUC_KLV_0_LEN) + 1;
	const char *string = "abcdefghijklmnopqrstvwxyz";
	u16 key = TEST_KEY;
	u32 *klvs;
	u32 *next;
	char *buf;
	u32 n;

	klvs = kunit_kcalloc(test, avail, sizeof(u32), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, klvs);

	buf = kunit_kzalloc(test, longest_str + 2, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	/* empty string, no space, must fail */
	for (n = 0; n < str_klv_size(""); n++) {
		klvs[0] = TEST_PAD;
		KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC),
				    xe_guc_klv_encode_string(klvs, n, key, ""));
		KUNIT_EXPECT_EQ(test, klvs[0], TEST_PAD);
	}

	/* empty string, must pass */
	KUNIT_EXPECT_PTR_EQ(test, klvs + str_klv_size(""),
			    xe_guc_klv_encode_string(klvs, str_klv_size(""), key, ""));
	KUNIT_EXPECT_PTR_EQ(test, klvs + str_klv_size(""),
			    xe_guc_klv_encode_string(klvs, avail, key, ""));

	/* demo string, no space, must fail */
	for (n = 0; n < str_klv_size(string); n++) {
		klvs[0] = TEST_PAD;
		KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-ENOSPC),
				    xe_guc_klv_encode_string(klvs, n, key, string));
		KUNIT_EXPECT_EQ(test, klvs[0], TEST_PAD);
	}

	/* different string len, must pass */
	for (n = 0; n <= strlen(string); n++) {
		strscpy(buf, string, n + 1);
		kunit_info(test, "%u: '%s'\n", n, buf);
		KUNIT_ASSERT_EQ(test, n, strlen(buf));
		memset32(klvs, TEST_PAD, avail);

		next = xe_guc_klv_encode_string(klvs, str_klv_size(buf), key, buf);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, next);
		KUNIT_EXPECT_PTR_EQ(test, next, klvs + str_klv_size(buf));
		KUNIT_EXPECT_STREQ_MSG(test, buf, (char *)(klvs + GUC_KLV_LEN_MIN), "n=%u", n);
		kunit_info(test, "%u: %*ph\n", n, (int)to_num_bytes(next - klvs), klvs);
		KUNIT_EXPECT_NE(test, *(next - 1), TEST_PAD);
		KUNIT_ASSERT_EQ(test, *next, TEST_PAD);

		/* bigger buf doesn't matter */
		KUNIT_EXPECT_PTR_EQ(test,
				    xe_guc_klv_encode_string(klvs, str_klv_size(buf), key, buf),
				    xe_guc_klv_encode_string(klvs, avail, key, buf));
	}

	/* don't crash if already failed */
	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-EROFS),
			    xe_guc_klv_encode_string(ERR_PTR(-EROFS), avail, key, ""));

	/* too long string, must fail */
	memset(buf, 'X', longest_str + 1);
	buf[longest_str + 1] = '\0';
	KUNIT_EXPECT_LT(test, longest_str, strlen(buf));
	KUNIT_EXPECT_PTR_EQ(test, ERR_PTR(-E2BIG),
			    xe_guc_klv_encode_string(klvs, avail, key, buf));

	/* longest string, should pass */
	buf[longest_str] = '\0';
	KUNIT_EXPECT_EQ(test, longest_str, strlen(buf));
	KUNIT_EXPECT_PTR_EQ(test, klvs + str_klv_size(buf),
			    xe_guc_klv_encode_string(klvs, avail, key, buf));
}

static struct kunit_case guc_klv_helpers_test_cases[] = {
	KUNIT_CASE(test_count),
	KUNIT_CASE(test_encode_u32),
	KUNIT_CASE(test_encode_u64),
	KUNIT_CASE(test_encode_string),
	{}
};

static struct kunit_suite guc_klv_helpers_suite = {
	.name = "guc_klv_helpers",
	.test_cases = guc_klv_helpers_test_cases,
};

kunit_test_suite(guc_klv_helpers_suite);
