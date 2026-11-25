/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

ZTEST(foc_util, test_clampf)
{
	zassert_equal(foc_clampf(0.5f, 0.0f, 1.0f), 0.5f);
	zassert_equal(foc_clampf(-1.0f, 0.0f, 1.0f), 0.0f);
	zassert_equal(foc_clampf(2.0f, 0.0f, 1.0f), 1.0f);
	zassert_equal(foc_clampf(1.0f, 0.0f, 1.0f), 1.0f);
	zassert_equal(foc_clampf(NAN, -1.0f, 1.0f), -1.0f);
}

ZTEST_SUITE(foc_util, NULL, NULL, NULL, NULL, NULL);
