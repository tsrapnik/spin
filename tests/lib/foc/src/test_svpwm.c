/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define VBUS     24.0f
#define MAX_DUTY 0.95f

ZTEST(foc_svpwm, test_zero_vector)
{
	struct foc_abc d = foc_svpwm((struct foc_ab){0}, VBUS, MAX_DUTY);

	zassert_equal(d.a, 0.5f);
	zassert_equal(d.b, 0.5f);
	zassert_equal(d.c, 0.5f);
}

ZTEST(foc_svpwm, test_vmax)
{
	/* max duty 1: classic linear limit Vbus / sqrt(3). */
	zassert_within(foc_svpwm_vmax(VBUS, 1.0f), VBUS / sqrtf(3.0f), 1e-5f);
	zassert_within(foc_svpwm_vmax(VBUS, 0.5f), 0.0f, 1e-6f);
}

ZTEST(foc_svpwm, test_linear_circle)
{
	float vmax = foc_svpwm_vmax(VBUS, MAX_DUTY);
	float hi_max = 0.0f;

	for (float th = -FOC_PI_F; th < FOC_PI_F; th += 0.01f) {
		struct foc_sincos sc = foc_sincos(th);
		struct foc_ab v = {.alpha = vmax * sc.cos, .beta = vmax * sc.sin};
		struct foc_abc d = foc_svpwm(v, VBUS, MAX_DUTY);
		float hi = fmaxf(fmaxf(d.a, d.b), d.c);
		float lo = fminf(fminf(d.a, d.b), d.c);
		struct foc_ab back = foc_duty_to_ab(d, VBUS);

		zassert_true(hi <= MAX_DUTY + 1e-6f, "th %f hi %f", (double)th, (double)hi);
		hi_max = fmaxf(hi_max, hi);
		zassert_within(0.5f * (hi + lo), 0.5f, 1e-6f);
		zassert_within(back.alpha, v.alpha, 1e-4f);
		zassert_within(back.beta, v.beta, 1e-4f);
	}

	/* The circle touches the limit at 30 deg + k * 60 deg. */
	zassert_within(hi_max, MAX_DUTY, 1e-4f, "hi_max %f", (double)hi_max);
}

ZTEST(foc_svpwm, test_overmodulation_clamps)
{
	float vmax = foc_svpwm_vmax(VBUS, MAX_DUTY);

	for (float th = -FOC_PI_F; th < FOC_PI_F; th += 0.05f) {
		struct foc_ab v = {.alpha = 3.0f * vmax * cosf(th), .beta = 3.0f * vmax * sinf(th)};
		struct foc_abc d = foc_svpwm(v, VBUS, MAX_DUTY);

		zassert_true(d.a >= 1.0f - MAX_DUTY && d.a <= MAX_DUTY);
		zassert_true(d.b >= 1.0f - MAX_DUTY && d.b <= MAX_DUTY);
		zassert_true(d.c >= 1.0f - MAX_DUTY && d.c <= MAX_DUTY);
	}
}

ZTEST(foc_svpwm, test_duty_to_ab_ignores_common_mode)
{
	struct foc_abc d = {.a = 0.7f, .b = 0.4f, .c = 0.5f};
	struct foc_abc e = {.a = 0.8f, .b = 0.5f, .c = 0.6f};
	struct foc_ab vd = foc_duty_to_ab(d, VBUS);
	struct foc_ab ve = foc_duty_to_ab(e, VBUS);

	/* v_a = (d_a - mean) * Vbus = (0.7 - 0.5333) * 24 = 4 V */
	zassert_within(vd.alpha, 4.0f, 1e-5f);
	zassert_within(vd.alpha, ve.alpha, 1e-5f);
	zassert_within(vd.beta, ve.beta, 1e-5f);
}

ZTEST_SUITE(foc_svpwm, NULL, NULL, NULL, NULL, NULL);
