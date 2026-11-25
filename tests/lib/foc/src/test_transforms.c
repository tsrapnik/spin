/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define TOL 1e-6f

static struct foc_abc balanced(float amp, float theta)
{
	return (struct foc_abc){
		.a = amp * cosf(theta),
		.b = amp * cosf(theta - 2.0f * FOC_PI_F / 3.0f),
		.c = amp * cosf(theta + 2.0f * FOC_PI_F / 3.0f),
	};
}

ZTEST(foc_transforms, test_clampf)
{
	zassert_equal(foc_clampf(0.5f, 0.0f, 1.0f), 0.5f);
	zassert_equal(foc_clampf(-1.0f, 0.0f, 1.0f), 0.0f);
	zassert_equal(foc_clampf(2.0f, 0.0f, 1.0f), 1.0f);
	zassert_equal(foc_clampf(1.0f, 0.0f, 1.0f), 1.0f);
	zassert_equal(foc_clampf(NAN, -1.0f, 1.0f), -1.0f);
}

ZTEST(foc_transforms, test_sincos_exact_points)
{
	struct foc_sincos sc = foc_sincos(0.0f);

	zassert_equal(sc.sin, 0.0f);
	zassert_equal(sc.cos, 1.0f);
}

ZTEST(foc_transforms, test_sincos_accuracy)
{
	double max_err = 0.0;

	for (int i = -200000; i <= 200000; i++) {
		float x = (float)i * 1e-4f; /* -20 .. 20 rad */
		struct foc_sincos sc = foc_sincos(x);

		max_err = fmax(max_err, fabs((double)sc.sin - sin((double)x)));
		max_err = fmax(max_err, fabs((double)sc.cos - cos((double)x)));
	}
	zassert_true(max_err < 2e-7, "max error %e", max_err);

	for (float x = -3000.0f; x <= 3000.0f; x += 0.37f) {
		struct foc_sincos sc = foc_sincos(x);

		max_err = fmax(max_err, fabs((double)sc.sin - sin((double)x)));
		max_err = fmax(max_err, fabs((double)sc.cos - cos((double)x)));
	}
	zassert_true(max_err < (double)TOL, "max error %e", max_err);
}

ZTEST(foc_transforms, test_wrap_pi)
{
	zassert_within(foc_wrap_pi(0.0f), 0.0f, TOL);
	zassert_within(foc_wrap_pi(3.0f), 3.0f, TOL);
	zassert_within(foc_wrap_pi(-3.0f), -3.0f, TOL);
	zassert_within(foc_wrap_pi(FOC_PI_F + 0.1f), -FOC_PI_F + 0.1f, TOL);
	zassert_within(foc_wrap_pi(-FOC_PI_F - 0.1f), FOC_PI_F - 0.1f, TOL);
	zassert_within(foc_wrap_pi(5.0f * FOC_2PI_F + 1.0f), 1.0f, 1e-5f);
}

ZTEST(foc_transforms, test_clarke_balanced)
{
	for (float th = -FOC_PI_F; th < FOC_PI_F; th += 0.1f) {
		struct foc_ab v = foc_clarke(balanced(2.0f, th));

		zassert_within(v.alpha, 2.0f * cosf(th), TOL * 4);
		zassert_within(v.beta, 2.0f * sinf(th), TOL * 4);
	}
}

ZTEST(foc_transforms, test_clarke_ignores_common_mode)
{
	struct foc_abc x = {.a = 1.5f, .b = -0.25f, .c = 0.75f};
	struct foc_abc y = {.a = x.a + 7.0f, .b = x.b + 7.0f, .c = x.c + 7.0f};
	struct foc_ab vx = foc_clarke(x);
	struct foc_ab vy = foc_clarke(y);
	struct foc_abc back = foc_inv_clarke(vx);
	float mean = (x.a + x.b + x.c) / 3.0f;

	zassert_within(vx.alpha, vy.alpha, TOL * 10);
	zassert_within(vx.beta, vy.beta, TOL * 10);
	zassert_within(back.a, x.a - mean, TOL);
	zassert_within(back.b, x.b - mean, TOL);
	zassert_within(back.c, x.c - mean, TOL);
}

ZTEST(foc_transforms, test_park_aligned)
{
	/* A vector rotating with theta is constant (A, 0) in dq. */
	for (float th = -FOC_PI_F; th < FOC_PI_F; th += 0.1f) {
		struct foc_dq dq = foc_park(foc_clarke(balanced(3.0f, th)), foc_sincos(th));

		zassert_within(dq.d, 3.0f, TOL * 10);
		zassert_within(dq.q, 0.0f, TOL * 10);
	}
}

ZTEST(foc_transforms, test_park_q_leads_d)
{
	struct foc_sincos sc = foc_sincos(0.3f);
	struct foc_ab ab = foc_inv_park((struct foc_dq){.d = 0.0f, .q = 1.0f}, sc);

	zassert_within(ab.alpha, cosf(0.3f + FOC_PI_F / 2.0f), TOL);
	zassert_within(ab.beta, sinf(0.3f + FOC_PI_F / 2.0f), TOL);
}

ZTEST(foc_transforms, test_park_round_trip)
{
	struct foc_dq in = {.d = -0.7f, .q = 1.3f};

	for (float th = -4.0f; th < 4.0f; th += 0.25f) {
		struct foc_sincos sc = foc_sincos(th);
		struct foc_dq out = foc_park(foc_inv_park(in, sc), sc);

		zassert_within(out.d, in.d, TOL);
		zassert_within(out.q, in.q, TOL);
	}
}

ZTEST(foc_transforms, test_limit_circle)
{
	struct foc_dq v;

	v = foc_limit_circle((struct foc_dq){.d = 0.3f, .q = -0.4f}, 1.0f);
	zassert_equal(v.d, 0.3f);
	zassert_equal(v.q, -0.4f);

	v = foc_limit_circle((struct foc_dq){.d = 0.6f, .q = 5.0f}, 1.0f);
	zassert_within(v.d, 0.6f, TOL);
	zassert_within(v.q, 0.8f, TOL);

	v = foc_limit_circle((struct foc_dq){.d = -2.0f, .q = 2.0f}, 1.0f);
	zassert_within(v.d, -1.0f, TOL);
	zassert_within(v.q, 0.0f, TOL);

	zassert_equal(foc_q_limit(2.0f, 1.0f), 0.0f);
}

ZTEST_SUITE(foc_transforms, NULL, NULL, NULL, NULL, NULL);
