/*
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define TOL 1e-6f

ZTEST(foc_pi, test_proportional)
{
	struct foc_pi pi;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 2.0f, .ki = 0.0f}, 1e-3f);
	zassert_within(foc_pi_step(&pi, 0.25f, -10.0f, 10.0f), 0.5f, TOL);
	zassert_within(foc_pi_step(&pi, -1.0f, -10.0f, 10.0f), -2.0f, TOL);
	zassert_within(foc_pi_step(&pi, 100.0f, -10.0f, 10.0f), 10.0f, TOL);
}

ZTEST(foc_pi, test_integral)
{
	struct foc_pi pi;
	float u = 0.0f;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.0f, .ki = 10.0f}, 0.01f);
	for (int i = 0; i < 5; i++) {
		u = foc_pi_step(&pi, 1.0f, -10.0f, 10.0f);
	}
	zassert_within(u, 0.5f, TOL);
	zassert_within(pi.integ, 0.5f, TOL);
}

ZTEST(foc_pi, test_anti_windup)
{
	struct foc_pi pi;
	float u;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.1f, .ki = 100.0f}, 1e-3f);

	/* Integrate up to the limit, then stay saturated for a long time. */
	for (int i = 0; i < 1000; i++) {
		u = foc_pi_step(&pi, 5.0f, -1.0f, 1.0f);
		zassert_true(pi.integ <= 1.0f);
	}
	zassert_equal(u, 1.0f);

	/* Error reverses: the output must leave the limit at once. */
	u = foc_pi_step(&pi, -0.5f, -1.0f, 1.0f);
	zassert_true(u < 1.0f, "u %f", (double)u);
	zassert_within(u, -0.05f + pi.integ, TOL);
}

/* An integration step that overshoots the limit must still reach it. */
ZTEST(foc_pi, test_integrator_reaches_limit)
{
	struct foc_pi pi;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.0f, .ki = 200.0f}, 1e-3f);
	foc_pi_reset(&pi, 0.9f);
	zassert_equal(foc_pi_step(&pi, 1.0f, -1.0f, 1.0f), 1.0f);
	zassert_equal(pi.integ, 1.0f);

	foc_pi_reset(&pi, -0.9f);
	zassert_equal(foc_pi_step(&pi, -1.0f, -1.0f, 1.0f), -1.0f);
	zassert_equal(pi.integ, -1.0f);

	/* With some kp: output lands on the limit, integrator = hi - p. */
	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.25f, .ki = 500.0f}, 1e-3f);
	foc_pi_reset(&pi, 0.5f);
	zassert_within(foc_pi_step(&pi, 1.0f, -1.0f, 1.0f), 1.0f, 1e-6f);
	zassert_within(pi.integ, 0.75f, 1e-6f);
}

ZTEST(foc_pi, test_shrinking_limit)
{
	struct foc_pi pi;
	float u;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.0f, .ki = 1.0f}, 1e-3f);
	foc_pi_reset(&pi, 0.8f);
	u = foc_pi_step(&pi, 0.0f, -0.5f, 0.5f);
	zassert_equal(u, 0.5f);
	zassert_equal(pi.integ, 0.5f);
}

ZTEST(foc_pi, test_reset_preload)
{
	struct foc_pi pi;

	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 1.0f, .ki = 1.0f}, 1e-3f);
	foc_pi_reset(&pi, 0.3f);
	zassert_within(foc_pi_step(&pi, 0.0f, -1.0f, 1.0f), 0.3f, TOL);
}

ZTEST_SUITE(foc_pi, NULL, NULL, NULL, NULL, NULL);
