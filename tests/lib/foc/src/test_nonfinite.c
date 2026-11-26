/*
 * SPDX-License-Identifier: MIT
 *
 * Non-finite and extreme inputs: every function that keeps state, sets a
 * limit or produces duties must return finite values in its documented range.
 */

#include <float.h>
#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

static const float bad[] = {NAN,   -NAN,   INFINITY, -INFINITY, FLT_MAX, -FLT_MAX,
			    1e30f, -1e30f, 3001.0f,  -3001.0f,  FLT_MIN, -0.0f};

#define N_BAD ARRAY_SIZE(bad)

static bool in_range(float x, float lo, float hi)
{
	return isfinite(x) && x >= lo && x <= hi;
}

ZTEST(foc_nonfinite, test_sanitizef)
{
	zassert_equal(foc_sanitizef(NAN, 2.0f), 0.0f);
	zassert_equal(foc_sanitizef(INFINITY, 2.0f), 2.0f);
	zassert_equal(foc_sanitizef(-INFINITY, 2.0f), -2.0f);
	zassert_equal(foc_sanitizef(1.5f, 2.0f), 1.5f);
}

ZTEST(foc_nonfinite, test_wrap_pi)
{
	for (size_t i = 0; i < N_BAD; i++) {
		float r = foc_wrap_pi(bad[i]);

		zassert_true(in_range(r, -FOC_PI_F, FOC_PI_F), "x %e -> %e", (double)bad[i],
			     (double)r);
	}
	zassert_equal(foc_wrap_pi(NAN), 0.0f);
	zassert_equal(foc_wrap_pi(INFINITY), 0.0f);
}

ZTEST(foc_nonfinite, test_sincos)
{
	for (size_t i = 0; i < N_BAD; i++) {
		struct foc_sincos sc = foc_sincos(bad[i]);

		zassert_true(in_range(sc.sin, -1.0f, 1.0f), "x %e", (double)bad[i]);
		zassert_true(in_range(sc.cos, -1.0f, 1.0f), "x %e", (double)bad[i]);
	}

	struct foc_sincos sc = foc_sincos(NAN);

	zassert_equal(sc.sin, 0.0f);
	zassert_equal(sc.cos, 1.0f);
}

ZTEST(foc_nonfinite, test_sincos_bounded)
{
	for (float x = -FOC_SINCOS_MAX_ARG; x <= FOC_SINCOS_MAX_ARG; x += 0.0137f) {
		struct foc_sincos sc = foc_sincos(x);

		zassert_true(fabsf(sc.sin) <= 1.0f && fabsf(sc.cos) <= 1.0f, "x %f", (double)x);
	}
}

ZTEST(foc_nonfinite, test_limit_circle)
{
	for (size_t i = 0; i < N_BAD; i++) {
		for (size_t j = 0; j < N_BAD; j++) {
			struct foc_dq v =
				foc_limit_circle((struct foc_dq){.d = bad[i], .q = bad[j]}, 2.0f);

			zassert_true(in_range(v.d, -2.0f, 2.0f));
			zassert_true(in_range(v.q, -2.0f, 2.0f));
			zassert_true(v.d * v.d + v.q * v.q <= 4.0f * (1.0f + 1e-6f));
		}
		/* Bad vmax: result finite; NaN or negative vmax means 0. */
		struct foc_dq v = foc_limit_circle((struct foc_dq){.d = 1.0f, .q = 1.0f}, bad[i]);

		zassert_true(isfinite(v.d) && isfinite(v.q), "vmax %e", (double)bad[i]);
	}
	struct foc_dq v = foc_limit_circle((struct foc_dq){.d = NAN, .q = NAN}, 2.0f);

	zassert_equal(v.d, 0.0f);
	zassert_equal(v.q, 0.0f);
	v = foc_limit_circle((struct foc_dq){.d = 1.0f, .q = 1.0f}, NAN);
	zassert_equal(v.d, 0.0f);
	zassert_equal(v.q, 0.0f);
}

ZTEST(foc_nonfinite, test_q_limit)
{
	for (size_t i = 0; i < N_BAD; i++) {
		for (size_t j = 0; j < N_BAD; j++) {
			float q = foc_q_limit(bad[i], bad[j]);

			zassert_true(in_range(q, 0.0f, FLT_MAX));
		}
	}
}

ZTEST(foc_nonfinite, test_svpwm_vmax)
{
	for (size_t i = 0; i < N_BAD; i++) {
		for (size_t j = 0; j < N_BAD; j++) {
			zassert_true(in_range(foc_svpwm_vmax(bad[i], bad[j]), 0.0f, FLT_MAX));
		}
		zassert_true(in_range(foc_svpwm_vmax(bad[i], 0.95f), 0.0f, FLT_MAX));
		zassert_true(in_range(foc_svpwm_vmax(24.0f, bad[i]), 0.0f, FLT_MAX));
	}
	zassert_equal(foc_svpwm_vmax(NAN, 0.95f), 0.0f);
	zassert_equal(foc_svpwm_vmax(-24.0f, 0.95f), 0.0f);
}

ZTEST(foc_nonfinite, test_svpwm)
{
	const float good[] = {0.0f, 3.0f, -5.0f};

	for (size_t i = 0; i < N_BAD + 3; i++) {
		float a = i < N_BAD ? bad[i] : good[i - N_BAD];

		for (size_t j = 0; j < N_BAD + 3; j++) {
			float b = j < N_BAD ? bad[j] : good[j - N_BAD];
			struct foc_ab v = {.alpha = a, .beta = b};
			const float vb[] = {24.0f, NAN, 0.0f, -24.0f, INFINITY, FLT_MIN};
			const float md[] = {0.95f, NAN, INFINITY, -1.0f, 2.0f};

			for (size_t k = 0; k < ARRAY_SIZE(vb); k++) {
				for (size_t m = 0; m < ARRAY_SIZE(md); m++) {
					struct foc_abc d = foc_svpwm(v, vb[k], md[m]);

					zassert_true(in_range(d.a, 0.0f, 1.0f) &&
							     in_range(d.b, 0.0f, 1.0f) &&
							     in_range(d.c, 0.0f, 1.0f),
						     "v %e %e vbus %e md %e", (double)a, (double)b,
						     (double)vb[k], (double)md[m]);
				}
			}
		}
	}
}

ZTEST(foc_nonfinite, test_svpwm_zero_voltage_cases)
{
	struct foc_ab v = {.alpha = 5.0f, .beta = -3.0f};
	const float vb[] = {NAN, 0.0f, -24.0f, 0.05f};
	struct foc_abc d;

	for (size_t k = 0; k < ARRAY_SIZE(vb); k++) {
		d = foc_svpwm(v, vb[k], 0.95f);
		zassert_true(d.a == 0.5f && d.b == 0.5f && d.c == 0.5f, "vbus %e", (double)vb[k]);
	}
	d = foc_svpwm(v, 24.0f, NAN);
	zassert_true(d.a == 0.5f && d.b == 0.5f && d.c == 0.5f);
	d = foc_svpwm((struct foc_ab){.alpha = NAN, .beta = NAN}, 24.0f, 0.95f);
	zassert_true(d.a == 0.5f && d.b == 0.5f && d.c == 0.5f);
}

ZTEST(foc_nonfinite, test_pi)
{
	struct foc_pi pi;

	for (size_t i = 0; i < N_BAD; i++) {
		foc_pi_init(&pi, (struct foc_pi_gains){.kp = 2.0f, .ki = 100.0f}, 1e-3f);
		foc_pi_reset(&pi, 0.25f);
		for (int n = 0; n < 3; n++) {
			float u = foc_pi_step(&pi, bad[i], -1.0f, 1.0f);

			zassert_true(in_range(u, -1.0f, 1.0f), "err %e", (double)bad[i]);
			zassert_true(in_range(pi.integ, -1.0f, 1.0f));
		}
	}

	/* NaN error holds the integrator. */
	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 2.0f, .ki = 100.0f}, 1e-3f);
	foc_pi_reset(&pi, 0.25f);
	zassert_equal(foc_pi_step(&pi, NAN, -1.0f, 1.0f), 0.25f);
	zassert_equal(pi.integ, 0.25f);

	/* Inf error saturates in its own direction, also with kp = 0. */
	foc_pi_init(&pi, (struct foc_pi_gains){.kp = 0.0f, .ki = 100.0f}, 1e-3f);
	zassert_equal(foc_pi_step(&pi, INFINITY, -1.0f, 1.0f), 1.0f);
	zassert_equal(foc_pi_step(&pi, -INFINITY, -1.0f, 1.0f), -1.0f);
}

ZTEST(foc_nonfinite, test_pi_init_and_reset)
{
	struct foc_pi pi;

	for (size_t i = 0; i < N_BAD; i++) {
		foc_pi_init(&pi, (struct foc_pi_gains){.kp = bad[i], .ki = bad[i]}, bad[i]);
		zassert_true(in_range(pi.kp, 0.0f, FLT_MAX));
		zassert_true(in_range(pi.ki_ts, 0.0f, FLT_MAX));
		zassert_true(in_range(foc_pi_step(&pi, 1.0f, -1.0f, 1.0f), -1.0f, 1.0f));

		foc_pi_reset(&pi, bad[i]);
		zassert_true(in_range(pi.integ, -FLT_MAX, FLT_MAX));
	}
	foc_pi_reset(&pi, NAN);
	zassert_equal(pi.integ, 0.0f);
}

ZTEST(foc_nonfinite, test_gains)
{
	for (size_t i = 0; i < N_BAD; i++) {
		struct foc_pi_gains g[] = {
			foc_gains_current(bad[i], 1e-3f, 1000.0f),
			foc_gains_current(1.0f, bad[i], 1000.0f),
			foc_gains_current(1.0f, 1e-3f, bad[i]),
			foc_gains_speed(bad[i], 0.18f, 10.0f),
			foc_gains_speed(6e-6f, bad[i], 10.0f),
			foc_gains_speed(6e-6f, 0.18f, bad[i]),
		};

		for (size_t k = 0; k < ARRAY_SIZE(g); k++) {
			zassert_true(in_range(g[k].kp, 0.0f, FLT_MAX), "in %e k %zu",
				     (double)bad[i], k);
			zassert_true(in_range(g[k].ki, 0.0f, FLT_MAX), "in %e k %zu",
				     (double)bad[i], k);
		}
	}
	struct foc_pi_gains g = foc_gains_speed(6e-6f, 0.0f, 10.0f);

	zassert_true(g.kp == 0.0f && g.ki == 0.0f);
}

ZTEST(foc_nonfinite, test_current_step)
{
	const float good[] = {0.0f, 1.0f, -2.0f};
	struct foc_current c;

	for (size_t i = 0; i < N_BAD + 3; i++) {
		float x = i < N_BAD ? bad[i] : good[i - N_BAD];
		/* Each input bad in turn, the others good. */
		struct {
			struct foc_abc i;
			float vbus, theta, omega;
			struct foc_dq ref;
		} cases[] = {
			{{x, 0.1f, -0.1f}, 24.0f, 0.5f, 100.0f, {0.0f, 1.0f}},
			{{0.1f, x, x}, 24.0f, 0.5f, 100.0f, {0.0f, 1.0f}},
			{{0.1f, 0.0f, -0.1f}, x, 0.5f, 100.0f, {0.0f, 1.0f}},
			{{0.1f, 0.0f, -0.1f}, 24.0f, x, 100.0f, {0.0f, 1.0f}},
			{{0.1f, 0.0f, -0.1f}, 24.0f, 0.5f, x, {0.0f, 1.0f}},
			{{0.1f, 0.0f, -0.1f}, 24.0f, 0.5f, 100.0f, {x, x}},
		};

		for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
			foc_current_init(&c, (struct foc_pi_gains){.kp = 5.0f, .ki = 1e4f}, 50e-6f,
					 0.95f, 75e-6f);
			for (int n = 0; n < 3; n++) {
				struct foc_abc d = foc_current_step(&c, cases[k].i, cases[k].vbus,
								    cases[k].theta, cases[k].omega,
								    cases[k].ref);

				zassert_true(in_range(d.a, 0.05f, 0.95f) &&
						     in_range(d.b, 0.05f, 0.95f) &&
						     in_range(d.c, 0.05f, 0.95f),
					     "x %e case %zu", (double)x, k);
				zassert_true(isfinite(c.v.d) && isfinite(c.v.q));
				zassert_true(isfinite(c.d.integ) && isfinite(c.q.integ));
			}
		}
	}
}

ZTEST_SUITE(foc_nonfinite, NULL, NULL, NULL, NULL, NULL);
