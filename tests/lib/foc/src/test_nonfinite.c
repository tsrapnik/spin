/*
 * SPDX-License-Identifier: MIT
 *
 * Non-finite and extreme inputs: every function that keeps state, sets a
 * limit or produces duties must return finite values in its documented range.
 */

#include <float.h>
#include <limits.h>
#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

/* NaN, Inf, overflow when squared or scaled, outside the sincos range, and
 * underflow: smallest normal and subnormal numbers.
 */
static const float bad[] = {NAN,   -NAN,   INFINITY,     -INFINITY,    FLT_MAX, -FLT_MAX,
			    1e30f, -1e30f, 3001.0f,      -3001.0f,     FLT_MIN, -FLT_MIN,
			    -0.0f, 1e20f,  FLT_TRUE_MIN, -FLT_TRUE_MIN};

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

/*
 * A bad speed only affects the angle advance: NaN counts as 0, so the
 * duties equal those for omega = 0. Also with t_adv = 0, where Inf * 0 is
 * NaN.
 */
ZTEST(foc_nonfinite, test_current_bad_speed_is_zero_advance)
{
	const struct foc_pi_gains g = {.kp = 5.0f, .ki = 1e4f};
	const struct foc_abc i = {.a = 0.1f, .b = 0.0f, .c = -0.1f};
	const struct foc_dq ref = {.d = 0.0f, .q = 1.0f};
	const float t_adv[] = {75e-6f, 0.0f};

	for (size_t t = 0; t < ARRAY_SIZE(t_adv); t++) {
		struct foc_current c0, c1;
		const float w_bad[] = {NAN, -NAN};

		for (size_t k = 0; k < ARRAY_SIZE(w_bad); k++) {
			foc_current_init(&c0, g, 50e-6f, 0.95f, t_adv[t]);
			foc_current_init(&c1, g, 50e-6f, 0.95f, t_adv[t]);

			struct foc_abc d0 = foc_current_step(&c0, i, 24.0f, 0.5f, 0.0f, ref);
			struct foc_abc d1 = foc_current_step(&c1, i, 24.0f, 0.5f, w_bad[k], ref);

			zassert_true(d0.a == d1.a && d0.b == d1.b && d0.c == d1.c,
				     "t_adv %e: %f %f %f vs %f %f %f", (double)t_adv[t],
				     (double)d0.a, (double)d0.b, (double)d0.c, (double)d1.a,
				     (double)d1.b, (double)d1.c);
		}

		/* +-Inf speed with t_adv = 0: no advance at all. */
		if (t_adv[t] == 0.0f) {
			foc_current_init(&c0, g, 50e-6f, 0.95f, 0.0f);
			foc_current_init(&c1, g, 50e-6f, 0.95f, 0.0f);

			struct foc_abc d0 = foc_current_step(&c0, i, 24.0f, 0.5f, 0.0f, ref);
			struct foc_abc d1 = foc_current_step(&c1, i, 24.0f, 0.5f, INFINITY, ref);

			zassert_true(d0.a == d1.a && d0.b == d1.b && d0.c == d1.c);
		}
	}
}

/*
 * The linear transforms propagate NaN and Inf on purpose (header comment):
 * a broken measurement must stay visible to the fault checks.
 */
ZTEST(foc_nonfinite, test_linear_transforms_propagate)
{
	const float nonfinite[] = {NAN, INFINITY, -INFINITY};
	struct foc_sincos sc = foc_sincos(0.3f);

	for (size_t i = 0; i < ARRAY_SIZE(nonfinite); i++) {
		float x = nonfinite[i];
		struct foc_ab ab = foc_clarke((struct foc_abc){.a = x, .b = 0.1f, .c = 0.2f});
		struct foc_abc abc = foc_inv_clarke((struct foc_ab){.alpha = x, .beta = 0.1f});
		struct foc_dq dq = foc_park((struct foc_ab){.alpha = x, .beta = 0.1f}, sc);
		struct foc_ab iab = foc_inv_park((struct foc_dq){.d = x, .q = 0.1f}, sc);
		struct foc_ab vab =
			foc_duty_to_ab((struct foc_abc){.a = x, .b = 0.5f, .c = 0.5f}, 24.0f);
		struct foc_ab vbus =
			foc_duty_to_ab((struct foc_abc){.a = 0.6f, .b = 0.5f, .c = 0.4f}, x);

		zassert_false(isfinite(ab.alpha), "clarke %e", (double)x);
		zassert_false(isfinite(abc.a), "inv_clarke %e", (double)x);
		zassert_false(isfinite(dq.d) && isfinite(dq.q), "park %e", (double)x);
		zassert_false(isfinite(iab.alpha) && isfinite(iab.beta), "inv_park %e", (double)x);
		zassert_false(isfinite(vab.alpha), "duty_to_ab duty %e", (double)x);
		zassert_false(isfinite(vbus.alpha), "duty_to_ab vbus %e", (double)x);
	}
	/* Finite overflow becomes Inf, also visible. */
	zassert_false(isfinite(foc_clarke((struct foc_abc){.a = FLT_MAX, .b = -FLT_MAX}).alpha));
}

ZTEST(foc_nonfinite, test_clampf_inf)
{
	zassert_equal(foc_clampf(INFINITY, -1.0f, 1.0f), 1.0f);
	zassert_equal(foc_clampf(-INFINITY, -1.0f, 1.0f), -1.0f);
	zassert_equal(foc_clampf(FLT_TRUE_MIN, 0.0f, 1.0f), FLT_TRUE_MIN);
}

/* Kt is a plain product; foc_gains_speed() rejects whatever it yields. */
ZTEST(foc_nonfinite, test_torque_constant_into_gains)
{
	const unsigned int pp[] = {0U, 1U, 11U, UINT_MAX};

	zassert_true(isnan(foc_torque_constant(NAN, 11)));
	for (size_t i = 0; i < N_BAD; i++) {
		for (size_t k = 0; k < ARRAY_SIZE(pp); k++) {
			struct foc_pi_gains g =
				foc_gains_speed(6e-6f, foc_torque_constant(bad[i], pp[k]), 10.0f);

			zassert_true(in_range(g.kp, 0.0f, FLT_MAX) && in_range(g.ki, 0.0f, FLT_MAX),
				     "psi %e pp %u", (double)bad[i], pp[k]);
		}
	}
}

/* Extreme gains, errors and finite limits: no overflow escapes. */
ZTEST(foc_nonfinite, test_pi_extreme)
{
	const float gains[] = {0.0f, 1.0f, 1e20f, FLT_MAX};
	const float lims[] = {0.0f, 1.0f, 1e30f, FLT_MAX};
	struct foc_pi pi;

	for (size_t a = 0; a < ARRAY_SIZE(gains); a++) {
		for (size_t b = 0; b < ARRAY_SIZE(gains); b++) {
			for (size_t l = 0; l < ARRAY_SIZE(lims); l++) {
				float lim = lims[l];

				foc_pi_init(&pi,
					    (struct foc_pi_gains){.kp = gains[a], .ki = gains[b]},
					    1.0f);
				for (size_t i = 0; i < N_BAD; i++) {
					float u = foc_pi_step(&pi, bad[i], -lim, lim);

					zassert_true(in_range(u, -lim, lim),
						     "kp %e ki %e lim %e err %e", (double)gains[a],
						     (double)gains[b], (double)lim, (double)bad[i]);
					zassert_true(in_range(pi.integ, -lim, lim));
				}
			}
		}
	}
}

ZTEST(foc_nonfinite, test_current_init)
{
	struct foc_current c;

	for (size_t i = 0; i < N_BAD; i++) {
		foc_current_init(&c, (struct foc_pi_gains){.kp = bad[i], .ki = bad[i]}, bad[i],
				 bad[i], bad[i]);
		zassert_true(in_range(c.max_duty, 0.5f, 1.0f), "%e", (double)bad[i]);
		zassert_true(in_range(c.t_adv, 0.0f, FLT_MAX));
		zassert_true(in_range(c.d.kp, 0.0f, FLT_MAX) && in_range(c.d.ki_ts, 0.0f, FLT_MAX));

		struct foc_abc d = foc_current_step(&c, (struct foc_abc){.a = 1.0f}, 24.0f, 0.5f,
						    100.0f, (struct foc_dq){.q = 1.0f});

		zassert_true(in_range(d.a, 0.0f, 1.0f) && in_range(d.b, 0.0f, 1.0f) &&
			     in_range(d.c, 0.0f, 1.0f));
	}
	foc_current_init(&c, (struct foc_pi_gains){0}, 1e-4f, INFINITY, INFINITY);
	zassert_equal(c.max_duty, 1.0f);
	zassert_equal(c.t_adv, 0.0f);
}

/* Every input bad at once, with extreme gains: duties and state stay finite. */
ZTEST(foc_nonfinite, test_current_step_all_bad)
{
	const float gains[] = {1.0f, FLT_MAX};
	struct foc_current c;

	for (size_t gi = 0; gi < ARRAY_SIZE(gains); gi++) {
		for (size_t i = 0; i < N_BAD; i++) {
			float x = bad[i];

			foc_current_init(&c,
					 (struct foc_pi_gains){.kp = gains[gi], .ki = gains[gi]},
					 50e-6f, 0.95f, 75e-6f);
			for (size_t j = 0; j < N_BAD; j++) {
				float y = bad[j];
				struct foc_abc d = foc_current_step(&c, (struct foc_abc){x, y, x},
								    y, x, y, (struct foc_dq){x, y});

				zassert_true(in_range(d.a, 0.05f, 0.95f) &&
						     in_range(d.b, 0.05f, 0.95f) &&
						     in_range(d.c, 0.05f, 0.95f),
					     "x %e y %e", (double)x, (double)y);
				zassert_true(isfinite(c.v.d) && isfinite(c.v.q));
				zassert_true(isfinite(c.d.integ) && isfinite(c.q.integ));
			}
		}
	}
}

static bool ab_in(struct foc_ab x, float lim)
{
	return in_range(x.alpha, -lim, lim) && in_range(x.beta, -lim, lim);
}

static bool obs_ok(const struct foc_obs *o)
{
	return ab_in(o->i, o->i_lim) && ab_in(o->e, o->v_lim) && ab_in(o->err, 2.0f * o->i_lim) &&
	       isfinite(o->a) && isfinite(o->b) && isfinite(o->g_i) && isfinite(o->g_e) &&
	       isfinite(o->ts_half);
}

/* Each init argument bad in turn: either a valid observer or an inert one. */
ZTEST(foc_nonfinite, test_obs_init)
{
	const float good[] = {3.6f, 1.5e-3f, 50e-6f, 2000.0f, 6.0f, 48.0f};

	for (size_t i = 0; i < N_BAD; i++) {
		for (size_t k = 0; k <= ARRAY_SIZE(good); k++) {
			float a[ARRAY_SIZE(good)];
			struct foc_obs o;

			for (size_t n = 0; n < ARRAY_SIZE(good); n++) {
				a[n] = (n == k || k == ARRAY_SIZE(good)) ? bad[i] : good[n];
			}
			foc_obs_init(&o, a[0], a[1], a[2], a[3], a[4], a[5]);
			zassert_true(obs_ok(&o), "x %e arg %zu", (double)bad[i], k);
			for (int n = 0; n < 3; n++) {
				struct foc_ab e = foc_obs_step(&o, (struct foc_ab){1.0f, -1.0f},
							       (struct foc_ab){5.0f, 2.0f}, 500.0f);

				zassert_true(ab_in(e, FLT_MAX) && obs_ok(&o), "x %e arg %zu",
					     (double)bad[i], k);
			}
		}
	}
	/* Inert observer: estimates stay 0. */
	struct foc_obs o;

	foc_obs_init(&o, 3.6f, 0.0f, 50e-6f, 2000.0f, 6.0f, 48.0f);
	struct foc_ab e =
		foc_obs_step(&o, (struct foc_ab){1.0f, 1.0f}, (struct foc_ab){5.0f, 0.0f}, 100.0f);

	zassert_true(e.alpha == 0.0f && e.beta == 0.0f && o.i.alpha == 0.0f);
}

/* Each step input bad in turn, then all at once; state stays in its limits. */
ZTEST(foc_nonfinite, test_obs_step)
{
	struct foc_obs o;

	for (size_t i = 0; i < N_BAD; i++) {
		float x = bad[i];
		struct {
			struct foc_ab i, v;
			float w;
		} cases[] = {
			{{x, 0.5f}, {5.0f, 2.0f}, 500.0f}, {{0.5f, x}, {5.0f, 2.0f}, 500.0f},
			{{1.0f, 0.5f}, {x, 2.0f}, 500.0f}, {{1.0f, 0.5f}, {5.0f, x}, 500.0f},
			{{1.0f, 0.5f}, {5.0f, 2.0f}, x},   {{x, x}, {x, x}, x},
		};

		for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
			foc_obs_init(&o, 3.6f, 1.5e-3f, 50e-6f, 2000.0f, 6.0f, 48.0f);
			for (int n = 0; n < 5; n++) {
				(void)foc_obs_step(&o, cases[k].i, cases[k].v, cases[k].w);
				zassert_true(obs_ok(&o), "x %e case %zu", (double)x, k);
			}
		}
		for (size_t j = 0; j < N_BAD; j++) {
			float y = bad[j];

			(void)foc_obs_step(&o, (struct foc_ab){x, y}, (struct foc_ab){y, x}, y);
			zassert_true(obs_ok(&o), "x %e y %e", (double)x, (double)y);
		}
	}
}

static bool pll_ok(const struct foc_pll *p)
{
	return in_range(p->theta, -FOC_PI_F, FOC_PI_F) && in_range(p->omega, -p->w_max, p->w_max) &&
	       in_range(p->err, -1.0f, 1.0f) && isfinite(p->f_err) && isfinite(p->pi.integ) &&
	       isfinite(p->e_prev.d) && isfinite(p->e_prev.q);
}

ZTEST(foc_nonfinite, test_pll_gains_init_reset)
{
	for (size_t i = 0; i < N_BAD; i++) {
		struct foc_pi_gains g = foc_gains_pll(bad[i]);
		struct foc_pll p;

		zassert_true(in_range(g.kp, 0.0f, FLT_MAX) && in_range(g.ki, 0.0f, FLT_MAX));

		const struct {
			struct foc_pi_gains g;
			float ts, w_max, e_min;
		} cases[] = {
			{{bad[i], bad[i]}, 50e-6f, 3000.0f, 0.1f},
			{foc_gains_pll(150.0f), bad[i], 3000.0f, 0.1f},
			{foc_gains_pll(150.0f), 50e-6f, bad[i], 0.1f},
			{foc_gains_pll(150.0f), 50e-6f, 3000.0f, bad[i]},
			{{bad[i], bad[i]}, bad[i], bad[i], bad[i]},
		};

		for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
			foc_pll_init(&p, cases[k].g, cases[k].ts, cases[k].w_max, cases[k].e_min);
			zassert_true(in_range(p.ts, 0.0f, FLT_MAX) &&
					     in_range(p.w_max, 0.0f, FLT_MAX) &&
					     in_range(p.e_min, FLT_MIN, FLT_MAX) &&
					     in_range(p.k_f_ts, 0.0f, FLT_MAX) &&
					     in_range(p.inv_ts, 0.0f, FLT_MAX),
				     "x %e case %zu", (double)bad[i], k);
			for (int n = 0; n < 3; n++) {
				foc_pll_step(&p, (struct foc_ab){-1.0f, 5.0f});
				zassert_true(pll_ok(&p), "x %e case %zu", (double)bad[i], k);
			}
			foc_pll_reset(&p, bad[i], bad[i]);
			zassert_true(pll_ok(&p), "x %e case %zu", (double)bad[i], k);
			foc_pll_step(&p, (struct foc_ab){-1.0f, 5.0f});
			zassert_true(pll_ok(&p), "x %e case %zu", (double)bad[i], k);
		}
	}
}

ZTEST(foc_nonfinite, test_pll_step)
{
	const float gains[] = {1.0f, FLT_MAX};
	struct foc_pll p;

	for (size_t gi = 0; gi < ARRAY_SIZE(gains); gi++) {
		for (size_t i = 0; i < N_BAD; i++) {
			foc_pll_init(&p, (struct foc_pi_gains){gains[gi], gains[gi]}, 50e-6f,
				     3000.0f, 0.1f);
			foc_pll_reset(&p, 0.5f, 300.0f);
			for (size_t j = 0; j < N_BAD; j++) {
				const struct foc_ab e[] = {
					{bad[i], 1.0f}, {1.0f, bad[i]}, {bad[i], bad[j]}};

				for (size_t k = 0; k < ARRAY_SIZE(e); k++) {
					foc_pll_step(&p, e[k]);
					zassert_true(pll_ok(&p), "x %e y %e k %zu", (double)bad[i],
						     (double)bad[j], k);
				}
			}
		}
	}
	/* A NaN EMF is no error: the speed coasts. */
	foc_pll_init(&p, foc_gains_pll(150.0f), 50e-6f, 3000.0f, 0.1f);
	foc_pll_reset(&p, 0.5f, 300.0f);
	foc_pll_step(&p, (struct foc_ab){NAN, NAN});
	zassert_equal(p.err, 0.0f);
	zassert_equal(p.omega, 300.0f);
}

ZTEST(foc_nonfinite, test_duty_hist_init)
{
	struct foc_duty_hist h;

	foc_duty_hist_init(&h, UINT_MAX);
	zassert_equal(h.delay, FOC_DUTY_DELAY_MAX);
	foc_duty_hist_push(&h, (struct foc_abc){0.1f, 0.2f, 0.3f});
	zassert_equal(foc_duty_hist_applied(&h).a, 0.5f);
}

ZTEST_SUITE(foc_nonfinite, NULL, NULL, NULL, NULL, NULL);
