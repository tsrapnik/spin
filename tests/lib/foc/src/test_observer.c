/*
 * SPDX-License-Identifier: MIT
 *
 * Duty history and current/back-EMF observer against its own discrete model.
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define PI_D 3.14159265358979323846

#define RS  3.6
#define LS  1.5e-3
#define TS  50e-6
#define PSI 0.011
#define BW  2000.0f

static void init(struct foc_obs *o)
{
	foc_obs_init(o, (float)RS, (float)LS, (float)TS, BW, 6.0f, 48.0f);
}

ZTEST(foc_observer, test_duty_hist_delay)
{
	for (unsigned int delay = 0; delay <= FOC_DUTY_DELAY_MAX + 1U; delay++) {
		unsigned int eff = MIN(delay, FOC_DUTY_DELAY_MAX);
		struct foc_duty_hist h;

		foc_duty_hist_init(&h, delay);
		/*
		 * Duty written in cycle k is d = k; at sample k the period
		 * before ran on k - 1 - delay.
		 */
		for (int k = 0; k < 8; k++) {
			struct foc_abc d = foc_duty_hist_applied(&h);
			float expect = (k - 1 - (int)eff >= 0) ? (float)(k - 1 - (int)eff) : 0.5f;

			zassert_equal(d.a, expect, "delay %u k %d: %f", delay, k, (double)d.a);
			zassert_true(d.b == d.a && d.c == d.a);
			foc_duty_hist_push(&h, (struct foc_abc){(float)k, (float)k, (float)k});
		}
	}
}

ZTEST(foc_observer, test_coefficients)
{
	struct foc_obs o;
	double a = exp(-RS * TS / LS), b = (1.0 - a) / RS;
	double z = exp(-2.0 * PI_D * (double)BW * TS);

	init(&o);
	zassert_within((double)o.a, a, 1e-6);
	zassert_within((double)o.b, b, 1e-6 * b);

	/* Error dynamics at omega = 0: both roots at z. */
	double tr = (1.0 - (double)o.g_i) * (double)o.a + 1.0 - (double)o.g_e * (double)o.b;
	double det = (1.0 - (double)o.g_i) * (double)o.a;

	zassert_within(tr, 2.0 * z, 1e-5, "trace %f", tr);
	zassert_within(det, z * z, 1e-5, "det %f", det);

	/* R = 0: b is the limit ts / L. */
	foc_obs_init(&o, 0.0f, (float)LS, (float)TS, BW, 6.0f, 48.0f);
	zassert_equal(o.a, 1.0f);
	zassert_within((double)o.b, TS / LS, 1e-7 * TS / LS);
}

/*
 * Data from the observer's own model (rotating EMF at mid-period, current by
 * exact ZOH) in double precision: the estimates converge to float accuracy
 * from zero, in either direction of rotation and with a voltage applied.
 */
ZTEST(foc_observer, test_converges_on_model)
{
	const double ws[] = {550.0, -550.0, 1200.0, 0.0};

	for (size_t n = 0; n < ARRAY_SIZE(ws); n++) {
		const double w = ws[n], a = exp(-RS * TS / LS), b = (1.0 - a) / RS;
		double ia = 0.0, ib = 0.0, th = 0.4;
		double e_max = 0.0;
		struct foc_obs o;

		init(&o);
		for (int k = 0; k < 400; k++) {
			double va = 3.0 * cos(1e3 * k * TS), vb = -2.0;
			double tm = th + 0.5 * w * TS;
			double ea = -w * PSI * sin(tm), eb = w * PSI * cos(tm);
			struct foc_ab e;

			ia = a * ia + b * (va - ea);
			ib = a * ib + b * (vb - eb);
			th += w * TS;
			e = foc_obs_step(&o, (struct foc_ab){(float)ia, (float)ib},
					 (struct foc_ab){(float)va, (float)vb}, (float)w);
			if (k >= 300) {
				e_max = fmax(e_max, hypot((double)e.alpha + w * PSI * sin(th),
							  (double)e.beta - w * PSI * cos(th)));
				zassert_within((double)o.i.alpha, ia, 1e-5);
				zassert_within((double)o.i.beta, ib, 1e-5);
			}
		}
		zassert_true(e_max < 1e-4, "w %f: |e error| %e", w, e_max);
	}
}

/* Step response of the error from a 1 V EMF offset: 2 % after about 6 / (2 pi bw). */
ZTEST(foc_observer, test_bandwidth)
{
	struct foc_obs o;
	const float e_true = 1.0f;
	/* Locked rotor with a constant 1 V "EMF": i settles at -1/R. */
	double i = 0.0, a = exp(-RS * TS / LS), b = (1.0 - a) / RS;
	int k98 = -1;

	init(&o);
	for (int k = 0; k < 200; k++) {
		i = a * i + b * (0.0 - (double)e_true);
		(void)foc_obs_step(&o, (struct foc_ab){(float)i, 0.0f}, (struct foc_ab){0}, 0.0f);
		if (k98 < 0 && fabsf(o.e.alpha - e_true) < 0.02f * e_true) {
			k98 = k;
		}
	}
	/* Critically damped double pole: 2 % at about 5.8 time constants. */
	double t98 = (k98 + 1) * TS * 2.0 * PI_D * (double)BW;

	zassert_true(t98 > 4.5 && t98 < 7.5, "t98 %f (k %d)", t98, k98);
	zassert_within(o.e.alpha, e_true, 1e-4f);
	zassert_within(o.e.beta, 0.0f, 1e-6f);
}

/* A NaN measurement makes the observer coast on its model. */
ZTEST(foc_observer, test_nan_current_coasts)
{
	struct foc_obs o, ref;
	const struct foc_ab v = {2.0f, -1.0f};

	init(&o);
	o.e = (struct foc_ab){1.0f, 0.5f};
	o.i = (struct foc_ab){0.3f, 0.1f};
	ref = o;
	(void)foc_obs_step(&o, (struct foc_ab){NAN, NAN}, v, 500.0f);
	zassert_equal(o.err.alpha, 0.0f);
	zassert_equal(o.err.beta, 0.0f);

	/* Same as feeding the predicted current. */
	struct foc_ab i_pred = o.i;

	(void)foc_obs_step(&ref, i_pred, v, 500.0f);
	zassert_within(ref.e.alpha, o.e.alpha, 1e-6f);
	zassert_within(ref.e.beta, o.e.beta, 1e-6f);
}

ZTEST(foc_observer, test_reset)
{
	struct foc_obs o;

	init(&o);
	(void)foc_obs_step(&o, (struct foc_ab){1.0f, 1.0f}, (struct foc_ab){5.0f, 0.0f}, 100.0f);
	foc_obs_reset(&o);
	zassert_true(o.i.alpha == 0.0f && o.i.beta == 0.0f && o.e.alpha == 0.0f &&
		     o.e.beta == 0.0f && o.err.alpha == 0.0f && o.err.beta == 0.0f);
	zassert_true(o.g_e > 0.0f);
}

ZTEST_SUITE(foc_observer, NULL, NULL, NULL, NULL, NULL);
