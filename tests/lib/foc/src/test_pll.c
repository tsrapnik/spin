/*
 * SPDX-License-Identifier: MIT
 *
 * Back-EMF PLL on a synthetic EMF e = w psi (-sin(th), cos(th)).
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define PI_D 3.14159265358979323846

#define TS    50e-6
#define PSI   0.011
#define BW    150.0f
#define W_MAX 3000.0f

static void init(struct foc_pll *p)
{
	foc_pll_init(p, foc_gains_pll(BW), (float)TS, W_MAX, 0.1f);
}

static struct foc_ab emf(double w, double th)
{
	return (struct foc_ab){(float)(-w * PSI * sin(th)), (float)(w * PSI * cos(th))};
}

static double wrap(double x)
{
	return x - 2.0 * PI_D * round(x / (2.0 * PI_D));
}

/*
 * Run at constant speed w from preload w0; return the first cycle after
 * which the angle stays within 0.01 rad and the speed within 1 rad/s.
 */
static int lock_time(struct foc_pll *p, double w, float w0, int n)
{
	double th = 0.3;
	int lock = -1;

	init(p);
	foc_pll_reset(p, 0.0f, w0);
	for (int k = 0; k < n; k++) {
		th += w * TS;
		foc_pll_step(p, emf(w, th));
		if (fabs(wrap((double)p->theta - th)) < 0.01 && fabs((double)p->omega - w) < 1.0) {
			lock = (lock < 0) ? k : lock;
		} else {
			lock = -1;
		}
	}
	return lock;
}

ZTEST(foc_pll, test_gains)
{
	struct foc_pi_gains g = foc_gains_pll(100.0f);
	float wn = 2.0f * FOC_PI_F * 100.0f;

	zassert_within(g.kp, 2.0f * wn, 1e-3f);
	zassert_within(g.ki, wn * wn, 1.0f);
}

ZTEST(foc_pll, test_constant_speed)
{
	const double ws[] = {100.0, 550.0, -550.0, 1200.0, -1200.0};
	struct foc_pll p;

	for (size_t i = 0; i < ARRAY_SIZE(ws); i++) {
		int lock = lock_time(&p, ws[i], 0.0f, 4000);

		zassert_true(lock >= 0 && lock < 400, "w %f lock %d", ws[i], lock);
		zassert_within((double)p.omega, ws[i], 1e-3 * fabs(ws[i]));
		zassert_true(fabsf(p.err) < 1e-4f, "err %e", (double)p.err);
	}
}

/*
 * A preload with the wrong sign, or far off, still locks; the frequency
 * aid keeps it fast. Without it the wrong-sign cases take up to 1000
 * cycles at these speeds.
 */
ZTEST(foc_pll, test_acquisition)
{
	const double ws[] = {300.0, 550.0, -550.0, 1100.0};
	struct foc_pll p;

	for (size_t i = 0; i < ARRAY_SIZE(ws); i++) {
		const float w0[] = {(float)-ws[i], (float)(2.0 * ws[i]), W_MAX, -W_MAX};

		for (size_t j = 0; j < ARRAY_SIZE(w0); j++) {
			int lock = lock_time(&p, ws[i], w0[j], 4000);

			zassert_true(lock >= 0 && lock < 400, "w %f w0 %f lock %d", ws[i],
				     (double)w0[j], lock);
		}
	}
}

/* Type-2 loop: under constant acceleration alpha, sin(error) = alpha / wn^2. */
ZTEST(foc_pll, test_acceleration_error)
{
	const double alpha = 1e5, wn = 2.0 * PI_D * (double)BW;
	struct foc_pll p;
	double w = 300.0, th = 0.0;

	init(&p);
	foc_pll_reset(&p, 0.0f, (float)w);
	/* Up to 2500 rad/s (22 ms), below w_max. */
	while (w < 2500.0) {
		w += alpha * TS;
		th += w * TS;
		foc_pll_step(&p, emf(w, th));
	}
	double expect = alpha / (wn * wn);

	zassert_within((double)p.err, expect, 0.05 * expect, "err %f expect %f", (double)p.err,
		       expect);
	zassert_within(wrap(th - (double)p.theta), asin(expect), 0.05 * expect);
}

/* No EMF: no error, the estimate coasts at the preloaded speed. */
ZTEST(foc_pll, test_zero_emf_coasts)
{
	struct foc_pll p;

	init(&p);
	foc_pll_reset(&p, 1.0f, 200.0f);
	for (int k = 0; k < 100; k++) {
		foc_pll_step(&p, (struct foc_ab){0});
	}
	zassert_equal(p.err, 0.0f);
	zassert_within(p.omega, 200.0f, 1e-3f);
	zassert_within((double)p.theta, wrap(1.0 + 100 * 200.0 * TS), 1e-4);
}

/* Below e_min the loop gain falls with |e|. */
ZTEST(foc_pll, test_low_emf_gain)
{
	struct foc_pll p;

	init(&p);
	/* 90 deg ahead, |e| = e_min / 10: error is 1/10 of the normalised one. */
	foc_pll_step(&p, (struct foc_ab){-0.01f, 0.0f});
	zassert_within(p.err, 0.1f, 1e-6f);
	init(&p);
	foc_pll_step(&p, (struct foc_ab){-1.0f, 0.0f});
	zassert_within(p.err, 1.0f, 1e-6f);
}

ZTEST(foc_pll, test_reset_clamps)
{
	struct foc_pll p;

	init(&p);
	foc_pll_reset(&p, 7.0f, 1e6f);
	zassert_within((double)p.theta, wrap(7.0), 1e-5);
	zassert_equal(p.omega, W_MAX);
	zassert_equal(p.pi.integ, W_MAX);
	foc_pll_reset(&p, 0.0f, -1e6f);
	zassert_equal(p.omega, -W_MAX);
}

ZTEST_SUITE(foc_pll, NULL, NULL, NULL, NULL, NULL);
