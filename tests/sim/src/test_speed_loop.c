/*
 * SPDX-License-Identifier: MIT
 *
 * Speed loop (1 kHz) over the current loop (20 kHz) with the true angle:
 * speed steps, reversal, load step, voltage-limited speed.
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "sim.h"

#define BAND 0.02 /* settling band, relative */

struct speed_result {
	double peak, low; /**< Highest and lowest speed after the step. */
	double settle;    /**< Last time outside the band around the reference, s. */
};

/* Run for t seconds and record the speed excursion around s->w_ref. */
static struct speed_result run(struct sim *s, float t)
{
	struct speed_result r = {.peak = -1e30, .low = 1e30};
	double t0 = s->p.t, ref = (double)s->w_ref;

	for (unsigned int k = 0; k < sim_cycles(s, t); k++) {
		sim_cycle(s);
		r.peak = fmax(r.peak, s->p.wm);
		r.low = fmin(r.low, s->p.wm);
		if (fabs(s->p.wm - ref) > BAND * fabs(ref)) {
			r.settle = s->p.t - t0;
		}
	}
	return r;
}

static double kt(void)
{
	return (double)foc_torque_constant(sim_motor.psi, sim_motor.pole_pairs);
}

static void start(struct sim *s)
{
	sim_init(s, &sim_motor, &sim_default_cfg);
	s->speed_loop = true;
}

/*
 * 0 -> 50 rad/s (about 480 rpm). The PI zero at wc/4 costs some overshoot
 * on a reference step; the M3 ramps remove it.
 */
ZTEST(sim_speed, test_speed_step)
{
	struct sim s;
	struct speed_result r;

	start(&s);
	s.w_ref = 50.0f;
	r = run(&s, 0.2f);

	zassert_true(r.peak < 1.25 * 50.0, "peak %f", r.peak);
	zassert_true(r.settle < 0.06, "settle %f", r.settle);
	zassert_within(s.p.wm, 50.0, 1e-3);
	/* Steady state: torque balances friction only. */
	zassert_within(s.p.iq, (double)sim_motor.b * 50.0 / kt(), 1e-4, "iq %f", s.p.iq);
	zassert_within(s.p.id, 0.0, 1e-4);
	zassert_true(fabsf(s.i_ref.q) <= sim_default_cfg.i_max);
}

ZTEST(sim_speed, test_reversal)
{
	struct sim s;
	struct speed_result r;

	start(&s);
	s.w_ref = 50.0f;
	(void)run(&s, 0.2f);
	s.w_ref = -50.0f;
	r = run(&s, 0.3f);

	/* Overshoot relative to the 100 rad/s step, as in test_speed_step. */
	zassert_true(r.low > -50.0 - 0.25 * 100.0, "low %f", r.low);
	zassert_true(r.settle < 0.1, "settle %f", r.settle);
	zassert_within(s.p.wm, -50.0, 1e-3);
}

/* Load step 0.05 Nm (about 15 % of the motor's max torque) at 50 rad/s. */
ZTEST(sim_speed, test_load_step)
{
	const double t_load = 0.05;
	struct sim s;
	struct speed_result r;

	start(&s);
	s.w_ref = 50.0f;
	(void)run(&s, 0.2f);
	s.p.t_load = t_load;
	r = run(&s, 0.3f);

	zassert_true(r.low > 0.5 * 50.0, "dip to %f", r.low);
	zassert_true(r.settle < 0.06, "recovery %f", r.settle);
	zassert_within(s.p.wm, 50.0, 1e-3);
	double iq_ref = (t_load + (double)sim_motor.b * 50.0) / kt();

	zassert_within(s.p.iq, iq_ref, 1e-3 * iq_ref, "iq %f ref %f", s.p.iq, iq_ref);
}

/*
 * 120 rad/s is above what 24 V can reach: the speed stops where the
 * back-EMF uses the whole voltage circle, the speed PI saturates at i_max
 * without winding up, and a reachable reference is tracked again without
 * undershoot.
 */
ZTEST(sim_speed, test_voltage_limited_speed)
{
	const float vmax = foc_svpwm_vmax(sim_motor.vbus, sim_default_cfg.max_duty);
	const double w_lim = (double)vmax / ((double)sim_motor.pole_pairs * (double)sim_motor.psi);
	struct sim s;
	struct speed_result r;

	start(&s);
	s.w_ref = 120.0f;
	(void)run(&s, 0.3f);
	zassert_within(s.p.wm, w_lim, 0.02 * w_lim, "wm %f limit %f", s.p.wm, w_lim);
	zassert_equal(s.i_ref.q, sim_default_cfg.i_max);
	zassert_true(fabsf(s.speed.integ) <= sim_default_cfg.i_max);

	s.w_ref = 50.0f;
	r = run(&s, 0.3f);
	zassert_true(r.low > (1.0 - BAND) * 50.0, "undershoot to %f", r.low);
	zassert_true(r.settle < 0.1, "settle %f", r.settle);
	zassert_within(s.p.wm, 50.0, 1e-3);
}

ZTEST_SUITE(sim_speed, NULL, NULL, NULL, NULL, NULL);
