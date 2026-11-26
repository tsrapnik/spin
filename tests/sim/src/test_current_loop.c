/*
 * SPDX-License-Identifier: MIT
 *
 * Current loop on the plant with the true angle: step responses at
 * standstill and at speed, angle advance, voltage limit.
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "sim.h"

/* Inertia so large that the speed stays where it is set. */
#define J_STIFF 1e6f

struct step_result {
	double t63;  /**< Time to 63.2 % of the step, s. */
	double peak; /**< Highest iq. */
	double id_max;
	double iq_end, id_end;
};

/* Step iq_ref 0 -> i_step after 20 ms at constant speed wm, run for t_run. */
static struct step_result current_step(float bw, float wm, float adv, float i_step, float t_run)
{
	struct plant_params m = sim_motor;
	struct sim_cfg c = sim_default_cfg;
	struct step_result r = {.t63 = -1.0};
	struct sim s;

	m.j = J_STIFF;
	c.bw_current_hz = bw;
	c.adv_periods = adv;
	sim_init(&s, &m, &c);
	s.p.wm = (double)wm;
	/* Let the short-circuit transient from the back-EMF die out first. */
	sim_run(&s, sim_cycles(&s, 0.02f));

	double t0 = s.p.t;

	s.i_ref.q = i_step;
	for (unsigned int k = 0; k < sim_cycles(&s, t_run); k++) {
		sim_cycle(&s);
		r.peak = fmax(r.peak, s.p.iq);
		r.id_max = fmax(r.id_max, fabs(s.p.id));
		if (r.t63 < 0.0 && s.p.iq >= 0.632 * (double)i_step) {
			r.t63 = s.p.t - t0;
		}
	}
	r.iq_end = s.p.iq;
	r.id_end = s.p.id;
	return r;
}

static double tau_of(float bw)
{
	return 1.0 / (2.0 * (double)FOC_PI_F * (double)bw);
}

/*
 * Standstill: pole cancellation gives a first-order response with time
 * constant 1/wc. The one-period duty delay adds no overshoot at fs/40.
 */
ZTEST(sim_current, test_step_standstill)
{
	const float bw = 500.0f;
	struct step_result r = current_step(bw, 0.0f, FOC_ADVANCE_PERIODS, 1.0f, 0.01f);
	double tau = tau_of(bw);

	zassert_true(r.t63 > 0.8 * tau && r.t63 < 1.2 * tau, "t63 %g tau %g", r.t63, tau);
	zassert_true(r.peak < 1.005, "peak %f", r.peak);
	zassert_within(r.iq_end, 1.0, 1e-4);
	zassert_within(r.id_max, 0.0, 1e-6);
}

/*
 * At speed (about 380 rpm, back-EMF 4.8 V) in both directions: the
 * integrators remove the back-EMF and cross-coupling, d sees only a small
 * transient.
 */
ZTEST(sim_current, test_step_at_speed)
{
	const float bw = 500.0f;
	const float speeds[] = {40.0f, -40.0f};

	for (size_t i = 0; i < ARRAY_SIZE(speeds); i++) {
		struct step_result r =
			current_step(bw, speeds[i], FOC_ADVANCE_PERIODS, 1.0f, 0.01f);
		double tau = tau_of(bw);

		zassert_true(r.t63 > 0.8 * tau && r.t63 < 1.2 * tau, "wm %f t63 %g",
			     (double)speeds[i], r.t63);
		zassert_true(r.peak < 1.01, "peak %f", r.peak);
		zassert_within(r.iq_end, 1.0, 1e-4);
		zassert_true(r.id_max < 0.08, "id_max %f", r.id_max);
		zassert_within(r.id_end, 0.0, 1e-4);
	}
}

/* Advancing the inverse-Park angle by 1.5 periods reduces the d excursion. */
ZTEST(sim_current, test_angle_advance_helps)
{
	struct step_result with = current_step(500.0f, 40.0f, FOC_ADVANCE_PERIODS, 1.0f, 0.01f);
	struct step_result without = current_step(500.0f, 40.0f, 0.0f, 1.0f, 0.01f);

	zassert_true(with.id_max < 0.9 * without.id_max, "with %f without %f", with.id_max,
		     without.id_max);
}

/* Rule of thumb in foc_gains_current(): bandwidth <= fs/20 keeps overshoot small. */
ZTEST(sim_current, test_bandwidth_rule)
{
	const float fs = 1.0f / sim_motor.ts;
	struct step_result r = current_step(fs / 20.0f, 0.0f, FOC_ADVANCE_PERIODS, 1.0f, 0.01f);

	zassert_true(r.peak < 1.05, "fs/20: peak %f", r.peak);
	zassert_within(r.iq_end, 1.0, 1e-4);
}

/*
 * Voltage limit at about 760 rpm (back-EMF 9.7 V of 12.5 V available): 1 A
 * is out of reach. The voltage sits on the circle, the integrators do not
 * wind up, and a reachable reference is tracked again within a few tau.
 */
ZTEST(sim_current, test_voltage_limit_recovery)
{
	struct plant_params m = sim_motor;
	const float vmax = foc_svpwm_vmax(sim_motor.vbus, sim_default_cfg.max_duty);
	struct sim s;

	m.j = J_STIFF;
	sim_init(&s, &m, &sim_default_cfg);
	s.p.wm = 80.0;
	s.i_ref.q = 1.0f;
	sim_run(&s, sim_cycles(&s, 0.05f));

	zassert_true(s.p.iq < 0.9, "iq %f must be voltage-limited", s.p.iq);
	zassert_within(hypotf(s.cur.v.d, s.cur.v.q), vmax, 1e-3f);
	zassert_true(fabsf(s.cur.q.integ) <= vmax);

	s.i_ref.q = 0.3f;
	sim_run(&s, sim_cycles(&s, 6.0f * (float)tau_of(sim_default_cfg.bw_current_hz)));
	zassert_within(s.p.iq, 0.3, 0.02, "iq %f", s.p.iq);
	sim_run(&s, sim_cycles(&s, 0.01f));
	zassert_within(s.p.iq, 0.3, 1e-4, "iq %f", s.p.iq);
}

ZTEST_SUITE(sim_current, NULL, NULL, NULL, NULL, NULL);
