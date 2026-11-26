/*
 * SPDX-License-Identifier: MIT
 *
 * Observer + PLL against the plant angle. The loops run on the true angle
 * (sensorless = false) unless a test switches over; the observer and PLL
 * always run on the sampled current and the applied duty, from zero state.
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "sim.h"

struct track {
	double ang;   /**< Largest |angle error|, rad electrical. */
	double w;     /**< Largest |speed error|, rad/s mechanical. */
	double e_rel; /**< Largest relative error of |e_hat| against p psi |wm|. */
	double r;     /**< Largest |prediction error|, A. */
};

static struct track measure(struct sim *s, float t)
{
	struct track r = {0};

	for (unsigned int k = 0; k < sim_cycles(s, t); k++) {
		/* EMF at this sample, before the plant moves on. */
		double e_true = s->p.pp * (double)sim_motor.psi * fabs(s->p.wm);

		sim_cycle(s);
		r.ang = fmax(r.ang, fabs(sim_angle_err(s)));
		r.w = fmax(r.w, fabs(sim_w_est(s) - s->p.wm));
		r.e_rel = fmax(r.e_rel,
			       fabs(hypot((double)s->obs.e.alpha, (double)s->obs.e.beta) - e_true) /
				       e_true);
		r.r = fmax(r.r, hypot((double)s->obs.err.alpha, (double)s->obs.err.beta));
	}
	return r;
}

/* Speed loop on the true angle to w (with load), then measure for 0.1 s. */
static struct track run_at(const struct plant_params *prm, const struct sim_cfg *cfg, float w,
			   double load)
{
	struct sim s;

	sim_init(&s, prm, cfg);
	s.speed_loop = true;
	s.w_ref = w;
	s.p.t_load = load;
	sim_run(&s, sim_cycles(&s, 0.3f));
	return measure(&s, 0.1f);
}

/* Each sample is paired with the duty the plant applied before it. */
ZTEST(sim_observer, test_duty_pairing)
{
	for (unsigned int d = 0; d <= PLANT_DUTY_DELAY_MAX; d++) {
		struct plant_params prm = sim_motor;
		struct sim_cfg cfg = sim_default_cfg;
		struct sim s;

		prm.duty_delay = d;
		cfg.obs_delay = d;
		sim_init(&s, &prm, &cfg);
		s.speed_loop = true;
		s.w_ref = 50.0f;
		for (unsigned int k = 0; k < sim_cycles(&s, 0.05f); k++) {
			sim_cycle(&s);
			struct foc_abc a = foc_duty_hist_applied(&s.hist);

			zassert_true(a.a == s.p.applied.a && a.b == s.p.applied.b &&
					     a.c == s.p.applied.c,
				     "delay %u cycle %u", d, k);
		}
	}
}

/*
 * Constant speed, both directions, with and without load: angle, speed and
 * EMF magnitude match the plant. The observer starts from zero at
 * standstill and acquires during the speed-up.
 */
ZTEST(sim_observer, test_constant_speed)
{
	const float ws[] = {5.0f, 20.0f, 50.0f, 90.0f, -50.0f};
	const double loads[] = {0.0, 0.05};

	for (size_t i = 0; i < ARRAY_SIZE(ws); i++) {
		for (size_t l = 0; l < ARRAY_SIZE(loads); l++) {
			double load = copysign(loads[l], (double)ws[i]);
			struct track r = run_at(&sim_motor, &sim_default_cfg, ws[i], load);

			zassert_true(r.ang < 1e-3, "w %f load %f: angle %e", (double)ws[i], load,
				     r.ang);
			zassert_true(r.w < 1e-3, "w %f load %f: speed %e", (double)ws[i], load,
				     r.w);
			zassert_true(r.e_rel < 1e-3, "w %f load %f: |e| %e", (double)ws[i], load,
				     r.e_rel);
		}
	}
}

/*
 * The observer must use the duty that was actually applied. Assuming one
 * period too few or too many costs 100 times the angle error, under load
 * at 50 rad/s. The prediction error does not show it: at constant speed
 * the EMF estimate absorbs the voltage error.
 */
ZTEST(sim_observer, test_delay_mismatch)
{
	struct sim_cfg cfg = sim_default_cfg;
	struct track ok = run_at(&sim_motor, &cfg, 50.0f, 0.05);

	zassert_true(ok.r < 1e-5, "r %e", ok.r);
	for (unsigned int d = 0; d <= 2U; d += 2U) {
		cfg.obs_delay = d;
		struct track bad = run_at(&sim_motor, &cfg, 50.0f, 0.05);

		zassert_true(bad.ang > 0.02 && bad.ang > 50.0 * ok.ang, "delay %u angle %e", d,
			     bad.ang);
	}
}

/*
 * Parameter errors of +-30 %, under load. With id = 0 an Rs error acts along
 * the EMF (q), so it only scales |e|; an Ls error acts across it (w L iq on
 * d) and shows as an angle error.
 */
ZTEST(sim_observer, test_parameter_error)
{
	const float sc[] = {0.7f, 1.3f};

	for (size_t i = 0; i < ARRAY_SIZE(sc); i++) {
		struct sim_cfg cfg = sim_default_cfg;
		struct track r;

		cfg.obs_rs_scale = sc[i];
		r = run_at(&sim_motor, &cfg, 50.0f, 0.05);
		zassert_true(r.ang < 2e-3, "rs x%f: angle %e", (double)sc[i], r.ang);
		zassert_true(r.e_rel < 0.1, "rs x%f: |e| %e", (double)sc[i], r.e_rel);

		cfg = sim_default_cfg;
		cfg.obs_ls_scale = sc[i];
		r = run_at(&sim_motor, &cfg, 50.0f, 0.05);
		zassert_true(r.ang < 0.02, "ls x%f: angle %e", (double)sc[i], r.ang);
	}
}

/* PLL preloaded with the wrong direction at 50 rad/s: relocks within 30 ms. */
ZTEST(sim_observer, test_wrong_direction_preload)
{
	struct sim s;
	struct track r;

	sim_init(&s, &sim_motor, &sim_default_cfg);
	s.speed_loop = true;
	s.w_ref = 50.0f;
	sim_run(&s, sim_cycles(&s, 0.3f));
	foc_pll_reset(&s.pll, s.pll.theta, -s.pll.omega);
	foc_obs_reset(&s.obs);
	sim_run(&s, sim_cycles(&s, 0.03f));
	r = measure(&s, 0.05f);
	zassert_true(r.ang < 1e-3 && r.w < 1e-3, "angle %e speed %e", r.ang, r.w);
}

/*
 * Sensorless closed loop: switch from the true to the estimated angle at
 * 50 rad/s, then a load step and a speed step. Start-up from standstill
 * follows in M3.2.
 */
ZTEST(sim_observer, test_sensorless_closed_loop)
{
	struct sim s;
	struct track r;

	sim_init(&s, &sim_motor, &sim_default_cfg);
	s.speed_loop = true;
	s.w_ref = 50.0f;
	sim_run(&s, sim_cycles(&s, 0.3f));
	s.sensorless = true;
	r = measure(&s, 0.1f);
	zassert_true(r.ang < 1e-3, "switch: angle %e", r.ang);

	s.p.t_load = 0.05;
	r = measure(&s, 0.2f);
	zassert_true(r.ang < 0.05, "load step: angle %e", r.ang);
	zassert_within(s.p.wm, 50.0, 1e-3);

	s.w_ref = 80.0f;
	r = measure(&s, 0.2f);
	zassert_true(r.ang < 0.1, "speed step: angle %e", r.ang);
	zassert_within(s.p.wm, 80.0, 1e-3);
	zassert_true(fabs(sim_angle_err(&s)) < 1e-3);
}

ZTEST_SUITE(sim_observer, NULL, NULL, NULL, NULL, NULL);
