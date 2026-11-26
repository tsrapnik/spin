/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "sim.h"

const struct plant_params sim_motor = {
	.rs = 3.6f,
	.ls = 1.5e-3f,
	.psi = 0.011f,
	.pole_pairs = 11U,
	.j = 6e-6f,
	.b = 1e-6f,
	.vbus = 24.0f,
	.ts = 50e-6f,
	.duty_delay = 1U,
	.substeps = 8U,
};

const struct sim_cfg sim_default_cfg = {
	.bw_current_hz = 500.0f,
	.bw_speed_hz = 50.0f,
	.i_max = 1.4f,
	.max_duty = 0.95f,
	.speed_div = 20U,
	.adv_periods = FOC_ADVANCE_PERIODS,
	.bw_obs_hz = 2000.0f,
	.bw_pll_hz = 150.0f,
	.w_max_e = 3000.0f,
	.e_min = 0.1f,
	.obs_delay = 1U,
};

static float scale(float x)
{
	return x > 0.0f ? x : 1.0f;
}

void sim_init(struct sim *s, const struct plant_params *prm, const struct sim_cfg *cfg)
{
	float kt = foc_torque_constant(prm->psi, prm->pole_pairs);

	*s = (struct sim){.cfg = *cfg};
	zassert_equal(plant_init(&s->p, prm), 0);
	foc_current_init(&s->cur, foc_gains_current(prm->rs, prm->ls, cfg->bw_current_hz), prm->ts,
			 cfg->max_duty, cfg->adv_periods * prm->ts);
	foc_pi_init(&s->speed, foc_gains_speed(prm->j, kt, cfg->bw_speed_hz),
		    prm->ts * (float)cfg->speed_div);
	foc_obs_init(&s->obs, prm->rs * scale(cfg->obs_rs_scale),
		     prm->ls * scale(cfg->obs_ls_scale), prm->ts, cfg->bw_obs_hz, 4.0f * cfg->i_max,
		     2.0f * prm->vbus);
	foc_pll_init(&s->pll, foc_gains_pll(cfg->bw_pll_hz), prm->ts, cfg->w_max_e, cfg->e_min);
	foc_duty_hist_init(&s->hist, cfg->obs_delay);
}

void sim_cycle(struct sim *s)
{
	struct plant_sample smp = plant_sample(&s->p);
	struct foc_abc i_abc = {.a = smp.ia, .b = smp.ib, .c = smp.ic};
	struct foc_ab v = foc_duty_to_ab(foc_duty_hist_applied(&s->hist), smp.vbus);

	s->theta_smp = s->p.theta_e;
	foc_pll_step(&s->pll, foc_obs_step(&s->obs, foc_clarke(i_abc), v, s->pll.omega));

	float theta = s->sensorless ? s->pll.theta : (float)s->p.theta_e;
	float wm = s->sensorless ? s->pll.omega / (float)s->p.pp : (float)s->p.wm;

	if (s->speed_loop && (s->k % s->cfg.speed_div) == 0U) {
		s->i_ref.q = foc_pi_step(&s->speed, s->w_ref - wm, -s->cfg.i_max, s->cfg.i_max);
	}

	struct foc_abc duty =
		foc_current_step(&s->cur, i_abc, smp.vbus, theta, (float)s->p.pp * wm, s->i_ref);

	plant_step(&s->p, duty);
	foc_duty_hist_push(&s->hist, duty);
	s->k++;
}

void sim_run(struct sim *s, unsigned int n)
{
	for (unsigned int i = 0; i < n; i++) {
		sim_cycle(s);
	}
}

unsigned int sim_cycles(const struct sim *s, float t)
{
	return (unsigned int)(t / (float)s->p.ts + 0.5f);
}

double sim_angle_err(const struct sim *s)
{
	double d = (double)s->pll.theta - s->theta_smp;

	return d - 2.0 * PLANT_PI * round(d / (2.0 * PLANT_PI));
}

double sim_w_est(const struct sim *s)
{
	return (double)s->pll.omega / s->p.pp;
}
