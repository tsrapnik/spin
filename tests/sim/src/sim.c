/*
 * SPDX-License-Identifier: MIT
 */

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
};

void sim_init(struct sim *s, const struct plant_params *prm, const struct sim_cfg *cfg)
{
	float kt = foc_torque_constant(prm->psi, prm->pole_pairs);

	*s = (struct sim){.cfg = *cfg};
	zassert_equal(plant_init(&s->p, prm), 0);
	foc_current_init(&s->cur, foc_gains_current(prm->rs, prm->ls, cfg->bw_current_hz), prm->ts,
			 cfg->max_duty, cfg->adv_periods * prm->ts);
	foc_pi_init(&s->speed, foc_gains_speed(prm->j, kt, cfg->bw_speed_hz),
		    prm->ts * (float)cfg->speed_div);
}

void sim_cycle(struct sim *s)
{
	struct plant_sample smp = plant_sample(&s->p);
	/* True angle and speed (M2); the observer replaces these in M3. */
	float theta = (float)s->p.theta_e;
	float wm = (float)s->p.wm;

	if (s->speed_loop && (s->k % s->cfg.speed_div) == 0U) {
		s->i_ref.q = foc_pi_step(&s->speed, s->w_ref - wm, -s->cfg.i_max, s->cfg.i_max);
	}

	struct foc_abc duty =
		foc_current_step(&s->cur, (struct foc_abc){.a = smp.ia, .b = smp.ib, .c = smp.ic},
				 smp.vbus, theta, (float)s->p.pp * wm, s->i_ref);

	plant_step(&s->p, duty);
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
