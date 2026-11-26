/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "drive.h"

const struct foc_cycle_cfg drive_cycle_cfg = {
	.ts = 50e-6f,
	.rs = 3.6f,
	.ls = 1.5e-3f,
	.max_duty = 0.95f,
	.t_adv = FOC_ADVANCE_PERIODS * 50e-6f,
	.delay = 1U,
	.bw_current_hz = 500.0f,
	.bw_obs_hz = 2000.0f,
	.bw_pll_hz = 150.0f,
	.w_max = 3000.0f,
	.e_min = 0.1f,
	.i_trip = 3.0f,
	.vbus_min = 10.0f,
	.vbus_max = 30.0f,
};

const struct foc_seq_cfg drive_seq_cfg = {
	.ts = 1e-3f,
	.pole_pairs = 11U,
	.j = 6e-6f,
	.psi = 0.011f,
	.bw_speed_hz = 50.0f,
	.i_max = 1.4f,
	.i_start = 1.0f,
	.t_align = 0.1f,
	.accel_start = 300.0f,
	.w_handover = 30.0f,
	.accel = 500.0f,
	.w_max = 100.0f,
};

void drive_init(struct drive *d, const struct plant_params *prm, const struct foc_cycle_cfg *cc,
		const struct foc_seq_cfg *sc)
{
	*d = (struct drive){0};
	zassert_equal(plant_init(&d->p, prm), 0);
	foc_cycle_init(&d->c, cc);
	foc_seq_init(&d->s, sc);
	d->div = (unsigned int)(sc->ts / cc->ts + 0.5f);
	d->cmd = d->s.cmd;
	d->st.state = FOC_STATE_STOP;
}

void drive_cycle(struct drive *d)
{
	if (d->k % d->div == 0U) {
		d->cmd = *foc_seq_step(&d->s, &d->st);
	}

	struct plant_sample smp = plant_sample(&d->p);
	struct foc_abc i = {
		.a = smp.ia + d->i_add.a, .b = smp.ib + d->i_add.b, .c = smp.ic + d->i_add.c};

	d->theta_smp = d->p.theta_e;

	struct foc_abc duty = foc_cycle_step(&d->c, &d->cmd, i, smp.vbus, &d->st);

	d->pwm = d->st.state != FOC_STATE_STOP && d->st.state != FOC_STATE_FAULT;
	if (d->pwm) {
		plant_step(&d->p, duty);
	} else {
		plant_coast(&d->p);
	}
	d->k++;
}

void drive_run(struct drive *d, float t)
{
	unsigned int n = (unsigned int)(t / (float)d->p.ts + 0.5f);

	for (unsigned int i = 0; i < n; i++) {
		drive_cycle(d);
	}
}

double drive_until(struct drive *d, enum foc_state st, float t_max)
{
	double t0 = d->p.t;

	while (d->p.t - t0 < (double)t_max) {
		drive_cycle(d);
		if (d->s.state == st) {
			return d->p.t - t0;
		}
	}
	return -1.0;
}

double drive_angle_err(const struct drive *d)
{
	double e = (double)d->st.theta - d->theta_smp;

	return e - 2.0 * PLANT_PI * round(e / (2.0 * PLANT_PI));
}
