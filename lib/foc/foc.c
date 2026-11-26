/*
 * SPDX-License-Identifier: MIT
 */

#include <pmsm/foc.h>

void foc_current_init(struct foc_current *c, struct foc_pi_gains g, float ts, float max_duty,
		      float t_adv)
{
	foc_pi_init(&c->d, g, ts);
	foc_pi_init(&c->q, g, ts);
	/* NaN -> 0.5 (fmaxf returns the non-NaN operand). */
	c->max_duty = foc_clampf(max_duty, 0.5f, 1.0f);
	c->t_adv = (t_adv >= 0.0f && t_adv <= FLT_MAX) ? t_adv : 0.0f;
	c->i = (struct foc_dq){0};
	c->v = (struct foc_dq){0};
}

struct foc_abc foc_current_step(struct foc_current *c, struct foc_abc i_abc, float vbus,
				float theta, float omega, struct foc_dq ref)
{
	struct foc_dq i = foc_park(foc_clarke(i_abc), foc_sincos(theta));
	float vmax = foc_svpwm_vmax(vbus, c->max_duty);
	float vd = foc_pi_step(&c->d, ref.d - i.d, -vmax, vmax);
	float qlim = foc_q_limit(vd, vmax);
	float vq = foc_pi_step(&c->q, ref.q - i.q, -qlim, qlim);
	/* Non-finite theta or omega: foc_sincos() treats the sum as 0. */
	struct foc_sincos sc_v = foc_sincos(theta + omega * c->t_adv);

	c->i = i;
	c->v = (struct foc_dq){.d = vd, .q = vq};

	return foc_svpwm(foc_inv_park(c->v, sc_v), vbus, c->max_duty);
}
