/*
 * SPDX-License-Identifier: MIT
 *
 * Back-EMF PLL, see struct foc_pll in pmsm/foc.h.
 */

#include <pmsm/foc.h>

static float nonneg_or_zero(float x)
{
	return (x >= 0.0f && x <= FLT_MAX) ? x : 0.0f;
}

void foc_pll_init(struct foc_pll *p, struct foc_pi_gains g, float ts, float w_max, float e_min)
{
	*p = (struct foc_pll){0};
	p->ts = nonneg_or_zero(ts);
	foc_pi_init(&p->pi, g, p->ts);
	p->w_max = nonneg_or_zero(w_max);
	/* Floor at FLT_MIN so the normalisation never divides by 0. */
	p->e_min = fmaxf(nonneg_or_zero(e_min), FLT_MIN);
	p->k_f_ts = foc_sanitizef(FOC_PLL_FLL_RATIO * p->pi.kp * p->ts, FLT_MAX);
	p->inv_ts = (p->ts > 0.0f) ? foc_sanitizef(1.0f / p->ts, FLT_MAX) : 0.0f;
}

void foc_pll_reset(struct foc_pll *p, float theta, float omega)
{
	p->theta = foc_wrap_pi(theta);
	p->omega = foc_sanitizef(omega, p->w_max);
	foc_pi_reset(&p->pi, p->omega);
	p->err = 0.0f;
	p->e_prev = (struct foc_dq){0};
}

void foc_pll_step(struct foc_pll *p, struct foc_ab e)
{
	p->theta = foc_wrap_pi(p->theta + p->omega * p->ts);

	/* EMF in the estimated rotor frame: e_d ~ -|e| sin(theta - theta_hat) sign(omega). */
	struct foc_dq e_dq = foc_park(e, foc_sincos(p->theta));
	float inv = 1.0f / fmaxf(sqrtf(e.alpha * e.alpha + e.beta * e.beta), p->e_min);
	float err = -e_dq.d * inv;
	/*
	 * Rotation of e in this frame since the last step, which already
	 * advanced by omega * ts: |e|^2 sin(dphi - omega ts) / ts ~ the speed
	 * error, whatever the angle error. Falls off with |e|^2 below e_min.
	 */
	float f_err = (p->e_prev.d * e_dq.q - p->e_prev.q * e_dq.d) * (inv * inv * p->inv_ts);

	/*
	 * Direction from the integrator, not omega: Kp * err can flip the sign
	 * of omega every step, which otherwise locks into a period-2 cycle.
	 * Non-finite e (or |e| overflowing when squared) -> NaN -> 0.
	 */
	err = foc_sanitizef((p->pi.integ < 0.0f) ? -err : err, 1.0f);
	p->err = err;
	p->f_err = foc_sanitizef(f_err, 2.0f * p->w_max);
	p->e_prev = (struct foc_dq){.d = foc_sanitizef(e_dq.d, FLT_MAX),
				    .q = foc_sanitizef(e_dq.q, FLT_MAX)};
	/* Frequency aid: pull the integrator by the measured speed error. */
	foc_pi_reset(&p->pi, p->pi.integ + p->k_f_ts * p->f_err);
	p->omega = foc_pi_step(&p->pi, err, -p->w_max, p->w_max);
}
