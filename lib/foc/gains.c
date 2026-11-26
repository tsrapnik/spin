/*
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>

#include <pmsm/foc.h>

static bool nonneg_finite(float x)
{
	return x >= 0.0f && x <= FLT_MAX;
}

static struct foc_pi_gains checked(float kp, float ki)
{
	if (!nonneg_finite(kp) || !nonneg_finite(ki)) {
		return (struct foc_pi_gains){0};
	}
	return (struct foc_pi_gains){.kp = kp, .ki = ki};
}

struct foc_pi_gains foc_gains_current(float rs, float ls, float bw_hz)
{
	float wc = FOC_2PI_F * bw_hz;

	if (!nonneg_finite(rs) || !nonneg_finite(ls) || !nonneg_finite(bw_hz)) {
		return (struct foc_pi_gains){0};
	}
	return checked(ls * wc, rs * wc);
}

struct foc_pi_gains foc_gains_speed(float j, float kt, float bw_hz)
{
	float wc = FOC_2PI_F * bw_hz;
	float kp;

	if (!nonneg_finite(j) || !(kt > 0.0f && kt <= FLT_MAX) || !nonneg_finite(bw_hz)) {
		return (struct foc_pi_gains){0};
	}
	kp = j * wc / kt;
	return checked(kp, kp * wc / FOC_SPEED_PI_ZERO_RATIO);
}

struct foc_pi_gains foc_gains_pll(float bw_hz)
{
	float wn = FOC_2PI_F * bw_hz;

	if (!nonneg_finite(bw_hz)) {
		return (struct foc_pi_gains){0};
	}
	return checked(2.0f * wn, wn * wn);
}
