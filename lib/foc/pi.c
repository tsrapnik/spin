/*
 * SPDX-License-Identifier: MIT
 */

#include <pmsm/foc.h>

/* Negative, NaN or +-Inf -> 0. */
static float nonneg_finite(float x)
{
	return (x >= 0.0f && x <= FLT_MAX) ? x : 0.0f;
}

void foc_pi_init(struct foc_pi *pi, struct foc_pi_gains g, float ts)
{
	pi->kp = nonneg_finite(g.kp);
	pi->ki_ts = nonneg_finite(nonneg_finite(g.ki) * nonneg_finite(ts));
	pi->integ = 0.0f;
}

float foc_pi_step(struct foc_pi *pi, float err, float lo, float hi)
{
	/*
	 * With finite err and finite non-negative gains, p and the new
	 * integrator have the sign of err, so p + integ cannot be Inf - Inf.
	 * An overflow to +-Inf is clamped below.
	 */
	float e = foc_sanitizef(err, FLT_MAX);
	float p = pi->kp * e;
	float integ = pi->integ + pi->ki_ts * e;

	/*
	 * Anti-windup: the integrator may move towards a limit only until the
	 * output reaches it (integ <= hi - p), and never moves away from where
	 * it was because of that rule (if p alone already saturates, it holds).
	 */
	integ = fminf(integ, fmaxf(pi->integ, hi - p));
	integ = fmaxf(integ, fminf(pi->integ, lo - p));

	/* Also covers limits that shrank since the last step. */
	pi->integ = foc_clampf(integ, lo, hi);

	return foc_clampf(p + pi->integ, lo, hi);
}
