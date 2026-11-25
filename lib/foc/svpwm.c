/*
 * SPDX-License-Identifier: MIT
 */

#include <pmsm/foc.h>

/* Keeps every intermediate below FLT_MAX: inverse Clarke adds two terms,
 * the zero-sequence offset two more.
 */
#define V_LIM (FLT_MAX / 8.0f)

struct foc_abc foc_svpwm(struct foc_ab v, float vbus, float max_duty)
{
	struct foc_ab vs = {
		.alpha = foc_sanitizef(v.alpha, V_LIM),
		.beta = foc_sanitizef(v.beta, V_LIM),
	};
	struct foc_abc p = foc_inv_clarke(vs);
	float vmax = fmaxf(fmaxf(p.a, p.b), p.c);
	float vmin = fminf(fminf(p.a, p.b), p.c);
	/* Zero-sequence offset that centres the phase voltages in the bus. */
	float v0 = -0.5f * (vmax + vmin);
	/* Low or NaN bus voltage: scale 0, i.e. all duties 0.5. */
	float scale = (vbus >= FOC_SVPWM_VBUS_MIN) ? 1.0f / vbus : 0.0f;
	float hi = foc_clampf(max_duty, 0.5f, 1.0f);
	float lo = 1.0f - hi;

	/* (p + v0) * scale may overflow to +-Inf; the clamp handles that. */
	return (struct foc_abc){
		.a = foc_clampf(0.5f + (p.a + v0) * scale, lo, hi),
		.b = foc_clampf(0.5f + (p.b + v0) * scale, lo, hi),
		.c = foc_clampf(0.5f + (p.c + v0) * scale, lo, hi),
	};
}
