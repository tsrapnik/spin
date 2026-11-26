/*
 * SPDX-License-Identifier: MIT
 *
 * Luenberger current/back-EMF observer, see struct foc_obs in pmsm/foc.h.
 */

#include <stdbool.h>

#include <pmsm/foc.h>

static bool pos_finite(float x)
{
	return x > 0.0f && x <= FLT_MAX;
}

static bool is_finite(float x)
{
	return x >= -FLT_MAX && x <= FLT_MAX;
}

/* Rotate x by the angle whose sine and cosine are given. */
FOC_INLINE struct foc_ab rot(struct foc_ab x, struct foc_sincos sc)
{
	return (struct foc_ab){
		.alpha = x.alpha * sc.cos - x.beta * sc.sin,
		.beta = x.alpha * sc.sin + x.beta * sc.cos,
	};
}

FOC_INLINE struct foc_ab clamp_ab(struct foc_ab x, float lim)
{
	return (struct foc_ab){
		.alpha = foc_sanitizef(x.alpha, lim),
		.beta = foc_sanitizef(x.beta, lim),
	};
}

void foc_obs_init(struct foc_obs *o, float rs, float ls, float ts, float bw_hz, float i_lim,
		  float v_lim)
{
	*o = (struct foc_obs){0};

	if (!(rs >= 0.0f && rs <= FLT_MAX) || !pos_finite(ls) || !pos_finite(ts) ||
	    !pos_finite(bw_hz) || !pos_finite(i_lim) || !pos_finite(v_lim)) {
		return;
	}

	float x = rs * ts / ls;
	float a = expf(-x);
	/* b = (1 - a) / R = ts / L * (1 - exp(-x)) / x, accurate for small x. */
	float b = (x > 0.0f) ? (ts / ls) * (-expm1f(-x) / x) : ts / ls;
	float z = expf(-FOC_2PI_F * bw_hz * ts);
	float g_i = 1.0f - z * z / a;
	float g_e = (1.0f - z) * (1.0f - z) / b;

	if (!pos_finite(a) || !pos_finite(b) || !is_finite(g_i) || !is_finite(g_e)) {
		return;
	}
	o->a = a;
	o->b = b;
	o->g_i = g_i;
	o->g_e = g_e;
	o->ts_half = 0.5f * ts;
	o->i_lim = i_lim;
	o->v_lim = v_lim;
}

struct foc_ab foc_obs_step(struct foc_obs *o, struct foc_ab i, struct foc_ab v, float omega)
{
	/* Half the rotation per period; NaN -> 0, at most pi per period. */
	struct foc_sincos sc = foc_sincos(foc_sanitizef(omega * o->ts_half, 0.5f * FOC_PI_F));
	struct foc_ab e_mid = rot(o->e, sc);
	struct foc_ab e_pred = rot(e_mid, sc);

	v = clamp_ab(v, o->v_lim);

	struct foc_ab i_pred = {
		.alpha = o->a * o->i.alpha + o->b * (v.alpha - e_mid.alpha),
		.beta = o->a * o->i.beta + o->b * (v.beta - e_mid.beta),
	};
	/* NaN measurement -> no correction. */
	struct foc_ab r = clamp_ab(
		(struct foc_ab){.alpha = i.alpha - i_pred.alpha, .beta = i.beta - i_pred.beta},
		2.0f * o->i_lim);

	o->err = r;
	o->i = clamp_ab((struct foc_ab){.alpha = i_pred.alpha + o->g_i * r.alpha,
					.beta = i_pred.beta + o->g_i * r.beta},
			o->i_lim);
	o->e = clamp_ab((struct foc_ab){.alpha = e_pred.alpha - o->g_e * r.alpha,
					.beta = e_pred.beta - o->g_e * r.beta},
			o->v_lim);
	return o->e;
}

void foc_duty_hist_init(struct foc_duty_hist *h, unsigned int delay)
{
	h->delay = (delay < FOC_DUTY_DELAY_MAX) ? delay : FOC_DUTY_DELAY_MAX;
	for (unsigned int k = 0; k <= FOC_DUTY_DELAY_MAX; k++) {
		h->d[k] = (struct foc_abc){.a = 0.5f, .b = 0.5f, .c = 0.5f};
	}
}
