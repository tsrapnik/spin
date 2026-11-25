/*
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>

#include <pmsm/foc.h>

/*
 * pi/2 split into three parts (Cody-Waite). The first two have at most 12
 * significant bits, so k * part is exact for |k| < 2^11, i.e. |x| < 3200.
 */
#define PIO2_HI  1.5703125f
#define PIO2_MID 4.837512969970703e-4f
#define PIO2_LO  7.549790126404332e-8f
#define INV_PIO2 0.636619772f

/* Taylor coefficients; on |r| <= pi/4 the truncation error is < 3e-8. */
#define S3 (-1.0f / 6.0f)
#define S5 (1.0f / 120.0f)
#define S7 (-1.0f / 5040.0f)
#define S9 (1.0f / 362880.0f)
#define C2 (-1.0f / 2.0f)
#define C4 (1.0f / 24.0f)
#define C6 (-1.0f / 720.0f)
#define C8 (1.0f / 40320.0f)

struct foc_sincos foc_sincos(float x)
{
	/* NaN, +-Inf and out-of-range arguments fail this test: use 0. */
	if (!(fabsf(x) <= FOC_SINCOS_MAX_ARG)) {
		x = 0.0f;
	}

	/* roundf() is VRINTA on FPv5; |kf| <= 1910, so the cast is safe. */
	float kf = roundf(x * INV_PIO2);
	int32_t k = (int32_t)kf;
	float r = ((x - kf * PIO2_HI) - kf * PIO2_MID) - kf * PIO2_LO;
	float r2 = r * r;
	float s = r + r * r2 * (S3 + r2 * (S5 + r2 * (S7 + r2 * S9)));
	float c = 1.0f + r2 * (C2 + r2 * (C4 + r2 * (C6 + r2 * C8)));

	/* x = k * pi/2 + r: rotate (sin r, cos r) by k quarter turns. */
	if (k & 1) {
		float tmp = s;

		s = c;
		c = -tmp;
	}
	if (k & 2) {
		s = -s;
		c = -c;
	}

	return (struct foc_sincos){.sin = s, .cos = c};
}

struct foc_dq foc_limit_circle(struct foc_dq v, float vmax)
{
	float lim = foc_clampf(vmax, 0.0f, FLT_MAX);
	float d = foc_sanitizef(v.d, lim);
	float qmax = foc_q_limit(d, lim);

	return (struct foc_dq){.d = d, .q = foc_sanitizef(v.q, qmax)};
}
