/*
 * SPDX-License-Identifier: MIT
 *
 * Portable FOC math core. Plain C99, float32, no Zephyr dependencies.
 */

#ifndef PMSM_FOC_H_
#define PMSM_FOC_H_

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Clamp \p x to [\p lo, \p hi].
 *
 * Compiles to VMAXNM + VMINNM on FPv5 (Cortex-M33/M55/M7), branch-free.
 * A NaN \p x yields \p lo (IEEE 754 maxNum semantics).
 *
 * \pre lo <= hi, neither is NaN
 */
static inline float foc_clampf(float x, float lo, float hi)
{
	return fminf(fmaxf(x, lo), hi);
}

#ifdef __cplusplus
}
#endif

#endif /* PMSM_FOC_H_ */
