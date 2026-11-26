/*
 * SPDX-License-Identifier: MIT
 *
 * Portable FOC math core. Plain C99, float32, no Zephyr dependencies.
 *
 * Conventions:
 * - Amplitude-invariant Clarke transform: a balanced set of amplitude A
 *   maps to an alpha/beta vector of magnitude A.
 * - Park: d axis at electrical angle theta, q leads d by 90 degrees.
 * - Angles in electrical radians; speeds in rad/s; SI units throughout.
 * - Duties are 0..1, 0.5 = zero phase voltage.
 *
 * The small transforms are static inline so they cost nothing in the
 * per-PWM-cycle ISR. Small aggregates of up to four floats are passed and
 * returned in FPU registers under the ARM hard-float ABI.
 *
 * Non-finite values (NaN, +-Inf) and overflow:
 * - Functions that keep state, set limits or produce duties never return
 *   NaN or Inf: foc_wrap_pi(), foc_sincos(), foc_limit_circle(),
 *   foc_q_limit(), foc_svpwm_vmax(), foc_svpwm(), foc_pi_step() and the
 *   gain functions. Inputs that are NaN map to the neutral value (0, or
 *   zero voltage), +-Inf saturates. See each function.
 * - The linear transforms (Clarke, Park, their inverses, foc_duty_to_ab())
 *   propagate NaN and Inf on purpose. A NaN there comes from a broken
 *   measurement, and replacing it with 0 would hide the fault (a current
 *   read as 0 A makes the current loop raise the voltage). The per-cycle
 *   fault checks must catch it instead; write them so that NaN fails,
 *   e.g. `!(vbus >= min && vbus <= max)`, or use isfinite().
 */

#ifndef PMSM_FOC_H_
#define PMSM_FOC_H_

#include <float.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Helpers used in the per-cycle path. Forced inline: at -Os (Zephyr's
 * default) GCC otherwise keeps some of them as out-of-line calls.
 */
#if defined(__GNUC__) || defined(__clang__)
#define FOC_INLINE static inline __attribute__((always_inline))
#else
#define FOC_INLINE static inline
#endif

#define FOC_PI_F        3.14159265f
#define FOC_2PI_F       6.28318531f
#define FOC_INV_2PI_F   0.159154943f
#define FOC_SQRT3_F     1.73205081f
#define FOC_INV_SQRT3_F 0.577350269f
#define FOC_SQRT3_2_F   0.866025404f

/** \brief Three-phase quantity (currents, voltages or duties). */
struct foc_abc {
	float a;
	float b;
	float c;
};

/** \brief Stationary-frame vector. */
struct foc_ab {
	float alpha;
	float beta;
};

/** \brief Rotating-frame vector. */
struct foc_dq {
	float d;
	float q;
};

/** \brief Sine and cosine of one angle. */
struct foc_sincos {
	float sin;
	float cos;
};

/**
 * \brief Clamp \p x to [\p lo, \p hi].
 *
 * Compiles to VMAXNM + VMINNM on FPv5 (Cortex-M33/M55/M7), branch-free.
 * A NaN \p x yields \p lo (IEEE 754 maxNum semantics).
 *
 * \pre lo <= hi, neither is NaN
 */
FOC_INLINE float foc_clampf(float x, float lo, float hi)
{
	return fminf(fmaxf(x, lo), hi);
}

/**
 * \brief Replace NaN with 0 and clamp to [-\p lim, \p lim].
 *
 * \pre lim >= 0, not NaN
 */
FOC_INLINE float foc_sanitizef(float x, float lim)
{
	return (x == x) ? foc_clampf(x, -lim, lim) : 0.0f;
}

/**
 * \brief Wrap an angle to [-pi, pi].
 *
 * The error grows with |x| / 2pi (about 1e-6 rad per turn removed), so this
 * is meant for angles that are wrapped every cycle and are therefore at most
 * a little outside the range. Always returns a value in [-pi, pi]; NaN and
 * +-Inf give 0. roundf() compiles to VRINTA on FPv5.
 */
FOC_INLINE float foc_wrap_pi(float x)
{
	float r = x - FOC_2PI_F * roundf(x * FOC_INV_2PI_F);

	return foc_sanitizef(r, FOC_PI_F);
}

/**
 * \brief Sine and cosine of \p x, computed together.
 *
 * Range reduction to [-pi/4, pi/4] plus Taylor polynomials. Absolute error
 * below 1e-6 for |x| <= #FOC_SINCOS_MAX_ARG. No libm calls, no tables.
 * Both results are always in [-1, 1]. NaN, +-Inf and |x| above
 * #FOC_SINCOS_MAX_ARG are treated as 0 (sin = 0, cos = 1).
 */
struct foc_sincos foc_sincos(float x);

/** \brief Largest |x| that foc_sincos() reduces exactly. */
#define FOC_SINCOS_MAX_ARG 3000.0f

/**
 * \brief Clarke transform (amplitude-invariant), three-input form.
 *
 * Any common-mode part of \p x is ignored, so this also turns duties into
 * line-neutral voltages.
 */
FOC_INLINE struct foc_ab foc_clarke(struct foc_abc x)
{
	return (struct foc_ab){
		.alpha = (2.0f * x.a - x.b - x.c) * (1.0f / 3.0f),
		.beta = (x.b - x.c) * FOC_INV_SQRT3_F,
	};
}

/** \brief Inverse Clarke transform; the result has zero common mode. */
FOC_INLINE struct foc_abc foc_inv_clarke(struct foc_ab x)
{
	float h = -0.5f * x.alpha;
	float k = FOC_SQRT3_2_F * x.beta;

	return (struct foc_abc){.a = x.alpha, .b = h + k, .c = h - k};
}

/** \brief Park transform: stationary to rotating frame. */
FOC_INLINE struct foc_dq foc_park(struct foc_ab x, struct foc_sincos sc)
{
	return (struct foc_dq){
		.d = x.alpha * sc.cos + x.beta * sc.sin,
		.q = x.beta * sc.cos - x.alpha * sc.sin,
	};
}

/** \brief Inverse Park transform: rotating to stationary frame. */
FOC_INLINE struct foc_ab foc_inv_park(struct foc_dq x, struct foc_sincos sc)
{
	return (struct foc_ab){
		.alpha = x.d * sc.cos - x.q * sc.sin,
		.beta = x.d * sc.sin + x.q * sc.cos,
	};
}

/**
 * \brief Limit a dq voltage to a circle of radius \p vmax, d axis first.
 *
 * d is clamped to +-vmax, q gets what is left. Use the same split for the
 * output limits of the d and q current PIs (see foc_q_limit()) so their
 * anti-windup sees the real limit.
 *
 * A NaN component becomes 0, +-Inf saturates. A NaN or negative \p vmax
 * counts as 0.
 */
struct foc_dq foc_limit_circle(struct foc_dq v, float vmax);

/**
 * \brief Room left for the q component once d is set: sqrt(vmax^2 - d^2).
 *
 * Returns 0 when |\p d| >= \p vmax, or when either argument is NaN.
 * Never Inf: the result is capped at FLT_MAX.
 */
FOC_INLINE float foc_q_limit(float d, float vmax)
{
	return fminf(sqrtf(fmaxf(vmax * vmax - d * d, 0.0f)), FLT_MAX);
}

/**
 * \brief Largest phase-voltage vector magnitude SVPWM can produce linearly.
 *
 * \param vbus DC-bus voltage.
 * \param max_duty Highest allowed duty (below 1 to leave a low-side window
 *        for current sampling); the lowest duty is 1 - max_duty.
 * \return Limit in [0, FLT_MAX]; NaN or negative inputs give 0.
 */
FOC_INLINE float foc_svpwm_vmax(float vbus, float max_duty)
{
	float m = foc_clampf(max_duty, 0.5f, 1.0f);

	return foc_clampf((2.0f * m - 1.0f) * vbus * FOC_INV_SQRT3_F, 0.0f, FLT_MAX);
}

/**
 * \brief Space-vector PWM via min/max zero-sequence injection.
 *
 * Duties are centred on 0.5 and clamped to [1 - max_duty, max_duty].
 * Inside the circle given by foc_svpwm_vmax() no clamping occurs and
 * foc_duty_to_ab() returns \p v exactly.
 *
 * Always returns duties in [0, 1]:
 * - a NaN component of \p v counts as 0, +-Inf saturates;
 * - \p vbus below #FOC_SVPWM_VBUS_MIN (or NaN) gives 0.5 on all phases,
 *   i.e. zero voltage;
 * - \p max_duty is clamped to [0.5, 1]; NaN gives 0.5 (zero voltage).
 */
struct foc_abc foc_svpwm(struct foc_ab v, float vbus, float max_duty);

/** \brief Lowest bus voltage foc_svpwm() modulates with, in volts. */
#define FOC_SVPWM_VBUS_MIN 0.1f

/**
 * \brief Applied alpha/beta voltage for given duties: Clarke(d) * Vbus.
 *
 * Equals v_x = (d_x - mean(d)) * Vbus followed by Clarke, since Clarke
 * ignores the common mode.
 */
FOC_INLINE struct foc_ab foc_duty_to_ab(struct foc_abc duty, float vbus)
{
	struct foc_ab v = foc_clarke(duty);

	return (struct foc_ab){.alpha = v.alpha * vbus, .beta = v.beta * vbus};
}

/**
 * \brief Discrete PI controller with clamping anti-windup.
 *
 * Forward-Euler integrator. Limits are passed on every step so they may
 * change from cycle to cycle (e.g. the q-axis voltage limit).
 */
struct foc_pi {
	float kp;    /**< Proportional gain. */
	float ki_ts; /**< Integral gain times the sample period. */
	float integ; /**< Integrator state, in output units. */
};

/** \brief Continuous-time PI gains. */
struct foc_pi_gains {
	float kp;
	float ki;
};

/**
 * \brief Set gains and clear the integrator.
 *
 * A gain or \p ts that is negative or not finite is replaced by 0, as is a
 * product ki * ts that overflows.
 *
 * \param ts Sample period in seconds.
 */
void foc_pi_init(struct foc_pi *pi, struct foc_pi_gains g, float ts);

/**
 * \brief Set the integrator, e.g. to preload the speed PI at handover.
 *
 * NaN becomes 0, +-Inf becomes +-FLT_MAX; the next foc_pi_step() clamps it
 * to its limits.
 */
FOC_INLINE void foc_pi_reset(struct foc_pi *pi, float integ)
{
	pi->integ = foc_sanitizef(integ, FLT_MAX);
}

/**
 * \brief Run one PI step.
 *
 * The integrator moves towards a limit only until the output reaches it,
 * never further; if the proportional part alone saturates the output, the
 * integrator holds. It is always kept inside [\p lo, \p hi]. Branch-free.
 *
 * A NaN \p err counts as 0 (integrator holds), +-Inf saturates. Output and
 * integrator stay finite as long as the limits are finite, which the gain
 * functions and foc_pi_init() guarantee for the gains.
 *
 * \param err Reference minus measurement.
 * \return Output clamped to [\p lo, \p hi].
 * \pre lo <= hi, neither is NaN
 */
float foc_pi_step(struct foc_pi *pi, float err, float lo, float hi);

/**
 * \brief Current PI gains by pole cancellation: Kp = L*wc, Ki = R*wc.
 *
 * The PI zero cancels the R/L pole, leaving a first-order closed loop with
 * bandwidth \p bw_hz. Keep \p bw_hz well below the PWM frequency (about
 * 1/20) for the loop delay to stay negligible.
 *
 * \param rs Phase resistance in ohm.
 * \param ls Phase inductance in henry.
 * \param bw_hz Closed-loop bandwidth in Hz.
 * \return Gains; both 0 if an input is negative or not finite, or a result
 *         overflows.
 */
struct foc_pi_gains foc_gains_current(float rs, float ls, float bw_hz);

/** \brief Ratio of speed-loop crossover to PI zero (phase margin ~76 deg). */
#define FOC_SPEED_PI_ZERO_RATIO 4.0f

/**
 * \brief Speed PI gains for the plant Kt / (J*s), output in amperes q-axis.
 *
 * Kp = J*wc/Kt puts the crossover at wc; the PI zero sits at
 * wc / #FOC_SPEED_PI_ZERO_RATIO. The input is mechanical speed in rad/s.
 * Keep \p bw_hz well below the current-loop bandwidth (about 1/10).
 *
 * \param j Rotor (plus load) inertia in kg*m^2.
 * \param kt Torque constant in Nm/A, see foc_torque_constant().
 * \param bw_hz Crossover frequency in Hz.
 * \return Gains; both 0 if \p kt is not positive, an input is negative or
 *         not finite, or a result overflows.
 */
struct foc_pi_gains foc_gains_speed(float j, float kt, float bw_hz);

/**
 * \brief Torque constant of a surface PMSM: Kt = 1.5 * p * psi.
 *
 * \param psi Permanent-magnet flux linkage in Wb (peak, per phase).
 * \param pole_pairs Number of pole pairs.
 */
FOC_INLINE float foc_torque_constant(float psi, unsigned int pole_pairs)
{
	return 1.5f * (float)pole_pairs * psi;
}

/**
 * \brief d/q current controller: Park, two PIs, voltage limit, SVPWM.
 *
 * The d PI is limited to +-vmax and the q PI to foc_q_limit(vd, vmax), with
 * vmax = foc_svpwm_vmax(vbus, max_duty), so both anti-windups see the real
 * circle and d has priority. No decoupling feed-forward: the PI treats the
 * cross-coupling and the back-EMF as disturbances.
 *
 * The inverse Park uses the angle advanced by omega * t_adv. The duty
 * written now acts during the next PWM period, whose centre lies 1.5
 * periods after the sample, so t_adv = 1.5 * ts compensates the rotation
 * during that delay.
 */
struct foc_current {
	struct foc_pi d; /**< d-axis PI, output in volts. */
	struct foc_pi q; /**< q-axis PI, output in volts. */
	float max_duty;  /**< Highest duty, see foc_svpwm(). */
	float t_adv;     /**< Angle advance time for the inverse Park, s. */
	struct foc_dq i; /**< Measured current of the last step (may be NaN). */
	struct foc_dq v; /**< Voltage command of the last step, always finite. */
};

/** \brief Angle advance for a one-period duty delay, in PWM periods. */
#define FOC_ADVANCE_PERIODS 1.5f

/**
 * \brief Set gains, limits and advance; clear the integrators.
 *
 * \param g Current PI gains, see foc_gains_current(); used for d and q.
 * \param ts Sample period in seconds.
 * \param max_duty Highest duty, clamped to [0.5, 1]; NaN gives 0.5 (no
 *        voltage).
 * \param t_adv Angle advance time in seconds, usually
 *        #FOC_ADVANCE_PERIODS * \p ts; negative or not finite gives 0.
 */
void foc_current_init(struct foc_current *c, struct foc_pi_gains g, float ts, float max_duty,
		      float t_adv);

/** \brief Clear both integrators, e.g. when the PWM is switched off. */
FOC_INLINE void foc_current_reset(struct foc_current *c)
{
	foc_pi_reset(&c->d, 0.0f);
	foc_pi_reset(&c->q, 0.0f);
	c->v = (struct foc_dq){0};
}

/**
 * \brief Run one current-loop step.
 *
 * Always returns duties in [1 - max_duty, max_duty]. A NaN current makes
 * the PIs hold (the per-cycle fault check must catch it, see the header
 * comment); a NaN angle or speed counts as 0; a NaN or too low \p vbus
 * gives zero voltage.
 *
 * \param i_abc Measured phase currents, A.
 * \param vbus Measured DC-bus voltage, V.
 * \param theta Electrical angle of the d axis at the sample, rad.
 * \param omega Electrical speed, rad/s (for the angle advance).
 * \param ref Current reference, A.
 * \return Duties for the next PWM period.
 */
struct foc_abc foc_current_step(struct foc_current *c, struct foc_abc i_abc, float vbus,
				float theta, float omega, struct foc_dq ref);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_FOC_H_ */
