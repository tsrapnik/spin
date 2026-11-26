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
 *   foc_q_limit(), foc_svpwm_vmax(), foc_svpwm(), foc_pi_init(),
 *   foc_pi_step(), the gain functions, foc_current_init(),
 *   foc_current_step(), the duty history, the observer and the PLL.
 *   Inputs that are NaN map to the neutral value (0, or zero voltage),
 *   +-Inf saturates. See each function.
 * - foc_torque_constant() is a plain product and propagates NaN;
 *   foc_gains_speed() rejects a non-finite or non-positive Kt.
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
 * \return Kt in Nm/A (peak phase current); NaN or Inf if \p psi is.
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
 * comment); a NaN or too low \p vbus gives zero voltage. A NaN speed
 * counts as 0 (no advance); the advance omega * t_adv is clamped to
 * +-pi. \p theta is expected wrapped to [-pi, pi]: beyond
 * #FOC_SINCOS_MAX_ARG, or NaN, it counts as 0 (foc_sincos()).
 *
 * \param i_abc Measured phase currents, A.
 * \param vbus Measured DC-bus voltage, V.
 * \param theta Electrical angle of the d axis at the sample, rad, wrapped.
 * \param omega Electrical speed, rad/s (for the angle advance).
 * \param ref Current reference, A.
 * \return Duties for the next PWM period.
 */
struct foc_abc foc_current_step(struct foc_current *c, struct foc_abc i_abc, float vbus,
				float theta, float omega, struct foc_dq ref);

/** \brief Longest duty-to-sample delay foc_duty_hist supports, in PWM periods. */
#define FOC_DUTY_DELAY_MAX 2U

/**
 * \brief History of written duties, to pair each sample with the duty that
 *        was applied during the PWM period before it.
 *
 * A duty written in cycle k is applied \p delay periods later (1 on the
 * hardware: latched at the next update event). The period that ends at
 * sample k therefore ran on the duty written in cycle k - 1 - delay.
 *
 * Per cycle: foc_duty_hist_applied() before the observer step, then
 * foc_duty_hist_push() with the new duty.
 */
struct foc_duty_hist {
	struct foc_abc d[FOC_DUTY_DELAY_MAX + 1U]; /**< d[0] is the oldest. */
	unsigned int delay;                        /**< d[0..delay] are in use. */
};

/**
 * \brief Set the delay and fill the history with 0.5 (zero voltage).
 *
 * \p delay above #FOC_DUTY_DELAY_MAX is clamped.
 */
void foc_duty_hist_init(struct foc_duty_hist *h, unsigned int delay);

/** \brief Duty applied during the PWM period that ended at the latest sample. */
FOC_INLINE struct foc_abc foc_duty_hist_applied(const struct foc_duty_hist *h)
{
	return h->d[0];
}

/** \brief Record the duty written in this cycle. */
FOC_INLINE void foc_duty_hist_push(struct foc_duty_hist *h, struct foc_abc duty)
{
	/* Clamped again so the index is provably in range. */
	unsigned int last = (h->delay < FOC_DUTY_DELAY_MAX) ? h->delay : FOC_DUTY_DELAY_MAX;

	/* Shift the whole array (fixed bound); entries above last are unused. */
	for (unsigned int k = 0U; k < FOC_DUTY_DELAY_MAX; k++) {
		h->d[k] = h->d[k + 1U];
	}
	h->d[last] = duty;
}

/**
 * \brief Luenberger observer for current and back-EMF in the alpha/beta frame.
 *
 * Model over one PWM period (voltage v constant in the stationary frame,
 * back-EMF e rotating at the electrical speed omega):
 *
 *   i[k] = a i[k-1] + b (v - e_mid),   a = exp(-R ts / L), b = (1 - a) / R
 *   e[k] = Rot(omega ts) e[k-1],       e_mid = Rot(omega ts / 2) e[k-1]
 *
 * This is the exact zero-order-hold solution of L di/dt = v - R i - e with
 * the back-EMF taken at mid-period. Each step predicts with the model and
 * corrects with the prediction error r = i - i_pred:
 *
 *   i_hat = i_pred + g_i r,   e_hat = e_pred - g_e r
 *
 * With omega = 0 the error dynamics have the characteristic polynomial
 * z^2 - ((1 - g_i) a + 1 - g_e b) z + (1 - g_i) a; foc_obs_init() places
 * both roots at z = exp(-2 pi bw ts). The rotating EMF model removes the
 * steady-state angle lag at constant speed when omega is right.
 *
 * For a surface PMSM e = omega psi (-sin(theta), cos(theta)), so the PLL
 * (foc_pll) gets the angle from e_hat.
 *
 * All state stays finite and within the limits given to foc_obs_init().
 */
struct foc_obs {
	float a;           /**< Current-model pole exp(-R ts / L). */
	float b;           /**< Voltage-to-current gain (1 - a) / R, A/V. */
	float g_i;         /**< Current correction gain. */
	float g_e;         /**< Back-EMF correction gain, V/A. */
	float ts_half;     /**< Half the sample period, s. */
	float i_lim;       /**< Clamp for the estimated current, A. */
	float v_lim;       /**< Clamp for the voltage input and the EMF, V. */
	struct foc_ab i;   /**< Estimated current at the latest sample, A. */
	struct foc_ab e;   /**< Estimated back-EMF at the latest sample, V. */
	struct foc_ab err; /**< Prediction error i - i_pred of the latest step, A. */
};

/**
 * \brief Compute the model and gains from motor parameters; clear the state.
 *
 * Keep \p bw_hz well above R / (2 pi L) and the electrical frequency at
 * top speed, and well below 1 / (2 pi ts).
 *
 * \param rs Phase resistance in ohm (0 allowed).
 * \param ls Phase inductance in henry.
 * \param ts Sample period in seconds.
 * \param bw_hz Error-dynamics bandwidth (double pole) in Hz.
 * \param i_lim Current clamp, A; a few times the largest phase current.
 * \param v_lim Voltage clamp, V; at least the bus voltage.
 *
 * If an input is negative, zero where it must be positive, not finite, or
 * makes a coefficient non-finite, the observer is inert: all coefficients
 * are 0 and the estimates stay 0.
 */
void foc_obs_init(struct foc_obs *o, float rs, float ls, float ts, float bw_hz, float i_lim,
		  float v_lim);

/** \brief Clear the estimates, e.g. when the PWM is switched off. */
FOC_INLINE void foc_obs_reset(struct foc_obs *o)
{
	o->i = (struct foc_ab){0};
	o->e = (struct foc_ab){0};
	o->err = (struct foc_ab){0};
}

/**
 * \brief Run one observer step.
 *
 * A NaN current component counts as zero prediction error (the estimate
 * coasts on the model); the error is clamped to +-2 i_lim. NaN voltage
 * components count as 0, +-Inf saturates at v_lim. A NaN speed counts as
 * 0; the rotation per step is clamped to +-pi.
 *
 * \param i Measured current at this sample, see foc_clarke().
 * \param v Voltage applied during the period that ended at this sample,
 *        see foc_duty_hist_applied() and foc_duty_to_ab().
 * \param omega Electrical speed estimate, rad/s (usually foc_pll.omega).
 * \return Estimated back-EMF at this sample, always finite.
 */
struct foc_ab foc_obs_step(struct foc_obs *o, struct foc_ab i, struct foc_ab v, float omega);

/**
 * \brief Phase-locked loop that extracts angle and speed from the back-EMF.
 *
 * Per step the angle is advanced by omega * ts to the new sample, then the
 * EMF is turned into the rotor frame at that angle. For a surface PMSM
 * e_d = -|e| sin(theta - theta_hat) * sign(omega), so the angle error is
 *
 *   err = -e_d * sign(omega_hat) / max(|e|, e_min)  ~  sin(theta - theta_hat)
 *
 * normalised so the loop gain does not depend on speed. A PI turns it into
 * the speed estimate. Lock at theta_hat = theta + pi is unstable as long as
 * the sign of omega_hat is right.
 *
 * A phase detector alone can stay out of lock when omega_hat starts far
 * from omega (e.g. with the wrong sign), because the error then averages
 * to almost zero. A frequency detector aids it: with e_dq the EMF in the
 * frame of theta_hat, whose rotation per step is omega_hat ts,
 *
 *   f_err = (e_dq_prev x e_dq) / (max(|e|, e_min)^2 ts)
 *         ~ sin((omega - omega_hat) ts) / ts
 *
 * is the speed error whatever the angle error. It is added to the
 * integrator with the gain k_f = #FOC_PLL_FLL_RATIO * Kp. It is zero at
 * constant speed in lock (no bias), and falls off with |e|^2 below e_min
 * (no pull towards 0 without EMF). Below
 * |e| = e_min the gain falls off with |e|, which keeps noise at standstill
 * from driving the estimate.
 */
struct foc_pll {
	struct foc_pi pi;     /**< Angle error (rad) to electrical speed (rad/s). */
	float ts;             /**< Sample period, s. */
	float w_max;          /**< Speed estimate limit, rad/s electrical. */
	float e_min;          /**< EMF magnitude below which the gain is reduced, V. */
	float theta;          /**< Angle estimate at the latest sample, wrapped. */
	float omega;          /**< Speed estimate, rad/s electrical. */
	float err;            /**< Normalised angle error of the latest step, [-1, 1]. */
	float f_err;          /**< Speed error seen by the frequency aid, rad/s. */
	float k_f_ts;         /**< Frequency-aid gain times ts. */
	float inv_ts;         /**< 1 / ts, 1/s. */
	struct foc_dq e_prev; /**< EMF of the previous step, in its PLL frame. */
};

/** \brief Frequency-aid gain of the PLL relative to its Kp. */
#define FOC_PLL_FLL_RATIO 0.25f

/**
 * \brief PLL gains for a critically damped loop: Kp = 2 wn, Ki = wn^2.
 *
 * Closed loop (Kp s + Ki) / (s^2 + Kp s + Ki) with natural frequency
 * wn = 2 pi \p bw_hz. Steady-state angle error under a constant electrical
 * acceleration alpha is alpha / wn^2. Keep \p bw_hz well below the
 * observer bandwidth.
 *
 * \return Gains; both 0 if \p bw_hz is negative or not finite, or a result
 *         overflows.
 */
struct foc_pi_gains foc_gains_pll(float bw_hz);

/**
 * \brief Set gains and limits; angle, speed and integrator start at 0.
 *
 * A \p ts, \p w_max or \p e_min that is negative or not finite is replaced
 * by 0 (gains as in foc_pi_init()).
 *
 * \param w_max Speed estimate limit, rad/s electrical.
 * \param e_min EMF magnitude in volts below which the loop gain falls off.
 */
void foc_pll_init(struct foc_pll *p, struct foc_pi_gains g, float ts, float w_max, float e_min);

/**
 * \brief Preload angle and speed, e.g. from the forced angle at startup.
 *
 * \p theta is wrapped (NaN gives 0) and \p omega clamped to +-w_max (NaN
 * gives 0); the integrator takes \p omega.
 */
void foc_pll_reset(struct foc_pll *p, float theta, float omega);

/**
 * \brief Run one PLL step on the back-EMF estimate of this sample.
 *
 * Updates theta, omega and err; all stay finite. A non-finite \p e counts
 * as zero angle error (the estimate coasts at the last speed).
 */
void foc_pll_step(struct foc_pll *p, struct foc_ab e);

/**
 * \brief Drive state, shared by the per-cycle controller and the sequencer.
 *
 * STOP: PWM off. ALIGN: current on d at the forced angle 0. IF: current on
 * d at a forced angle ramped open loop (I/f), observer and PLL acquiring.
 * RUN: observer angle, speed loop closed. FAULT: PWM off, latched until a
 * STOP command.
 */
enum foc_state {
	FOC_STATE_STOP = 0,
	FOC_STATE_ALIGN,
	FOC_STATE_IF,
	FOC_STATE_RUN,
	FOC_STATE_FAULT,
};

/**
 * \name Fault bits of foc_status.faults
 * \{
 */
#define FOC_FAULT_OC  0x1U /**< A phase current beyond the trip level (or NaN). */
#define FOC_FAULT_OV  0x2U /**< Bus voltage above the maximum (or NaN). */
#define FOC_FAULT_UV  0x4U /**< Bus voltage below the minimum (or NaN). */
#define FOC_FAULT_EXT 0x8U /**< Raised by the caller, e.g. the hardware break input. */
/** \} */

/**
 * \brief Command from the sequencer (thread) to the per-cycle controller
 *        (ISR). Written only by foc_seq_step().
 */
struct foc_cmd {
	enum foc_state state; /**< Requested state; FAULT means PWM off. */
	struct foc_dq i_ref;  /**< Current reference, A, in the control frame. */
	float w_forced;       /**< Forced-angle speed (ALIGN, IF), rad/s electrical. */
	/** In IF: the PLL follows the forced angle instead of the observer. */
	unsigned char pll_hold;
};

/**
 * \brief Status from the per-cycle controller (ISR) to the sequencer
 *        (thread). Written only by foc_cycle_step(); always finite except
 *        \p i and \p vbus, which are the measurements as they came.
 */
struct foc_status {
	enum foc_state state; /**< State the controller is in. */
	unsigned int faults;  /**< FOC_FAULT_* bits latched since the last STOP. */
	float theta;          /**< PLL angle, rad electrical, wrapped. */
	float omega;          /**< PLL speed, rad/s electrical. */
	/** Forced angle minus PLL angle, wrapped: the rotor lag in I/f; 0 in RUN. */
	float delta;
	float e_mag;      /**< Back-EMF estimate magnitude, V. */
	struct foc_dq i;  /**< Measured current in the control frame, A. */
	struct foc_dq v;  /**< Voltage command in the control frame, V. */
	float vbus;       /**< Measured bus voltage, V. */
	unsigned int seq; /**< Incremented on every step. */
};

/**
 * \brief Settings of the per-cycle controller, in SI units and electrical
 *        radians.
 */
struct foc_cycle_cfg {
	float ts;            /**< PWM period, s. */
	float rs;            /**< Phase resistance, ohm. */
	float ls;            /**< Phase inductance, H. */
	float max_duty;      /**< See foc_svpwm(). */
	float t_adv;         /**< Angle advance, s (#FOC_ADVANCE_PERIODS * ts). */
	unsigned int delay;  /**< Duty-to-sample delay, periods, see foc_duty_hist. */
	float bw_current_hz; /**< Current-loop bandwidth. */
	float bw_obs_hz;     /**< Observer bandwidth. */
	float bw_pll_hz;     /**< PLL natural frequency. */
	float w_max;         /**< PLL and forced speed limit, rad/s electrical. */
	float e_min;         /**< PLL gain fall-off below this EMF, V. */
	float i_trip;        /**< Over-current trip level, A (peak phase). */
	float vbus_min;      /**< Under-voltage trip level, V. */
	float vbus_max;      /**< Over-voltage trip level, V. */
};

/**
 * \brief Per-PWM-cycle controller, run in the ADC interrupt.
 *
 * Owns everything that happens every cycle: fault checks, forced-angle
 * integration, observer + PLL, angle selection and the current loop. It
 * follows foc_cmd.state with these rules:
 *
 * - Leaving STOP resets the current loop, observer, PLL, duty history and
 *   the forced angle (to 0).
 * - In ALIGN, IF and RUN, the per-cycle checks run first. Any fault bit
 *   enters FAULT at once and the step returns zero voltage.
 * - FAULT is left only on a STOP command, which clears the fault bits.
 * - Entering RUN (the handover) rotates the current-PI integrators from the
 *   forced to the PLL frame, so the voltage vector does not jump.
 *
 * The observer runs in ALIGN, IF and RUN on the applied duty, the PLL
 * follows the forced angle while foc_cmd.pll_hold is set in ALIGN and IF.
 */
struct foc_cycle {
	struct foc_current cur;
	struct foc_obs obs;
	struct foc_pll pll;
	struct foc_duty_hist hist;
	float ts;
	float w_max;
	float i_trip, vbus_min, vbus_max;
	float theta_f;           /**< Forced angle, rad electrical, wrapped. */
	enum foc_state state;    /**< Current state. */
	unsigned int faults;     /**< Latched fault bits. */
	unsigned int ext;        /**< Pending bits from foc_cycle_trip(). */
	unsigned int cfg_faults; /**< Bits raised on every active cycle (bad settings). */
	unsigned int seq;        /**< Step counter. */
};

/**
 * \brief Compute gains and limits; start in STOP.
 *
 * Non-finite or out-of-range settings make the controller fail safe: a bad
 * \p i_trip, \p vbus_min or \p vbus_max trips on the first active cycle,
 * other settings are handled as in foc_current_init(), foc_obs_init() and
 * foc_pll_init().
 */
void foc_cycle_init(struct foc_cycle *c, const struct foc_cycle_cfg *cfg);

/**
 * \brief Run one PWM cycle.
 *
 * \param cmd Latest command; a state outside enum foc_state counts as STOP.
 *        References are clamped (current to +-i_trip, forced speed to
 *        +-w_max); NaN counts as 0.
 * \param i_abc Measured phase currents, A.
 * \param vbus Measured bus voltage, V.
 * \param st Filled with the status after this step.
 * \return Duties for the next period, in [1 - max_duty, max_duty]; 0.5 on
 *         all phases in STOP and FAULT. The caller switches the PWM off
 *         when st->state is STOP or FAULT.
 */
struct foc_abc foc_cycle_step(struct foc_cycle *c, const struct foc_cmd *cmd, struct foc_abc i_abc,
			      float vbus, struct foc_status *st);

/**
 * \brief Raise fault bits from outside, e.g. the hardware break interrupt.
 *
 * Takes effect at the next foc_cycle_step(), in the same context as that
 * call (no locking).
 */
FOC_INLINE void foc_cycle_trip(struct foc_cycle *c, unsigned int bits)
{
	c->ext |= bits;
}

/**
 * \brief Startup and speed sequencer, run in the motor thread.
 *
 * STOP --start--> ALIGN --t_align--> IF --|w| >= w_handover--> RUN, and
 * back to STOP on stop. A FAULT seen in the status latches until stop.
 *
 * - ALIGN: d current ramps 0 -> i_start over the first half of t_align,
 *   then holds, at the forced angle 0.
 * - IF: the forced speed ramps at accel_start in the start direction with
 *   i_start on d. The rotor lags the current vector by delta, with
 *   Kt i_start sin(delta) = load torque. The PLL follows the forced angle
 *   below w_handover / 2, then runs on the observer to lock before the
 *   switch.
 * - Handover at w_handover: the speed PI integrator is preloaded with the
 *   torque-producing part i_start sin(delta), the speed reference starts
 *   at the forced speed and d goes to 0.
 * - RUN: the speed reference ramps at accel towards the target; the speed
 *   PI sets i_q within +-i_max.
 *
 * Speeds here are mechanical rad/s. The direction is fixed at start: a
 * target of the other sign, or below w_handover, runs at w_handover
 * (reversal needs stop and start). The target is clamped to +-w_max.
 */
struct foc_seq_cfg {
	float ts;                /**< Step period, s. */
	unsigned int pole_pairs; /**< Number of pole pairs. */
	float j;                 /**< Inertia, kg*m^2 (speed-PI gains). */
	float psi;               /**< Flux linkage, Wb (speed-PI gains). */
	float bw_speed_hz;       /**< Speed-loop crossover. */
	float i_max;             /**< Speed-PI q-current limit, A. */
	float i_start;           /**< ALIGN and IF current, A. */
	float t_align;           /**< ALIGN duration, s. */
	float accel_start;       /**< IF ramp, rad/s^2. */
	float w_handover;        /**< Switch to the observer, rad/s. */
	float accel;             /**< RUN reference ramp, rad/s^2. */
	float w_max;             /**< Largest target speed, rad/s. */
};

/** \brief Sequencer state, see struct foc_seq_cfg. */
struct foc_seq {
	struct foc_pi speed;
	float ts, pp, i_max, i_start, t_align, accel_start, w_handover, accel, w_max;
	enum foc_state state;
	unsigned int faults; /**< Fault bits that caused FAULT. */
	float dir;           /**< +1 or -1, fixed at start. */
	float target;        /**< Requested speed, rad/s mechanical. */
	float w_ref;         /**< Ramped speed reference, rad/s mechanical. */
	float w_forced;      /**< Forced speed, rad/s mechanical. */
	float t;             /**< Time in the current state, s. */
	unsigned char run;   /**< Start requested and no stop since. */
	struct foc_cmd cmd;  /**< Last command written. */
};

/**
 * \brief Compute the speed-PI gains, clamp the settings; start in STOP.
 *
 * Negative or non-finite settings are replaced by 0, which keeps the motor
 * from starting usefully but never yields non-finite commands.
 */
void foc_seq_init(struct foc_seq *s, const struct foc_seq_cfg *cfg);

/**
 * \brief Request a start towards \p w (rad/s mechanical, sign = direction).
 *
 * Ignored unless in STOP; NaN counts as 0 (positive direction). ALIGN
 * begins at the first step that sees the controller in STOP.
 */
void foc_seq_start(struct foc_seq *s, float w);

/** \brief Change the target speed; see struct foc_seq_cfg for the clamping. */
void foc_seq_set_speed(struct foc_seq *s, float w);

/** \brief Request a stop; also clears a FAULT. Takes effect at the next step. */
FOC_INLINE void foc_seq_stop(struct foc_seq *s)
{
	s->run = 0U;
}

/**
 * \brief Run one sequencer step on the latest status and write the command.
 *
 * \return The command, also kept in s->cmd; always finite.
 */
const struct foc_cmd *foc_seq_step(struct foc_seq *s, const struct foc_status *st);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_FOC_H_ */
