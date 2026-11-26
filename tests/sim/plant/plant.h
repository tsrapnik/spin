/*
 * SPDX-License-Identifier: MIT
 *
 * Discrete surface-PMSM plant with an ideal three-phase inverter, for the
 * closed-loop simulation. Plain C99, no Zephyr dependencies, deterministic.
 *
 * Model (rotor dq frame, same conventions as pmsm/foc.h: amplitude-invariant
 * Clarke, d at theta_e, q leads by 90 degrees, SI units):
 *
 *   L did/dt = vd - Rs id + we L iq
 *   L diq/dt = vq - Rs iq - we L id - we psi
 *   J dwm/dt = Te - B wm - t_load,   Te = 1.5 p psi iq
 *   dtheta_e/dt = we = p wm
 *
 * Inverter: v_x = (d_x - mean(d)) * vbus, held constant in the stationary
 * frame for one PWM period (cycle-averaged, no dead time or switch drops).
 * The duty passed to plant_step() is applied #plant_params.duty_delay
 * periods later, as on the real hardware where a duty written in the ISR is
 * latched at the next update event.
 *
 * The state is integrated in double precision with RK4, so the plant is an
 * accurate reference for the float32 controller under test.
 *
 * Per PWM cycle the caller does:
 *
 *   struct plant_sample s = plant_sample(&p);   (ADC sample)
 *   duty = controller(s);
 *   plant_step(&p, duty);                       (one PWM period)
 */

#ifndef PMSM_SIM_PLANT_H_
#define PMSM_SIM_PLANT_H_

#include <pmsm/foc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** \brief pi in double precision (M_PI is not standard C). */
#define PLANT_PI 3.14159265358979323846

/**
 * \brief Longest supported duty delay, in PWM periods.
 *
 * The hardware has 1 (duty latched at the next update event); 2 covers a
 * latch that misses one update and allows robustness tests at twice the
 * expected delay.
 */
#define PLANT_DUTY_DELAY_MAX 2U

/** \brief Plant and inverter parameters. */
struct plant_params {
	float rs;                /**< Phase resistance, ohm. */
	float ls;                /**< Phase inductance (Ld = Lq), henry. */
	float psi;               /**< PM flux linkage (peak, per phase), Wb. */
	unsigned int pole_pairs; /**< Number of pole pairs. */
	float j;                 /**< Rotor plus load inertia, kg*m^2. */
	float b;                 /**< Viscous friction, Nm*s/rad. */
	float vbus;              /**< DC-bus voltage, V. */
	float ts;                /**< PWM period, s. */
	unsigned int duty_delay; /**< PWM periods from plant_step() to applied duty. */
	unsigned int substeps;   /**< RK4 steps per PWM period; 0 counts as 1. */
};

/**
 * \brief Plant state. The state fields may be read, and set between steps
 *        (initial conditions, load and bus-voltage steps).
 */
struct plant {
	/* Parameters, converted once by plant_init(). */
	double rs, ls, psi, j, b, ts;
	double pp;
	unsigned int duty_delay;
	unsigned int substeps;

	/* Inputs. */
	double t_load; /**< External load torque, Nm; positive opposes positive speed. */
	double vbus;   /**< DC-bus voltage, V. */

	/* State. */
	double id, iq;    /**< Rotor-frame currents, A. */
	double wm;        /**< Mechanical speed, rad/s. */
	double theta_e;   /**< Electrical angle, rad, kept in [-pi, pi]. */
	double t;         /**< Simulated time, s. */
	struct foc_ab va; /**< Stationary-frame voltage applied during the last step. */

	/* Duties waiting to be applied; hist[0] is applied next. */
	struct foc_abc hist[PLANT_DUTY_DELAY_MAX + 1U];
	struct foc_abc applied; /**< Duty applied during the last step. */
};

/** \brief Current and bus-voltage sample, as the power-stage driver delivers it. */
struct plant_sample {
	float ia, ib, ic;
	float vbus;
};

/**
 * \brief Initialise the plant at standstill, zero current, theta_e = 0.
 *
 * Pending duties start at 0.5 (zero voltage).
 *
 * \pre \p prm has positive rs, ls, j, ts and pole_pairs, non-negative psi and
 *      b, and duty_delay <= #PLANT_DUTY_DELAY_MAX
 * \return 0, or -1 if a precondition does not hold.
 */
int plant_init(struct plant *p, const struct plant_params *prm);

/** \brief Phase currents and bus voltage at the current instant. */
struct plant_sample plant_sample(const struct plant *p);

/**
 * \brief Queue \p duty and advance the plant by one PWM period.
 *
 * The duty applied during this period is the one passed
 * #plant_params.duty_delay calls earlier (or \p duty itself for delay 0).
 * Duties are clamped to [0, 1].
 */
void plant_step(struct plant *p, struct foc_abc duty);

/** \brief Electromagnetic torque at the current instant, Nm. */
double plant_torque(const struct plant *p);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_SIM_PLANT_H_ */
