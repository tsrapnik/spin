/*
 * SPDX-License-Identifier: MIT
 *
 * Closed-loop harness: plant + FOC core. The current loop runs every PWM
 * cycle, the speed PI every speed_div cycles (the 1 kHz motor thread), as
 * decided in docs/PLAN.md. The observer and PLL run every cycle on the
 * sampled current and the applied duty; with sensorless = false the loops
 * use the true angle and speed, otherwise the estimates.
 */

#ifndef PMSM_SIM_SIM_H_
#define PMSM_SIM_SIM_H_

#include <stdbool.h>
#include <pmsm/foc.h>

#include "plant.h"

/** \brief Controller settings. */
struct sim_cfg {
	float bw_current_hz;    /**< Current-loop bandwidth. */
	float bw_speed_hz;      /**< Speed-loop crossover. */
	float i_max;            /**< q-current limit of the speed PI, A. */
	float max_duty;         /**< SVPWM duty limit. */
	unsigned int speed_div; /**< PWM cycles per speed-PI step. */
	float adv_periods;      /**< Angle advance, PWM periods. */
	float bw_obs_hz;        /**< Observer bandwidth. */
	float bw_pll_hz;        /**< PLL natural frequency. */
	float w_max_e;          /**< PLL speed limit, rad/s electrical. */
	float e_min;            /**< PLL gain fall-off below this EMF, V. */
	unsigned int obs_delay; /**< Duty delay the observer assumes. */
	float obs_rs_scale;     /**< Observer Rs / plant Rs (0 counts as 1). */
	float obs_ls_scale;     /**< Observer Ls / plant Ls (0 counts as 1). */
};

struct sim {
	struct plant p;
	struct sim_cfg cfg;
	struct foc_current cur;
	struct foc_pi speed;
	struct foc_obs obs;
	struct foc_pll pll;
	struct foc_duty_hist hist;
	bool sensorless;     /**< Control with the estimated angle and speed. */
	bool speed_loop;     /**< false: i_ref is set directly. */
	float w_ref;         /**< Mechanical speed reference, rad/s. */
	struct foc_dq i_ref; /**< Current reference, A (speed PI writes q). */
	unsigned int k;      /**< PWM cycle count. */
	double theta_smp;    /**< True angle at the latest sample. */
};

/** \brief FIT1036 estimates from docs/HARDWARE.md, 24 V, 20 kHz, delay 1. */
extern const struct plant_params sim_motor;
/**
 * \brief Default controller: 500 Hz current, 50 Hz speed, 1.4 A, 0.95, 1 kHz;
 *        observer 2 kHz, PLL 150 Hz.
 */
extern const struct sim_cfg sim_default_cfg;

void sim_init(struct sim *s, const struct plant_params *prm, const struct sim_cfg *cfg);
/** \brief One PWM cycle: sample, (speed PI), current loop, plant step. */
void sim_cycle(struct sim *s);
/** \brief Run \p n PWM cycles. */
void sim_run(struct sim *s, unsigned int n);
/** \brief PWM cycles in \p t seconds. */
unsigned int sim_cycles(const struct sim *s, float t);
/** \brief PLL angle minus true angle at the latest sample, wrapped to [-pi, pi]. */
double sim_angle_err(const struct sim *s);
/** \brief PLL speed estimate, rad/s mechanical. */
double sim_w_est(const struct sim *s);

#endif /* PMSM_SIM_SIM_H_ */
