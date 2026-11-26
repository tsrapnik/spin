/*
 * SPDX-License-Identifier: MIT
 *
 * Closed-loop harness: plant + FOC core with the true rotor angle. The
 * current loop runs every PWM cycle, the speed PI every speed_div cycles
 * (the 1 kHz motor thread), as decided in docs/PLAN.md.
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
};

struct sim {
	struct plant p;
	struct sim_cfg cfg;
	struct foc_current cur;
	struct foc_pi speed;
	bool speed_loop;     /**< false: i_ref is set directly. */
	float w_ref;         /**< Mechanical speed reference, rad/s. */
	struct foc_dq i_ref; /**< Current reference, A (speed PI writes q). */
	unsigned int k;      /**< PWM cycle count. */
};

/** \brief FIT1036 estimates from docs/HARDWARE.md, 24 V, 20 kHz, delay 1. */
extern const struct plant_params sim_motor;
/** \brief Default controller: 500 Hz current, 50 Hz speed, 1.4 A, 0.95, 1 kHz. */
extern const struct sim_cfg sim_default_cfg;

void sim_init(struct sim *s, const struct plant_params *prm, const struct sim_cfg *cfg);
/** \brief One PWM cycle: sample, (speed PI), current loop, plant step. */
void sim_cycle(struct sim *s);
/** \brief Run \p n PWM cycles. */
void sim_run(struct sim *s, unsigned int n);
/** \brief PWM cycles in \p t seconds. */
unsigned int sim_cycles(const struct sim *s, float t);

#endif /* PMSM_SIM_SIM_H_ */
