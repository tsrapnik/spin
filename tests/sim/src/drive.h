/*
 * SPDX-License-Identifier: MIT
 *
 * Full drive on the plant: foc_cycle every PWM cycle (the ISR), foc_seq every
 * div cycles (the 1 kHz motor thread), exchanging foc_cmd and foc_status as
 * in the motor device. Deterministic: no threads or timers.
 */

#ifndef PMSM_SIM_DRIVE_H_
#define PMSM_SIM_DRIVE_H_

#include <stdbool.h>
#include <pmsm/foc.h>

#include "plant.h"

struct drive {
	struct plant p;
	struct foc_cycle c;
	struct foc_seq s;
	struct foc_status st;
	struct foc_cmd cmd;   /**< Latest command, as the ISR sees it. */
	unsigned int div;     /**< PWM cycles per sequencer step. */
	unsigned int k;       /**< PWM cycle count. */
	double theta_smp;     /**< True angle at the latest sample. */
	bool pwm;             /**< Power stage on (state ALIGN, IF or RUN). */
	struct foc_abc i_add; /**< Added to the sampled currents (fault injection). */
};

/** \brief Controller settings for the FIT1036 estimates (see sim_motor). */
extern const struct foc_cycle_cfg drive_cycle_cfg;
extern const struct foc_seq_cfg drive_seq_cfg;

void drive_init(struct drive *d, const struct plant_params *prm, const struct foc_cycle_cfg *cc,
		const struct foc_seq_cfg *sc);
/** \brief One PWM cycle; with the PWM off the plant sees open phases (no current). */
void drive_cycle(struct drive *d);
void drive_run(struct drive *d, float t);
/** \brief Run until the sequencer is in \p st, at most \p t_max; returns the time taken or -1. */
double drive_until(struct drive *d, enum foc_state st, float t_max);
/** \brief PLL angle minus true angle at the latest sample, wrapped. */
double drive_angle_err(const struct drive *d);

#endif /* PMSM_SIM_DRIVE_H_ */
