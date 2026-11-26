/*
 * SPDX-License-Identifier: MIT
 *
 * Extras of the simulated power stage (compatible "pmsm,sim-power-stage"):
 * access to the plant, fault injection and manual stepping.
 *
 * Each PWM period the driver samples the plant (plus the injected sense
 * offsets, minus the calibrated ones), calls the callback, then advances
 * the plant by one period: plant_step() with the latest duty if the
 * outputs are on, plant_coast() otherwise.
 *
 * Cycle source: with tick-period-us > 0 in the devicetree a kernel timer
 * runs, in its interrupt, the cycles due since the previous expiry, so
 * simulated time follows the kernel clock (in batches of at least one
 * kernel tick: 200 cycles at 20 kHz with native_sim's 100 Hz tick). With
 * tick-period-us = 0 nothing runs until power_stage_sim_run().
 */

#ifndef PMSM_POWER_STAGE_SIM_H_
#define PMSM_POWER_STAGE_SIM_H_

#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <pmsm/plant.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Run \p n PWM cycles now, in the caller's context.
 *
 * The callback runs in the caller's context too.
 *
 * \retval 0 Done.
 * \retval -EBUSY The device runs on its timer (tick-period-us > 0).
 */
int power_stage_sim_run(const struct device *dev, unsigned int n);

/**
 * \brief Lock the simulation and return the plant.
 *
 * Between this call and power_stage_sim_unlock() no cycle runs, so the
 * plant state and inputs (load, friction, bus voltage, speed, angle) can be
 * read and changed consistently. Do not call driver functions while
 * locked. Not needed with tick-period-us = 0 when only one thread uses the
 * device.
 */
struct plant *power_stage_sim_lock(const struct device *dev, k_spinlock_key_t *key);

/** \brief Release the lock taken by power_stage_sim_lock(). */
void power_stage_sim_unlock(const struct device *dev, k_spinlock_key_t key);

/**
 * \brief Add a current-sense offset to every raw sample, A per phase.
 *
 * Models an ADC or amplifier offset; power_stage_calibrate_offsets()
 * measures it.
 */
void power_stage_sim_set_offset(const struct device *dev, float a, float b, float c);

/**
 * \brief Drive the simulated hardware break input.
 *
 * \param active true fires the break: the outputs go off now, the next
 *        sample carries #POWER_STAGE_FLAG_BREAK, and power_stage_enable()
 *        fails while the input stays active. false releases the input. A
 *        short pulse is true followed by false.
 */
void power_stage_sim_brk(const struct device *dev, bool active);

/** \brief Number of PWM cycles run since init. */
uint64_t power_stage_sim_cycles(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_POWER_STAGE_SIM_H_ */
