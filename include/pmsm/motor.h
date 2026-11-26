/*
 * SPDX-License-Identifier: MIT
 *
 * Motor device (compatible "pmsm,motor"): sensorless FOC of one PMSM on a
 * power stage (pmsm/power_stage.h).
 *
 * Runtime structure:
 * - Per PWM cycle, in the power stage's interrupt: foc_cycle_step() (fault
 *   checks, observer + PLL, current loop) and the bridge on/off switching.
 *   No kernel calls.
 * - Every CONFIG_PMSM_MOTOR_PERIOD_US, in the motor thread:
 *   foc_seq_step() (startup, speed loop, ramps) and fault reporting.
 * - The two exchange struct foc_cmd and struct foc_status, each with a
 *   single writer, through double buffers switched by an atomic counter
 *   (no locks in the interrupt).
 *
 * The functions here are called from threads (not from interrupts) and
 * take the device's mutex. Speeds are mechanical rad/s, the sign is the
 * direction.
 */

#ifndef PMSM_MOTOR_H_
#define PMSM_MOTOR_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <pmsm/foc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Motor device
 * \defgroup pmsm_motor Motor
 * \{
 */

/** \brief Snapshot of a motor, see motor_get_status(). */
struct motor_status {
	/**
	 * Drive state. FAULT latches until motor_stop(); the outputs are off
	 * in STOP and FAULT.
	 */
	enum foc_state state;
	unsigned int faults; /**< FOC_FAULT_* bits that caused FAULT, else 0. */
	float speed;         /**< Estimated speed (PLL), rad/s; 0 in STOP and FAULT. */
	float speed_ref;     /**< Ramped speed reference (RUN), rad/s. */
	float speed_target;  /**< Target after clamping, rad/s. */
	struct foc_dq i;     /**< Measured current in the control frame, A. */
	struct foc_dq v;     /**< Voltage command in the control frame, V. */
	float vbus;          /**< Measured bus voltage, V. */
	float emf;           /**< Back-EMF estimate magnitude, V. */
	/** Motor-thread periods missed since init (the thread ran late). */
	uint32_t late;
};

/**
 * \brief Start towards \p speed (rad/s, sign = direction).
 *
 * Calibrates the current-sense offsets first (the outputs are off in
 * STOP), then begins align -> I/f -> closed loop. The target is clamped to
 * between handover-speed-rpm and max-speed-rpm in the start direction.
 *
 * \retval 0 Started.
 * \retval -EINVAL \p speed is not finite.
 * \retval -EALREADY Already running.
 * \retval -EIO In FAULT; call motor_stop() first.
 * \retval -EBUSY A stop is still in progress (within one thread period).
 * \retval -ERANGE The offset calibration failed (sensing fault).
 */
int motor_start(const struct device *dev, float speed);

/**
 * \brief Stop: the outputs go off and the motor coasts. Also clears FAULT.
 *
 * Takes effect at the next motor-thread period.
 *
 * \retval 0 Always.
 */
int motor_stop(const struct device *dev);

/**
 * \brief Change the target speed while running (rad/s).
 *
 * Clamped as in motor_start(); the direction cannot change (stop, then
 * start). motor_start() sets the target anew.
 *
 * \retval 0 Done.
 * \retval -EINVAL \p speed is not finite.
 */
int motor_set_speed(const struct device *dev, float speed);

/**
 * \brief Read a consistent snapshot, at most one motor-thread period old.
 *
 * \retval 0 Done.
 */
int motor_get_status(const struct device *dev, struct motor_status *st);

/** \} */

#ifdef __cplusplus
}
#endif

#endif /* PMSM_MOTOR_H_ */
