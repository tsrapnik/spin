/*
 * SPDX-License-Identifier: MIT
 *
 * Power-stage driver class: a three-phase bridge with synchronised PWM and
 * current/bus-voltage sampling. This is the only per-MCU part of the motor
 * stack (see docs/PLAN.md, "Abstraction boundary").
 *
 * Timing contract, per PWM period k:
 *
 *   1. The driver samples the phase currents and the bus voltage at a fixed
 *      point of the period (centre of the low-side on-time for low-side
 *      shunts) and calls the registered callback with them, in interrupt
 *      context.
 *   2. The callback computes the next duties and calls
 *      power_stage_set_duty(). The duties are latched at the next update
 *      event, so a duty written in cycle k is applied
 *      power_stage_info.duty_delay periods later (1 on hardware that
 *      latches at every update). The callback must return well before the
 *      update event.
 *
 * The callback runs every PWM period whether the outputs are on or off,
 * so the controller sees every sample and can react to state changes.
 * power_stage_enable() and power_stage_disable() switch the bridge outputs
 * only.
 *
 * Fault path: a hardware break (over-current comparator) switches the
 * outputs off without software. The driver reports it with
 * #POWER_STAGE_FLAG_BREAK in the next sample; the outputs stay off until
 * power_stage_enable().
 *
 * Duties are 0..1 per phase, 0.5 = zero phase voltage (as in pmsm/foc.h).
 * Currents are positive into the motor, in A; the bus voltage is in V.
 */

#ifndef PMSM_POWER_STAGE_H_
#define PMSM_POWER_STAGE_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Power-stage driver class
 * \defgroup pmsm_power_stage Power stage
 * \{
 */

/** \brief The hardware break fired since the previous sample; the outputs are off. */
#define POWER_STAGE_FLAG_BREAK BIT(0)

/** \brief One sample, delivered once per PWM period. */
struct power_stage_sample {
	float ia, ib, ic; /**< Phase currents, A, offsets removed. */
	float vbus;       /**< Bus voltage, V. */
	uint32_t flags;   /**< POWER_STAGE_FLAG_* bits. */
};

/** \brief Fixed properties of a power stage. */
struct power_stage_info {
	float ts;        /**< PWM period, s. */
	float dead_time; /**< Dead time per switching edge, s. */
	/**
	 * Highest duty the driver applies; the lowest is 1 - max_duty. Leaves
	 * room for the current sampling window. Pass it to foc_svpwm().
	 */
	float max_duty;
	uint8_t duty_delay; /**< PWM periods from set_duty() to the applied duty. */
};

/**
 * \brief Per-cycle callback, called in interrupt context.
 *
 * \param dev The power stage.
 * \param s The sample of this period; valid during the call only.
 * \param user As passed to power_stage_register_cb().
 */
typedef void (*power_stage_cb_t)(const struct device *dev, const struct power_stage_sample *s,
				 void *user);

/** \cond INTERNAL_HIDDEN */
__subsystem struct power_stage_driver_api {
	int (*calibrate_offsets)(const struct device *dev);
	int (*enable)(const struct device *dev);
	void (*disable)(const struct device *dev);
	void (*set_duty)(const struct device *dev, float da, float db, float dc);
	int (*register_cb)(const struct device *dev, power_stage_cb_t cb, void *user);
	const struct power_stage_info *(*get_info)(const struct device *dev);
};
/** \endcond */

/**
 * \brief Measure the current-sense offsets and remove them from later samples.
 *
 * Blocks until done (a few ms on hardware). Call from a thread with the
 * outputs off and the phase currents decayed to zero.
 *
 * \retval 0 Done.
 * \retval -EBUSY The outputs are on.
 * \retval -ERANGE An offset is beyond what the current sensing can have.
 */
static inline int power_stage_calibrate_offsets(const struct device *dev)
{
	return DEVICE_API_GET(power_stage, dev)->calibrate_offsets(dev);
}

/**
 * \brief Switch the bridge outputs on, at the duty last set.
 *
 * Also re-arms the hardware break. ISR-safe.
 *
 * \retval 0 The outputs are on.
 * \retval -EIO The break input is still active; the outputs stay off.
 */
static inline int power_stage_enable(const struct device *dev)
{
	return DEVICE_API_GET(power_stage, dev)->enable(dev);
}

/**
 * \brief Switch the bridge outputs off (all switches open) at once.
 *
 * The duty is reset to 0.5 on all phases. ISR-safe.
 */
static inline void power_stage_disable(const struct device *dev)
{
	DEVICE_API_GET(power_stage, dev)->disable(dev);
}

/**
 * \brief Set the duties for the next period.
 *
 * Drivers clamp each duty to [1 - max_duty, max_duty] and treat NaN as 0.5.
 * Called from the per-cycle callback.
 */
static inline void power_stage_set_duty(const struct device *dev, float da, float db, float dc)
{
	DEVICE_API_GET(power_stage, dev)->set_duty(dev, da, db, dc);
}

/**
 * \brief Register the per-cycle callback, replacing any previous one.
 *
 * Call with the outputs off. NULL unregisters.
 *
 * \retval 0 Done.
 * \retval -EBUSY The outputs are on.
 */
static inline int power_stage_register_cb(const struct device *dev, power_stage_cb_t cb, void *user)
{
	return DEVICE_API_GET(power_stage, dev)->register_cb(dev, cb, user);
}

/** \brief Fixed properties of \p dev; valid for the device's lifetime. */
static inline const struct power_stage_info *power_stage_get_info(const struct device *dev)
{
	return DEVICE_API_GET(power_stage, dev)->get_info(dev);
}

/** \} */

#ifdef __cplusplus
}
#endif

#endif /* PMSM_POWER_STAGE_H_ */
