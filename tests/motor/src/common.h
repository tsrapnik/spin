/*
 * SPDX-License-Identifier: MIT
 *
 * Helpers shared by the motor test suites.
 */

#ifndef TESTS_MOTOR_COMMON_H_
#define TESTS_MOTOR_COMMON_H_

#include <stdbool.h>
#include <zephyr/device.h>
#include <pmsm/foc.h>
#include <pmsm/motor.h>
#include <pmsm/plant.h>

extern const struct device *const motor;
extern const struct device *const ps;

/** \brief The motor's status; asserts that the read succeeds. */
struct motor_status status(void);

/** \brief Wait up to \p ms for \p state; returns whether it was reached. */
bool wait_state(enum foc_state state, unsigned int ms);

/** \brief Copy of the simulated motor. */
struct plant plant_get(void);

/** \brief Rotor back to standstill at angle 0. */
void plant_reset(void);

/** \brief ztest before-hook: motor stopped, no break, no offsets, rotor at rest. */
void motor_test_before(void *f);

#endif /* TESTS_MOTOR_COMMON_H_ */
