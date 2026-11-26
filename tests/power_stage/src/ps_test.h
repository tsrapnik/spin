/*
 * SPDX-License-Identifier: MIT
 *
 * Shared fixture of the power-stage tests.
 */

#ifndef PMSM_TEST_PS_TEST_H_
#define PMSM_TEST_PS_TEST_H_

#include <zephyr/device.h>
#include <pmsm/plant.h>

/** \brief The manually stepped instance (tick-period-us = 0). */
extern const struct device *const ps_manual;
/** \brief The timer-driven instance. */
extern const struct device *const ps_timer;

/** \brief Plant parameters of both devicetree nodes (app.overlay). */
extern const struct plant_params ps_test_plant;

/**
 * \brief Outputs off, no callback, no offsets, plant back to its initial
 *        state.
 */
void ps_test_reset(const struct device *dev);

#endif /* PMSM_TEST_PS_TEST_H_ */
