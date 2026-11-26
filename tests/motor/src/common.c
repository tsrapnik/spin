/*
 * SPDX-License-Identifier: MIT
 *
 * Helpers shared by the motor test suites.
 */

#include <zephyr/ztest.h>
#include <pmsm/power_stage_sim.h>

#include "common.h"

const struct device *const motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
const struct device *const ps = DEVICE_DT_GET(DT_NODELABEL(ps));

/* As app.overlay. */
static const struct plant_params plant_prm = {
	.rs = 3.6f,
	.ls = 1.5e-3f,
	.psi = 0.011f,
	.pole_pairs = 11U,
	.j = 6e-6f,
	.b = 0.0f,
	.vbus = 24.0f,
	.ts = 50e-6f,
	.duty_delay = 1U,
	.substeps = 8U,
};

struct motor_status status(void)
{
	struct motor_status st;

	zassert_ok(motor_get_status(motor, &st));
	return st;
}

bool wait_state(enum foc_state state, unsigned int ms)
{
	for (unsigned int t = 0; t <= ms; t++) {
		if (status().state == state) {
			return true;
		}
		k_msleep(1);
	}
	return false;
}

struct plant plant_get(void)
{
	k_spinlock_key_t key;
	struct plant p = *power_stage_sim_lock(ps, &key);

	power_stage_sim_unlock(ps, key);
	return p;
}

void plant_reset(void)
{
	k_spinlock_key_t key;
	int ret = plant_init(power_stage_sim_lock(ps, &key), &plant_prm);

	power_stage_sim_unlock(ps, key);
	zassert_ok(ret);
}

void motor_test_before(void *f)
{
	ARG_UNUSED(f);
	zassert_true(device_is_ready(motor));
	zassert_ok(motor_stop(motor));
	zassert_true(wait_state(FOC_STATE_STOP, 5));
	k_msleep(2); /* Controller in STOP too. */
	power_stage_sim_brk(ps, false);
	power_stage_sim_set_offset(ps, 0.0f, 0.0f, 0.0f);
	plant_reset();
}
