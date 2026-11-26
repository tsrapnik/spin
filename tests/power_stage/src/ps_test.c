/*
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/ztest.h>
#include <pmsm/power_stage.h>
#include <pmsm/power_stage_sim.h>

#include "ps_test.h"

const struct device *const ps_manual = DEVICE_DT_GET(DT_NODELABEL(ps_manual));
const struct device *const ps_timer = DEVICE_DT_GET(DT_NODELABEL(ps_timer));

const struct plant_params ps_test_plant = {
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

void ps_test_reset(const struct device *dev)
{
	k_spinlock_key_t key;

	zassert_true(device_is_ready(dev));
	power_stage_disable(dev);
	power_stage_sim_brk(dev, false);
	zassert_ok(power_stage_register_cb(dev, NULL, NULL));
	power_stage_sim_set_offset(dev, 0.0f, 0.0f, 0.0f);

	struct plant *p = power_stage_sim_lock(dev, &key);
	int ret = plant_init(p, &ps_test_plant);

	power_stage_sim_unlock(dev, key);
	zassert_ok(ret);

	/* Zero current and no injected offset: calibrates the offsets to 0. */
	zassert_ok(power_stage_calibrate_offsets(dev));
	/* Drop a break flag still pending from a test. */
	if (dev == ps_manual) {
		zassert_ok(power_stage_sim_run(dev, 1U));
	}
}
