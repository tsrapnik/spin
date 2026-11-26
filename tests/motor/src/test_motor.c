/*
 * SPDX-License-Identifier: MIT
 *
 * Motor device on the timer-driven sim power stage: the ISR/thread split,
 * the cmd/status exchange and the API, in simulated kernel time.
 */

#include <errno.h>
#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/motor.h>
#include <pmsm/plant.h>
#include <pmsm/power_stage_sim.h>

#include "common.h"

ZTEST(motor, test_start_run_speed)
{
	zassert_ok(motor_start(motor, 50.0f));
	zassert_equal(motor_start(motor, 50.0f), -EALREADY);
	k_msleep(500);

	struct motor_status st = status();

	zassert_equal(st.state, FOC_STATE_RUN, "state %d", st.state);
	k_msleep(300);
	st = status();
	zassert_within(plant_get().wm, 50.0, 2.5, "plant %f rad/s", plant_get().wm);
	zassert_within((double)st.speed, plant_get().wm, 1.0, "estimate %f rad/s",
		       (double)st.speed);
	zassert_within((double)st.speed_target, 50.0, 1e-4);
	zassert_within((double)st.speed_ref, 50.0, 1e-4);
	zassert_within((double)st.vbus, 24.0, 1e-3);
	zassert_true(st.emf > 0.0f);
	zassert_equal(st.faults, 0U);
	zassert_equal(st.late, 0U);

	zassert_ok(motor_set_speed(motor, 80.0f));
	k_msleep(300);
	zassert_within(plant_get().wm, 80.0, 4.0, "plant %f rad/s", plant_get().wm);
}

ZTEST(motor, test_reverse)
{
	zassert_ok(motor_start(motor, -50.0f));
	k_msleep(800);
	zassert_equal(status().state, FOC_STATE_RUN);
	zassert_within(plant_get().wm, -50.0, 2.5, "plant %f rad/s", plant_get().wm);

	/* The direction is fixed at start: the other sign holds the handover speed. */
	zassert_ok(motor_set_speed(motor, 50.0f));
	zassert_true(status().speed_target < 0.0f);
}

ZTEST(motor, test_stop_restart)
{
	zassert_ok(motor_start(motor, 50.0f));
	k_msleep(600);
	zassert_ok(motor_stop(motor));
	/* The sequencer has not seen the stop yet. */
	zassert_equal(motor_start(motor, 50.0f), -EBUSY);
	zassert_true(wait_state(FOC_STATE_STOP, 2));
	k_msleep(2);

	struct plant p = plant_get();

	/* Outputs off: no current, the rotor coasts. */
	zassert_equal(status().speed, 0.0f);
	zassert_true(fabs(p.id) + fabs(p.iq) < 1e-9, "i %g %g", p.id, p.iq);
	zassert_true(p.wm > 40.0, "coasting at %f", p.wm);

	/* Restart from standstill (no flying start in v1). */
	plant_reset();
	zassert_ok(motor_start(motor, 60.0f));
	k_msleep(800);
	zassert_equal(status().state, FOC_STATE_RUN);
	zassert_within(plant_get().wm, 60.0, 3.0, "plant %f rad/s", plant_get().wm);
}

ZTEST(motor, test_break_fault)
{
	zassert_ok(motor_start(motor, 50.0f));
	k_msleep(600);
	zassert_equal(status().state, FOC_STATE_RUN);

	power_stage_sim_brk(ps, true);
	power_stage_sim_brk(ps, false);
	zassert_true(wait_state(FOC_STATE_FAULT, 3));
	zassert_equal(status().faults, FOC_FAULT_EXT);
	zassert_equal(motor_start(motor, 50.0f), -EIO);

	zassert_ok(motor_stop(motor));
	zassert_true(wait_state(FOC_STATE_STOP, 2));
	zassert_equal(status().faults, 0U);
	k_msleep(2);

	plant_reset();
	zassert_ok(motor_start(motor, 50.0f));
	k_msleep(800);
	zassert_equal(status().state, FOC_STATE_RUN);
	zassert_within(plant_get().wm, 50.0, 2.5);
}

/* Break input held active at start: enable fails, the controller faults at once. */
ZTEST(motor, test_break_held)
{
	power_stage_sim_brk(ps, true);
	zassert_ok(motor_start(motor, 50.0f));
	zassert_true(wait_state(FOC_STATE_FAULT, 5));
	zassert_equal(status().faults, FOC_FAULT_EXT);
	power_stage_sim_brk(ps, false);
}

ZTEST(motor, test_offsets)
{
	/* Calibrated away at start. */
	power_stage_sim_set_offset(ps, 0.2f, -0.1f, 0.05f);
	zassert_ok(motor_start(motor, 50.0f));
	k_msleep(800);
	zassert_equal(status().state, FOC_STATE_RUN);
	zassert_within(plant_get().wm, 50.0, 2.5);
	zassert_ok(motor_stop(motor));
	zassert_true(wait_state(FOC_STATE_STOP, 2));
	k_msleep(2);

	/* Beyond what the sensing can have: refused. */
	power_stage_sim_set_offset(ps, 2.0f, 0.0f, 0.0f);
	zassert_equal(motor_start(motor, 50.0f), -ERANGE);
	k_msleep(5);
	zassert_equal(status().state, FOC_STATE_STOP);
}

ZTEST(motor, test_bad_args)
{
	zassert_equal(motor_start(motor, NAN), -EINVAL);
	zassert_equal(motor_start(motor, INFINITY), -EINVAL);
	zassert_equal(motor_set_speed(motor, NAN), -EINVAL);
	zassert_equal(motor_set_speed(motor, -INFINITY), -EINVAL);
	k_msleep(5);
	zassert_equal(status().state, FOC_STATE_STOP);
}

ZTEST_SUITE(motor, NULL, NULL, motor_test_before, NULL, NULL);
