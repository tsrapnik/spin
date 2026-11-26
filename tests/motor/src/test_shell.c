/*
 * SPDX-License-Identifier: MIT
 *
 * motor shell commands through the dummy shell backend, checked against
 * the motor API and the simulated plant.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/ztest.h>
#include <pmsm/motor.h>
#include <pmsm/power_stage_sim.h>

#include "common.h"

#define RPM (6.28318530718 / 60.0)

static const char *out;

/* Run \p cmd; its output is in out afterwards. */
static int run(const char *cmd)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	size_t len;

	shell_backend_dummy_clear_output(sh);
	int ret = shell_execute_cmd(sh, cmd);

	out = shell_backend_dummy_get_output(sh, &len);
	return ret;
}

static void zassert_out(const char *s)
{
	zassert_not_null(strstr(out, s), "'%s' not in:\n%s", s, out);
}

static void *setup(void)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();

	/* The shell thread initialises the backend. */
	for (int t = 0; t < 1000 && sh->ctx->state != SHELL_STATE_ACTIVE; t++) {
		k_msleep(1);
	}
	zassert_equal(sh->ctx->state, SHELL_STATE_ACTIVE);
	return NULL;
}

ZTEST(motor_shell, test_status_stop)
{
	zassert_ok(run("motor status"));
	zassert_out("motor: stop");
	/* ref and target keep their last values in STOP. */
	zassert_out("speed   0.0 rpm (ref ");
	zassert_out("vbus    24.00 V");
	zassert_out("late    0");
	zassert_is_null(strstr(out, "faults"));
}

ZTEST(motor_shell, test_start_speed_stop)
{
	zassert_ok(run("motor start 500"));
	zassert_equal(run("motor start 500"), -EALREADY);
	zassert_out("already running");
	k_msleep(800);

	struct motor_status st = status();

	zassert_equal(st.state, FOC_STATE_RUN, "state %d", st.state);
	zassert_within((double)st.speed_target, 500.0 * RPM, 1e-3);
	zassert_within(plant_get().wm, 500.0 * RPM, 2.5, "plant %f rad/s", plant_get().wm);

	zassert_ok(run("motor status"));
	zassert_out("motor: run");
	zassert_out("(ref 500.0, target 500.0)");

	zassert_ok(run("motor speed 700"));
	k_msleep(300);
	zassert_within((double)status().speed_target, 700.0 * RPM, 1e-3);
	zassert_within(plant_get().wm, 700.0 * RPM, 3.5, "plant %f rad/s", plant_get().wm);

	zassert_ok(run("motor stop"));
	zassert_equal(run("motor start 500"), -EBUSY);
	zassert_out("still stopping");
	zassert_true(wait_state(FOC_STATE_STOP, 2));
	zassert_ok(run("motor status"));
	zassert_out("motor: stop");
}

ZTEST(motor_shell, test_reverse_named)
{
	zassert_ok(run("motor start -400 motor"));
	k_msleep(800);
	zassert_equal(status().state, FOC_STATE_RUN);
	zassert_within(plant_get().wm, -400.0 * RPM, 2.0, "plant %f rad/s", plant_get().wm);
	zassert_ok(run("motor status motor"));
	zassert_out("target -400.0)");
	zassert_ok(run("motor stop motor"));
	zassert_true(wait_state(FOC_STATE_STOP, 2));
}

ZTEST(motor_shell, test_fault)
{
	zassert_ok(run("motor start 500"));
	k_msleep(600);
	power_stage_sim_brk(ps, true);
	power_stage_sim_brk(ps, false);
	zassert_true(wait_state(FOC_STATE_FAULT, 3));

	zassert_ok(run("motor status"));
	zassert_out("motor: fault");
	zassert_out("faults  0x8 break");
	zassert_equal(run("motor start 500"), -EIO);
	zassert_out("in fault");

	zassert_ok(run("motor stop"));
	zassert_true(wait_state(FOC_STATE_STOP, 2));
	zassert_ok(run("motor status"));
	zassert_is_null(strstr(out, "faults"));
}

ZTEST(motor_shell, test_offset_refused)
{
	power_stage_sim_set_offset(ps, 2.0f, 0.0f, 0.0f);
	zassert_equal(run("motor start 500"), -ERANGE);
	zassert_out("offset calibration failed");
	k_msleep(5);
	zassert_equal(status().state, FOC_STATE_STOP);
}

ZTEST(motor_shell, test_bad_args)
{
	static const char *const bad[] = {
		"motor start",          /* Missing speed. */
		"motor speed",          /* Missing speed. */
		"motor start abc",      /* Not a number. */
		"motor start 500x",     /* Trailing junk. */
		"motor start 1.5",      /* Whole rpm only. */
		"motor start 1000001",  /* Out of range. */
		"motor speed -1000001", /* Out of range. */
		"motor start 500 ps",   /* Not a motor. */
		"motor status nope",    /* No such device. */
		"motor stop nope",      /* No such device. */
		"motor start 1 motor x" /* Too many arguments. */
	};

	for (size_t k = 0; k < ARRAY_SIZE(bad); k++) {
		zassert_not_equal(run(bad[k]), 0, "'%s' accepted", bad[k]);
	}
	k_msleep(5);
	zassert_equal(status().state, FOC_STATE_STOP);

	zassert_equal(run("motor start 500 ps"), -EINVAL);
	zassert_out("ps: no such motor");
	zassert_equal(run("motor start abc"), -EINVAL);
	zassert_out("abc: not a speed in rpm");
}

ZTEST_SUITE(motor_shell, NULL, setup, motor_test_before, NULL, NULL);
