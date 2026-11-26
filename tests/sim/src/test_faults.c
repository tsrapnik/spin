/*
 * SPDX-License-Identifier: MIT
 *
 * Per-cycle faults on the full drive: over-current, over- and under-voltage,
 * a broken current reading and an external trip. Each switches the PWM off
 * in the cycle it is seen, latches until stop, and a restart then works.
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "drive.h"
#include "sim.h"

static void running(struct drive *d)
{
	drive_init(d, &sim_motor, &drive_cycle_cfg, &drive_seq_cfg);
	foc_seq_start(&d->s, 50.0f);
	drive_run(d, 0.4f);
	zassert_equal(d->s.state, FOC_STATE_RUN);
	zassert_within(d->p.wm, 50.0, 1e-3);
}

/* The fault takes the PWM off in this very cycle and latches. */
static void expect_trip(struct drive *d, unsigned int bit)
{
	drive_cycle(d);
	zassert_equal(d->st.state, FOC_STATE_FAULT);
	zassert_true((d->st.faults & bit) != 0U, "faults %x", d->st.faults);
	zassert_false(d->pwm);
	zassert_equal(d->p.id, 0.0);
	/* The sequencer sees it at its next step and stays in FAULT. */
	drive_run(d, 0.05f);
	zassert_equal(d->s.state, FOC_STATE_FAULT);
	zassert_true((d->s.faults & bit) != 0U);
	zassert_equal(d->cmd.state, FOC_STATE_FAULT);
	zassert_false(d->pwm);
}

/* Fault condition gone: stop clears it, a start from rest works again. */
static void clear_and_restart(struct drive *d)
{
	foc_seq_start(&d->s, 50.0f); /* ignored: not stopped */
	drive_run(d, 0.01f);
	zassert_equal(d->s.state, FOC_STATE_FAULT);

	foc_seq_stop(&d->s);
	drive_run(d, 0.01f);
	zassert_equal(d->s.state, FOC_STATE_STOP);
	zassert_equal(d->st.state, FOC_STATE_STOP);
	zassert_equal(d->st.faults, 0U);

	d->p.wm = 0.0;
	foc_seq_start(&d->s, 50.0f);
	zassert_true(drive_until(d, FOC_STATE_RUN, 0.3f) > 0.0);
	drive_run(d, 0.3f);
	zassert_true(d->pwm);
	zassert_within(d->p.wm, 50.0, 1e-3);
}

/* A current-sense offset pushes one phase past the trip level. */
ZTEST(sim_faults, test_over_current)
{
	struct drive d;

	running(&d);
	d.i_add.a = drive_cycle_cfg.i_trip;
	expect_trip(&d, FOC_FAULT_OC);
	d.i_add.a = 0.0f;
	clear_and_restart(&d);
}

ZTEST(sim_faults, test_over_voltage)
{
	struct drive d;

	running(&d);
	d.p.vbus = 31.0;
	expect_trip(&d, FOC_FAULT_OV);
	d.p.vbus = 24.0;
	clear_and_restart(&d);
}

ZTEST(sim_faults, test_under_voltage)
{
	struct drive d;

	running(&d);
	d.p.vbus = 9.0;
	expect_trip(&d, FOC_FAULT_UV);
	d.p.vbus = 24.0;
	clear_and_restart(&d);
}

/* A NaN reading is a fault, not a zero. */
ZTEST(sim_faults, test_nan_reading)
{
	struct drive d;

	running(&d);
	d.i_add.b = NAN;
	expect_trip(&d, FOC_FAULT_OC);
	d.i_add.b = 0.0f;
	clear_and_restart(&d);
}

/* External trip (hardware break input). */
ZTEST(sim_faults, test_external_trip)
{
	struct drive d;

	running(&d);
	foc_cycle_trip(&d.c, FOC_FAULT_EXT);
	expect_trip(&d, FOC_FAULT_EXT);
	clear_and_restart(&d);
}

/* Faults with the PWM off are ignored: an under-voltage at rest does not latch. */
ZTEST(sim_faults, test_ignored_while_stopped)
{
	struct drive d;

	drive_init(&d, &sim_motor, &drive_cycle_cfg, &drive_seq_cfg);
	d.p.vbus = 5.0;
	drive_run(&d, 0.01f);
	zassert_equal(d.st.state, FOC_STATE_STOP);
	zassert_equal(d.st.faults, 0U);
	/* Starting at that voltage trips at the first active cycle. */
	foc_seq_start(&d.s, 50.0f);
	zassert_true(drive_until(&d, FOC_STATE_FAULT, 0.01f) > 0.0);
	zassert_equal(d.s.faults, FOC_FAULT_UV);
}

ZTEST_SUITE(sim_faults, NULL, NULL, NULL, NULL, NULL);
