/*
 * SPDX-License-Identifier: MIT
 *
 * The whole drive through the power-stage API: foc_cycle in the per-cycle
 * callback, as the motor device will run it. The sequencer is stepped in
 * the callback too (every 1 ms), so the run stays deterministic; M4.2 moves
 * it to the motor thread.
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>
#include <pmsm/power_stage.h>
#include <pmsm/power_stage_sim.h>

#include "ps_test.h"

#define SEQ_TS 1e-3f

struct ctl {
	struct foc_cycle c;
	struct foc_seq s;
	struct foc_cmd cmd;
	struct foc_status st;
	unsigned int div, k;
	bool on;
};

static struct ctl ctl;

/* Per-cycle glue: break -> fault, controller step, outputs follow the state. */
static void on_sample(const struct device *dev, const struct power_stage_sample *smp, void *user)
{
	struct ctl *x = user;

	if (x->k++ % x->div == 0U) {
		x->cmd = *foc_seq_step(&x->s, &x->st);
	}
	if ((smp->flags & POWER_STAGE_FLAG_BREAK) != 0U) {
		foc_cycle_trip(&x->c, FOC_FAULT_EXT);
	}

	struct foc_abc i = {.a = smp->ia, .b = smp->ib, .c = smp->ic};
	struct foc_abc d = foc_cycle_step(&x->c, &x->cmd, i, smp->vbus, &x->st);
	bool on = x->st.state != FOC_STATE_STOP && x->st.state != FOC_STATE_FAULT;

	if (!on) {
		power_stage_disable(dev);
		x->on = false;
		return;
	}
	power_stage_set_duty(dev, d.a, d.b, d.c);
	if (!x->on) {
		x->on = power_stage_enable(dev) == 0;
	}
}

static void before(void *f)
{
	ARG_UNUSED(f);
	ps_test_reset(ps_manual);

	const struct power_stage_info *info = power_stage_get_info(ps_manual);
	/* As tests/sim/src/drive.c, with the timing from the power stage. */
	const struct foc_cycle_cfg cc = {
		.ts = info->ts,
		.rs = 3.6f,
		.ls = 1.5e-3f,
		.max_duty = info->max_duty,
		.t_adv = FOC_ADVANCE_PERIODS * info->ts,
		.delay = info->duty_delay,
		.bw_current_hz = 500.0f,
		.bw_obs_hz = 2000.0f,
		.bw_pll_hz = 150.0f,
		.w_max = 3000.0f,
		.e_min = 0.1f,
		.i_trip = 3.0f,
		.vbus_min = 10.0f,
		.vbus_max = 30.0f,
	};
	const struct foc_seq_cfg sc = {
		.ts = SEQ_TS,
		.pole_pairs = 11U,
		.j = 6e-6f,
		.psi = 0.011f,
		.bw_speed_hz = 50.0f,
		.i_max = 1.4f,
		.i_start = 1.0f,
		.t_align = 0.1f,
		.accel_start = 300.0f,
		.w_handover = 30.0f,
		.accel = 500.0f,
		.w_max = 100.0f,
	};

	ctl = (struct ctl){0};
	foc_cycle_init(&ctl.c, &cc);
	foc_seq_init(&ctl.s, &sc);
	ctl.cmd = ctl.s.cmd;
	ctl.st.state = FOC_STATE_STOP;
	ctl.div = (unsigned int)lroundf(SEQ_TS / info->ts);
	zassert_ok(power_stage_register_cb(ps_manual, on_sample, &ctl));
}

static void after(void *f)
{
	ARG_UNUSED(f);
	ps_test_reset(ps_manual);
}

static void run_s(float t)
{
	unsigned int n = (unsigned int)lroundf(t / power_stage_get_info(ps_manual)->ts);

	zassert_ok(power_stage_sim_run(ps_manual, n));
}

static double plant_wm(void)
{
	k_spinlock_key_t key;
	double w = power_stage_sim_lock(ps_manual, &key)->wm;

	power_stage_sim_unlock(ps_manual, key);
	return w;
}

/* Standstill -> RUN -> 50 rad/s, then the break trips it and stop clears it. */
ZTEST(power_stage_closed_loop, test_start_run_break_stop)
{
	foc_seq_start(&ctl.s, 50.0f);
	run_s(0.5f);
	zassert_equal(ctl.s.state, FOC_STATE_RUN, "state %d", ctl.s.state);
	run_s(0.3f);
	zassert_within(plant_wm(), 50.0, 2.5, "speed %f rad/s", plant_wm());
	zassert_true(ctl.on);

	power_stage_sim_brk(ps_manual, true);
	power_stage_sim_brk(ps_manual, false);
	run_s(0.005f);
	zassert_equal(ctl.st.state, FOC_STATE_FAULT);
	zassert_equal(ctl.st.faults, FOC_FAULT_EXT);
	zassert_false(ctl.on);

	foc_seq_stop(&ctl.s);
	run_s(0.005f);
	zassert_equal(ctl.st.state, FOC_STATE_STOP);
	zassert_equal(ctl.st.faults, 0U);
	zassert_false(ctl.on);
}

ZTEST_SUITE(power_stage_closed_loop, NULL, NULL, before, after, NULL);
