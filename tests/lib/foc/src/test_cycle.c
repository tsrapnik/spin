/*
 * SPDX-License-Identifier: MIT
 *
 * Per-cycle controller: state transitions, fault checks and latching,
 * forced angle and PLL hold, handover rotation of the current integrators.
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

static const struct foc_cycle_cfg cfg = {
	.ts = 50e-6f,
	.rs = 3.6f,
	.ls = 1.5e-3f,
	.max_duty = 0.95f,
	.t_adv = FOC_ADVANCE_PERIODS * 50e-6f,
	.delay = 1U,
	.bw_current_hz = 500.0f,
	.bw_obs_hz = 2000.0f,
	.bw_pll_hz = 150.0f,
	.w_max = 3000.0f,
	.e_min = 0.1f,
	.i_trip = 3.0f,
	.vbus_min = 10.0f,
	.vbus_max = 30.0f,
};

static const struct foc_abc i0 = {0};

static bool is_zero_voltage(struct foc_abc d)
{
	return d.a == 0.5f && d.b == 0.5f && d.c == 0.5f;
}

static struct foc_abc step(struct foc_cycle *c, enum foc_state s, struct foc_abc i, float vbus,
			   struct foc_status *st)
{
	struct foc_cmd cmd = {.state = s, .i_ref = {.d = 1.0f}, .pll_hold = 1U};

	return foc_cycle_step(c, &cmd, i, vbus, st);
}

ZTEST(foc_cycle, test_stop_is_zero_voltage)
{
	struct foc_cycle c;
	struct foc_status st;

	foc_cycle_init(&c, &cfg);
	zassert_equal(c.state, FOC_STATE_STOP);
	zassert_true(is_zero_voltage(step(&c, FOC_STATE_STOP, i0, 24.0f, &st)));
	zassert_equal(st.state, FOC_STATE_STOP);
	zassert_equal(st.seq, 1U);
	/* A FAULT command is PWM off as well, and no latched fault. */
	zassert_true(is_zero_voltage(step(&c, FOC_STATE_FAULT, i0, 24.0f, &st)));
	zassert_equal(st.state, FOC_STATE_STOP);
	zassert_equal(st.faults, 0U);
	/* A state outside the enum counts as STOP. */
	zassert_true(is_zero_voltage(step(&c, (enum foc_state)77, i0, 24.0f, &st)));
	zassert_equal(st.state, FOC_STATE_STOP);
}

/* ALIGN drives current on d at the forced angle 0: alpha voltage only. */
ZTEST(foc_cycle, test_align_drives_d)
{
	struct foc_cycle c;
	struct foc_status st;
	struct foc_abc d;

	foc_cycle_init(&c, &cfg);
	d = step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_ALIGN);
	zassert_false(is_zero_voltage(d));

	struct foc_ab v = foc_duty_to_ab(d, 24.0f);

	zassert_true(v.alpha > 0.0f);
	zassert_within(v.beta, 0.0f, 1e-5f);
	zassert_true(st.v.d > 0.0f && st.v.q == 0.0f);
}

/* Each fault bit, from each active state; the PWM goes off in that cycle. */
ZTEST(foc_cycle, test_fault_checks)
{
	const enum foc_state states[] = {FOC_STATE_ALIGN, FOC_STATE_IF, FOC_STATE_RUN};
	const struct {
		struct foc_abc i;
		float vbus;
		unsigned int f;
	} cases[] = {
		{{3.01f, 0.0f, -3.01f}, 24.0f, FOC_FAULT_OC},
		{{0.0f, -3.01f, 0.0f}, 24.0f, FOC_FAULT_OC},
		{{0.0f, 0.0f, NAN}, 24.0f, FOC_FAULT_OC},
		{{INFINITY, 0.0f, 0.0f}, 24.0f, FOC_FAULT_OC},
		{i0, 30.01f, FOC_FAULT_OV},
		{i0, 9.99f, FOC_FAULT_UV},
		{i0, NAN, FOC_FAULT_OV | FOC_FAULT_UV},
		{{NAN, NAN, NAN}, NAN, FOC_FAULT_OC | FOC_FAULT_OV | FOC_FAULT_UV},
	};

	for (size_t s = 0; s < ARRAY_SIZE(states); s++) {
		for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
			struct foc_cycle c;
			struct foc_status st;

			foc_cycle_init(&c, &cfg);
			(void)step(&c, states[s], i0, 24.0f, &st);
			zassert_equal(st.state, states[s]);
			zassert_true(is_zero_voltage(
				step(&c, states[s], cases[k].i, cases[k].vbus, &st)));
			zassert_equal(st.state, FOC_STATE_FAULT, "state %d case %zu", states[s], k);
			zassert_equal(st.faults, cases[k].f, "case %zu: %x", k, st.faults);
		}
	}
	/* At the limits: no trip. */
	struct foc_cycle c;
	struct foc_status st;

	foc_cycle_init(&c, &cfg);
	(void)step(&c, FOC_STATE_ALIGN, (struct foc_abc){3.0f, -3.0f, 0.0f}, 30.0f, &st);
	(void)step(&c, FOC_STATE_ALIGN, i0, 10.0f, &st);
	zassert_equal(st.state, FOC_STATE_ALIGN);
}

/* FAULT holds against any command but STOP; STOP clears the bits. */
ZTEST(foc_cycle, test_fault_latches)
{
	struct foc_cycle c;
	struct foc_status st;

	foc_cycle_init(&c, &cfg);
	(void)step(&c, FOC_STATE_RUN, i0, 24.0f, &st);
	(void)step(&c, FOC_STATE_RUN, i0, 40.0f, &st);
	zassert_equal(st.state, FOC_STATE_FAULT);
	/* Condition gone, still latched; more faults add bits. */
	(void)step(&c, FOC_STATE_RUN, i0, 24.0f, &st);
	(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	(void)step(&c, FOC_STATE_FAULT, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_FAULT);
	zassert_equal(st.faults, FOC_FAULT_OV);

	zassert_true(is_zero_voltage(step(&c, FOC_STATE_STOP, i0, 24.0f, &st)));
	zassert_equal(st.state, FOC_STATE_STOP);
	zassert_equal(st.faults, 0U);
	(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_ALIGN);
}

/* External trips: taken while active, dropped while off. */
ZTEST(foc_cycle, test_external_trip)
{
	struct foc_cycle c;
	struct foc_status st;

	foc_cycle_init(&c, &cfg);
	foc_cycle_trip(&c, FOC_FAULT_EXT);
	(void)step(&c, FOC_STATE_STOP, i0, 24.0f, &st);
	(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_ALIGN);
	foc_cycle_trip(&c, FOC_FAULT_EXT);
	(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_FAULT);
	zassert_equal(st.faults, FOC_FAULT_EXT);
}

/* The current reference is clamped to the trip level. */
ZTEST(foc_cycle, test_reference_clamped)
{
	struct foc_cycle_cfg b = cfg;
	struct foc_cycle c;
	struct foc_status st;
	struct foc_cmd cmd = {.state = FOC_STATE_ALIGN, .i_ref = {.d = 100.0f, .q = -NAN}};

	b.i_trip = 1.0f; /* keeps the first PI output inside the voltage limit */
	foc_cycle_init(&c, &b);
	(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	/* First PI step: (Kp + Ki ts) * err with err clamped to 1 A. */
	zassert_within(st.v.d, c.cur.d.kp + c.cur.d.ki_ts, 1e-4f);
	zassert_equal(st.v.q, 0.0f);
}

/*
 * Forced angle: integrated from w_forced; with pll_hold the PLL follows it,
 * without it the PLL runs on the observer. Leaving STOP resets it to 0.
 */
ZTEST(foc_cycle, test_forced_angle_and_hold)
{
	struct foc_cycle c;
	struct foc_status st;
	struct foc_cmd cmd = {.state = FOC_STATE_IF, .w_forced = 1000.0f, .pll_hold = 1U};

	foc_cycle_init(&c, &cfg);
	for (int k = 0; k < 20; k++) {
		(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	}
	zassert_within(c.theta_f, 20.0f * 1000.0f * cfg.ts, 1e-5f);
	zassert_equal(st.theta, c.theta_f);
	zassert_equal(st.omega, 1000.0f);
	zassert_equal(st.delta, 0.0f);

	/* Released: the PLL coasts on its own (no EMF here), the forced angle goes on. */
	cmd.pll_hold = 0U;
	(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	zassert_within(c.theta_f, 21.0f * 1000.0f * cfg.ts, 1e-5f);

	/* The speed is clamped to w_max. */
	cmd.w_forced = 1e9f;
	cmd.pll_hold = 1U;
	(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	zassert_equal(st.omega, cfg.w_max);

	(void)step(&c, FOC_STATE_STOP, i0, 24.0f, &st);
	(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
	zassert_equal(c.theta_f, 0.0f);
}

/*
 * Handover: the voltage vector in the stationary frame is the same just
 * before and after the switch to the PLL frame. The integrators carry it;
 * a zero current error keeps the proportional part out.
 */
ZTEST(foc_cycle, test_handover_keeps_voltage)
{
	struct foc_cycle c;
	struct foc_status st;
	struct foc_cmd cmd = {.state = FOC_STATE_IF, .i_ref = {.d = 1.0f}, .pll_hold = 1U};

	foc_cycle_init(&c, &cfg);
	(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	/* Integrators as after some I/f time; PLL somewhere else. */
	foc_pi_reset(&c.cur.d, 2.0f);
	foc_pi_reset(&c.cur.q, 0.5f);
	c.theta_f = 0.3f;
	foc_pll_reset(&c.pll, -0.6f, 0.0f);

	struct foc_ab before = foc_inv_park((struct foc_dq){.d = c.cur.d.integ, .q = c.cur.q.integ},
					    foc_sincos(c.theta_f));

	cmd.state = FOC_STATE_RUN;
	cmd.i_ref = (struct foc_dq){0};
	/* Zero current in, zero reference: only the integrators act. */
	(void)foc_cycle_step(&c, &cmd, i0, 24.0f, &st);
	zassert_equal(st.state, FOC_STATE_RUN);

	struct foc_ab after = foc_inv_park((struct foc_dq){.d = c.cur.d.integ, .q = c.cur.q.integ},
					   foc_sincos(-0.6f));

	zassert_within(after.alpha, before.alpha, 1e-5f);
	zassert_within(after.beta, before.beta, 1e-5f);
	/* In RUN the forced angle follows the PLL. */
	zassert_equal(st.delta, 0.0f);
}

/* Bad limits fail safe: the first active cycle trips. */
ZTEST(foc_cycle, test_bad_limits_trip)
{
	const struct {
		float i_trip, vmin, vmax;
		unsigned int f;
	} cases[] = {
		{NAN, 10.0f, 30.0f, FOC_FAULT_OC},     {-1.0f, 10.0f, 30.0f, FOC_FAULT_OC},
		{3.0f, NAN, 30.0f, FOC_FAULT_UV},      {3.0f, -1.0f, 30.0f, FOC_FAULT_UV},
		{3.0f, 10.0f, INFINITY, FOC_FAULT_OV}, {3.0f, 10.0f, 0.0f, FOC_FAULT_OV},
	};

	for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
		struct foc_cycle_cfg b = cfg;
		struct foc_cycle c;
		struct foc_status st;

		b.i_trip = cases[k].i_trip;
		b.vbus_min = cases[k].vmin;
		b.vbus_max = cases[k].vmax;
		foc_cycle_init(&c, &b);
		(void)step(&c, FOC_STATE_ALIGN, i0, 24.0f, &st);
		zassert_equal(st.state, FOC_STATE_FAULT, "case %zu", k);
		zassert_true((st.faults & cases[k].f) != 0U, "case %zu: %x", k, st.faults);
	}
}

ZTEST_SUITE(foc_cycle, NULL, NULL, NULL, NULL, NULL);
