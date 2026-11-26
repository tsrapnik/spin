/*
 * SPDX-License-Identifier: MIT
 *
 * Sequencer on a scripted status: state sequence and timing, align ramp
 * and turn, I/f ramp and PLL release, handover preload, target clamping,
 * fault latching and stop.
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

static const struct foc_seq_cfg cfg = {
	.ts = 1e-3f,
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

/* Status as the controller reports it once it follows the command. */
static struct foc_status follow(const struct foc_cmd *c)
{
	return (struct foc_status){.state = c->state, .omega = c->w_forced};
}

/* Step until the state changes, at most n steps; returns the steps taken. */
static int until_change(struct foc_seq *s, struct foc_status *st, int n)
{
	enum foc_state from = s->state;

	for (int k = 1; k <= n; k++) {
		*st = follow(foc_seq_step(s, st));
		if (s->state != from) {
			return k;
		}
	}
	return -1;
}

ZTEST(foc_seq, test_sequence_and_timing)
{
	struct foc_seq s;
	struct foc_status st = {0};

	foc_seq_init(&s, &cfg);
	for (int k = 0; k < 10; k++) {
		zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_STOP);
	}
	foc_seq_start(&s, 60.0f);
	zassert_equal(until_change(&s, &st, 1), 1);
	zassert_equal(s.state, FOC_STATE_ALIGN);
	zassert_equal(s.cmd.pll_hold, 1U);
	/* 100 ms align (one step was the transition), 100 ms I/f to 30 rad/s. */
	int n = until_change(&s, &st, 200);

	zassert_true(n >= 99 && n <= 101, "align %d", n);
	zassert_equal(s.state, FOC_STATE_IF);
	n = until_change(&s, &st, 200);
	zassert_true(n >= 99 && n <= 101, "I/f %d", n);
	zassert_equal(s.state, FOC_STATE_RUN);
	zassert_equal(s.cmd.state, FOC_STATE_RUN);
	zassert_equal(s.w_ref, 30.0f);
}

/* Align: d ramps over the first half, the vector turns +90 degrees in the third quarter. */
ZTEST(foc_seq, test_align_profile)
{
	const float dirs[] = {1.0f, -1.0f};

	for (size_t i = 0; i < ARRAY_SIZE(dirs); i++) {
		struct foc_seq s;
		struct foc_status st = {0};
		float turn = 0.0f, id_max = 0.0f;

		foc_seq_init(&s, &cfg);
		foc_seq_start(&s, dirs[i] * 50.0f);
		(void)foc_seq_step(&s, &st);
		while (s.state == FOC_STATE_ALIGN) {
			const struct foc_cmd *c = foc_seq_step(&s, &st);

			if (s.state != FOC_STATE_ALIGN) {
				break;
			}
			zassert_true(c->i_ref.d >= id_max, "d current falls");
			id_max = c->i_ref.d;
			zassert_true(s.t > 0.05f || c->w_forced == 0.0f);
			turn += c->w_forced * cfg.ts;
			zassert_equal(c->i_ref.q, 0.0f);
		}
		zassert_equal(id_max, cfg.i_start);
		zassert_within(turn, dirs[i] * 0.5f * FOC_PI_F, 0.07f, "turn %f", (double)turn);
	}
}

/* I/f: forced speed ramps in the start direction, PLL released at half the handover. */
ZTEST(foc_seq, test_if_ramp_and_release)
{
	struct foc_seq s;
	struct foc_status st = {0};

	foc_seq_init(&s, &cfg);
	foc_seq_start(&s, -50.0f);
	while (s.state != FOC_STATE_IF) {
		st = follow(foc_seq_step(&s, &st));
	}
	float w_prev = 0.0f;

	while (s.state == FOC_STATE_IF) {
		const struct foc_cmd *c = foc_seq_step(&s, &st);

		if (s.state != FOC_STATE_IF) {
			break;
		}
		zassert_equal(c->i_ref.d, cfg.i_start);
		zassert_within(c->w_forced, w_prev - 300.0f * cfg.ts * 11.0f, 1e-2f);
		w_prev = c->w_forced;
		zassert_equal(c->pll_hold, (s.w_forced > -15.0f) ? 1U : 0U, "w %f",
			      (double)s.w_forced);
	}
}

/* Handover: speed PI preloaded with i_start sin(delta). */
ZTEST(foc_seq, test_handover_preload)
{
	const float deltas[] = {0.0f, 0.3f, -0.4f};

	for (size_t i = 0; i < ARRAY_SIZE(deltas); i++) {
		struct foc_seq s;
		struct foc_status st = {0};

		foc_seq_init(&s, &cfg);
		foc_seq_start(&s, 50.0f);
		while (s.state != FOC_STATE_RUN) {
			st = follow(foc_seq_step(&s, &st));
			st.delta = deltas[i];
		}
		zassert_within(s.speed.integ, sinf(deltas[i]), 1e-5f);
		zassert_within(s.cmd.i_ref.q, sinf(deltas[i]), 1e-5f);
		zassert_equal(s.cmd.i_ref.d, 0.0f);
	}
}

/* RUN: reference ramps at accel; target clamped to [w_handover, w_max] in the start direction. */
ZTEST(foc_seq, test_run_ramp_and_clamp)
{
	struct foc_seq s;
	struct foc_status st = {0};

	foc_seq_init(&s, &cfg);
	foc_seq_start(&s, 60.0f);
	while (s.state != FOC_STATE_RUN) {
		st = follow(foc_seq_step(&s, &st));
	}
	st.omega = 30.0f * 11.0f;
	(void)foc_seq_step(&s, &st);
	zassert_within(s.w_ref, 30.5f, 1e-4f);
	for (int k = 0; k < 100; k++) {
		(void)foc_seq_step(&s, &st);
	}
	zassert_equal(s.w_ref, 60.0f);
	/* Speed below the reference: positive q current. */
	zassert_true(s.cmd.i_ref.q > 0.0f && s.cmd.i_ref.q <= cfg.i_max);

	const struct {
		float w, t;
	} cases[] = {{1000.0f, 100.0f}, {-50.0f, 30.0f},    {5.0f, 30.0f},
		     {NAN, 30.0f},      {INFINITY, 100.0f}, {45.0f, 45.0f}};

	for (size_t k = 0; k < ARRAY_SIZE(cases); k++) {
		foc_seq_set_speed(&s, cases[k].w);
		zassert_equal(s.target, cases[k].t, "w %f", (double)cases[k].w);
	}
	/* Start in reverse: the target is negative. */
	foc_seq_stop(&s);
	(void)foc_seq_step(&s, &st);
	foc_seq_start(&s, -10.0f);
	zassert_equal(s.target, -30.0f);
	zassert_equal(s.dir, -1.0f);
}

/* Start is ignored unless stopped; stop works from every state. */
ZTEST(foc_seq, test_start_stop)
{
	const enum foc_state at[] = {FOC_STATE_ALIGN, FOC_STATE_IF, FOC_STATE_RUN};

	for (size_t i = 0; i < ARRAY_SIZE(at); i++) {
		struct foc_seq s;
		struct foc_status st = {0};

		foc_seq_init(&s, &cfg);
		foc_seq_start(&s, 50.0f);
		while (s.state != at[i]) {
			st = follow(foc_seq_step(&s, &st));
		}
		foc_seq_start(&s, -80.0f);
		zassert_equal(s.dir, 1.0f);
		zassert_equal(s.target, 50.0f);
		foc_seq_stop(&s);
		st = follow(foc_seq_step(&s, &st));
		zassert_equal(st.state, FOC_STATE_STOP);
		zassert_equal(s.state, FOC_STATE_STOP);
		/* A restart from STOP works at once. */
		foc_seq_start(&s, 50.0f);
		zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_ALIGN);
	}
}

/* A FAULT status latches with its bits; only stop leaves it. */
ZTEST(foc_seq, test_fault_latches)
{
	struct foc_seq s;
	struct foc_status st = {0};

	foc_seq_init(&s, &cfg);
	foc_seq_start(&s, 50.0f);
	while (s.state != FOC_STATE_IF) {
		st = follow(foc_seq_step(&s, &st));
	}
	st = (struct foc_status){.state = FOC_STATE_FAULT, .faults = FOC_FAULT_UV};
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_FAULT);
	zassert_equal(s.faults, FOC_FAULT_UV);
	/* Later bits do not overwrite the cause; the status clearing does not leave FAULT. */
	st.faults = FOC_FAULT_UV | FOC_FAULT_OC;
	(void)foc_seq_step(&s, &st);
	st = (struct foc_status){.state = FOC_STATE_STOP};
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_FAULT);
	zassert_equal(s.faults, FOC_FAULT_UV);
	foc_seq_start(&s, 50.0f);
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_FAULT);

	foc_seq_stop(&s);
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_STOP);
	/* A stale FAULT status while stopped does not latch again. */
	st.state = FOC_STATE_FAULT;
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_STOP);
	/* Start waits until the controller is off as well. */
	foc_seq_start(&s, 50.0f);
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_STOP);
	zassert_equal(s.state, FOC_STATE_STOP);
	st.state = FOC_STATE_STOP;
	zassert_equal(foc_seq_step(&s, &st)->state, FOC_STATE_ALIGN);
	zassert_equal(s.faults, 0U);
}

ZTEST_SUITE(foc_seq, NULL, NULL, NULL, NULL, NULL);
