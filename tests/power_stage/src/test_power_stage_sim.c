/*
 * SPDX-License-Identifier: MIT
 *
 * Simulated power stage: API contract (timing, duty handling, offsets,
 * break) against the plant behind it.
 */

#include <float.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>
#include <pmsm/power_stage.h>
#include <pmsm/power_stage_sim.h>

#include "ps_test.h"

#define VBUS 24.0f

/* What the recording callback saw. */
struct rec {
	unsigned int n;
	struct power_stage_sample last;
	unsigned int brk; /* Samples with POWER_STAGE_FLAG_BREAK. */
	const struct device *dev;
	struct foc_abc duty; /* Set every cycle if set_duty is true. */
	bool set_duty;
};

static struct rec rec;

static void record(const struct device *dev, const struct power_stage_sample *s, void *user)
{
	struct rec *r = user;

	r->n++;
	r->last = *s;
	r->dev = dev;
	if ((s->flags & POWER_STAGE_FLAG_BREAK) != 0U) {
		r->brk++;
	}
	if (r->set_duty) {
		power_stage_set_duty(dev, r->duty.a, r->duty.b, r->duty.c);
	}
}

static struct plant plant_copy(const struct device *dev)
{
	k_spinlock_key_t key;
	struct plant p = *power_stage_sim_lock(dev, &key);

	power_stage_sim_unlock(dev, key);
	return p;
}

static void run(unsigned int n)
{
	zassert_ok(power_stage_sim_run(ps_manual, n));
}

static void before(void *f)
{
	ARG_UNUSED(f);
	ps_test_reset(ps_manual);
	rec = (struct rec){0};
	zassert_ok(power_stage_register_cb(ps_manual, record, &rec));
}

static void after(void *f)
{
	ARG_UNUSED(f);
	ps_test_reset(ps_manual);
}

ZTEST(power_stage_sim, test_info)
{
	const struct power_stage_info *info = power_stage_get_info(ps_manual);

	zassert_within(info->ts, 50e-6f, 1e-12f);
	zassert_within(info->max_duty, 0.95f, 1e-6f);
	zassert_equal(info->duty_delay, 1U);
	zassert_equal(info->dead_time, 0.0f);
}

/* One callback per cycle, with the plant's sample, whether the outputs are on or off. */
ZTEST(power_stage_sim, test_callback_every_cycle)
{
	uint64_t c0 = power_stage_sim_cycles(ps_manual);

	run(10U);
	zassert_equal(rec.n, 10U);
	zassert_equal(rec.dev, ps_manual);
	zassert_equal(power_stage_sim_cycles(ps_manual) - c0, 10U);
	zassert_within(rec.last.vbus, VBUS, 1e-6f);
	zassert_equal(rec.last.flags, 0U);

	zassert_ok(power_stage_enable(ps_manual));
	run(10U);
	zassert_equal(rec.n, 20U);
}

/* The sample is the plant state before the period, the currents are its phase currents. */
ZTEST(power_stage_sim, test_sample_matches_plant)
{
	rec.duty = (struct foc_abc){.a = 0.6f, .b = 0.45f, .c = 0.45f};
	rec.set_duty = true;
	zassert_ok(power_stage_enable(ps_manual));
	run(50U);

	struct plant p = plant_copy(ps_manual);
	struct plant_sample want = plant_sample(&p);

	run(1U);
	zassert_within(rec.last.ia, want.ia, 1e-6f);
	zassert_within(rec.last.ib, want.ib, 1e-6f);
	zassert_within(rec.last.ic, want.ic, 1e-6f);
	zassert_true(rec.last.ia > 0.1f, "no current with the outputs on: %f", (double)rec.last.ia);
}

/* A duty written in cycle k is applied in the period after cycle k + delay. */
ZTEST(power_stage_sim, test_duty_delay)
{
	const struct foc_abc d = {.a = 0.7f, .b = 0.4f, .c = 0.5f};

	zassert_ok(power_stage_enable(ps_manual));
	run(3U);

	/* Cycle k: the callback sets d. */
	rec.duty = d;
	rec.set_duty = true;
	run(1U);
	zassert_equal(plant_copy(ps_manual).applied.a, 0.5f, "applied without delay");

	run(1U);
	struct plant p = plant_copy(ps_manual);

	zassert_equal(p.applied.a, d.a);
	zassert_equal(p.applied.b, d.b);
	zassert_equal(p.applied.c, d.c);
}

/* The duty holds when not set again, as a compare register does. */
ZTEST(power_stage_sim, test_duty_holds)
{
	zassert_ok(power_stage_enable(ps_manual));
	power_stage_set_duty(ps_manual, 0.6f, 0.5f, 0.4f);
	run(5U);
	zassert_equal(plant_copy(ps_manual).applied.a, 0.6f);
	zassert_equal(plant_copy(ps_manual).applied.c, 0.4f);
}

/* Duties are clamped to [1 - max_duty, max_duty]; NaN counts as 0.5. */
ZTEST(power_stage_sim, test_duty_clamp)
{
	/* As the driver computes them (950 * 1e-3f, 1 - that), not the literals. */
	const float hi = power_stage_get_info(ps_manual)->max_duty;
	const float lo = 1.0f - hi;
	const struct {
		float in, out;
	} v[] = {
		{0.0f, lo},     {1.0f, hi},    {-INFINITY, lo}, {INFINITY, hi},
		{NAN, 0.5f},    {-NAN, 0.5f},  {lo, lo},        {hi, hi},
		{-FLT_MAX, lo}, {FLT_MAX, hi}, {FLT_MIN, lo},   {0.3f, 0.3f},
	};

	zassert_ok(power_stage_enable(ps_manual));
	for (size_t i = 0; i < ARRAY_SIZE(v); i++) {
		power_stage_set_duty(ps_manual, v[i].in, 0.5f, v[i].in);
		run(2U);

		struct plant p = plant_copy(ps_manual);

		zassert_equal(p.applied.a, v[i].out, "in %f: %f", (double)v[i].in,
			      (double)p.applied.a);
		zassert_equal(p.applied.c, v[i].out);
		zassert_equal(p.applied.b, 0.5f);
	}
}

/* Outputs off: the plant coasts and the duty is ignored. */
ZTEST(power_stage_sim, test_outputs_off)
{
	rec.duty = (struct foc_abc){.a = 0.9f, .b = 0.1f, .c = 0.5f};
	rec.set_duty = true;
	run(100U);

	struct plant p = plant_copy(ps_manual);

	zassert_equal(p.id, 0.0);
	zassert_equal(p.iq, 0.0);
	zassert_equal(p.applied.a, 0.5f);
	zassert_within(p.t, 101.0 * 50e-6, 1e-9, "time advances while off");
}

/* Disable switches off at once and resets the duty; enable resumes at 0.5. */
ZTEST(power_stage_sim, test_disable)
{
	zassert_ok(power_stage_enable(ps_manual));
	power_stage_set_duty(ps_manual, 0.7f, 0.4f, 0.4f);
	run(40U);
	zassert_true(fabs(plant_copy(ps_manual).id) + fabs(plant_copy(ps_manual).iq) > 0.1);

	power_stage_disable(ps_manual);
	run(1U);
	zassert_equal(plant_copy(ps_manual).id, 0.0);

	zassert_ok(power_stage_enable(ps_manual));
	run(3U);

	struct plant p = plant_copy(ps_manual);

	zassert_equal(p.applied.a, 0.5f);
	zassert_equal(p.applied.b, 0.5f);
}

/* Callback changes and calibration need the outputs off. */
ZTEST(power_stage_sim, test_busy_when_on)
{
	zassert_ok(power_stage_enable(ps_manual));
	zassert_equal(power_stage_register_cb(ps_manual, NULL, NULL), -EBUSY);
	zassert_equal(power_stage_calibrate_offsets(ps_manual), -EBUSY);
	run(1U);
	zassert_equal(rec.n, 1U, "callback was replaced");
}

/* Calibration removes an injected sense offset. */
ZTEST(power_stage_sim, test_offset_calibration)
{
	power_stage_sim_set_offset(ps_manual, 0.12f, -0.05f, 0.3f);
	run(1U);
	zassert_within(rec.last.ia, 0.12f, 1e-6f);
	zassert_within(rec.last.ic, 0.3f, 1e-6f);

	zassert_ok(power_stage_calibrate_offsets(ps_manual));
	run(1U);
	zassert_within(rec.last.ia, 0.0f, 1e-6f);
	zassert_within(rec.last.ib, 0.0f, 1e-6f);
	zassert_within(rec.last.ic, 0.0f, 1e-6f);

	/* Calibrated offsets stay removed with current flowing. */
	zassert_ok(power_stage_enable(ps_manual));
	power_stage_set_duty(ps_manual, 0.6f, 0.45f, 0.45f);
	run(50U);

	struct plant p = plant_copy(ps_manual);
	struct plant_sample want = plant_sample(&p);

	run(1U);
	zassert_within(rec.last.ia, want.ia, 1e-5f);
	zassert_within(rec.last.ib, want.ib, 1e-5f);
	zassert_within(rec.last.ic, want.ic, 1e-5f);
}

/* An offset beyond the sensing range is rejected; the old calibration stays. */
ZTEST(power_stage_sim, test_offset_out_of_range)
{
	power_stage_sim_set_offset(ps_manual, 2.0f, 0.0f, 0.0f);
	zassert_equal(power_stage_calibrate_offsets(ps_manual), -ERANGE);
	power_stage_sim_set_offset(ps_manual, NAN, 0.0f, 0.0f);
	zassert_equal(power_stage_calibrate_offsets(ps_manual), -ERANGE);

	power_stage_sim_set_offset(ps_manual, 0.0f, 0.0f, 0.0f);
	run(1U);
	zassert_within(rec.last.ia, 0.0f, 1e-6f);
}

/* The break switches off at once, is reported in one sample, and blocks enable while active. */
ZTEST(power_stage_sim, test_break)
{
	zassert_ok(power_stage_enable(ps_manual));
	power_stage_set_duty(ps_manual, 0.7f, 0.4f, 0.4f);
	run(20U);

	power_stage_sim_brk(ps_manual, true);
	run(1U);
	zassert_equal(rec.brk, 1U);
	zassert_equal(rec.last.flags, POWER_STAGE_FLAG_BREAK);
	zassert_equal(plant_copy(ps_manual).id, 0.0, "outputs still on");
	zassert_equal(power_stage_enable(ps_manual), -EIO);

	run(5U);
	zassert_equal(rec.brk, 1U, "break reported more than once");
	zassert_equal(plant_copy(ps_manual).id, 0.0);

	power_stage_sim_brk(ps_manual, false);
	zassert_ok(power_stage_enable(ps_manual));
	power_stage_set_duty(ps_manual, 0.7f, 0.4f, 0.4f);
	run(20U);
	zassert_true(plant_copy(ps_manual).id > 0.1);
	zassert_equal(rec.brk, 1U);
}

/* Disable from the callback (the controller's fault path) acts in the same cycle. */
static void disable_in_cb(const struct device *dev, const struct power_stage_sample *s, void *user)
{
	ARG_UNUSED(s);
	ARG_UNUSED(user);
	power_stage_set_duty(dev, 0.9f, 0.1f, 0.1f);
	power_stage_disable(dev);
}

ZTEST(power_stage_sim, test_disable_in_callback)
{
	zassert_ok(power_stage_register_cb(ps_manual, disable_in_cb, NULL));
	zassert_ok(power_stage_enable(ps_manual));
	run(1U);

	struct plant p = plant_copy(ps_manual);

	zassert_equal(p.id, 0.0);
	zassert_equal(p.applied.a, 0.5f);
}

/* The timer instance runs on its own, in batches; manual stepping is refused. */
ZTEST(power_stage_sim, test_timer_source)
{
	struct rec r = {0};

	ps_test_reset(ps_timer);
	zassert_equal(power_stage_sim_run(ps_timer, 1U), -EBUSY);
	zassert_ok(power_stage_register_cb(ps_timer, record, &r));

	uint64_t c0 = power_stage_sim_cycles(ps_timer);

	k_msleep(100);

	uint64_t n = power_stage_sim_cycles(ps_timer) - c0;

	zassert_ok(power_stage_register_cb(ps_timer, NULL, NULL));
	/* 20 cycles per ms, run in batches of up to one kernel tick. */
	unsigned int batch = 20000U / CONFIG_SYS_CLOCK_TICKS_PER_SEC + 1U;

	zassert_true(n + batch >= 2000U && n <= 2000U + batch, "%u cycles in 100 ms",
		     (unsigned int)n);
	zassert_true(r.n + batch >= 2000U && r.n <= n);
	zassert_equal(r.dev, ps_timer);
}

ZTEST_SUITE(power_stage_sim, NULL, NULL, before, after, NULL);
