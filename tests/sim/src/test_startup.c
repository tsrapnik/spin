/*
 * SPDX-License-Identifier: MIT
 *
 * Full sensorless drive from standstill: align, I/f, handover, closed loop,
 * then speed and load steps and a stop, all through foc_seq and foc_cycle.
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "drive.h"
#include "sim.h"

static void start(struct drive *d, double theta0, float w)
{
	drive_init(d, &sim_motor, &drive_cycle_cfg, &drive_seq_cfg);
	d->p.theta_e = theta0;
	foc_seq_start(&d->s, w);
}

/* Largest |angle error| and speed range over t seconds. */
struct span {
	double ang, lo, hi;
};

static struct span watch(struct drive *d, float t)
{
	struct span r = {.lo = 1e30, .hi = -1e30};
	unsigned int n = (unsigned int)(t / (float)d->p.ts + 0.5f);

	for (unsigned int k = 0; k < n; k++) {
		drive_cycle(d);
		zassert_true(d->pwm, "PWM off at t %f, faults %x", d->p.t, d->st.faults);
		r.ang = fmax(r.ang, fabs(drive_angle_err(d)));
		r.lo = fmin(r.lo, d->p.wm);
		r.hi = fmax(r.hi, d->p.wm);
	}
	return r;
}

/* Align time plus the I/f ramp to the handover speed. */
static double t_handover(void)
{
	return (double)drive_seq_cfg.t_align +
	       (double)(drive_seq_cfg.w_handover / drive_seq_cfg.accel_start);
}

/* Standstill to 50 rad/s from any rotor angle, both directions. */
ZTEST(sim_startup, test_start_any_angle)
{
	const double th0[] = {0.0, 1.0, 2.0, 3.0, PLANT_PI, -1.5, -2.5};
	const float ws[] = {50.0f, -50.0f};

	for (size_t w = 0; w < ARRAY_SIZE(ws); w++) {
		for (size_t n = 0; n < ARRAY_SIZE(th0); n++) {
			struct drive d;
			double t_run;
			struct span r;

			start(&d, th0[n], ws[w]);
			t_run = drive_until(&d, FOC_STATE_RUN, 0.3f);
			zassert_within(t_run, t_handover(), 3e-3, "th0 %f w %f: RUN at %f", th0[n],
				       (double)ws[w], t_run);
			/* Rotor follows the forced angle in the start direction. */
			zassert_true(d.p.wm * (double)ws[w] > 0.0, "th0 %f: wm %f", th0[n], d.p.wm);
			r = watch(&d, 0.2f);
			zassert_true(r.ang < 0.05, "th0 %f w %f: angle %e", th0[n], (double)ws[w],
				     r.ang);
			zassert_true(fabs(r.hi) < 1.05 * 50.0 && fabs(r.lo) < 1.05 * 50.0,
				     "th0 %f: overshoot %f %f", th0[n], r.lo, r.hi);
			r = watch(&d, 0.1f);
			zassert_within(d.p.wm, (double)ws[w], 1e-3, "th0 %f: wm %f", th0[n],
				       d.p.wm);
			zassert_true(r.ang < 1e-3, "th0 %f: steady angle %e", th0[n], r.ang);
		}
	}
}

/*
 * Against 0.05 Nm of Coulomb friction (about 30 % of the torque i_start
 * makes): the rotor lags the forced angle, and the preload carries the
 * torque over the handover, so the speed does not dip.
 */
ZTEST(sim_startup, test_start_loaded)
{
	const double load = 0.05;
	const double kt = (double)foc_torque_constant(sim_motor.psi, sim_motor.pole_pairs);
	struct drive d;
	struct span r;

	start(&d, 0.5, 50.0f);
	d.p.t_fric = load;
	zassert_true(drive_until(&d, FOC_STATE_IF, 0.2f) > 0.0);
	/* Past the PLL release at w_handover / 2, before the handover. */
	drive_run(&d, 0.08f);
	zassert_equal(d.s.state, FOC_STATE_IF);
	/* Lag from Kt i_start sin(delta) = load (+ acceleration), on the PLL. */
	double delta = asin(load / (kt * (double)drive_seq_cfg.i_start));

	zassert_within((double)d.st.delta, delta, 0.1, "delta %f expected %f", (double)d.st.delta,
		       delta);
	zassert_true(drive_until(&d, FOC_STATE_RUN, 0.1f) > 0.0);
	zassert_within((double)d.s.speed.integ, load / kt, 0.15 * load / kt, "preload %f",
		       (double)d.s.speed.integ);
	r = watch(&d, 0.02f);
	zassert_true(r.lo > 0.9 * (double)drive_seq_cfg.w_handover, "dip to %f", r.lo);
	zassert_true(r.ang < 0.1, "angle %e", r.ang);
	r = watch(&d, 0.3f);
	zassert_within(d.p.wm, 50.0, 1e-3);
	zassert_within(d.p.iq, load / kt, 1e-2 * load / kt);
}

/* Speed steps, a load step and a target below the handover speed. */
ZTEST(sim_startup, test_speed_and_load_steps)
{
	struct drive d;
	struct span r;

	start(&d, 0.0, 40.0f);
	(void)watch(&d, 0.4f);
	zassert_within(d.p.wm, 40.0, 1e-3);

	foc_seq_set_speed(&d.s, 90.0f);
	r = watch(&d, 0.3f);
	zassert_true(r.hi < 1.03 * 90.0, "overshoot %f", r.hi);
	zassert_true(r.ang < 0.05, "up: angle %e", r.ang);
	zassert_within(d.p.wm, 90.0, 1e-3);

	d.p.t_load = 0.05;
	r = watch(&d, 0.3f);
	zassert_true(r.lo > 0.6 * 90.0, "dip %f", r.lo);
	zassert_true(r.ang < 0.05, "load: angle %e", r.ang);
	zassert_within(d.p.wm, 90.0, 1e-3);

	/* Below the handover speed, or reversed: held at w_handover. */
	foc_seq_set_speed(&d.s, -40.0f);
	r = watch(&d, 0.4f);
	zassert_true(r.ang < 0.05, "down: angle %e", r.ang);
	zassert_within(d.p.wm, (double)drive_seq_cfg.w_handover, 1e-3, "wm %f", d.p.wm);
	zassert_true(r.lo > 0.8 * (double)drive_seq_cfg.w_handover, "low %f", r.lo);
}

/* Stop: PWM off, the rotor coasts to rest; a restart then works again. */
ZTEST(sim_startup, test_stop_and_restart)
{
	struct drive d;

	start(&d, 0.0, 50.0f);
	drive_run(&d, 0.4f);
	zassert_within(d.p.wm, 50.0, 1e-3);
	d.p.t_fric = 0.02;
	foc_seq_stop(&d.s);
	drive_run(&d, 0.002f);
	zassert_equal(d.s.state, FOC_STATE_STOP);
	zassert_equal(d.st.state, FOC_STATE_STOP);
	zassert_false(d.pwm);
	zassert_equal(d.p.id, 0.0);
	zassert_equal(d.p.iq, 0.0);
	drive_run(&d, 0.3f);
	zassert_true(fabs(d.p.wm) < 0.1, "still turning at %f", d.p.wm);
	zassert_equal(d.st.state, FOC_STATE_STOP);
	foc_seq_start(&d.s, -40.0f);
	zassert_true(drive_until(&d, FOC_STATE_RUN, 0.3f) > 0.0);
	(void)watch(&d, 0.3f);
	zassert_within(d.p.wm, -40.0, 1e-3);
}

/*
 * Observer (and current gains) with Rs or Ls 30 % off, against friction.
 * In I/f a wrong Rs turns the resistive drop of the d current into a false
 * EMF along d, so the PLL is up to ~0.6 rad off at the handover (Rs x0.7).
 * The switch then costs a speed bump, but the loop locks within 50 ms.
 */
ZTEST(sim_startup, test_start_parameter_error)
{
	const float sc[] = {0.7f, 1.3f};

	for (size_t i = 0; i < ARRAY_SIZE(sc); i++) {
		for (size_t k = 0; k < 2U; k++) {
			struct foc_cycle_cfg cc = drive_cycle_cfg;
			struct drive d;
			struct span hand, r;

			if (k == 0U) {
				cc.rs *= sc[i];
			} else {
				cc.ls *= sc[i];
			}
			drive_init(&d, &sim_motor, &cc, &drive_seq_cfg);
			d.p.t_fric = 0.03;
			foc_seq_start(&d.s, 50.0f);
			zassert_true(drive_until(&d, FOC_STATE_RUN, 0.3f) > 0.0);
			hand = watch(&d, 0.05f);
			r = watch(&d, 0.35f);
			printk("x%.1f %s: handover angle %.3f speed %.1f..%.1f\n", (double)sc[i],
			       k ? "ls" : "rs", hand.ang, hand.lo, hand.hi);
			zassert_true(hand.ang < 0.8 && hand.hi < 60.0, "x%f %s: handover %e %f",
				     (double)sc[i], k ? "ls" : "rs", hand.ang, hand.hi);
			zassert_true(r.ang < 0.03, "x%f %s: angle %e", (double)sc[i],
				     k ? "ls" : "rs", r.ang);
			zassert_within(d.p.wm, 50.0, 1e-2, "x%f: wm %f", (double)sc[i], d.p.wm);
		}
	}
}

ZTEST_SUITE(sim_startup, NULL, NULL, NULL, NULL, NULL);
