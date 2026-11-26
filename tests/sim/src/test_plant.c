/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#include "plant.h"

#define RS      3.6f
#define LS      1.5e-3f
#define PSI     0.011f
#define PP      11U
#define VBUS    24.0f
#define TS      50e-6f
/* Large enough that the speed stays constant: a "locked" or "driven" rotor. */
#define J_STIFF 1e6f

static const struct plant_params base = {
	.rs = RS,
	.ls = LS,
	.psi = PSI,
	.pole_pairs = PP,
	.j = 6e-6f,
	.b = 0.0f,
	.vbus = VBUS,
	.ts = TS,
	.duty_delay = 1U,
	.substeps = 8U,
};

/* Duties that apply the stationary-frame voltage v (max duty 1: no clamping). */
static struct foc_abc duty_for(float valpha, float vbeta)
{
	return foc_svpwm((struct foc_ab){.alpha = valpha, .beta = vbeta}, VBUS, 1.0f);
}

static void init(struct plant *p, struct plant_params prm)
{
	zassert_equal(plant_init(p, &prm), 0);
}

ZTEST(plant, test_init_rejects_bad_params)
{
	struct plant p;
	struct plant_params prm = base;

	prm.ls = 0.0f;
	zassert_equal(plant_init(&p, &prm), -1);
	prm = base;
	prm.rs = NAN;
	zassert_equal(plant_init(&p, &prm), -1);
	prm = base;
	prm.pole_pairs = 0U;
	zassert_equal(plant_init(&p, &prm), -1);
	prm = base;
	prm.duty_delay = PLANT_DUTY_DELAY_MAX + 1U;
	zassert_equal(plant_init(&p, &prm), -1);
}

/* Locked rotor at theta = 0: alpha voltage step gives the RL response on d. */
ZTEST(plant, test_rl_step_locked_rotor)
{
	struct plant_params prm = base;
	struct plant p;
	const double tau = (double)LS / (double)RS;

	prm.j = J_STIFF;
	init(&p, prm);

	for (int k = 1; k <= 200; k++) {
		plant_step(&p, duty_for(RS, 0.0f));

		/* One period of delay: the voltage acts from the second step on. The
		 * reference uses the voltage actually applied (float duty rounding).
		 */
		double i_inf = (double)p.va.alpha / (double)RS;
		double t = (double)(k - 1) * (double)TS;
		double ref = i_inf * (1.0 - exp(-t / tau));
		struct plant_sample s = plant_sample(&p);

		zassert_within(p.id, ref, 1e-7, "k %d id %f ref %f", k, p.id, ref);
		zassert_within(p.iq, 0.0, 1e-12);
		zassert_within((double)s.ia, p.id, 1e-6);
		zassert_within((double)s.ib, -0.5 * p.id, 1e-6);
		zassert_within((double)s.ic, -0.5 * p.id, 1e-6);
		zassert_equal(s.vbus, VBUS);
	}
	zassert_within(p.wm, 0.0, 1e-12, "no torque from d current");
}

ZTEST(plant, test_duty_delay)
{
	struct foc_abc on = duty_for(5.0f, 0.0f);

	for (unsigned int delay = 0U; delay <= PLANT_DUTY_DELAY_MAX; delay++) {
		struct plant_params prm = base;
		struct plant p;

		prm.duty_delay = delay;
		init(&p, prm);
		plant_step(&p, on);
		for (unsigned int k = 1U; k <= delay; k++) {
			zassert_equal(p.id, 0.0, "delay %u step %u", delay, k);
			zassert_equal(p.applied.a, 0.5f);
			zassert_equal(p.va.alpha, 0.0f);
			plant_step(&p, on);
		}
		zassert_true(p.id > 0.0, "delay %u", delay);
		zassert_equal(p.applied.a, on.a);
		zassert_within(p.va.alpha, 5.0f, 1e-5f);
	}
}

ZTEST(plant, test_duty_history_order)
{
	struct plant_params prm = base;
	struct plant p;

	prm.duty_delay = PLANT_DUTY_DELAY_MAX;
	init(&p, prm);
	for (int k = 0; k < 10; k++) {
		plant_step(&p, (struct foc_abc){.a = 0.1f * (float)k, .b = 0.5f, .c = 0.5f});
		float expect = (k < (int)PLANT_DUTY_DELAY_MAX)
				       ? 0.5f
				       : 0.1f * (float)(k - (int)PLANT_DUTY_DELAY_MAX);

		zassert_equal(p.applied.a, expect, "k %d", k);
	}
}

ZTEST(plant, test_common_mode_and_clamp)
{
	struct plant p;

	init(&p, base);
	for (int k = 0; k < 20; k++) {
		plant_step(&p, (struct foc_abc){.a = 0.8f, .b = 0.8f, .c = 0.8f});
	}
	zassert_equal(p.id, 0.0);
	zassert_equal(p.iq, 0.0);

	/* Out-of-range and NaN duties clamp to [0, 1]: (1, 0, 0) gives alpha = 2/3 Vbus. */
	init(&p, base);
	plant_step(&p, (struct foc_abc){.a = 7.0f, .b = -1.0f, .c = NAN});
	plant_step(&p, (struct foc_abc){.a = 0.5f, .b = 0.5f, .c = 0.5f});
	zassert_within(p.va.alpha, 2.0f / 3.0f * VBUS, 1e-5f);
	zassert_within(p.va.beta, 0.0f, 1e-6f);
	zassert_true(isfinite(p.id) && p.id > 0.0);
}

/*
 * Rotor driven at constant speed, windings shorted (zero voltage). The
 * steady state checks the cross-coupling and back-EMF signs:
 *   0 = -R id + we L iq,   0 = -R iq - we L id - we psi
 */
ZTEST(plant, test_short_circuit_steady_state)
{
	struct plant_params prm = base;
	struct plant p;
	const double wm = 100.0, we = (double)PP * wm;
	const double r = (double)RS, x = we * (double)LS, psi = (double)PSI;
	const double den = r * r + x * x;
	const double id_ref = -x * we * psi / den;
	const double iq_ref = -r * we * psi / den;
	const struct foc_abc zero = {.a = 0.5f, .b = 0.5f, .c = 0.5f};

	prm.j = J_STIFF;
	init(&p, prm);
	p.wm = wm;

	for (int k = 1; k <= 2000; k++) {
		plant_step(&p, zero);
		zassert_true(p.theta_e >= -PLANT_PI && p.theta_e <= PLANT_PI, "k %d", k);
	}

	zassert_within(p.id, id_ref, 1e-6 * fabs(id_ref), "id %f ref %f", p.id, id_ref);
	zassert_within(p.iq, iq_ref, 1e-6 * fabs(iq_ref), "iq %f ref %f", p.iq, iq_ref);
	zassert_within(p.wm, wm, 1e-6);

	/* Angle advanced by we * t, wrapped. */
	double th_ref = remainder(we * p.t, 2.0 * PLANT_PI);

	zassert_within(remainder(p.theta_e - th_ref, 2.0 * PLANT_PI), 0.0, 1e-6, "theta %f ref %f",
		       p.theta_e, th_ref);

	/* The sample is the same current seen in the stationary frame. */
	struct plant_sample s = plant_sample(&p);
	struct foc_ab iab = foc_clarke((struct foc_abc){.a = s.ia, .b = s.ib, .c = s.ic});
	struct foc_dq idq = foc_park(iab, foc_sincos((float)p.theta_e));

	zassert_within((double)(s.ia + s.ib + s.ic), 0.0, 1e-6);
	zassert_within((double)idq.d, p.id, 1e-5);
	zassert_within((double)idq.q, p.iq, 1e-5);
}

/* Positive q current accelerates the rotor: J dw/dt = 1.5 p psi iq. */
ZTEST(plant, test_torque_accelerates)
{
	struct plant p;
	double impulse = 0.0;

	init(&p, base);
	/* At theta = 0 the q axis is beta. */
	for (int k = 0; k < 40; k++) {
		double te0 = plant_torque(&p);

		plant_step(&p, duty_for(0.0f, 3.0f));
		impulse += 0.5 * (te0 + plant_torque(&p)) * (double)TS;
	}

	zassert_true(p.iq > 0.0 && p.wm > 0.0 && p.theta_e > 0.0);
	zassert_within(plant_torque(&p), 1.5 * (double)PP * (double)PSI * p.iq, 1e-12);
	/* Trapezoidal impulse vs. momentum, 0.5 % (the integration is coarse). */
	zassert_within(p.wm * (double)base.j, impulse, 5e-3 * impulse, "Jw %g impulse %g",
		       p.wm * (double)base.j, impulse);
}

/* psi = 0: no torque, only friction and load act on the rotor. */
ZTEST(plant, test_mechanics)
{
	struct plant_params prm = base;
	struct plant p;
	const struct foc_abc zero = {.a = 0.5f, .b = 0.5f, .c = 0.5f};
	const double wm0 = 100.0, j = (double)prm.j;

	/* Viscous friction: exponential decay with time constant J / B. */
	prm.psi = 0.0f;
	prm.b = 1e-5f;
	init(&p, prm);
	p.wm = wm0;
	for (int k = 0; k < 1000; k++) {
		plant_step(&p, zero);
	}
	double ref = wm0 * exp(-(double)prm.b * p.t / j);

	zassert_within(p.wm, ref, 1e-9 * wm0, "wm %f ref %f", p.wm, ref);

	/* Constant load torque: linear deceleration (and reversal) at t_load / J. */
	prm.b = 0.0f;
	init(&p, prm);
	p.wm = wm0;
	p.t_load = 2e-2;
	for (int k = 0; k < 1000; k++) {
		plant_step(&p, zero);
	}
	ref = wm0 - p.t_load * p.t / j;
	zassert_within(p.wm, ref, 1e-9 * wm0, "wm %f ref %f", p.wm, ref);
	zassert_true(p.wm < 0.0, "the load is active, it reverses the rotor");
}

ZTEST_SUITE(plant, NULL, NULL, NULL, NULL, NULL);
