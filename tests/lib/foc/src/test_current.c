/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

#define VBUS     24.0f
#define MAX_DUTY 0.95f
#define TS       50e-6f

static const struct foc_pi_gains g = {.kp = 10.0f, .ki = 20000.0f};

static struct foc_abc zero_i(void)
{
	return (struct foc_abc){0};
}

ZTEST(foc_current, test_zero_ref_zero_voltage)
{
	struct foc_current c;
	struct foc_abc d;

	foc_current_init(&c, g, TS, MAX_DUTY, FOC_ADVANCE_PERIODS * TS);
	d = foc_current_step(&c, zero_i(), VBUS, 0.3f, 100.0f, (struct foc_dq){0});
	zassert_equal(d.a, 0.5f);
	zassert_equal(d.b, 0.5f);
	zassert_equal(d.c, 0.5f);
}

/* Measured current is reported in the rotor frame. */
ZTEST(foc_current, test_measured_park)
{
	struct foc_current c;
	const float th = 0.7f;
	struct foc_abc i = foc_inv_clarke(
		foc_inv_park((struct foc_dq){.d = 0.2f, .q = -1.1f}, foc_sincos(th)));

	foc_current_init(&c, g, TS, MAX_DUTY, 0.0f);
	(void)foc_current_step(&c, i, VBUS, th, 0.0f, (struct foc_dq){0});
	zassert_within(c.i.d, 0.2f, 1e-5f);
	zassert_within(c.i.q, -1.1f, 1e-5f);
}

/* The applied voltage points along q, rotated by the angle advance. */
ZTEST(foc_current, test_angle_advance)
{
	struct foc_current c;
	const float th = 1.0f, w = 1000.0f, t_adv = FOC_ADVANCE_PERIODS * TS;
	struct foc_abc d;
	struct foc_ab v;

	foc_current_init(&c, g, TS, MAX_DUTY, t_adv);
	d = foc_current_step(&c, zero_i(), VBUS, th, w, (struct foc_dq){.q = 0.5f});
	v = foc_duty_to_ab(d, VBUS);

	float ang = atan2f(v.beta, v.alpha);
	float expect = th + w * t_adv + 0.5f * FOC_PI_F;

	zassert_true(c.v.q > 0.0f && c.v.d == 0.0f);
	zassert_within(foc_wrap_pi(ang - expect), 0.0f, 1e-4f, "ang %f", (double)ang);
	zassert_within(hypotf(v.alpha, v.beta), c.v.q, 1e-4f);
}

/* Large references: the voltage stays on the SVPWM circle, d first. */
ZTEST(foc_current, test_voltage_limit_d_first)
{
	struct foc_current c;
	const float vmax = foc_svpwm_vmax(VBUS, MAX_DUTY);
	struct foc_abc d;

	foc_current_init(&c, g, TS, MAX_DUTY, 0.0f);
	for (int k = 0; k < 50; k++) {
		d = foc_current_step(&c, zero_i(), VBUS, 0.0f, 0.0f,
				     (struct foc_dq){.d = 0.1f, .q = 100.0f});
		zassert_true(hypotf(c.v.d, c.v.q) <= vmax * (1.0f + 1e-6f));
		zassert_true(d.a <= MAX_DUTY && d.a >= 1.0f - MAX_DUTY);
	}
	zassert_true(c.v.d > 0.0f && c.v.d < vmax);
	zassert_within(hypotf(c.v.d, c.v.q), vmax, 1e-4f, "q takes what d leaves");

	for (int k = 0; k < 50; k++) {
		(void)foc_current_step(&c, zero_i(), VBUS, 0.0f, 0.0f,
				       (struct foc_dq){.d = 100.0f, .q = 100.0f});
	}
	zassert_within(c.v.d, vmax, 1e-5f);
	zassert_within(c.v.q, 0.0f, 1e-5f);
}

/* After saturation the integrators sit at the limit, not beyond it. */
ZTEST(foc_current, test_no_windup)
{
	struct foc_current c;
	const float vmax = foc_svpwm_vmax(VBUS, MAX_DUTY);

	foc_current_init(&c, g, TS, MAX_DUTY, 0.0f);
	for (int k = 0; k < 1000; k++) {
		(void)foc_current_step(&c, zero_i(), VBUS, 0.0f, 0.0f,
				       (struct foc_dq){.q = 100.0f});
	}
	zassert_true(c.q.integ <= vmax);
	/* Reference to 0 with the current at 0: the output drops at once. */
	(void)foc_current_step(&c, zero_i(), VBUS, 0.0f, 0.0f, (struct foc_dq){0});
	zassert_true(c.v.q <= vmax);
	foc_current_reset(&c);
	zassert_equal(c.d.integ, 0.0f);
	zassert_equal(c.q.integ, 0.0f);
}

ZTEST(foc_current, test_init_sanitizes)
{
	struct foc_current c;

	foc_current_init(&c, g, TS, NAN, NAN);
	zassert_equal(c.max_duty, 0.5f);
	zassert_equal(c.t_adv, 0.0f);
	foc_current_init(&c, g, TS, 2.0f, -1.0f);
	zassert_equal(c.max_duty, 1.0f);
	zassert_equal(c.t_adv, 0.0f);
}

ZTEST_SUITE(foc_current, NULL, NULL, NULL, NULL, NULL);
