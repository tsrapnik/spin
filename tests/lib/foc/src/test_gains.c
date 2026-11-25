/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <pmsm/foc.h>

ZTEST(foc_gains, test_current_values)
{
	struct foc_pi_gains g = foc_gains_current(2.0f, 1e-3f, 1000.0f);

	zassert_within(g.kp, 2.0f * FOC_PI_F, 1e-5f);
	zassert_within(g.ki, 4000.0f * FOC_PI_F, 1e-2f);
}

ZTEST(foc_gains, test_torque_constant)
{
	zassert_within(foc_torque_constant(0.011f, 11), 0.1815f, 1e-6f);
}

/*
 * Current loop on an RL phase (exact ZOH discretisation), with the one-cycle
 * duty delay of the real system. Pole cancellation should give a first-order
 * response with time constant 1/wc.
 */
ZTEST(foc_gains, test_current_loop_rl)
{
	const float fs = 20000.0f, ts = 1.0f / fs;
	const float r = 3.6f, l = 1.5e-3f, bw = 200.0f, vlim = 12.0f;
	const float a = expf(-r * ts / l);
	const int n_tau = (int)lroundf(fs / (FOC_2PI_F * bw));
	struct foc_pi pi;
	float i = 0.0f, v_applied = 0.0f, i_peak = 0.0f, i_tau = 0.0f;

	foc_pi_init(&pi, foc_gains_current(r, l, bw), ts);
	for (int k = 0; k < 20 * n_tau; k++) {
		float v_next = foc_pi_step(&pi, 1.0f - i, -vlim, vlim);

		i = a * i + (1.0f - a) / r * v_applied;
		v_applied = v_next;
		i_peak = fmaxf(i_peak, i);
		if (k == n_tau) {
			i_tau = i;
		}
	}

	zassert_within(i_tau, 0.632f, 0.06f, "i(tau) %f", (double)i_tau);
	zassert_true(i_peak < 1.02f, "overshoot, peak %f", (double)i_peak);
	zassert_within(i, 1.0f, 1e-3f);
}

/* Speed loop on Kt/(J s) with an ideal current loop, run at 1 kHz. */
ZTEST(foc_gains, test_speed_loop_inertia)
{
	const float ts = 1e-3f, j = 6e-6f, bw = 10.0f, imax = 1.4f, ref = 50.0f;
	const float kt = foc_torque_constant(0.011f, 11);
	struct foc_pi pi;
	float w = 0.0f, w_peak = 0.0f;

	foc_pi_init(&pi, foc_gains_speed(j, kt, bw), ts);
	for (int k = 0; k < 1000; k++) {
		float iq = foc_pi_step(&pi, ref - w, -imax, imax);

		w += ts * kt / j * iq;
		w_peak = fmaxf(w_peak, w);
	}

	zassert_true(w_peak < 1.4f * ref, "overshoot, peak %f", (double)w_peak);
	zassert_within(w, ref, 0.01f * ref, "w %f", (double)w);
}

ZTEST_SUITE(foc_gains, NULL, NULL, NULL, NULL, NULL);
