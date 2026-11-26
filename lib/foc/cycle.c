/*
 * SPDX-License-Identifier: MIT
 *
 * Per-PWM-cycle controller (ISR side), see struct foc_cycle in pmsm/foc.h.
 */

#include <stdbool.h>

#include <pmsm/foc.h>

/* Positive and finite, else 0. */
static float pos_or_zero(float x)
{
	return (x > 0.0f && x <= FLT_MAX) ? x : 0.0f;
}

void foc_cycle_init(struct foc_cycle *c, const struct foc_cycle_cfg *cfg)
{
	float ts = pos_or_zero(cfg->ts);

	*c = (struct foc_cycle){0};
	c->ts = ts;
	c->w_max = pos_or_zero(cfg->w_max);
	/* Bad limits trip on every active cycle: OC via cfg_faults, OV at 0, UV at FLT_MAX. */
	c->i_trip = pos_or_zero(cfg->i_trip);
	c->cfg_faults = (c->i_trip > 0.0f) ? 0U : FOC_FAULT_OC;
	c->vbus_max = pos_or_zero(cfg->vbus_max);
	c->vbus_min = (cfg->vbus_min >= 0.0f && cfg->vbus_min <= FLT_MAX) ? cfg->vbus_min : FLT_MAX;

	foc_current_init(&c->cur, foc_gains_current(cfg->rs, cfg->ls, cfg->bw_current_hz), ts,
			 cfg->max_duty, cfg->t_adv);
	/* State clamps: twice the trip level, twice the highest bus voltage. */
	foc_obs_init(&c->obs, cfg->rs, cfg->ls, ts, cfg->bw_obs_hz,
		     foc_clampf(2.0f * c->i_trip, 0.0f, FLT_MAX),
		     foc_clampf(2.0f * c->vbus_max, 0.0f, FLT_MAX));
	foc_pll_init(&c->pll, foc_gains_pll(cfg->bw_pll_hz), ts, c->w_max, cfg->e_min);
	foc_duty_hist_init(&c->hist, cfg->delay);
	c->state = FOC_STATE_STOP;
}

/* NaN-aware: a NaN reading fails every comparison and so trips. */
static unsigned int check(const struct foc_cycle *c, struct foc_abc i, float vbus)
{
	unsigned int f = c->cfg_faults;

	if (!(fabsf(i.a) <= c->i_trip && fabsf(i.b) <= c->i_trip && fabsf(i.c) <= c->i_trip)) {
		f |= FOC_FAULT_OC;
	}
	if (!(vbus <= c->vbus_max)) {
		f |= FOC_FAULT_OV;
	}
	if (!(vbus >= c->vbus_min)) {
		f |= FOC_FAULT_UV;
	}
	return f;
}

static void reset(struct foc_cycle *c)
{
	foc_current_reset(&c->cur);
	foc_obs_reset(&c->obs);
	foc_pll_reset(&c->pll, 0.0f, 0.0f);
	foc_duty_hist_init(&c->hist, c->hist.delay);
	c->theta_f = 0.0f;
}

/* Rotate a PI integrator pair by the angle with sine/cosine sc. */
static void rotate_integ(struct foc_current *cur, struct foc_sincos sc)
{
	float d = cur->d.integ;
	float q = cur->q.integ;

	foc_pi_reset(&cur->d, d * sc.cos - q * sc.sin);
	foc_pi_reset(&cur->q, d * sc.sin + q * sc.cos);
}

/* Follow the command; returns true if the PWM is to be on. */
static bool transition(struct foc_cycle *c, enum foc_state req)
{
	if (c->state == FOC_STATE_FAULT) {
		if (req != FOC_STATE_STOP) {
			return false;
		}
		c->faults = 0U;
		c->state = FOC_STATE_STOP;
		return false;
	}
	if (req == FOC_STATE_STOP || req == FOC_STATE_FAULT) {
		c->state = FOC_STATE_STOP;
		return false;
	}
	if (c->state == FOC_STATE_STOP) {
		reset(c);
	} else if (req == FOC_STATE_RUN && c->state != FOC_STATE_RUN) {
		/* Handover: keep the voltage vector, now in the PLL frame. */
		rotate_integ(&c->cur, foc_sincos(foc_wrap_pi(c->theta_f - c->pll.theta)));
	}
	c->state = req;
	return true;
}

static void fill_status(const struct foc_cycle *c, struct foc_status *st, float vbus)
{
	st->state = c->state;
	st->faults = c->faults;
	st->theta = c->pll.theta;
	st->omega = c->pll.omega;
	st->delta = foc_wrap_pi(c->theta_f - c->pll.theta);
	st->e_mag =
		foc_clampf(sqrtf(c->obs.e.alpha * c->obs.e.alpha + c->obs.e.beta * c->obs.e.beta),
			   0.0f, FLT_MAX);
	st->i = c->cur.i;
	st->v = c->cur.v;
	st->vbus = vbus;
	st->seq = c->seq;
}

struct foc_abc foc_cycle_step(struct foc_cycle *c, const struct foc_cmd *cmd, struct foc_abc i_abc,
			      float vbus, struct foc_status *st)
{
	const struct foc_abc zero = {.a = 0.5f, .b = 0.5f, .c = 0.5f};
	enum foc_state req = ((unsigned int)cmd->state <= (unsigned int)FOC_STATE_FAULT)
				     ? cmd->state
				     : FOC_STATE_STOP;
	struct foc_abc duty = zero;

	c->seq++;
	if (transition(c, req)) {
		unsigned int f = check(c, i_abc, vbus) | c->ext;

		c->ext = 0U;
		if (f != 0U) {
			c->faults |= f;
			c->state = FOC_STATE_FAULT;
		}
	} else {
		/* Trips while off are dropped: there is nothing to protect. */
		c->ext = 0U;
	}

	if (c->state == FOC_STATE_STOP || c->state == FOC_STATE_FAULT) {
		c->cur.i = (struct foc_dq){0};
		c->cur.v = (struct foc_dq){0};
		fill_status(c, st, vbus);
		return zero;
	}

	float w_f = foc_sanitizef(cmd->w_forced, c->w_max);
	struct foc_ab v = foc_duty_to_ab(foc_duty_hist_applied(&c->hist), vbus);
	struct foc_ab e = foc_obs_step(&c->obs, foc_clarke(i_abc), v, c->pll.omega);

	c->theta_f = foc_wrap_pi(c->theta_f + w_f * c->ts);
	if (c->state == FOC_STATE_RUN || cmd->pll_hold == 0U) {
		foc_pll_step(&c->pll, e);
	} else {
		foc_pll_reset(&c->pll, c->theta_f, w_f);
	}

	bool run = (c->state == FOC_STATE_RUN);

	if (run) {
		/* Keeps delta at 0 and a later forced angle continuous. */
		c->theta_f = c->pll.theta;
	}
	float theta = run ? c->pll.theta : c->theta_f;
	float omega = run ? c->pll.omega : w_f;
	struct foc_dq ref = {
		.d = foc_sanitizef(cmd->i_ref.d, c->i_trip),
		.q = foc_sanitizef(cmd->i_ref.q, c->i_trip),
	};

	duty = foc_current_step(&c->cur, i_abc, vbus, theta, omega, ref);
	foc_duty_hist_push(&c->hist, duty);
	fill_status(c, st, vbus);
	return duty;
}
