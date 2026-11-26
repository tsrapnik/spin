/*
 * SPDX-License-Identifier: MIT
 *
 * Startup and speed sequencer (thread side), see struct foc_seq_cfg in
 * pmsm/foc.h.
 */

#include <pmsm/foc.h>

/* Non-negative and finite, else 0. */
static float nonneg(float x)
{
	return (x >= 0.0f && x <= FLT_MAX) ? x : 0.0f;
}

void foc_seq_init(struct foc_seq *s, const struct foc_seq_cfg *cfg)
{
	float kt = foc_torque_constant(cfg->psi, cfg->pole_pairs);

	*s = (struct foc_seq){0};
	s->ts = nonneg(cfg->ts);
	foc_pi_init(&s->speed, foc_gains_speed(cfg->j, kt, cfg->bw_speed_hz), s->ts);
	s->pp = (float)cfg->pole_pairs;
	s->i_max = nonneg(cfg->i_max);
	s->i_start = nonneg(cfg->i_start);
	s->t_align = nonneg(cfg->t_align);
	s->accel_start = nonneg(cfg->accel_start);
	s->w_max = nonneg(cfg->w_max);
	/* At most the top speed, so the clamps below stay ordered. */
	s->w_handover = fminf(nonneg(cfg->w_handover), s->w_max);
	s->accel = nonneg(cfg->accel);
	s->dir = 1.0f;
	s->state = FOC_STATE_STOP;
	s->cmd.state = FOC_STATE_STOP;
}

/* Target in the start direction, between w_handover and w_max. */
static float clamp_target(const struct foc_seq *s, float w)
{
	return s->dir * foc_clampf(s->dir * foc_sanitizef(w, FLT_MAX), s->w_handover, s->w_max);
}

void foc_seq_start(struct foc_seq *s, float w)
{
	if (s->state != FOC_STATE_STOP) {
		return;
	}
	s->dir = (w < 0.0f) ? -1.0f : 1.0f;
	s->target = clamp_target(s, w);
	s->run = 1U;
}

void foc_seq_set_speed(struct foc_seq *s, float w)
{
	s->target = clamp_target(s, w);
}

static void enter(struct foc_seq *s, enum foc_state st)
{
	s->state = st;
	s->t = 0.0f;
}

const struct foc_cmd *foc_seq_step(struct foc_seq *s, const struct foc_status *st)
{
	struct foc_cmd *c = &s->cmd;

	if (st->state == FOC_STATE_FAULT && s->state != FOC_STATE_STOP) {
		/* Also catches a fault the controller raised before it saw our command. */
		if (s->state != FOC_STATE_FAULT) {
			s->faults = st->faults;
		}
		enter(s, FOC_STATE_FAULT);
	}
	if (s->run == 0U && s->state != FOC_STATE_STOP) {
		enter(s, FOC_STATE_STOP);
	}

	s->t += s->ts;
	switch (s->state) {
	case FOC_STATE_STOP:
		*c = (struct foc_cmd){.state = FOC_STATE_STOP};
		/* Wait for the controller to be off too (it leaves FAULT on STOP). */
		if (s->run != 0U && st->state == FOC_STATE_STOP) {
			s->faults = 0U;
			s->w_forced = 0.0f;
			enter(s, FOC_STATE_ALIGN);
			*c = (struct foc_cmd){.state = FOC_STATE_ALIGN, .pll_hold = 1U};
		}
		break;
	case FOC_STATE_ALIGN: {
		float x = s->t / fmaxf(s->t_align, FLT_MIN);

		/* Ramp over the first half: no current step to kick the rotor. */
		c->i_ref.d = s->i_start * foc_clampf(2.0f * x, 0.0f, 1.0f);
		/*
		 * Third quarter: turn the vector by 90 degrees in the start
		 * direction. A rotor at the dead point of the first half (180
		 * degrees off) now sees full torque.
		 */
		c->w_forced = (x > 0.5f && x <= 0.75f) ? s->dir * (0.5f * FOC_PI_F) /
								 fmaxf(0.25f * s->t_align, FLT_MIN)
						       : 0.0f;
		if (s->t >= s->t_align) {
			enter(s, FOC_STATE_IF);
			c->state = FOC_STATE_IF;
			c->i_ref.d = s->i_start;
			c->w_forced = 0.0f;
		}
		break;
	}
	case FOC_STATE_IF:
		s->w_forced = s->dir *
			      fminf(s->dir * s->w_forced + s->accel_start * s->ts, s->w_handover);
		c->w_forced = s->pp * s->w_forced;
		c->pll_hold = (s->dir * s->w_forced < 0.5f * s->w_handover) ? 1U : 0U;
		if (s->dir * s->w_forced >= s->w_handover) {
			/* Torque-producing part of the I/f current, in the PLL frame. */
			foc_pi_reset(&s->speed, s->i_start * foc_sincos(st->delta).sin);
			s->w_ref = s->w_forced;
			enter(s, FOC_STATE_RUN);
			*c = (struct foc_cmd){
				.state = FOC_STATE_RUN,
				.i_ref.q = foc_clampf(s->speed.integ, -s->i_max, s->i_max)};
		}
		break;
	case FOC_STATE_RUN: {
		float step = s->accel * s->ts;

		s->w_ref += foc_clampf(s->target - s->w_ref, -step, step);
		c->i_ref.d = 0.0f;
		c->i_ref.q = foc_pi_step(&s->speed, s->w_ref - st->omega / fmaxf(s->pp, 1.0f),
					 -s->i_max, s->i_max);
		break;
	}
	default:
		*c = (struct foc_cmd){.state = FOC_STATE_FAULT};
		break;
	}
	return c;
}
