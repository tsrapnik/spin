/*
 * SPDX-License-Identifier: MIT
 */

#include <math.h>

#include "plant.h"

#define SQRT3_D   1.73205080756887729353
#define ZERO_DUTY ((struct foc_abc){.a = 0.5f, .b = 0.5f, .c = 0.5f})

/* Time derivative of the state vector x = (id, iq, wm, theta_e). */
struct plant_dx {
	double id, iq, wm, theta_e;
};

int plant_init(struct plant *p, const struct plant_params *prm)
{
	if (!(prm->rs > 0.0f && prm->ls > 0.0f && prm->j > 0.0f && prm->ts > 0.0f &&
	      prm->psi >= 0.0f && prm->b >= 0.0f && prm->pole_pairs > 0U &&
	      prm->duty_delay <= PLANT_DUTY_DELAY_MAX)) {
		return -1;
	}

	*p = (struct plant){
		.rs = (double)prm->rs,
		.ls = (double)prm->ls,
		.psi = (double)prm->psi,
		.j = (double)prm->j,
		.b = (double)prm->b,
		.ts = (double)prm->ts,
		.pp = (double)prm->pole_pairs,
		.duty_delay = prm->duty_delay,
		.substeps = (prm->substeps > 0U) ? prm->substeps : 1U,
		.vbus = (double)prm->vbus,
		.applied = ZERO_DUTY,
	};
	for (unsigned int i = 0; i <= PLANT_DUTY_DELAY_MAX; i++) {
		p->hist[i] = ZERO_DUTY;
	}

	return 0;
}

double plant_torque(const struct plant *p)
{
	return 1.5 * p->pp * p->psi * p->iq;
}

struct plant_sample plant_sample(const struct plant *p)
{
	double s = sin(p->theta_e);
	double c = cos(p->theta_e);
	double ia = p->id * c - p->iq * s; /* alpha */
	double ib = p->id * s + p->iq * c; /* beta */
	double h = -0.5 * ia;
	double k = 0.5 * SQRT3_D * ib;

	return (struct plant_sample){
		.ia = (float)ia,
		.ib = (float)(h + k),
		.ic = (float)(h - k),
		.vbus = (float)p->vbus,
	};
}

static double clamp01(float x)
{
	double d = (double)x;

	return (d > 1.0) ? 1.0 : ((d >= 0.0) ? d : 0.0); /* NaN -> 0 */
}

/* Viscous + Coulomb (smoothed over PLANT_FRIC_W_EPS) + active load torque. */
static double load(const struct plant *p, double wm)
{
	return p->b * wm + p->t_fric * tanh(wm / PLANT_FRIC_W_EPS) + p->t_load;
}

static struct plant_dx deriv(const struct plant *p, double id, double iq, double wm, double th,
			     double valpha, double vbeta)
{
	double s = sin(th);
	double c = cos(th);
	double vd = valpha * c + vbeta * s;
	double vq = vbeta * c - valpha * s;
	double we = p->pp * wm;
	double te = 1.5 * p->pp * p->psi * iq;

	return (struct plant_dx){
		.id = (vd - p->rs * id + we * p->ls * iq) / p->ls,
		.iq = (vq - p->rs * iq - we * (p->ls * id + p->psi)) / p->ls,
		.wm = (te - load(p, wm)) / p->j,
		.theta_e = we,
	};
}

void plant_step(struct plant *p, struct foc_abc duty)
{
	unsigned int n = p->duty_delay;

	p->hist[n] = duty;
	p->applied = p->hist[0];
	for (unsigned int i = 0; i < n; i++) {
		p->hist[i] = p->hist[i + 1U];
	}

	double da = clamp01(p->applied.a);
	double db = clamp01(p->applied.b);
	double dc = clamp01(p->applied.c);
	/* Clarke of the phase voltages; the common mode drops out. */
	double valpha = (2.0 * da - db - dc) / 3.0 * p->vbus;
	double vbeta = (db - dc) / SQRT3_D * p->vbus;
	double h = p->ts / (double)p->substeps;

	p->va = (struct foc_ab){.alpha = (float)valpha, .beta = (float)vbeta};

	for (unsigned int i = 0; i < p->substeps; i++) {
		double x0 = p->id, x1 = p->iq, x2 = p->wm, x3 = p->theta_e;
		struct plant_dx k1 = deriv(p, x0, x1, x2, x3, valpha, vbeta);
		struct plant_dx k2 =
			deriv(p, x0 + 0.5 * h * k1.id, x1 + 0.5 * h * k1.iq, x2 + 0.5 * h * k1.wm,
			      x3 + 0.5 * h * k1.theta_e, valpha, vbeta);
		struct plant_dx k3 =
			deriv(p, x0 + 0.5 * h * k2.id, x1 + 0.5 * h * k2.iq, x2 + 0.5 * h * k2.wm,
			      x3 + 0.5 * h * k2.theta_e, valpha, vbeta);
		struct plant_dx k4 = deriv(p, x0 + h * k3.id, x1 + h * k3.iq, x2 + h * k3.wm,
					   x3 + h * k3.theta_e, valpha, vbeta);

		p->id = x0 + h / 6.0 * (k1.id + 2.0 * k2.id + 2.0 * k3.id + k4.id);
		p->iq = x1 + h / 6.0 * (k1.iq + 2.0 * k2.iq + 2.0 * k3.iq + k4.iq);
		p->wm = x2 + h / 6.0 * (k1.wm + 2.0 * k2.wm + 2.0 * k3.wm + k4.wm);
		p->theta_e =
			x3 +
			h / 6.0 * (k1.theta_e + 2.0 * k2.theta_e + 2.0 * k3.theta_e + k4.theta_e);
	}

	p->theta_e = remainder(p->theta_e, 2.0 * PLANT_PI);
	p->t += p->ts;
}

void plant_coast(struct plant *p)
{
	double h = p->ts / (double)p->substeps;

	p->id = 0.0;
	p->iq = 0.0;
	p->va = (struct foc_ab){0};
	p->applied = ZERO_DUTY;
	for (unsigned int i = 0; i <= PLANT_DUTY_DELAY_MAX; i++) {
		p->hist[i] = ZERO_DUTY;
	}
	for (unsigned int i = 0; i < p->substeps; i++) {
		double w = p->wm;
		double k1 = -load(p, w) / p->j;
		double k2 = -load(p, w + 0.5 * h * k1) / p->j;
		double k3 = -load(p, w + 0.5 * h * k2) / p->j;
		double k4 = -load(p, w + h * k3) / p->j;

		p->wm = w + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
		/* Trapezoidal rule for the angle. */
		p->theta_e += p->pp * 0.5 * (w + p->wm) * h;
	}
	p->theta_e = remainder(p->theta_e, 2.0 * PLANT_PI);
	p->t += p->ts;
}
