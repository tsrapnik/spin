/*
 * SPDX-License-Identifier: MIT
 *
 * "sim" shell command: look at and load the simulated motor behind
 * pmsm,sim-power-stage.
 *
 *   sim show               true speed, current, torque and loads
 *   sim load <mNm>         active load torque (positive opposes positive speed)
 *   sim friction <mNm>     Coulomb friction (always opposes the motion)
 *   sim vbus <mV>          bus voltage (try the under/over-voltage trips)
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <pmsm/plant.h>
#include <pmsm/power_stage_sim.h>

#define RPM (6.283185307179586 / 60.0)

/* Argument limits: far beyond the simulated motor, but keep the plant sane. */
#define TORQUE_MAX_MNM 10000L
#define VBUS_MAX_MV    100000L

static const struct device *const ps = DEVICE_DT_GET(DT_NODELABEL(ps));

static int parse(const struct shell *sh, const char *arg, long min, long max, long *v)
{
	int err = 0;

	*v = shell_strtol(arg, 10, &err);
	if (err != 0 || *v < min || *v > max) {
		shell_error(sh, "%s: expected an integer in [%ld, %ld]", arg, min, max);
		return -EINVAL;
	}
	return 0;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	k_spinlock_key_t key;
	struct plant p = *power_stage_sim_lock(ps, &key);

	power_stage_sim_unlock(ps, key);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	double te = 1.5 * p.pp * p.psi * p.iq;

	shell_print(sh, "speed     %.1f rpm", p.wm / RPM);
	shell_print(sh, "i_dq      %.3f %.3f A (rotor frame)", p.id, p.iq);
	shell_print(sh, "torque    %.1f mNm", te * 1e3);
	shell_print(sh, "load      %.1f mNm, friction %.1f mNm, viscous %.1f mNm", p.t_load * 1e3,
		    p.t_fric * 1e3, p.b * p.wm * 1e3);
	shell_print(sh, "vbus      %.2f V", p.vbus);
	shell_print(sh, "time      %.3f s", p.t);
	return 0;
}

enum input {
	LOAD,
	FRICTION,
	VBUS,
};

/* Set one plant input from \p arg, in thousandths (mNm, mV). */
static int set_input(const struct shell *sh, const char *arg, enum input in)
{
	long v;
	int ret = in == VBUS ? parse(sh, arg, 0, VBUS_MAX_MV, &v)
			     : parse(sh, arg, in == LOAD ? -TORQUE_MAX_MNM : 0, TORQUE_MAX_MNM, &v);

	if (ret != 0) {
		return ret;
	}

	k_spinlock_key_t key;
	struct plant *p = power_stage_sim_lock(ps, &key);
	double x = (double)v * 1e-3;

	switch (in) {
	case LOAD:
		p->t_load = x;
		break;
	case FRICTION:
		p->t_fric = x;
		break;
	case VBUS:
		p->vbus = x;
		break;
	}
	power_stage_sim_unlock(ps, key);
	return 0;
}

static int cmd_load(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return set_input(sh, argv[1], LOAD);
}

static int cmd_friction(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return set_input(sh, argv[1], FRICTION);
}

static int cmd_vbus(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return set_input(sh, argv[1], VBUS);
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sim,
	SHELL_CMD_ARG(show, NULL, "True speed, current, torque and loads", cmd_show, 1, 0),
	SHELL_CMD_ARG(load, NULL,
		      "Load torque: load <mNm> (positive opposes positive speed)", cmd_load, 2, 0),
	SHELL_CMD_ARG(friction, NULL,
		      "Coulomb friction: friction <mNm> (opposes the motion)", cmd_friction, 2, 0),
	SHELL_CMD_ARG(vbus, NULL, "Bus voltage: vbus <mV>", cmd_vbus, 2, 0),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(sim, &sub_sim, "Simulated motor (sim power stage)", NULL);
