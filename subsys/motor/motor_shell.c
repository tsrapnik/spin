/*
 * SPDX-License-Identifier: MIT
 *
 * Shell commands for pmsm,motor devices:
 *
 *   motor start <rpm> [device]
 *   motor stop [device]
 *   motor speed <rpm> [device]
 *   motor status [device]
 *
 * Speeds are mechanical rpm, the sign is the direction. Without a device
 * name the commands act on the first pmsm,motor instance.
 */

#define DT_DRV_COMPAT pmsm_motor

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <pmsm/foc.h>
#include <pmsm/motor.h>

/* rpm <-> rad/s. */
#define RPM (6.28318530718f / 60.0f)

/* Speed limit for the argument; far above any motor, keeps the float conversion exact. */
#define RPM_ARG_MAX 1000000L

#define MOTOR_DEV(n) DEVICE_DT_INST_GET(n),

static const struct device *const motors[] = {DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEV)};

BUILD_ASSERT(ARRAY_SIZE(motors) > 0, "no pmsm,motor instance");

/* The motor named \p name, the first one if NULL; prints the error. */
static const struct device *get_motor(const struct shell *sh, const char *name)
{
	const struct device *dev = NULL;

	if (name == NULL) {
		dev = motors[0];
	} else {
		for (size_t k = 0; k < ARRAY_SIZE(motors); k++) {
			if (strcmp(motors[k]->name, name) == 0) {
				dev = motors[k];
				break;
			}
		}
		if (dev == NULL) {
			shell_error(sh, "%s: no such motor", name);
			return NULL;
		}
	}
	if (!device_is_ready(dev)) {
		shell_error(sh, "%s: not ready", dev->name);
		return NULL;
	}
	return dev;
}

/* Parse a signed rpm argument into rad/s; prints the error. */
static int parse_rpm(const struct shell *sh, const char *arg, float *speed)
{
	int err = 0;
	long rpm = shell_strtol(arg, 10, &err);

	if (err != 0 || rpm < -RPM_ARG_MAX || rpm > RPM_ARG_MAX) {
		shell_error(sh, "%s: not a speed in rpm", arg);
		return -EINVAL;
	}
	*speed = (float)rpm * RPM;
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_motor(sh, argc > 2 ? argv[2] : NULL);
	float speed;

	if (dev == NULL || parse_rpm(sh, argv[1], &speed) != 0) {
		return -EINVAL;
	}

	int ret = motor_start(dev, speed);

	switch (ret) {
	case 0:
		break;
	case -EALREADY:
		shell_error(sh, "%s: already running (use speed)", dev->name);
		break;
	case -EIO:
		shell_error(sh, "%s: in fault (stop clears it)", dev->name);
		break;
	case -EBUSY:
		shell_error(sh, "%s: still stopping, retry", dev->name);
		break;
	case -ERANGE:
		shell_error(sh, "%s: current-sense offset calibration failed", dev->name);
		break;
	default:
		shell_error(sh, "%s: start failed (%d)", dev->name, ret);
		break;
	}
	return ret;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_motor(sh, argc > 1 ? argv[1] : NULL);

	if (dev == NULL) {
		return -EINVAL;
	}
	return motor_stop(dev);
}

static int cmd_speed(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_motor(sh, argc > 2 ? argv[2] : NULL);
	float speed;

	if (dev == NULL || parse_rpm(sh, argv[1], &speed) != 0) {
		return -EINVAL;
	}

	int ret = motor_set_speed(dev, speed);

	if (ret != 0) {
		shell_error(sh, "%s: set speed failed (%d)", dev->name, ret);
	}
	return ret;
}

static const char *state_name(enum foc_state s)
{
	switch (s) {
	case FOC_STATE_STOP:
		return "stop";
	case FOC_STATE_ALIGN:
		return "align";
	case FOC_STATE_IF:
		return "i/f";
	case FOC_STATE_RUN:
		return "run";
	case FOC_STATE_FAULT:
		return "fault";
	default:
		return "?";
	}
}

/*
 * \p x with \p decimals (0..3) decimals into \p buf, without float printf
 * support; "nan"/"inf" when out of range (i and vbus are raw measurements).
 */
static const char *fmt(char *buf, size_t size, float x, unsigned int decimals)
{
	static const int32_t scale[] = {1, 10, 100, 1000};
	float y = x * (float)scale[decimals];

	if (isnan(x)) {
		return "nan";
	}
	if (!(fabsf(y) < 2e9f)) {
		return x > 0.0f ? "inf" : "-inf";
	}

	int32_t v = (int32_t)roundf(y);
	uint32_t mag = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
	const char *sign = v < 0 ? "-" : "";

	if (decimals == 0U) {
		snprintk(buf, size, "%s%u", sign, mag);
	} else {
		snprintk(buf, size, "%s%u.%0*u", sign, mag / (uint32_t)scale[decimals],
			 (int)decimals, mag % (uint32_t)scale[decimals]);
	}
	return buf;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_motor(sh, argc > 1 ? argv[1] : NULL);
	struct motor_status st;
	char b[3][32];

	if (dev == NULL) {
		return -EINVAL;
	}
	(void)motor_get_status(dev, &st);

	shell_print(sh, "%s: %s", dev->name, state_name(st.state));
	if (st.state == FOC_STATE_FAULT) {
		shell_print(sh, "  faults  0x%x%s%s%s%s", st.faults,
			    (st.faults & FOC_FAULT_OC) ? " over-current" : "",
			    (st.faults & FOC_FAULT_OV) ? " over-voltage" : "",
			    (st.faults & FOC_FAULT_UV) ? " under-voltage" : "",
			    (st.faults & FOC_FAULT_EXT) ? " break" : "");
	}
	shell_print(sh, "  speed   %s rpm (ref %s, target %s)",
		    fmt(b[0], sizeof(b[0]), st.speed / RPM, 1),
		    fmt(b[1], sizeof(b[1]), st.speed_ref / RPM, 1),
		    fmt(b[2], sizeof(b[2]), st.speed_target / RPM, 1));
	shell_print(sh, "  i_dq    %s %s A", fmt(b[0], sizeof(b[0]), st.i.d, 3),
		    fmt(b[1], sizeof(b[1]), st.i.q, 3));
	shell_print(sh, "  v_dq    %s %s V", fmt(b[0], sizeof(b[0]), st.v.d, 2),
		    fmt(b[1], sizeof(b[1]), st.v.q, 2));
	shell_print(sh, "  vbus    %s V, emf %s V", fmt(b[0], sizeof(b[0]), st.vbus, 2),
		    fmt(b[1], sizeof(b[1]), st.emf, 2));
	shell_print(sh, "  late    %u", st.late);
	return 0;
}

/* Device names for completion; only motors. */
static void motor_name_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = idx < ARRAY_SIZE(motors) ? motors[idx]->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_motor_name, motor_name_get);

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD_ARG(start, NULL,
		      "Start towards a speed: start <rpm> [device]", cmd_start, 2, 1),
	SHELL_CMD_ARG(stop, &dsub_motor_name,
		      "Stop (coast) and clear a fault: stop [device]", cmd_stop, 1, 1),
	SHELL_CMD_ARG(speed, NULL,
		      "Change the target speed: speed <rpm> [device]", cmd_speed, 2, 1),
	SHELL_CMD_ARG(status, &dsub_motor_name,
		      "Show state, speed, currents: status [device]", cmd_status, 1, 1),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(motor, &sub_motor, "PMSM motor control (speeds in rpm)", NULL);
