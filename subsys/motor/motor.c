/*
 * SPDX-License-Identifier: MIT
 *
 * Motor device: foc_cycle in the power stage's per-cycle callback,
 * foc_seq in a motor thread, cmd/status exchanged via double buffers. See
 * include/pmsm/motor.h.
 */

#define DT_DRV_COMPAT pmsm_motor

#include <errno.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <pmsm/foc.h>
#include <pmsm/motor.h>
#include <pmsm/power_stage.h>

LOG_MODULE_REGISTER(pmsm_motor, CONFIG_PMSM_MOTOR_LOG_LEVEL);

BUILD_ASSERT(CONFIG_PMSM_MOTOR_INIT_PRIORITY > CONFIG_PMSM_POWER_STAGE_INIT_PRIORITY,
	     "motors must initialise after their power stages");
BUILD_ASSERT(((uint64_t)CONFIG_PMSM_MOTOR_PERIOD_US * CONFIG_SYS_CLOCK_TICKS_PER_SEC) % 1000000U ==
		     0U,
	     "CONFIG_PMSM_MOTOR_PERIOD_US must be a whole number of kernel ticks "
	     "(raise CONFIG_SYS_CLOCK_TICKS_PER_SEC)");

#define SEQ_TS (CONFIG_PMSM_MOTOR_PERIOD_US * 1e-6f)

/* rpm -> rad/s. */
#define RPM (6.28318530718f / 60.0f)

/*
 * Single-writer double buffers. The writer fills b[(n + 1) & 1], then
 * increments n (atomic_inc() wraps); the reader uses b[n & 1]. Zephyr's
 * atomics are sequentially consistent, so they also order the buffer
 * accesses.
 *
 * cmd: written by the thread, read in the interrupt. The interrupt
 * cannot be preempted by the thread, so its read is never torn.
 *
 * status: written in the interrupt, read by the thread. The interrupt can
 * run during the thread's copy; one write goes into the other buffer, a
 * second one reuses the buffer being copied, so the thread retries when n
 * advanced by two or more.
 */
struct cmd_buf {
	struct foc_cmd b[2];
	atomic_t n;
};

struct status_buf {
	struct foc_status b[2];
	atomic_t n;
};

struct motor_config {
	const struct device *ps;
	struct foc_cycle_cfg cycle; /* Without the power-stage timing (ts, max_duty, delay). */
	struct foc_seq_cfg seq;
	k_thread_stack_t *stack;
	size_t stack_size;
};

struct motor_data {
	/* Interrupt side. */
	struct foc_cycle cycle;
	bool pwm_on;

	struct cmd_buf cmd;
	struct status_buf status;

	/* Thread side, guarded by lock (the API callers and the thread). */
	struct k_mutex lock;
	struct foc_seq seq;
	struct foc_status st; /* Latest status read by the thread. */
	uint32_t late;

	struct k_timer timer;
	struct k_thread thread;
};

static void cmd_publish(struct cmd_buf *x, const struct foc_cmd *c)
{
	atomic_val_t n = atomic_get(&x->n);

	x->b[(n + 1) & 1] = *c;
	atomic_inc(&x->n);
}

static void status_read(struct status_buf *x, struct foc_status *st)
{
	atomic_val_t n1, n2;

	do {
		n1 = atomic_get(&x->n);
		*st = x->b[n1 & 1];
		n2 = atomic_get(&x->n);
	} while ((unsigned long)n2 - (unsigned long)n1 >= 2U);
}

static bool is_on(enum foc_state s)
{
	return s == FOC_STATE_ALIGN || s == FOC_STATE_IF || s == FOC_STATE_RUN;
}

/* Per PWM cycle, in interrupt context: no kernel calls. */
static void on_sample(const struct device *ps, const struct power_stage_sample *s, void *user)
{
	struct motor_data *d = user;
	struct status_buf *sb = &d->status;
	atomic_val_t n = atomic_get(&sb->n);
	struct foc_status *st = &sb->b[(n + 1) & 1];
	const struct foc_cmd *cmd = &d->cmd.b[atomic_get(&d->cmd.n) & 1];
	struct foc_abc i = {.a = s->ia, .b = s->ib, .c = s->ic};

	if ((s->flags & POWER_STAGE_FLAG_BREAK) != 0U) {
		foc_cycle_trip(&d->cycle, FOC_FAULT_EXT);
	}

	struct foc_abc duty = foc_cycle_step(&d->cycle, cmd, i, s->vbus, st);

	atomic_inc(&sb->n);

	if (!is_on(st->state)) {
		/* Every cycle: also resets the duty after a hardware break. */
		power_stage_disable(ps);
		d->pwm_on = false;
		return;
	}
	power_stage_set_duty(ps, duty.a, duty.b, duty.c);
	if (!d->pwm_on) {
		d->pwm_on = power_stage_enable(ps) == 0;
		if (!d->pwm_on) {
			/* Break input still active: fault instead of winding up. */
			foc_cycle_trip(&d->cycle, FOC_FAULT_EXT);
		}
	}
}

static void motor_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct motor_data *d = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		uint32_t n = k_timer_status_sync(&d->timer);

		k_mutex_lock(&d->lock, K_FOREVER);
		status_read(&d->status, &d->st);

		enum foc_state prev = d->seq.state;
		const struct foc_cmd *cmd = NULL;

		/* Late: step once per elapsed period so the ramps keep their rate. */
		for (uint32_t k = 0; k < MAX(n, 1U); k++) {
			cmd = foc_seq_step(&d->seq, &d->st);
		}
		if (n > 1U) {
			d->late += n - 1U;
		}
		cmd_publish(&d->cmd, cmd);

		bool fault = d->seq.state == FOC_STATE_FAULT && prev != FOC_STATE_FAULT;
		unsigned int faults = d->seq.faults;

		k_mutex_unlock(&d->lock);

		if (fault) {
			LOG_ERR("%s: fault 0x%x%s%s%s%s", dev->name, faults,
				(faults & FOC_FAULT_OC) ? " over-current" : "",
				(faults & FOC_FAULT_OV) ? " over-voltage" : "",
				(faults & FOC_FAULT_UV) ? " under-voltage" : "",
				(faults & FOC_FAULT_EXT) ? " break" : "");
		}
	}
}

int motor_start(const struct device *dev, float speed)
{
	const struct motor_config *cfg = dev->config;
	struct motor_data *d = dev->data;
	struct foc_status st;
	int ret = 0;

	if (!isfinite(speed)) {
		return -EINVAL;
	}

	k_mutex_lock(&d->lock, K_FOREVER);
	status_read(&d->status, &st);
	if (d->seq.state == FOC_STATE_FAULT) {
		ret = -EIO;
	} else if (d->seq.run != 0U) {
		ret = -EALREADY;
	} else if (d->seq.state != FOC_STATE_STOP || st.state != FOC_STATE_STOP) {
		ret = -EBUSY;
	} else {
		ret = power_stage_calibrate_offsets(cfg->ps);
		if (ret == 0) {
			foc_seq_start(&d->seq, speed);
		} else {
			LOG_ERR("%s: offset calibration failed (%d)", dev->name, ret);
		}
	}
	k_mutex_unlock(&d->lock);
	return ret;
}

int motor_stop(const struct device *dev)
{
	struct motor_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	foc_seq_stop(&d->seq);
	k_mutex_unlock(&d->lock);
	return 0;
}

int motor_set_speed(const struct device *dev, float speed)
{
	struct motor_data *d = dev->data;

	if (!isfinite(speed)) {
		return -EINVAL;
	}
	k_mutex_lock(&d->lock, K_FOREVER);
	foc_seq_set_speed(&d->seq, speed);
	k_mutex_unlock(&d->lock);
	return 0;
}

int motor_get_status(const struct device *dev, struct motor_status *st)
{
	struct motor_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	*st = (struct motor_status){
		.state = d->seq.state,
		.faults = (d->seq.state == FOC_STATE_FAULT) ? d->seq.faults : 0U,
		.speed = is_on(d->st.state) ? d->st.omega / d->seq.pp : 0.0f,
		.speed_ref = d->seq.w_ref,
		.speed_target = d->seq.target,
		.i = d->st.i,
		.v = d->st.v,
		.vbus = d->st.vbus,
		.emf = d->st.e_mag,
		.late = d->late,
	};
	k_mutex_unlock(&d->lock);
	return 0;
}

static int motor_init(const struct device *dev)
{
	const struct motor_config *cfg = dev->config;
	struct motor_data *d = dev->data;

	if (!device_is_ready(cfg->ps)) {
		LOG_ERR("%s: power stage %s not ready", dev->name, cfg->ps->name);
		return -ENODEV;
	}

	const struct power_stage_info *info = power_stage_get_info(cfg->ps);
	struct foc_cycle_cfg cc = cfg->cycle;
	struct foc_seq_cfg sc = cfg->seq;

	cc.ts = info->ts;
	cc.max_duty = info->max_duty;
	cc.t_adv = FOC_ADVANCE_PERIODS * info->ts;
	cc.delay = info->duty_delay;
	sc.ts = SEQ_TS;

	foc_cycle_init(&d->cycle, &cc);
	foc_seq_init(&d->seq, &sc);
	d->cmd.b[0] = d->seq.cmd;
	atomic_set(&d->cmd.n, 0);
	d->status.b[0].state = FOC_STATE_STOP;
	atomic_set(&d->status.n, 0);
	d->st = d->status.b[0];
	k_mutex_init(&d->lock);

	power_stage_disable(cfg->ps);
	int ret = power_stage_register_cb(cfg->ps, on_sample, d);

	if (ret != 0) {
		LOG_ERR("%s: cannot register with %s (%d)", dev->name, cfg->ps->name, ret);
		return ret;
	}

	k_timer_init(&d->timer, NULL, NULL);
	k_timer_start(&d->timer, K_USEC(CONFIG_PMSM_MOTOR_PERIOD_US),
		      K_USEC(CONFIG_PMSM_MOTOR_PERIOD_US));

	k_tid_t tid = k_thread_create(&d->thread, cfg->stack, cfg->stack_size, motor_thread,
				      (void *)dev, NULL, NULL, CONFIG_PMSM_MOTOR_THREAD_PRIORITY,
				      K_FP_REGS, K_NO_WAIT);

	k_thread_name_set(tid, dev->name);
	return 0;
}

/* PLL and forced-speed limit: twice the top speed, electrical rad/s. */
#define W_MAX_ELEC(n) (2.0f * DT_INST_PROP(n, pole_pairs) * DT_INST_PROP(n, max_speed_rpm) * RPM)

#define MOTOR_DEFINE(n)                                                                            \
	BUILD_ASSERT(DT_INST_PROP(n, pole_pairs) > 0, "pole-pairs must be > 0");                   \
	BUILD_ASSERT(DT_INST_PROP(n, trip_current_milliamps) >                                     \
				     DT_INST_PROP(n, max_current_milliamps) &&                     \
			     DT_INST_PROP(n, trip_current_milliamps) >                             \
				     DT_INST_PROP(n, start_current_milliamps),                     \
		     "trip-current must exceed max-current and start-current");                    \
	BUILD_ASSERT(DT_INST_PROP(n, vbus_max_millivolts) > DT_INST_PROP(n, vbus_min_millivolts),  \
		     "vbus-max must exceed vbus-min");                                             \
	BUILD_ASSERT(DT_INST_PROP(n, handover_speed_rpm) < DT_INST_PROP(n, max_speed_rpm),         \
		     "handover-speed must be below max-speed");                                    \
                                                                                                   \
	K_KERNEL_STACK_DEFINE(motor_stack_##n, CONFIG_PMSM_MOTOR_THREAD_STACK_SIZE);               \
                                                                                                   \
	static const struct motor_config motor_config_##n = {                                      \
		.ps = DEVICE_DT_GET(DT_INST_PHANDLE(n, power_stage)),                              \
		.cycle =                                                                           \
			{                                                                          \
				.rs = DT_INST_PROP(n, resistance_micro_ohms) * 1e-6f,              \
				.ls = DT_INST_PROP(n, inductance_nano_henries) * 1e-9f,            \
				.bw_current_hz = DT_INST_PROP(n, current_bandwidth_hz),            \
				.bw_obs_hz = DT_INST_PROP(n, observer_bandwidth_hz),               \
				.bw_pll_hz = DT_INST_PROP(n, pll_bandwidth_hz),                    \
				.w_max = W_MAX_ELEC(n),                                            \
				.e_min = DT_INST_PROP(n, emf_min_millivolts) * 1e-3f,              \
				.i_trip = DT_INST_PROP(n, trip_current_milliamps) * 1e-3f,         \
				.vbus_min = DT_INST_PROP(n, vbus_min_millivolts) * 1e-3f,          \
				.vbus_max = DT_INST_PROP(n, vbus_max_millivolts) * 1e-3f,          \
			},                                                                         \
		.seq =                                                                             \
			{                                                                          \
				.pole_pairs = DT_INST_PROP(n, pole_pairs),                         \
				.j = DT_INST_PROP(n, inertia_nano_kg_m2) * 1e-9f,                  \
				.psi = DT_INST_PROP(n, flux_linkage_micro_webers) * 1e-6f,         \
				.bw_speed_hz = DT_INST_PROP(n, speed_bandwidth_hz),                \
				.i_max = DT_INST_PROP(n, max_current_milliamps) * 1e-3f,           \
				.i_start = DT_INST_PROP(n, start_current_milliamps) * 1e-3f,       \
				.t_align = DT_INST_PROP(n, align_time_ms) * 1e-3f,                 \
				.accel_start =                                                     \
					DT_INST_PROP(n, start_acceleration_rpm_per_s) * RPM,       \
				.w_handover = DT_INST_PROP(n, handover_speed_rpm) * RPM,           \
				.accel = DT_INST_PROP(n, acceleration_rpm_per_s) * RPM,            \
				.w_max = DT_INST_PROP(n, max_speed_rpm) * RPM,                     \
			},                                                                         \
		.stack = motor_stack_##n,                                                          \
		.stack_size = K_KERNEL_STACK_SIZEOF(motor_stack_##n),                              \
	};                                                                                         \
	static struct motor_data motor_data_##n;                                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, motor_init, NULL, &motor_data_##n, &motor_config_##n,             \
			      POST_KERNEL, CONFIG_PMSM_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
