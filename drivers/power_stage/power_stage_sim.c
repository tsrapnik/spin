/*
 * SPDX-License-Identifier: MIT
 *
 * Simulated power stage: the plant model (lib/plant) behind the power-stage
 * driver API. See include/pmsm/power_stage_sim.h.
 */

#define DT_DRV_COMPAT pmsm_sim_power_stage

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <pmsm/plant.h>
#include <pmsm/power_stage.h>
#include <pmsm/power_stage_sim.h>

LOG_MODULE_REGISTER(pmsm_power_stage_sim, CONFIG_PMSM_POWER_STAGE_LOG_LEVEL);

/* Largest sense offset calibration accepts, A. */
#define OFFSET_MAX 1.0f

/* Bits of ps_sim_data.flags. */
enum {
	FLAG_ENABLED,     /* Outputs on. */
	FLAG_BRK_ACTIVE,  /* Break input held active. */
	FLAG_BRK_PENDING, /* Break fired, not yet reported in a sample. */
};

struct ps_sim_config {
	struct plant_params plant;
	struct power_stage_info info;
	uint32_t pwm_hz;
	uint32_t tick_us; /* Timer period; 0: manual stepping. */
};

struct ps_sim_data {
	const struct device *dev;
	struct k_spinlock lock; /* Guards the plant, the callback and the offsets. */
	struct k_timer timer;
	struct plant p;
	power_stage_cb_t cb;
	void *user;
	struct foc_abc duty;    /* Latest set_duty(), applied by plant_step(). */
	struct foc_abc off_inj; /* Injected sense offset, A. */
	struct foc_abc off_cal; /* Calibrated offset, removed from samples. */
	atomic_t flags;
	uint64_t cycles;
	int64_t t0; /* Kernel uptime at the timer start, ticks. */
};

#define ZERO_DUTY ((struct foc_abc){.a = 0.5f, .b = 0.5f, .c = 0.5f})

/* Clamp to [1 - max_duty, max_duty]; NaN -> 0.5. */
static float clamp_duty(float x, float max_duty)
{
	float lo = 1.0f - max_duty;

	if (x >= lo) {
		return (x <= max_duty) ? x : max_duty;
	}
	return (x < lo) ? lo : 0.5f;
}

/* One PWM period. Called with data->lock held. */
static void run_cycle(const struct device *dev, struct ps_sim_data *data)
{
	struct plant_sample ps = plant_sample(&data->p);
	struct power_stage_sample s = {
		.ia = ps.ia + data->off_inj.a - data->off_cal.a,
		.ib = ps.ib + data->off_inj.b - data->off_cal.b,
		.ic = ps.ic + data->off_inj.c - data->off_cal.c,
		.vbus = ps.vbus,
		.flags = atomic_test_and_clear_bit(&data->flags, FLAG_BRK_PENDING)
				 ? POWER_STAGE_FLAG_BREAK
				 : 0U,
	};

	if (data->cb != NULL) {
		data->cb(dev, &s, data->user);
	}

	if (atomic_test_bit(&data->flags, FLAG_ENABLED)) {
		plant_step(&data->p, data->duty);
	} else {
		plant_coast(&data->p);
	}
	data->cycles++;
}

static void run_cycles(const struct device *dev, unsigned int n)
{
	struct ps_sim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	for (unsigned int i = 0; i < n; i++) {
		run_cycle(dev, data);
	}
	k_spin_unlock(&data->lock, key);
}

/*
 * Run the cycles due by now, so simulated time follows the kernel clock
 * however the timer period rounds to ticks. Only the timer runs cycles in
 * this mode, so data->cycles is read without the lock.
 */
static void timer_expiry(struct k_timer *timer)
{
	struct ps_sim_data *data = CONTAINER_OF(timer, struct ps_sim_data, timer);
	const struct ps_sim_config *cfg = data->dev->config;
	uint64_t ticks = (uint64_t)(k_uptime_ticks() - data->t0);
	uint64_t due = ticks * cfg->pwm_hz / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	if (due > data->cycles) {
		run_cycles(data->dev, (unsigned int)MIN(due - data->cycles, UINT_MAX));
	}
}

static int ps_sim_calibrate_offsets(const struct device *dev)
{
	struct ps_sim_data *data = dev->data;
	int ret = 0;

	if (atomic_test_bit(&data->flags, FLAG_ENABLED)) {
		return -EBUSY;
	}

	/*
	 * The raw sample with the outputs off, as the hardware would average
	 * it. The plant is deterministic, so one sample is the average.
	 */
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	struct plant_sample ps = plant_sample(&data->p);
	struct foc_abc off = {
		.a = ps.ia + data->off_inj.a,
		.b = ps.ib + data->off_inj.b,
		.c = ps.ic + data->off_inj.c,
	};

	if (fabsf(off.a) <= OFFSET_MAX && fabsf(off.b) <= OFFSET_MAX &&
	    fabsf(off.c) <= OFFSET_MAX) {
		data->off_cal = off;
	} else {
		ret = -ERANGE;
	}
	k_spin_unlock(&data->lock, key);

	if (ret != 0) {
		LOG_ERR("%s: sense offset out of range", dev->name);
	}
	return ret;
}

static int ps_sim_enable(const struct device *dev)
{
	struct ps_sim_data *data = dev->data;

	if (atomic_test_bit(&data->flags, FLAG_BRK_ACTIVE)) {
		return -EIO;
	}
	atomic_set_bit(&data->flags, FLAG_ENABLED);
	return 0;
}

static void ps_sim_disable(const struct device *dev)
{
	struct ps_sim_data *data = dev->data;

	atomic_clear_bit(&data->flags, FLAG_ENABLED);
	data->duty = ZERO_DUTY;
}

static void ps_sim_set_duty(const struct device *dev, float da, float db, float dc)
{
	const struct ps_sim_config *cfg = dev->config;
	struct ps_sim_data *data = dev->data;
	float m = cfg->info.max_duty;

	data->duty = (struct foc_abc){
		.a = clamp_duty(da, m),
		.b = clamp_duty(db, m),
		.c = clamp_duty(dc, m),
	};
}

static int ps_sim_register_cb(const struct device *dev, power_stage_cb_t cb, void *user)
{
	struct ps_sim_data *data = dev->data;

	if (atomic_test_bit(&data->flags, FLAG_ENABLED)) {
		return -EBUSY;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->cb = cb;
	data->user = user;
	k_spin_unlock(&data->lock, key);
	return 0;
}

static const struct power_stage_info *ps_sim_get_info(const struct device *dev)
{
	const struct ps_sim_config *cfg = dev->config;

	return &cfg->info;
}

int power_stage_sim_run(const struct device *dev, unsigned int n)
{
	const struct ps_sim_config *cfg = dev->config;

	if (cfg->tick_us != 0U) {
		return -EBUSY;
	}
	run_cycles(dev, n);
	return 0;
}

struct plant *power_stage_sim_lock(const struct device *dev, k_spinlock_key_t *key)
{
	struct ps_sim_data *data = dev->data;

	*key = k_spin_lock(&data->lock);
	return &data->p;
}

void power_stage_sim_unlock(const struct device *dev, k_spinlock_key_t key)
{
	struct ps_sim_data *data = dev->data;

	k_spin_unlock(&data->lock, key);
}

void power_stage_sim_set_offset(const struct device *dev, float a, float b, float c)
{
	struct ps_sim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->off_inj = (struct foc_abc){.a = a, .b = b, .c = c};
	k_spin_unlock(&data->lock, key);
}

void power_stage_sim_brk(const struct device *dev, bool active)
{
	struct ps_sim_data *data = dev->data;

	if (active) {
		atomic_set_bit(&data->flags, FLAG_BRK_ACTIVE);
		atomic_clear_bit(&data->flags, FLAG_ENABLED);
		data->duty = ZERO_DUTY;
		atomic_set_bit(&data->flags, FLAG_BRK_PENDING);
	} else {
		atomic_clear_bit(&data->flags, FLAG_BRK_ACTIVE);
	}
}

uint64_t power_stage_sim_cycles(const struct device *dev)
{
	struct ps_sim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint64_t n = data->cycles;

	k_spin_unlock(&data->lock, key);
	return n;
}

static int ps_sim_init(const struct device *dev)
{
	const struct ps_sim_config *cfg = dev->config;
	struct ps_sim_data *data = dev->data;

	if (plant_init(&data->p, &cfg->plant) != 0) {
		LOG_ERR("%s: invalid plant parameters", dev->name);
		return -EINVAL;
	}
	data->dev = dev;
	data->duty = ZERO_DUTY;

	if (cfg->tick_us != 0U) {
		data->t0 = k_uptime_ticks();
		k_timer_init(&data->timer, timer_expiry, NULL);
		k_timer_start(&data->timer, K_USEC(cfg->tick_us), K_USEC(cfg->tick_us));
	}
	return 0;
}

static DEVICE_API(power_stage, ps_sim_api) = {
	.calibrate_offsets = ps_sim_calibrate_offsets,
	.enable = ps_sim_enable,
	.disable = ps_sim_disable,
	.set_duty = ps_sim_set_duty,
	.register_cb = ps_sim_register_cb,
	.get_info = ps_sim_get_info,
};

#define PS_SIM_DEFINE(n)                                                                           \
	BUILD_ASSERT(DT_INST_PROP(n, pwm_frequency_hz) > 0, "pwm-frequency-hz must be > 0");       \
	BUILD_ASSERT(DT_INST_PROP(n, max_duty_permille) > 500 &&                                   \
			     DT_INST_PROP(n, max_duty_permille) <= 1000,                           \
		     "max-duty-permille must be in 501..1000");                                    \
                                                                                                   \
	static const struct ps_sim_config ps_sim_config_##n = {                                    \
		.plant =                                                                           \
			{                                                                          \
				.rs = DT_INST_PROP(n, resistance_micro_ohms) * 1e-6f,              \
				.ls = DT_INST_PROP(n, inductance_nano_henries) * 1e-9f,            \
				.psi = DT_INST_PROP(n, flux_linkage_micro_webers) * 1e-6f,         \
				.pole_pairs = DT_INST_PROP(n, pole_pairs),                         \
				.j = DT_INST_PROP(n, inertia_nano_kg_m2) * 1e-9f,                  \
				.b = DT_INST_PROP(n, viscous_friction_nano_nm_s) * 1e-9f,          \
				.vbus = DT_INST_PROP(n, vbus_millivolts) * 1e-3f,                  \
				.ts = 1.0f / DT_INST_PROP(n, pwm_frequency_hz),                    \
				.duty_delay = DT_INST_PROP(n, duty_delay),                         \
				.substeps = DT_INST_PROP(n, substeps),                             \
			},                                                                         \
		.info =                                                                            \
			{                                                                          \
				.ts = 1.0f / DT_INST_PROP(n, pwm_frequency_hz),                    \
				.dead_time = DT_INST_PROP(n, dead_time_ns) * 1e-9f,                \
				.max_duty = DT_INST_PROP(n, max_duty_permille) * 1e-3f,            \
				.duty_delay = DT_INST_PROP(n, duty_delay),                         \
			},                                                                         \
		.pwm_hz = DT_INST_PROP(n, pwm_frequency_hz),                                       \
		.tick_us = DT_INST_PROP(n, tick_period_us),                                        \
	};                                                                                         \
	static struct ps_sim_data ps_sim_data_##n;                                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ps_sim_init, NULL, &ps_sim_data_##n, &ps_sim_config_##n,          \
			      POST_KERNEL, CONFIG_PMSM_POWER_STAGE_INIT_PRIORITY, &ps_sim_api);

DT_INST_FOREACH_STATUS_OKAY(PS_SIM_DEFINE)
