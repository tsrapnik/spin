/*
 * SPDX-License-Identifier: MIT
 *
 * Motor shell sample: the motor device and its shell commands do all the
 * work (see README.rst).
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

static const struct device *const motor = DEVICE_DT_GET(DT_NODELABEL(motor0));

int main(void)
{
	if (!device_is_ready(motor)) {
		printk("%s: not ready\n", motor->name);
		return 0;
	}
	printk("%s ready: try 'motor start 500', 'motor status', 'motor stop'\n", motor->name);
	if (IS_ENABLED(CONFIG_PMSM_POWER_STAGE_SIM)) {
		printk("simulated motor: 'sim show', 'sim load <mNm>', 'sim vbus <mV>'\n");
	}
	return 0;
}
