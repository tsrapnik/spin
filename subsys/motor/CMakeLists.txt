# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources(motor.c)
zephyr_library_sources_ifdef(CONFIG_PMSM_MOTOR_SHELL motor_shell.c)
