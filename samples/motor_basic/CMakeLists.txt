# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_basic)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_PMSM_POWER_STAGE_SIM app PRIVATE src/sim_shell.c)
