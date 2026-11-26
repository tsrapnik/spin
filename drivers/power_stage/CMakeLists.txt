# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_PMSM_POWER_STAGE_SIM power_stage_sim.c)
