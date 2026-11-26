# SPDX-License-Identifier: MIT

zephyr_include_directories(include)

add_subdirectory_ifdef(CONFIG_PMSM_FOC lib/foc)
add_subdirectory_ifdef(CONFIG_PMSM_PLANT lib/plant)
add_subdirectory(drivers)
