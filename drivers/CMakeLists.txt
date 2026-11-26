# SPDX-License-Identifier: MIT

if(CONFIG_PMSM_POWER_STAGE)
  # Scanned for __subsystem, so DEVICE_API(power_stage, ...) gets its linker section.
  zephyr_syscall_header(${CMAKE_CURRENT_SOURCE_DIR}/../include/pmsm/power_stage.h)
  add_subdirectory(power_stage)
endif()
