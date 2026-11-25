# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pmsm_foc_test)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# UBSan only reports and continues by default; make every finding abort the test.
if(CONFIG_UBSAN)
  zephyr_compile_options(-fno-sanitize-recover=all)
endif()
