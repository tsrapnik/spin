# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Sensorless PMSM field-oriented control as an out-of-tree Zephyr module. Target: nucleo_c542rc +
X-NUCLEO-IHM16M1 + DFRobot FIT1036; CI runs on `native_sim`.

## Living docs — update after every step

- `docs/PLAN.md`: goal, **decisions table** (binding: read it before designing anything), milestones
  with status markers, and an **insights log** at the end. Mark progress and append insights as you go.
- `docs/FEATURES.md`: backlog. Anything not needed for v1 goes here, not into code.
- `docs/HARDWARE.md`: BOM, board mods, pin map, motor parameters (many marked ⚠ unverified/estimate).
- `README.md` + `docs/img/*.svg`: physics and control strategy, mostly diagrams. Update when an
  algorithm, frame convention or timing changes; drop *(planned)* markers as parts land.

## Workspace and commands

T2 west workspace: topdir `/workdir` (holds `.west/`, `zephyr/`, `modules/hal/{cmsis_6,stm32}`),
this repo at `/workdir/spin` is the manifest repo (`west.yml` pins Zephyr v4.5.0 by SHA). Run west
from `/workdir`. The devcontainer (`.devcontainer/`) sets this up; `post-create.sh` runs
`west init -l spin && west update`.

```sh
# before committing: clang-format, checkpatch, GCC analyzer, all tests (incl. ASan/UBSan)
LINT_OUT=<outdir> spin/scripts/lint.sh          # --fix reformats; step names run a subset

# all tests: native_sim runs them, native_sim/native/64 runs them under ASan + UBSan,
# nucleo_c542rc is build-only (catches ARM warnings)
west twister -T spin/tests -p native_sim -p native_sim/native/64 -p nucleo_c542rc -O <outdir>

# one scenario
west twister -T spin/tests/lib/foc -p native_sim -s pmsm.lib.foc

# one ztest case, fastest loop
west build -p -b native_sim spin/tests/lib/foc -d <builddir>
<builddir>/zephyr/zephyr.exe -test="foc_pi::test_anti_windup"   # suite::test, '*' wildcards ok

# target build
west build -b nucleo_c542rc <app>
```

Keep build/twister output out of the repo (use a scratch dir). Zephyr compiles with
`-Wdouble-promotion -Werror`: use `f` suffixes, cast explicitly to `double` in test references.

To check codegen, disassemble the twister ARM build:
`/opt/toolchains/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d <build>/zephyr/zephyr.elf`.

## Architecture

Layers (see the module layout and power-stage API sketch in `docs/PLAN.md`):

1. **Math core** — `include/pmsm/foc.h` + `lib/foc/` (`CONFIG_PMSM_FOC`). Plain C99, float32,
   **no Zephyr includes**, so it is portable and testable as-is. Planned additions: observer, PLL,
   per-cycle step (`foc.c`).
2. **Motor subsystem** (planned, `subsys/motor/`) — `pmsm,motor` device: FOC in the ADC ISR (no kernel
   calls), speed loop/state machine in a 1 kHz motor thread, single-writer `cmd`/`status` structs
   exchanged via double buffer + atomic index.
3. **Power-stage driver class** (planned, `drivers/power_stage/`) — the only per-MCU part: synchronised
   PWM + current/Vbus sampling (STM32 TIM1 + ADC1 injected, and a sim driver wrapping the plant).
   Deliberately not Zephyr's `pwm`/`adc` APIs.

`zephyr/module.yml` registers the module (cmake, Kconfig, `dts_root`); `Kconfig` makes
`CONFIG_PMSM_FOC` select `FPU` + `FPU_SHARING` (ISR and thread both use float; Zephyr defaults to
soft-float).

## Core conventions (`foc.h`)

- Amplitude-invariant Clarke; d at θ, q leads by 90°; electrical radians, SI units; duties 0..1.
- Vectors are small float structs passed/returned by value (FPU registers under hard-float ABI).
- Hot-path helpers are `FOC_INLINE` (`always_inline`): at `-Os` GCC otherwise emits out-of-line calls.
  Prefer forms that map to FPv5 instructions (`fminf`/`fmaxf` → VMAXNM/VMINNM, `roundf` → VRINTA,
  `sqrtf` with a provably non-negative argument → VSQRT without errno check). Verify in disassembly.
- **Non-finite policy**: functions that keep state, set limits or produce duties must always return
  finite, in-range values (NaN → neutral value, ±Inf saturates; `foc_sanitizef`). Linear transforms
  propagate NaN on purpose so faults stay visible; fault checks must be NaN-aware
  (`!(x >= min && x <= max)` or `isfinite()`). Add cases to `tests/lib/foc/src/test_nonfinite.c`:
  every new API function goes through its `bad[]` set (NaN, ±Inf, ±FLT_MAX, overflow-when-squared,
  sincos range, subnormals), each argument alone and all at once.
- Gains are derived from motor parameters (`foc_gains_*`); DT parameters are integers with unit
  suffixes, converted to float once at init.

## Rules

- Implement from first principles (textbook algorithms). Never copy code or structure from ST MCSDK,
  TI or other vendor motor libraries; avoid vendor-branded observers and single-shunt schemes.
- Doxygen comments use backslash commands (`\brief`, `\p`, `\param`, `\pre`, `\return`), never `@`.
- Source files start with `SPDX-License-Identifier: MIT`.
- Code follows Zephyr style: `zephyr/.clang-format` and `checkpatch.pl` with Zephyr's
  `.checkpatch.conf` (`scripts/lint.sh` checks both). `lib/` has no Zephyr includes, so no
  `ARRAY_SIZE` etc. there.
- Tests: one ztest suite per file in `tests/<area>/src/`; `testcase.yaml` lists tags as YAML lists
  (twister rejects space-separated strings).
