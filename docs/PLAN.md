# Sensorless PMSM FOC — Zephyr module (living plan)

Status legend: `[ ]` todo · `[~]` in progress · `[x]` done · `[!]` blocked/needs decision.

This file, `docs/FEATURES.md` (backlog) and `docs/HARDWARE.md` (BOM, board mods, pin map) are living documents: update status and the insights
log after every step. Anything not needed for the v1 goal goes to `FEATURES.md`.

## Goal (v1 done =)

One PMSM (DFRobot FIT1036) on nucleo_c542rc + X-NUCLEO-IHM16M1 (3 low-side shunts) goes standstill → closed-loop speed
control → stop, commanded from the shell, with HW over-current protection. Same code passes a
closed-loop simulation in CI on `native_sim`.

Idiomatic, MCU-agnostic Zephyr module, implemented from first principles (textbook Clarke/Park,
SVPWM, PI, Luenberger observer + PLL); no code copied from ST MCSDK, TI, or others.

## Decisions

| Topic | Decision |
|---|---|
| Abstraction boundary | Power-stage driver class (synchronised PWM + current/Vbus sampling), not Zephyr `pwm`/`adc` APIs → full TIM1/ADC HW acceleration kept. Portable = math core + motor subsystem; per-MCU = one power-stage driver. Cost: one indirect call per PWM cycle |
| Target | nucleo_c542rc (STM32C542RC; C562RE unavailable) + X-NUCLEO-IHM16M1 + DFRobot FIT1036, see `docs/HARDWARE.md`. v1 uses only what the cheapest C5 (C531) also has: TIM1 + ADC1 |
| Numerics | `float32` (FPU), no fixed-point |
| PWM | Frequency from DT (default 20 kHz), center-aligned, FOC loop every cycle |
| Current sensing | 3 low-side shunts, ADC1 only: per cycle sample the 2 phases with longest low-side window + Vbus; 3rd from ia+ib+ic=0. Phase selection lives in the driver |
| Phase voltages | Computed from duties: `v_x = (d_x − mean(d)) · Vbus`. Duty→sample delay modelled explicitly (duty history buffer); the observer uses the duty actually applied during the interval preceding the sample. Sim models the same delay; M5 verifies it on a scope |
| ISR context | FOC in ADC-EOC ISR (`IRQ_DIRECT_CONNECT` in the driver, callback into motor), no kernel calls. ISR owns everything per-cycle: sampling, forced-angle integration, observer + PLL, angle select, current loop, per-cycle fault checks |
| Thread context | Dedicated motor thread at 1 kHz (not the system work queue): speed PI, reference ramps, state transitions, fault handling/reporting |
| ISR ↔ thread data | Two structs, each single-writer: `cmd` (thread → ISR: state, id/iq refs, forced-angle speed) and `status` (ISR → thread). Exchanged via double buffer + atomic index; no locks in ISR |
| Startup | Align → I/f ramp (current loop closed, forced angle) → hard switch to observer at fixed handover speed, speed-PI integrator preloaded with the I/f current → closed loop |
| Observer | Luenberger on αβ current model; back-EMF states with rotating model (de/dt = ω·J·e, ω from PLL). Angle + speed from PLL |
| Gains | Computed from motor params at init: PI by pole cancellation (Kp = L·ωc, Ki = R·ωc), observer poles and PLL bandwidth as multiples. Bandwidths in DT |
| Faults v1 | HW break (TIM1 BRK) → PWM off. SW per cycle: over-current, over/under-voltage → PWM off, state = FAULT. Clear via `stop` |
| API | Zephyr device per motor (`compatible = "pmsm,motor"`) referencing a power-stage node. API: `start`, `stop`, `set_speed`, `get_status` |
| DT params | Integers with explicit units (`resistance-micro-ohms`, `inductance-nano-henries`, `flux-linkage-micro-webers`, `pole-pairs`, `inertia-nano-kg-m2`, `max-current-milliamps`, bandwidths in Hz…), converted to float once at init |
| Verification | ztest unit tests + closed-loop sim. Sim (M2/M3) = plain C plant + FOC core in a ztest loop, no devices/timers → deterministic. `sim-power-stage` driver only in M4 to test the device layer |
| Packaging | This repo = Zephyr module + `west.yml` (T2 manifest pinning Zephyr). West workspace lives outside the repo |

## IP / patents

All used techniques are textbook and decades old. Avoid: vendor-branded observers, single-shunt
phase-shift schemes, copying vendor code/structures. Not legal advice — FTO check before product
release.

## Module layout

```
west.yml                        # pins zephyr + hal_stm32 + cmsis_6
.devcontainer/                  # devcontainer.json, post-create.sh (west init/update)
zephyr/module.yml  CMakeLists.txt  Kconfig
include/pmsm/foc.h              # pure-C core API (no Zephyr deps); tiny transforms inline
include/pmsm/motor.h            # public device API
include/pmsm/power_stage.h      # driver class API
lib/foc/                        # transforms.c (sincos, circle limit) svpwm.c pi.c gains.c foc.c (current loop) observer.c pll.c
subsys/motor/                   # motor.c (device, ISR glue, thread, faults), motor_shell.c
drivers/power_stage/            # power_stage_sim.c, power_stage_stm32_tim.c
dts/bindings/                   # pmsm,motor.yaml, pmsm,sim-power-stage.yaml, st,stm32-tim-power-stage.yaml
tests/lib/foc/                  # unit tests
tests/sim/                      # plant/ (pure-C plant model) + src/ closed-loop scenarios
tests/motor/                    # device layer on sim power stage
samples/motor_basic/            # shell sample; boards/nucleo_c542rc.overlay
docs/PLAN.md  docs/FEATURES.md  docs/HARDWARE.md
```

### Power-stage API (sketch)

- `calibrate_offsets(dev)`
- `enable(dev)`, `disable(dev)` (ISR-safe)
- `set_duty(dev, da, db, dc)` — 0..1, latched at next update event
- `register_cb(dev, fn(const struct ps_sample *, void *), user)` with
  `ps_sample = {ia, ib, ic, vbus}`
- `get_info(dev)` — pwm freq, dead-time, max duty for sampling window

STM32 impl: TIM1 center-aligned complementary + dead-time, preload, TRGO2/OC4 → ADC injected
sequence (2 currents per sector + Vbus), BRK input, JEOC IRQ.

### Core algorithm per cycle (ISR)

sample → Clarke → observer update (applied v_αβ) → PLL → angle select (forced / observer per
state) → Park (sin/cos once) → PI d/q with anti-windup → voltage circle limit → inverse Park →
SVPWM (min/max zero-sequence injection) → `set_duty` → fault checks.

## Milestones

### M0 Workspace & skeleton
- [x] M0.1 `docs/PLAN.md`, `docs/FEATURES.md`, `docs/HARDWARE.md`
- [x] M0.2 `west.yml` (T2, Zephyr v4.5.0 pinned to SHA, allowlist `cmsis_6` + `hal_stm32`); devcontainer (`ghcr.io/zephyrproject-rtos/ci:v0.29.4`, workspace volume `/workdir`, repo at `/workdir/spin`)
- [x] M0.3 Verify STM32C5 support upstream (SoC, dtsi, nucleo_c542rc board, HAL2 LL tim/adc)
- [x] M0.4 Module skeleton (`module.yml`, Kconfig `CONFIG_PMSM_FOC`, `lib/foc`, `include/pmsm/foc.h`, `tests/lib/foc` with one trivial test); twister on native_sim passes

### M1 Core math (host)
- [x] M1.1 Clarke/Park/inverse Park, fast sin/cos
- [x] M1.2 SVPWM (min/max injection), voltage circle limit
- [x] M1.3 PI with output limits + anti-windup
- [x] M1.4 Gain calculation from motor params
- [x] M1.5 ztest vectors for all of the above

### M2 Plant + closed-loop sim (ideal angle)
- [x] M2.1 Discrete PMSM model (dq: Rs, Ls, flux, pole pairs, J, B, load torque) + ideal inverter (duty → voltage) with the duty delay
- [x] M2.2 FOC current loop + speed loop with true angle; CI tests: current step response, speed step

### M3 Sensorless
- [ ] M3.1 Luenberger observer (rotating EMF) + PLL; test against plant angle at constant speed
- [ ] M3.2 Startup state machine (align → I/f → handover → closed loop), ISR/thread split as decided
- [ ] M3.3 Per-cycle faults (OC, OV, UV)
- [ ] M3.4 Sim scenarios in CI: start-up, speed steps, load step

### M4 Zephyr device
- [ ] M4.1 Power-stage API header + `pmsm,sim-power-stage` driver (wraps the plant)
- [ ] M4.2 `pmsm,motor` binding + device, motor thread, cmd/status exchange
- [ ] M4.3 Shell: `motor start|stop|speed <rpm>|status`
- [ ] M4.4 `samples/motor_basic` on native_sim; `tests/motor`

### M5 STM32 power stage
- [ ] M5.1 TIM1 PWM, verify on scope without motor (frequency, dead-time, center alignment)
- [ ] M5.2 ADC injected trigger + sampling point + duty→sample delay on scope; offset calibration
- [ ] M5.3 BRK input
- [ ] M5.4 Align at low current with current-limited PSU → I/f → closed loop
- [ ] M5.5 ISR timing (DWT cycle counter / GPIO) < 50 % of PWM period at 20 kHz

### M6 Docs
- [~] M6.1 README + integration guide (binding, params, bring-up procedure)

## Verification

- `west twister -T tests -p native_sim` runs unit + sim + device tests (CI-ready).
- `scripts/lint.sh`: Zephyr clang-format + checkpatch, GCC `-fanalyzer` on the ARM build, and the
  tests on `native_sim/native/64` under ASan + UBSan (a finding fails the test).
- `west build -b nucleo_c542rc samples/motor_basic` builds.
- On target: shell commands; ISR budget < 50 % of PWM period at 20 kHz.

## Insights log

- 2026-11-25 (M0): Devcontainer works: `west list` shows zephyr @ v4.5.0 SHA, `cmsis_6`, `hal_stm32`;
  SDK 1.0.1 in `/opt/toolchains`. The manifest repo is picked up as a Zephyr module via
  `zephyr/module.yml` without extra config.
- STM32C5 upstream in v4.5.0: `soc/st/stm32/stm32c5x`, `dts/arm/st/c5/stm32c542.dtsi` (includes c532),
  board `nucleo_c542rc` (hello_world builds: 16 KB flash). TIM1/ADC1 nodes exist (ADC compatible
  `st,stm32n6-adc`). `hal_stm32` ships STM32CubeC5 2.1.0 LL with `LL_ADC_INJ_*` (JSQR), TIM break,
  TRGO2, dead-time. Board yaml lists no `adc`/`pwm` support — irrelevant, we use our own driver.
- Twister rejects space-separated `tags`; use YAML lists.
- Run tests: `west twister -T spin/tests -p native_sim` from `/workdir`.
- Zephyr leaves the FPU off by default (hello_world: soft-float). `CONFIG_PMSM_FOC` now selects
  `FPU` + `FPU_SHARING` when `CPU_HAS_FPU` (ISR and motor thread both use float).
- Small math helpers are `static inline` in `foc.h`. `fminf`/`fmaxf` compile to VMAXNM/VMINNM
  on FPv5 without `-ffast-math` (checked with the real `nucleo_c542rc` build flags, `-Os`).
- 2026-11-25 (M1): Core math done. Clarke/Park/inverse Park, `foc_wrap_pi`, `foc_q_limit`,
  `foc_duty_to_ab` are `static inline`; others are in `lib/foc/*.c`. Vectors are small float structs,
  passed and returned in s0–s3 under the hard-float ABI.
- `foc_sincos`: Cody-Waite reduction with pi/2 split into 3 parts (exact for |x| < 3200), then
  Taylor to r^9 / r^8. Measured max error < 2e-7 on ±20 rad, < 1e-6 up to ±3000 rad.
  ~60 instructions, no libm calls, no tables.
- SVPWM: `max_duty` clamps both ends symmetrically; the linear limit is
  `(2·max_duty − 1)·Vbus/√3`. The top duty only reaches `max_duty` at 30° + k·60° on that circle.
- Voltage limit: d first. The q-PI limit is `foc_q_limit(vd, vmax)` per cycle, so anti-windup sees
  the real circle. PI anti-windup: the integrator moves towards a limit only until the output
  reaches it (`integ ≤ hi − p`), holds if p alone saturates, and is clamped to the current limits
  (handles a shrinking q limit). The first version held the integrator whenever a step would
  overshoot the limit, which froze a low-Kp PI below the limit. Caught by an Inf test, now covered
  by `test_integrator_reaches_limit`.
- Speed-PI gains need J → new DT param `inertia-nano-kg-m2`. PI zero at wc/4 (~76° PM).
  FIT1036 rotor J ≈ 6e-6 kg·m² (estimate, **⚠ unverified**).
- Zephyr builds with `-Wdouble-promotion -Werror`; native_sim uses `-m32 -msse2 -mfpmath=sse`,
  so there is no x87 excess precision in tests.
- Twister: `pmsm.lib.foc` runs on native_sim, `pmsm.lib.foc.build` is build-only on nucleo_c542rc.
  Run both: `west twister -T spin/tests -p native_sim -p nucleo_c542rc`.
- Non-finite policy (see `foc.h` header comment): functions that keep state, set limits or produce
  duties always return finite values in range (NaN → neutral value, ±Inf saturates). The linear
  transforms propagate NaN on purpose; **M3.3 fault checks must be NaN-aware** (`!(x >= min && x <= max)`
  or `isfinite()`). A plain `x > max` test is false for NaN and would let a broken reading through.
  Covered by `tests/lib/foc/src/test_nonfinite.c`.
- Intrinsics (FPv5, checked in the disassembly): VMAXNM/VMINNM (`fmaxf`/`fminf`), VSQRT (`sqrtf`, no
  errno check because the argument is provably ≥ 0), VRINTA (`roundf`, no `-fno-math-errno`
  needed), VSEL, VMLA/VMLS (contracted multiply-add). `lroundf` → VCVTA only with `-fno-math-errno`;
  not needed because `roundf` + cast is just as short. Header helpers are `always_inline`:
  at `-Os` GCC otherwise calls `foc_clampf` out of line.
- Per call (instructions, nucleo_c542rc `-Os`): `foc_sincos` 49 (branch-free), `foc_svpwm` 62,
  `foc_pi_step` 29, `foc_limit_circle` 31.
- CORDIC (present on C5): not used. See FEATURES.md.
- 2026-11-25: Zephyr's static checks, added as `scripts/lint.sh`. Pitfalls:
  `ZEPHYR_SCA_VARIANT=gcc` fails on `native_sim` (it reads `GCC_COMPILER_VERSION`, which only the SDK
  toolchain sets), so it runs on `nucleo_c542rc`. Its 11 findings are all in Zephyr `cbprintf`
  and the STM32 LL HAL, none in this module. Sanitizers need the 64-bit `native_sim`. Zephyr's
  `CONFIG_ASAN_RECOVER` defaults to y and UBSan only reports by default, so without twister's
  `--enable-*san` flags a finding would not fail the test. The `.sanitize` scenario sets
  `ASAN_RECOVER=n` + `-fno-sanitize-recover=all` (checked with an injected signed overflow).
  checkpatch must run from `zephyr/` to pick up `.checkpatch.conf` (it ignores the SPDX tag
  placement). The first run found only an `ARRAY_SIZE` hint and formatting in two test files.
- 2026-11-26 (M2.1): Plant in `tests/sim/plant/` (`plant.h`/`plant.c`, no Zephyr includes): surface
  PMSM in the rotor dq frame + ideal inverter, double precision, RK4 with `substeps` per PWM period
  (default 8 in tests). The inverter voltage is held constant in the **stationary** frame over the
  period (cycle average), so the dq voltage rotates within a step as on the real motor. Call order
  per cycle: `plant_sample()` → controller → `plant_step(duty)`. `duty_delay` (0..2, default 1; 1 is what the hardware should have, 2 covers a missed latch and robustness tests)
  is a FIFO: the duty passed now is applied `duty_delay` steps later; `p.applied` / `p.va` hold
  the duty and αβ voltage of the last step (what the M3 observer must use). `t_load` is an active
  torque (it reverses a stalled rotor); passive loads come with M3.4 if needed. State fields are
  public, so scenarios set initial speed/angle, load and Vbus steps directly.
- Tests (`pmsm.sim`, + `.sanitize`; host only, no target build): locked-rotor RL step vs. the analytic
  response (1e-7 A), duty delay 0..2 and FIFO order, common mode / clamping, short-circuit steady
  state at constant speed (checks the cross-coupling and back-EMF signs, 1e-6 rel.), torque sign and
  Kt via impulse = J·Δω, viscous decay and constant-load deceleration. Comparisons use the applied
  `p.va`, not the requested voltage: float duty rounding alone is ~4e-7 A.
- Twister merges `common: platform_allow` with the per-scenario list, so a common list makes every
  scenario run on every platform. Keep `platform_allow` per scenario only.
- M4.1 (sim power stage) will need the plant inside the module; move `tests/sim/plant/` then (it has
  no Zephyr dependencies, so only the path changes).
- 2026-11-26: `README.md` documents the physics and control strategy, mostly as diagrams: P&ID
  (`docs/img/pid.svg`), signal flow, startup states and layers (Mermaid), frames and PWM timing
  (`docs/img/*.svg`, light/dark via `prefers-color-scheme`), plus the equations (amplitude-invariant
  Clarke and the 3/2 in torque, motor model, gains, observer/PLL). Integration guide still to do.
- 2026-11-26 (M2.2): `foc_current_step()` in `lib/foc/foc.c`: Clarke → Park (θ) → d PI (±vmax) → q PI
  (±`foc_q_limit(vd, vmax)`) → inverse Park at θ + ω·t_adv → SVPWM. Takes the angle and speed as inputs, so
  the M3 observer/forced angle plug in without changes. No decoupling feed-forward (backlog). The speed PI
  stays a plain `foc_pi` stepped by the caller (the motor thread in M4; `tests/sim/src/sim.c` now).
- Angle advance `t_adv = 1.5·Ts` (`FOC_ADVANCE_PERIODS`): the duty acts during the next period, centred
  1.5 periods after the sample. At 40 rad/s mech it cuts the d excursion on a 1 A q step from 63 to 53 mA.
  Costs a second `foc_sincos`.
- Sim results (FIT1036 estimates, 24 V, 20 kHz, delay 1, current 500 Hz, speed 50 Hz): current step
  t63 = 0.94 τ, no overshoot at standstill or ±40 rad/s; at fs/20 (1 kHz) 2 % overshoot, at fs/10 13 %.
  Speed step 0 → 50 rad/s: 18 % overshoot (PI zero at wc/4 on a reference step; M3 ramps remove it),
  settles to 2 % in 36 ms. Load step 0.05 Nm: dip to 31 rad/s (J is tiny), back in 35 ms, iq = TL/Kt.
  Top speed at 24 V is where the back-EMF fills the circle: vmax/(p·ψ) ≈ 103 rad/s ≈ 980 rpm with the
  ψ estimate; no windup in either PI when saturated there.
- With the duty delay the PI "catches up": delay 0, 1 and 2 cross 63 % on the same cycle at 500 Hz, so
  the response is a poor detector of the delay. M3 observer tests must check the delay directly.
- `foc_current_step` on nucleo_c542rc `-Os`: 79 instructions plus calls to `foc_sincos` (2×),
  `foc_pi_step` (2×) and `foc_svpwm`: about 300 instructions per cycle, a few % of the 20 kHz budget.
- 2026-11-26 non-finite audit of every `foc.h` function. Bug: a NaN speed in `foc_current_step()`
  made the inverse Park use angle 0 instead of θ (the doc said "speed counts as 0"); Inf·0 with
  t_adv = 0 did the same. Fix: the advance ω·t_adv goes through `foc_sanitizef(…, π)`. Gaps closed:
  subnormals and 1e20 (overflows when squared) in `bad[]`; NaN/Inf propagation asserted for the
  linear transforms; `foc_torque_constant` → `foc_gains_speed`; PI with FLT_MAX gains and limits;
  `foc_current_init` with bad gains/ts/max_duty/t_adv; `foc_current_step` with all inputs bad at once.
  Known, documented: θ beyond `FOC_SINCOS_MAX_ARG` counts as 0, so callers must pass a wrapped angle.
