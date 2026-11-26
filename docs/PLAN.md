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
| Startup | Align (d current ramp at 0, then the vector turns 90° in the start direction: no 180° dead point) → I/f ramp (current loop closed, current on the forced d axis; PLL slaved to the forced angle below ½ handover, then free on the observer) → hard switch to observer at fixed handover speed: current-PI integrators rotated into the PLL frame, speed-PI integrator preloaded with the torque part i_start·sin(δ) of the I/f current → closed loop. Handover speed ≥ the speed where ω·ψ clearly exceeds R·i_start (Rs errors read as EMF in I/f, see insights M3.2). Direction fixed at start, minimum speed = handover speed |
| Observer | Luenberger on αβ current model; back-EMF states with rotating model (de/dt = ω·J·e, ω from PLL). Angle + speed from a normalised PLL with a frequency aid (see insights M3.1) |
| Gains | Computed from motor params at init: PI by pole cancellation (Kp = L·ωc, Ki = R·ωc), observer poles and PLL bandwidth as multiples. Bandwidths in DT |
| Faults v1 | HW break (TIM1 BRK, raised via `foc_cycle_trip`) → PWM off. SW per cycle while the PWM is on: over-current (any phase, NaN trips), over/under-voltage (NaN trips both) → zero voltage in the same cycle, state = FAULT, PWM off. Latched in ISR and thread; cleared only via `stop`. The ISR clamps the current reference to the trip level |
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
include/pmsm/power_stage.h      # driver class API; power_stage_sim.h: sim extras (plant access, BRK, offsets)
include/pmsm/plant.h            # plant model API (sim only)
lib/plant/                      # plant.c: pure-C PMSM + ideal inverter, double precision (CONFIG_PMSM_PLANT)
lib/foc/                        # transforms.c (sincos, circle limit) svpwm.c pi.c gains.c foc.c (current loop) observer.c (+ duty history) pll.c
                                # cycle.c (per-cycle ISR part: faults, forced angle, angle select) seq.c (1 kHz startup/speed sequencer)
subsys/motor/                   # motor.c (device, ISR glue, thread, faults), motor_shell.c
drivers/power_stage/            # power_stage_sim.c, power_stage_stm32_tim.c
dts/bindings/                   # vendor-prefixes.txt (pmsm); power-stage/pmsm,sim-power-stage.yaml,
                                # st,stm32-tim-power-stage.yaml; motor/pmsm,motor.yaml
tests/lib/foc/                  # unit tests
tests/sim/                      # closed-loop scenarios on the plant (no devices)
tests/power_stage/              # sim power-stage driver: API contract + closed loop through the API
tests/motor/                    # device layer on sim power stage
samples/motor_basic/            # shell sample; boards/nucleo_c542rc.overlay
docs/PLAN.md  docs/FEATURES.md  docs/HARDWARE.md
```

### Power-stage API (`include/pmsm/power_stage.h`)

- `power_stage_calibrate_offsets(dev)` — thread, outputs off
- `power_stage_enable(dev)` (re-arms BRK; `-EIO` while BRK active), `power_stage_disable(dev)` (duty → 0.5); ISR-safe
- `power_stage_set_duty(dev, da, db, dc)` — 0..1, clamped to [1 − max_duty, max_duty], NaN → 0.5, latched at the
  next update event
- `power_stage_register_cb(dev, fn(dev, const struct power_stage_sample *, user), user)` with
  `power_stage_sample = {ia, ib, ic, vbus, flags}`; `POWER_STAGE_FLAG_BREAK` reports a hardware break.
  The callback runs every PWM period, outputs on or off; enable/disable switch the bridge only
- `power_stage_get_info(dev)` — `ts`, `dead_time`, `max_duty` (sampling window), `duty_delay`

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
- [x] M3.1 Luenberger observer (rotating EMF) + PLL; test against plant angle at constant speed
- [x] M3.2 Startup state machine (align → I/f → handover → closed loop), ISR/thread split as decided
- [x] M3.3 Per-cycle faults (OC, OV, UV)
- [x] M3.4 Sim scenarios in CI: start-up, speed steps, load step

### M4 Zephyr device
- [x] M4.1 Power-stage API header + `pmsm,sim-power-stage` driver (wraps the plant)
- [x] M4.2 `pmsm,motor` binding + device, motor thread, cmd/status exchange; `tests/motor` (API, faults, offsets)
- [ ] M4.3 Shell: `motor start|stop|speed <rpm>|status`
- [ ] M4.4 `samples/motor_basic` on native_sim; extend `tests/motor` (shell)

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
  (`docs/img/pid.svg`), signal flow, frames and PWM timing (`docs/img/*.svg`, light/dark via
  `prefers-color-scheme`), startup states and layers (Mermaid), plus the equations (amplitude-invariant
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
- 2026-11-26 (M3.1): `foc_obs` (`lib/foc/observer.c`): exact ZOH discretisation of the αβ RL model
  (a = e^(−R·Ts/L), b = (1 − a)/R via `expm1f`, so it stays accurate for small R·Ts/L and R = 0). The EMF
  is taken at mid-period (half rotation applied twice, so one `foc_sincos`). Predict, then correct with
  r = i − î⁻. Gains in closed form put a double pole at z = e^(−2π·bw·Ts): g_i = 1 − z²/a,
  g_e = (1 − z)²/b. Bad parameters give an inert observer (ê = 0). State is clamped to `i_lim`/`v_lim`.
  `foc_duty_hist` (delay 0..2) pairs each sample with the duty applied in the period before it, i.e. the
  duty written `delay + 1` cycles earlier; the sim checks it against `plant.applied` every cycle.
- `foc_pll` (`lib/foc/pll.c`): ε = −e_d·sign/max(|ê|, e_min), critically damped PI
  (`foc_gains_pll`: Kp = 2ω_n, Ki = ω_n²), angle advanced by ω̂·Ts before each step. Two pitfalls found
  in the sim:
  1. Taking the sign from ω̂ (the PI output) gave a period-2 limit cycle when Kp·ε > |integrator|:
     ω̂ alternated +836/−1933 rad/s with the true speed at −550. The sign now comes from the integrator.
  2. A phase detector alone pulls in slowly: wrong-sign preloads took up to 9000 cycles. A frequency aid
     fixes this. It adds k_f·Ts·(ê_dq,prev × ê_dq)/(|ê|²·Ts) to the integrator, with k_f = Kp/4. That is the
     rotation residual in the PLL frame (≈ ω − ω̂) and does not depend on the angle error. The first
     version used the αβ rotation sin(Δφ)/Ts. It was biased by (ωTs)²/6 (3.8e-4 rad error at
     1200 rad/s) and pulled ω̂ to 0 without EMF. Measuring in the PLL frame fixes both. Lock now takes
     < 400 cycles from any preload above ≈ 100 rad/s electrical. At 30 rad/s electrical
     (|ê| = 0.33 V), a wrong-sign preload can still fail to lock, so M3.2 must preload from the forced angle.
- Sim results (defaults: observer 2 kHz, PLL 150 Hz, e_min 0.1 V; all from zero state at standstill,
  acquired during the speed-up on the true angle). Steady state at 5..90 rad/s, either direction, 0 or
  0.05 Nm: angle error < 0.3 mrad, speed error < 2e-4 rad/s, |ê| within 1e-3 of p·ψ·ω. Rs ±30 %:
  no angle error (acts along q at i_d = 0), |ê| off by 5 %. Ls ±30 %: 11 mrad. Duty delay assumed ±1
  period: 32 mrad, 100× the matched error. The prediction error does **not** reveal a wrong delay at
  constant speed (the EMF estimate absorbs the error); the angle against the plant does. On the
  hardware (M5.2) check the delay on the scope, not via the observer residual.
- Sensorless closed loop after a switch from the true angle at 50 rad/s: 0.05 Nm load step costs
  ≤ 45 mrad of angle error during the dip, a 50 → 80 rad/s step ≤ 60 mrad; steady state back to < 1 mrad.
  The PLL angle lag under acceleration matches asin(α/ω_n²) (5 %, unit test).
- Cost on nucleo_c542rc `-Os`: `foc_obs_step` 137 instructions + 1 `foc_sincos`,
  `foc_pll_step` 119 + `foc_sincos` + `foc_pi_step` (now one VDIV, one VSQRT). Observer + PLL add
  ≈ 400 instructions per cycle; the whole per-cycle FOC path is ≈ 700.
- Host `math.h` declares `finite()`; don't name static helpers after libm functions (broke a host
  build of `lib/foc`).
- 2026-11-26 (M3.2–M3.4): Split as decided, both in the math core (no Zephyr): `foc_cycle` (`lib/foc/cycle.c`,
  ISR) and `foc_seq` (`lib/foc/seq.c`, motor thread), exchanging `struct foc_cmd` (thread → ISR: state,
  i_ref, forced speed, `pll_hold`) and `struct foc_status` (ISR → thread). M4 only adds the double buffer
  and the power-stage glue: PWM on iff `status.state` is ALIGN/IF/RUN. The sim harness
  `tests/sim/src/drive.c` runs `foc_cycle` every PWM cycle and `foc_seq` every 20 cycles on the plant.
- I/f puts the current on the forced **d** axis (same frame as align, so the I/f start causes no torque
  step). The rotor lags by δ with Kt·i·sin δ = load; at the handover `i_q = i_start·sin δ` (δ = forced − PLL
  angle, from the status) keeps the torque, and the current-PI integrators are rotated by δ so the voltage
  vector is continuous. The forced angle follows the PLL in RUN, so δ reads 0 there.
- Two-step align: a rotor exactly 180° off is a dead point of a one-step align; it then ran I/f with the
  PLL locked 180° off. Turning the align vector by 90° in the third quarter of `t_align` fixes it (any
  start angle tested, both directions). In the sim the current loop damps the align swing within ~50 ms;
  no active damping needed (the plant has almost no friction, so the hardware should be easier).
- Handover speed: this motor has a large R·i against a small ψ: at 1 A and 15 rad/s mech, R·i = 3.6 V
  vs ω·ψ = 1.8 V. In I/f with i_d = 1 A an Rs error turns ΔR·i_d into a false EMF along d. With Rs +30 %
  and handover at 15 rad/s the PLL locked 180° off and the motor ran backwards. At 30 rad/s (≈ 290 rpm)
  Rs/Ls ±30 % all start: handover angle error ≤ 0.29 rad, speed dip to 14 rad/s at worst (Rs ×1.3), back
  in lock < 50 ms. On the hardware: measure Rs, or lower i_start before raising the handover speed.
- Plant: the active `t_load` turns a stalled rotor backwards, so a loaded start needs a passive load. New
  `t_fric` (Coulomb, sign smoothed with tanh over 0.5 rad/s) and `plant_coast()` (PWM off: currents 0,
  mechanics only; valid below the voltage-limited top speed where the diodes stay off).
- Faults: checked only while the PWM is on (trips while off are dropped, so a low Vbus at rest does not
  latch; starting at that voltage trips in the first active cycle). Bad limits fail safe: they trip on the
  first active cycle. The sequencer leaves STOP only once the status also says STOP, so a lagging FAULT
  status cannot re-latch on restart. An over-current test "trip during align" was dropped: the ISR clamps
  i_ref to i_trip, so with working sensing the loop cannot reach the trip level; OC comes from sensing
  faults and shorts (covered by an injected offset and NaN, and by BRK on the hardware).
- Sim results (defaults: i_start 1 A, align 0.1 s, I/f 300 rad/s² to 30 rad/s, ramp 500 rad/s²): standstill
  → RUN in 0.2 s from any rotor angle; 50 rad/s reached with < 5 % overshoot, steady angle error < 1 mrad.
  Start against 0.05 Nm friction: δ = asin(load/(Kt·i)) within 0.1 rad, preload within 15 % of load/Kt,
  no dip below 90 % of the handover speed. 40 → 90 rad/s step < 3 % overshoot; 0.05 Nm load step at 90
  rad/s dips < 40 %; target below the handover speed (or reversed) holds at the handover speed.
- Cost on nucleo_c542rc `-Os`: `foc_cycle_step` 297 instructions (includes the rarely taken transition
  code), `fill_status` 55 (one VSQRT), no libm calls; the per-cycle path is ≈ 850 instructions.
  `foc_seq_step` 188, at 1 kHz.
- 2026-11-26 (M4.1): Plant moved to `lib/plant/` + `include/pmsm/plant.h` (`CONFIG_PMSM_PLANT`, selects
  `PMSM_FOC`); `tests/sim` uses it unchanged. Power-stage class in `include/pmsm/power_stage.h`
  (`__subsystem` + `DEVICE_API(power_stage, …)`), `CONFIG_PMSM_POWER_STAGE`. Out of tree, the header must be
  registered with `zephyr_syscall_header()` (done in `drivers/CMakeLists.txt`), otherwise the API's iterable
  section is not generated and `DEVICE_API_GET` fails to link (`_power_stage_driver_api_list_start`).
  Twister treats an unknown vendor prefix as a devicetree error: `dts/bindings/vendor-prefixes.txt` adds `pmsm`.
- BRK is reported in-band (`POWER_STAGE_FLAG_BREAK` in the next sample) instead of a second callback, so the
  motor ISR glue raises `foc_cycle_trip(FOC_FAULT_EXT)` in the same context as `foc_cycle_step` (no locking).
- `pmsm,sim-power-stage` (`drivers/power_stage/power_stage_sim.c`): per cycle sample (+ injected − calibrated
  offset) → callback → `plant_step(latest duty)` if on, else `plant_coast()`. DT holds the "true" motor (integer
  units), separate from the controller's parameters, so mismatch tests stay possible. Cycle source: a kernel
  timer that runs the cycles due by `k_uptime_ticks()` (`tick-period-us`, default 1000), or none
  (`tick-period-us = 0`) and the test steps with `power_stage_sim_run()` (deterministic). native_sim's tick is
  100 Hz, so a 1 ms timer fires every 10 ms: a fixed batch per expiry ran 10× too slow; following the uptime
  fixes it (200-cycle batches at 20 kHz). Extras: `power_stage_sim_lock()` (plant access), `_set_offset()`,
  `_brk()`, `_cycles()`.
- Tests `pmsm.power_stage.sim` (+ `.sanitize`): sample = plant state, one callback per cycle on and off, duty delay
  and hold, clamp incl. non-finite, disable in the callback acts in the same cycle, `-EBUSY`/`-EIO`/`-ERANGE`
  paths, offset calibration, BRK reported once and blocking enable while active, timer source tracks uptime.
  Closed loop through the API (`foc_cycle` in the callback, `foc_seq` every 20 cycles in it for determinism):
  standstill → RUN in < 0.5 s, 50 rad/s ± 5 %, BRK → FAULT with `FOC_FAULT_EXT`, stop clears it.
  Pitfall: 950 · 1e-3f ≠ 0.95f and 1 − 0.95f ≠ 0.05f in float; tests compare against `info->max_duty`.
- 2026-11-26 (M4.2): `pmsm,motor` device (`subsys/motor/motor.c`, `include/pmsm/motor.h`, `CONFIG_PMSM_MOTOR`,
  default y with a `pmsm,motor` node; selects `PMSM_FOC` + `PMSM_POWER_STAGE`). The API is plain functions on the
  device (one implementation, no vtable). Speeds are mechanical rad/s (SI like the core); the binding takes rpm.
  All gains and limits come from DT at build time, the timing (`ts`, `max_duty`, `duty_delay`) from
  `power_stage_get_info()` at init. PLL/forced-speed limit = 2 · p · max speed. Currents, Vbus trip levels and
  max speed are required properties (no safe default); BUILD_ASSERTs check trip > max/start current,
  vbus-max > vbus-min, handover < max speed, init priority above the power stage's.
- cmd/status exchange: each buffer pair has one writer, which fills `b[(n + 1) & 1]` and then `atomic_inc(n)`
  (wraps, no signed overflow). The ISR reads cmd without a retry (the thread cannot preempt it); the thread
  retries its status copy if n advanced by ≥ 2 (the second ISR write reuses the buffer being copied).
  The ISR writes `foc_cycle_step`'s status straight into the inactive buffer (no copy). Single-core argument;
  revisit for SMP.
- Motor thread: `k_timer` + `k_timer_status_sync()`; when late it steps `foc_seq` once per elapsed period (ramps keep
  their rate) and counts the misses in `motor_status.late`. The period must be whole kernel ticks (BUILD_ASSERT):
  native_sim's 100 ticks/s would run the 1 kHz speed loop at 100 Hz. `tests/motor` sets 10000 ticks/s.
- ISR glue: disable every off cycle (cheap, also resets the duty after a BRK); if `power_stage_enable()` fails
  (BRK input still active) it raises `FOC_FAULT_EXT`, otherwise ALIGN would wind the current PIs up with the
  bridge off. `motor_start()` calibrates the sense offsets every time (outputs are off in STOP), so drift is
  removed per start; out-of-range offsets refuse the start (`-ERANGE`).
- `foc_seq.faults` keeps its bits after STOP; `motor_status.faults` reports them only in FAULT (caught by the test).
- `tests/motor` (`pmsm.motor`, `.sanitize`, `.build` on nucleo_c542rc with the sim power stage as a stand-in for
  ARM warnings): start → RUN → 50 rad/s ±5 %, estimate within 1 rad/s of the plant, set_speed 80, reverse start,
  direction fixed, stop → coasting with zero current, `-EBUSY` right after stop, restart, BRK pulse → FAULT/`-EIO`
  → stop clears → restart, BRK held at start → FAULT, offsets calibrated / `-ERANGE`, NaN/Inf → `-EINVAL`;
  `late == 0` throughout. native_sim runs in simulated time, so `k_msleep` waits are deterministic.
