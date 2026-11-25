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
| DT params | Integers with explicit units (`resistance-micro-ohms`, `inductance-nano-henries`, `flux-linkage-micro-webers`, `pole-pairs`, `max-current-milliamps`, bandwidths in Hz…), converted to float once at init |
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
include/pmsm/foc.h              # pure-C core API (no Zephyr deps)
include/pmsm/motor.h            # public device API
include/pmsm/power_stage.h      # driver class API
lib/foc/                        # transforms.c svpwm.c pi.c gains.c observer.c pll.c foc.c
subsys/motor/                   # motor.c (device, ISR glue, thread, faults), motor_shell.c
drivers/power_stage/            # power_stage_sim.c, power_stage_stm32_tim.c
dts/bindings/                   # pmsm,motor.yaml, pmsm,sim-power-stage.yaml, st,stm32-tim-power-stage.yaml
tests/lib/foc/                  # unit tests
tests/sim/                      # plant model + closed-loop scenarios (pure C)
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
- [ ] M1.1 Clarke/Park/inverse Park, fast sin/cos
- [ ] M1.2 SVPWM (min/max injection), voltage circle limit
- [ ] M1.3 PI with output limits + anti-windup
- [ ] M1.4 Gain calculation from motor params
- [ ] M1.5 ztest vectors for all of the above

### M2 Plant + closed-loop sim (ideal angle)
- [ ] M2.1 Discrete PMSM model (dq: Rs, Ls, flux, pole pairs, J, B, load torque) + ideal inverter (duty → voltage) with the duty delay
- [ ] M2.2 FOC current loop + speed loop with true angle; CI tests: current step response, speed step

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
- [ ] M6.1 README + integration guide (binding, params, bring-up procedure)

## Verification

- `west twister -T tests -p native_sim` runs unit + sim + device tests (CI-ready).
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
