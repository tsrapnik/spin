# Feature backlog (not in v1)

Candidates that would improve the library. Move an item into `docs/PLAN.md` when scheduled.

## Deferred from v1 scope
- Auto-restart after fault (configurable retries/back-off)
- Stall / observer-loss detection
- Smooth angle-error blend from forced angle to observer (v1: hard switch)
- Runtime parameter override (API + shell `motor params`)
- Gain/parameter persistence via Zephyr settings
- Fault callback in the public API
- Multi-motor validation (two instances on one MCU, ISR budget)

## Control & estimation
- Measure phase voltages (resistor dividers) → better observer at low speed, dead-time-free voltage
- Closed-loop startup (HFI / signal injection, initial position detection, zero-speed operation)
- Current-loop decoupling feed-forward (−ωL·iq on d, +ω(L·id + ψ) on q): faster disturbance rejection at
  high speed; v1 relies on the PI integrators
- Half-period duty latch: TIM1 update at both peak and valley (RCR = 0), duty from ISR k latched at the
  next valley → loop delay 1.0·Ts instead of 1.5·Ts (~1.5× current bandwidth at equal phase margin).
  Needs: ISR + ADC done within Ts/2 every cycle (top-priority direct IRQ, overrun detection; a late
  ISR latches at the peak, moves the window off the sample and makes the delay jitter); observer duty
  history pairing each sample with ½ d_k−1 + ½ d_k; plant with half-period steps; delay reported by the
  power-stage driver (`get_info`) instead of the fixed `FOC_ADVANCE_PERIODS`. See `docs/img/timing-half.svg`.
  Decide after M5.2/M5.5
- Dead-time & switch voltage-drop compensation
- Flying start / catch spinning motor (observer pre-sync with PWM off)
- Motor parameter identification (Rs, Ls, Ld/Lq, flux, inertia) + auto-tuning
- IPMSM: MTPA, Ld≠Lq observer, field weakening
- Overmodulation, 6-step at high speed
- Adaptive observer gains / online Rs estimation (temperature)
- Torque mode, position control layer, Zephyr `sensor`-based encoder/Hall fallback
- Regenerative braking / DC-bus over-voltage chopper control

## Hardware
- Single-shunt / 2-shunt topologies (mind patents)
- Dual-ADC simultaneous sampling of 2 phase currents (C562+): no skew, shorter window → higher max duty
- 3rd ADC (C5A3) for 3rd-current sum check / offset diagnostics or phase-voltage sampling
- Further power-stage drivers (other STM32 families, nRF, NXP, …)
- Fixed-point core for MCUs without FPU
- CORDIC sin/cos hook (STM32G4/H7/U5/C5). On a float core it saves maybe ~20–30 cycles per call
  (q1.31 conversion via `vcvt #31`, AHB write/read) against ~50 for `foc_sincos`, i.e. < 1 % of the
  20 kHz budget. It is also a shared peripheral (ISR-exclusive or locked) and not portable. Worth
  it for a fixed-point core or an MCU without FPU

## Tooling
- More static checks: CodeChecker/clang-tidy and sparse (`ZEPHYR_SCA_VARIANT`), MISRA C:2012 checking
  of the Zephyr guideline subset (ECLAIR/Coverity, commercial). Only if safety-style compliance is wanted
- Real-time data logging (ring buffer → RTT/shell), scope/tuning host tool
- Non-ideal inverter in sim (dead-time, switch drops, ADC quantisation/noise)
