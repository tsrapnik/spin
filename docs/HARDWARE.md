# Hardware setup (v1 bring-up)

Reference hardware for v1 bring-up: which parts to use, the board modifications they need, the jumper settings and the pin map.
Pin facts below were checked against the X-NUCLEO-IHM16M1 user manual (UM2415) and the Zephyr
board / `hal_stm32` pinctrl files for `nucleo_c542rc`. Items marked **⚠ unverified** still need a
check on the real hardware.

## Bill of materials

| Part | Role | Source | Notes |
|---|---|---|---|
| NUCLEO-C542RC (MB2213) | Control board, STM32C542RC | Mouser | Same PCB and headers as NUCLEO-C562RE, but only **ADC1**. Fine for v1, which uses only TIM1 + ADC1 |
| X-NUCLEO-IHM16M1 | Power stage: STSPIN830, 3 low-side shunts, TSV994 op-amps | Mouser | 7–45 V, 1.5 A rms. Morpho-only connection (CN7/CN10) |
| DFRobot FIT1036 (4015 gimbal) | PMSM, 24N22P, AS5600 encoder | Mouser `426-FIT1036` | Sold as "BLDC", but it is a concentrated-winding gimbal outrunner and is meant for FOC (see Motor) |
| Bench PSU | DC bus | Already owned | 30 V / 5 A, adjustable, with current limit |

Alternative control board: NUCLEO-C5A3ZG (Nucleo-144) connected with jumper wires. See the appendix.

## Motor: DFRobot FIT1036

| Parameter | Value | Source |
|---|---|---|
| Supply | 12–36 V (24 V rated) | DFRobot |
| Rated current | 0.65 A @ 12 V, 1.4 A @ 24 V; max 4 A | DFRobot wiki |
| Speed | 1100 rpm @ 24 V | DFRobot |
| Max torque | 3500 g·cm ≈ 0.34 Nm (≈ 15 N at the 22.5 mm rim) | DFRobot |
| Pole pairs | 11 (24 slots, 22 poles) | DFRobot wiki |
| Resistance | Wiki gives "winding 4.8 Ω" and "phase 7.2 Ω". **⚠ Ambiguous**: measure line-to-line and halve it for `resistance-micro-ohms` | DFRobot wiki |
| Inductance | **⚠ Unknown**: measure (LCR meter at ~1 kHz, line-to-line / 2) | — |
| Flux linkage | **⚠ Estimate ≈ 11 mWb** (assuming 1100 rpm is no-load at 24 V with SVPWM): ψ ≈ (24/√3) / (1100·2π/60·11). Measure the back-EMF on a scope by spinning the motor by hand | estimate |
| Kt | **⚠ Estimate ≈ 0.18–0.2 Nm/A** (1.5·p·ψ) | estimate |
| Encoder | AS5600, 12-bit absolute, I²C (optional; v1 is sensorless) | DFRobot |

Is it a PMSM? Probably yes. Check by spinning the rotor by hand and scoping one line-to-line voltage: a sinusoid means
it behaves as a PMSM. A non-ideal back-EMF only adds torque ripple under FOC.

## Power supply settings

- Set the PSU to **24 V** (or 12 V for the first runs), with the **current limit at about 1.5 A**.
- Never use 30 V without a working current loop. Locked-rotor current is 24 V / ~7 Ω ≈ 3.4 A, which is above the
  IHM16M1's 1.5 A rms rating.
- Set the firmware `max-current-milliamps` to **≤ 1400**.
- Connect the motor supply to the IHM16M1 CN1 screw terminal (1 = GND, 2 = VIN) or to the J4 jack. The Nucleo is powered from
  ST-LINK USB.

## IHM16M1 configuration

### Jumpers: FOC, 3-shunt (board default)

| Jumper | Setting | Meaning |
|---|---|---|
| JP4, JP7 (bottom solder bridges) | **Open** | Three-shunt topology |
| J5, J6 | **Closed** | Op-amps: gain 1.53, offset 1.56 V (FOC) |
| J2 | **2-3** | STSPIN830 PWM current limiter disabled (SNS to GND) |
| J3 | 1-2 (fixed 0.497 V) or 2-3 (MCU REF via PB4 PWM) | Only relevant if J2 = 1-2 |

### Resistor modifications (needed for NUCLEO-C542RC)

| Resistor | Action | Reason |
|---|---|---|
| **R34** | **Remove** | It routes phase W to CN10-15 = PA7. On the C542, PA7 and PB1 (phase V) are both ADC1_IN7, so they would couple. Phase W still reaches PB0 through R36 |
| **R35** | **Remove** | It routes EN/FAULT to CN10-14 = PA11, which is also the user USB-C D−. EN/FAULT still reaches PB12 (TIM1_BKIN) through R37 |
| R42 | Remove **if** CN10-24 is not a free pin (see below) | It routes phase V to CN10-24. Phase V also reaches PB1 through R41 on CN10-18 |

**⚠ Unverified:** the Zephyr morpho map for MB2213 does not define **CN10-1** and **CN10-24**, and UM3615
could not be retrieved. Before stacking the boards, use a multimeter to check that neither pin has continuity to GND, 3V3
or 5V on the bare Nucleo.
- CN10-24: the phase V op-amp output drives it. If the pin is not free, remove R42.
- CN10-1: the BEMF-divider GPIO (directly connected, no resistor). If the pin is not free, do not stack the boards: cut or lift the pin, or wire the boards instead.

## Pin map: NUCLEO-C542RC + IHM16M1 (stacked)

### Used by v1

| Signal | IHM16M1 | MCU pin | Peripheral | Notes |
|---|---|---|---|---|
| INU / INV / INW | CN10-23 / 21 / 33 | PA8 / PA9 / PA10 | TIM1_CH1 / CH2 / CH3 | High-side PWM (MODE = L: INx selects HS/LS) |
| ENU / ENV / ENW | CN10-30 / 28 / 26 | PB13 / PB14 / PB15 | GPIO out | Half-bridge enables. PB14/PB15 also appear on CN10-35/37 and Arduino D1/D0 (USART1): do not use those |
| EN/FAULT | CN10-16 | PB12 | TIM1_BKIN | Open-drain fault from the STSPIN830, LED D1 red when low. Pull-up |
| STBY (active low) | CN10-29 | PC1 | GPIO out | Drive high to run. On older Nucleo-64 boards this pin was PB5 |
| I_U | CN7-30 | PA1 | ADC1_IN1 | 1.56 V offset, gain 1.53 |
| I_V | CN10-18 | PB1 | ADC1_IN7 | Through R41 |
| I_W | CN7-34 | PB0 | ADC1_IN6 | Through R36 |
| VBUS | CN7-28 | PA0 | ADC1_IN0 | Divider on the IHM16M1 (**⚠ ratio to read from the UM2415 schematic**) |

Only ADC1 is available, so the two phase currents per cycle are sampled sequentially, with about 1 µs skew. This matches the
"ADC1 only" decision in `PLAN.md`.

### Optional / later

| Signal | IHM16M1 | MCU pin | Peripheral |
|---|---|---|---|
| Speed trimmer R17 | CN7-35 | PC2 | ADC1_IN10 |
| PCB NTC | CN10-34 | PC4 | ADC1_IN2. Shared with user-USB VBUS detection |
| Current-limit REF (PWM) | CN10-27 | PB4 | TIM15_CH1 / LPTIM1_CH2 |
| BEMF U/V/W dividers | CN7-36 / 37 / 38 | PC5 / PC3 / PC0 | ADC (6-step only). PC3 = ADC1_IN11 |
| Hall / encoder J1 (H1/H2/H3) | CN7-17 / CN10-31 / CN10-25 | PA15 / PB3 / PB10 | TIM2_CH1/2/3. PA15/PB3 are shared with SWD (SB14/SB16/SB18). FIT1036 has no Halls |
| AS5600 encoder (not via IHM16M1) | — | PB6 SCL / PB7 SDA | I2C1 (Arduino D15/D14, morpho CN10-3/5, unused by IHM16M1). Power from 3V3 |

### Duplicated nets: configure as input / analog, never output

The IHM16M1 routes some signals to two morpho positions. On MB2213 the second position lands on these pins:

| MCU pin | Net |
|---|---|
| PB5 | VBUS (via CN10-6, R31). No ADC on PB5 |
| PC7 | Hall 1 (CN10-4, R81) |
| PC6 | Hall 2 (CN10-19, R85) |
| PC8 | Hall 3 (CN10-2, R87) |

## Bring-up checklist

1. Fit the IHM16M1 resistor mods (R34, R35, and R42 if needed). Measure CN10-1 and CN10-24 on the bare Nucleo.
2. Check the jumpers against the FOC 3-shunt table.
3. With no motor and the PSU off, flash the firmware and scope PA8–PA10: frequency, centre alignment, dead-time (PLAN M5.1).
4. With the PSU at 12 V / 0.3 A and no motor, check that the ADC offsets read ≈ 1.56 V and that the VBUS reading is plausible.
5. Connect the motor to CN1 pins 3-4-5 and set the PSU to 12 V / 1 A. Run align at low current, then I/f, then closed loop (M5.4).
6. Go to 24 V / 1.5 A only after the current loop and the OC fault are verified.

## Appendix: NUCLEO-C5A3ZG (Nucleo-144) with jumper wires

Wire from the IHM16M1 header tails to the C5A3ZG morpho pins. Only the listed nets are connected, so none of the
resistor mods are needed. Keep the wires ≤ 10–15 cm and run a GND wire next to the current-sense lines.

| Signal | IHM16M1 | C5A3ZG pin | Peripheral |
|---|---|---|---|
| INU / INV / INW | CN10-23 / 21 / 33 | PA8 / PA9 / PA10 | TIM1_CH1/2/3 |
| ENU / ENV / ENW | CN10-30 / 28 / 26 | PB13 / PB14 / PB15 | GPIO |
| EN/FAULT | CN10-16 | PB12 | TIM1_BKIN |
| STBY | CN10-29 | any GPIO (e.g. PA0) | GPIO |
| I_U / I_V / I_W | CN7-30 / CN10-18 / CN7-34 | PH5 / PB1 / PB0 | ADC2_IN13 / ADC3_IN1 / ADC2_IN6 |
| VBUS | CN7-28 | PH4 | ADC2_IN12 |
| VDD (pull-up rail) | CN7-12 | 3V3 | — |
| GND | CN7-20, CN7-22 | GND | — |

According to Zephyr, the C5A3ZG morpho positions 1–38 seem to follow the Nucleo-64 layout. Direct stacking might
therefore work, but it is **⚠ mechanically unverified** (alignment, clearance of the Zio headers).

## References

- UM2415, X-NUCLEO-IHM16M1 user manual (incl. schematic)
- UM3615, STM32C5 Nucleo-64 (MB2213) user manual
- UM3616, STM32C5 Nucleo-144 (MB2310) user manual
- Zephyr `boards/st/nucleo_c542rc`, `nucleo_c5a3zg`; `hal_stm32` `dts/st/c5/*-pinctrl.dtsi`
- DFRobot FIT1036 wiki: https://wiki.dfrobot.com/fit1036/
