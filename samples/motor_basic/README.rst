.. SPDX-License-Identifier: MIT

Motor basic
###########

A ``pmsm,motor`` device on a power stage, commanded from the shell. On
``native_sim`` the power stage is ``pmsm,sim-power-stage`` (the plant model
of a DFRobot FIT1036, see ``docs/HARDWARE.md``); the nucleo_c542rc overlay
follows with the STM32 power stage (M5).

Build and run (from the west topdir):

.. code-block:: sh

   west build -b native_sim spin/samples/motor_basic -d <builddir>
   <builddir>/zephyr/zephyr.exe -uart_stdinout

Commands (speeds in mechanical rpm, the sign is the direction; the device
name is optional and defaults to the first motor):

.. code-block:: none

   uart:~$ motor start 500
   uart:~$ motor speed 700
   uart:~$ motor status
   motor: run
     speed   700.0 rpm (ref 700.0, target 700.0)
     i_dq    0.000 0.404 A
     v_dq    -0.49 10.32 V
     vbus    24.00 V, emf 8.87 V
     late    0
   uart:~$ motor stop

``motor start`` calibrates the current-sense offsets, then runs align, I/f
and closed loop; ``motor stop`` lets the motor coast and clears a fault.

On native_sim, ``sim`` acts on the simulated motor (1 mNm per rad/s viscous
friction from the overlay):

.. code-block:: none

   uart:~$ sim load 100       # 100 mNm load step: iq rises, speed dips and recovers
   uart:~$ sim show           # true speed, current, torque and loads
   uart:~$ sim friction 20    # Coulomb friction, mNm
   uart:~$ sim vbus 8000      # bus to 8 V: under-voltage fault ("motor stop" clears it)

At 24 V the bridge runs out of voltage at about 840 rpm with this friction:
higher targets settle there (``v_dq`` at about 12.5 V).
