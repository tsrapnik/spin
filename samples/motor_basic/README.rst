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
     i_dq    0.000 0.000 A
     v_dq    0.00 8.87 V
     vbus    24.00 V, emf 8.87 V
     late    0
   uart:~$ motor stop

``motor start`` calibrates the current-sense offsets, then runs align, I/f
and closed loop; ``motor stop`` lets the motor coast and clears a fault.
