# Prebuilt images (23 Oct 2026, pin revision 2)

Built from exactly this folder (course template c3b0954 + our code, patches P1 + P2),
Pico SDK 2.2.0, Arm GCC 13.2.1. USB console, Wi-Fi radio on, no MQTT.

| File | Same as | What |
|---|---|---|
| `car_TEST_bench.uf2` | `make -j8 TEST=1` | dual core (default). On-target tests, then normal running. Prints `[diag]` counters every 10 s; `[tasks]` dumps when core 0 starves or a core-1 loop stalls (docs/SKELETON.md section 10) |
| `car_TEST_bench_1core.uf2` | `make -j8 SMP=0 TEST=1` | the same, **single core** (no cross-core interrupts) |
| `car_main.uf2` | `make -j8` | normal firmware, dual core |
| `car_main_1core.uf2` | `make -j8 SMP=0` | normal firmware, single core |

**Wheels off the ground** for the test images: S1-U02 and S1-U05 spin the motors.

Flash: hold BOOTSEL, plug in USB, copy the .uf2 to the RPI-RP2 drive.
Console: USB serial, 115200 8N1. The tests wait up to 30 s for the serial monitor.
No UART0 console (GP0/GP1 = right encoder).

**Post-mortem watch (all four images).** If a control loop stops, a core
hangs, a HardFault happens, or the buzzer stays on for 3 s, the car switches
the motors and buzzer off, prints a `[pm]` record if it still can, and
reboots 2.5 s later. After the reboot the record is printed again (test
images: right after the serial monitor connects, so reconnect it if it
dropped). Send the `[pm]` lines; they are decoded against the matching .elf.
