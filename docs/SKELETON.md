# P1-2 shared skeleton

This is the starting code for the robot car: the shared layer everyone
codes against, plus a first version of each subsystem with the hardware
init, task loop and interfaces in place. Plan IDs (`JL-1.14`, `RY-1.9`, ...)
in `TODO` comments say what each owner fills in next.

Status (23 Oct 2026, **pin revision 2**: encoders on Grove 1 / Grove 7,
IR sensors on GP26 / GP27 / GP17 with AO only, see section 9):

- Builds with **zero warnings** in three profiles (SMP / single core,
  TEST), and with `-Wall -Wextra -Wshadow -Wconversion`. `CRASH_REPORT=1`
  is now refused on purpose: GP0/GP1 are the right encoder.
- **557 host unit checks pass** (`tests/host`).
- Boots and runs in an RP2040 emulator. The on-target tests pass, except
  the ones that need real sensors. See section 7 for what that does and
  does not prove.
- **Not yet run on the car.** Section 8 is the bench bring-up order.

---

## 1. Build and flash

This folder is the course template
([sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate),
commit `c3b0954`) with our code on top, still uncommitted: `git status` shows
exactly what the team changed. Patches P1 and P2 are already applied.

1. Set up the toolchain as in the template's README section 1: Arm GCC
   (13.2.1 tested), Pico SDK **2.2.0** with
   `git submodule update --init lib/tinyusb lib/cyw43-driver`, and
   `PICO_SDK_PATH`. For MQTT also run `git submodule update --init lib/lwip`.
   On Windows use WSL or MSYS2 (`make` is needed).
2. Build from `build_make/`:
   ```
   make -j8                       # normal firmware  -> mtk3pico_smp1_usb_cdc_wifi.uf2
   make -j8 TEST=1                # on-target tests  -> ..._test.uf2
   make -j8 NET=lwip MQTT=1       # with MQTT: first copy config/wifi_secrets.example.h
                                  # to config/wifi_secrets.h and fill it in (git-ignored)
   ```
   All of these, plus `SMP=0`, build with zero warnings.
   The console is USB only: `CRASH_REPORT=1` and `CONSOLE=uart` would put
   UART0 on GP0/GP1, which are now the right encoder, so `app.h` refuses them.
   `WIFI=none` is refused too (the SDK include paths come with the radio).
3. Flash: hold BOOTSEL, plug in, copy the `.uf2` to RPI-RP2. Serial monitor
   at 115200.
4. Host tests (any laptop C compiler, no Pico needed): `make -C tests/host -f host_tests.mk`.

---

## 2. Five rules everyone must know

**R-1. Two sides, never mixed.** µT-Kernel's `tk/syslib.h` defines
`size_t` as `long`, and the SDK defines it as `unsigned int`. Both also
define `PWM_BASE`, `TIMER_BASE` and so on. A `.c` file therefore includes
**either** `app.h` (kernel side: tasks, `tk_*`) **or** `hardware/...`
(SDK side: files named `hal_*.c` / `drv_*.c`), never both. They talk
through headers that use only `<stdint.h>` types (`hal.h`, `drv_*.h`).
`app.h` stops the build with a clear message if you get this wrong.

**R-2. GPIO interrupts only through `gpio_irq.c`.** In this port the
kernel owns the vector table. The SDK's `gpio_set_irq_enabled_with_callback()`,
`gpio_add_raw_irq_handler()` and `irq_*` calls are **do-nothing stubs**
(`lib/libwifi/.../cyw43_utk_compat.c`). They compile, link, and then never
fire. Use `gpio_irq_register()`. Only the pins listed in `hw_alloc.h` are
allowed: GP5 (echo), GP7 + GP28 (left encoder), GP20 / GP21 (buttons). This answers CH-1.8 and makes R5 worse than the plan assumed.

**R-3. Our PIO programs live on pio1; CYW43 keeps pio0.** CYW43 claims
"the first PIO with room", trying pio0 first, from a kernel task that may
run before or after `usermain()`. The measured program sizes are:
encoder 24 instructions (must sit at offset 0), WS2812 4, CYW43 6.
24 + 4 + 6 = 34 > 32, so all three can never share one block. With ours
on pio1, the result is the same whichever runs first (R7 closed).
Only the **right** encoder uses PIO now: the program reads pin and pin + 1,
and GP7 / GP28 are neighbours on the Grove 7 plug but not consecutive GPIO
numbers. The left encoder is decoded by interrupts (`enc_irq.c`).

**R-4. Never block inside a control loop.** Motion Control is the safety
path. Anything that can wait forever (a blocking FIFO read, `TMO_FEVR`)
can stop the fault cut. The pico-examples encoder helper uses
`pio_sm_get_blocking()`. Our driver does not: the emulator showed that it
can hold the CPU and starve every lower-priority task.

**R-5. lwIP / MQTT only inside `cyw43_utk_task`.** The stack runs with
`NO_SYS=1` in the template's Wi-Fi service task (core 0, priority 4).
Telemetry leaves its frame in `telem_outbox`, and a hook in that task
publishes it (patch P5). The template's own TEAM.md says the same and asks
for instructor agreement first.

---

## 3. File map

| File | Side | Owner | What |
|---|---|---|---|
| `app_main.c` | K | Jia Lung | `usermain()`: shared objects → `subsystemN_init()` (S1 first) → start; init failure → `FAULT_INIT_FAIL` |
| `app.h` | K | team | template API + `app_start_periodic()` (task + cyclic), `MEM_BARRIER()`, `LOG()` |
| `rtos_config.h` | K | Jia Lung | priorities, periods, cores, stack; `#error` if the tick is too coarse |
| `hw_alloc.h` | both | Jia Lung | every pin, PWM slice, PIO SM, DMA rule, IRQ user; robot geometry |
| `hal.h`, `hal_rp2040.c` | S | Jia Lung | time, GPIO, GPIO-IRQ registers, PWM (with reset release) |
| `fault_flags.h/.c` | K | Christian | the fault flag: `fault_raise / clear / peek` |
| `msg_types.h` | K | Christian | every packed message: `SUP_EVENT`, `MOTION_CMD`, `SCAN_REQ`, `TELEM_MSG`, line error word |
| `msg_bus.h/.c` | K | Christian | creates the channels; `sup_post()` keeps a reserve for must-keep events |
| `nav_state.h/.c` | K | Jia Lung / Jian Yi | one writer per half, mutex with priority inheritance |
| `gpio_irq.h/.c` | K | Christian | the one IO_IRQ_BANK0 handler |
| `wcet.h/.c` | K | Jia Lung | exec time, release jitter, overruns; GP19 scope probe |
| `diag.h/.c` | K | Jia Lung | task-state dump, `[diag]` counter line, core-0 watcher run from core 1 (section 10) |
| `diag_pm.c` | K | Jia Lung | post-mortem watch: 1 ms alarm per core, trips on a stuck loop / dead core / HardFault / buzzer on 3 s; motors + buzzer off, record in SRAM4, watchdog reboot (section 10) |
| `subsystem1/` | | Jia Lung | `task_motion.c`, `drv_motor`, `encoders.h`: right = `drv_encoder` (PIO, + `.pio` / `.pio.h`), left = `enc_irq.c` (interrupts) |
| `subsystem2/` | | Ryan | `task_line.c` (left analog + right digital), `task_barcode.c`, `line_search.c`, `drv_adc` (ADC0 + ADC1) |
| `subsystem3/` | | Jian Yi | `task_imu.c`, `drv_imu` (stub) |
| `subsystem4/` | | Lui Khee | `task_obstacle.c`, `us_echo.c`, `drv_servo` |
| `subsystem5/` | | Christian | `supervisor.c`, `task_telemetry.c`, `telem_frame`, `drv_ui` (+ `ws2812.pio`) |
| `tests/host/` | host | all | laptop unit tests with kernel shims |

K = kernel side, S = SDK side (rule R-1).

---

## 4. What works now, and what each owner does next

| Owner | Working in the skeleton | Next (plan IDs) |
|---|---|---|
| **Jia Lung** | 20 kHz motor PWM (xA = forward, xB = reverse, as wired); right encoder PIO + non-blocking read; left encoder x4 interrupt decoder; fault cut every cycle; command handling (newest wins, held while OBST_STOP, rejected on trip); 64-bit odometry; stall / encoder-fault detection; open-loop line steering; distance drive; encoder pivots | measure counts/rev, wheel, track, W (JL-1.4/1.5) into `hw_alloc.h`; check encoder signs (JL-1.3), the brake behaviour (JL-1.2) and S1-U04 (left edges/s, errors); per-wheel PID (JL-1.14); x/y (JL-1.18); compass check after turns |
| **Ryan** | line sensors: left on ADC0 with a threshold, right on GP17 as digital (3-level error); ADC0 / ADC1 polled reads; line error word at 5 ms; line-lost after 30 mm; LineSearch sample tick; calibrate handshake; LineSearch first cut (creep, confirm, fail after S) | **ADC0 / ADC1 DMA ring first** (RY-1.5..1.7): barcodes cannot be decoded without it; GP17 black level and the 0.8 V / 2.0 V check (RY-1.3); thresholds (RY-1.4); spacing vs line width (RY-1.2); barcode decode (RY-1.11); junctions (RY-1.9); sweeps + β check (RY-1.15) |
| **Jian Yi** | task, failure counting → IMU_SENSOR, hump enter/exit statechart, height estimate, calibrate handshake | pick the I2C route (`drv_imu.h`, P3 warning); driver (JY-1.3..1.6); roll/pitch/heading (JY-1.7/1.8); impact (JY-1.12); roll **or** pitch (JY-1.18) |
| **Lui Khee** | 50 Hz servo; TRIG + ECHO by edge interrupt; GUARD 2-of-3 / emergency stop with clear hysteresis; US_FAULT; mode changes release OBST_STOP | PROFILE scan (LK-1.11) filling `PROF_A` + `PROF_B`; PASS modes (LK-1.14); check the servo end stops |
| **Christian** | statechart (all 10 states); START / STOP / MODE short + long with debounce; Calibrate with timeout; Fault and clear; obstacle stop-and-wait; hump slow-down; letter → junction turn; LineSearch hooks; LED colours; beeps; telemetry frame + CRC-8; WIFI_DOWN | MQTT publish hook (P5, CH-1.11); mapping payload + swap (CH-1.13/1.14); default mapping table; snapshot (CH-1.15); bypass planning (CH-1.21) |

---

## 5. Template patches

| ID | File | Change | Why | Required? |
|---|---|---|---|---|
| **P1** | `config/config.h` | `CNF_TIMER_PERIOD 10` → `1` | A cyclic handler cannot beat the tick: a 10 ms tick makes the 5 ms Line Following period impossible. The port allows 1–50. `rtos_config.h` refuses to build without it. | **yes** (`patches/P1_tick_1ms.patch`) |
| **P2** | `config/config_device.h` | `DEVCNF_USE_ADC 1` → `0` | Otherwise the kernel's sample ADC driver also owns the ADC and its interrupt. Ryan's driver (and later the DMA ring) must be the only owner. | **yes** (`patches/P2_kernel_adc_off.patch`) |
| P3 | `device/i2c/.../i2c_cnf_sysdep.h`, `kernel/sysdepend/pico_rp2040/devinit.c`, `config_device.h` | I2C1 pins → GP2/GP3; init unit 1, not 0; `DEVCNF_USE_IIC 1` | **Hazard as shipped:** I2C0 defaults to GP8/GP9 (left motor), I2C1 to GP6/GP7 (GP7 = left encoder A), and devinit starts only unit 0. Needed only if Jian Yi uses the kernel I2C driver. | only for option (b) in `drv_imu.h` |
| P5 | `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | in the poll loop, when the link is up: if `telem_outbox_seq` changed, `mqtt_publish()` the frame | lwIP may only be called from this task (R-5). Agree with the instructor first. | for MQTT (CH-1.11) |

Keep `USE_PTMR 0` in `config.h`: the kernel's physical timer would take a PWM slice.

---

## 6. Changes from Task List v2 (for the Tribunal change log)

| # | Change | Plan text | Now | Evidence |
|---|---|---|---|---|
| 1 | Kernel tick | (not mentioned) | 1 ms (P1) | port's `CNF_TIMER_PERIOD 10`; `MIN_TIMER_PERIOD 1` |
| 2 | PIO blocks | pio0 = encoders + WS2812, pio1 = CYW43 | pio1 = encoders + WS2812, pio0 = CYW43 | sizes 24/4/6 from pioasm; CYW43 claim order in `cyw43_bus_pio_spi.c` |
| 3 | GPIO interrupts | dispatcher or `gpio_add_raw_irq_handler()` | dispatcher via `tk_def_int` only; SDK calls are stubs | `cyw43_utk_compat.c` |
| 4 | Header split | (not mentioned) | rule R-1 | compile error on `size_t` |
| 5 | Cores | (not set) | Motion, Line, Barcode, Supervisor on core 1; IMU, Obstacle, Telemetry on core 0 | USB (3) and CYW43 (4) preempt everything on core 0 |
| 6 | D1 motion command | 7-bit angle, 5° steps | 8-bit angle, 2° steps; 11-bit signed distance; TURN_ABS; 4-bit seq | 3° heading error not expressible in 5° steps; "drive d", "reverse 10 cm" need distance |
| 7 | `mbf_line_err` | queue every 5 ms | latest-value word with seq | 5 ms producer / 10 ms consumer only fills a queue |
| 8 | Supervisor wait | `tk_wai_flg` + `mbf_sup_event` | waits on the message buffer only (10 ms timeout); `fault_raise()` also posts EV_FAULT | a task blocks on one object at a time |
| 9 | OBST_STOP clearer | Obstacle Scan / Supervisor | Obstacle Scan only; set only in GUARD / EMERG_ONLY; released on a mode change | otherwise the bypass cannot start while the box is ahead |
| 10 | Scan request | (no mode field) | `SCAN_MODE`: GUARD, INHIBIT, PROFILE, PASS_L/R, EMERG_ONLY | in-place turns are D_EMERG-only (CH-1.18); PassAlong needs its own side limit |
| 11 | New fault bit | 8 bits | bit 8 INIT_FAIL; telemetry fault field 16 bits | Design Review §5 "driver init fails → Fault" |
| 12 | State field | 3 bits | 4 bits (10 leaf states); frame header = type 3, version 1, state 4 | CH-1.17 lists 10 states |
| 13 | Profile event | one 32-bit word | + word b: closest distance, line-to-edge left / right | FR10 closest point; offset must not assume a centred box |
| 14 | Distances | D_CLEAR 10 cm | D_CLEAR_HYST 35 cm (clear), D_STOP_MIN 10 cm (FR9 target) | one name meant two things |
| 15 | LineSearch tick | (none) | Line Following posts EV_LINE_SAMPLE every 5 ms in LineSearch | the event-driven Supervisor needs something to drive `line_search_step()` |
| 16 | Event drops | "never drop junction / barcode" | droppable events need ≥ 6 free slots; losses counted in `bus_stats` | a FIFO cannot drop selectively |
| 17 | Encoder read | pico-examples helper | non-blocking | R-4; emulator run |
| 18 | ADC owner | (not mentioned) | kernel ADC driver off (P2) | single-owner rule |
| 19 | Pin revision 2 | encoders GP16/17 + GP6/7 (breakouts), IR on ADC0-2 | right enc GP0/1 (PIO), left enc GP7/GP28 (interrupts), line L GP26 (ADC0), barcode GP27 (ADC1), line R GP17 (digital); IR DO unused | team rewiring 23 Oct; removes the Grove breakouts (R1) |
| 20 | Left encoder | PIO quadrature | GPIO interrupts + x4 table | GP7 / GP28 not consecutive |
| 21 | Line sensors | both analog, proportional error | left analog (ADC0), right AO on GP17 read as digital; 3-level error | only two ADC pins left; GP17 has no ADC |
| 22 | Barcode input | ADC2 | ADC1 (GP27), ring round-robin with ADC0 | GP28 is now the left encoder |
| 23 | Debug console | UART0 on GP0/GP1 (CRASH_REPORT) | USB CDC only | GP0/GP1 are the right encoder |
| 25 | Liveness watchdog (JL-2.7, early) | (planned for later) | `diag_pm.c`: Supervisor / Motion loop stuck 1.5 s, core silent 0.5 s, HardFault or buzzer on 3 s → outputs off, `[pm]` record, watchdog reboot (max 2 in a row) | console died with the buzzer on (23 Oct): the failure has to be recorded, not printed |
| 24 | Run-time diagnostics | (none) | `diag.c`: task dump on stall / core-0 starvation, kernel-lock and GPIO-IRQ counters, buzzer read-back; GPIO dispatcher switches off any unregistered pin that interrupts | bench run 23 Oct: early serial connect left core 0's tasks without CPU and the buzzer on |

---

## 7. What was verified, and how far it goes

**Builds:** `SMP=1` and `SMP=0`, with `TEST=1`. Zero warnings, including
the strict flag set. `CRASH_REPORT=1` fails with the intended message.

**Host tests** (`tests/host`, 557 checks): every angle and distance value
round-trips through `MOTION_CMD`; out-of-range values clamp instead of
flipping sign; all other packed fields; `SUP_EVENT` / `TELEM_MSG` are 16 B;
the reserve rule (droppable events stop at 6 free slots, must-keep events
still fit, losses are counted); fault set / clear semantics; CRC-8
check value `0xF4`; the frame round trip and corruption detection;
LineSearch creep → confirm → found, noise rejection, and fail after S.

**Emulator** ([rp2040js](https://github.com/wokwi/rp2040js) 1.1.1, single
core, `SMP=0 CRASH_REPORT=1`). These caveats matter:
- To get a console it used UART0, so the emulator copy moved the right
  encoder to GP12/GP13 (the real code keeps GP0/GP1).
- I patched the emulator locally: its SysTick could not wake a core from WFI.
- The radio start was removed (the unmodified template also stalls there
  in the emulator).
- For the obstacle run, stall detection was disabled (the emulated wheels
  never turn).
- Timing is approximate.
Treat it as a logic check, **not** as Tribunal evidence.

What it showed:
- Boot: encoders load at offset 0 on pio1, WS2812 after them, GPIO IRQ
  on core 0, Supervisor Init → Idle.
- On-target suite: S1–S5 pass except IMU-present and HC-SR04-present.
  Motion loop 10 µs average, no overruns; fault-to-cut ≤ one 10 ms cycle.
- Buttons through the real dispatcher: START → LineFollow; stall → Fault;
  MODE short clears; MODE held → Calibrate at 2.0 s; Calibrate stays with
  a failed IMU (chain 4.1); MODE short aborts; a 4-edge 15 ms bounce →
  one action.
- Obstacle with a faked HC-SR04: box at 25 cm → OBST_STOP + ObstacleAvoid
  92 ms later → motors braked 5 ms after that; box removed → LineFollow
  137 ms later → motors resume.
- Two bugs found and fixed: the blocking encoder read (R-4), and US_FAULT's
  counter not resetting between runs.
- Pin revision 2: a quadrature waveform on GP7 / GP28 (500 steps forward,
  100 back) counted exactly +400 with 0 missed edges; 3000-edge bursts up
  to 100 000 edges/s still counted exactly; left-encoder callback ~3 µs,
  whole GPIO handler ≤ 5 µs; the 10 ms motion loop kept its timing
  throughout.
- Line sensors (left ADC0, right GP17): both black → wheels equal; left
  black only → left 2625 / right 4875 (turns left); right black only →
  the mirror; both white → error held, line-lost waits for 30 mm of travel.

---

## 8. First bring-up on the car (suggested order)

Wheels off the ground for 1–4.

1. Flash the `TEST=1` image, open the USB serial port, read the boot log:
   `encoders: OK`, `WS2812 PIO load: OK`, `init OK, gpio irq on core 0`.
2. **Jia Lung** — S1-U02: check the motors spin and stop. Check the
   direction of each wheel and the brake behaviour against `drv_motor.h`.
   Spin each wheel by hand: is the count sign right? Fix `ENC_SIGN_*`.
   S1-U04: spin the left wheel during its 1 s window; errors should be 0.
3. **Lui Khee** — S4-U01/U02 against a wall at 30 cm. Measure ECHO at GP5 ≤ 3.3 V first.
4. **Christian** — LED colour in Idle (blue), buttons, buzzer; Wi-Fi link in `[telem]`.
5. **Ryan** — S2-U01 on white and on black tape (RY-1.3): left ADC0,
   right GP17 level (`LINE_R_BLACK`), barcode ADC1. GP17 must read 0 on
   white and 1 on black; if it flickers, the AO sits between 0.8 V and
   2.0 V: change the sensor height. Power the IR modules from 3.3 V.
6. Normal image on the floor: START → line follow at SLOW speed first
   (change `speed_pm[SPD_CRUISE]` temporarily), box in front → stop.

---

## 9. Pin revision 2 (23 Oct 2026)

| Signal | Before | Now | Counted / read by |
|---|---|---|---|
| Right encoder A / B | GP6 / GP7 (breakouts) | **GP0 / GP1** (Grove 1) | PIO, pio1 SM0 |
| Left encoder A / B | GP16 / GP17 | **GP7 / GP28** (Grove 7) | GPIO interrupts, `enc_irq.c` |
| Motors | GP8–GP11 | unchanged; **A = forward, B = reverse** confirmed | PWM 4, 5 |
| Line sensor L | GP26 (ADC0) | **GP26 (ADC0)**, AO | `task_line.c`, threshold `thr_l` |
| Line sensor R | GP27 (ADC1) | **GP17**, AO into a non-ADC pin: digital | `task_line.c`, `LINE_R_BLACK` |
| Barcode | GP28 (ADC2) | **GP27 (ADC1)**, AO | `task_barcode.c` (needs the DMA ring) |
| IR DO outputs | — | **unconnected** | — |
| UART0 console | GP0 / GP1 (CRASH_REPORT) | **gone** (USB only) | — |
| Free now | GP12–14 | GP6, GP12, GP13, GP14, GP16 | — |

**GP17 caveat.** The RP2040 has no ADC on GP17. The pad reads the
sensor's analog voltage as 0 below 0.8 V and 1 above 2.0 V; in between it
is undefined (the Schmitt input stops it flickering, but not reliably).
There is no software threshold to calibrate, so check on the bench that
white is well below 0.8 V and black well above 2.0 V, and adjust the
sensor height if not. The IR modules must run from 3.3 V: a 5 V AO would
exceed the pin's maximum rating (this applies to GP26 / GP27 too).

---

## 10. Reading the diagnostics (added 23 Oct, change 24)

| Line | Printed by | Means |
|---|---|---|
| `[tasks] id N name pri P RUN` | `diag_task_dump()` | on a core right now |
| `... RDY` | | ready but **no core free**: something above it is hogging that core |
| `... WAI sleep / delay / mbf-recv / MUTEX ...` | | waiting; `MUTEX` or `mbf-SEND` held for long is a bug |
| `[diag] core 0: imu/obstacle/telemetry have not run for N ms` | Supervisor (core 1) via `diag_core0_watch()` | core 0 starved; the dump under it shows who has core 0 (the RUN task that is not the supervisor) |
| `[telem] this task ... got no CPU for N ms` | Telemetry | same, seen afterwards from core 0 |
| `[telem] WARNING ... STALLED` | Telemetry | a core-1 loop made no run in 200 ms of real time |
| `[diag] ... kernel lock max hold N us` | | longest time the kernel lock (interrupts off) was held |
| `[diag] gpio irq N spur S (pin P off M)` | | `P` = an unregistered pin that interrupted and was switched off |
| `[diag] ... buzzer L` | | GP22 PWM level now: 0 = silent, 500 = sounding |

**`[pm]` lines (post-mortem watch, `diag_pm.c`).** When a control loop
stops, a core stops taking interrupts, a HardFault happens, or the buzzer
stays on for 3 s, the watch switches the motors and buzzer off, prints
`[pm] watch tripped` (if any task can still print) and reboots 2.5 s later.
After the reboot, the test image prints `[pm] *** PREVIOUS RUN STOPPED: ...`
right after the serial monitor connects. The PCs in it are resolved
against the exact `.elf` of that build:
`arm-none-eabi-addr2line -f -e build_make/<image>.elf <pc>`.
Two reboots in a row at most; after that the car stays up with outputs off.

Wait about 15 s after power-on before pressing START: until the Wi-Fi
bring-up finishes, the CYW43 task (priority 4, core 0) appears to hold core 0
and the IMU / obstacle / telemetry tasks do not run.
