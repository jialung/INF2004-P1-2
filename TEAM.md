# Team and subsystems — INF2004 P1-2

Single place that says **who owns what**. Generated from the shared
skeleton (`docs/SKELETON.md`) and the Revised Task List v2. Fill in the
student IDs and GitHub usernames, and keep this file up to date.

Rule: **only the owner edits a subsystem's folder.** Shared files in
`app_program/` (no subfolder) change only by team agreement.

## 1. Members and subsystems

| # | Subsystem (what it does) | Owner | Student ID | GitHub | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Motion Control: motors, encoders, PID, odometry, x/y | Jia Lung | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Line Following, Barcode Decode, LineSearch | Ryan | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IMU: heading, tilt, humps, impact | Jian Yi | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Obstacle Scan: ultrasonic + scan servo | Lui Khee | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Supervisor, Telemetry, buttons, RGB LED, buzzer | Christian | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared modules (`app_program/`), owner in brackets:
`rtos_config.h`, `hw_alloc.h`, `hal.h` / `hal_rp2040.c`, `nav_state`, `wcet`, `app_main.c` (Jia Lung);
`fault_flags`, `msg_types.h`, `msg_bus`, `gpio_irq` (Christian).

## 2. Pins

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio (GP29 is also VSYS/3: do not use for R8) |
| GP0 / GP1 | Jia Lung | Right encoder A / B, PIO (Grove 1). No UART0 console: never build with `CRASH_REPORT=1` |
| GP2 / GP3 | Jian Yi | I2C1 SDA / SCL, GY-511 (Maker port) |
| GP4 | Lui Khee | HC-SR04 TRIG |
| GP5 | Lui Khee | HC-SR04 ECHO via 1 k / 2 k divider, edge interrupt |
| GP6 | — | free |
| GP7 | Jia Lung | Left encoder A, edge interrupt (Grove 7) |
| GP8 / GP9 | Jia Lung | Left motor M1A (forward) / M1B (reverse), PWM 4A / 4B |
| GP10 / GP11 | Jia Lung | Right motor M2A (forward) / M2B (reverse), PWM 5A / 5B |
| GP12, GP13, GP14 | — | free |
| GP15 | Lui Khee | SG90 scan servo, PWM 7B |
| GP16 | — | free |
| GP17 | Ryan | Right line sensor AO, read as digital (no ADC on this pin) |
| GP18 | Christian | WS2812 status LED, PIO |
| GP19 | Jia Lung | WCET / jitter scope probe (`wcet.h`) |
| GP20 | Christian | START button (interrupt) |
| GP21 | Christian | MODE button (interrupt) |
| GP22 | Christian | Buzzer, PWM 3A |
| GP26 | Ryan | Left line sensor AO, ADC0 (Grove 6) |
| GP27 | Ryan | Barcode sensor AO, ADC1 (Grove 6) |
| GP28 | Jia Lung | Left encoder B, edge interrupt (Grove 7), used as digital |

## 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| PWM slices 4, 5 | S1 | motion | GP8–GP11 |
| PIO1 SM0 (right encoder, offset 0) | S1 | motion | GP0 / GP1 |
| ADC0 + ADC1 (sole owner: kernel ADC driver OFF, patch P2) | S2 | line, barcode (later: DMA ring) | GP26, GP27 |
| DMA channel for the ADC ring, claimed | S2 | — | — |
| I2C1 | S3 | imu | GP2 / GP3 |
| PWM slice 7 | S4 | obstacle | GP15 |
| PIO1 SM2 (WS2812) | S5 | supervisor | GP18 |
| PWM slice 3 | S5 | supervisor | GP22 |
| IO_IRQ_BANK0 (GPIO interrupts) | shared | `gpio_irq.c` dispatcher | GP7 + GP28 (left encoder, S1), GP5 (S4), GP20 / GP21 (S5) |
| PIO0, 2 DMA channels | *kernel* | `cyw43_utk_task` | radio |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` | GP23–25, GP29 |

I2C0, SPI0/1, UART0, UART1, PWM slice 2: unused. ADC2 (GP28) is used as a digital input (left encoder B). IR modules: AO only, DO unconnected, powered from 3.3 V.

## 4. Task priorities

Team agreement: the plan's numeric priorities (section 1.1) replace the
template's default bands.

| Subsystem | Tasks (name – priority – core – period) |
|---|---|
| 1 | motion – 8 – core 1 – 10 ms cyclic |
| 2 | line – 9 – core 1 – 5 ms cyclic; barcode – 11 – core 1 – 10 ms cyclic |
| 3 | imu – 12 – core 0 – 20 ms cyclic |
| 4 | obstacle – 13 – core 0 – 50 ms + on request |
| 5 | supervisor – 10 – core 1 – event-driven (10 ms poll); telemetry – 15 – core 0 – 500 ms cyclic |

Kernel: initial task 1, USB console 3, CYW43 4 (both core 0).
Kernel tick: **1 ms** (`CNF_TIMER_PERIOD`, patch P1).

## 5. Interfaces between subsystems

| ID | From → To | Mechanism | Object | Data / meaning |
|---|---|---|---|---|
| IF-01 | all → S5 | message buffer, 16 × 16 B | `mbf_sup_event` | `SUP_EVENT` (`msg_types.h` §1); send with `sup_post()` |
| IF-02 | S5 → S1 | message buffer | `mbf_motion_cmd` | packed `MOTION_CMD` (§2): type, speed, ±2° angle, ±mm distance, seq |
| IF-03 | S1 → S5 | via IF-01 | `EV_MOTION_DONE` | seq + result (done / aborted / stalled / rejected) |
| IF-04 | S2 → S1 | latest-value word | `line_err_publish()` / `line_err_read()` | 8-bit error, seen flags, seq (§5) |
| IF-05 | S5 → S4 | message buffer | `mbf_scan_req` | packed `SCAN_REQ` (§3): mode decides who may stop |
| IF-06 | S5 → S5 telemetry | message buffer | `mbf_telem` | `TELEM_MSG` (§4) |
| IF-07 | S1, S3 → all | shared data under a mutex | `nav_read()` | `NAV_STATE`: S1 writes motion half, S3 writes IMU half |
| IF-08 | all → all | event flag | `flg_fault` | fault bits (`fault_flags.h`); Motion reads every 10 ms |
| IF-09 | S5 → S2, S3 | event flag | `flg_calib` | `CAL_REQ_IR`, `CAL_REQ_IMU`; reply `EV_CALIB_DONE` |
| IF-10 | S5 → S2 | function calls in the Supervisor task | `line_search_start/step()` | returns the next motion request |
| IF-11 | S5 → S5 (all, read) | atomic word | `sys_state_get()` | current `SYS_STATE` |

## 6. Kernel object budget

| Object | Limit | shared | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|---|
| Task | 32 | | 1 | 2 | 1 | 1 | 2 |
| Semaphore | 16 | | | | | 1 | |
| Event flag | 16 | 2 | | | | | |
| Mutex | 12 | 1 | | | | | |
| Message buffer | 12 | 4 | | | | | |
| Cyclic handler | 12 | | 1 | 2 | 1 | | 1 |
