# Subsystem 3 — test log

**Subsystem:** IMU: heading, tilt, humps, impact  ·  **Student ID:** 2501080  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | The GY-511 is found on I2C1 (ID bytes `H43`, CTRL_REG1 reads back) | FR8, FR9, JY-1.3 | auto | TEST image | Boot log: `[imu] GY-511 (LSM303DLHC) found on I2C1 GP2/GP3`. `[TEST] S3-U01 PASS` |
| S3-U02 | The IMU half of `nav_state` updates (20 ms task) | NFR9, IF-07 | auto | TEST image | `[TEST] S3-U02 PASS` |
| S3-U03 | The IMU loop never overruns its 20 ms period | NFR2 | auto | TEST image | `[TEST] S3-U03 PASS`: 0 overruns |
| S3-U04 | Axis signs | FR8, FR9, JY-1.7 | manual | NORMAL image, Idle. Read `[imu] idle:` (printed every 2 s) in each pose | Flat: roll and pitch within ± 8 (2°) *(p)*. Left side lifted: roll +. Nose lifted: pitch +. Car turned clockwise seen from above: heading goes up |
| S3-U05 | Tilt angle accuracy | FR9, JY-1.7 | manual | Put a 10° wedge under the left side, then under the nose | roll 40 ± 8, then pitch 40 ± 8, in 0.25° units *(p)* |
| S3-U06 | Compass heading after calibration | FR8, NFR4, JY-1.8 | manual | After S3-U07, away from steel and motors, turn the car by hand to 4 floor marks 90° apart and read the heading in Idle | Each step reads 90 ± 5°. The idle line says `compass calibrated` |
| S3-U07 | IMU calibration: level offsets and compass | FR15 (Calibrate), IF-09, JY-1.13 | manual | The same run as S2-U07 | `[imu] calibrate done: level roll .. pitch .. (0.25 deg), lateral .. (Q14); compass OK (span f F, l L; offsets ...)`, with F ≥ 200 and L ≥ 200 |
| S3-U08 | Calibration without a full turn keeps the old compass values | FR15, JY-1.13 | manual | Calibrate with the car on a stand, so the spin cannot turn it. The IR part will also fail (no line); leave with MODE short | `[imu] calibrate done: ... compass KEPT OLD - turn the car through a full circle`. The IMU still reports done |
| S3-U09 | 3 failed IMU reads in a row raise IMU_SENSOR | FR16 (IMU read failure), JY-1.5 | manual | NORMAL image, on a stand, START, then unplug the GY-511 cable | Within about 0.1 s (3 reads at 20 ms): `[sup] FAULT: IMU_SENSOR (0x...)`, red LED, motors stopped. Plug it back in, MODE short: Idle |
| S3-U10 | The car runs without an IMU fitted at boot | FR16 (start-up), `IMU_REQUIRED` 0 | manual | Boot with the GY-511 unplugged | `[imu] GY-511 NOT found on I2C1 GP2/GP3 (no answer): running without IMU`, then `init OK`. LED blue, START works, no IMU_SENSOR fault. (S1-U22 covers `IMU_REQUIRED 1`) |
| S3-U11 | Right-wheel height in Idle (`HUMP_LEVER_MM` 97) | FR10 | manual | Idle, after calibration. Put a 10 mm spacer under the right wheel; the left wheel stays on the floor | `right wheel X mm` reads 10 ± 1.5 mm *(p)* with the spacer and 0 ± 1 mm *(p)* without it |
| S3-U12 | No false humps on a flat track | FR9 | manual | 3 m of flat line following (`HUMP_DEBUG` 1) | No `[imu] hump start` line. The max in each `[imu] hump watch` line stays at or below the trigger (55, i.e. 5.5 mm) |
| S3-U13 | A hump under the right wheel is detected | FR9 | manual | A line with the 10 mm practice hump under the right wheel's path; 10 passes (the same runs as S3-U14) | Each pass gives one `[imu] hump start: right wheel up ...` and then one `[imu] hump end: ...`, with no `TOO SHORT` marker |
| S3-U14 | Hump height, hump count and highest peak | FR10, NFR6 | manual | Measure the hump's true height. Request a snapshot. Make the 10 passes of S3-U13. Request a snapshot again | The `-> N mm` value in each `hump end` line is within ± 5 mm of the true height in at least 8 of the 10 passes (NFR6). The snapshot `humps` went up by 10, and `hump_peak_mm` equals the largest N (0 to 31) |
| S3-U15 | An impact raises IMPACT | FR16 (impact over 2 g for 2 samples), JY-1.12 | manual | NORMAL image, START on the floor, let the car run into a soft barrier at cruise speed | `[imu] IMPACT: N mg horizontal` with N > 2000, then Fault. No IMPACT during normal driving. If no run reaches 2 g, record the highest N seen |
| S3-U16 | Pickup by tilt raises PICKUP | FR16 (car lifted) | manual | START on the floor, pick the car up and hold it tilted more than 25° for more than 0.3 s | `[imu] PICKUP: tilted, roll .. pitch .. (0.25 deg, limit 100) ...`, `[sup] FAULT: PICKUP (0x...)`, motors stopped. MODE short clears it once the car is put down |
| S3-U17 | Pickup by a level lift raises PICKUP | FR16 (car lifted) | manual | START on the floor, lift the car straight up quickly while keeping it level | `[imu] PICKUP: lifted, ... mg (limit 1400)`. `PICKUP_LIFT_MG` 1400 is a placeholder: compare with the max acceleration in the `hump watch` lines. No PICKUP during normal driving or on humps |
| S3-U18 | IMU maths | FR8, FR9, NFR9 | auto | Laptop unit test, no Pico: `make -C tests/host -f host_tests.mk CC=gcc` | No FAIL for `atan2 ...`, `isqrt`, `level roll 0`, `roll 10 deg`, `heading all round`, `heading tilt-compensated`, `sin/cos` |
| S3-U19 | Impact is ignored for 300 ms after a hump | FR16 (impact hold-off) | manual | The 10 passes of S3-U13, at cruise speed | No IMPACT fault in any pass. The hold-off is only exercised if landing off the hump jolts the car: record the highest max acceleration in the `hump watch` lines |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
An **auto** row whose steps start "Laptop unit test" runs on a laptop with `make -C tests/host -f host_tests.mk CC=gcc`; one whose steps start "Check not written yet" needs its `TEST_CHECK` added first.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
