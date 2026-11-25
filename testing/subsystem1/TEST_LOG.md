# Subsystem 1 — test log

**Subsystem:** Motion Control: motors, encoders, PID, odometry, x/y  ·  **Student ID:** 2500025  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Motor PWM and both encoders initialise at boot; the right-encoder PIO program sits at offset 0 on pio1 | FR7, FR16 (start-up), R7 | auto | TEST image; read the boot log and the test output | Boot log: `[motion] encoders: OK (right: PIO 24 instr. at offset 0 on pio1; left: interrupts on GP7/GP28)`. `[TEST] S1-U01 PASS` |
| S1-U02 | A stop bit in `flg_fault` cuts the motors in the next 10 ms cycle | NFR3, FR16, IF-08 | auto | TEST image, on a stand. The test sends FOLLOW SLOW, then raises MOTOR_STALL 50 ms later | `S1-U02a PASS` (FOLLOW drives the motors). `S1-U02b PASS` (both duties 0). `S1-U02c PASS` (cut latency ≤ 10 500 µs). Record the `S1-U02 cut latency N us` line |
| S1-U03 | The motion loop keeps its 10 ms period | NFR2, JL-1.21 | auto | TEST image; 1 s measuring window | `[TEST] S1-U03 PASS`: ≥ 90 runs in 1 s and jitter_max ≤ 500 µs. Save the `wcet:` table printed after it |
| S1-U04 | Left encoder (GPIO interrupt decoder): cost per edge and missed edges | FR7 (encoder feedback), R2 | auto + manual | TEST image. Spin the left wheel by hand during the 1 s window that starts after the `wcet:` table | `[TEST] S1-U04 PASS` (ISR max < 10 µs). In the line `S1-U04 left N counts/s, E errors`: N ≠ 0 and E = 0 |
| S1-U05 | Both wheels count UP when driving forward (`ENC_SIGN_L` −1, `ENC_SIGN_R` +1) | FR7, JL-1.3 | auto | TEST image, on a stand; FOLLOW SLOW for 400 ms | `S1-U05a PASS`, `S1-U05b PASS`. In the `S1-U05` line: odometer > 0 and left edge errors 0 |
| S1-U06 | Wheel direction and active brake | FR7, FR15 (start / stop), JL-1.2 | manual | 1. NORMAL image, on a stand, nothing within 50 cm in front. 2. Wait 15 s, press START. 3. Watch both wheels. 4. Within 2 s, press START again. 5. Try to turn each wheel by hand | Step 3: both wheels turn forward. Step 4: both stop within 0.2 s *(p)*. Step 5: the wheels resist turning (both bridge pins at 100 % = brake), they do not spin freely. Video |
| S1-U07 | Motor PWM is 20 kHz and the B pin stays low while going forward | FR7, JL-1.1 | manual | Scope on GP8 + GP9, then GP10 + GP11, during S1-U06 step 3 and after step 4 | Driving: GP8 / GP10 show PWM at 20 kHz ± 1 % *(p)*, and GP9 / GP11 are low. Braked: all four pins are high |
| S1-U08 | Odometer scale (`ENC_COUNTS_PER_REV` 2425, `WHEEL_DIAMETER_MM` 65). Barcode widths and every distance depend on it | FR4, FR13, JL-1.4 | manual | 1. NORMAL image, car in Idle beside a tape measure. 2. Note `odo` in the `[telem] st ... odo N` line (printed every 2 s). 3. Push the car straight 1000 mm by hand. 4. Read `odo` again. Do it 3 times | Each run: the odometer moves 1000 ± 20 mm *(p)* |
| S1-U09 | MC_DRIVE stops at the commanded distance (the 130 mm junction drive uses it) | FR5, IF-02, JL-1.16 | auto | Check not written yet. Add to `subsystem1_test.c`, on a stand: clear the trip bits, send `MC_DRIVE SPD_SLOW 130 mm`, poll `nav_read()` for up to 10 s | The odometer moves 130 to 140 mm *(p)*. `motion_state` returns to `MS_IDLE`. Both duties are 0 |
| S1-U10 | Encoder pivot angle (`TURN_TRACK_X10` 1208 was estimated by eye) | FR8, NFR4, JL-1.5 | manual | 1. NORMAL image, car on a smooth floor with ~25 cm free all round. 2. Mark the car's heading on the floor. 3. In Idle, hold MODE 2 s: Calibrate pivots the car 2 × 200° by itself. 4. Mark the final heading | Total turn 400 ± 10° *(p)*, i.e. within ± 2.5 % so a 90° pivot is within ± 2.3°, inside NFR4's ± 5° before any compass correction. If it is outside that, set `TURN_TRACK_X10` = 1208 × 400 / measured angle in `hw_alloc.h` and re-run |
| S1-U11 | Both wheels blocked raises MOTOR_STALL | FR16 (motor stall), JL-1.20 | manual | NORMAL image, on a stand, START, then hold both wheels still by hand | Within 1.0 s *(p)* of holding the wheels: `[sup] FAULT: MOTOR_STALL (0x...)`, red LED, 800 ms beep, motors stopped. A short MODE press returns to Idle (blue). FR16's 300 ms starts when the duty reaches 30 %, which the PI takes about 0.2 s to reach at cruise speed |
| S1-U12 | One silent wheel raises ENC_FAULT | FR16 (encoder fault), JL-1.20 | manual | As S1-U11, but hold only the left wheel. Repeat with only the right wheel | `[sup] FAULT: ENC_FAULT (0x...)`, red LED, motors stopped, for each wheel |
| S1-U13 | OBST_STOP holds the running command and resumes it when cleared | FR11, NFR3, IF-08 | auto | Check not written yet. In `subsystem1_test.c`: send FOLLOW SLOW, raise `FAULT_OBST_STOP`, wait 30 ms, check the duties; clear the bit, wait 30 ms, check again; send STOP | Duties are 0 while the bit is set and non-zero again within 20 ms *(p)* of clearing it |
| S1-U14 | A command sent while a trip bit is set is rejected and does not start later | FR16, IF-02, IF-03 | auto | Check not written yet. Raise `FAULT_ENC_FAULT`, send FOLLOW SLOW, wait 50 ms, check the duties. Clear the bit, wait 50 ms, check again | Duties are 0 in both checks |
| S1-U15 | OBST_STOP, then a trip bit, then both cleared, with no new command in between: the held command must not restart | FR16, IF-08 | auto | Check not written yet. Send FOLLOW SLOW, raise OBST_STOP, raise ENC_FAULT, clear both, wait 50 ms | Duties are 0. **Expect FAIL on `67cb365`**: when the trip arrives the state is already `MS_CUT`, so the first-cut block in `motion_task` is skipped and the held FOLLOW is not dropped. In normal use the Supervisor's MC_STOP on entering Fault overwrites it. The owner should decide whether to fix it or drop this row |
| S1-U16 | The wheel speed PI holds each speed level on the floor | FR7, JL-1.14 | manual | NORMAL image. START on a straight line with two marks 1000 mm apart. Time the car between the marks and read `v` in the `[telem]` lines | Cruise: 60 to 80 mm/s *(p)*; the target is `speed_pm` 86, about 72 mm/s, and 10 % lower while Line Following flags a curve. `v` stays within ± 10 mm/s *(p)* on the straight. Hump speed is checked in INT-12 |
| S1-U17 | Dead-reckoned x / y | FR13 (x/y on events), JL-1.18 | manual | MQTT image. 1. Power on with the car on a start mark. x/y start at 0 at power-on, with +x along the nose. 2. Push the car 1000 mm straight. 3. Send any payload to `mtk3smp-pico/snapshot`. 4. Turn the car 90° left on the spot by hand and push it 500 mm. 5. Request a snapshot again | Step 3: `x_mm` 1000 ± 30 and `y_mm` 0 ± 30 *(p)*. Step 5: `x_mm` about 1000 and `y_mm` +500 ± 50 *(p)* |
| S1-U18 | The GP19 probe shows the motion loop timing | NFR2, JL-1.21 | manual | Scope on GP19, TEST or NORMAL image | One pulse every 10 ms ± 0.5 ms. The pulse width is the loop's execution time (tens of µs) |
| S1-U19 | The post-mortem watch reports a clean boot and is armed | FR16 (safe stop), JL-2.7 | manual | Unplug USB and plug it back in (power-on reset). TEST image; read the first lines after the serial monitor connects | `[pm] reset cause: power-on` and `[pm] no post-mortem record from the previous run`. A later `[watch] ... armed 1` line |
| S1-U20 | The motion half of `nav_state` is published every cycle | NFR9, IF-07 | auto | Check not written yet. `nav_read()`, wait 30 ms, `nav_read()` again | `m.t_us` has changed |
| S1-U21 | The car drives straight on encoder feedback alone | FR7 | auto + manual | Check not written yet. Add to `subsystem1_test.c`: clear the trip bits, then `MC_DRIVE SPD_SLOW 1000 mm`. Run the TEST image with the car on the floor, nose on a 1 m tape line (the earlier S1 checks move it a few cm first). Measure how far the nose ends up from the tape. 3 runs | Nose within 30 mm *(p)* of the tape line after 1000 mm. Distance 1000 ± 20 mm *(p)* |
| S1-U22 | A failed start-up enters Fault | FR16 (failed start-up) | manual | Build NORMAL with `IMU_REQUIRED 1` in `subsystem3.h`, unplug the GY-511, flash. Restore `IMU_REQUIRED 0` afterwards | `[imu] GY-511 NOT found on I2C1 GP2/GP3 (no answer): INIT FAIL`, `INIT FAILED subsystem3_init (imu): ...`, `init FAILED -> Fault`, `[sup] 0 -> 9`, `[sup] FAULT: INIT_FAIL (0x...)`, red LED, START does nothing. Note: a short MODE press clears INIT_FAIL (it is in `FAULT_USER_CLEAR_MASK`); see open point 6 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
An **auto** row whose steps start "Laptop unit test" runs on a laptop with `make -C tests/host -f host_tests.mk CC=gcc`; one whose steps start "Check not written yet" needs its `TEST_CHECK` added first.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
