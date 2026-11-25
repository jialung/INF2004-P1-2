# Subsystem 4 — test log

**Subsystem:** Obstacle Scan: ultrasonic + scan servo  ·  **Student ID:** 2501142  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | ECHO edges reach the handler through `gpio_irq.c` | FR11, R2 | auto | TEST image; the sensor faces a wall about 30 cm away | `[TEST] S4-U01 PASS`. The `S4-U01 ping result 0, N cm` line shows N ≈ 30 |
| S4-U02 | The HC-SR04 answers | FR11, FR16 | auto | as S4-U01 | `[TEST] S4-U02 PASS`: the result is not US_NO_RISE |
| S4-U03 | GPIO handler time | — (design limit, R5) | auto | as S4-U01 | `[TEST] S4-U03 PASS`: max < 20 µs. **Known FAIL** on the car (up to 47 µs, `docs/HANDOFF.md` section 5); record the value |
| S4-U04 | ECHO at GP5 stays within 3.3 V | — (hardware safety) | manual | Do this before any other S4 test: scope GP5 (after the 1 k / 2 k divider) while the sensor pings | The high level is ≤ 3.3 V |
| S4-U05 | Distance accuracy | FR11, NFR5 | manual | MQTT image, Idle. A flat board square to the car at 150, 200, 300, 400 and 500 mm from the sensor. Send `esweep` at each distance and read `[obst] sweep start: ahead a b c mm` | At every distance the middle value b is within ± 10 mm *(p)* of the tape measure |
| S4-U06 | GUARD stops on 2 of 3 readings under 21 cm, and clears on 3 readings over 26 cm | FR11, IF-08 | auto | Check not written yet. Add to `subsystem4_test.c`: send `SCAN_GUARD`, sample `fault_peek()` every 10 ms, and print a line when OBST_STOP changes. The test prints prompts; the tester places a box at 15 cm, moves it to 23 cm, then removes it | Box at 15 cm: OBST_STOP set within 150 ms *(p)* (3 pings). Moved to 23 cm: stays set (hysteresis). Removed: cleared after 3 readings over 26 cm, about 150 ms |
| S4-U07 | One reading under 6 cm stops at once | FR11 | auto | Check not written yet. Same harness with `SCAN_EMERG_ONLY`: box at 15 cm, then at 4 cm | 15 cm: no stop. 4 cm: OBST_STOP within one ping (50 ms) |
| S4-U08 | "No echo" is not a fault | FR16 | auto | Check not written yet. Same harness, GUARD for 5 s with nothing within 4 m (point the sensor at open space) | US_FAULT never set and OBST_STOP never set |
| S4-U09 | 10 pings with no echo start raise US_FAULT and enter Fault | FR16 (ultrasonic fault), LK-1.7 | auto + manual | Check not written yet. (a) Same harness, GUARD for 1 s with the ECHO wire unplugged. (b) NORMAL image, on a stand with a sensor on black tape, START, then unplug the ECHO wire | (a) US_FAULT set after 10 pings (about 0.5 s). (b) `[sup] FAULT: US_FAULT (0x...)`, red LED, motors stopped, within 1 s *(p)* |
| S4-U10 | Servo straight ahead, ± 35° limits and smooth movement | FR11, LK-1.2, `SERVO_TRIM_US` 80 | manual | MQTT image, Idle, box at 40 cm; send `esweep` and film the servo | At rest the sensor points along the car's axis ± 2° *(p)*. The sweep reaches ± 35° without hitting an end stop and moves evenly with no jumps. The sensor stays on the horn |
| S4-U11 | Width sweep accuracy | NFR5, FR11 | manual | Idle; box face 40 cm from the sensor, centred, square to the car. Send `esweep` 5 times each with the 53 mm and the 96 mm box. Then once each with the 120 mm and 152 mm boxes | 53 mm and 96 mm: every `[sup] edge sweep: ... width W mm` within ± 15 mm of the real width (NFR5 covers 50 to 100 mm boxes). 120 and 152 mm are outside NFR5: record them (152 mm is known to read about 26 mm low) |
| S4-U12 | An edge out of the servo's view is flagged | FR11 | manual | Idle; send `esweep` with a 400 mm wide board at 40 cm | The `[obst] width sweep` line ends in `(left edge out of view)` and/or `(right edge out of view)`. `[sup] edge sweep` marks that edge `(out of view)` and prints width −1 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
An **auto** row whose steps start "Laptop unit test" runs on a laptop with `make -C tests/host -f host_tests.mk CC=gcc`; one whose steps start "Check not written yet" needs its `TEST_CHECK` added first.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
