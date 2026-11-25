# Subsystem 5 — test log

**Subsystem:** Supervisor, Telemetry, buttons, RGB LED, buzzer  ·  **Student ID:** 2502365  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | The WS2812 PIO program is loaded on pio1 after the encoders | FR15 (LED), R7 | auto | TEST image | Boot log: `WS2812 PIO load: OK`. `[TEST] S5-U01 PASS` |
| S5-U02 | The GPIO interrupt is owned by core 0 | NFR9, R2 | auto | TEST image | `[TEST] S5-U02 PASS`. Boot log: `init OK, gpio irq on core 0` |
| S5-U03 | A trip bit puts the Supervisor in Fault | FR16, IF-08 | auto | TEST image | `S5-U03a PASS`. The car stays in Fault afterwards, by design. S5-U10 covers clearing it |
| S5-U04 | Telemetry frame round trip and CRC-8 | FR13 | auto | TEST image | `S5-U04a PASS` (round trip) and `S5-U04b PASS` (CRC rejects a flipped bit) |
| S5-U05 | LED colour per state | FR15 | manual | Take the car through each state and film the LED | Idle blue, Calibrate yellow, LineFollow green, Junction white, Hump cyan, Obstacle orange, LineSearch purple, Fault red. (Finished, pink, cannot be reached yet: SYS-20) |
| S5-U06 | START toggles between Idle and running | FR15 | manual | NORMAL image, on a stand. Press START, then again within 2 s | `[sup] 1 -> 3`, then `[sup] 3 -> 1`; one transition per press |
| S5-U07 | Button debounce (30 ms) | FR15 | manual | Press and release START 10 times, quickly but deliberately | Exactly 10 transitions, alternating `1 -> 3` and `3 -> 1`, with no extra ones |
| S5-U08 | A 2 s MODE hold enters Calibrate, and only from Idle | FR15 | manual | In Idle, hold MODE for 1.5 s and release. Then hold it for 2.5 s. Repeat the 2.5 s hold in LineFollow | 1.5 s: no change. 2.5 s: `[sup] 1 -> 2` (yellow) about 2.0 s after the press. In LineFollow: no change |
| S5-U09 | A short MODE press aborts Calibrate | FR15 | manual | Start Calibrate, press MODE short after about 5 s | `[sup] 2 -> 1`, `[line] calibrate aborted`, `[imu] calibrate aborted`. The old thresholds are kept |
| S5-U10 | Entering Fault, and clearing it with a short MODE press | FR15, FR16 | manual | Cause a trip (as in S1-U11 or S3-U16), then press MODE short. Repeat with the cause still present (IMU still unplugged, as in S3-U09) | Entry: `[sup] N -> 9`, `[sup] FAULT: NAME (0x..)`, red LED, 800 ms beep, motors stopped. Clear: `[sup] 9 -> 1`, blue. Cause still present: a 200 ms beep if trip bits remain, or straight back to Fault at the next failed read |
| S5-U11 | A Calibrate that never finishes stays in Calibrate and re-beeps every 20 s | FR15, `CALIBRATE_TIMEOUT_MS` | manual | Calibrate on plain white floor with the car on a stand; wait 45 s | 400 ms beeps at about 15 s (IR failed), 20 s and 40 s. The state stays 2 (yellow) |
| S5-U12 | The buzzer beeps on events and faults, and switches off by itself | FR15 | manual | Scope on GP22 while the car enters Fault; listen during a calibration and a mapping change | Fault: 2 kHz, 50 % duty, for 800 ± 20 ms *(p)*, then 0, and the next `[diag]` line shows `buzzer 0`. Calibrate done: 100 ms. New mapping: 100 ms |
| S5-U13 | Losing Wi-Fi is not a fault, and MQTT reconnects by itself | NFR8, FR16 | manual | MQTT image, line following on the track. Switch the hotspot off for 30 s, then back on | While it is down: `[telem] ... flt 0x080 ...` and the car keeps following, with no Fault and no stop. After the link returns, with nobody touching the car: `flt 0x000` and `mqtt: connected to ...` |
| S5-U14 | MQTT heartbeat | NFR8 | manual | `mosquitto_sub -h localhost -t "mtk3smp-pico/#" -v` for 60 s | `mtk3smp-pico/heartbeat hb N` every 5 ± 0.5 s *(p)*, with N going up by 1 each time |
| S5-U15 | Status frame: 14 bytes, CRC-8, every 500 ms, each sent within 100 ms of being made | FR13, NFR7 | manual | Record `mtk3smp-pico/telem` with timestamps for 60 s (see "Timing a reply over MQTT"). Request one snapshot during the recording | 120 ± 1 frames in 60 s, each 14 bytes, every CRC valid. Every gap between frames is 400 to 600 ms (500 ms ± the 100 ms publish delay). Decoded fields (state, faults, odometer, speed, heading, terrain, speed level, roll, pitch, hump count, last letter) agree with the snapshot taken at the same time |
| S5-U16 | Mapping over MQTT: a good payload replaces the table, a bad one changes nothing | FR6 | manual | Publish to `mtk3smp-pico/map` (retained, QoS 1): (1) `A=left,B=right,C=uturn,default=forward`; (2) `A=right,B=fwd` (first pair valid, second bad); (3) a 150-byte payload. Request a snapshot after each | (1) `map/ack`: `ok 4: ...` (the `default=` pair counts), console `[sup] new mapping: ...`, 100 ms beep, snapshot `map` shows `A=left B=right C=uturn`; D is now forward because the table is replaced as a whole. (2) `error -2: unknown word ...`, and the snapshot still shows `A=left`. (3) `error: mapping longer than 127 bytes`, mapping unchanged |
| S5-U17 | Remote command rules | FR15 (PC start / stop), FR12 (depth) | manual | Publish each of these to `mtk3smp-pico/cmd` (not retained) and read `cmd/ack`: `start` in Idle; `start` in Fault; `calibrate` in LineFollow; `mode` in Idle; `depth 5`; `depth 140`; `wcal 300 300`; `hello` | In order: `ok start (car was Idle)`; `refused start: car is in Fault, start only from Idle (clear the fault with MODE)`; `refused calibrate: car is in LineFollow, ...`; `refused mode: ...`; `error: depth needs a whole number of mm, 10 to 300 ...`; `ok depth 140 mm ...` with a 150 ms beep; `error: wcal needs ...`; `error: unknown command ...` |
| S5-U18 | Snapshot reply: content and time | FR14, NFR7 | manual | Publish any payload to `mtk3smp-pico/snapshot` 5 times in Idle and 5 times while driving, recording with timestamps | Every reply arrives on `snapshot/reply` within 500 ms of its request. Each is valid JSON with: state, faults, odo_mm, speed_mm_s, x_mm, y_mm, heading_deg, roll_q, pitch_q (tilt), humps, hump_peak_mm, barcodes, misreads, last_letter, pending, obstacle_width_mm, bypass_side, depth_mm, map |
| S5-U19 | `gpio_irq.c` refuses pins not listed in `hw_alloc.h`, and duplicate pins | NFR9, R2 | auto | Check not written yet. In `subsystem5_test.c`: call `gpio_irq_register(6, ...)`, then register GP20 again | The first call returns E_PAR and prints `gpio_irq: GP6 is not in hw_alloc.h`. The second returns E_OBJ |
| S5-U20 | The web panel's `--local-only` option | — (security, README "Run") | manual | Run `python tools/mqtt/web_panel.py --local-only`. Open the panel on this PC, then from a phone on the hotspot. Then run it again without the option and try the phone again | With the option: the panel works on this PC, and the phone cannot connect. Without it: the phone can open the panel |
| S5-U21 | Message packing uses only the bits each field needs | NFR10, NFR9 | auto + manual | Laptop unit test, no Pico: Host suite as S5-U22. Then read `msg_types.h` and `telem_frame.h` | No FAIL for `MC-...`, `PROF`, `PROF_B`, `LE`, `HUMP`, `BC`, `SCAN`, `SUP_EVENT 16 B`, `TELEM_MSG 16 B`, `states fit 4 bits`. Widths in the source: state 4 b (`SYS_STATE_BITS`), letter 5 b (`BC_A`), junction type 2 b (`JCT_TYPE`, 4 values), hump height 5 b (`HUMP_A`), heading 9 b (frame bytes 7 to 8), all packed with shift and mask macros |
| S5-U22 | The Supervisor queue reserve rule (`msg_bus.c`) | NFR11, NFR9 | auto | Laptop unit test, no Pico: `make -C tests/host -f host_tests.mk CC=gcc` | No FAIL for `reserve kept` (droppable events stop with 6 of 16 slots free), `keep fits`, `no keep lost`, `queue full`, `loss counted` |
| S5-U23 | Fault flag rules | FR16, NFR9 | auto | Laptop unit test, no Pico: as S5-U22 | No FAIL for `raise`, `raise posts EV_FAULT`, `no repeat post`, `clear keeps others`, `stop mask`, `wifi is status only`, `pickup trips and stops`, `pickup name`, `all clear` |
| S5-U24 | Telemetry frame CRC-8 | FR13, NFR9 | auto | Laptop unit test, no Pico: as S5-U22 | No FAIL for `CRC-8/0x07 check value` (0xF4), `pack len`, `unpack`, `round trip`, `crc catches` |
| S5-U25 | Mapping parser | FR6, NFR9 | auto | Laptop unit test, no Pico: as S5-U22 | No FAIL for the `map ...` checks: pairs, JSON, word first, `default=` / `else=`, typo, no partner, empty, pack round trip, format |
| S5-U26 | Bypass geometry | FR12, NFR9 | auto | Laptop unit test, no Pico: as S5-U22 | No FAIL for `BG ...`, `SCAN side probe`, `SCAN_SIDE_PROBE = 6`, `SCAN last mode`, `SIDE round trip`, `EV_OBST_SIDE / NEAR` |
| S5-U27 | Obstacle width and bypass side choice | FR11, FR12, NFR9 | auto | Laptop unit test, no Pico: as S5-U22 | No FAIL for `OW ...`, `WCAL ...`, `REMOTE codes`, `WM ...`, `EV_OBST_AHEAD` |
| S5-U28 | The default mapping is used until one arrives | FR6 | manual | With the car off, clear any retained mapping: `mosquitto_pub -h localhost -t mtk3smp-pico/map -r -n`. MQTT image, power on, request a snapshot | Snapshot `"map":"B=left C=right D=uturn"` (A and every other letter forward) |
| S5-U29 | Junction, barcode and fault events survive a burst of line samples | NFR11 | auto | Check not written yet. In `subsystem5_test.c`: post 40 `EV_LINE_SAMPLE`, then `EV_JUNCTION`, `EV_BARCODE`, and `fault_raise(FAULT_IMPACT)` (which posts `EV_FAULT`); then clear FAULT_IMPACT | `bus_stats.sup_lost_keep` unchanged. Any dropped line samples show in `sup_dropped_low` |
| S5-U30 | Telemetry and commands keep flowing while snapshots are answered | FR14 | manual | MQTT image, Idle, recording `mtk3smp-pico/#` with timestamps. Send 20 snapshot requests 100 ms apart (`1..20 \| % { mosquitto_pub -t mtk3smp-pico/snapshot -m x; Start-Sleep -Milliseconds 100 }`), and during them `cmd start`, then `cmd stop` | 20 replies. `telem` frames keep arriving with no gap over 600 ms *(p)*. Both `cmd/ack` replies arrive and the car starts and stops |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
An **auto** row whose steps start "Laptop unit test" runs on a laptop with `make -C tests/host -f host_tests.mk CC=gcc`; one whose steps start "Check not written yet" needs its `TEST_CHECK` added first.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
