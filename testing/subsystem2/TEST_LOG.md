# Subsystem 2 — test log

**Subsystem:** Line Following, Barcode Decode, LineSearch  ·  **Student ID:** 2503473  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | ADC0 and ADC1 readings are in range; raw values recorded on white and on black | FR1, FR4, RY-1.3 | auto | TEST image. Run it twice: all three sensors over white floor, then over black tape | `[TEST] S2-U01 PASS`. Record the `line L (ADC0) = .., line R (GP17) = .., barcode (ADC1) = ..` line from both runs |
| S2-U02 | The line error word is published and updates every 5 ms | FR1, IF-04 | auto | TEST image | `[TEST] S2-U02 PASS`: the seq changed within 20 ms and LE_VALID is set |
| S2-U03 | The line loop keeps its 5 ms period | NFR2 | auto | TEST image | `[TEST] S2-U03 PASS`: jitter_max ≤ 500 µs |
| S2-U04 | A burst of 40 droppable `EV_LINE_SAMPLE` events does not cost a must-keep `EV_BARCODE` | NFR11, IF-01, RY-1.26 | auto | TEST image | `[TEST] S2-U04 PASS`: `bus_stats.sup_lost_keep` is unchanged |
| S2-U05 | Sensor contrast in Idle | FR1, FR4, RY-1.3 | manual | NORMAL image, Idle. Read the `[line] idle:` line (printed every 1 s), 10 lines with each sensor over white, then 10 over black tape | Left (ADC0): white < 500 and black > 2500 *(p)*; the car read about 200 and 3300 to 3900 on 30 Oct. Right (GP17): 0 on white and 1 on black, with no flicker in 10 lines. Barcode (ADC1): white < 500 and black > 2500 *(p)*; about 200 and 3500 on 30 Oct. The `(black)` / `(white)` words match the surface |
| S2-U06 | Sensor output voltages are safe, and GP17 stays out of its undefined band | FR1 (right sensor digital), RY-1.3 | manual | Modules powered from 3.3 V. Meter on each AO pin with its sensor over white, then over black | Every AO ≤ 3.3 V. GP17 AO < 0.8 V on white and > 2.0 V on black. If not, change the sensor height |
| S2-U07 | IR calibration sets each threshold half-way between white and black | FR15 (Calibrate), IF-09, RY-1.4 | manual | NORMAL image. Car on the line with ~25 cm free all round. Hold MODE 2 s; the car spins by itself; wait for the beep | `[line] calibrate OK: left a..b -> thr t, barcode c..d -> thr u, right (GP17) black n of 3000`, with b − a ≥ 400 and t = a + (b − a) / 2. The next `[line] idle:` lines show the new `thr` and `bc thr` |
| S2-U08 | IR calibration that sees no black fails and keeps the old thresholds | FR15, RY-1.4 | manual | As S2-U07, on plain white floor far from any line. After it fails, press MODE short | `[line] calibrate FAILED: ... (kept, too little contrast)`, a 400 ms beep, and the LED stays yellow. After MODE short: Idle, and `[line] idle:` shows the old `thr` |
| S2-U09 | Line lost after 150 mm with both sensors white | FR2, RY-1.10 | manual | NORMAL image. Use a line that ends in open white floor. START at least 50 cm before the end. Mark where the sensors leave the line and where the LED turns purple | `[sup] 3 -> 7` (LineSearch) after 150 ± 20 mm *(p)* of travel past the end of the line |
| S2-U10 | Exactly one junction event per cross, and none from barcodes or tape edges | FR3, RY-1.9 | manual | MQTT image. Send the map `A=forward,B=forward,C=forward,D=forward` (with no default given, the default is forward). Drive a track with 5 crosses and at least one barcode. Count the console lines | Exactly 5 `[line] cross at odo N mm (left .., both black M mm)` lines, one at each cross, each with M ≥ 8. None at a barcode or tape edge |
| S2-U11 | Barcodes are read driving forward | FR4, RY-1.11 | manual | NORMAL image. For each course barcode (Code 39, on the car's left), drive over it 5 times at cruise speed | `[bc] read 'X' (forward)` with the printed letter, 5 out of 5 per barcode *(p)*. Each read is followed by `[sup] barcode 'X' stored -> ...` |
| S2-U12 | Barcodes are read in the reverse direction | FR4, RY-1.11 | manual | As S2-U11, with the barcode strip turned end for end (still on the car's left) | `[bc] read 'X' (reversed)` with the same letter, 5 out of 5 *(p)* |
| S2-U13 | A damaged or partial barcode is logged and counted, never read as a wrong letter, and the car keeps following | FR4, RY-1.11 | manual | Request a snapshot. Drive over a barcode with one bar covered, a barcode only half under the sensor, and a plain cross line. Request a snapshot again | Only `[bc] misread (error N)` or `[bc] dropped (...)` lines, or nothing for the cross line. Never a `[bc] read` with a wrong letter. No `[sup]` state change caused by them. The snapshot `misreads` value went up by one for each misread or dropped line |
| S2-U14 | The barcode task (the only ADC reader) runs every 1 ms | NFR2 | auto | Check not written yet. Add a `TEST_CHECK` on `wcet_tab[WCET_BARCODE]` after a 1 s wait | ≥ 900 runs in 1 s, jitter_max ≤ 500 µs *(p)*, and 0 overruns |
| S2-U15 | Code 39 decoder | FR4, NFR9, RY-1.11 | auto | Laptop unit test, no Pico: `make -C tests/host -f host_tests.mk CC=gcc` | No FAIL for the `C39 ...` checks: the A / M / W / Z / `*` patterns; all 26 letters forward and reversed with bar bias and jitter; too few elements; flat widths rejected; the window search with junk before and a cross line after |
| S2-U16 | Barcode edge detector, run on traces recorded on the car | FR4, NFR9, RY-1.11 | auto | Laptop unit test, no Pico: as S2-U15 | No FAIL for the `BC edge ...` checks at swing 700 and 400: narrow gaps split the bars (6 flips), line-edge drift gives no edges, a weak bar counts as one bar, paper noise gives no edge |
| S2-U17 | LineSearch logic | FR2, NFR9, IF-10 | auto | Laptop unit test, no Pico: as S2-U15 | No FAIL for the `LS ...` checks: sweeps toward the last-seen side first; ignores ABORTED replies and replies for other seq numbers; creeps 50 mm; fails after 300 mm and raises LINE_SEARCH_FAIL; found after 3 samples; noise rejected; the odometer going backwards does not count as distance |
| S2-U18 | A dark mark shorter than 8 mm of travel is not a cross | FR3 | manual | Lay a 3 mm wide black tape strip across the line. Drive over it 5 times | No `[line] cross` line. Each pass prints `[line] both black for only N mm at odo M: not a cross` with N < 8, or nothing if both sensors were black for under 10 ms. The car keeps following |
| S2-U19 | Two crossings closer together than 40 mm give one event | FR3 | manual | Two 18 mm cross lines with a 15 mm white gap between them, then a third cross 100 mm further on. Default mapping, no barcode. 5 passes | Each pass: one `[line] cross` line for the close pair, and one for the third cross |
| S2-U20 | Barcode reading holds when the speed changes | FR4 | manual | Read the same barcode: (a) on a straight at cruise speed; (b) in a curve, where the car runs 10 % slower and weaves; (c) placed 50 mm *(p)* ahead of the car's start position, so it is read while the car speeds up after START. 5 passes each | The same letter in all 15 passes |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
An **auto** row whose steps start "Laptop unit test" runs on a laptop with `make -C tests/host -f host_tests.mk CC=gcc`; one whose steps start "Check not written yet" needs its `TEST_CHECK` added first.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
