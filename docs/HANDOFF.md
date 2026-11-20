# Robot car (INF2004 P1-2): handoff

State of the project as of 30 Oct 2026. Read this before changing code or flashing the car. Everything in "Verified on the car" was seen in a serial log. Everything else is written and builds, but has **not** run on the car yet.

---

## 1. Hardware and pins (as wired)

| Device | Pins | Notes |
|---|---|---|
| Left motor M1 | GP8 (M1A, forward), GP9 (M1B) | PWM slice 4, 20 kHz |
| Right motor M2 | GP10 (M2A, forward), GP11 (M2B) | PWM slice 5 |
| Left encoder | A = GP7, B = GP28 | GPIO edge interrupts (pins not consecutive), count sign −1 |
| Right encoder | A = GP0, B = GP1 | PIO quadrature on pio1 SM0; UART0 is therefore unusable |
| Left line sensor | GP26 = ADC0 (analog) | |
| Right line sensor | GP17 (analog output read as digital) | not calibratable; 3.3 V supply |
| Barcode sensor | GP27 = ADC1 | about 20 mm left of the line |
| IMU GY-511 (LSM303DLHC) | SDA GP2, SCL GP3 (I2C1, 400 kHz) | register-level driver, kernel I2C driver off |
| Ultrasonic HC-SR04 | TRIG GP4, ECHO GP5 (1 k / 2 k divider) | |
| Scan servo SG90 | GP15 | limited to ±25° from straight ahead |
| Status RGB LED | GP18 (WS2812) | colour per state, see section 6 |
| Buttons | START GP20, MODE GP21 | active low |
| Buzzer | GP22 | |

All pin numbers are in `app_program/hw_alloc.h`. **The design review's pin table is out of date** (it lists GP16/17 and GP6/7 for encoders, and GP28 for the barcode sensor).

Line geometry: one solid line, 18 mm wide. The two line sensors are 24 mm apart and **straddle** the line (both read white when centred). The only junction type is a cross (+). Barcodes are on the left of the car, about 141 mm long.

---

## 2. Build and flash

Tools on this PC: `C:\Users\luikh\.pico-sdk\toolchain\15_2_Rel1\bin`, `C:\msys64\usr\bin` (make), Pico SDK `C:/Users/luikh/.pico-sdk/sdk/2.3.1`.

```powershell
cd build_make
$env:PATH = "C:\Users\luikh\.pico-sdk\toolchain\15_2_Rel1\bin;C:\msys64\usr\bin;" + $env:PATH
$env:PICO_SDK_PATH = "C:/Users/luikh/.pico-sdk/sdk/2.3.1"
make -j8                    # normal       -> mtk3pico_smp1_usb_cdc_wifi.uf2
make -j8 TEST=1             # self tests   -> ..._test.uf2
make -j8 NET=lwip MQTT=1    # Wi-Fi + MQTT + Telnet -> ..._lwip_mqtt.uf2
```

Laptop unit tests (no Pico needed): `make -C tests/host -f host_tests.mk CC=gcc` from the project root (with `C:\msys64\ucrt64\bin` on PATH). Last result: **595 passed, 0 failed**.

Flash: hold BOOTSEL, plug in USB, copy the `.uf2` from **`build_make\`** to the RPI-RP2 drive. Do **not** use the old images in `firmware\` (23 Oct); they predate every change below.

Which image to use:

| Image | Use |
|---|---|
| `..._test.uf2` | Self tests run at boot. One test raises a fault on purpose, so the car ends in Fault (red); status lines stop printing after 30 s. Not for driving. |
| `mtk3pico_smp1_usb_cdc_wifi.uf2` | Normal driving, USB console only. |
| `..._lwip_mqtt.uf2` | Normal driving plus Wi-Fi: MQTT, mapping over MQTT, Telnet console. Wi-Fi settings are compiled in (section 8). |

---

## 3. Changes made in this session

### Fixes
- **Microsecond timer ran ~43× slow** (`kernel/sysdepend/pico_rp2040/hw_setting.c`). The kernel's own clock start-up never set the RP2040 watchdog tick, so `now_us()` was wrong. Effects before the fix: ultrasonic read 0–1 cm (false obstacle stops), buzzer stuck on, debounce 1.3 s, Wi-Fi busy-waits starving core 0. Fixed with `WATCHDOG TICK = ENABLE | 12`.
- Line-lost and line-search distances now handle the odometer going backwards (pushing the car by hand caused an instant `LINE_SEARCH_FAIL`).
- Line search and the Supervisor now only react to replies to **their own** motion commands (the replaced FOLLOW's ABORTED reply used to advance them).
- A new motion command cancels any command held during an obstacle stop.
- Host test `memcmp` on uninitialised padding fixed.

### Features
| Feature | Where |
|---|---|
| Wheel speed PI (feed-forward + P + position-lag I, anti-windup), slow-down near the end of drives and turns | `subsystem1/task_motion.c` |
| Speeds 3.5× slower than the earlier hand-set values: `{0, 31, 43, 86}`, turn 71, steering 6/7 | `task_motion.c` |
| Dead-reckoned x/y position | `task_motion.c` |
| Straddle line following (`LINE_STRADDLE 1`), line lost after 150 mm both-white | `subsystem2/task_line.c` |
| Cross-junction detection (both sensors black for 10 ms, re-arm after 40 mm) | `task_line.c` |
| Line search: sweep ±20° towards the last-seen side, then creep 50 mm, up to 300 mm | `subsystem2/line_search.c` |
| IR calibration (15 s window) | `task_line.c` |
| Barcode: ADC sampled every 1 ms by the barcode task (the only ADC reader); Code 39 decoder, A–Z, both directions | `subsystem2/task_barcode.c`, `barcode_decode.c/.h` |
| Letter held until the next cross; at the cross: drive `SENSOR_TO_AXLE_MM`, then pivot 90 / −90 / 180°; compass check ±5° with one correction | `subsystem5/supervisor.c` |
| Letter → movement mapping over MQTT (not hard-coded); default A forward, B left, C right, D U-turn until one arrives | `subsystem5/map_parse.c/.h`, `supervisor.c`, `mqtt_utk.c` |
| IMU driver, tilt-compensated compass, roll / pitch, hump detection and peak height, impact fault, IMU calibration | `subsystem3/drv_imu.c`, `imu_math.c/.h`, `task_imu.c` |
| Obstacle side probe 0–30° in 5° steps, pass guard at 30°, smooth servo ramp (servo limit ±35° since 13 Nov, used only by the width sweep) | `subsystem4/task_obstacle.c` |
| **Obstacle width** (12 Nov): two-stage stop. At 40 cm (`D_WIDTH_CM`) the car stops once, the servo sweeps (face straight ahead, then each side in 5° looks to the first miss and 1° looks to pin the boundary, about 5 s since 13 Nov: servo signal 100 Hz (`SERVO_PWM_HZ`, was 50), moves 1° per 10 ms frame and waits 150 ms before each ping), the width is computed and sent, then the car follows the line again and stops at 21 cm for the bypass. The window shows the finished width only. Replaces the old profile scan and the old `2 × near edge` estimate. Since 20 Nov the two edges also choose the bypass side (next row). | `subsystem4/task_obstacle.c` (`width_sweep`, `EV_OBST_AHEAD`), `subsystem4/us_echo.c` (`us_ping_mm`), `subsystem5/obst_width.c/.h`, `supervisor.c` (`width_result`), `task_telemetry.c`, `tools/mqtt/web_panel.py` |
| **Bypass side** (20 Nov): from the width sweep, round the side with more room. The edge that reaches more than `OW_SIDE_MARGIN_MM` (20) further from the line than the other loses; an edge out of view counts as reaching past the sweep. Centred, both edges out of view, no sweep for this obstacle, or the sweep more than `WM_SIDE_VALID_MM` (400 mm) before the stop: `BYPASS_SIDE_DEFAULT` (LEFT). The log shows `[sup] bypass side RIGHT: the box reaches further left (left edge .. mm, right edge .. mm)` and the reason in the `[sup] bypass RIGHT: stopped at ...` line. Replaces `avoid left/right` | `obst_width.c` (`ow_bypass_side`), `supervisor.c` (`width_result`, `enter`), `tools/mqtt/web_panel.py` |
| Width calibration, per side (13 Nov): `wcal <left mm> <right mm>` in Idle adds one run; each side's S and K are fitted once its edge positions are 40 mm apart, and printed to hard-code. `esweep` sweeps and prints both edges without storing a run. `wcal clear` ("Clear runs") starts again. The **Edge calibration** window draws each sweep and runs these commands | `supervisor.c`, `obst_width.c` (`ow_fit_side`), `tools/mqtt/edge_cal.py` |
| Servo straight-ahead trim `SERVO_TRIM_US` (80) and end calibration `SERVO_END_LEFT_US` / `SERVO_END_RIGHT_US` (−50 / −70), fixed in `drv_servo.h` (the `servo` / `servoend` commands and the raw scan were removed 20 Nov); servo limit ±35° | `subsystem4/drv_servo.c/.h`, `task_obstacle.c`, `subsystem4.h` |
| Obstacle bypass state machine: 50° side-step bypass (stop 15 cm from the car's front, two out-steps with a side scan after each, pass, turn back 50° and drive onto the line), side from the width sweep (row above), box depth from the PC (`depth <mm>`), 60 s limit, `NO_BYPASS` fault | `supervisor.c`, `subsystem5/bypass_geom.c/.h`, `fault_flags.h/.c` |
| MQTT: telemetry frames, JSON events, snapshot reply, mapping with ack | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `subsystem5/task_telemetry.c` |
| Telnet console on port 23 (copy of the USB console) | `lib/liblwip/.../telnet_utk.c/.h`, `usb_console.c`, `cyw43_utk.c` |
| Timing diagnostics: longest / shortest release gap per task | `wcet.c/.h` |
| Idle readouts every 1–2 s: `[line] idle:` and `[imu] idle:` | `task_line.c`, `task_imu.c` |
| PC tools: mapping window, Mosquitto config | `tools/mqtt/` |

---

## 4. Verified on the car

- Both motors and both encoders count the right way (S1-U05); subsystem 1 tests 8/8.
- Timer fix: task periods correct, motion jitter about 0.2 ms; ultrasonic reads real distances (49–75 cm).
- Line following on a 60 cm straight at the slow speed; stops for a hand in front of the ultrasonic sensor and resumes.
- Sensors: left line sensor about 200 white / 3300–3900 black; right GP17 0/1; barcode sensor about 200 / 3500.
- IMU found; axis signs checked and corrected (nose up = pitch +, left side up = roll +, clockwise = heading up).
- Calibration succeeded: left thr 1785, barcode thr 1838, level and compass OK.

## 5. Not yet verified on the car
Junction detection and turns, barcode decoding on a real printed barcode, mapping over MQTT, the obstacle bypass, the obstacle width sweep while driving (bench calibration done 12 Nov: about ±15 mm on 53 / 96 mm boxes, a 152 mm box read 26 mm low; boxes wider than 96 mm are not in the fit yet), the compass correction after turns, hump height, impact, MQTT events / snapshot, Telnet console, mapping UI.

Known test failure: **S4-U03** (GPIO handler up to 47 µs, limit 20 µs). The encoder callback itself is 3 µs, and there are no edge errors; likely kernel-lock waits in the ECHO / button callbacks. Not blocking.

---

## 6. Operating the car

LED (GP18): blue Idle · yellow Calibrate · green Line follow · white Junction turn · cyan Hump · orange Obstacle · purple Line search · pink Finished · red Fault.

| Action | How |
|---|---|
| Drive / stop | START (GP20) short press in Idle / while driving |
| Calibrate | In Idle, hold MODE (GP21) 2 s → yellow. Keep still 1 s, then within ~14 s turn the car a full circle while sliding the front across the line. Ends by itself: beep + blue. Lost on power-off: redo each power-up. |
| Leave Calibrate after a failure / clear a Fault | MODE short press |

Console states in `[sup] A -> B`: 0 Init, 1 Idle, 2 Calibrate, 3 LineFollow, 4 Junction, 5 Hump, 6 Obstacle, 7 LineSearch, 8 Finished, 9 Fault.

Demo sequence: send the mapping from the PC → START → car reads a barcode (`[bc] read 'X'`, `[sup] barcode 'X' stored`) → at the next cross it performs the mapped move (`[sup] junction: letter X -> LEFT`) → repeats.

---

## 7. Placeholders still to measure

| Value | File | Now | How |
|---|---|---|---|
| `ENC_COUNTS_PER_REV`, `WHEEL_DIAMETER_MM` | `hw_alloc.h` | 1440, 65 | push the car exactly 1 m, read `odo` before / after |
| `TRACK_WIDTH_MM` | `hw_alloc.h` | **113, measured** | wheel centre to centre (affects all turn angles) |
| `CAR_WIDTH_MM` | `hw_alloc.h` | **180, measured** (150 body + 30 barcode sensor on the left) | bypass gap check; offset uses the half facing the obstacle (75 right / 105 left) |
| `SENSOR_TO_AXLE_MM` | `hw_alloc.h` | **130, measured** | line sensors to wheel axle (junction turns) |
| `SERVO_MIN_US` / `SERVO_MAX_US` | `subsystem4/drv_servo.h` | 500 / 2500 | check straight ahead is really ahead |
| `SERVO_TRIM_US` | `subsystem4/drv_servo.h` | **80, set on the car 20 Nov** (about 7°; was 95) | straight-ahead trim, added to every servo pulse. Re-measure and edit it here if the sensor or horn is remounted (the live `servo <us>` command was removed 20 Nov) |
| `SERVO_LEFT_SIGN` | `subsystem4/subsystem4.h` | +1 | coarse scan should start to the RIGHT |
| `IMPACT_MG` | `subsystem3/subsystem3.h` | 2000 | bench test |
| `PICKUP_LIFT_MG` | `subsystem3/subsystem3.h` | 1400 (placeholder) | drive the track with `HUMP_DEBUG 1`, read "max \|a\|" in the "hump watch" lines, set it well above the highest value; then lift the running car level and check it trips. 0 turns the lift test off (the 25° tilt test stays) |
| `BYPASS_MARGIN_MM` | `hw_alloc.h` | 50 | gap kept between the car and the obstacle |
| `BG_E_MAX_MM` | `subsystem5/bypass_geom.h` | 120 | largest obstacle edge from the line |
| `BG_DEPTH_DEFAULT_MM` | `subsystem5/bypass_geom.h` | 140 (the 14 cm deep practice box) | box depth the pass assumes after power-up; set it from the PC instead (`depth <mm>`, 10–300, "Box depth" in the control window). Shallower than the real box = the car's side comes closer to its back corner when it turns back to the line |
| `BG_STEP1_MAX_MM` | `subsystem5/bypass_geom.h` | 150 | do not raise without re-checking the corner-swing note above it in `bypass_geom.h` |
| `BG_FACE_TOL_MM` | `subsystem5/bypass_geom.h` | 50 | scan 1: how close a reading must be to the expected face |
| `DRIVE_BYPASS_PM` | `task_motion.c` | 55 (46 mm/s) | bypass straight legs only |
| `APPROACH_SLOW_MM`, `APPROACH_OVER_MM` | `subsystem5/subsystem5.h` | 60, 120 | raise `APPROACH_SLOW_MM` if the line is found in the fast part |
| `CPS_PER_PM`, `PI_KP` | `task_motion.c` | 10, 4 | lower `PI_KP` if the car surges |
| `D_DETECT_CM` | `subsystem4.h` | 21 | stop 15 cm from the car's front; adjust by 1 if the car stops outside 14–16 cm |
| `D_WIDTH_CM` | `subsystem4.h` | 40 | the car stops to measure the width when 2 of 3 readings are under this. It must leave a face distance of about 33 cm or more, so both edges of a 20 cm box (and ±2 cm off the line) are inside the servo's ±35° after the beam spread. Keep `D_CLEAR_HYST_CM` < it < `D_WIDTH_CLEAR_CM` |
| `OW_PIVOT_MM` | `subsystem5/obst_width.h` | **12, measured** | servo axis to the front of the transducer, mm |
| `OW_SL_E4`, `OW_KL_E4`, `OW_SR_E4`, `OW_KR_E4` | `subsystem5/obst_width.h` | **4992, 6079, 4760, 6079** (20 Nov one-point correction: 120 mm box centred at 394 mm, trim 80, ends −50 / −70; K kept, S moved so it reads 120 mm. Not a fit: other widths unchecked) | per-side width calibration (below). Redo as a real fit by shifting the box sideways |
| `OW_FACE_TOL_MM` | `subsystem5/obst_width.h` | 40 | a reading is the box face when its forward distance is this close to the straight-ahead one |
| `SCAN_SIDE_MAX_DEG` | `subsystem4.h` | **35** (13 Nov, was 30) | servo limit each side; only the width sweep goes past 30. `SCAN_PASS_DEG` (pass guard) and the bypass side probe stay at 30 |

### Calibrating the obstacle width
The ultrasonic beam is wide, so each side's boundary is far outside the real edge, and it moves less than the edge does. A single "beam angle" correction cannot fit boxes of different widths (12 Nov: the best value was 15.5 to 16.4° for a 96 mm box, 16.9 to 17.9° for a 53 mm box and 12.5° for a 152 mm box). The sensor mount and servo aren't symmetric, so **each side has its own two constants** (13 Nov):

`tan(boundary) = S + K × edge / F` (F = face reading + `OW_PIVOT_MM`; edge = that edge of the box face from the car's centre line, + = out on its side). Width = left edge + right edge.

Use **Edge calibration...** in the window (under Obstacle width). It draws every look of a sweep from above: green rays hit the box face, grey rays missed it. Dashed lines are each side's boundary, orange marks the edges the car worked out, and blue marks the edges you measured.

1. Set the servo trim first (`SERVO_TRIM_US`), so 0° is really straight ahead.
2. Car in Idle (blue). Press **Clear runs** (or power-cycle) to start a new calibration.
3. Put one box in front, flat face square to the car, **face about 40 cm from the sensor**. Press **Measure only**. Check that the green rays stop at the box's corners. If they reach far past the box, something beside or behind it is close enough to count as the face: clear about 25 cm either side. If the box's left side shows on the RIGHT of the drawing, `SERVO_LEFT_SIGN` is wrong.
4. Measure each edge of the box's front face from the car's centre line with a ruler. Enter them as Left edge / Right edge → **Add calibration run** (`wcal <left> <right>`). The log shows `[sup] width cal run N: face ... left ... deg (edge ... mm, reads ...), right ...`.
5. Shift the **same box** sideways and repeat, 2 runs per position, so each edge covers at least 40 mm (for example left edge 0, 60 and 120 mm). Once a side's edges are 40 mm apart, every run prints
   `[sup] width cal fit over N runs: set OW_SL_E4 a OW_KL_E4 b OW_SR_E4 c OW_KR_E4 d in obst_width.h (used now until power-off)` and what each run then reads. The window shows the four values in a box you can copy from.
6. Put the four values in `obst_width.h`, rebuild (`make -j8 NET=lwip MQTT=1`) and flash. Update the comment above them.

The car keeps the last 8 runs. Expect about ±15 mm: repeat runs with the same box differed by up to 2° in the boundaries, and 1° of boundary is about 7 mm per edge at 40 cm.

If calibration says an edge is out of view, the box reaches past the servo's ±35° on that side: move it towards the centre, use a narrower box, or move it further away.

Barcode symbology is assumed to be **standard Code 39** (`* letter *`, 9 elements per character, 3 wide). Check against the course's printed barcodes.

---

## 8. Wi-Fi, MQTT and Telnet

Settings are compiled in from `config/wifi_secrets.h` (git-ignored; template `config/wifi_secrets.example.h`). Currently: hotspot **`Lk`** (case-sensitive, must be **2.4 GHz**), broker **172.20.10.2** (the laptop's Wi-Fi address on that hotspot), client ID `mtk3smp-pico`. If the laptop's IP changes, edit the file and rebuild the MQTT image. SIT-WIFI does not work (5 GHz, and devices can't reach each other).

**Broker** (keep the window open):
```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -c "<project>\tools\mqtt\mosquitto_robot.conf" -v
```
Must print `Opening ipv4 listen socket on port 1883`. Double-clicking `mosquitto.exe` listens on localhost only and the car can't connect. Firewall (once, as admin): `New-NetFirewallRule -DisplayName "Mosquitto 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow`.

**Topics** (prefix `mtk3smp-pico/`):

| Topic | Direction | Content |
|---|---|---|
| `map` | PC → car | text pairs, e.g. `A=forward,B=left,C=right,D=uturn` (JSON also accepted); send retained, QoS 1 |
| `map/ack` | car → PC | `ok 4: ...` or `error ...` |
| `snapshot` → `snapshot/reply` | PC → car → PC | any payload → JSON state (state, faults, odometer, speed, x/y, heading, tilt, humps, barcodes, pending letter, `obstacle_width_mm` (last measured, 0 = none), `bypass_side` (the running bypass, else the next one), bypass depth, mapping) |
| `event` | car → PC | JSON per state change, barcode, junction, command, obstacle (`{"ev":"obstacle","width_mm":N,...}`, once per measured obstacle), hump, with x/y |
| `cmd` | PC → car | `start`, `stop`, `mode`, `calibrate`, `depth <mm>`, `wcal <left> <right>` / `esweep` (width calibration, Idle only), `wcal clear`; never retained |
| `telem` | car → PC | 14-byte binary frame every 500 ms (`subsystem5/telem_frame.h`) |
| `heartbeat` | car → PC | `hb N` every 5 s |

Watch everything: `mosquitto_sub -h localhost -t "mtk3smp-pico/#" -v`.

**Web control panel**: double-click `tools/mqtt/web_panel.bat`. It opens http://127.0.0.1:8765 with every control, the readouts and the filterable log; keep its console window open. Needs Python 3 and Mosquitto's command-line tools. (The older Tkinter window, `mapping_ui.py`, was removed on 19 Nov; its shared code is in `tools/mqtt/car_common.py`.)


**Telnet console**: Tera Term → File → New connection → TCP/IP, host = the car's IP (`wifi: link up, ip=...` on USB, or the hotspot's device list), service Telnet, port 23. Output only, one viewer at a time, lines printed before connecting are not sent, drops lines rather than slowing the car.

`docs/SKELETON.md` says the MQTT publishing hook (patch P5) needs instructor agreement.

---

## 9. Remaining work

- Measure the placeholders in section 7 (turns and bypass depend on them).
- Test on the car everything in section 5, in this order: junction straight-through → barcode read → mapping over MQTT → junction turns → obstacle bypass.
- End-of-course trigger (Finished state): needs the course details.
- Saving calibration to flash (currently lost on power-off).
- Smoother edge-following steering (RY-1.8); hardware watchdog (JL-2.7; the post-mortem watch already reboots on a stuck loop).
- S4-U03 GPIO handler time.
- Update the design review pin table and the subsystem owners' plans to match what was implemented (much of RY-, JY-, LK- and CH- work listed as TODO is now written).

## 10. Troubleshooting seen in this session

| Symptom | Cause / fix |
|---|---|
| Boot log says "driver not written yet" / "ADC ring not written yet" | An old image was flashed; use `build_make\` |
| `MOTOR_STALL` right after boot, red LED | The test image; flash the normal one |
| Left sensor stuck at 4095 | Wiring fault on GP26 (was fixed by reseating) |
| Calibration `FAILED`, LED stays yellow | A sensor never saw black, or the compass circle was incomplete; MODE short press, then redo |
| `SSID not found` | Hotspot is 5 GHz or the name's case is wrong |
| `mqtt: connection status ... retrying` | Wrong broker IP, Mosquitto not started with the config, or the firewall |
| Servo popped the sensor out | Fixed with the ramp; also fix the sensor to the horn mechanically |
