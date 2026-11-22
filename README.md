# INF2004 P1-2: autonomous robot car

Firmware for a line-following robot car on a **Raspberry Pi Pico W**, built
on **micro T-Kernel 3.0** running on both RP2040 cores. The car follows a
line, reads barcodes, turns at junctions by a map sent from a PC, slows over
humps, and stops for or drives around obstacles. A PC control panel talks to
it over Wi-Fi and MQTT.

Started from the course template
([sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)).
Our code is in `app_program/`.

## Team

| # | Subsystem | Owner |
|---|---|---|
| 1 | Motion control: motors, encoders, PID, odometry | Jia Lung |
| 2 | Line following, barcode decode, line search | Ryan |
| 3 | IMU: heading, tilt, humps, impact | Jian Yi |
| 4 | Obstacle scan: ultrasonic sensor + scan servo | Lui Khee |
| 5 | Supervisor, telemetry, buttons, RGB LED, buzzer | Christian |

Pins, peripherals, task priorities, interfaces and the kernel object budget
are in [`TEAM.md`](TEAM.md).

## How it works

Seven tasks, each with a unique priority (`app_program/rtos_config.h`):

| Task | Priority | Core | Period |
|---|---|---|---|
| motion | 8 | 1 | 10 ms |
| line | 9 | 1 | 5 ms |
| supervisor | 10 | 1 | event-driven (10 ms poll) |
| barcode | 11 | 1 | 1 ms |
| imu | 12 | 0 | 20 ms |
| obstacle | 13 | 0 | 50 ms + on request |
| telemetry | 15 | 0 | 500 ms |

- **Supervisor** owns the state machine: Init, Idle, Calibrate, LineFollow,
  Junction, Hump, Obstacle, LineSearch, Finished, Fault. Every subsystem
  reports to it through one message buffer, `mbf_sup_event`.
- **Faults** go through one event flag, `flg_fault` (`fault_flags.h`). Motion
  checks it every 10 ms and stops the motors.
- **Messages** are packed with shift/mask macros (`msg_types.h`). Telemetry is
  a 14-byte binary frame with a CRC-8 (`subsystem5/telem_frame.h`).
- Kernel tick is 1 ms (patch P1). The kernel ADC driver is off, so subsystem 2
  owns the ADC (patch P2). Both patches are already applied in `config/`.

## Hardware

Pico W, two DC motors with encoders, three IR line sensors (left, right,
barcode), a GY-511 IMU (LSM303DLHC), an HC-SR04 ultrasonic sensor on an SG90
servo, a WS2812 RGB LED, START and MODE buttons, and a buzzer. Pin numbers are
in `app_program/hw_alloc.h` and `TEAM.md` section 2.

GP0/GP1 are the right encoder, so there is **no UART console**.
`CRASH_REPORT=1` and `CONSOLE=uart` are refused on purpose.

## Repo layout

| Path | What |
|---|---|
| `app_program/` | our code: shared layer + `subsystem1/` ... `subsystem5/` |
| `build_make/` | makefiles; images are built here |
| `config/` | kernel config; `wifi_secrets.example.h` |
| `tests/host/` | unit tests that run on a laptop, no Pico needed |
| `testing/` | test logs per subsystem, integration and system |
| `tools/mqtt/` | PC control panel and Mosquitto config |
| `tools/elf2uf2/` | turns the `.elf` into a `.uf2` |
| `patches/` | template patches P1 and P2 (already applied) |
| `docs/HANDOFF.md` | car state, operating it, values still to measure |
| `docs/SKELETON.md` | first shared skeleton and change log (23 Oct) |
| `videos/` | links to test and demo videos (unlisted YouTube) |
| `kernel/`, `lib/`, `include/`, `device/`, `etc/` | micro T-Kernel 3.0 (template) |

## Setup

You need:

- **Arm GNU Toolchain** (`arm-none-eabi-gcc`), from
  [Arm](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  or the Pico VS Code extension.
- **Pico SDK** 2.2.0 or later, with the `tinyusb`, `cyw43-driver` and `lwip`
  submodules:
  ```sh
  git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
  cd pico-sdk
  git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
  ```
- **make** and a host C/C++ compiler. On Windows, MSYS2 gives both.

Then copy the Wi-Fi settings and fill them in:

```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

`config/wifi_secrets.h` is git-ignored. Never commit it. `MQTT_BROKER_IP`
must be the PC's IPv4 address on the car's hotspot. The hotspot must be
2.4 GHz.

## Build

### Windows (PowerShell + MSYS2)

Set the paths once per terminal. Change them to match your PC:

```powershell
cd <project>\build_make
$env:PATH = "C:\Users\<you>\.pico-sdk\toolchain\<version>\bin;C:\msys64\usr\bin;" + $env:PATH
$env:PICO_SDK_PATH = "C:/Users/<you>/.pico-sdk/sdk/<version>"
```

### macOS / Linux / WSL

```sh
export PATH="<toolchain>/bin:$PATH"
export PICO_SDK_PATH="<path>/pico-sdk"
cd build_make
```

### Images

```sh
make -j8 NET=lwip MQTT=1          # demo image: Wi-Fi + MQTT + Telnet
make -j8                          # USB console only, no networking
make -j8 TEST=1                   # runs the subsystem self-tests at power-up
make -j8 SMP=0 NET=lwip MQTT=1    # single core (debugging only)
```

A good build ends with an `elf2uf2 ... .uf2` line. If you see `error:` or
`*** ... Stop.`, the build failed and the `.uf2` is still the **old** one.

Options are not remembered: give them on every `make`. Switching options needs
no `make clean`. The makefile options are documented in
`build_make/pico_rp2040.mk`.

## Flash

1. Hold **BOOTSEL**, plug in USB, release. The **RPI-RP2** drive appears.
2. Copy the `.uf2` from `build_make/` onto it, for example
   `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
3. Serial console: USB, 115200 baud.

## Run

1. Start the MQTT broker and leave it open:
   ```powershell
   & "C:\Program Files\mosquitto\mosquitto.exe" -c tools\mqtt\mosquitto_robot.conf -v
   ```
   It must print `Opening ipv4 listen socket on port 1883`.
   This config allows connections with no password, so use it only on your
   own hotspot.
2. Open the control panel: `python tools/mqtt/web_panel.py` (or double-click
   `tools\mqtt\web_panel.bat`). It opens http://127.0.0.1:8765.
3. Send the barcode map, then press START (on the car or in the panel).
   MODE short press clears a fault. MODE long press in Idle starts
   calibration.

Anyone on the car's network can open the panel and drive the car. Use
`--local-only` to allow this PC only. `docs/HANDOFF.md` sections 6 and 8 have
the details.

## Tests

Host unit tests (message formats, barcode decoder, map parser, telemetry
frame, bypass geometry), run from the project root:

```sh
make -C tests/host -f host_tests.mk CC=gcc
```

On Windows, put `C:\msys64\ucrt64\bin` on `PATH` first. Last result:
`host tests: 606 passed, 0 failed`. Run them before every push.

On-target tests: build with `TEST=1`. Keep the wheels off the ground, because
some tests spin the motors. Record results in `testing/` (see
`testing/README.md`).

## Troubleshooting

| Message | Fix |
|---|---|
| `arm-none-eabi-gcc: command not found` | toolchain `bin` folder is not on `PATH` |
| `PICO_SDK_PATH is not set` | set `PICO_SDK_PATH` |
| `No rule to make target 'C:/Users/<someone-else>/...'` | stale `.d` files from another PC: `make clean`, then build again |
| `CRASH_REPORT=1` or `CONSOLE=uart` refused | on purpose: GP0/GP1 are the right encoder |
| `wifi_secrets.h: No such file` | copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` |
| Car never connects to MQTT | broker started without the config file, wrong `MQTT_BROKER_IP`, or a 5 GHz network |

## License

micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, under the T-License 2.2. The
dual-core RP2040 port comes from the official μT-Kernel 3.0 BSP
(`tron-forum/mtk3_bsp`, branch `pico_rp2040`).
