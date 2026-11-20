# INF2004 P1-2 — robot car firmware

The course µT-Kernel template (git history kept) with the team's code on top.

| Folder / file | What |
|---|---|
| `docs/SKELETON.md` | **read first**: build and flash, the 5 rules, file map, who does what next, plan changes, pin revision 2 |
| `app_program/` | our code: shared layer + 5 subsystems |
| `firmware/` | prebuilt `.uf2` images for bench bring-up |
| `TEAM.md` | pins, peripherals, tasks, interfaces, kernel objects |
| `patches/` | P1 (1 ms tick) and P2 (kernel ADC driver off), **already applied** to `config/` |
| `tests/host/` | laptop unit tests: `make -C tests/host -f host_tests.mk` |
| `README.md` | the course template's own setup guide (toolchain, SDK) |

Quick build: `cd build_make && make -j8` (needs `PICO_SDK_PATH`, see README.md section 1).
