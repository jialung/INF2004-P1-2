# Laptop unit tests for the skeleton's pure logic (no Pico needed).
#   make -C tests/host -f host_tests.mk        build and run
# Needs any host C compiler (gcc / clang; on Windows use WSL or MSYS2).

CC      ?= cc
APP     := ../../app_program
CFLAGS  := -std=gnu11 -Wall -Wextra -Werror -O1 -g -Ishim -DTM_WIFI_CYW43=1 -DAPP_TEST=0
SRCS    := test_logic.c $(APP)/msg_bus.c $(APP)/fault_flags.c \
           $(APP)/subsystem2/line_search.c $(APP)/subsystem5/telem_frame.c \
           $(APP)/subsystem2/barcode_decode.c $(APP)/subsystem5/map_parse.c \
           $(APP)/subsystem3/imu_math.c $(APP)/subsystem5/bypass_geom.c \
           $(APP)/subsystem5/obst_width.c

run: test_logic
	./test_logic

test_logic: $(SRCS) $(wildcard shim/*/*.h) $(wildcard $(APP)/*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f test_logic

.PHONY: run clean
