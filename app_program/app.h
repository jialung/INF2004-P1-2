/*
 *      app.h
 *      Definitions shared by the whole team.  Changed only by team agreement.
 *      KERNEL SIDE ONLY: never include this in a hal_*.c or drv_*.c file
 *      (see the two-sides rule in hal.h).
 *
 *      Anything that belongs to ONE subsystem goes in that subsystem's own
 *      header instead (app_program/subsystemN/subsystemN.h).
 *
 *      Subsystems (TEAM.md section 1)
 *        subsystem1/  Jia Lung   Motion Control, encoders, odometry
 *        subsystem2/  Ryan       Line Following, Barcode Decode, LineSearch
 *        subsystem3/  Jian Yi    IMU: heading, tilt, humps, impact
 *        subsystem4/  Lui Khee   Obstacle Scan: ultrasonic + servo
 *        subsystem5/  Christian  Supervisor, Telemetry, buttons, LED, buzzer
 *      Shared modules (this folder): rtos_config.h, hw_alloc.h, hal.h,
 *      fault_flags, msg_types/msg_bus, nav_state, gpio_irq, wcet.
 */

#ifndef APP_H
#define APP_H

#if defined(_PICO_H)
#error "A Pico SDK header was included before app.h. Kernel and SDK headers" \
    " cannot share a .c file: see the two-sides rule in hal.h"
#endif

#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output */

#include "rtos_config.h"
#include "hw_alloc.h"

#if !TM_WIFI_CYW43
#error "Build with WIFI=cyw43 (the default): the SDK include paths and" \
    " the radio are needed"
#endif
#if TM_CONSOLE_UART
#error "GP0/GP1 are the right wheel encoder: build without CRASH_REPORT=1" \
    " and without CONSOLE=uart (USB console only)"
#endif

#define TASK_STACK_SIZE STACK_DEFAULT

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char * name);

/* Create and start a task plus a cyclic handler that wakes it with
   tk_wup_tsk every period_ms, on the same core.  The task loop is
       for (;;) { tk_slp_tsk(TMO_FEVR); ... }
   Returns the task ID; the cyclic ID goes to *cycid if not NULL. */
IMPORT ID app_start_periodic(FP entry, PRI pri, UW prc, RELTIM period_ms,
                             const char * name, ID * cycid);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects, init
 *                        its hardware (in this order: S1 first, because
 *                        the encoder PIO program needs offset 0)
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)                                            \
    do                                                             \
    {                                                              \
        INT check_er_ = (INT) (er); /* evaluate once */            \
        if (check_er_ < E_OK)                                      \
        {                                                          \
            tm_printf((UB *) "ERROR %s: %d\n", (what), check_er_); \
        }                                                          \
    } while (0)

/* Memory barrier: make a plain shared write visible to the other core
   before the next one (single-writer words, flags).  Host builds (unit
   tests) use the compiler builtin. */
#if defined(__arm__)
#define MEM_BARRIER() __asm__ volatile("dmb" ::: "memory")
#else
#define MEM_BARRIER() __sync_synchronize()
#endif

/* Tag every log line with its plan ID owner, e.g. LOG("motion", "...") */
#define LOG(tag, ...)                   \
    do                                  \
    {                                   \
        tm_printf((UB *) "[" tag "] "); \
        tm_printf((UB *) __VA_ARGS__);  \
    } while (0)

#endif /* APP_H */

/*** end of file ***/
