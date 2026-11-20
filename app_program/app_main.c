/*
 *      app_main.c
 *      usermain(): the program's entry point.  Changed only by team agreement.
 *      Owner: Jia Lung (RTOS base, JL-1.8).  Plan ref: chain 4.1 step 1.
 *
 *      reset -> kernel init -> devinit (starts the CYW43 service task, which
 *      brings Wi-Fi up WITHOUT blocking us) -> initial task -> usermain()
 *
 *      usermain() runs at priority 1 on core 0 and works in phases:
 *        0. shared objects: fault flag, message buffers, nav_state mutex,
 *           GPIO interrupt dispatcher, WCET probe
 *        1. subsystemN_init(): each creates its objects and inits its
 *           hardware.  S1 runs first: encoders load first on pio1 (R7).
 *        2. subsystemN_start(): each starts its tasks
 *      Any failure sets FAULT_INIT_FAIL, so the Supervisor starts in Fault
 *      ("driver init fails -> Fault", Design Review section 5).
 *      If usermain() RETURNS, the kernel shuts the whole system down.
 */

#include "app.h"
#include "fault_flags.h"
#include "msg_bus.h"
#include "nav_state.h"
#include "gpio_irq.h"
#include "wcet.h"
#include "hal.h"
#include "diag.h"

EXPORT ID app_start_task (FP entry, PRI pri, UW prc, const char * name)
{
    T_CTSK ctsk = {0};
    ID     tid;
    ER     er;

    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = TASK_STACK_SIZE;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc = prc;
    }
#else
    (void) prc;
#endif
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }
    diag_name_task(tid, name);
    er = tk_sta_tsk(tid, 0);
    CHECK(er, name);
    return (er < E_OK) ? er : tid;
}

/* Cyclic handler shared by every periodic task: exinf carries the task ID */
LOCAL void periodic_wakeup (void * exinf)
{
    (void) tk_wup_tsk((ID) (UW) exinf);
}

EXPORT ID app_start_periodic (FP entry, PRI pri, UW prc, RELTIM period_ms,
                              const char * name, ID * cycid)
{
    T_CCYC ccyc = {0};
    ID     tid, cid;

    tid = app_start_task(entry, pri, prc, name);
    if (tid < E_OK)
    {
        return tid;
    }
    ccyc.exinf  = (void *) (UW) tid;
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP) periodic_wakeup;
    ccyc.cyctim = period_ms;
    ccyc.cycphs = period_ms;
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ccyc.cycatr |= TA_ASSPRC; /* run the handler on the task's core */
        ccyc.assprc = prc;
    }
#endif
    cid = tk_cre_cyc(&ccyc);
    CHECK(cid, name);
    if (cycid != NULL)
    {
        *cycid = cid;
    }
    return (cid < E_OK) ? cid : tid;
}

LOCAL BOOL init_ok = TRUE;

LOCAL void step (ER er, const char * what)
{
    if (er < E_OK)
    {
        tm_printf((UB *) "INIT FAILED %s: %d\n", what, (INT) er);
        init_ok = FALSE;
    }
}

EXPORT INT usermain (void)
{
    T_RVER ver;

    diag_pm_boot(); /* keep last run's post-mortem, start a new one */
    tk_ref_ver(&ver);
    tm_printf((UB *) "\n=== INF2004 P1-2 robot car, micro T-Kernel %04x"
                     ", tick %d ms ===\n",
              ver.prver, CNF_TIMER_PERIOD);

    /* Phase 0: shared objects (order matters: flag first) */
    step(fault_flags_init(), "fault_flags_init");
    step(msg_bus_init(), "msg_bus_init");
    step(nav_state_init(), "nav_state_init");
    step(gpio_irq_init(), "gpio_irq_init");
    wcet_init();

    /* Phase 1: objects + hardware.  S1 FIRST (encoders need pio1 offset 0) */
    step(subsystem1_init(), "subsystem1_init (motion)");
    step(subsystem2_init(), "subsystem2_init (line/barcode)");
    step(subsystem3_init(), "subsystem3_init (imu)");
    step(subsystem4_init(), "subsystem4_init (obstacle)");
    step(subsystem5_init(), "subsystem5_init (supervisor/telemetry)");

    if (!init_ok)
    {
        fault_raise(FAULT_INIT_FAIL); /* Supervisor starts in Fault */
    }

    /* Phase 2: tasks.  Supervisor last, so it sees every other task. */
    step(subsystem1_start(), "subsystem1_start");
    step(subsystem2_start(), "subsystem2_start");
    step(subsystem3_start(), "subsystem3_start");
    step(subsystem4_start(), "subsystem4_start");
    step(subsystem5_start(), "subsystem5_start");

    if (!init_ok)
    {
        fault_raise(FAULT_INIT_FAIL);
    }
    tm_printf((UB *) "init %s, gpio irq on core %d\n\n",
              init_ok ? "OK" : "FAILED -> Fault", (INT) gpio_irq_stats.core);
    diag_pm_start_here(); /* core 0's watch alarm (core 1: Supervisor) */
#if !APP_TEST
    diag_pm_print();
#endif

#if APP_TEST
#if TM_CONSOLE_USB_CDC
    /* Wait (max 30 s) until a serial monitor has the USB port open, so
       the results are not lost while Windows brings the COM port up.
       tm_usb_state() == 3: enumerated and the host asserted DTR. */
    {
        IMPORT UW tm_usb_state(void);
        INT       i;
        for (i = 0; i < 300 && tm_usb_state() != 3; i++)
        {
            tk_dly_tsk(100);
        }
        tk_dly_tsk(500);
        tm_printf((UB *) "\n[TEST] test task on core %d (SMP %d)\n",
                  (INT) hal_core_id(), (INT) TK_SUPPORT_SMP);
        tm_printf((UB *) "\n[TEST] serial monitor %s after %d.%d s; init %s\n",
                  tm_usb_state() == 3 ? "connected"
                                      : "NOT seen, starting anyway",
                  i / 10, i % 10,
                  init_ok ? "OK" : "FAILED (scroll up for which step)");
    }
#endif
    diag_pm_print();
    diag_name_task(tk_get_tid(), "usermain/test");
    diag_task_dump("task states at test start");
    diag_kernel_line();
    wcet_reset(); /* boot (CYW43 firmware load) is not part of the test */
    gpio_irq_stats.max_us = 0;
    tk_dly_tsk(100);
    tm_printf((UB *) "[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *) "[TEST] === test run end ===\n");
    /* 30 s of status, once a second, from the highest-priority task */
    {
        INT i;
        for (i = 0; i < 30; i++)
        {
            tk_dly_tsk(1000);
            tm_printf((UB *) "[TEST] +%d s after the tests\n", i + 1);
            diag_kernel_line();
        }
    }
#endif

    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
