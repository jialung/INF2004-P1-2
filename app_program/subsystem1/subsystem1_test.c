/*
 *      subsystem1_test.c
 *      On-target tests for Motion Control.  Built only with: make TEST=1
 *      WHEELS OFF THE GROUND: S1-U02 spins the motors briefly.
 *      Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "drv_motor.h"
#include "encoders.h"
#include "../gpio_irq.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../wcet.h"
#include "../nav_state.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test (void)
{
    TEST_INIT();

    /* let the Supervisor reach Idle first: its entry STOP would cancel
       the test's FOLLOW command below */
    while (sys_state_get() == SYS_INIT)
    {
        tk_dly_tsk(10);
    }

    /* S1-U01 (R7): encoder program loaded at offset 0 on pio1 */
    TEST_CHECK("S1-U01", motion_hw_ok, "motor PWM + both encoders initialised");

    /* S1-U02 (JL-1.33 seed): a stop bit cuts the motors within 1 cycle */
    {
        UW   t_raise, dt;
        BOOL was_running;

        (void) motion_cmd_send(motion_cmd_make(MC_FOLLOW, SPD_SLOW, 0, 0, 1));
        tk_dly_tsk(50);
        was_running = (drv_motor_get(0) != 0) || (drv_motor_get(1) != 0);

        motion_cut_t_us = 0;
        t_raise         = now_us();
        fault_raise(FAULT_MOTOR_STALL); /* a bit Motion Control owns */
        tk_dly_tsk(25);
        dt = motion_cut_t_us - t_raise;

        TEST_CHECK("S1-U02a", was_running, "FOLLOW drives the motors");
        TEST_CHECK("S1-U02b", drv_motor_get(0) == 0 && drv_motor_get(1) == 0,
                   "motors cut after a stop bit");
        TEST_CHECK("S1-U02c", motion_cut_t_us != 0 && dt <= 10500,
                   "cut within one 10 ms cycle (+0.5 ms jitter)");
        tm_printf(
            (UB *) "[TEST] S1-U02 cut latency %d us (fault raised at %d ms)\n",
            (INT) dt, (INT) (t_raise / 1000U));

        (void) motion_cmd_send(motion_cmd_make(MC_STOP, SPD_STOP, 0, 0, 2));
        fault_clear(FAULT_MOTOR_STALL);
    }

    /* S1-U03: the 10 ms loop keeps time (CH-1.34 runs the long version) */
    tk_dly_tsk(1000);
    TEST_CHECK("S1-U03",
               wcet_tab[WCET_MOTION].runs >= 90 &&
                   wcet_tab[WCET_MOTION].jitter_max_us <= 500,
               "motion loop ran ~100x/s with jitter <= 500 us");
    wcet_report();

    /* S1-U04: left encoder interrupt cost and missed edges.  Spin the left
       wheel by hand during this second (or run S1-U02 with wheels free). */
    {
        W c0 = encoders_count(ENC_LEFT);
        tk_dly_tsk(1000);
        tm_printf(
            (UB *) "[TEST] S1-U04 left %d counts/s, %d errors, isr max %d us, "
                   "gpio irq max %d us\n",
            (INT) (encoders_count(ENC_LEFT) - c0), (INT) encoders_left_errors(),
            (INT) encoders_left_isr_max_us(), (INT) gpio_irq_stats.max_us);
        TEST_CHECK("S1-U04", encoders_left_isr_max_us() < 10,
                   "left encoder edge handled in < 10 us");
    }

    /* S1-U05 (JL-1.3): drive FORWARD slowly for 400 ms (wheels off the
       ground) and check both wheels count up.  If a wheel counts down,
       flip its ENC_SIGN_* in subsystem1.h.  If it stays 0, check its wiring. */
    {
        W         l0, r0, dl, dr;
        NAV_STATE ns0, ns1;

        fault_clear(FAULT_TRIP_MASK); /* earlier tests left bits set */
        l0 = encoders_count(ENC_LEFT);
        r0 = encoders_count(ENC_RIGHT);
        nav_read(&ns0);
        (void) motion_cmd_send(motion_cmd_make(MC_FOLLOW, SPD_SLOW, 0, 0, 3));
        tk_dly_tsk(400);
        (void) motion_cmd_send(motion_cmd_make(MC_STOP, SPD_STOP, 0, 0, 4));
        tk_dly_tsk(100);
        dl = encoders_count(ENC_LEFT) - l0;
        dr = encoders_count(ENC_RIGHT) - r0;
        nav_read(&ns1);
        tm_printf((UB *) "[TEST] S1-U05 forward 400 ms: raw left %d, raw right "
                         "%d counts; "
                         "odometer %d mm; left edge errors %d\n",
                  (INT) dl, (INT) dr, (INT) (H) (ns1.m.odo_mm - ns0.m.odo_mm),
                  (INT) encoders_left_errors());
        TEST_CHECK("S1-U05a", ENC_SIGN_L * dl > 0,
                   "left wheel counts UP going forward");
        TEST_CHECK("S1-U05b", ENC_SIGN_R * dr > 0,
                   "right wheel counts UP going forward");
    }

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
