/*
 *      subsystem1.c
 *      Motion Control: objects, hardware init, task start.   Owner: Jia Lung
 */

#include "subsystem1.h"
#include "drv_motor.h"
#include "encoders.h"

EXPORT ID   tid_motion;
EXPORT BOOL motion_hw_ok;

EXPORT ER subsystem1_init (void)
{
    ER r;

    /* Motors first, so they are held stopped from here on */
    if (drv_motor_init() != 0)
    {
        LOG("motion", "motor PWM init failed\n");
        return E_SYS;
    }

    /* Right encoder first on pio1 (offset 0, R7); left encoder on
       GPIO interrupts (GP7 / GP28 are not consecutive) */
    r = encoders_init();
    LOG("motion",
        "encoders: %s (right: PIO %d instr. at offset 0 on pio1; "
        "left: interrupts on GP%d/GP%d)\n",
        r == E_OK ? "OK" : "FAILED", (INT) encoders_pio_len(), PIN_ENC_L_A,
        PIN_ENC_L_B);
    if (r != E_OK)
    {
        return E_SYS; /* -> FAULT_INIT_FAIL in usermain */
    }
    motion_hw_ok = TRUE;
    return E_OK;
}

EXPORT ER subsystem1_start (void)
{
    tid_motion = app_start_periodic(motion_task, PRI_MOTION, CORE_MOTION,
                                    PERIOD_MOTION_MS, "motion", NULL);
    return (tid_motion < E_OK) ? tid_motion : E_OK;
}

/*** end of file ***/
