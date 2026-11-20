/*
 *      subsystem4.c
 *      Obstacle Scan: hardware init and task start.           Owner: Lui Khee
 */

#include "subsystem4.h"
#include "drv_servo.h"

EXPORT ID tid_obstacle;

EXPORT ER subsystem4_init (void)
{
    ER er;

    if (drv_servo_init() != 0)
    {
        return E_SYS;
    }
    er = us_init(); /* registers ECHO with gpio_irq.c (R5) */
    if (er < E_OK)
    {
        LOG("obstacle", "ECHO interrupt registration failed: %d\n", (INT) er);
    }
    return er;
}

EXPORT ER subsystem4_start (void)
{
    /* Not a cyclic task: it waits on mbf_scan_req with a 50 ms timeout,
       so a mode change (e.g. leave GUARD) is handled at once. */
    tid_obstacle =
        app_start_task(obstacle_task, PRI_OBSTACLE, CORE_OBSTACLE, "obstacle");
    return (tid_obstacle < E_OK) ? tid_obstacle : E_OK;
}

/*** end of file ***/
