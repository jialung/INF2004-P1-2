/*
 *      wcet.h
 *      Execution time and release jitter for every periodic task.
 *                                                            Owner: Jia Lung
 *      Plan refs: JL-1.9, CH-1.34 / JL-1.32 (jitter <= 500 us), RY-1.27,
 * JL-2.7.
 *
 *      In each periodic task loop:
 *          tk_slp_tsk(TMO_FEVR);              // woken by the cyclic handler
 *          UW t0 = wcet_release(WCET_MOTION); // start of this cycle
 *          ... work ...
 *          wcet_end(WCET_MOTION, t0);
 *
 *      The slot named by WCET_PROBE_SLOT also drives PIN_WCET_PROBE (GP19)
 *      high during the work, for a scope or logic analyser (GPIO capture
 *      evidence).  Print the table with wcet_report().
 */

#ifndef WCET_H
#define WCET_H

#include <tk/tkernel.h>

typedef enum
{
    WCET_MOTION = 0,
    WCET_LINE,
    WCET_SUPERVISOR,
    WCET_BARCODE,
    WCET_IMU,
    WCET_OBSTACLE,
    WCET_TELEMETRY,
    WCET_SLOTS
} WCET_SLOT;

#ifndef WCET_PROBE_SLOT
#define WCET_PROBE_SLOT WCET_MOTION
#endif

typedef struct
{
    UW period_us; /* nominal, 0 = event-driven (no jitter stats) */
    UW last_release;
    UW runs;
    UW exec_max_us;
    UW exec_sum_us;   /* for the average; wraps after a long run */
    UW jitter_max_us; /* max |actual period - nominal| */
    UW overruns;      /* exec time > period */
    /* Where the jitter came from: the longest and shortest gap between
       two releases, and when (now_us()) the release that closed it ran.
       A long gap = a late release; a short one = a catch-up release. */
    UW gap_max_us;
    UW gap_max_at_us;
    UW gap_min_us; /* 0xFFFFFFFF until the second release */
    UW gap_min_at_us;
} WCET_REC;

IMPORT volatile WCET_REC wcet_tab[WCET_SLOTS];

IMPORT void wcet_init(void);
IMPORT void wcet_set_period(WCET_SLOT s, UW period_ms);
IMPORT UW   wcet_release(WCET_SLOT s); /* returns t0 */
IMPORT void wcet_end(WCET_SLOT s, UW t0);
IMPORT void wcet_reset(void);  /* clear stats (e.g. after boot) */
IMPORT void wcet_report(void); /* tm_printf the table (task ctx) */

#endif /* WCET_H */

/*** end of file ***/
