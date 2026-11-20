/*
 *      wcet.c
 *      Owner: Jia Lung.  See wcet.h.
 */

#include "app.h"
#include "wcet.h"
#include "hw_alloc.h"
#include "msg_bus.h"
#include "hal.h"

EXPORT volatile WCET_REC wcet_tab[WCET_SLOTS];

LOCAL const char * const slot_name[WCET_SLOTS] = {
    "motion", "line", "supervisor", "barcode", "imu", "obstacle", "telemetry"};

EXPORT void wcet_init (void)
{
    INT i;

    hal_gpio_out(PIN_WCET_PROBE, 0);
    for (i = 0; i < WCET_SLOTS; i++)
    {
        wcet_tab[i].gap_min_us = 0xFFFFFFFFU; /* tasks start later */
    }
}

EXPORT void wcet_set_period (WCET_SLOT s, UW period_ms)
{
    wcet_tab[s].period_us = period_ms * 1000U;
}

EXPORT UW wcet_release (WCET_SLOT s)
{
    volatile WCET_REC * r  = &wcet_tab[s];
    UW                  t0 = now_us();

    if (r->runs != 0 && r->period_us != 0)
    {
        UW actual = t0 - r->last_release; /* wrap-safe */
        UW dev    = (actual > r->period_us) ? actual - r->period_us
                                            : r->period_us - actual;
        if (dev > r->jitter_max_us)
        {
            r->jitter_max_us = dev;
        }
        if (actual > r->gap_max_us)
        {
            r->gap_max_us    = actual;
            r->gap_max_at_us = t0;
        }
        if (actual < r->gap_min_us)
        {
            r->gap_min_us    = actual;
            r->gap_min_at_us = t0;
        }
    }
    r->last_release = t0;
    if (s == WCET_PROBE_SLOT)
    {
        hal_gpio_put(PIN_WCET_PROBE, 1);
    }
    return t0;
}

EXPORT void wcet_end (WCET_SLOT s, UW t0)
{
    volatile WCET_REC * r  = &wcet_tab[s];
    UW                  dt = now_us() - t0;

    if (s == WCET_PROBE_SLOT)
    {
        hal_gpio_put(PIN_WCET_PROBE, 0);
    }
    r->runs++;
    r->exec_sum_us += dt;
    if (dt > r->exec_max_us)
    {
        r->exec_max_us = dt;
    }
    if (r->period_us != 0 && dt > r->period_us)
    {
        r->overruns++;
    }
}

EXPORT void wcet_reset (void)
{
    INT i;

    for (i = 0; i < WCET_SLOTS; i++)
    {
        UW p                      = wcet_tab[i].period_us;
        wcet_tab[i].runs          = 0;
        wcet_tab[i].exec_max_us   = 0;
        wcet_tab[i].exec_sum_us   = 0;
        wcet_tab[i].jitter_max_us = 0;
        wcet_tab[i].overruns      = 0;
        wcet_tab[i].gap_max_us    = 0;
        wcet_tab[i].gap_max_at_us = 0;
        wcet_tab[i].gap_min_us    = 0xFFFFFFFFU;
        wcet_tab[i].gap_min_at_us = 0;
        wcet_tab[i].period_us     = p;
    }
}

EXPORT void wcet_report (void)
{
    INT i;

    tm_printf((
        UB *) "wcet: slot        runs   max_us  avg_us  jitter_us  overruns\n");
    for (i = 0; i < WCET_SLOTS; i++)
    {
        volatile WCET_REC * r = &wcet_tab[i];
        tm_printf((UB *) "wcet: %-10s %7d %8d %7d %10d %9d\n", slot_name[i],
                  (INT) r->runs, (INT) r->exec_max_us,
                  (INT) (r->runs ? r->exec_sum_us / r->runs : 0),
                  (INT) r->jitter_max_us, (INT) r->overruns);
    }
    /* Gap detail: times are ms since boot, same clock as now_us() */
    tm_printf((UB *) "wcet: slot       gap_max_us  at_ms  gap_min_us  at_ms  "
                     "(now %d ms)\n",
              (INT) (now_us() / 1000U));
    for (i = 0; i < WCET_SLOTS; i++)
    {
        volatile WCET_REC * r = &wcet_tab[i];
        if (r->period_us == 0 || r->runs < 2)
        {
            continue; /* event-driven or no data */
        }
        tm_printf((UB *) "wcet: %-10s %11d %6d %11d %6d\n", slot_name[i],
                  (INT) r->gap_max_us, (INT) (r->gap_max_at_us / 1000U),
                  (INT) r->gap_min_us, (INT) (r->gap_min_at_us / 1000U));
    }
}

/*** end of file ***/
