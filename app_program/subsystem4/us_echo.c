/*
 *      us_echo.c
 *      HC-SR04: 10 us TRIG, ECHO timed by edge interrupt + 1 us timer.
 *      Kernel side (semaphore, gpio_irq).                     Owner: Lui Khee
 *      Plan refs: LK-1.5..LK-1.9, R5 (ECHO goes through gpio_irq.c ONLY).
 */

#include "subsystem4.h"
#include "../gpio_irq.h"
#include "../hal.h"

#define ECHO_WAIT_MS 30 /* > 400 cm round trip (23 ms) */
#define US_PER_CM    58 /* round trip, ~343 m/s */

LOCAL ID            sem_echo;
LOCAL volatile UW   t_rise;
LOCAL volatile UW   width_us;
LOCAL volatile BOOL got_rise;
EXPORT volatile UW  echo_isr_count;

/* Interrupt context: no waiting, no tm_printf */
LOCAL RAMFUNC void echo_isr (UINT gpio, UINT events, UW t_us)
{
    (void) gpio;
    echo_isr_count++;
    if (events & GPIO_EV_RISE)
    {
        t_rise   = t_us;
        got_rise = TRUE;
    }
    if ((events & GPIO_EV_FALL) && got_rise)
    {
        width_us = t_us - t_rise; /* wrap-safe, 32 bit */
        (void) tk_sig_sem(sem_echo, 1);
    }
}

EXPORT ER us_init (void)
{
    T_CSEM csem = {0};

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = 1;
    sem_echo     = tk_cre_sem(&csem);
    if (sem_echo < E_OK)
    {
        return sem_echo;
    }

    hal_gpio_out(PIN_US_TRIG, 0);
    hal_gpio_in(PIN_US_ECHO, HAL_PULL_NONE); /* via the 1k/2k divider */
    return gpio_irq_register(PIN_US_ECHO, GPIO_EV_RISE | GPIO_EV_FALL,
                             echo_isr);
}

/* One ping: US_OK with the echo width in us, else US_NO_ECHO / US_NO_RISE */
LOCAL US_RESULT ping_us (UW * w_us)
{
    ER er;

    /* drop a late echo from the previous ping */
    while (tk_wai_sem(sem_echo, 1, TMO_POL) == E_OK)
    {
        /* drain */
    }
    got_rise = FALSE;

    hal_gpio_put(PIN_US_TRIG, 1);
    hal_busy_wait_us(10); /* LK-1.5 */
    hal_gpio_put(PIN_US_TRIG, 0);

    er = tk_wai_sem(sem_echo, 1, ECHO_WAIT_MS);
    if (er == E_OK)
    {
        *w_us = width_us;
        return US_OK;
    }
    /* no falling edge in time: rising edge seen = nothing in range
       (normal); no rising edge at all = sensor or wiring problem */
    return got_rise ? US_NO_ECHO : US_NO_RISE;
}

EXPORT US_RESULT us_ping (UW * dist_cm)
{
    UW        w;
    US_RESULT r = ping_us(&w);
    UW        cm;

    if (r != US_OK)
    {
        return r;
    }
    cm       = w / US_PER_CM;
    *dist_cm = (cm > 400) ? 400 : cm; /* 9 bits (LK-1.9) */
    return (cm < 2) ? US_NO_ECHO : US_OK;
}

/* Same ping, in mm (the width sweep needs better than 1 cm) */
EXPORT US_RESULT us_ping_mm (UW * dist_mm)
{
    UW        w;
    US_RESULT r = ping_us(&w);
    UW        mm;

    if (r != US_OK)
    {
        return r;
    }
    mm       = w * 10U / US_PER_CM;
    *dist_mm = (mm > 4000U) ? 4000U : mm;
    return (mm < 20U) ? US_NO_ECHO : US_OK;
}

/*** end of file ***/
