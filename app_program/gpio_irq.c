/*
 *      gpio_irq.c
 *      One IO_IRQ_BANK0 handler that dispatches to each pin's owner.
 *      Kernel side (tk_def_int); registers through hal.h.     Owner: Christian
 */

#include "app.h"
#include "gpio_irq.h"
#include "hw_alloc.h"
#include "msg_bus.h"
#include "hal.h"

typedef struct
{
    UINT        gpio;
    UINT        events;
    GPIO_IRQ_FN fn;
} GPIO_IRQ_USER;

LOCAL GPIO_IRQ_USER            users[GPIO_IRQ_MAX_USERS];
LOCAL volatile UINT            n_users;
LOCAL BOOL                     ready;
EXPORT volatile GPIO_IRQ_STATS gpio_irq_stats;

/* A pending event on a pin nobody registered would re-enter this handler
   forever (edges stay latched, levels stay true) and starve everything
   below interrupt level 1 on this core, including the system tick.  So:
   switch that pin's interrupt off on this core, count it, and say which
   pin it was ([diag] line, gpio spur).  Only runs when no registered pin
   was pending, so it costs the normal path nothing. */
LOCAL RAMFUNC void kill_unregistered (UINT core)
{
    UINT reg, nib;

    for (reg = 0; reg < 4; reg++)
    {
        UW ints = hal_gpio_irq_ints(core, reg);
        if (ints == 0)
        {
            continue;
        }
        for (nib = 0; nib < 8; nib++)
        {
            UINT pin = reg * 8 + nib, i, mine = 0;
            UINT ev = (ints >> (4 * nib)) & 0xFU;
            if (ev == 0)
            {
                continue;
            }
            for (i = 0; i < n_users; i++)
            {
                if (users[i].gpio == pin)
                {
                    mine = 1;
                }
            }
            if (!mine)
            {
                hal_gpio_irq_disable(core, pin, 0xFU);
                gpio_irq_stats.spurious_last_pin = pin;
                gpio_irq_stats.killed_mask |= 1UL << pin;
            }
        }
    }
}

LOCAL RAMFUNC void gpio_bank0_handler (UINT intno)
{
    UW   t0 = now_us();
    UINT i, hit = 0;
    UINT core = gpio_irq_stats.core;

    (void) intno;
    gpio_irq_stats.count++;

    for (i = 0; i < n_users; i++)
    {
        UINT g  = users[i].gpio;
        UINT ev = hal_gpio_irq_status(core, g);

        if (ev != 0)
        {
            hal_gpio_irq_ack(g, ev); /* before the callback */
            users[i].fn(g, ev, t0);
            hit = 1;
        }
    }
    if (!hit)
    {
        gpio_irq_stats.spurious++;
        kill_unregistered(core);
    }
    {
        UW dt = now_us() - t0;
        if (dt > gpio_irq_stats.max_us)
        {
            gpio_irq_stats.max_us = dt;
        }
    }
}

EXPORT ER gpio_irq_init (void)
{
    T_DINT dint;
    ER     er;

    /* The IRQ goes to the NVIC of the core that enables it.  The port's
       rule is "peripheral IRQs on core 0"; usermain runs there.  If it
       ever does not, this still works, on the calling core. */
    gpio_irq_stats.core              = hal_core_id();
    gpio_irq_stats.spurious_last_pin = 0xFFU;
    if (gpio_irq_stats.core != 0)
    {
        tm_printf((UB *) "gpio_irq: WARNING init on core %d, not core 0\n",
                  (INT) gpio_irq_stats.core);
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) gpio_bank0_handler;
    er          = tk_def_int(INTNO_GPIO_BANK0, &dint);
    if (er < E_OK)
    {
        return er;
    }
    ClearInt(INTNO_GPIO_BANK0);
    EnableInt(INTNO_GPIO_BANK0, INTLVL_GPIO);
    ready = TRUE;
    return E_OK;
}

EXPORT ER gpio_irq_register (UINT gpio, UINT events, GPIO_IRQ_FN fn)
{
    UINT i;

    if (!ready)
    {
        return E_OBJ;
    }
    if (gpio > 29 || fn == NULL || (events & ~0xFU))
    {
        return E_PAR;
    }
    if ((GPIO_IRQ_ALLOWED_MASK & (1UL << gpio)) == 0)
    {
        tm_printf((UB *) "gpio_irq: GP%d is not in hw_alloc.h\n", (INT) gpio);
        return E_PAR;
    }
    for (i = 0; i < n_users; i++)
    {
        if (users[i].gpio == gpio)
        {
            return E_OBJ; /* one owner per pin */
        }
    }
    if (n_users >= GPIO_IRQ_MAX_USERS)
    {
        return E_LIMIT;
    }

    users[n_users].gpio   = gpio;
    users[n_users].events = events;
    users[n_users].fn     = fn;
    MEM_BARRIER();
    n_users++; /* entry complete before it is visible */

    hal_gpio_irq_enable(gpio_irq_stats.core, gpio, events);
    return E_OK;
}

/*** end of file ***/
