/*
 *      enc_irq.c
 *      Left encoder (GP7 = A, GP28 = B) by edge interrupts, and the kernel-side
 *      encoders.h wrapper for both wheels.                    Owner: Jia Lung
 *
 *      Why not PIO: the quadrature program reads pin and pin + 1.  GP7 and
 *      GP28 are neighbours on the Grove 7 plug but not consecutive GPIO
 *      numbers, so no PIO input mapping can see them as a pair.
 *
 *      Decoding: on every edge of A or B, read BOTH pins in one register read,
 *      build state = (A << 1) | B, and look up (previous << 2) | state in a
 *      16-entry table: +1, -1, 0 (no change) or invalid (both bits changed =
 *      an edge was missed).  On invalid, assume the wheel kept its last
 *      direction (2 steps) and count it in left_errors.
 *      Cost: one table lookup per edge, a few microseconds at most.
 *      CPU load check (S1-U04): edges/s = counts/rev x rev/s; e.g. 2425 x 3
 *      = ~7300 edges/s x ~3 us = ~2.2 % of core 0 (2425: measured 30 Oct).
 */

#include "encoders.h"
#include "drv_encoder.h"
#include "../gpio_irq.h"
#include "../hal.h"
#include "../msg_bus.h"

LOCAL volatile W  left_count;
LOCAL volatile UW left_errors;
LOCAL volatile UW left_isr_max;
LOCAL UINT        left_state;
LOCAL B           left_dir   = 1;
LOCAL INT         pio_status = ENC_ERR_NO_SPACE;

/* index = (previous << 2) | current, state = (A << 1) | B.
   Forward (A leads): 00 -> 01 -> 11 -> 10 -> 00.  2 = invalid. */
LOCAL const B quad_table[16] = {0,  +1, -1, 2,  -1, 0,  2,  +1,
                                +1, 2,  0,  -1, 2,  -1, +1, 0};

LOCAL RAMFUNC UINT read_left (void)
{
    UW in = hal_gpio_get_all(); /* one read: A and B from the same instant */
    return (((in >> PIN_ENC_L_A) & 1U) << 1) | ((in >> PIN_ENC_L_B) & 1U);
}

/* Interrupt context (gpio_irq.c): registered for both pins */
LOCAL RAMFUNC void left_edge (UINT gpio, UINT events, UW t_us)
{
    UINT cur  = read_left();
    B    step = quad_table[(left_state << 2) | cur];

    (void) gpio;
    (void) events;
    if (step == 2)
    {
        left_errors++;
        left_count += 2 * left_dir; /* both bits flipped: 2 steps */
    }
    else if (step != 0)
    {
        left_count += step;
        left_dir = step;
    }
    left_state = cur;
    {
        UW dt = now_us() - t_us;
        if (dt > left_isr_max)
        {
            left_isr_max = dt;
        }
    }
}

EXPORT ER encoders_init (void)
{
    ER er;

    /* Right: PIO, must be first on pio1 (offset 0, R7) */
    pio_status = drv_encoder_init();
    if (pio_status != ENC_OK)
    {
        return E_SYS;
    }

    /* Left: interrupts.  Registered before anyone else's pins, so the
       busiest user is first in the dispatcher's list. */
    hal_gpio_in(PIN_ENC_L_A, HAL_PULL_UP);
    hal_gpio_in(PIN_ENC_L_B, HAL_PULL_UP);
    left_state = read_left();
    er = gpio_irq_register(PIN_ENC_L_A, GPIO_EV_RISE | GPIO_EV_FALL, left_edge);
    if (er < E_OK)
    {
        return er;
    }
    return gpio_irq_register(PIN_ENC_L_B, GPIO_EV_RISE | GPIO_EV_FALL,
                             left_edge);
}

EXPORT W encoders_count (UINT side)
{
    return (side == ENC_RIGHT) ? (W) drv_encoder_count() : left_count;
}

EXPORT UW encoders_left_errors (void)
{
    return left_errors;
}

EXPORT UW encoders_left_isr_max_us (void)
{
    return left_isr_max;
}

EXPORT UINT encoders_pio_len (void)
{
    return (UINT) drv_encoder_program_len();
}

/*** end of file ***/
