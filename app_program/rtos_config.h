/*
 *      rtos_config.h
 *      Task table: priorities, periods, cores, stack sizes.   Owner: Jia Lung
 *      Changed only by team agreement.  Plan refs: section 1.1, JL-1.8,
 * JL-1.21.
 *
 *      Priorities: 1 is highest.  The kernel's own tasks sit above ours:
 *        initial task (usermain) 1, USB console 3, CYW43 Wi-Fi poll 4.
 *      Both kernel service tasks are pinned to core 0, so the two fast
 *      control loops (Motion Control, Line Following) are pinned to core 1,
 *      where Wi-Fi polling and USB can never preempt them (NFR jitter).
 *
 *      Cores: TP_PRC1 = physical core 0, TP_PRC2 = physical core 1.
 *      Peripheral IRQs (GPIO bank, I2C, USB) are owned by core 0 in this port
 *      (docs/PORT_RP2040.md), so the tasks that own those devices run there.
 */

#ifndef RTOS_CONFIG_H
#define RTOS_CONFIG_H

#include <config.h> /* CNF_TIMER_PERIOD (kernel tick, ms) */

/* ---------------------------------------------------------------------
 *  Task table (plan section 1.1)
 * --------------------------------------------------------------------- */
/*      name                    priority        period (ms, 0 = event-driven)
 * core   */
#define PRI_MOTION     8
#define PRI_LINE       9
#define PRI_SUPERVISOR 10
#define PRI_BARCODE    11
#define PRI_IMU        12
#define PRI_OBSTACLE   13
#define PRI_TELEMETRY  15

#define PERIOD_MOTION_MS    10
#define PERIOD_LINE_MS      5
#define PERIOD_BARCODE_MS   1 /* NFR3; also the only ADC reader */
#define PERIOD_IMU_MS       20
#define PERIOD_OBSTACLE_MS  50
#define PERIOD_TELEMETRY_MS 500

#define CORE_MOTION     TP_PRC2
#define CORE_LINE       TP_PRC2
#define CORE_SUPERVISOR TP_PRC2
#define CORE_BARCODE    TP_PRC2
#define CORE_IMU        TP_PRC1
#define CORE_OBSTACLE   TP_PRC1
#define CORE_TELEMETRY  TP_PRC1

/* Supervisor wakes at least this often even with no event, to check the
   fault flag, time button holds and run timeouts (fix for "one wait
   object": it blocks on mbf_sup_event only, never on the flag). */
#define SUP_POLL_MS 10

/* ---------------------------------------------------------------------
 *  Tick check.  A cyclic handler cannot run faster than the kernel tick.
 *  The template ships with a 10 ms tick, which makes the 5 ms Line
 *  Following period impossible.  Set CNF_TIMER_PERIOD to 1 in
 *  config/config.h (allowed range on RP2040: 1..50).
 * --------------------------------------------------------------------- */
#if (PERIOD_LINE_MS % CNF_TIMER_PERIOD) != 0 || \
    (PERIOD_MOTION_MS % CNF_TIMER_PERIOD) != 0
#error "Kernel tick too coarse: set CNF_TIMER_PERIOD to 1 in config/config.h" \
    " (see docs/SKELETON.md, patch P1)"
#endif

/* ---------------------------------------------------------------------
 *  Stack size (bytes).  tm_printf needs about 1 KB, so every task that
 *  logs keeps the template default.  Measure the high-water mark before
 *  shrinking (JL-2.7).
 * --------------------------------------------------------------------- */
#define STACK_DEFAULT 2048

#endif /* RTOS_CONFIG_H */

/*** end of file ***/
