/*
 *      diag_pm.c
 *      Post-mortem watch: catches a stuck control loop even when the console
 *      is dead, makes the car safe, and reports it on the NEXT boot.
 *                                                            Owner: Jia Lung
 *
 *      Why: 23 Oct bench runs with the serial monitor open from power-on ended
 *      with the buzzer stuck on and the console silent, so nothing printed at
 *      the time of the failure could be seen.  This records instead of prints.
 *
 *      How:
 *        - each core runs a 1 ms timer alarm (alarm 2 on core 0, alarm 3 on
 *          core 1) at NVIC priority 0, installed raw (TA_ASM), outside the
 *          kernel's interrupt layer.  It only reads memory and registers: no
 *          kernel call, no lock, no tm_printf.
 *        - every 4 ms it records where its core is: interrupted PC / LR,
 *          exception number, current task (ring of 24 = last ~100 ms).
 *        - it trips when: the Supervisor or Motion loop has not completed a
 *          run for 1.5 s, the other core's alarm has not run for 0.5 s (that
 *          core is masked or dead), the buzzer has been on for 3 s, or a
 *          HardFault happens.
 *        - on a trip: freezes the ring, copies every task's state (and the
 *          saved PC / stack of the stuck ones) into SRAM4 (not cleared by a
 *          reboot) and switches the motors and buzzer off.  Any task that
 *          still runs prints the record at once (diag_pm_live_report); 2.5 s
 *          later the hardware watchdog reboots the chip (at most 2 reboots in
 *          a row, then it stays up with the outputs off).
 *        - diag_pm_print() on the next boot prints the record ("[pm]" lines).
 *          Send those lines with the matching firmware: the PCs are resolved
 *          against the .elf (arm-none-eabi-addr2line -f -e <elf> <pc>).
 *
 *      Kernel side, and it reads kernel internals (TCBs) on purpose: it is a
 *      diagnostic.  If the kernel's TCB layout changes, rebuild; nothing else.
 */

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "../tkernel/task.h"
#include <tm/tmonitor.h>
#include "diag.h"
#include "wcet.h"
#include "hw_alloc.h"
#include "hal.h"

#define PM_ADDR  0x20040000U /* SRAM4 bottom; see note below */
#define PM_MAGIC 0x504D3031U /* "PM01" */
#define PM_NS    24          /* samples per core */
#define PM_NT    14          /* tasks recorded */
#define PM_NW    16          /* stack words per stuck task */
#define PM_NSTK  3

#define SAMPLE_EVERY  4 /* ms between samples */
#define LOOP_LIMIT_US 1500000U
#define CORE_LIMIT_US 500000U
#define BUZZ_LIMIT_US 3000000U
#define MAX_REBOOTS   2

/* Reasons */
#define R_SUP       1
#define R_MOTION    2
#define R_CORE0     3
#define R_CORE1     4
#define R_HARDFAULT 5
#define R_BUZZER    6
#define R_PIN       7

/* Saved task context on ARMv6-M (kernel/sysdepend/cpu/core/armv6m/
   cpu_task.h, PM_SAVED_CTX): exp_ret, r4-r11, then the hardware frame */
typedef struct
{
    UW    exp_ret, r_[8], r[4], ip;
    void *lr, *pc;
    UW    xpsr;
} PM_SAVED_CTX;

/* info: exc | task<<8 | age_ms<<16 */
typedef struct
{
    UW pc, lr, info;
} PM_SAMPLE;
typedef struct
{
    UB id, state, pri, run; /* run: 1 + core when running */
    UW tskwait;
    UH wid;
    H  wupcnt;
    UW pc, lr; /* saved context when not running */
} PM_TASK;

typedef struct
{
    UW        magic, nmagic, reason, core, t_us, boot_ms;
    UW        sup_runs, motion_runs, hb[2], buzzer, mot[4];
    UW        late_max_us[2], late_pc[2];
    UW        fault_pc, fault_lr, fault_psr, fault_core;
    UW        head[2];
    PM_SAMPLE s[2][PM_NS];
    PM_TASK   t[PM_NT];
    UW        stk_id[PM_NSTK];
    UW        stk[PM_NSTK][PM_NW];
} PM;

/* SRAM4/5 hold core 1's exception stack (kernel smp_rp2040.h, 8 KiB from
   0x20042000 down).  It never gets near the bottom 1.3 KiB used here; a
   watchdog reboot and the boot ROM's flash boot leave SRAM4 untouched. */
#define pm ((volatile PM *) PM_ADDR)

LOCAL PM   prev; /* last run's record, copied at boot */
LOCAL BOOL have_prev;
LOCAL UW   prev_cause;

/* live state (normal RAM, cleared at boot) */
LOCAL volatile UW hb[2], next_due[2], tick_n[2];
LOCAL volatile UW last_sup, last_mot, t_sup, t_mot, t_hb_seen[2],
    last_hb_seen[2];
LOCAL volatile UW   buzz_bucket, pin_bad_ms, buzz_on_ms_total;
LOCAL volatile BOOL frozen, started[2], armed, want_reboot, live_printed;
LOCAL volatile UW   t_frozen;
/* time for a live task to print the record */
#define REBOOT_DELAY_US 2500000U
LOCAL volatile ID id_sup, id_mot, id_test;

IMPORT UW exchdr_tbl[]; /* the kernel's RAM vector table */

EXPORT void diag_pm_isr(UW * frame);
EXPORT void diag_pm_hardfault(UW * frame);

/* Raw entry: find the interrupted frame (MSP or PSP), call C, return. */
__asm__("\t.syntax unified\t\t\t\n"
        "\t.thumb\t\t\t\t\n"
        "\t.text\t\t\t\t\n"
        "\t.align\t2\t\t\t\n"
        "\t.thumb_func\t\t\t\n"
        "diag_pm_vec:\t\t\t\n"
        "\tmov\tr0, lr\t\t\t\n"
        "\tmovs\tr1, #4\t\t\t\n"
        "\ttst\tr0, r1\t\t\t\n"
        "\tbeq\t1f\t\t\t\n"
        "\tmrs\tr0, psp\t\t\t\n"
        "\tb\t2f\t\t\t\n"
        "1:\tmrs\tr0, msp\t\t\t\n"
        "2:\tpush\t{r4, lr}\t\t\n"
        "\tbl\tdiag_pm_isr\t\t\n"
        "\tpop\t{r4, pc}\t\t\n"
        "\t.thumb_func\t\t\t\n"
        "diag_pm_hf_vec:\t\t\t\n"
        "\tmov\tr0, lr\t\t\t\n"
        "\tmovs\tr1, #4\t\t\t\n"
        "\ttst\tr0, r1\t\t\t\n"
        "\tbeq\t3f\t\t\t\n"
        "\tmrs\tr0, psp\t\t\t\n"
        "\tb\t4f\t\t\t\n"
        "3:\tmrs\tr0, msp\t\t\t\n"
        "4:\tbl\tdiag_pm_hardfault\t\n"
        "5:\tb\t5b\t\t\t\n");
IMPORT void diag_pm_vec(void);
IMPORT void diag_pm_hf_vec(void);

LOCAL UW cur_core (void)
{
    return hal_core_id();
}

LOCAL ID task_id (TCB * t)
{
    return (t == NULL) ? 0 : t->tskid;
}

LOCAL void safe_outputs (void)
{
    hal_pwm_set(PIN_MOTOR_L_A, 0);
    hal_pwm_set(PIN_MOTOR_L_B, 0);
    hal_pwm_set(PIN_MOTOR_R_A, 0);
    hal_pwm_set(PIN_MOTOR_R_B, 0);
    hal_pwm_set(PIN_BUZZER, 0);
}

LOCAL void save_stack (INT slot, ID id)
{
    TCB * t;
    UW *  sp;
    INT   i;

    pm->stk_id[slot] = (UW) id;
    if (id <= 0 || id > PM_NT)
    {
        return;
    }
    t  = &knl_tcb_table[id - 1];
    sp = (UW *) t->tskctxb.ssp;
    if (sp == NULL || (UW) sp < 0x20000000U || (UW) sp >= 0x20040000U)
    {
        return;
    }
    sp += sizeof(PM_SAVED_CTX) / sizeof(UW); /* above the saved context */
    for (i = 0; i < PM_NW && (UW) (sp + i) < 0x20040000U; i++)
    {
        pm->stk[slot][i] = sp[i];
    }
}

LOCAL void freeze (UW reason, UW core, UW now)
{
    INT i, n = 0;
    UW  reboots;

    if (frozen)
    {
        return;
    }
    frozen = TRUE;

    pm->reason      = reason;
    pm->core        = core;
    pm->t_us        = now;
    pm->sup_runs    = wcet_tab[WCET_SUPERVISOR].runs;
    pm->motion_runs = wcet_tab[WCET_MOTION].runs;
    pm->hb[0]       = hb[0];
    pm->hb[1]       = hb[1];
    pm->buzzer = hal_pwm_get(PIN_BUZZER) | (hal_gpio_funcsel(PIN_BUZZER) << 16);
    pm->mot[0] = hal_pwm_get(PIN_MOTOR_L_A);
    pm->mot[1] = hal_pwm_get(PIN_MOTOR_L_B);
    pm->mot[2] = hal_pwm_get(PIN_MOTOR_R_A);
    pm->mot[3] = hal_pwm_get(PIN_MOTOR_R_B);

    safe_outputs();

    for (i = 1; i <= PM_NT; i++)
    {
        TCB *              t = &knl_tcb_table[i - 1];
        volatile PM_TASK * r = &pm->t[n];
        UINT               c, run = 0;

        if (t->state == TS_NONEXIST)
        {
            continue;
        }
        for (c = 0; c < TK_MAX_CORE; c++)
        {
            if (*knl_ctxtsk_core_slot(c) == t)
            {
                run = c + 1;
            }
        }
        r->id    = (UB) i;
        r->state = t->state;
        /* internal 0-based -> tk_cre_tsk value */
        r->pri = (UB) (t->priority + 1);
        r->run = (UB) run;
        r->tskwait =
            (t->wspec != NULL && (t->state & TS_WAIT)) ? t->wspec->tskwait : 0;
        r->wid    = (UH) t->wid;
        r->wupcnt = (H) t->wupcnt;
        r->pc = r->lr = 0;
        if (!run && t->state != TS_DORMANT && t->tskctxb.ssp != NULL)
        {
            PM_SAVED_CTX * f = (PM_SAVED_CTX *) t->tskctxb.ssp;
            if ((UW) f >= 0x20000000U && (UW) f < 0x20040000U)
            {
                r->pc = (UW) f->pc;
                r->lr = (UW) f->lr;
            }
        }
        n++;
    }
    for (; n < PM_NT; n++)
    {
        pm->t[n].id = 0;
    }

    save_stack(0, id_sup);
    save_stack(1, id_mot);
    save_stack(2, id_test);

    pm->nmagic = ~PM_MAGIC;
    pm->magic  = PM_MAGIC; /* record complete */

    /* Reboot 2.5 s from now (the alarm does it), so that a task that
       still runs can print the record live first (diag_pm_live_report).
       After MAX_REBOOTS in a row: stay up with the outputs off, so the
       record can still be read live or after a RUN-pin reset. */
    reboots = hal_scratch_get(0);
    if (reboots < MAX_REBOOTS)
    {
        hal_scratch_set(0, reboots + 1);
        hal_scratch_set(1, PM_MAGIC); /* "this reboot was the watch" */
        t_frozen    = now;
        want_reboot = TRUE;
    }
}

EXPORT void diag_pm_isr (UW * frame)
{
    UW core = cur_core() & 1U;
    UW now  = hal_now_us();
    UW due  = next_due[core], late, next;

    hal_alarm_ack(2 + core);
    late = now - due;
    if ((W) late < 0)
    {
        late = 0;
    }

    next = due + 1000U;
    if ((W) (next - now) < 50)
    {
        next = now + 1000U;
    }
    next_due[core] = next;
    hal_alarm_set(2 + core, next);

    hb[core]++;
    if (frozen)
    {
        safe_outputs(); /* a live loop must not restart them */
        if (want_reboot && (now - t_frozen) > REBOOT_DELAY_US)
        {
            hal_reboot();
        }
        return;
    }

    if (late > pm->late_max_us[core] && late < 10000000U)
    {
        pm->late_max_us[core] = late;
        pm->late_pc[core]     = frame[6];
    }

    if (++tick_n[core] >= SAMPLE_EVERY)
    {
        UW    h      = pm->head[core];
        TCB * t      = *knl_ctxtsk_core_slot(core < TK_MAX_CORE ? core : 0);
        tick_n[core] = 0;
        pm->s[core][h].pc   = frame[6];
        pm->s[core][h].lr   = frame[5];
        pm->s[core][h].info = (frame[7] & 0x3FU) |
                              ((UW) (task_id(t) & 0xFF) << 8) |
                              ((now / 1000U) << 16);
        pm->head[core] = (h + 1U) % PM_NS;
    }

    if (!armed)
    {
        return;
    }

    if (core == 0)
    {
        UW s = wcet_tab[WCET_SUPERVISOR].runs, m = wcet_tab[WCET_MOTION].runs;
        if (s != last_sup)
        {
            last_sup = s;
            t_sup    = now;
        }
        if (m != last_mot)
        {
            last_mot = m;
            t_mot    = now;
        }
        if ((now - t_sup) > LOOP_LIMIT_US)
        {
            freeze(R_SUP, core, now);
            return;
        }
        if ((now - t_mot) > LOOP_LIMIT_US)
        {
            freeze(R_MOTION, core, now);
            return;
        }

        /* buzzer mostly on for ~3 s (leaky bucket: +2 on, -1 off), so a
           chain of short beeps trips it too; one 800 ms beep does not */
        if (hal_pwm_get(PIN_BUZZER) != 0)
        {
            buzz_on_ms_total++;
            buzz_bucket += 2;
            if (buzz_bucket > 6000)
            {
                freeze(R_BUZZER, core, now);
                return;
            }
        }
        else if (buzz_bucket > 0)
        {
            buzz_bucket--;
        }
        /* GP22 must stay a PWM output (funcsel 4) */
        if (hal_gpio_funcsel(PIN_BUZZER) != 4)
        {
            if (++pin_bad_ms > 100)
            {
                freeze(R_PIN, core, now);
                return;
            }
        }
        else
        {
            pin_bad_ms = 0;
        }
    }
#if TK_SUPPORT_SMP
    /* the other core's alarm must keep running */
    {
        UW other = core ^ 1U;
        if (started[other])
        {
            if (hb[other] != last_hb_seen[core])
            {
                last_hb_seen[core] = hb[other];
                t_hb_seen[core]    = now;
            }
            else if ((now - t_hb_seen[core]) > CORE_LIMIT_US)
            {
                freeze(other == 0 ? R_CORE0 : R_CORE1, core, now);
            }
        }
    }
#endif
}

EXPORT void diag_pm_hardfault (UW * frame)
{
    UW core = cur_core() & 1U;

    if (!frozen)
    {
        pm->fault_pc   = frame[6];
        pm->fault_lr   = frame[5];
        pm->fault_psr  = frame[7];
        pm->fault_core = core;
    }
    freeze(R_HARDFAULT, core, hal_now_us());
    safe_outputs();
#if TK_SUPPORT_SMP
    /* the other core's alarm reboots in 2.5 s; its tasks may print first */
    for (;;)
    {
        __asm volatile("nop");
    }
#else
    if (want_reboot)
    {
        hal_reboot(); /* the only core is stuck in here */
    }
    for (;;)
    {
        __asm volatile("nop");
    }
#endif
}

/* ------------------------------------------------------------------ */
/* Boot: keep last run's record, start a fresh one.  From usermain, first. */
EXPORT void diag_pm_boot (void)
{
    INT i;

    hal_alarm_irq_all_off(); /* nothing left over from before a reboot */
    prev_cause = hal_reset_cause();
    if (pm->magic == PM_MAGIC && pm->nmagic == ~PM_MAGIC)
    {
        UB *                d = (UB *) &prev;
        const volatile UB * s = (const volatile UB *) pm;
        for (i = 0; i < (INT) sizeof(PM); i++)
        {
            d[i] = s[i];
        }
        have_prev = TRUE;
    }
    if (hal_scratch_get(1) != PM_MAGIC)
    {
        hal_scratch_set(0, 0); /* not our reboot: reset the count */
    }
    hal_scratch_set(1, 0);
    {
        volatile UB * p = (volatile UB *) pm;
        for (i = 0; i < (INT) sizeof(PM); i++)
        {
            p[i] = 0;
        }
    }
    /* HardFault: ours records and reboots (the kernel's report would only
       reach the console if core 0 still runs USB, which it often does not) */
    exchdr_tbl[3] = (UW) diag_pm_hf_vec;
}

EXPORT void diag_pm_name (ID tid, const char * name)
{
    const char *a = name, *b;

    b = "supervisor";
    while (*a && *a == *b)
    {
        a++;
        b++;
    }
    if (*a == *b)
    {
        id_sup = tid;
        return;
    }
    a = name;
    b = "motion";
    while (*a && *a == *b)
    {
        a++;
        b++;
    }
    if (*a == *b)
    {
        id_mot = tid;
        return;
    }
    a = name;
    b = "usermain/test";
    while (*a && *a == *b)
    {
        a++;
        b++;
    }
    if (*a == *b)
    {
        id_test = tid;
        return;
    }
}

/* Start the watch on the CALLING core (NVIC is per core).  Core 0: from
   usermain after the tasks start.  Core 1: from a task bound to core 1
   (the Supervisor calls it once).  Arms the loop checks from now. */
EXPORT void diag_pm_start_here (void)
{
    T_DINT     d;
    UW         core = cur_core() & 1U, now = hal_now_us();
    LOCAL BOOL vec_done;

    if (started[core])
    {
        return;
    }
    if (!vec_done)
    {
        d.intatr = TA_ASM;
        d.inthdr = (FP) diag_pm_vec;
        (void) tk_def_int(2, &d);
        (void) tk_def_int(3, &d);
        vec_done = TRUE;
    }
    t_sup = t_mot = now;
    t_hb_seen[0] = t_hb_seen[1] = now;
    next_due[core]              = now + 1000U;
    hal_alarm_set(2 + core, next_due[core]);
    hal_alarm_irq_on_this_core(2 + core);
    started[core] = TRUE;
#if TK_SUPPORT_SMP
    if (core == 0)
    {
        armed = TRUE; /* the loops run on core 1: never starved */
    }
#endif
}

/* Single core: the CYW43 bring-up (priority 4) holds the only core for
   seconds, so the loop checks start once it is done (telemetry calls this) */
EXPORT void diag_pm_arm (void)
{
    if (!armed && started[0])
    {
        UW now = hal_now_us();
        t_sup = t_mot = now;
        armed         = TRUE;
    }
}

/* ------------------------------------------------------------------ */
LOCAL const char * reason_text (UW r)
{
    switch (r)
    {
        case R_SUP:
            return "SUPERVISOR loop made no run for 1.5 s";
        case R_MOTION:
            return "MOTION loop made no run for 1.5 s";
        case R_CORE0:
            return "CORE 0 stopped taking interrupts for 0.5 s";
        case R_CORE1:
            return "CORE 1 stopped taking interrupts for 0.5 s";
        case R_HARDFAULT:
            return "HARD FAULT";
        case R_BUZZER:
            return "BUZZER (GP22 PWM) mostly on for 3 s";
        case R_PIN:
            return "GP22 (buzzer) is no longer a PWM pin: something else took "
                   "it";
        default:
            return "?";
    }
}

LOCAL void line_pause (void)
{
    tk_dly_tsk(15); /* let USB drain: the console ring is 4 KiB */
}

LOCAL void print_record (const char * when)
{
    INT c, i;

    tm_printf((UB *) "[pm] *** %s: %s (seen by core %d at %d ms after boot)\n",
              when, reason_text(prev.reason), (INT) prev.core,
              (INT) (prev.t_us / 1000U));
    line_pause();
    tm_printf((UB *) "[pm] runs sup %d motion %d | alarm beats c0 %d c1 %d | "
                     "buzzer was %d (pin func %d)"
                     " | motors L %d/%d R %d/%d (all switched off)\n",
              (INT) prev.sup_runs, (INT) prev.motion_runs, (INT) prev.hb[0],
              (INT) prev.hb[1], (INT) (prev.buzzer & 0xFFFF),
              (INT) (prev.buzzer >> 16), (INT) prev.mot[0], (INT) prev.mot[1],
              (INT) prev.mot[2], (INT) prev.mot[3]);
    tm_printf((UB *) "[pm] longest interrupts-off: c0 %d us (ended at pc %08x) "
                     " c1 %d us (pc %08x)\n",
              (INT) prev.late_max_us[0], (INT) prev.late_pc[0],
              (INT) prev.late_max_us[1], (INT) prev.late_pc[1]);
    if (prev.reason == R_HARDFAULT)
    {
        tm_printf((UB *) "[pm] fault on core %d: pc %08x lr %08x psr %08x\n",
                  (INT) prev.fault_core, (INT) prev.fault_pc,
                  (INT) prev.fault_lr, (INT) prev.fault_psr);
    }
    line_pause();
    tm_printf((UB *) "[pm] tasks: id pri state(1 rdy/run 2 wait 4 sus 8 dmt) "
                     "run-on-core wait-factor(hex) obj wup | saved pc lr\n");
    for (i = 0; i < PM_NT; i++)
    {
        PM_TASK * t = &prev.t[i];
        if (t->id == 0)
        {
            continue;
        }
        tm_printf((UB *) "[pm]  id %2d pri %2d st %d core %d wait %04x obj %d "
                         "wup %d | %08x %08x\n",
                  (INT) t->id, (INT) t->pri, (INT) t->state, (INT) t->run - 1,
                  (INT) t->tskwait, (INT) t->wid, (INT) t->wupcnt, (INT) t->pc,
                  (INT) t->lr);
        line_pause();
    }
    for (c = 0; c < 2; c++)
    {
        tm_printf((UB *) "[pm] core %d last samples, newest first: pc/lr exc "
                         "task @ms\n",
                  c);
        for (i = 0; i < PM_NS; i += 3)
        {
            INT k, j;
            tm_printf((UB *) "[pm]  c%d", c);
            for (k = 0; k < 3; k++)
            {
                PM_SAMPLE * s;
                j = ((INT) prev.head[c] - 1 - (i + k) + 2 * PM_NS) % PM_NS;
                s = &prev.s[c][j];
                tm_printf((UB *) "  %08x/%08x %d t%d @%d", (INT) s->pc,
                          (INT) s->lr, (INT) (s->info & 0x3F),
                          (INT) ((s->info >> 8) & 0xFF), (INT) (s->info >> 16));
            }
            tm_printf((UB *) "\n");
            line_pause();
        }
    }
    for (c = 0; c < PM_NSTK; c++)
    {
        if (prev.stk_id[c] == 0)
        {
            continue;
        }
        tm_printf((UB *) "[pm] stack of task %d:", (INT) prev.stk_id[c]);
        for (i = 0; i < PM_NW; i++)
        {
            tm_printf((UB *) " %08x", (INT) prev.stk[c][i]);
        }
        tm_printf((UB *) "\n");
        line_pause();
    }
    tm_printf((UB *) "[pm] end of record\n");
}

EXPORT void diag_pm_print (void)
{
    tm_printf((UB *) "[pm] reset cause:%s%s%s%s\n",
              (prev_cause & 1U) ? " power-on" : "",
              (prev_cause & 2U) ? " RUN-pin" : "",
              (prev_cause & 4U) ? " watchdog" : "",
              prev_cause ? "" : " (none recorded)");
    if (!have_prev)
    {
        tm_printf((UB *) "[pm] no post-mortem record from the previous run\n");
        return;
    }
    print_record("PREVIOUS RUN STOPPED");
}

/* Called often from tasks that may still run (Telemetry on core 0, the
   Supervisor's watcher on core 1): prints a fresh record once, live. */
EXPORT void diag_pm_live_report (void)
{
    INT i;

    if (!frozen || live_printed || pm->magic != PM_MAGIC)
    {
        return;
    }
    live_printed = TRUE;
    {
        UB *                d = (UB *) &prev;
        const volatile UB * s = (const volatile UB *) pm;
        for (i = 0; i < (INT) sizeof(PM); i++)
        {
            d[i] = s[i];
        }
    }
    have_prev = TRUE;
    tm_printf((UB *) "\n[pm] watch tripped; motors and buzzer are OFF%s\n",
              want_reboot ? "; rebooting in 2.5 s"
                          : " (reboot limit reached: staying up)");
    print_record("STOPPED NOW");
}

/* One status line: proves the watch is alive, and the buzzer pin's real
   state (compare level, pin function, whether the pad is toggling) */
EXPORT void diag_pm_line (void)
{
    UW sl = hal_pwm_slice_of(PIN_BUZZER);

    tm_printf((UB *) "[watch] beats c0 %d c1 %d armed %d | irq-off max c0 %d "
                     "us c1 %d us"
                     " | buzzer cc %d func %d edges %d/200 slice%d en %d top "
                     "%d div %d on-ms %d\n",
              (INT) hb[0], (INT) hb[1], (INT) armed, (INT) pm->late_max_us[0],
              (INT) pm->late_max_us[1], (INT) hal_pwm_get(PIN_BUZZER),
              (INT) hal_gpio_funcsel(PIN_BUZZER),
              (INT) hal_gpio_edges(PIN_BUZZER, 200, 7), (INT) sl,
              (INT) (hal_pwm_csr(sl) & 1U), (INT) hal_pwm_top(sl),
              (INT) hal_pwm_div(sl), (INT) buzz_on_ms_total);
}

/*** end of file ***/
