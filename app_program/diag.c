/*
 *	diag.c
 *	Owner: Jia Lung.  See diag.h.
 */

#include "app.h"
#include "diag.h"
#include "wcet.h"
#include "gpio_irq.h"
#include "hw_alloc.h"
#include "msg_bus.h"
#include "hal.h"
#include "cyw43_utk.h"		/* template header, plain stdint: kernel-side safe */

#define DIAG_MAX_TSKID		32	/* CNF_MAX_TSKID in config/config.h */
#define WATCH_LIMIT_US		300000U	/* core 0 silent this long -> dump */
#define WATCH_MAX_EPISODES	5	/* then stay quiet */

LOCAL const char *task_name[DIAG_MAX_TSKID + 1];

EXPORT void diag_name_task(ID tid, const char *name)
{
	if (tid > 0 && tid <= DIAG_MAX_TSKID) {
		task_name[tid] = name;
	}
	diag_pm_name(tid, name);
}

/* Tasks the application did not create, recognised by their priority */
LOCAL const char *guess_name(PRI pri)
{
	switch (pri) {
	case 1:  return "usermain/test";
	case 3:  return "usb console";
	case 4:  return "cyw43 wifi";
	default: return "?";
	}
}

LOCAL const char *state_name(UINT s)
{
	switch (s & 0x1FU) {
	case TTS_RUN: return "RUN";
	case TTS_RDY: return "RDY";
	case TTS_WAI: return "WAI";
	case TTS_SUS: return "SUS";
	case TTS_WAS: return "WAS";
	case TTS_DMT: return "DMT";
	default:      return "???";
	}
}

LOCAL const char *wait_name(UW w)
{
	if (w & TTW_SLP)  return "sleep";
	if (w & TTW_DLY)  return "delay";
	if (w & TTW_SEM)  return "sem";
	if (w & TTW_FLG)  return "flag";
	if (w & TTW_MBX)  return "mbx";
	if (w & TTW_MTX)  return "MUTEX";
	if (w & TTW_SMBF) return "mbf-SEND";
	if (w & TTW_RMBF) return "mbf-recv";
	return "other";
}

EXPORT void diag_task_dump(const char *why)
{
	T_RTSK	r;
	ID	id;

	tm_printf((UB*)"[tasks] --- %s (printed by a task on core %d) ---\n",
		  why, (INT)hal_core_id());
	for (id = 1; id <= DIAG_MAX_TSKID; id++) {
		if (tk_ref_tsk(id, &r) != E_OK || (r.tskstat & 0x1FU) == TTS_DMT) {
			continue;
		}
		if ((r.tskstat & 0x1FU) == TTS_WAI || (r.tskstat & 0x1FU) == TTS_WAS) {
			tm_printf((UB*)"[tasks] id %2d %-14s pri %2d %s %s (obj %d) wup %d\n",
				  (INT)id, task_name[id] ? task_name[id] : guess_name(r.tskpri),
				  (INT)r.tskpri, state_name(r.tskstat),
				  wait_name(r.tskwait), (INT)r.wid, (INT)r.wupcnt);
		} else {
			tm_printf((UB*)"[tasks] id %2d %-14s pri %2d %s  wup %d\n",
				  (INT)id, task_name[id] ? task_name[id] : guess_name(r.tskpri),
				  (INT)r.tskpri, state_name(r.tskstat), (INT)r.wupcnt);
		}
	}
}

IMPORT volatile UW tm_usb_dropped;	/* libtm usb_console.c: bytes lost, ring full */

#if TK_SUPPORT_SMP
/* Read-only mirrors of kernel diagnostics (kernel/tkernel/timer.h,
   kernel/knlinc/smp_lock.h, smp_rp2040.c, libtm_printf.c) */
typedef struct {
	UW ticks, wrong_core_ticks, expired_events, max_expired_per_tick;
	UW handler_enqueued[2], handler_executed[2], callback_max_us[2];
	UW callback_detached[2], callback_reconciled[2], callback_stale[2];
} KNL_TIMER_DIAG_MIRROR;
typedef struct {
	UW acquisitions, contentions, recursive_entries, max_depth;
	UW max_hold_us, max_wait_us, detached_sections, illegal_suspends;
	UW illegal_unlocks, dead_ctxtsk_dispatches, owner, depth[2];
} KNL_LOCK_DIAG_MIRROR;
IMPORT KNL_TIMER_DIAG_MIRROR knl_timer_diag;
IMPORT void knl_kernel_lock_audit(KNL_LOCK_DIAG_MIRROR *diag);
IMPORT volatile UW knl_smp_fifo_errors;
IMPORT volatile UW tm_printf_smp_contentions;

EXPORT void diag_kernel_line(void)
{
	T_CYW43_UTK_STATUS	ws;
	KNL_LOCK_DIAG_MIRROR	lk;

	cyw43_utk_get_status(&ws);
	knl_kernel_lock_audit(&lk);
	tm_printf((UB*)"[diag] tick %d | core1 cb %d/%d core0 cb %d/%d | fifo err %d"
		  " | kernel lock max hold %d us wait %d us | printf waits %d\n",
		  (INT)knl_timer_diag.ticks,
		  (INT)knl_timer_diag.handler_executed[1], (INT)knl_timer_diag.handler_enqueued[1],
		  (INT)knl_timer_diag.handler_executed[0], (INT)knl_timer_diag.handler_enqueued[0],
		  (INT)knl_smp_fifo_errors, (INT)lk.max_hold_us, (INT)lk.max_wait_us,
		  (INT)tm_printf_smp_contentions);
	tm_printf((UB*)"[diag] gpio irq %d spur %d (pin %d off 0x%x) | console lost %d B"
		  " | buzzer %d | wifi init %d/%d link %d\n",
		  (INT)gpio_irq_stats.count, (INT)gpio_irq_stats.spurious,
		  (INT)gpio_irq_stats.spurious_last_pin, (INT)gpio_irq_stats.killed_mask,
		  (INT)tm_usb_dropped, (INT)hal_pwm_get(PIN_BUZZER),
		  (INT)ws.init_complete, (INT)ws.ready, (INT)ws.link_status);
	diag_pm_line();
}

/* Called from the Supervisor (core 1) every wake-up, ~10 ms */
EXPORT void diag_core0_watch(void)
{
	LOCAL UW	last_runs, last_change_us;
	LOCAL BOOL	reported, primed;
	LOCAL INT	episodes;
	UW		now = now_us();
	UW		runs = wcet_tab[WCET_IMU].runs + wcet_tab[WCET_OBSTACLE].runs +
			       wcet_tab[WCET_TELEMETRY].runs;

	diag_pm_live_report();

	if (!primed || runs != last_runs) {
		if (reported) {
			tm_printf((UB*)"[diag] core 0 tasks running again after %d ms\n",
				  (INT)((now - last_change_us) / 1000U));
		}
		primed = TRUE;
		reported = FALSE;
		last_runs = runs;
		last_change_us = now;
		return;
	}
	if (!reported && episodes < WATCH_MAX_EPISODES &&
	    (now - last_change_us) > WATCH_LIMIT_US) {
		reported = TRUE;
		episodes++;
		tm_printf((UB*)"[diag] core 0: imu/obstacle/telemetry have not run for %d ms."
			  " RUN (other than this one) = what holds core 0:\n",
			  (INT)((now - last_change_us) / 1000U));
		diag_task_dump("core 0 busy");
		diag_kernel_line();
	}
}

#else	/* single core */

EXPORT void diag_kernel_line(void)
{
	T_CYW43_UTK_STATUS ws;

	cyw43_utk_get_status(&ws);
	tm_printf((UB*)"[diag] single core | gpio irq %d spur %d (pin %d off 0x%x)"
		  " | console lost %d B | buzzer %d | wifi init %d/%d link %d\n",
		  (INT)gpio_irq_stats.count, (INT)gpio_irq_stats.spurious,
		  (INT)gpio_irq_stats.spurious_last_pin, (INT)gpio_irq_stats.killed_mask,
		  (INT)tm_usb_dropped, (INT)hal_pwm_get(PIN_BUZZER),
		  (INT)ws.init_complete, (INT)ws.ready, (INT)ws.link_status);
	diag_pm_line();
}

EXPORT void diag_core0_watch(void)
{
	/* one core: a starved low-priority task starves the Supervisor too */
	diag_pm_live_report();
}

#endif
