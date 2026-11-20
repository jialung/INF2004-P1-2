/*
 *	diag.h
 *	Run-time diagnostics shared by every subsystem.        Owner: Jia Lung
 *
 *	Added after the 23 Oct bench runs, where opening the serial monitor
 *	early (tests overlapping the CYW43 Wi-Fi bring-up) left core 0's
 *	low-priority tasks without CPU for seconds and the buzzer on.  These
 *	print what each task is doing at that moment instead of guessing.
 *
 *	  diag_task_dump(why)   one line per task: priority, state, what it
 *	                        waits for.  RUN = on a core now; RDY = ready
 *	                        but no core free for it (starved).
 *	  diag_kernel_line()    the "[diag]" counter line (tick, cross-core
 *	                        callbacks, kernel lock hold/wait, GPIO IRQ,
 *	                        console drops, buzzer level, Wi-Fi state).
 *	  diag_core0_watch()    call from a CORE 1 loop (the Supervisor does):
 *	                        when core 0's imu/obstacle/telemetry tasks have
 *	                        not run for 300 ms, dumps the task states once
 *	                        per episode, i.e. while the hog is still there.
 *	  diag_name_task()      app_start_task() calls it, so the dump can say
 *	                        "motion" instead of "id 7".
 *
 *	All of it prints with tm_printf: task context only, never from an ISR.
 */

#ifndef DIAG_H
#define DIAG_H

#include <tk/tkernel.h>

IMPORT void diag_name_task(ID tid, const char *name);
IMPORT void diag_task_dump(const char *why);
IMPORT void diag_kernel_line(void);
IMPORT void diag_core0_watch(void);

/* Post-mortem watch (diag_pm.c): see the comment at the top of that file */
IMPORT void diag_pm_boot(void);		/* usermain, first thing */
IMPORT void diag_pm_start_here(void);	/* once on each core */
IMPORT void diag_pm_arm(void);		/* single core: after Wi-Fi bring-up */
IMPORT void diag_pm_name(ID tid, const char *name);
IMPORT void diag_pm_print(void);	/* last run's record, if any */
IMPORT void diag_pm_line(void);		/* "[watch]" status line */
IMPORT void diag_pm_live_report(void);	/* tasks that may still run call this often */

#endif /* DIAG_H */
