/*
 *	subsystem1.h
 *	Subsystem 1 -- Motion Control.  Owner: Jia Lung (Real-Time Taskmaster).
 *	Public interface only.  Plan section 6.
 *
 *	Inputs   mbf_motion_cmd (MOTION_CMD, msg_types.h), line error word,
 *	         fault flag (every cycle), nav_state heading (turns, D1)
 *	Outputs  motor PWM (sole writer), nav_state motion half,
 *	         EV_MOTION_DONE, MOTOR_STALL / ENC_FAULT bits
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

IMPORT ID tid_motion;

/* For the fault-cut test (JL-1.33): when Motion Control first saw a stop
   bit, and the time it was raised by the test. */
IMPORT volatile UW motion_cut_t_us;

/* Motion Control's own state, in nav_state.m.motion_state (JL-1.16) */
typedef enum {
	MS_IDLE = 0,	/* braked, no command */
	MS_FOLLOW,	/* line following */
	MS_DRIVE,	/* fixed distance */
	MS_TURN,	/* pivot */
	MS_CUT,		/* stopped by the fault flag */
	MS_STATE_COUNT
} MOTION_STATE;

/* Encoder direction: +1 or -1 so that driving FORWARD counts UP on both
   wheels.  Set from test S1-U05 (prints the raw counts).  JL-1.3
   Bench 23 Oct: forward 400 ms gave raw left -962, raw right +960. */
#define ENC_SIGN_L		(-1)
#define ENC_SIGN_R		(+1)

/* Private to subsystem 1 */
IMPORT void motion_task(INT stacd, void *exinf);
IMPORT BOOL motion_hw_ok;

#endif /* SUBSYSTEM1_H */
