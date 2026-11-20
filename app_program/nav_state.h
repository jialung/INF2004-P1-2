/*
 *	nav_state.h
 *	Shared navigation state: one writer per half, many readers.
 *	Owners: Jia Lung (motion half), Jian Yi (IMU half).
 *	Plan refs: section 2.2, JL-1.10, JL-1.17, JL-1.18, JY-1.8, JY-1.11.
 *
 *	Rules
 *	- Motion Control writes ONLY nav_motion_write(); IMU writes ONLY
 *	  nav_imu_write().  Nobody else writes.
 *	- Readers call nav_read() and work on their own copy.  The mutex is
 *	  held only for the struct copy, never while computing.
 *	- Never call these from an interrupt or cyclic handler (mutex).
 *	- Units are fixed point.  Widths follow the plan's packed fields so
 *	  telemetry can copy them without rescaling.
 */

#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <tk/tkernel.h>

typedef struct {			/* written by Motion Control (Jia Lung) */
	UH	odo_mm;			/* 16-bit wrapping distance, mm (JL-1.17).
					   Use (UH)(now - then) for intervals.   */
	H	v_mm_s;			/* signed forward speed, mm/s             */
	H	a_mm_s2;		/* forward acceleration (tilt comp.)      */
	H	turn_q;			/* encoder heading change since boot,
					   0.25 deg units, wraps                  */
	W	x_mm;			/* dead-reckoned position (JL-1.18)       */
	W	y_mm;
	UB	motion_state;		/* Motion Control's own state (JL-1.16)   */
	UB	enc_ok;			/* 1 = both encoders counting             */
	UW	t_us;			/* when this half was written             */
} NAV_MOTION;

typedef struct {			/* written by IMU (Jian Yi) */
	UH	heading_deg;		/* 0..359, tilt compensated (9 bits)      */
	B	roll_q;			/* 0.25 deg, +-31 deg (JY-1.11)            */
	B	pitch_q;
	UB	terrain;		/* hump statechart state (3 bits)         */
	UB	imu_ok;			/* 1 = last read good                     */
	UW	t_us;
} NAV_IMU;

typedef struct {
	NAV_MOTION	m;
	NAV_IMU		i;
} NAV_STATE;

IMPORT ER   nav_state_init(void);			/* once, before tasks */
IMPORT void nav_motion_write(const NAV_MOTION *m);	/* Motion Control only */
IMPORT void nav_imu_write(const NAV_IMU *i);		/* IMU only */
IMPORT void nav_read(NAV_STATE *out);			/* anyone (task ctx) */

#endif /* NAV_STATE_H */
