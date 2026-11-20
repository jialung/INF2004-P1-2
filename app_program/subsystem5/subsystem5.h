/*
 *	subsystem5.h
 *	Subsystem 5 -- Supervisor, Telemetry, buttons, RGB LED, buzzer.
 *	Owner: Christian (Network Gatekeeper).  Public interface only.  Plan section 5.
 *
 *	Supervisor  reads mbf_sup_event (its ONLY wait object), sends
 *	            mbf_motion_cmd / mbf_scan_req / mbf_telem, publishes
 *	            sys_state, runs line_search.c (Ryan's code) in LineSearch.
 *	Telemetry   500 ms: packs telem_frame, sets / clears WIFI_DOWN.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../msg_types.h"

IMPORT ID tid_supervisor;
IMPORT ID tid_telemetry;

/* Button timing (CH-1.7) */
#define BTN_DEBOUNCE_US		30000	/* ignore edges closer than this */
#define BTN_LONG_US		2000000	/* MODE held >= 2 s = long */
#define CALIBRATE_TIMEOUT_MS	20000	/* > the 15 s IR / IMU windows */

/* Calibrate auto spin.  1: after the IMU's 1 s still (level) phase the
   car pivots on the spot through more than a full circle, so nobody has
   to turn it by hand: the compass sees every heading, and the line and
   barcode sensors (130 mm ahead of the axle) sweep across the line.
   Place the car ON the line with ~25 cm free all round.  0: turn it by
   hand as before.  Turn commands are limited to +-254 deg, so the spin
   is CAL_SPIN_STEPS pivots of CAL_SPIN_STEP_DEG; 2 x 200 = 400 deg
   still covers a full circle if the encoder turn scale (TURN_TRACK_X10,
   not yet checked) is up to 10 % off.  ~2 s per 200 deg at turn speed. */
#define CAL_AUTO_SPIN		1
#define CAL_SPIN_DELAY_MS	1300	/* > IMU_CAL_LEVEL_MS + one 20 ms sample */
#define CAL_SPIN_STEP_DEG	200
/* 6 Nov: back to 2 steps.  With the corrected turn scale
   (TURN_TRACK_X10 1208) 2 x 200 = 400 deg is a real full circle plus
   ~10 % margin for the estimated scale.  (3 steps were a work-around
   while the encoders read turns ~1.8 x too high.) */
#define CAL_SPIN_STEPS		2U
/* a pivot with no Motion reply after this long: start the next one */
#define CAL_SPIN_STEP_TIMEOUT_MS 4000
/* Pending letter dropped after this many mm without a junction.
   0 = never: the demo stores the letter until the next cross-junction,
   and a new barcode replaces it. */
#define LETTER_DISCARD_MM	0

/* Junction turn: detect the cross, roll SENSOR_TO_AXLE_MM so the axle is
   over the crossing, STOP, wait this long standing still, then pivot fast
   (task_motion.c TURN_FAST_PM) and carry on line following.  The wait
   lets the car stop completely so the pivot starts from rest. */
#define JCT_SETTLE_MS		200

/* Compass check after a junction pivot (FR6, NFR4 +-5 deg) */
#define COMPASS_CORRECT		1	/* 0: only log the error */
#define COMPASS_TOL_DEG		5
#define COMPASS_MAX_FIX_DEG	30	/* bigger: compass not trusted */

/* Obstacle bypass: 50 deg side-step (FR11, 11 Nov).  The geometry
   constants are in bypass_geom.h. */
#define BYPASS_SPEED		SPD_HUMP	/* straight legs; Motion maps SPD_HUMP drives to DRIVE_BYPASS_PM (46 mm/s) */
#define BYPASS_TIMEOUT_MS	60000	/* whole bypass, from the stop until
					   LineFollow / LineSearch */
#define BYPASS_SIDE_DEFAULT	(+1)	/* +1 left, -1 right, after power-up */
#define BYPASS_EXTRA_STEP_MM	80	/* extra out-step when scan 2 or the
					   pass guard says blocked */
#define BYPASS_MAX_EXTRA	2	/* then FAULT_NO_BYPASS */
#define APPROACH_SLOW_MM	60	/* SPD_SLOW this far before the line
					   is expected */
#define APPROACH_OVER_MM	120	/* keep looking this far past it */
#define LINE_SEEN_POLLS		3	/* consecutive fresh line samples with
					   a sensor on black = line found */
#define LOST_LINE_OFFSET_MM	100	/* to line search when the line was
					   not found (only the sign matters) */
#define AVOID_BEEP_MS		150	/* "avoid left / right" or "depth" applied */

/* Obstacle width sweep (12 Nov, obst_width.h): the car stops at
   D_WIDTH_CM, measures, and goes on.  The sweep takes about 6 s (13 Nov:
   slower, smoother servo moves). */
#define WIDTH_TIMEOUT_MS	15000	/* no sweep result: carry on anyway */
#define WM_CAL_MIN_MM		20	/* "wcal": the known box width */
#define WM_CAL_MAX_MM		300
#define WM_EDGE_MIN_MM		(-150)	/* "wcal <l> <r>": one edge from the centre */
#define WM_EDGE_MAX_MM		250	/* line, + = out on its own side */

/* Barcode -> junction command mapping, 2 bits per letter (FR3, CH-1.13) */
typedef enum { MAP_FORWARD = 0, MAP_LEFT, MAP_RIGHT, MAP_UTURN } MAP_CMD;

/* Counters for telemetry and tests */
typedef struct {
	UW	events;
	UW	letters_dropped;
	UW	transitions;
	UW	humps;
	UB	last_letter;		/* 31 = none */
	UB	last_letter_rev;
} SUP_STATS;
IMPORT volatile SUP_STATS sup_stats;

/* Read-only views of the Supervisor's state, for the snapshot */
IMPORT void sup_map_get(UB out[26]);	/* the active mapping */
IMPORT INT  sup_pending_letter(void);	/* 0..25, -1 = none held */
IMPORT INT  sup_avoid_side(void);	/* +1 left, -1 right (PC setting) */
IMPORT INT  sup_bypass_depth(void);	/* obstacle depth mm (PC setting) */
IMPORT INT  sup_obstacle_width_mm(void);	/* last measured obstacle width, 0 = none */

/* Private to subsystem 5 */
IMPORT void supervisor_task(INT stacd, void *exinf);
IMPORT void telemetry_task(INT stacd, void *exinf);
IMPORT ER   buttons_init(void);
IMPORT BOOL ui_led_ok;

#endif /* SUBSYSTEM5_H */
