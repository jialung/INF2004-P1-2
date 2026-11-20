/*
 *	line_search.h
 *	LineSearch logic, end to end.  Owner: Ryan.  Called by the Supervisor
 *	in Running.LineSearch; never touches the motors itself.
 *	Plan refs: chain 4.6, RY-1.14..RY-1.16, R9, D3.
 *
 *	Driving: the Supervisor calls line_search_start() on entry, then
 *	line_search_step() on every EV_LINE_SAMPLE (Line Following sends one
 *	every 5 ms while the state is SYS_LINE_SEARCH) and on every
 *	EV_MOTION_DONE.  Each call returns what to do next:
 *	  LS_WAIT   nothing new, keep waiting
 *	  LS_MOVE   send result.cmd to Motion Control
 *	  LS_FOUND  line confirmed: Supervisor -> LineFollow
 *	  LS_FAIL   search distance S used up: LINE_SEARCH_FAIL was raised
 */

#ifndef LINE_SEARCH_H
#define LINE_SEARCH_H

#include "../app.h"
#include "../msg_types.h"

/* D3 starting values (tune in RY-2.4) */
#define LS_S1_MM		50	/* creep distance per step */
#define LS_S_MM			300	/* total search distance */
#define LS_ALPHA_DEG		20	/* sweep half-angle */
#define LS_BETA_DEG		20	/* accept window around the saved heading */
#define LS_CONFIRM_MM		10	/* line seen this far before "found" */
#define LS_CONFIRM_SAMPLES	3	/* ... or this many 5 ms samples in a row
					   (pivots: the odometer hardly moves) */

typedef enum {
	LS_REASON_LINE_LOST = 0,
	LS_REASON_AFTER_BYPASS,
	LS_REASON_AFTER_TURN
} LS_REASON;

typedef struct {
	UH	heading_deg;		/* saved line heading (compass) */
	UH	odo_mm;			/* odometer at the reference */
	H	expected_offset_mm;	/* + = line expected to the left */
} LS_REF;

typedef enum { LS_WAIT = 0, LS_MOVE, LS_FOUND, LS_FAIL } LS_KIND;

typedef struct {
	LS_KIND	kind;
	UW	cmd;			/* MOTION_CMD when kind == LS_MOVE */
} LS_RESULT;

/* Search state (3 bits, RY-1.13), for telemetry */
typedef enum {
	LSS_IDLE = 0, LSS_CREEP, LSS_SWEEP_L, LSS_SWEEP_R, LSS_CONFIRM,
	LSS_CENTRE, LSS_FAIL
} LS_STATE;

IMPORT void      line_search_start(LS_REASON reason, const LS_REF *ref);
IMPORT LS_RESULT line_search_step(EV_CODE code, UW a, UW odo_mm);
IMPORT LS_STATE  line_search_state(void);

#endif /* LINE_SEARCH_H */
