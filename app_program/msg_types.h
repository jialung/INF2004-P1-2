/*
 *	msg_types.h
 *	Every message format between tasks, packed with shifts and masks.
 *	                       Owner: Christian merges; each owner writes fields
 *	Plan refs: section 2.2, CH-1.10, JL-1.15 (D1), RY-1.13, JY-1.11, LK-1.12
 *	Changed only by team agreement.  Unit test: tests/host/test_msg_types.c
 *
 *	Channels (created by msg_bus.c)
 *	  mbf_sup_event   everyone   -> Supervisor   SUP_EVENT (16 B)
 *	  mbf_motion_cmd  Supervisor -> Motion       MOTION_CMD (packed UW)
 *	  mbf_scan_req    Supervisor -> Obstacle     SCAN_REQ (packed UW)
 *	  mbf_telem       Supervisor -> Telemetry    TELEM_MSG (16 B)
 *	  line error      Line Following -> Motion   latest-value word, NOT a
 *	                  queue: Motion Control always wants the newest value,
 *	                  and a 5 ms producer / 10 ms consumer queue only fills.
 */

#ifndef MSG_TYPES_H
#define MSG_TYPES_H

#include <tk/tkernel.h>

/* ---------------------------------------------------------------------
 *  Bit-field helpers.  pos = lowest bit, w = width in bits.
 * --------------------------------------------------------------------- */
#define BF_MASK(w)		((w) >= 32 ? 0xFFFFFFFFUL : ((1UL << (w)) - 1UL))
#define BF_PUT(v, pos, w)	((((UW)(v)) & BF_MASK(w)) << (pos))
#define BF_GET(x, pos, w)	((((UW)(x)) >> (pos)) & BF_MASK(w))
/* Signed field: sign-extend a w-bit two's-complement value */
#define BF_GETS(x, pos, w)	((W)(BF_GET((x), (pos), (w)) ^ (1UL << ((w) - 1))) \
				 - (W)(1L << ((w) - 1)))

/* ---------------------------------------------------------------------
 *  System state (4 bits: 10 leaf states of the Supervisor statechart).
 *  Published by the Supervisor; read by anyone with sys_state_get().
 * --------------------------------------------------------------------- */
typedef enum {
	SYS_INIT = 0,
	SYS_IDLE,
	SYS_CALIBRATE,
	SYS_LINE_FOLLOW,		/* Running.LineFollow */
	SYS_JUNCTION_ACTION,		/* Running.JunctionAction */
	SYS_HUMP_TRAVERSE,		/* Running.HumpTraverse */
	SYS_OBSTACLE_AVOID,		/* Running.ObstacleAvoid */
	SYS_LINE_SEARCH,		/* Running.LineSearch */
	SYS_FINISHED,
	SYS_FAULT,
	SYS_STATE_COUNT
} SYS_STATE;
#define SYS_STATE_BITS		4
#define SYS_IS_RUNNING(s)	((s) >= SYS_LINE_FOLLOW && (s) <= SYS_LINE_SEARCH)

/* ---------------------------------------------------------------------
 *  Speed level (2 bits), shared by motion commands and telemetry
 * --------------------------------------------------------------------- */
typedef enum { SPD_STOP = 0, SPD_SLOW, SPD_HUMP, SPD_CRUISE } SPEED_LEVEL;

/* =====================================================================
 *  1. SUP_EVENT  (everyone -> Supervisor), 16 bytes
 * ===================================================================== */
typedef enum {
	SRC_SYSTEM = 0, SRC_MOTION, SRC_LINE, SRC_BARCODE, SRC_IMU,
	SRC_OBSTACLE, SRC_SUPERVISOR, SRC_TELEMETRY, SRC_BUTTON
} EV_SRC;

typedef enum {
	/* payload layout:          word a                       word b       */
	EV_NONE = 0,
	EV_FAULT,		/* fault bits just raised                -        */
	EV_BUTTON_EDGE,		/* BTN_A_*                               -        */
	EV_CALIB_DONE,		/* CAL_A_*                               -        */
	EV_LINE_LOST,		/* odometer mm                           -        */
	EV_LINE_FOUND,		/* odometer mm                           -        */
	EV_LINE_SAMPLE,		/* LS_A_* (LineSearch tick, droppable)   odometer */
	EV_JUNCTION,		/* JCT_A_* type                          odometer */
	EV_BARCODE,		/* BC_A_* letter, direction              odometer */
	EV_BARCODE_MISREAD,	/* misread count                         odometer */
	EV_HUMP_CLIMB,		/* roll at trigger, 0.25 deg (signed)    -        */
	EV_HUMP_LEVEL,		/* HUMP_A_* height, peak roll            -        */
	EV_MOTION_DONE,		/* MD_A_* seq, result                    odometer */
	EV_OBST_DETECT,		/* distance cm                           -        */
	EV_OBST_PROFILE,	/* WM_* width sweep result (12 Nov)      -        */
	EV_OBST_PASSED,		/* depth cm                              -        */
	EV_MAP_NEW,		/* mapping sequence number               -        */
	EV_SNAPSHOT_REQ,	/* request id (droppable)                -        */
	EV_NET,			/* link status (CYW43_LINK_*)            -        */
	EV_REMOTE,		/* REMOTE_* from the PC over MQTT        value    */	/* 12 Nov: b = depth mm for REMOTE_BYPASS_DEPTH */
	EV_OBST_SIDE,		/* side probe: 7 ranges, SIDE_GET()      SIDE_GET() */	/* 11 Nov: obstacle bypass */
	EV_OBST_NEAR,		/* pass guard: sideways cm               range cm   */	/* 11 Nov: obstacle bypass */
	EV_OBST_AHEAD,		/* something under D_WIDTH_CM: reading cm  -        */	/* 12 Nov: width sweep */
	EV_CODE_COUNT
} EV_CODE;

/* EV_REMOTE.a: the PC's START / STOP buttons (same rules as the START
   button on the car: start only from Idle) */
#define REMOTE_START		1U
#define REMOTE_STOP		2U
#define REMOTE_MODE_SHORT	3U	/* MODE short press: clear Fault / leave Calibrate */
#define REMOTE_MODE_LONG	4U	/* MODE held 2 s: Calibrate (from Idle only) */
/* 5, 6 (avoid left / right), 9 (servo trim), 11 (servo ends) and 12 (raw
   scan) were removed 20 Nov: the bypass side comes from the width sweep
   and the servo constants are fixed in drv_servo.h.  Not reused. */
#define REMOTE_BYPASS_DEPTH	7U	/* obstacle depth for the bypass, mm in word b */	/* 12 Nov: bypass depth from the PC */
#define REMOTE_WIDTH_CAL	8U	/* width calibration (Idle), known box width mm in word b */	/* 12 Nov: width sweep */
#define REMOTE_EDGE_SWEEP	10U	/* width sweep in Idle, measure only, print both edges */	/* 13 Nov */
/* REMOTE_WIDTH_CAL word b (13 Nov): the box's left / right edge from the
   car's centre line, mm, signed 16 bits each (+ = out on that side).
   0 = "wcal clear" (a real run never has both edges at 0). */
#define WCAL_B(l_mm, r_mm)	(BF_PUT((UW)(l_mm), 0, 16) | BF_PUT((UW)(r_mm), 16, 16))
#define WCAL_L(b)		BF_GETS(b, 0, 16)
#define WCAL_R(b)		BF_GETS(b, 16, 16)

typedef struct {
	UB	code;		/* EV_CODE */
	UB	src;		/* EV_SRC */
	UH	seq;		/* per-source counter: gaps = drops */
	UW	t_us;		/* timer_hw low word at the event, wraps 71 min */
	UW	a;		/* payload word A (layout per code, above) */
	UW	b;		/* payload word B */
} SUP_EVENT;

/* EV_BUTTON_EDGE.a */
#define BTN_A(pin, level)	(BF_PUT(pin, 0, 5) | BF_PUT(level, 5, 1))
#define BTN_PIN(a)		BF_GET(a, 0, 5)
#define BTN_LEVEL(a)		BF_GET(a, 5, 1)	/* 0 = pressed (active low) */

/* EV_CALIB_DONE.a */
#define CAL_WHO_IR		1U
#define CAL_WHO_IMU		2U
#define CAL_A(who, ok)		(BF_PUT(who, 0, 2) | BF_PUT(ok, 2, 1))
#define CAL_WHO(a)		BF_GET(a, 0, 2)
#define CAL_OK(a)		BF_GET(a, 2, 1)

/* EV_LINE_SAMPLE.a: raw 12-bit ADC of both line sensors + seen bits */
#define LS_A(l, r, seen_l, seen_r) \
	(BF_PUT(l, 0, 12) | BF_PUT(r, 12, 12) | BF_PUT(seen_l, 24, 1) | BF_PUT(seen_r, 25, 1))
#define LS_RAW_L(a)		BF_GET(a, 0, 12)
#define LS_RAW_R(a)		BF_GET(a, 12, 12)
#define LS_SEEN_L(a)		BF_GET(a, 24, 1)
#define LS_SEEN_R(a)		BF_GET(a, 25, 1)

/* EV_JUNCTION.a (2 bits, RY-1.13) */
typedef enum { JCT_NONE = 0, JCT_LEFT, JCT_RIGHT, JCT_CROSS } JCT_TYPE;

/* EV_BARCODE.a (RY-1.13): letter 0-25, 31 = invalid; direction */
#define BC_LETTER_INVALID	31U
#define BC_A(letter, reversed)	(BF_PUT(letter, 0, 5) | BF_PUT(reversed, 5, 1))
#define BC_LETTER(a)		BF_GET(a, 0, 5)
#define BC_REVERSED(a)		BF_GET(a, 5, 1)

/* EV_HUMP_LEVEL.a (JY-1.11): height 0-31 mm (saturates), peak 0.25 deg */
#define HUMP_A(h_mm, peak_q)	(BF_PUT(h_mm, 0, 5) | BF_PUT(peak_q, 8, 8))
#define HUMP_HEIGHT_MM(a)	BF_GET(a, 0, 5)
#define HUMP_PEAK_Q(a)		BF_GETS(a, 8, 8)

/* EV_MOTION_DONE.a */
typedef enum { MD_DONE = 0, MD_ABORTED, MD_STALLED, MD_REJECTED } MD_RESULT;
#define MD_A(seq, result)	(BF_PUT(seq, 0, 4) | BF_PUT(result, 4, 2))
#define MD_SEQ(a)		BF_GET(a, 0, 4)
#define MD_RESULT_OF(a)		BF_GET(a, 4, 2)

/* The old profile word (LK-1.12), no longer sent: EV_OBST_PROFILE carries
 * the WM_* word below since 12 Nov.  Kept for its packing tests.
 *   width 7 (cm) | left gap 7 (cm) | right gap 7 (cm) | gap side 2 | servo step 4 */
typedef enum { GAP_NONE = 0, GAP_LEFT, GAP_RIGHT, GAP_BOTH } GAP_SIDE;
#define PROF_A(w, gl, gr, side, step) \
	(BF_PUT(w, 0, 7) | BF_PUT(gl, 7, 7) | BF_PUT(gr, 14, 7) | \
	 BF_PUT(side, 21, 2) | BF_PUT(step, 23, 4))
#define PROF_WIDTH(a)		BF_GET(a, 0, 7)
#define PROF_GAP_L(a)		BF_GET(a, 7, 7)
#define PROF_GAP_R(a)		BF_GET(a, 14, 7)
#define PROF_SIDE(a)		BF_GET(a, 21, 2)
#define PROF_STEP(a)		BF_GET(a, 23, 4)
/* EV_OBST_PROFILE.b: closest distance (FR10, map position, bypass check)
 *   and line-to-edge distances, so the bypass offset does not assume the
 *   obstacle is centred on the line.                                     */
#define PROF_B(closest_cm, edge_l_cm, edge_r_cm) \
	(BF_PUT(closest_cm, 0, 9) | BF_PUT(edge_l_cm, 9, 7) | BF_PUT(edge_r_cm, 16, 7))
#define PROF_CLOSEST(b)		BF_GET(b, 0, 9)
#define PROF_EDGE_L(b)		BF_GET(b, 9, 7)	/* line to left edge, cm */
#define PROF_EDGE_R(b)		BF_GET(b, 16, 7)	/* line to right edge, cm */

/* EV_OBST_PROFILE.a (12 Nov): the width sweep, see subsystem5/obst_width.h
 *   face 12 (mm, straight ahead) | left boundary 7 | right boundary 7 | flags 3
 *   boundary = last hit deg + first miss deg (half-degrees); flags WM_F_*  */
#define WM_F_OOV_L		1U	/* still on the box at the left limit: edge out of view */
#define WM_F_OOV_R		2U	/* same on the right */
#define WM_F_NONE		4U	/* nothing straight ahead (box gone) */
#define WM_PACK(face_mm, el_half, er_half, flags) \
	(BF_PUT(face_mm, 0, 12) | BF_PUT(el_half, 12, 7) | \
	 BF_PUT(er_half, 19, 7) | BF_PUT(flags, 26, 3))
#define WM_FACE(a)		BF_GET(a, 0, 12)
#define WM_EDGE_L(a)		BF_GET(a, 12, 7)
#define WM_EDGE_R(a)		BF_GET(a, 19, 7)
#define WM_FLAGS(a)		BF_GET(a, 26, 3)

/* EV_OBST_SIDE.a / .b: side probe ranges */	/* 11 Nov: obstacle bypass */
#define SIDE_LOOKS		7	/* looks at 0, 5, ... 30 deg */
#define SIDE_STEP_DEG		5
#define SIDE_R_MAX_CM		63	/* 6-bit field: readings >= 63 cm or no echo -> 63 */
/* look i (0..6) is at i * SIDE_STEP_DEG towards the probed side.
   Looks 0..4 in word a, looks 5..6 in word b, 6 bits each, look 0 / 5 lowest. */
#define SIDE_PUT(i, cm)		BF_PUT((cm), ((i) % 5) * 6, 6)
#define SIDE_GET(a, b, i)	(((i) < 5) ? BF_GET((a), (i) * 6, 6) : BF_GET((b), ((i) - 5) * 6, 6))
/* EV_OBST_NEAR.a = sideways distance cm (lateral_cm(SCAN_PASS_DEG, r)),
   .b = the reading in cm */	/* 11 Nov: obstacle bypass */

/* =====================================================================
 *  2. MOTION_CMD  (Supervisor -> Motion Control), one packed UW
 *     Decision D1, revised: 8-bit signed angle in 2 deg steps (+-254 deg),
 *     plus a signed distance so "drive d", "creep S1" and "reverse 10 cm"
 *     can be expressed.
 *
 *     bits  0- 2  type      MC_TYPE
 *     bits  3- 4  speed     SPEED_LEVEL.  Turns: SPD_CRUISE = fast pivot
 *                           (junctions), anything else = the slow pivot.
 *     bits  5-12  angle     signed, 2 deg units.  TURN_REL: relative.
 *                           TURN_ABS: absolute compass heading, -180..178
 *     bits 13-23  dist      signed mm, -1024..1023 (negative = reverse)
 *     bits 24-27  seq       echoed back in EV_MOTION_DONE
 * ===================================================================== */
typedef enum {
	MC_STOP = 0,		/* brake to zero now */
	MC_FOLLOW,		/* follow the line at 'speed' until replaced */
	MC_DRIVE,		/* straight for 'dist' mm (sign = direction) */
	MC_TURN_REL,		/* pivot by 'angle' */
	MC_TURN_ABS,		/* pivot to compass heading 'angle' */
	MC_TYPE_COUNT
} MC_TYPE;

#define MC_ANGLE_UNIT_DEG	2
#define MC_DIST_MIN_MM		(-1024)
#define MC_DIST_MAX_MM		1023

#define MC_PACK(type, speed, angle_deg, dist_mm, seq) \
	(BF_PUT(type, 0, 3) | BF_PUT(speed, 3, 2) |			\
	 BF_PUT((W)(angle_deg) / MC_ANGLE_UNIT_DEG, 5, 8) |		\
	 BF_PUT(dist_mm, 13, 11) | BF_PUT(seq, 24, 4))
#define MC_TYPE_OF(w)		((MC_TYPE)BF_GET(w, 0, 3))
#define MC_SPEED(w)		((SPEED_LEVEL)BF_GET(w, 3, 2))
#define MC_ANGLE_DEG(w)		(BF_GETS(w, 5, 8) * MC_ANGLE_UNIT_DEG)
#define MC_DIST_MM(w)		BF_GETS(w, 13, 11)
#define MC_SEQ(w)		BF_GET(w, 24, 4)

/* =====================================================================
 *  3. SCAN_REQ  (Supervisor -> Obstacle Scan), one packed UW
 *     The mode decides who may set OBST_STOP (fault_flags.h).
 *     bits 0-2 mode | bits 3-6 seq | bits 8-15 param (cm)
 * ===================================================================== */
typedef enum {
	SCAN_GUARD = 0,		/* servo ahead, ping 50 ms, may set OBST_STOP */
	SCAN_INHIBIT,		/* no pings (in-place turns, Idle) */
	SCAN_PROFILE,		/* car stopped: width sweep (face, both edges), then EV_OBST_PROFILE */
	SCAN_PASS_LEFT,		/* servo to the left side, watch the obstacle */
	SCAN_PASS_RIGHT,	/* servo to the right side */
	SCAN_EMERG_ONLY,	/* servo ahead, only D_EMERG may stop (bypass legs) */
	SCAN_SIDE_PROBE,	/* car stopped: 7 looks at 0..30 deg to one side -> EV_OBST_SIDE */	/* 11 Nov: obstacle bypass */
	SCAN_MODE_COUNT		/* <= 8: the mode is 3 bits in SCAN_REQ */
} SCAN_MODE;

/* param: PASS_*: the sideways distance in cm below which the pass guard
   reports EV_OBST_NEAR (0 = PASS_SIDE_CM).  SIDE_PROBE: SIDE_PROBE_*. */	/* 11 Nov: obstacle bypass */
#define SIDE_PROBE_RIGHT	0U	/* look at negative servo angles */	/* 11 Nov: obstacle bypass */
#define SIDE_PROBE_LEFT		1U	/* look at positive servo angles */	/* 11 Nov: obstacle bypass */
#define SCAN_PACK(mode, seq, param) \
	(BF_PUT(mode, 0, 3) | BF_PUT(seq, 3, 4) | BF_PUT(param, 8, 8))
#define SCAN_MODE_OF(w)		((SCAN_MODE)BF_GET(w, 0, 3))
#define SCAN_SEQ(w)		BF_GET(w, 3, 4)
#define SCAN_PARAM(w)		BF_GET(w, 8, 8)

/* =====================================================================
 *  4. TELEM_MSG  (Supervisor -> Telemetry), 16 bytes
 * ===================================================================== */
typedef enum {
	TM_STATE = 0,		/* state change: value = new SYS_STATE */
	TM_MAP_BARCODE,		/* value = letter */
	TM_MAP_JUNCTION,	/* value = JCT_TYPE */
	TM_MAP_OBSTACLE,	/* value = width mm (measured by the width sweep) */
	TM_MAP_HUMP,		/* value = height mm */
	TM_COMMAND,		/* value = MOTION_CMD type, extra = packed cmd */
	TM_KIND_COUNT
} TM_KIND;

typedef struct {
	UB	kind;		/* TM_KIND */
	UB	state;		/* SYS_STATE when sent */
	UH	value;		/* per kind, above */
	W	x_mm;		/* dead-reckoned position when it happened */
	W	y_mm;
	UW	extra;
} TELEM_MSG;

/* =====================================================================
 *  5. Line error, latest-value word (Line Following -> Motion Control)
 *     bits 0-7 error (signed, L - R scaled, RY-1.8) | bits 8-15 flags |
 *     bits 16-23 sequence (Motion Control: unchanged for 3 cycles = stale)
 * ===================================================================== */
#define LE_SEEN_L		0x01U
#define LE_SEEN_R		0x02U
#define LE_VALID		0x04U	/* calibrated and sampling */
/* The line runs away to one side (an arc), not alternating between the two
   sensors (a straight): Line Following's own curve / straight test, the one
   that picks LINE_ERR_STEP over LINE_STEP_STRAIGHT.  Motion Control slows
   down by CURVE_SLOW_PCT while it is set, which tightens the arc it turns
   for the same steering correction.  Firmware that never sets it just runs
   at full speed, as before. */
#define LE_CURVE		0x08U
#define LE_PACK(err, flags, seq) \
	(BF_PUT(err, 0, 8) | BF_PUT(flags, 8, 8) | BF_PUT(seq, 16, 8))
#define LE_ERR(w)		BF_GETS(w, 0, 8)
#define LE_FLAGS(w)		BF_GET(w, 8, 8)
#define LE_SEQ(w)		BF_GET(w, 16, 8)

/* =====================================================================
 *  6. Calibrate requests (Supervisor -> Ryan, Jian Yi): event flag bits.
 *     The owner clears its own bit when it starts, then posts
 *     EV_CALIB_DONE with CAL_A(who, ok).
 * ===================================================================== */
#define CAL_REQ_IR		(1U << 0)
#define CAL_REQ_IMU		(1U << 1)

#endif /* MSG_TYPES_H */
