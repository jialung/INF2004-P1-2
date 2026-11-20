/*
 *	supervisor.c
 *	Supervisor task: the system statechart.  Event-driven, core 1, pri 10.
 *	Owner: Christian.  Plan refs: CH-1.17..CH-1.25, chains 4.1, 4.3-4.6.
 *
 *	One wait object only: tk_rcv_mbf(mbf_sup_event, SUP_POLL_MS).  Every
 *	wake-up (event or timeout) also re-reads the fault flag, times
 *	button holds, the Calibrate timeout, the buzzer and the letter
 *	discard distance.
 *
 *	States: Init, Idle, Calibrate, Running {LineFollow, JunctionAction,
 *	HumpTraverse, ObstacleAvoid, LineSearch}, Finished, Fault.
 *
 *	Behaviour: start / stop / calibrate / fault / clear; line follow;
 *	  hump slow-down; barcode letter -> junction turn; line lost ->
 *	  LineSearch (Ryan's first cut); obstacle -> 50 deg side-step bypass
 *	  (FR11, 11 Nov) round the side with more room, from the width sweep
 *	  (20 Nov).
 *	TODO: CH-2.3 Finished trigger.
 */

#include "subsystem5.h"
#include "drv_ui.h"
#include "../diag.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../gpio_irq.h"
#include "../hal.h"
#include "../wcet.h"
#include "../subsystem2/line_search.h"
#include "map_parse.h"
#include "remote_cmd.h"
#include "../subsystem3/subsystem3.h"	/* imu_cal: compass calibrated? */
#include "bypass_geom.h"
#include "obst_width.h"

EXPORT volatile SUP_STATS sup_stats;

LOCAL SYS_STATE	st = SYS_INIT;
LOCAL UINT	mseq;			/* motion command sequence, 4 bits */
LOCAL UINT	sseq;			/* scan request sequence */
LOCAL NAV_STATE	ns;			/* fresh copy each wake-up */

/* Mapping: active table + shadow (CH-1.14 swaps them in one step) */
LOCAL UB	map_active[26];		/* MAP_CMD per letter.  TODO CH-1.13 */
/* What to do at a junction reached with NO barcode pending.  A barcode
   counts for ONE junction (have_letter is cleared as it is used), so every
   junction after that uses this until the next barcode.  Set from the UI
   ("default=..." on the map topic); MAP_FORWARD = drive straight across,
   which is what the car did before this was settable. */
LOCAL UB	map_default = (UB)MAP_FORWARD;

/* JunctionAction: 1 = driving the wheels over the crossing, 4 = stopped,
   settling (JCT_SETTLE_MS), 2 = pivoting, 3 = compass correction */
LOCAL UINT	jct_step;
LOCAL INT	jct_angle;
LOCAL UW	jct_t0;			/* when the settle pause began */
LOCAL UH	jct_head0;		/* compass heading before the pivot */

/* ObstacleAvoid: the 50 deg side-step bypass (FR11, 11 Nov).
   Out-step = pivot out 50, drive, pivot back; the first one is
   sized by geometry, the second by scan 1.  Then pass, turn in 50 towards
   the line, approach until a line sensor sees it, roll onto it, align. */
typedef enum {
	BY_NONE = 0, BY_TURN_OUT, BY_STEP_OUT, BY_STRAIGHTEN, BY_SCAN, BY_PASS,
	BY_TURN_IN, BY_APPROACH_FAST, BY_APPROACH_SLOW, BY_ON_LINE, BY_ALIGN,
	BY_LOST_ALIGN
} BY_STEP;
LOCAL BY_STEP	byp_step;
LOCAL W		byp_depth_pref = BG_DEPTH_DEFAULT_MM;	/* PC setting, mm */
LOCAL INT	byp_side;		/* latched at entry: +1 left, -1 right */
LOCAL W		byp_depth_mm;		/* latched at entry: obstacle depth */
LOCAL UW	byp_t0;			/* entry time, us: the 60 s limit */
LOCAL UINT	byp_stop_cm;		/* reading that stopped the car */
LOCAL W		byp_xf_mm;		/* axle to obstacle face at the stop */
LOCAL W		byp_need_mm;		/* 125 left / 155 right */
LOCAL W		byp_edge_mm;		/* obstacle edge from the line, bypass side */
LOCAL W		byp_next_mm;		/* length of the out-step being made */
LOCAL W		byp_slow_mm;		/* slow approach length */
LOCAL UINT	byp_steps;		/* out-steps completed */
LOCAL UINT	byp_extra;		/* extra out-steps used */
LOCAL UINT	byp_line_n;		/* consecutive fresh samples on black */
LOCAL UINT	byp_line_seq;		/* LE_SEQ of the last sample counted */

LOCAL void byp_step_to(BY_STEP s)
{
	byp_step = s;
}

/* Obstacle width (12 Nov).  Two-stage stop: at D_WIDTH_CM the obstacle task
   sends EV_OBST_AHEAD, the car stops, the width sweep runs, then the car
   follows the line again and stops for the bypass at D_DETECT_CM as before.
   The width is shown; the two edges choose the bypass side (wm_side). */
LOCAL BOOL	wm_active;		/* LineFollow: stopped, waiting for the sweep */
LOCAL UW	wm_t0;			/* when it began, us */
/* Idle sweeps from the PC: WM_IDLE_CAL = a calibration run ("wcal"),
   WM_IDLE_SWEEP = measure only ("esweep") */
#define WM_IDLE_NONE	0U
#define WM_IDLE_CAL	1U
#define WM_IDLE_SWEEP	2U
LOCAL UINT	wm_idle;
LOCAL W		wm_known_l, wm_known_r;	/* "wcal": where the box edges really are */
LOCAL OW_CAL	wm_cal = { OW_SL_E4, OW_KL_E4, OW_SR_E4, OW_KR_E4 };	/* in use */
LOCAL OW_POINT	wm_pts[OW_CAL_MAX_POINTS];	/* "wcal" runs since power-up / "wcal clear" */
LOCAL UINT	wm_npts;
LOCAL INT	wm_width_mm;		/* last measured width, 0 = none yet */
/* Bypass side from the last LineFollow sweep (OW_GO_*), for the next
   bypass only: used up at the stop, dropped by a new sweep or by any
   state other than LineFollow, and too old after WM_SIDE_VALID_MM. */
LOCAL BOOL	wm_side_have;
LOCAL INT	wm_side;
LOCAL UH	wm_side_odo;		/* odometer at the sweep result */

/* Pending letter (chain 4.3 steps 5-7) */
LOCAL BOOL	have_letter;
LOCAL UB	pend_letter;
LOCAL UH	pend_odo;		/* odometer when read */
LOCAL UH	pause_odo;		/* discard counter paused in ObstacleAvoid */

/* ObstacleAvoid line reference (chain 4.5 step 4) */
LOCAL NAV_STATE	line_ref;

/* Calibrate */
LOCAL UINT	cal_wait;
LOCAL SYSTIM	cal_t0;
/* Auto spin (CAL_AUTO_SPIN): 0 = still phase, 1..CAL_SPIN_STEPS = that
   pivot running, CAL_SPIN_STEPS + 1 = finished or given up */
LOCAL UINT	cal_spin;
LOCAL UW	cal_spin_t0;

/* Buttons: index 0 = START, 1 = MODE */
typedef struct {
	BOOL	down;
	BOOL	long_fired;
	UW	t_down;
	UW	t_edge;
} BTN;
LOCAL BTN	btn[2];

/* Buzzer */
LOCAL BOOL	beeping;
LOCAL UW	beep_until;

/* ------------------------------------------------------------------ */
LOCAL void beep(UW ms)
{
	drv_buzzer(1);
	beeping = TRUE;
	beep_until = now_us() + ms * 1000U;
}

LOCAL void motion(MC_TYPE t, SPEED_LEVEL s, INT angle, INT dist)
{
	mseq = (mseq + 1U) & 0xFU;
	(void)motion_cmd_send(motion_cmd_make(t, s, angle, dist, mseq));
}

LOCAL void scan(SCAN_MODE m, UINT param)
{
	sseq = (sseq + 1U) & 0xFU;
	(void)scan_req_send(SCAN_PACK(m, sseq, param));
}

/* Calibrate auto spin: start the next pivot, or stop after the last one
   (or at once if 'give_up', e.g. a wheel stalled). */
LOCAL void cal_spin_next(BOOL give_up)
{
	if (!give_up && cal_spin < CAL_SPIN_STEPS) {
		cal_spin++;
		cal_spin_t0 = now_us();			/* step timeout from here */
		tm_printf((UB*)"[sup] calibrate: pivot %d of %d (%d deg)\n",
			  (INT)cal_spin, (INT)CAL_SPIN_STEPS, (INT)CAL_SPIN_STEP_DEG);
		motion(MC_TURN_REL, SPD_SLOW, CAL_SPIN_STEP_DEG, 0);
		return;
	}
	tm_printf((UB*)(give_up ?
		"[sup] calibrate: spin stopped: turn the car by hand to finish\n" :
		"[sup] calibrate: spin done, keep still until the beep\n"));
	cal_spin = CAL_SPIN_STEPS + 1U;
	motion(MC_STOP, SPD_STOP, 0, 0);
}

/* signed angle difference b - a, wrapped to -180..179 deg */
LOCAL INT wrap180(INT d)
{
	while (d >= 180) d -= 360;
	while (d < -180) d += 360;
	return d;
}

LOCAL void tm_log(TM_KIND k, UH value, UW extra)
{
	TELEM_MSG m;

	m.kind  = (UB)k;
	m.state = (UB)st;
	m.value = value;
	m.x_mm  = ns.m.x_mm;
	m.y_mm  = ns.m.y_mm;
	m.extra = extra;
	(void)telem_send(&m);
}

/* LED colour per state (CH-1.25), dim */
LOCAL void led_for(SYS_STATE s)
{
	LOCAL const UB rgb[SYS_STATE_COUNT][3] = {
		{ 8, 8, 8 },	/* Init: white */
		{ 0, 0, 24 },	/* Idle: blue */
		{ 24, 16, 0 },	/* Calibrate: yellow */
		{ 0, 24, 0 },	/* LineFollow: green */
		{ 16, 16, 16 },	/* JunctionAction: white */
		{ 0, 16, 16 },	/* HumpTraverse: cyan */
		{ 24, 8, 0 },	/* ObstacleAvoid: orange */
		{ 16, 0, 16 },	/* LineSearch: purple */
		{ 24, 0, 12 },	/* Finished: pink */
		{ 32, 0, 0 },	/* Fault: red */
	};
	drv_led_rgb(rgb[s][0], rgb[s][1], rgb[s][2]);
}

/* ---- ObstacleAvoid helpers (FR11) ---------------------------------- */
LOCAL void byp_pose(W *along, W *lat_s)		/* lat_s > 0 = towards the bypass side */
{
	int32_t a, l;
	bg_pose(ns.m.x_mm - line_ref.m.x_mm, ns.m.y_mm - line_ref.m.y_mm,
		(W)line_ref.m.turn_q / 4, &a, &l);
	*along = a;
	*lat_s = l * byp_side;
}

/* Pivot 50 deg on the spot.  dir = +1: towards the bypass side (+s * 50),
   dir = -1: away from it (-s * 50).  Sets the step the reply will advance. */
LOCAL void pivot(BY_STEP next, INT dir)
{
	byp_step_to(next);
	scan(SCAN_INHIBIT, 0);
	motion(MC_TURN_REL, SPD_SLOW, dir * byp_side * BG_TURN_DEG, 0);
}

LOCAL void out_step(W mm)			/* pivot out; bypass_next then drives mm */
{
	byp_next_mm = mm;
	pivot(BY_TURN_OUT, +1);
}

LOCAL void bypass_fail(const char *why)
{
	tm_printf((UB*)"[sup] bypass: %s\n", why);
	byp_step_to(BY_NONE);
	fault_raise(FAULT_NO_BYPASS);			/* -> Fault */
}

/* ------------------------------------------------------------------ */
LOCAL void enter(SYS_STATE s)
{
	SYS_STATE from = st;
	W	  d1;
	const char *side_why;

	if (wm_active || wm_idle != WM_IDLE_NONE) {
		/* any state change ends a width sweep.  The states that send a
		   scan request of their own cut the sweep short; these three do
		   not, so give the guard back. */
		wm_active = FALSE;
		wm_idle = WM_IDLE_NONE;
		if (s == SYS_HUMP_TRAVERSE || s == SYS_JUNCTION_ACTION || s == SYS_LINE_SEARCH) {
			scan(SCAN_GUARD, 0);
		}
	}
	if (s != SYS_OBSTACLE_AVOID) byp_step = BY_NONE;	/* STOP, Fault ... end a bypass */
	/* a junction, hump, line search, STOP ...: the swept box is not the
	   one the car meets next */
	if (s != SYS_LINE_FOLLOW && s != SYS_OBSTACLE_AVOID) wm_side_have = FALSE;
	st = s;
	sys_state_set(s);
	sup_stats.transitions++;
	led_for(s);
	tm_log(TM_STATE, (UH)s, (UW)from);
	tm_printf((UB*)"[sup] %d -> %d\n", (INT)from, (INT)s);

	switch (s) {
	case SYS_IDLE:
	case SYS_FINISHED:
		motion(MC_STOP, SPD_STOP, 0, 0);
		scan(SCAN_INHIBIT, 0);
		have_letter = FALSE;
		break;
	case SYS_CALIBRATE:
		motion(MC_STOP, SPD_STOP, 0, 0);
		cal_wait = CAL_WHO_IR | CAL_WHO_IMU;
		tk_get_otm(&cal_t0);
		cal_spin = 0;
		cal_spin_t0 = now_us();
		(void)tk_set_flg(flg_calib, CAL_REQ_IR | CAL_REQ_IMU);
#if CAL_AUTO_SPIN
		tm_printf((UB*)"[sup] calibrate: keep still, the car spins by itself in %d ms\n",
			  (INT)CAL_SPIN_DELAY_MS);
#endif
		break;
	case SYS_LINE_FOLLOW:
		scan(SCAN_GUARD, 0);
		motion(MC_FOLLOW, SPD_CRUISE, 0, 0);
		break;
	case SYS_HUMP_TRAVERSE:
		motion(MC_FOLLOW, SPD_HUMP, 0, 0);	/* 50 % (chain 4.4 step 3) */
		break;
	case SYS_OBSTACLE_AVOID:
		line_ref  = ns;				/* save the line reference */
		pause_odo = ns.m.odo_mm;		/* pause the discard counter */
		/* the side with more room, from this obstacle's width sweep */
		byp_side = BYPASS_SIDE_DEFAULT;
		if (!wm_side_have) {
			side_why = "default side: no width sweep for this obstacle";
		} else if ((UH)(ns.m.odo_mm - wm_side_odo) > WM_SIDE_VALID_MM) {
			side_why = "default side: the width sweep was too long ago";
		} else if (wm_side == OW_GO_EITHER) {
			side_why = "default side: the width sweep gave no preference";
		} else {
			side_why = "side from the width sweep";
			byp_side = wm_side;
		}
		wm_side_have = FALSE;			/* used up */
		byp_depth_mm = byp_depth_pref;
		byp_steps = 0;
		byp_extra = 0;
		byp_t0    = now_us();
		byp_xf_mm = US_FACE_TO_AXLE_MM + (W)byp_stop_cm * 10;
		byp_need_mm = (byp_side > 0) ? BG_NEED_LEFT_MM : BG_NEED_RIGHT_MM;
		byp_edge_mm = BG_E_MAX_MM;
		/* Stop for good (drop Motion's held FOLLOW) BEFORE the scan
		   request: SCAN_INHIBIT clears OBST_STOP, and the FOLLOW must
		   not resume then.  Order: MC_STOP, SCAN_INHIBIT, MC_TURN_REL. */
		motion(MC_STOP, SPD_STOP, 0, 0);
		d1 = bg_step1_mm(byp_xf_mm, (byp_side > 0) ? BG_CORNER_R_MM : BG_CORNER_L_MM);
		if (d1 < 0) {
			scan(SCAN_INHIBIT, 0);
			bypass_fail("stopped too close to the obstacle to turn");
			break;
		}
		tm_printf((UB*)"[sup] bypass %s: stopped at %d cm, step 1 = %d mm, depth %d mm (%s)\n",
			  (byp_side > 0) ? "LEFT" : "RIGHT", (INT)byp_stop_cm, (INT)d1,
			  (INT)byp_depth_mm, side_why);
		out_step(d1);				/* sends SCAN_INHIBIT, then the turn */
		break;
	case SYS_LINE_SEARCH:
		break;					/* started by the caller */
	case SYS_FAULT:
		motion(MC_STOP, SPD_STOP, 0, 0);
		scan(SCAN_INHIBIT, 0);
		beep(800);
		tm_printf((UB*)"[sup] FAULT: %s (0x%x)\n",
			  fault_name(fault_peek() & FAULT_TRIP_MASK),
			  (INT)fault_peek());
		break;
	default:
		break;
	}
}

LOCAL void leave_obstacle_avoid(void)
{
	/* resume the discard counter: shift the letter's start by the pause */
	if (have_letter) {
		pend_odo = (UH)(pend_odo + (UH)(ns.m.odo_mm - pause_odo));
	}
}

/* offset_mm: + = the line is expected to the LEFT; the search sweeps
   that way first.  Only the sign is used. */
LOCAL void start_line_search(LS_REASON why, H offset_mm)
{
	LS_REF		ref;
	LS_RESULT	r;

	ref.heading_deg = line_ref.i.heading_deg;
	ref.odo_mm = ns.m.odo_mm;
	ref.expected_offset_mm = offset_mm;
	if (why == LS_REASON_LINE_LOST) {
		ref.heading_deg = ns.i.heading_deg;
	}
	enter(SYS_LINE_SEARCH);
	line_search_start(why, &ref);
	r = line_search_step(EV_NONE, 0, ns.m.odo_mm);
	if (r.kind == LS_MOVE) (void)motion_cmd_send(r.cmd);
}

LOCAL void junction(JCT_TYPE type)
{
	MAP_CMD c = (MAP_CMD)map_default;	/* no barcode: the UI's default */
	INT	used = '-';

	tm_log(TM_MAP_JUNCTION, (UH)type, 0);
	if (have_letter) {
		c = (MAP_CMD)map_active[pend_letter];	/* CURRENT mapping (step 7) */
		used = 'A' + pend_letter;
		have_letter = FALSE;			/* one letter, one junction */
	}
	/* TODO CH-1.17: a command the junction type does not allow (e.g. LEFT
	   at a right-only junction) -> team rule needed */
	/* Two steps: drive SENSOR_TO_AXLE_MM so the wheels are over the
	   crossing (the sensors are at the front), then pivot.  The Motion
	   replies drive the steps (EV_MOTION_DONE in handle()). */
	switch (c) {
	case MAP_LEFT:  jct_angle = 90;   break;
	case MAP_RIGHT: jct_angle = -90;  break;
	case MAP_UTURN: jct_angle = 180;  break;
	case MAP_FORWARD:
	default:        jct_angle = 0;    break;	/* keep following */
	}
	tm_printf((UB*)"[sup] junction: letter %c -> %s%s\n", used,
		  (c == MAP_LEFT) ? "LEFT" : (c == MAP_RIGHT) ? "RIGHT" :
		  (c == MAP_UTURN) ? "U-TURN" : "straight on",
		  (used == '-') ? " (no barcode: the default)" :
				  " (this barcode is now used up)");
	if (jct_angle != 0) {
		jct_head0 = ns.i.heading_deg;
		enter(SYS_JUNCTION_ACTION);
		scan(SCAN_EMERG_ONLY, 0);
		jct_step = 1;
		motion(MC_DRIVE, SPD_SLOW, 0, SENSOR_TO_AXLE_MM);
	}
	tm_log(TM_COMMAND, (UH)c, 0);
}

/* FR6 / JY-1.20: after a junction pivot, compare the compass with the
   commanded angle.  More than COMPASS_TOL_DEG off: one corrective pivot.
   Only with a calibrated compass; an error larger than COMPASS_MAX_FIX
   is not trusted (wrong axis sign, magnet nearby) and only logged.
   Returns TRUE if a correction was sent. */
LOCAL BOOL compass_fix_needed(void)
{
	INT moved, err;

	if (!ns.i.imu_ok) {
		return FALSE;
	}
	moved = wrap180((INT)ns.i.heading_deg - (INT)jct_head0);	/* + = clockwise */
	err   = wrap180(-jct_angle - moved);	/* + left turn = heading down */
	tm_printf((UB*)"[sup] turn check: compass moved %d deg, wanted %d, error %d%s\n",
		  moved, -jct_angle, err,
		  imu_cal.mag_ok ? "" : " (compass not calibrated: not used)");
	if (!imu_cal.mag_ok || !COMPASS_CORRECT) return FALSE;
	if (err > COMPASS_MAX_FIX_DEG || err < -COMPASS_MAX_FIX_DEG) return FALSE;
	if (err <= COMPASS_TOL_DEG && err >= -COMPASS_TOL_DEG) return FALSE;
	motion(MC_TURN_REL, SPD_SLOW, -err, 0);	/* + = counter-clockwise */
	return TRUE;
}

/* ---- ObstacleAvoid: scans, pass and rejoin (FR11) ------------------ */

/* The scan says clear: drive past the obstacle, the pass guard watching
   the side the obstacle is on (the other side from the bypass). */
LOCAL void start_pass(void)
{
	W along, lat, p;

	byp_pose(&along, &lat);
	p = bg_pass_mm(byp_xf_mm, byp_edge_mm, byp_need_mm, lat, along, byp_depth_mm);
	tm_printf((UB*)"[sup] bypass: scan %d clear, pass %d mm\n", (INT)byp_steps, (INT)p);
	if (p == 0) {
		pivot(BY_TURN_IN, -1);
		return;
	}
	byp_step_to(BY_PASS);
	scan((byp_side > 0) ? SCAN_PASS_RIGHT : SCAN_PASS_LEFT,
	     (UINT)((byp_need_mm + 9) / 10));
	motion(MC_DRIVE, BYPASS_SPEED, 0, p);
}

/* EV_OBST_SIDE: the side probe after an out-step */
LOCAL void bypass_side_result(UW a, UW b)
{
	uint8_t	r[SIDE_LOOKS];
	W	along, lat, f;
	UINT	i;

	for (i = 0; i < SIDE_LOOKS; i++) r[i] = (uint8_t)SIDE_GET(a, b, i);
	byp_pose(&along, &lat);
	tm_printf((UB*)"[sup] bypass: scan %d at along %d lat %d: %d %d %d %d %d %d %d cm\n",
		  (INT)byp_steps, (INT)along, (INT)lat, (INT)r[0], (INT)r[1],
		  (INT)r[2], (INT)r[3], (INT)r[4], (INT)r[5], (INT)r[6]);
	if (byp_steps == 1U) {				/* scan 1: size step 2 */
		W d2;

		f = byp_xf_mm - (US_FACE_TO_AXLE_MM + along);
		byp_edge_mm = bg_edge_mm(r, lat, f);
		d2 = bg_step2_mm(byp_edge_mm, byp_need_mm, lat);
		tm_printf((UB*)"[sup] bypass: edge %d mm, step 2 = %d mm\n",
			  (INT)byp_edge_mm, (INT)d2);
		out_step(d2);
	} else if (!bg_blocked(r, byp_need_mm)) {
		start_pass();
	} else if (byp_extra < BYPASS_MAX_EXTRA) {
		byp_extra++;
		tm_printf((UB*)"[sup] bypass: scan %d blocked, extra step %d of %d\n",
			  (INT)byp_steps, (INT)byp_extra, (INT)BYPASS_MAX_EXTRA);
		out_step(BYPASS_EXTRA_STEP_MM);
	} else {
		bypass_fail("still blocked after the extra steps");
	}
}

/* EV_OBST_NEAR: the pass guard saw the obstacle too close beside the car */
LOCAL void bypass_near(UW side_cm)
{
	W along, lat;

	byp_pose(&along, &lat);
	if (lat - (W)side_cm * 10 > byp_edge_mm) byp_edge_mm = lat - (W)side_cm * 10;
	if (byp_extra < BYPASS_MAX_EXTRA) {
		byp_extra++;
		tm_printf((UB*)"[sup] bypass: too close while passing (%d cm), extra step %d of %d\n",
			  (INT)side_cm, (INT)byp_extra, (INT)BYPASS_MAX_EXTRA);
		out_step(BYPASS_EXTRA_STEP_MM);		/* replaces the pass drive */
	} else {
		bypass_fail("obstacle too close while passing");
	}
}

/* After the turn-in: drive towards the line, fast, then slow for the last
   APPROACH_SLOW_MM and APPROACH_OVER_MM past where it is expected.
   poll_timers() watches the line sensors. */
LOCAL void begin_approach(void)
{
	W along, lat, sx, fast;

	byp_pose(&along, &lat);
	sx   = bg_line_mm(lat);
	fast = sx - APPROACH_SLOW_MM;
	byp_slow_mm  = ((fast > 0) ? APPROACH_SLOW_MM : ((sx > 0) ? sx : 0)) + APPROACH_OVER_MM;
	byp_line_n   = 0;
	byp_line_seq = LE_SEQ(line_err_read());
	scan(SCAN_EMERG_ONLY, 0);
	if (fast > 0) {
		byp_step_to(BY_APPROACH_FAST);
		motion(MC_DRIVE, BYPASS_SPEED, 0, fast);
	} else {
		byp_step_to(BY_APPROACH_SLOW);
		motion(MC_DRIVE, SPD_SLOW, 0, byp_slow_mm);
	}
}

/* the current bypass move finished (matching MD_DONE) */
LOCAL void bypass_next(void)
{
	switch (byp_step) {
	case BY_TURN_OUT:
		byp_step_to(BY_STEP_OUT);
		scan(SCAN_EMERG_ONLY, 0);
		motion(MC_DRIVE, BYPASS_SPEED, 0, byp_next_mm);
		break;
	case BY_STEP_OUT:
		byp_steps++;
		pivot(BY_STRAIGHTEN, -1);
		break;
	case BY_STRAIGHTEN:
		/* no motion: the reply is EV_OBST_SIDE */
		byp_step_to(BY_SCAN);
		scan(SCAN_SIDE_PROBE, (byp_side > 0) ? SIDE_PROBE_RIGHT : SIDE_PROBE_LEFT);
		break;
	case BY_PASS:
		pivot(BY_TURN_IN, -1);
		break;
	case BY_TURN_IN:
		begin_approach();
		break;
	case BY_APPROACH_FAST:
		byp_step_to(BY_APPROACH_SLOW);
		motion(MC_DRIVE, SPD_SLOW, 0, byp_slow_mm);
		break;
	case BY_APPROACH_SLOW:
		tm_printf((UB*)"[sup] bypass: no line where expected, searching\n");
		pivot(BY_LOST_ALIGN, +1);
		break;
	case BY_ON_LINE:
		pivot(BY_ALIGN, +1);
		break;
	case BY_ALIGN:
		byp_step_to(BY_NONE);
		tm_printf((UB*)"[sup] bypass: done\n");
		leave_obstacle_avoid();
		enter(SYS_LINE_FOLLOW);			/* SCAN_GUARD + MC_FOLLOW */
		break;
	case BY_LOST_ALIGN:
		byp_step_to(BY_NONE);
		leave_obstacle_avoid();
		/* the line is on the side away from the bypass */
		start_line_search(LS_REASON_LINE_LOST, (H)(-byp_side * LOST_LINE_OFFSET_MM));
		break;
	case BY_SCAN:
	case BY_NONE:
	default:
		break;					/* no motion is running */
	}
}

/* ------------------------------------------------------------------ */
LOCAL void on_button(UINT idx, BOOL pressed, BOOL is_long)
{
	if (idx == 0 && pressed && !is_long) {			/* START */
		if (st == SYS_IDLE) {
			enter(SYS_LINE_FOLLOW);
		} else if (SYS_IS_RUNNING(st) || st == SYS_FINISHED) {
			enter(SYS_IDLE);
		}
	} else if (idx == 1 && !pressed && !is_long) {		/* MODE short */
		if (st == SYS_FAULT) {
			fault_clear(FAULT_USER_CLEAR_MASK);
			if ((fault_peek() & FAULT_TRIP_MASK) == 0) {
				enter(SYS_IDLE);
			} else {
				beep(200);			/* still faulty */
			}
		} else if (st == SYS_CALIBRATE) {
			enter(SYS_IDLE);			/* abort calibration */
		}
	} else if (idx == 1 && is_long) {			/* MODE long */
		if (st == SYS_IDLE) {
			enter(SYS_CALIBRATE);
		}
	}
}

/* Debounce + short / long (CH-1.7).  START acts on press, MODE on release
   (short) or after 2 s held (long). */
LOCAL void button_edge(UW a, UW t_us)
{
	UINT	idx = (BTN_PIN(a) == PIN_BTN_START) ? 0U : 1U;
	BTN	*b = &btn[idx];
	BOOL	pressed = (BTN_LEVEL(a) == 0);	/* active low */

	if ((t_us - b->t_edge) < BTN_DEBOUNCE_US) {
		return;
	}
	b->t_edge = t_us;
	if (pressed && !b->down) {
		b->down = TRUE;
		b->long_fired = FALSE;
		b->t_down = t_us;
		on_button(idx, TRUE, FALSE);
	} else if (!pressed && b->down) {
		b->down = FALSE;
		if (!b->long_fired) {
			on_button(idx, FALSE, FALSE);
		}
	}
}

/* After the sweep, or its timeout: follow the line again (what
   enter(SYS_LINE_FOLLOW) does), still in LineFollow. */
LOCAL void width_resume(void)
{
	wm_active = FALSE;
	scan(SCAN_GUARD, 0);
	motion(MC_FOLLOW, SPD_CRUISE, 0, 0);
}

/* EV_OBST_PROFILE: the sweep is done (obst_width.h turns it into mm) */
LOCAL void width_result(UW word)
{
	W	face = (W)WM_FACE(word), el = (W)WM_EDGE_L(word), er = (W)WM_EDGE_R(word);
	UINT	fl = WM_FLAGS(word);
	W	mm;

	if (wm_idle == WM_IDLE_SWEEP) {		/* "esweep": measure only, Idle */
		wm_idle = WM_IDLE_NONE;
		if (fl & WM_F_NONE) {
			tm_printf((UB*)"[sup] edge sweep: nothing straight ahead\n");
			return;
		}
		/* the edges even when one is out of view: the window shows it */
		tm_printf((UB*)"[sup] edge sweep: face %d mm, left %d.%d deg -> %d mm%s, "
			  "right %d.%d deg -> %d mm%s, width %d mm\n", (INT)face,
			  (INT)(el / 2), (INT)((el % 2) * 5),
			  (INT)ow_edge_mm(face, el, wm_cal.sl_e4, wm_cal.kl_e4),
			  (fl & WM_F_OOV_L) ? " (out of view)" : "",
			  (INT)(er / 2), (INT)((er % 2) * 5),
			  (INT)ow_edge_mm(face, er, wm_cal.sr_e4, wm_cal.kr_e4),
			  (fl & WM_F_OOV_R) ? " (out of view)" : "",
			  (fl == 0U) ? (INT)ow_width_mm(face, el, er, &wm_cal) : -1);
		return;
	}
	if (wm_idle == WM_IDLE_CAL) {		/* "wcal": known edge positions, Idle */
		OW_POINT *pt;
		UINT	 i;
		int	 okl, okr;
		int32_t	 sl = 0, kl = 0, sr = 0, kr = 0;

		wm_idle = WM_IDLE_NONE;
		if (fl != 0U) {
			tm_printf((UB*)"[sup] width cal failed: %s\n",
				  (fl & WM_F_NONE) ? "nothing straight ahead" :
				  (fl & WM_F_OOV_L) ? "the left edge is out of view (box too far "
						      "left, too wide or too close)" :
						      "the right edge is out of view (box too far "
						      "right, too wide or too close)");
			return;
		}
		if (wm_npts >= OW_CAL_MAX_POINTS) {	/* full: drop the oldest */
			for (i = 1; i < OW_CAL_MAX_POINTS; i++) wm_pts[i - 1] = wm_pts[i];
			wm_npts = OW_CAL_MAX_POINTS - 1U;
		}
		pt = &wm_pts[wm_npts++];
		pt->face_mm = face;
		pt->el_half = el;
		pt->er_half = er;
		pt->known_l_mm = wm_known_l;
		pt->known_r_mm = wm_known_r;
		beep(AVOID_BEEP_MS);
		tm_printf((UB*)"[sup] width cal run %d: face %d mm, left %d.%d deg (edge %d mm, reads %d), "
			  "right %d.%d deg (edge %d mm, reads %d)\n", (INT)wm_npts, (INT)face,
			  (INT)(el / 2), (INT)((el % 2) * 5), (INT)wm_known_l,
			  (INT)ow_edge_mm(face, el, wm_cal.sl_e4, wm_cal.kl_e4),
			  (INT)(er / 2), (INT)((er % 2) * 5), (INT)wm_known_r,
			  (INT)ow_edge_mm(face, er, wm_cal.sr_e4, wm_cal.kr_e4));
		okl = (ow_fit_side(wm_pts, (int)wm_npts, OW_SIDE_LEFT, &sl, &kl) == 0);
		okr = (ow_fit_side(wm_pts, (int)wm_npts, OW_SIDE_RIGHT, &sr, &kr) == 0);
		if (!okl || !okr) {
			tm_printf((UB*)"[sup] width cal: %d run(s) so far; the %s edge needs runs at "
				  "least %d mm apart (shift the box sideways)\n", (INT)wm_npts,
				  (!okl && !okr) ? "left and right" : (!okl) ? "left" : "right",
				  (INT)OW_CAL_MIN_SPREAD_MM);
		}
		if (okl) { wm_cal.sl_e4 = sl; wm_cal.kl_e4 = kl; }
		if (okr) { wm_cal.sr_e4 = sr; wm_cal.kr_e4 = kr; }
		if (!okl && !okr) return;
		tm_printf((UB*)"[sup] width cal fit over %d runs: set OW_SL_E4 %d OW_KL_E4 %d "
			  "OW_SR_E4 %d OW_KR_E4 %d in obst_width.h (used now until power-off)\n",
			  (INT)wm_npts, (INT)wm_cal.sl_e4, (INT)wm_cal.kl_e4,
			  (INT)wm_cal.sr_e4, (INT)wm_cal.kr_e4);
		for (i = 0; i < wm_npts; i++) {
			const OW_POINT *q = &wm_pts[i];

			tm_printf((UB*)"[sup] width cal   run %d: left %d mm (edge %d), right %d mm "
				  "(edge %d), width %d mm (box %d)\n", (INT)(i + 1U),
				  (INT)ow_edge_mm(q->face_mm, q->el_half, wm_cal.sl_e4, wm_cal.kl_e4),
				  (INT)q->known_l_mm,
				  (INT)ow_edge_mm(q->face_mm, q->er_half, wm_cal.sr_e4, wm_cal.kr_e4),
				  (INT)q->known_r_mm,
				  (INT)ow_width_mm(q->face_mm, q->el_half, q->er_half, &wm_cal),
				  (INT)(q->known_l_mm + q->known_r_mm));
		}
		return;
	}
	if (!wm_active) {
		return;				/* late: a STOP or the bypass got there first */
	}
	mm = (fl == 0U) ? ow_width_mm(face, el, er, &wm_cal) : -1;
	if (mm > 0) {
		wm_width_mm = (INT)mm;
		tm_log(TM_MAP_OBSTACLE, (UH)mm, 0);
		tm_printf((UB*)"[sup] obstacle width %d mm (face %d mm)\n", (INT)mm, (INT)face);
	} else {
		tm_printf((UB*)"[sup] obstacle width not measured: %s\n",
			  (fl & WM_F_NONE) ? "nothing straight ahead any more" :
			  (fl != 0U) ? "an edge is out of view" :
			  "the sweep is narrower than the calibration's zero width");
	}
	if ((fl & WM_F_NONE) == 0U) {
		BOOL	oov_l = (fl & WM_F_OOV_L) != 0U, oov_r = (fl & WM_F_OOV_R) != 0U;
		int32_t	l, r;
		INT	side = ow_bypass_side(face, el, er, oov_l, oov_r, &wm_cal, &l, &r);
		INT	go = (side != OW_GO_EITHER) ? side : BYPASS_SIDE_DEFAULT;

		wm_side_have = TRUE;
		wm_side = side;
		wm_side_odo = ns.m.odo_mm;
		tm_printf((UB*)"[sup] bypass side %s%s: %s (left edge %d mm%s, right edge %d mm%s)\n",
			  (go > 0) ? "LEFT" : "RIGHT",
			  (side == OW_GO_EITHER) ? " (default)" : "",
			  (oov_l && oov_r) ? "both edges out of view" :
			  oov_l ? "the left edge is out of view" :
			  oov_r ? "the right edge is out of view" :
			  (side == OW_GO_RIGHT) ? "the box reaches further left" :
			  (side == OW_GO_LEFT) ? "the box reaches further right" :
			  "the box is centred",
			  (INT)l, oov_l ? "+" : "", (INT)r, oov_r ? "+" : "");
	}
	width_resume();
}

LOCAL void poll_timers(void)
{
	UW now = now_us();

	if (btn[1].down && !btn[1].long_fired && (now - btn[1].t_down) >= BTN_LONG_US) {
		btn[1].long_fired = TRUE;
		on_button(1, TRUE, TRUE);
	}
	if (beeping && (W)(now - beep_until) >= 0) {
		drv_buzzer(0);
		beeping = FALSE;
	}
	if (st == SYS_CALIBRATE) {
		SYSTIM t;
		tk_get_otm(&t);
		if ((t.lo - cal_t0.lo) > CALIBRATE_TIMEOUT_MS) {
			beep(400);			/* stay in Calibrate (chain 4.1) */
			cal_t0 = t;
		}
#if CAL_AUTO_SPIN
		/* after the IMU's still (level) phase: pivot on the spot so the
		   compass sees a full circle and the line / barcode sensors,
		   130 mm ahead of the axle, sweep across the line */
		if (cal_spin == 0U && (now - cal_spin_t0) >= (UW)CAL_SPIN_DELAY_MS * 1000U) {
			tm_printf((UB*)"[sup] calibrate: spinning %d x %d deg\n",
				  (INT)CAL_SPIN_STEPS, (INT)CAL_SPIN_STEP_DEG);
			cal_spin_next(FALSE);
		} else if (cal_spin >= 1U && cal_spin <= CAL_SPIN_STEPS &&
			   (now - cal_spin_t0) >= (UW)CAL_SPIN_STEP_TIMEOUT_MS * 1000U) {
			/* no reply: the pivot crept to a stop just short of its
			   target, or the reply was lost.  Go on anyway. */
			tm_printf((UB*)"[sup] calibrate: pivot %d gave no reply in %d ms, going on\n",
				  (INT)cal_spin, (INT)CAL_SPIN_STEP_TIMEOUT_MS);
			cal_spin_next(FALSE);
		}
#endif
	}
	if (st == SYS_JUNCTION_ACTION && jct_step == 4U &&
	    (now - jct_t0) >= (UW)JCT_SETTLE_MS * 1000U) {
		/* stopped for long enough: the 90 / 180 deg pivot, FAST
		   (SPD_CRUISE = fast pivot, task_motion.c TURN_FAST_PM) */
		jct_step = 2;
		tm_printf((UB*)"[sup] junction: stopped over the crossing, pivoting %d deg\n",
			  jct_angle);
		motion(MC_TURN_REL, SPD_CRUISE, jct_angle, 0);
	}
#if LETTER_DISCARD_MM > 0
	if (have_letter && st != SYS_OBSTACLE_AVOID &&
	    (UH)(ns.m.odo_mm - pend_odo) > LETTER_DISCARD_MM) {
		have_letter = FALSE;			/* no junction in time */
		sup_stats.letters_dropped++;
	}
#endif
	/* Bypass rejoin: LINE_SEEN_POLLS fresh 5 ms samples in a row with a
	   front line sensor on black.  The sensors are then SENSOR_TO_AXLE_MM
	   ahead of the axle: roll that far (axle on the line), then align. */
	if (st == SYS_OBSTACLE_AVOID &&
	    (byp_step == BY_APPROACH_FAST || byp_step == BY_APPROACH_SLOW)) {
		UW le = line_err_read();
		if (LE_SEQ(le) != byp_line_seq) {		/* a fresh 5 ms sample */
			byp_line_seq = LE_SEQ(le);
			if ((LE_FLAGS(le) & LE_VALID) && (LE_FLAGS(le) & (LE_SEEN_L | LE_SEEN_R))) {
				if (++byp_line_n >= LINE_SEEN_POLLS) {
					tm_printf((UB*)"[sup] bypass: line found\n");
					byp_step_to(BY_ON_LINE);
					motion(MC_DRIVE, BYPASS_SPEED, 0, SENSOR_TO_AXLE_MM);
				}
			} else {
				byp_line_n = 0;
			}
		}
	}
	/* one limit for the whole bypass, no per-step timeout */
	if (st == SYS_OBSTACLE_AVOID && byp_step != BY_NONE &&
	    (now - byp_t0) > (UW)BYPASS_TIMEOUT_MS * 1000U) {
		bypass_fail("bypass took longer than 60 s");
	}
	/* the width sweep never answered: do not leave the car standing */
	if (wm_active && (now - wm_t0) > (UW)WIDTH_TIMEOUT_MS * 1000U) {
		tm_printf((UB*)"[sup] obstacle width: no result in %d ms, carrying on\n",
			  (INT)WIDTH_TIMEOUT_MS);
		width_resume();
	}
}

LOCAL void handle(const SUP_EVENT *ev)
{
	LS_RESULT r;

	sup_stats.events++;
	switch ((EV_CODE)ev->code) {
	case EV_BUTTON_EDGE:
		button_edge(ev->a, ev->t_us);
		break;

	case EV_CALIB_DONE:
		if (st == SYS_CALIBRATE) {
			if (CAL_OK(ev->a)) {
				cal_wait &= ~CAL_WHO(ev->a);
				if (cal_wait == 0) {
					beep(100);
					enter(SYS_IDLE);
				}
			} else {
				beep(400);		/* failed: stay, beep (chain 4.1) */
			}
		}
		break;

	case EV_LINE_LOST:
		if (st == SYS_LINE_FOLLOW || st == SYS_HUMP_TRAVERSE) {
			/* Line Following holds the last error while both
			   sensors are white: + = the line was last seen on
			   the LEFT. */
			start_line_search(LS_REASON_LINE_LOST,
					  (H)LE_ERR(line_err_read()));
		}
		break;

	case EV_LINE_SAMPLE:
	case EV_MOTION_DONE:
		if (st == SYS_LINE_SEARCH) {
			r = line_search_step((EV_CODE)ev->code, ev->a, ns.m.odo_mm);
			if (r.kind == LS_MOVE) {
				(void)motion_cmd_send(r.cmd);
			} else if (r.kind == LS_FOUND) {
				enter(SYS_LINE_FOLLOW);
			}
			/* LS_FAIL: LINE_SEARCH_FAIL is raised -> Fault below */
		} else if (st == SYS_OBSTACLE_AVOID && ev->code == EV_MOTION_DONE &&
			   MD_SEQ(ev->a) == mseq &&
			   MD_RESULT_OF(ev->a) == MD_DONE) {
			bypass_next();
		} else if (st == SYS_JUNCTION_ACTION && ev->code == EV_MOTION_DONE &&
			   MD_SEQ(ev->a) == mseq &&
			   MD_RESULT_OF(ev->a) != MD_ABORTED) {
			/* Only the reply to OUR last command: the FOLLOW being
			   replaced also replies (ABORTED) and must not count. */
			if (jct_step == 1) {
				/* Over the crossing and braked.  Stand still for
				   JCT_SETTLE_MS (poll_timers) before the pivot, so
				   it starts from rest and the encoder angle is
				   not disturbed by the car still rolling. */
				jct_step = 4;
				jct_t0 = now_us();
			} else if (jct_step == 2 && compass_fix_needed()) {
				jct_step = 3;			/* one correction */
			} else {
				/* The line sensors straddle the line, so both
				   white is normal here: just follow.  If the
				   turn missed, line-lost starts the search. */
				jct_step = 0;
				enter(SYS_LINE_FOLLOW);
			}
		} else if (st == SYS_CALIBRATE && ev->code == EV_MOTION_DONE &&
			   cal_spin >= 1U && cal_spin <= CAL_SPIN_STEPS &&
			   MD_SEQ(ev->a) == mseq &&
			   MD_RESULT_OF(ev->a) != MD_ABORTED) {
			/* auto spin: next pivot, or stop (done / stalled) */
			tm_printf((UB*)"[sup] calibrate: pivot %d of %d finished (result %d)\n",
				  (INT)cal_spin, (INT)CAL_SPIN_STEPS, (INT)MD_RESULT_OF(ev->a));
			cal_spin_next(MD_RESULT_OF(ev->a) != MD_DONE);
		}
		break;

	case EV_BARCODE:
		if (SYS_IS_RUNNING(st) && BC_LETTER(ev->a) < 26) {
			tm_printf((UB*)"[sup] barcode '%c' stored -> %s at the next junction\n",
				  (INT)('A' + BC_LETTER(ev->a)),
				  map_move_name(map_active[BC_LETTER(ev->a)]));
			have_letter = TRUE;		/* store the LETTER (step 5) */
			pend_letter = (UB)BC_LETTER(ev->a);
			pend_odo = (UH)ev->b;
			sup_stats.last_letter = pend_letter;
			sup_stats.last_letter_rev = (UB)BC_REVERSED(ev->a);
			tm_log(TM_MAP_BARCODE, pend_letter, ev->a);
		}
		break;

	case EV_JUNCTION:
		if (st == SYS_LINE_FOLLOW) {
			junction((JCT_TYPE)ev->a);
		}
		break;

	case EV_HUMP_CLIMB:
		if (st == SYS_LINE_FOLLOW) {
			enter(SYS_HUMP_TRAVERSE);
		}
		break;

	case EV_HUMP_LEVEL:
		sup_stats.humps++;
		tm_log(TM_MAP_HUMP, (UH)HUMP_HEIGHT_MM(ev->a), ev->a);
		if (st == SYS_HUMP_TRAVERSE) {
			enter(SYS_LINE_FOLLOW);
		}
		break;

	case EV_OBST_DETECT:
		/* CH-1.18: obstacles handled at the Running level, every substate */
		if (SYS_IS_RUNNING(st) && st != SYS_OBSTACLE_AVOID) {
			byp_stop_cm = ev->a;
			enter(SYS_OBSTACLE_AVOID);
		}
		break;

	case EV_OBST_AHEAD:
		/* something under D_WIDTH_CM while following the line: stop and
		   measure its width.  MC_STOP first (it drops the held FOLLOW),
		   then the sweep. */
		if (st == SYS_LINE_FOLLOW && !wm_active) {
			wm_active = TRUE;
			wm_side_have = FALSE;		/* a new obstacle */
			wm_t0 = now_us();
			tm_printf((UB*)"[sup] obstacle ahead at %d cm: stopping to measure its width\n",
				  (INT)ev->a);
			motion(MC_STOP, SPD_STOP, 0, 0);
			scan(SCAN_PROFILE, 0);
		}
		break;

	case EV_OBST_PROFILE:
		width_result(ev->a);
		break;

	case EV_OBST_SIDE:
		if (st == SYS_OBSTACLE_AVOID && byp_step == BY_SCAN) {
			bypass_side_result(ev->a, ev->b);
		}
		break;

	case EV_OBST_NEAR:
		if (st == SYS_OBSTACLE_AVOID && byp_step == BY_PASS) {
			bypass_near(ev->a);
		}
		break;

	case EV_MAP_NEW:
		/* CH-1.14: the whole table arrives in the event (a, b) and is
		   swapped in here, between two events, so never mid-junction.
		   A held letter is looked up only at the junction, so it uses
		   this new table (design review gap check, row 1). */
		{
			LOCAL char	txt[300];
			UINT		k, mask = 0;

			UB		d;

			map_unpack(ev->a, ev->b, map_active, &d);
			map_default = d;
			for (k = 0; k < 26U; k++) {
				if (map_active[k] != MAP_FORWARD) mask |= 1UL << k;
			}
			(void)map_format(map_active, mask, txt, sizeof(txt));
			tm_printf((UB*)"[sup] new mapping: %s (all other letters forward); "
				  "a junction with no barcode -> %s\n",
				  txt, map_move_name(map_default));
			beep(100);
		}
		break;

	case EV_REMOTE:
		/* PC START / STOP buttons (mqtt_utk.c -> remote_cmd_rx).  The
		   state is checked again here: it may have changed since the
		   Wi-Fi task wrote its reply. */
		if (ev->a == REMOTE_START) {
			if (st == SYS_IDLE) {
				tm_printf((UB*)"[sup] remote START\n");
				enter(SYS_LINE_FOLLOW);
			} else {
				tm_printf((UB*)"[sup] remote START ignored in state %d (only from Idle)\n",
					  (INT)st);
			}
		} else if (ev->a == REMOTE_STOP) {
			if (SYS_IS_RUNNING(st) || st == SYS_FINISHED || st == SYS_CALIBRATE) {
				tm_printf((UB*)"[sup] remote STOP\n");
				enter(SYS_IDLE);
			} else {
				tm_printf((UB*)"[sup] remote STOP: already stopped (state %d)\n",
					  (INT)st);
			}
		} else if (ev->a == REMOTE_MODE_SHORT) {
			/* exactly what the MODE button does on release */
			tm_printf((UB*)"[sup] remote MODE (short) in state %d\n", (INT)st);
			on_button(1, FALSE, FALSE);
		} else if (ev->a == REMOTE_MODE_LONG) {
			/* exactly what holding MODE for 2 s does */
			tm_printf((UB*)"[sup] remote MODE (long) in state %d\n", (INT)st);
			on_button(1, TRUE, TRUE);
		} else if (ev->a == REMOTE_BYPASS_DEPTH) {
			/* any state; a running bypass keeps its latched depth.
			   remote_cmd_rx checked the range: clamp anyway. */
			W d = (W)ev->b;
			if (d < BG_DEPTH_MIN_MM) d = BG_DEPTH_MIN_MM;
			if (d > BG_DEPTH_MAX_MM) d = BG_DEPTH_MAX_MM;
			byp_depth_pref = d;
			beep(AVOID_BEEP_MS);
			tm_printf((UB*)"[sup] bypass depth set to %d mm%s\n", (INT)d,
				  (st == SYS_OBSTACLE_AVOID) ? " (from the next obstacle)" : "");
		} else if (ev->a == REMOTE_WIDTH_CAL && ev->b == 0U) {
			/* "wcal clear": any state; the constants in use stay */
			wm_npts = 0;
			tm_printf((UB*)"[sup] width cal runs cleared (still using OW_SL_E4 %d OW_KL_E4 %d "
				  "OW_SR_E4 %d OW_KR_E4 %d)\n", (INT)wm_cal.sl_e4, (INT)wm_cal.kl_e4,
				  (INT)wm_cal.sr_e4, (INT)wm_cal.kr_e4);
		} else if (ev->a == REMOTE_WIDTH_CAL || ev->a == REMOTE_EDGE_SWEEP) {
			/* Idle only (remote_cmd_rx checked; the state may have changed) */
			if (st == SYS_IDLE && wm_idle == WM_IDLE_NONE) {
				if (ev->a == REMOTE_WIDTH_CAL) {
					wm_idle = WM_IDLE_CAL;
					wm_known_l = (W)(H)WCAL_L(ev->b);
					wm_known_r = (W)(H)WCAL_R(ev->b);
					tm_printf((UB*)"[sup] width calibration: left edge %d mm, right "
						  "edge %d mm, sweeping\n", (INT)wm_known_l, (INT)wm_known_r);
				} else {
					wm_idle = WM_IDLE_SWEEP;
					tm_printf((UB*)"[sup] edge sweep: sweeping\n");
				}
				scan(SCAN_PROFILE, 0);
			} else {
				tm_printf((UB*)"[sup] width sweep ignored in state %d%s (only from Idle)\n",
					  (INT)st, (wm_idle != WM_IDLE_NONE) ? ", one already running" : "");
			}
		}
		break;

	case EV_FAULT:
	default:
		break;		/* the flag itself is checked after every event */
	}
}

/* ------------------------------------------------------------------ */
/* Read-only views for the snapshot (other tasks).  A byte copy: a table
   swap in the middle can at worst mix old and new for one reply. */
EXPORT void sup_map_get(UB out[26])
{
	UINT k;
	for (k = 0; k < 26U; k++) out[k] = map_active[k];
}

EXPORT INT sup_pending_letter(void)
{
	return have_letter ? (INT)pend_letter : -1;
}

EXPORT INT sup_bypass_side(void)
{
	if (st == SYS_OBSTACLE_AVOID) return byp_side;
	if (wm_side_have && wm_side != OW_GO_EITHER) return wm_side;
	return BYPASS_SIDE_DEFAULT;
}

EXPORT INT sup_bypass_depth(void)
{
	return (INT)byp_depth_pref;
}

EXPORT INT sup_obstacle_width_mm(void)
{
	return wm_width_mm;
}

/* Called from the Wi-Fi task (mqtt_utk.c) with a map-topic payload.
   Parses here, applies in the Supervisor task via EV_MAP_NEW. */
EXPORT int32_t map_rx_payload(const uint8_t *p, uint32_t len, char *reply, uint32_t reply_size)
{
	uint8_t		t[26], d = (uint8_t)MP_FORWARD;
	uint32_t	mask = 0, a, b, used;
	int32_t		r;

	if (reply_size < 64U) return -9;
	r = map_parse(p, len, t, &mask, &d);
	if (r < 0) {
		(void)tm_sprintf((UB*)reply, (const UB*)"error %d: %s (example: "
				 "A=forward,B=left,C=right,D=uturn,default=forward)", (INT)r,
				 (r == MP_ERR_WORD) ? "unknown word" :
				 (r == MP_ERR_PAIR) ? "a letter or movement has no partner" :
				 "no letter=movement pair");
		return r;
	}
	map_pack(t, d, &a, &b);
	if (sup_post(SRC_TELEMETRY, EV_MAP_NEW, a, b) < E_OK) {
		(void)tm_sprintf((UB*)reply, (const UB*)"error: supervisor queue full, send again");
		return -8;
	}
	used = (uint32_t)tm_sprintf((UB*)reply, (const UB*)"ok %d: ", (INT)r);
	used += (uint32_t)map_format(t, mask, reply + used, reply_size - used);
	(void)tm_sprintf((UB*)(reply + used), (const UB*)", no barcode -> %s",
			 map_move_name(d));
	return r;
}

/* Case-insensitive: does p[0..len) equal word, ignoring spaces / CR LF? */
LOCAL BOOL payload_is(const uint8_t *p, uint32_t len, const char *word)
{
	uint32_t i = 0;

	while (i < len && (p[i] == ' ' || p[i] == '\t' || p[i] == '\r' || p[i] == '\n')) i++;
	while (*word != '\0') {
		uint8_t c;
		if (i >= len) return FALSE;
		c = p[i++];
		if (c >= 'A' && c <= 'Z') c = (uint8_t)(c - 'A' + 'a');
		if (c != (uint8_t)*word++) return FALSE;
	}
	while (i < len && (p[i] == ' ' || p[i] == '\t' || p[i] == '\r' || p[i] == '\n')) i++;
	return i == len;
}

#define IS_WS(c)	((c) == ' ' || (c) == '\t' || (c) == '\r' || (c) == '\n')

/* "word <number>" with an optional "mm" after the number, case-insensitive,
   spaces around ignored.  signed_ok: a leading '-' or '+' is allowed and
   *val is then a two's-complement W.  1 = matched (*val set), 0 = not
   this word, -1 = this word but no usable number. */
LOCAL INT payload_num(const uint8_t *p, uint32_t len, const char *word, UW *val,
		      BOOL signed_ok)
{
	uint32_t i = 0, digits = 0;
	UW	 v = 0;
	BOOL	 neg = FALSE;

	while (i < len && IS_WS(p[i])) i++;
	while (*word != '\0') {
		uint8_t c;
		if (i >= len) return 0;
		c = p[i++];
		if (c >= 'A' && c <= 'Z') c = (uint8_t)(c - 'A' + 'a');
		if (c != (uint8_t)*word++) return 0;
	}
	if (i < len && !IS_WS(p[i])) return 0;		/* a longer word */
	while (i < len && IS_WS(p[i])) i++;
	if (signed_ok && i < len && (p[i] == '-' || p[i] == '+')) {
		neg = (p[i] == '-');
		i++;
	}
	while (i < len && p[i] >= '0' && p[i] <= '9') {
		if (++digits > 5) return -1;
		v = v * 10U + (UW)(p[i++] - '0');
	}
	if (digits == 0) return -1;
	while (i < len && IS_WS(p[i])) i++;
	if (i + 1 < len && (p[i] | 0x20) == 'm' && (p[i + 1] | 0x20) == 'm') i += 2;
	while (i < len && IS_WS(p[i])) i++;
	if (i != len) return -1;
	*val = neg ? (UW)(-(W)v) : v;
	return 1;
}

/* Unsigned "word <number>" (depth) */
LOCAL INT payload_number(const uint8_t *p, uint32_t len, const char *word, UW *val)
{
	return payload_num(p, len, word, val, FALSE);
}

/* "word <n1> [<n2>]", signed, an optional "mm" after each, case-insensitive.
   Returns how many numbers (1 or 2), 0 = not this word, -1 = this word
   but not 1 or 2 usable numbers. */
LOCAL INT payload_two(const uint8_t *p, uint32_t len, const char *word, W *a, W *b)
{
	uint32_t i = 0;
	INT	 n = 0;

	while (i < len && IS_WS(p[i])) i++;
	while (*word != '\0') {
		uint8_t c;
		if (i >= len) return 0;
		c = p[i++];
		if (c >= 'A' && c <= 'Z') c = (uint8_t)(c - 'A' + 'a');
		if (c != (uint8_t)*word++) return 0;
	}
	if (i < len && !IS_WS(p[i])) return 0;		/* a longer word */
	for (;;) {
		BOOL	 neg = FALSE;
		uint32_t digits = 0;
		W	 v = 0;

		while (i < len && IS_WS(p[i])) i++;
		if (i >= len) break;
		if (n >= 2) return -1;			/* a third thing */
		if (p[i] == '-' || p[i] == '+') neg = (p[i++] == '-');
		while (i < len && p[i] >= '0' && p[i] <= '9') {
			if (++digits > 4) return -1;
			v = v * 10 + (W)(p[i++] - '0');
		}
		if (digits == 0) return -1;
		if (i + 1 < len && (p[i] | 0x20) == 'm' && (p[i + 1] | 0x20) == 'm') i += 2;
		if (i < len && !IS_WS(p[i])) return -1;
		if (n == 0) *a = neg ? -v : v; else *b = neg ? -v : v;
		n++;
	}
	return (n == 0) ? -1 : n;
}

/* Called from the Wi-Fi task (mqtt_utk.c) with a cmd-topic payload.
   Applied in the Supervisor task via EV_REMOTE. */
EXPORT int32_t remote_cmd_rx(const uint8_t *p, uint32_t len, char *reply, uint32_t reply_size)
{
	LOCAL const char *const name[SYS_STATE_COUNT] = {
		"Init", "Idle", "Calibrate", "LineFollow", "Junction", "Hump",
		"Obstacle", "LineSearch", "Finished", "Fault"
	};
	SYS_STATE	s = sys_state_get();
	const char	*now = name[(s < SYS_STATE_COUNT) ? s : 0];
	UW		cmd, value = 0;
	INT		got;
	W		el_mm = 0, er_mm = 0;

	/* the longest reply (wcal error) is ~140 B; mqtt_utk.c passes 160 */
	if (reply_size < 160U) return -9;
	if (payload_is(p, len, "start")) {
		cmd = REMOTE_START;
		if (s != SYS_IDLE) {
			(void)tm_sprintf((UB*)reply, (const UB*)"refused start: car is in %s, "
					 "start only from Idle%s", now,
					 (s == SYS_FAULT) ? " (clear the fault with MODE)" : "");
			return -2;
		}
	} else if (payload_is(p, len, "stop")) {
		cmd = REMOTE_STOP;
	} else if (payload_is(p, len, "mode")) {
		cmd = REMOTE_MODE_SHORT;
		if (s != SYS_FAULT && s != SYS_CALIBRATE) {
			(void)tm_sprintf((UB*)reply, (const UB*)"refused mode: car is in %s; "
					 "MODE short only clears a Fault or leaves Calibrate", now);
			return -2;
		}
	} else if (payload_is(p, len, "calibrate")) {
		cmd = REMOTE_MODE_LONG;
		if (s != SYS_IDLE) {
			(void)tm_sprintf((UB*)reply, (const UB*)"refused calibrate: car is in %s, "
					 "calibrate only from Idle", now);
			return -2;
		}
	} else if ((got = payload_number(p, len, "depth", &value)) != 0) {
		cmd = REMOTE_BYPASS_DEPTH;		/* any state */
		if (got < 0 || value < BG_DEPTH_MIN_MM || value > BG_DEPTH_MAX_MM) {
			(void)tm_sprintf((UB*)reply, (const UB*)"error: depth needs a whole "
					 "number of mm, %d to %d (example: depth 140)",
					 (INT)BG_DEPTH_MIN_MM, (INT)BG_DEPTH_MAX_MM);
			return -3;
		}
	} else if (payload_is(p, len, "wcal clear")) {
		cmd = REMOTE_WIDTH_CAL;			/* value 0 = clear, any state */
	} else if (payload_is(p, len, "esweep")) {
		cmd = REMOTE_EDGE_SWEEP;
		if (s != SYS_IDLE) {
			(void)tm_sprintf((UB*)reply, (const UB*)"refused esweep: car is in %s, "
					 "edge sweep only from Idle", now);
			return -2;
		}
	} else if ((got = payload_two(p, len, "wcal", &el_mm, &er_mm)) != 0) {
		/* "wcal <left> <right>": each edge's distance from the centre
		   line, + = out on its side (Edge calibration in the panel) */
		cmd = REMOTE_WIDTH_CAL;
		if (got != 2 ||
		    el_mm < WM_EDGE_MIN_MM || el_mm > WM_EDGE_MAX_MM ||
		    er_mm < WM_EDGE_MIN_MM || er_mm > WM_EDGE_MAX_MM ||
		    el_mm + er_mm < WM_CAL_MIN_MM || el_mm + er_mm > WM_CAL_MAX_MM) {
			(void)tm_sprintf((UB*)reply, (const UB*)"error: wcal needs each edge from the "
					 "centre line, %d to %d mm, adding up to %d to %d mm "
					 "(e.g. wcal 120 30)",
					 (INT)WM_EDGE_MIN_MM, (INT)WM_EDGE_MAX_MM,
					 (INT)WM_CAL_MIN_MM, (INT)WM_CAL_MAX_MM);
			return -3;
		}
		if (s != SYS_IDLE) {
			(void)tm_sprintf((UB*)reply, (const UB*)"refused wcal: car is in %s, "
					 "width calibration only from Idle", now);
			return -2;
		}
		value = WCAL_B(el_mm, er_mm);
	} else {
		(void)tm_sprintf((UB*)reply, (const UB*)"error: unknown command (use start, "
				 "stop, mode, calibrate, depth <mm>, wcal <left> <right>, "
				 "wcal clear or esweep)");
		return -1;
	}
	if (sup_post(SRC_TELEMETRY, EV_REMOTE, cmd, value) < E_OK) {
		(void)tm_sprintf((UB*)reply, (const UB*)"error: supervisor queue full, send again");
		return -8;
	}
	if (cmd == REMOTE_BYPASS_DEPTH) {
		(void)tm_sprintf((UB*)reply, (const UB*)"ok depth %d mm (from the next obstacle; car is %s)",
				 (INT)value, now);
		return 0;
	}
	if (cmd == REMOTE_WIDTH_CAL && value == 0U) {
		(void)tm_sprintf((UB*)reply, (const UB*)"ok wcal clear: calibration runs forgotten");
		return 0;
	}
	if (cmd == REMOTE_WIDTH_CAL) {
		(void)tm_sprintf((UB*)reply, (const UB*)"ok wcal left %d mm right %d mm: sweeping, "
				 "the result is in the log (car is %s)", (INT)el_mm, (INT)er_mm, now);
		return 0;
	}
	if (cmd == REMOTE_EDGE_SWEEP) {
		(void)tm_sprintf((UB*)reply, (const UB*)"ok esweep: sweeping (car is %s)", now);
		return 0;
	}
	(void)tm_sprintf((UB*)reply, (const UB*)"ok %s (car was %s)%s",
			 (cmd == REMOTE_START) ? "start" :
			 (cmd == REMOTE_STOP) ? "stop" :
			 (cmd == REMOTE_MODE_SHORT) ? "mode" : "calibrate", now,
			 (cmd == REMOTE_MODE_SHORT && s == SYS_FAULT) ? ": clearing the fault" :
			 (cmd == REMOTE_MODE_SHORT) ? ": leaving Calibrate" :
			 (cmd == REMOTE_MODE_LONG) ?
#if CAL_AUTO_SPIN
				": keep the car on the line with room to spin; it "
				"turns a full circle by itself" :
#else
				": keep still 1 s, then turn a full circle "
				"across the line within 14 s" :
#endif
				"");
	return 0;
}

LOCAL void btn_isr(UINT gpio, UINT events, UW t_us)
{
	(void)events; (void)t_us;
	/* read the level now; the Supervisor debounces */
	(void)sup_post(SRC_BUTTON, EV_BUTTON_EDGE,
		       BTN_A(gpio, hal_gpio_get(gpio)), 0);
}

EXPORT ER buttons_init(void)
{
	ER er;

	hal_gpio_in(PIN_BTN_START, HAL_PULL_UP);
	hal_gpio_in(PIN_BTN_MODE, HAL_PULL_UP);
	er = gpio_irq_register(PIN_BTN_START, GPIO_EV_FALL | GPIO_EV_RISE, btn_isr);
	if (er < E_OK) return er;
	return gpio_irq_register(PIN_BTN_MODE, GPIO_EV_FALL | GPIO_EV_RISE, btn_isr);
}

EXPORT void supervisor_task(INT stacd, void *exinf)
{
	SUP_EVENT	ev;
	INT		n;
	UINT		i;

	(void)stacd; (void)exinf;
	LOG("sup", "task on core %d\n", (INT)hal_core_id());
	diag_pm_start_here();	/* shared diag_pm.c: the watch alarm on this core */
	/* Default mapping (FR4): A forward, B left, C right, D U-turn, the
	   rest forward.  TODO CH-1.14: replaced over Wi-Fi. */
	for (i = 0; i < 26; i++) {
		map_active[i] = MAP_FORWARD;
	}
	map_active[1] = MAP_LEFT;
	map_active[2] = MAP_RIGHT;
	map_active[3] = MAP_UTURN;
	map_default = (UB)MAP_FORWARD;		/* no barcode: straight across */
	sup_stats.last_letter = BC_LETTER_INVALID;
	nav_read(&ns);
	enter((fault_peek() & FAULT_TRIP_MASK) ? SYS_FAULT : SYS_IDLE);

	for (;;) {
		UW t0;

		n = tk_rcv_mbf(mbf_sup_event, &ev, SUP_POLL_MS);
		t0 = wcet_release(WCET_SUPERVISOR);
		nav_read(&ns);

		if (n == (INT)sizeof(ev)) {
			handle(&ev);
		}
		poll_timers();
		diag_core0_watch();	/* shared diag.c: watches core 0 from core 1 */

		/* Fault check after EVERY wake-up, event or not */
		if (st != SYS_FAULT && (fault_peek() & FAULT_TRIP_MASK)) {
			enter(SYS_FAULT);
		}
		wcet_end(WCET_SUPERVISOR, t0);
	}
}
