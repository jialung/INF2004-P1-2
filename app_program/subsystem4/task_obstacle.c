/*
 *      task_obstacle.c
 *      Obstacle Scan task, 50 ms + on request, core 0, priority 13.
 *      Owner: Lui Khee.  Plan refs: LK-1.7, LK-1.10..LK-1.16, chain 4.5,
 *      Design Review 6.2, FR9, FR10.
 *
 *      Modes (SCAN_REQ from the Supervisor):
 *        GUARD       servo ahead, ping every 50 ms; 2 of 3 < D_DETECT or one
 *                    < D_EMERG -> OBST_STOP; cleared with hysteresis
 *        EMERG_ONLY  as GUARD but only D_EMERG stops (bypass legs)
 *        INHIBIT     no pings, servo ahead
 *        PROFILE     car stopped: the width sweep (12 Nov).  Face straight
 *                    ahead, then each side outwards in 5 deg looks to the
 *                    first miss and 1 deg looks to pin the boundary.
 *                    -> EV_OBST_PROFILE (WM_*), then INHIBIT.
 *        GUARD also sends EV_OBST_AHEAD once when 2 of 3 readings are under
 *        D_WIDTH_CM: the Supervisor stops the car and asks for PROFILE.
 *        SIDE_PROBE  car stopped: 7 looks at 0, 5, ... 30 deg towards the
 *                    side in the request param -> EV_OBST_SIDE, then INHIBIT.
 *        PASS_LEFT / PASS_RIGHT  servo at SCAN_PASS_DEG to that side, ping
 *                    every 50 ms; when 2 of the last 3 pings put a point
 *                    closer sideways than param -> EV_OBST_NEAR once, then
 *                    INHIBIT.  Never sets OBST_STOP (it is beside the car).
 *
 *      Geometry: angle a from straight ahead, + = left.  A reading of d cm
 *      at a is the point (forward d cos a, left d sin a) from the sensor,
 *      which sits on the car's centre line, i.e. on the line being followed.
 */

#include "subsystem4.h"
#include "drv_servo.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../wcet.h"
#include "../subsystem3/imu_math.h"   /* integer sin / cos */
#include "../subsystem5/obst_width.h" /* ow_is_face */

LOCAL SCAN_MODE mode = SCAN_INHIBIT; /* Supervisor sends GUARD on Start */
LOCAL UW        hist[3];             /* last 3 readings, cm (no echo = 400) */
LOCAL INT       n_hist;
LOCAL INT       clear_run;
LOCAL INT       no_rise_run;
LOCAL UINT      pass_param; /* PASS_*: side distance limit, cm */
LOCAL BOOL      profile_due;
LOCAL BOOL      side_due; /* SIDE_PROBE requested, run from the main loop */
LOCAL INT       side_dir; /* +1 left, -1 right */
/* pass guard: last 3 pings, bit 0 = newest, 1 = near */
LOCAL UINT near_hist;
LOCAL BOOL near_sent;
LOCAL BOOL ahead_sent;  /* EV_OBST_AHEAD sent for this obstacle */
LOCAL INT  ahead_clear; /* consecutive readings over D_WIDTH_CLEAR_CM */

EXPORT volatile OBST_STATS obst_stats;

EXPORT SCAN_MODE obstacle_mode (void)
{
    return mode;
}

/* Servo position, degrees from straight ahead (+ = left).  drv_servo_init
   leaves it ahead (0). */
LOCAL INT servo_now;

/* Returns the pulse sent, us.  The side's end calibration (drv_servo.h
   SERVO_END_*_US, + = further out) is scaled with |a|: all of it at
   SCAN_SIDE_MAX_DEG, none straight ahead. */
LOCAL UW servo_raw (INT a)
{
    INT deg = (INT) SERVO_AHEAD_DEG + SERVO_LEFT_SIGN * a;
    W   end = (a > 0) ? SERVO_END_LEFT_US : SERVO_END_RIGHT_US;
    W   adj;
    if (end > SERVO_END_MAX_US)
    {
        end = SERVO_END_MAX_US;
    }
    if (end < -SERVO_END_MAX_US)
    {
        end = -SERVO_END_MAX_US;
    }
    /* out on the left = towards the servo's 180 end if SERVO_LEFT_SIGN +1 */
    adj = (W) SERVO_LEFT_SIGN * ((a > 0) ? end : -end) *
          (W) ((a < 0) ? -a : a) / (W) SCAN_SIDE_MAX_DEG;
    if (deg < 0)
    {
        deg = 0;
    }
    if (deg > 180)
    {
        deg = 180;
    }
    return drv_servo_set_deg_adj((UW) deg, adj);
}

/* Move to a = degrees from straight ahead, + = left, at most max_step deg
   every SERVO_RAMP_MS (one servo PWM frame): a jump at the SG90's full
   speed shakes the sensor off its mount.  Blocks (this task only). */
LOCAL void servo_move (INT a, INT max_step)
{
    if (a > SCAN_SIDE_MAX_DEG)
    {
        a = SCAN_SIDE_MAX_DEG;
    }
    if (a < -SCAN_SIDE_MAX_DEG)
    {
        a = -SCAN_SIDE_MAX_DEG;
    }
    while (servo_now != a)
    {
        INT step = a - servo_now;
        if (step > max_step)
        {
            step = max_step;
        }
        if (step < -max_step)
        {
            step = -max_step;
        }
        servo_now += step;
        servo_raw(servo_now);
        tk_dly_tsk(SERVO_RAMP_MS);
    }
}

/* The normal speed (SERVO_RAMP_DEG per frame, 100 deg/s): 60 deg ~ 0.6 s */
LOCAL void servo_to (INT a)
{
    servo_move(a, SERVO_RAMP_DEG);
}

LOCAL void push (UW cm)
{
    hist[2] = hist[1];
    hist[1] = hist[0];
    hist[0] = cm;
    if (n_hist < 3)
    {
        n_hist++;
    }
}

LOCAL void set_mode (SCAN_MODE m, UINT param)
{
    /* Idle, a turn, a bypass: the next obstacle is a new one */
    if (m == SCAN_INHIBIT)
    {
        ahead_sent = FALSE;
    }
    if (m == mode && m != SCAN_PROFILE && m != SCAN_SIDE_PROBE)
    {
        return;
    }
    /* Leaving the stopping modes: release OBST_STOP, since only GUARD /
       EMERG_ONLY may hold it.  This is what lets a bypass start while the
       box is still in front (fault_flags.h). */
    if (m != SCAN_GUARD && m != SCAN_EMERG_ONLY)
    {
        fault_clear(FAULT_OBST_STOP);
    }
    mode        = m;
    n_hist      = 0;
    clear_run   = 0;
    no_rise_run = 0; /* US_FAULT counts consecutive pings per run */
    switch (m)
    {
        case SCAN_PASS_LEFT:
            servo_to(+SCAN_PASS_DEG);
            break;
        case SCAN_PASS_RIGHT:
            servo_to(-SCAN_PASS_DEG);
            break;
        case SCAN_PROFILE:
            profile_due = TRUE;
            break;
        case SCAN_SIDE_PROBE:
            /* the servo is moved by the probe itself */
            side_due = TRUE;
            side_dir = (param == SIDE_PROBE_LEFT) ? +1 : -1;
            break;
        default:
            servo_to(0);
            break;
    }
    if (m == SCAN_PASS_LEFT || m == SCAN_PASS_RIGHT)
    {
        pass_param = (param != 0) ? param : PASS_SIDE_CM;
        near_hist  = 0;
        near_sent  = FALSE;
    }
}

/* One ping, cm; no echo / nothing in range = 400.  Counts US_FAULT. */
LOCAL UW ping (void)
{
    UW        cm = 400;
    US_RESULT r  = us_ping(&cm);

    if (r == US_NO_RISE)
    {
        if (++no_rise_run >= US_FAIL_LIMIT)
        {
            fault_raise(FAULT_US_FAULT); /* LK-1.7 */
        }
        return 400;
    }
    no_rise_run = 0;
    return (r == US_NO_ECHO) ? 400 : cm;
}

/* GUARD only: something under D_WIDTH_CM -> EV_OBST_AHEAD, once per
   obstacle.  Armed again after 3 readings over D_WIDTH_CLEAR_CM, or by a
   SCAN_INHIBIT request (set_mode).  hold: the car is being stopped for
   this obstacle anyway (D_DETECT / D_EMERG), so there is nothing to
   measure first. */
LOCAL void ahead_check (UW cm, BOOL hold)
{
    INT below = 0, i;

    if (cm > D_WIDTH_CLEAR_CM)
    {
        if (++ahead_clear >= 3)
        {
            ahead_sent = FALSE;
        }
    }
    else
    {
        ahead_clear = 0;
    }
    if (ahead_sent || hold)
    {
        return;
    }
    for (i = 0; i < n_hist; i++)
    {
        if (hist[i] < D_WIDTH_CM)
        {
            below++;
        }
    }
    if (below >= 2)
    {
        ahead_sent = TRUE;
        (void) sup_post(SRC_OBSTACLE, EV_OBST_AHEAD, cm, 0);
    }
}

LOCAL void judge (UW cm)
{
    BOOL stop_now = FALSE;
    INT  below    = 0, i;
    BOOL stopped  = (fault_peek() & FAULT_OBST_STOP) != 0;

    push(cm);
    for (i = 0; i < n_hist; i++)
    {
        if (hist[i] < D_DETECT_CM)
        {
            below++;
        }
    }
    if (cm < D_EMERG_CM)
    {
        stop_now = TRUE;
    }
    else if (mode == SCAN_GUARD && below >= 2)
    {
        stop_now = TRUE;
    }

    if (stop_now && !stopped)
    {
        fault_raise(FAULT_OBST_STOP); /* Motion cuts in <= 10 ms */
        (void) sup_post(SRC_OBSTACLE, EV_OBST_DETECT, cm, 0);
        clear_run = 0;
    }
    else if (stopped)
    {
        clear_run = (cm > D_CLEAR_HYST_CM) ? clear_run + 1 : 0;
        if (clear_run >= 3)
        {
            fault_clear(FAULT_OBST_STOP); /* path clear */
            clear_run = 0;
        }
    }
    if (mode == SCAN_GUARD)
    {
        ahead_check(cm, stop_now || stopped);
    }
}

/* ---- PROFILE (LK-1.11) ------------------------------------------------ */

/* A new request arrived: abort the sweep (Stop, Fault, ...) */
LOCAL BOOL interrupted (void)
{
    UW req;

    if (tk_rcv_mbf(mbf_scan_req, &req, TMO_POL) == (INT) sizeof(req))
    {
        set_mode(SCAN_MODE_OF(req), SCAN_PARAM(req));
        return TRUE;
    }
    return FALSE;
}

/* servo to a, let it settle, ping twice, keep the nearer.  FALSE if a
   new request arrived meanwhile. */
LOCAL BOOL look (INT a, UINT settle_ms, UW * cm)
{
    UW c1, c2;

    servo_to(a);
    tk_dly_tsk(settle_ms);
    if (interrupted())
    {
        return FALSE;
    }
    c1 = ping();
    tk_dly_tsk(SCAN_PING_GAP_MS);
    c2  = ping();
    *cm = (c1 < c2) ? c1 : c2;
    return TRUE;
}

LOCAL W lateral_cm (INT a, UW cm)
{
    return ((W) cm * imu_sin_q14(a)) >> 14;
}

/* ---- WIDTH SWEEP (12 Nov) ----------------------------------------------- */
/* Car stopped about 35 cm from the box.  The straight-ahead reading is the
   box face; then, on each side, the servo walks outwards until the readings
   stop being that face.  subsystem5/obst_width.h turns the two boundaries
   into a width.  Every look is printed, so the sweep can be checked against
   a tape measure when tuning. */

/* One ping in mm; no echo / out of range = WS_NO_ECHO_MM.  Counts US_FAULT. */
LOCAL UW ping_mm (void)
{
    UW        mm = WS_NO_ECHO_MM;
    US_RESULT r  = us_ping_mm(&mm);

    if (r == US_NO_RISE)
    {
        if (++no_rise_run >= US_FAIL_LIMIT)
        {
            fault_raise(FAULT_US_FAULT);
        }
        return WS_NO_ECHO_MM;
    }
    no_rise_run = 0;
    return (r == US_NO_ECHO) ? WS_NO_ECHO_MM : mm;
}

/* Servo to a, settle, ping (twice if the first is not the face).  *hit =
   a reading is the box face (the same forward distance as straight ahead).
   FALSE if a new request arrived meanwhile. */
LOCAL BOOL face_look (INT a, UW face_mm, BOOL * hit, UW * shown)
{
    UW c;

    servo_move(a, WS_RAMP_DEG); /* slow and even (subsystem4.h) */
    tk_dly_tsk(WS_SETTLE_MS);
    if (interrupted())
    {
        return FALSE;
    }
    c      = ping_mm();
    *hit   = (ow_is_face((INT) c, a, (INT) face_mm) != 0) ? TRUE : FALSE;
    *shown = c;
    if (!*hit)
    {
        tk_dly_tsk(SCAN_PING_GAP_MS);
        c = ping_mm();
        if (ow_is_face((INT) c, a, (INT) face_mm) != 0)
        {
            *hit   = TRUE;
            *shown = c;
        }
    }
    tm_printf((UB *) "[obst] sweep %d deg: %d mm %s\n", a, (INT) *shown,
              *hit ? "face" : "off");
    return TRUE;
}

/* One side (dir +1 left, -1 right): coarse looks every WS_COARSE_DEG up to
   the first miss, then 1 deg looks between the last hit and that miss.
   *half = last hit deg + first miss deg.  *oov = still on the box at the
   servo limit: that edge is out of view.  FALSE if interrupted. */
LOCAL BOOL edge_side (INT dir, UW face_mm, UINT * half, BOOL * oov)
{
    INT  hit_deg = 0, miss, a;
    BOOL hit;
    UW   shown;

    *oov = FALSE;
    for (a = WS_COARSE_DEG;; a += WS_COARSE_DEG)
    {
        if (a > SCAN_SIDE_MAX_DEG)
        { /* never missed */
            *oov  = TRUE;
            *half = 2U * SCAN_SIDE_MAX_DEG;
            return TRUE;
        }
        if (!face_look(dir * a, face_mm, &hit, &shown))
        {
            return FALSE;
        }
        if (!hit)
        {
            break;
        }
        hit_deg = a;
    }
    miss = a;
    for (a = hit_deg + 1; a < miss; a++)
    {
        if (!face_look(dir * a, face_mm, &hit, &shown))
        {
            return FALSE;
        }
        if (!hit)
        {
            miss = a;
            break;
        }
        hit_deg = a;
    }
    *half = (UINT) (hit_deg + miss);
    return TRUE;
}

LOCAL void width_sweep (void)
{
    UW   rd[3], face = 0, t;
    UINT el = 0, er = 0, flags = 0, i;
    BOOL oov_l = FALSE, oov_r = FALSE;

    profile_due = FALSE;
    obst_stats.profiles++;

    /* the face straight ahead: the middle of 3 readings */
    servo_move(0, WS_RAMP_DEG);
    tk_dly_tsk(SCAN_COARSE_SETTLE_MS);
    if (interrupted())
    {
        return;
    }
    for (i = 0; i < 3; i++)
    {
        rd[i] = ping_mm();
        tk_dly_tsk(SCAN_PING_GAP_MS);
    }
    if (rd[0] > rd[1])
    {
        t     = rd[0];
        rd[0] = rd[1];
        rd[1] = t;
    }
    if (rd[1] > rd[2])
    {
        t     = rd[1];
        rd[1] = rd[2];
        rd[2] = t;
    }
    if (rd[0] > rd[1])
    {
        t     = rd[0];
        rd[0] = rd[1];
        rd[1] = t;
    }
    face = rd[1];
    tm_printf((UB *) "[obst] sweep start: ahead %d %d %d mm\n", (INT) rd[0],
              (INT) rd[1], (INT) rd[2]);

    if (face > WS_FACE_MAX_MM)
    {
        flags = WM_F_NONE; /* the box is gone */
        face  = 0;
    }
    else
    {
        if (!edge_side(+1, face, &el, &oov_l))
        {
            return;
        }
        if (!edge_side(-1, face, &er, &oov_r))
        {
            return;
        }
        if (oov_l)
        {
            flags |= WM_F_OOV_L;
        }
        if (oov_r)
        {
            flags |= WM_F_OOV_R;
        }
    }
    servo_move(0, WS_RAMP_DEG);

    tm_printf((UB *) "[obst] width sweep: face %d mm, left %d.%d deg, right "
                     "%d.%d deg%s%s%s\n",
              (INT) face, (INT) (el / 2U), (INT) ((el % 2U) * 5U),
              (INT) (er / 2U), (INT) ((er % 2U) * 5U),
              oov_l ? " (left edge out of view)" : "",
              oov_r ? " (right edge out of view)" : "",
              (flags & WM_F_NONE) ? " (nothing ahead)" : "");
    (void) sup_post(SRC_OBSTACLE, EV_OBST_PROFILE, WM_PACK(face, el, er, flags),
                    0);
    mode = SCAN_INHIBIT;
}

/* ---- SIDE_PROBE (bypass, 11 Nov) ---------------------------------------- */
/* 7 looks at 0, 5, ... 30 deg towards side_dir, car stopped.  Ranges are
   capped at SIDE_R_MAX_CM (no echo, 400, -> 63) to fit the 6-bit fields. */
LOCAL void side_probe (void)
{
    UW   cm, a = 0, b = 0, r[SIDE_LOOKS];
    UINT i;

    side_due = FALSE;
    for (i = 0; i < SIDE_LOOKS; i++)
    {
        if (!look(side_dir * (INT) (i * SIDE_STEP_DEG), SCAN_FINE_SETTLE_MS,
                  &cm))
        {
            return; /* a new request arrived: mode already changed */
        }
        /* no echo (400) -> 63 */
        r[i] = (cm > SIDE_R_MAX_CM) ? SIDE_R_MAX_CM : cm;
        if (i < 5)
        {
            a |= SIDE_PUT(i, r[i]);
        }
        else
        {
            b |= SIDE_PUT(i, r[i]);
        }
    }
    servo_to(0);
    tm_printf((UB *) "[obst] side probe %s: %d %d %d %d %d %d %d cm\n",
              (side_dir > 0) ? "left" : "right", (INT) r[0], (INT) r[1],
              (INT) r[2], (INT) r[3], (INT) r[4], (INT) r[5], (INT) r[6]);
    (void) sup_post(SRC_OBSTACLE, EV_OBST_SIDE, a, b);
    mode = SCAN_INHIBIT;
}

/* ---- PASS (LK-1.14) ----------------------------------------------------- */
/* Pass guard: 2 of the last 3 pings closer sideways than pass_param ->
   EV_OBST_NEAR once.  lateral_cm() of the positive SCAN_PASS_DEG is
   positive for either side. */
LOCAL void pass_step (UW cm)
{
    BOOL near = (cm < 400U) && (lateral_cm(SCAN_PASS_DEG, cm) < (W) pass_param);
    UINT n;

    near_hist = ((near_hist << 1) | (near ? 1U : 0U)) & 7U;
    n = (near_hist & 1U) + ((near_hist >> 1) & 1U) + ((near_hist >> 2) & 1U);
    if (!near_sent && n >= 2U)
    {
        near_sent = TRUE;
        tm_printf((UB *) "[obst] pass guard: %d cm sideways (reading %d cm)\n",
                  (INT) lateral_cm(SCAN_PASS_DEG, cm), (INT) cm);
        (void) sup_post(SRC_OBSTACLE, EV_OBST_NEAR,
                        (UW) lateral_cm(SCAN_PASS_DEG, cm), cm);
        mode = SCAN_INHIBIT;
        servo_to(0);
    }
}

EXPORT void obstacle_task (INT stacd, void * exinf)
{
    UW req, next = 0;

    (void) stacd;
    (void) exinf;
    wcet_set_period(WCET_OBSTACLE, PERIOD_OBSTACLE_MS);

    for (;;)
    {
        SYSTIM now;
        TMO    wait;
        W      left;

        if (profile_due)
        {
            width_sweep(); /* blocking, ~6 s */
            continue;
        }
        if (side_due)
        {
            side_probe(); /* blocking, ~1.7 s */
            continue;
        }

        tk_get_otm(&now);
        left = (W) (next - now.lo);
        wait = (left > 0) ? (TMO) left : TMO_POL;

        if (tk_rcv_mbf(mbf_scan_req, &req, wait) == (INT) sizeof(req))
        {
            set_mode(SCAN_MODE_OF(req), SCAN_PARAM(req));
            continue; /* re-check the deadline */
        }

        tk_get_otm(&now);
        next = now.lo + PERIOD_OBSTACLE_MS; /* ms */

        if (mode == SCAN_GUARD || mode == SCAN_EMERG_ONLY ||
            mode == SCAN_PASS_LEFT || mode == SCAN_PASS_RIGHT)
        {
            UW t0 = wcet_release(WCET_OBSTACLE);
            UW cm = ping();

            if (mode == SCAN_GUARD || mode == SCAN_EMERG_ONLY)
            {
                judge(cm);
            }
            else
            {
                pass_step(cm);
            }
            wcet_end(WCET_OBSTACLE, t0);
        }
    }
}

/*** end of file ***/
