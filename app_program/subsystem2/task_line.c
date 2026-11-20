/*
 *      task_line.c
 *      Line Following task, 5 ms, core 1, priority 9.              Owner: Ryan
 *      Plan refs: RY-1.4, RY-1.8..RY-1.10, chain 4.2 steps 2 and 5.
 *
 *      Pin revision 2: left sensor analog (ADC0), right sensor digital (GP17).
 *      The error is three-level, from the two black / white decisions.  How
 *      they map depends on the sensor layout: see LINE_STRADDLE below (the
 *      car as built has the sensors just outside the line, both white when
 *      centred).
 *      Option for RY-1.8: the left sensor is analog, so it can give a
 *      proportional error on its own by following the line EDGE (aim for
 *      the ADC value half way between black and white), with the right
 *      sensor as a lost / junction detector.  The raw left value is in
 *      EV_LINE_SAMPLE for that.
 *
 *      Done:  reads, error word, line-lost after L_lost, LineSearch sample
 *             tick, calibrate handshake.
 *      TODO:  RY-1.4 calibration, RY-1.8 scale / edge following, RY-1.9
 * junctions.
 */

#include "subsystem2.h"
#include "drv_adc.h"
#include "../hw_alloc.h"
#include "../hal.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../wcet.h"

/* Sensor layout (RY-1.2).
 *   LINE_STRADDLE 1: the two sensors sit just OUTSIDE the line, one each
 *   side, about one line width apart (the car as built, 23 Oct).  Centred
 *   = both WHITE.  A sensor seeing black means the line has moved under
 *   it: steer towards that side.
 *     left only   -> +LINE_ERR_STEP       (steer left)
 *     right only  -> -LINE_ERR_STEP       (steer right)
 *     both white  -> half a step towards the side last seen.  This is
 *                    right whether the line went back to the centre (a
 *                    gentle correction) or past the sensor to the outside
 *                    (keeps turning back towards it).
 *     both black  -> 0: a marking across the line (junction, barcode)
 *   Line lost: both white for L_LOST_MM.  Normal centred driving is also
 *   both white, but the half-step keeps the car turning slowly, so on a
 *   real line one sensor meets the line again well within that distance.
 *   LINE_STRADDLE 0: the original layout, sensors closer than the line
 *   width, both BLACK when centred, both white = lost after 30 mm.       */
#ifndef LINE_STRADDLE
#define LINE_STRADDLE 1
#endif
#if LINE_STRADDLE
/* both white this far = line lost (TODO tune) */
#define L_LOST_MM 150
#else
#define L_LOST_MM 30
#endif
/* error for one sensor on the line (TODO tune) */
#define LINE_ERR_STEP 60

/* Smoother steering (RY-1.8), straddle layout only.  1 = on, 0 = the old
 * step steering (each sensor flips the error between -60, 0 and +60 at
 * once, so the car snaps from turning one way to the other: the weave
 * that pushed the barcode sensor on and off the bars, 30 Oct trace, and
 * jolted the IMU into false humps).
 *   - LEFT sensor, analog: the error grows with how far the line has come
 *     under it, 0 on white up to +LINE_ERR_STEP at the threshold (see
 *     "Arcs" below), so the car starts correcting as the line edge
 *     arrives, not all at once.
 *     The RIGHT sensor is digital (GP17) and stays a step.
 *   - Slew limit: the error moves at most LINE_ERR_SLEW per 5 ms towards
 *     its new value (0 -> 60 in 30 ms), so the wheel speeds ramp.
 *   - Both white (centred): the same pull towards the side last seen as
 *     before (LINE_ERR_BIAS = half a step), now reached through the ramp.
 *   - Both black (a cross): 0 at once, so the car crosses straight.
 * Placeholders: tune on the track.  "Line lost" on straights: raise
 * LINE_ERR_BIAS (never below 30).  Leaves the line in curves: raise
 * LINE_ERR_SLEW.  Too much weave: lower LINE_ERR_SLEW.                 */
#ifndef LINE_SMOOTH
#define LINE_SMOOTH 1
#endif
#define LINE_ERR_SLEW 10 /* max error change per 5 ms sample */
/* both white: pull towards the last side.
   30 Oct: 15 was too weak, the car went
   150 mm with both sensors white and
   reported line lost 6 times in one run.
   30 = the old half step. */
#define LINE_ERR_BIAS 30
/* Straight-line mode (31 Oct): while the two sensors touch the line in
   turn (left, right, left ...) the car is on a straight, and the
   corrections are made smaller so it weaves less (the barcode sensor,
   80 mm off-centre, then moves at a steadier speed).  The same sensor
   touching twice in a row = a curve: full corrections again at once. */
/* one sensor on the line, straight (60 in curves) */
#define LINE_STEP_STRAIGHT 35
/* both white on a straight: pull starts here ... */
#define LINE_BIAS_MIN 10
/* ... and reaches LINE_ERR_BIAS after this far */
#define LINE_BIAS_RAMP_MM 60
/* left sensor on white (cal. 118, idle 150-460) */
#define L_WHITE_EST 200
/* above white by this much = line edge arriving */
#define L_EDGE_MIN 150

/* Arcs (30 Oct: the car went wide on the arcs and lost the line).
 *   - Left sensor scale: 0 where the edge starts to show, the FULL step
 *     at the threshold (seen_l), like the digital right sensor.  It used
 *     to reach the full step only at the black estimate (~2700), so at
 *     the threshold it gave 30, no more than the both-white pull.
 *   - While a sensor is on the line, the error never drops below what
 *     the car was already steering towards that side when the touch
 *     began (the line arriving must not make the car turn less).
 *   - LINE_CURVE_TOUCH_MM: the first touch of an arc after a straight
 *     counts as "straight" (other sensor than last time).  One sensor on
 *     the line this far = the line runs away to that side = curve.
 *   - LINE_HOLD_MM: in a curve, when the inner sensor goes white again,
 *     the line has either come back to the centre or slipped past the
 *     sensor to the outside; both look the same.  Keep the full turn this
 *     far, then the normal pull.  The sensors are 130 mm ahead of the
 *     axle, so at the full turn (~95 mm radius) they swing sideways
 *     ~1.4 mm per mm driven: 15 mm sweeps back across the 24 mm gap.
 * Placeholders: tune on the track.  Still wide on arcs: raise
 * LINE_HOLD_MM.  Weaves after arcs (the outer sensor hits the line just
 * after the inner one leaves it): lower LINE_HOLD_MM.  0 = off.          */
#define LINE_CURVE_TOUCH_MM 20
#define LINE_HOLD_MM        15

/* Junction (RY-1.9).  The course has only cross (+) junctions.  Line
 * 18 mm, sensors 24 mm apart (each 3 mm outside the line), barcodes 20 mm
 * further left: nothing but the crossing line makes BOTH sensors black at
 * once.  A crossing at up to ~35 deg still overlaps both sensors
 * (24 mm x tan(a) < 18 mm).  One EV_JUNCTION per crossing: re-armed only
 * after both sensors have been off it for JCT_REARM_MM.                 */
#define JCT_MIN_SAMPLES 2 /* both black this many samples (10 ms) */
/* ... AND for this far.  30 Oct: one cross was reported twice, 30 cm
   apart; 10 ms alone is ~1 mm at 120 mm/s, so a paper edge, a finger or
   a barcode bar under the left sensor could pass.  The crossing line is
   18 mm wide, so 8 mm still leaves room for a crossing at an angle. */
#define JCT_MIN_MM   8
#define JCT_REARM_MM 40

LOCAL BOOL lost_reported;
LOCAL BOOL white_run;
LOCAL UH   white_since_mm;
LOCAL INT  last_err;
LOCAL INT  last_side;     /* +1 left, -1 right, 0 none yet */
LOCAL INT  hit_side;      /* sensor of the last touch: +1 left, -1 right */
LOCAL BOOL in_touch;      /* a sensor is on the line now */
LOCAL BOOL on_straight;   /* last two touches were by different sensors */
LOCAL UH   white_from_mm; /* odometer when both went white */
LOCAL UH   touch_from_mm; /* odometer when the current touch began */
LOCAL INT  touch_floor;   /* left touch: error at its start, if > 0 */
LOCAL UINT both_n;        /* consecutive both-black samples */
LOCAL BOOL jct_armed = TRUE;
LOCAL UH   jct_off_mm; /* where both-black last ended */
LOCAL UH   both_on_mm; /* where the current both-black began */

/* ---- IR calibration (RY-1.4) -----------------------------------------
 * Calibrate (MODE held 2 s, yellow LED): the car is stopped, and for
 * CAL_SAMPLES x 5 ms the user slides it slowly sideways across the line
 * and back, by hand, so every sensor sees both white floor and black
 * tape.  Each analog sensor's threshold = half way between the darkest
 * and the brightest reading.  GP17 (right sensor, digital) cannot be
 * calibrated: its black / white counts are only printed as a check.
 * Fails (and keeps the old thresholds) if the left sensor saw too little
 * contrast; the barcode threshold is updated only if it saw enough.     */
/* 15 s at 5 ms: same window as the IMU
   (1 s still, then turn + slide) */
#define CAL_SAMPLES 3000
#define CAL_MIN_GAP 400 /* ADC counts between white and black */

LOCAL BOOL cal_on;
LOCAL UINT cal_n;
LOCAL UH   cal_min_l, cal_max_l, cal_min_b, cal_max_b;
LOCAL UINT cal_black_r;

LOCAL void check_calibrate (UW l)
{
    T_RFLG rflg;
    UW     b;

    if (!cal_on)
    {
        if (tk_ref_flg(flg_calib, &rflg) == E_OK && (rflg.flgptn & CAL_REQ_IR))
        {
            (void) tk_clr_flg(flg_calib, ~CAL_REQ_IR); /* my bit only */
            cal_on    = TRUE;
            cal_n     = 0;
            cal_min_l = cal_min_b = 4095;
            cal_max_l = cal_max_b = 0;
            cal_black_r           = 0;
            tm_printf((UB *) "[line] calibrate: slide the car across the line "
                             "for %d s\n",
                      (INT) (CAL_SAMPLES * PERIOD_LINE_MS / 1000));
        }
        return;
    }

    if (sys_state_get() != SYS_CALIBRATE)
    {
        cal_on = FALSE; /* aborted with MODE: keep old values */
        tm_printf((UB *) "[line] calibrate aborted\n");
        return;
    }
    b = adc_latest(ADC_CH_BARCODE);
    if (l < cal_min_l)
    {
        cal_min_l = (UH) l;
    }
    if (l > cal_max_l)
    {
        cal_max_l = (UH) l;
    }
    if (b < cal_min_b)
    {
        cal_min_b = (UH) b;
    }
    if (b > cal_max_b)
    {
        cal_max_b = (UH) b;
    }
    if (hal_gpio_get(PIN_IR_LINE_R) == LINE_R_BLACK)
    {
        cal_black_r++;
    }

    if (++cal_n >= CAL_SAMPLES)
    {
        BOOL ok_l = (cal_max_l - cal_min_l) >= CAL_MIN_GAP;
        BOOL ok_b = (cal_max_b - cal_min_b) >= CAL_MIN_GAP;

        cal_on = FALSE;
        if (ok_l)
        {
            ir_cal.thr_l = (UH) (cal_min_l + (cal_max_l - cal_min_l) / 2);
        }
        if (ok_b)
        {
            ir_cal.thr_bc = (UH) (cal_min_b + (cal_max_b - cal_min_b) / 2);
        }
        ir_cal.calibrated = ok_l ? 1U : ir_cal.calibrated;
        tm_printf(
            (UB *) "[line] calibrate %s: left %d..%d -> thr %d%s, "
                   "barcode %d..%d -> thr %d%s, right (GP17) black %d of %d\n",
            ok_l ? "OK" : "FAILED", (INT) cal_min_l, (INT) cal_max_l,
            (INT) ir_cal.thr_l, ok_l ? "" : " (kept, too little contrast)",
            (INT) cal_min_b, (INT) cal_max_b, (INT) ir_cal.thr_bc,
            ok_b ? "" : " (kept, too little contrast)", (INT) cal_black_r,
            (INT) CAL_SAMPLES);
        (void) sup_post(SRC_LINE, EV_CALIB_DONE, CAL_A(CAL_WHO_IR, ok_l), 0);
    }
}

EXPORT void line_task (INT stacd, void * exinf)
{
    NAV_STATE ns;

    (void) stacd;
    (void) exinf;
    LOG("line", "task on core %d\n", (INT) hal_core_id());
    wcet_set_period(WCET_LINE, PERIOD_LINE_MS);

    for (;;)
    {
        UW        t0, l;
        BOOL      seen_l, seen_r;
        INT       err;
        UINT      flags;
        SYS_STATE s;

        tk_slp_tsk(TMO_FEVR);
        t0 = wcet_release(WCET_LINE);

        l = adc_latest(ADC_CH_LINE_L); /* sampled by the barcode task */
        check_calibrate(l);

        /* Sensor check for setting the height / calibrating: in Idle
           only (the car is stopped, so printf jitter does not matter),
           once a second. */
        {
            LOCAL UINT idle_n;
            if (sys_state_get() == SYS_IDLE)
            {
                if (++idle_n >= (1000 / PERIOD_LINE_MS))
                {
                    idle_n = 0;
                    /* "thr" is the LEFT sensor's threshold; the
                       barcode has its own ("bc thr"). */
                    tm_printf(
                        (UB *) "[line] idle: left %d (%s), right GP17 %d (%s), "
                               "barcode %d, thr %d, bc thr %d\n",
                        (INT) l, (l > ir_cal.thr_l) ? "black" : "white",
                        (INT) hal_gpio_get(PIN_IR_LINE_R),
                        (hal_gpio_get(PIN_IR_LINE_R) == LINE_R_BLACK) ? "black"
                                                                      : "white",
                        (INT) adc_latest(ADC_CH_BARCODE), (INT) ir_cal.thr_l,
                        (INT) ir_cal.thr_bc);
                }
            }
            else
            {
                idle_n = 0;
            }
        }
        seen_l = (l > ir_cal.thr_l);
        seen_r = (hal_gpio_get(PIN_IR_LINE_R) == LINE_R_BLACK);

#if LINE_STRADDLE && LINE_SMOOTH
        {
            /* how far the line is under the LEFT sensor, 0..STEP:
               0 where the edge starts to show, the full step from
               the threshold up (see "Arcs" above). */
            W   edge0 = L_WHITE_EST + L_EDGE_MIN;
            W   span  = (W) ir_cal.thr_l - edge0;
            W   f     = 0, target, d;
            H   gone;
            INT side_now, step, bias;

            if (seen_l)
            {
                f = LINE_ERR_STEP;
            }
            else if (span > 0 && (W) l > edge0)
            {
                f = (((W) l - edge0) * LINE_ERR_STEP) / span;
                if (f > LINE_ERR_STEP)
                {
                    f = LINE_ERR_STEP;
                }
            }

            nav_read(&ns);

            /* Straight or curve?  A new touch of the line by the
               OTHER sensor than last time = alternating = straight;
               the SAME sensor again = the line runs away to that side
               = curve. */
            side_now =
                (seen_l && seen_r) ? 0 : (seen_r ? -1 : ((f > 0) ? +1 : 0));
            if (side_now != 0 && !in_touch)
            {
                on_straight   = (hit_side != 0 && side_now != hit_side);
                hit_side      = side_now;
                in_touch      = TRUE;
                touch_from_mm = ns.m.odo_mm;
                /* left touch: keep at least the left turn the car
                   already has (the both-white pull or the hold) */
                touch_floor = (side_now > 0 && last_err > 0) ? last_err : 0;
            }
            else if (side_now == 0 && in_touch)
            {
                in_touch      = FALSE;
                white_from_mm = ns.m.odo_mm; /* both white from here */
            }
            /* an arc after a straight: one sensor on the line for
               LINE_CURVE_TOUCH_MM = curve, full corrections now */
            if (in_touch && on_straight && side_now == hit_side &&
                (H) (ns.m.odo_mm - touch_from_mm) >= LINE_CURVE_TOUCH_MM)
            {
                on_straight = FALSE;
            }
            step = on_straight ? LINE_STEP_STRAIGHT : LINE_ERR_STEP;

            if (seen_l && seen_r)
            {
                err    = 0; /* marking across: straight, now */
                target = 0;
            }
            else if (seen_r)
            {
                target    = -step;
                last_side = -1;
            }
            else if (f > 0)
            {
                target = (f * step) / LINE_ERR_STEP; /* left edge */
                if (target < touch_floor)
                {
                    target = touch_floor;
                }
                last_side = +1;
            }
            else
            {
                /* both white.  Curve: the full turn for LINE_HOLD_MM
                   (the line may have slipped past the inner sensor),
                   then the full pull.  Straight: start small and grow
                   with distance, full after LINE_BIAS_RAMP_MM, so a
                   line that really goes away is still found long
                   before L_LOST_MM. */
                gone = (H) (ns.m.odo_mm - white_from_mm);
                if (gone < 0)
                {
                    gone = 0;
                }
                if (!on_straight)
                {
                    bias =
                        (gone < LINE_HOLD_MM) ? LINE_ERR_STEP : LINE_ERR_BIAS;
                }
                else
                {
                    bias = (gone >= LINE_BIAS_RAMP_MM)
                               ? LINE_ERR_BIAS
                               : LINE_BIAS_MIN +
                                     ((LINE_ERR_BIAS - LINE_BIAS_MIN) * gone) /
                                         LINE_BIAS_RAMP_MM;
                }
                target = last_side * bias;
            }
            if (!(seen_l && seen_r))
            {
                d = target - last_err;
                if (d > LINE_ERR_SLEW)
                {
                    d = LINE_ERR_SLEW;
                }
                if (d < -LINE_ERR_SLEW)
                {
                    d = -LINE_ERR_SLEW;
                }
                err = (INT) (last_err + d);
            }
        }
#elif LINE_STRADDLE
        if (seen_l && !seen_r)
        {
            err       = +LINE_ERR_STEP;
            last_side = +1;
        }
        else if (seen_r && !seen_l)
        {
            err       = -LINE_ERR_STEP;
            last_side = -1;
        }
        else if (seen_l && seen_r)
        {
            err = 0; /* marking across the line */
        }
        else
        {
            err = last_side * (LINE_ERR_STEP / 2);
        }
#else
        if (seen_l && !seen_r)
        {
            err = +LINE_ERR_STEP;
        }
        else if (seen_r && !seen_l)
        {
            err = -LINE_ERR_STEP;
        }
        else if (seen_l && seen_r)
        {
            err = 0;
        }
        else
        {
            err = last_err; /* lost: hold */
        }
#endif
        last_err = err;
        flags =
            (seen_l ? LE_SEEN_L : 0U) | (seen_r ? LE_SEEN_R : 0U) | LE_VALID;
#if LINE_STRADDLE && LINE_SMOOTH
        /* Tell Motion Control to slow down in arcs (LE_CURVE).  Only
           this branch keeps on_straight up to date; the others leave
           the flag clear, so they keep the old full-speed behaviour. */
        if (!on_straight)
        {
            flags |= LE_CURVE;
        }
#endif
        line_err_publish(err, flags);

        nav_read(&ns);
        s = sys_state_get();

        /* Line lost: both white for L_lost (RY-1.10).  Only counted
           while following: a white run that began in Idle (the car
           pushed by hand, odometer moved backwards) must not carry
           over into the run.  The distance is signed: rolling back a
           few mm is not 65 m forward. */
        if (!seen_l && !seen_r &&
            (s == SYS_LINE_FOLLOW || s == SYS_HUMP_TRAVERSE))
        {
            if (!white_run)
            {
                white_run      = TRUE;
                white_since_mm = ns.m.odo_mm;
            }
            else if (!lost_reported &&
                     (H) (ns.m.odo_mm - white_since_mm) >= L_LOST_MM)
            {
                (void) sup_post(SRC_LINE, EV_LINE_LOST, ns.m.odo_mm, 0);
                lost_reported = TRUE;
            }
        }
        else
        {
            white_run     = FALSE;
            lost_reported = FALSE;
        }

        /* LineSearch tick: raw left ADC, right as 0 / 4095 */
        if (s == SYS_LINE_SEARCH)
        {
            (void) sup_post(SRC_LINE, EV_LINE_SAMPLE,
                            LS_A(l, seen_r ? 4095U : 0U, seen_l, seen_r),
                            ns.m.odo_mm);
        }

        /* Junction (RY-1.9): both black = the crossing line.  Only in
           LineFollow (not on a hump, not while turning at a junction). */
        if (seen_l && seen_r)
        {
            if (both_n++ == 0)
            {
                both_on_mm = ns.m.odo_mm;
            }
            if (both_n >= JCT_MIN_SAMPLES && jct_armed &&
                s == SYS_LINE_FOLLOW &&
                (H) (ns.m.odo_mm - both_on_mm) >= JCT_MIN_MM)
            {
                tm_printf((UB *) "[line] cross at odo %d mm (left %d, both "
                                 "black %d mm)\n",
                          (INT) ns.m.odo_mm, (INT) l,
                          (INT) (H) (ns.m.odo_mm - both_on_mm));
                (void) sup_post(SRC_LINE, EV_JUNCTION, JCT_CROSS, ns.m.odo_mm);
                jct_armed = FALSE;
            }
            jct_off_mm = ns.m.odo_mm;
        }
        else
        {
            if (jct_armed && both_n >= JCT_MIN_SAMPLES && s == SYS_LINE_FOLLOW)
            {
                tm_printf((UB *) "[line] both black for only %d mm at odo %d: "
                                 "not a cross\n",
                          (INT) (H) (ns.m.odo_mm - both_on_mm),
                          (INT) ns.m.odo_mm);
            }
            both_n = 0;
            if (!jct_armed && (H) (ns.m.odo_mm - jct_off_mm) >= JCT_REARM_MM)
            {
                jct_armed = TRUE;
            }
        }
        if (!SYS_IS_RUNNING(s))
        {
            jct_armed = TRUE; /* fresh run */
        }

        wcet_end(WCET_LINE, t0);
    }
}

/*** end of file ***/
