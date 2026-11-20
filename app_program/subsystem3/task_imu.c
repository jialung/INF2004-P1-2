/*
 *      task_imu.c
 *      IMU task, 20 ms, core 0, priority 12.                   Owner: Jian Yi
 *      Plan refs: JY-1.5..JY-1.13, chain 4.4, Design Review 6.3.
 *
 *      Every 20 ms: read the GY-511, map the sensor axes to the car frame
 *      (IMU_AXIS_* in subsystem3.h), then
 *        roll / pitch   from the accelerometer, the car's own forward
 *                       acceleration (Motion Control, nav_state) subtracted
 *                       first so speeding up is not read as a nose-up tilt,
 *                       minus the level offsets from calibration (JY-1.6/1.7)
 *        heading        tilt-compensated compass, minus the hard-iron
 *                       offsets from calibration (JY-1.8)
 *        hump           |roll| statechart: Level -> Climbing -> Level, peak
 *                       -> height (FR7, FR8).  One wheel on the hump = roll.
 *        impact         horizontal acceleration above IMPACT_MG twice in a
 *                       row -> IMPACT fault (JY-1.12)
 *        pickup         while running: tilt past PICKUP_TILT_Q, or |a| above
 *                       PICKUP_LIFT_MG off a hump -> PICKUP fault, motors cut
 *        sensor fault   IMU_FAIL_LIMIT failed reads in a row -> IMU_SENSOR
 *
 *      Calibrate (MODE held 2 s, together with the IR sensors), JY-1.13:
 *        first IMU_CAL_LEVEL_MS: keep the car flat and still -> level zero;
 *        then until IMU_CAL_MS: turn the car slowly through a full circle
 *        (and slide it across the line for the IR sensors) -> compass
 *        hard-iron offsets = centre of the min / max box.  The compass part
 *        is kept only if the field moved enough on both X and Y.
 */

#include "subsystem3.h"
#include "drv_imu.h"
#include "imu_math.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../wcet.h"

LOCAL NAV_IMU navi;
/* Idle right-wheel readout: 2 s mean / range, 0.1 mm */
LOCAL W   idle_sum;
LOCAL INT idle_lo, idle_hi, idle_n;
LOCAL INT fails;
LOCAL INT run_n; /* consecutive samples past the threshold */
LOCAL INT peak_q;
LOCAL INT impact_n;
/* no impact / lift check right after a hump */
LOCAL UW  impact_hold_until;
LOCAL INT pickup_tilt_n; /* consecutive samples past PICKUP_TILT_Q */
LOCAL INT pickup_lift_n; /* consecutive samples past PICKUP_LIFT_MG */

/* Offsets, replaced by each calibration.  Power-up values from the
   calibration on 30 Oct (evening):
     "[imu] calibrate done: level roll -3 pitch -4 (0.25 deg);
      compass OK (span f 274, l 694; offsets 645 -40 -119)"
   The compass offsets belong to THIS car as built: moving the battery,
   motors or wires near the GY-511 changes them; recalibrate then.  The
   turn check never corrects more than COMPASS_MAX_FIX_DEG (subsystem5.h). */
/* 6 Nov: axes corrected (subsystem3.h), so the 30 Oct values are moved to
   the new car frame: the old "pitch" was the real roll and the other way
   round, and the field's old (f, l) is the new (l, -f).
   lat0_q14 -286 = sin(-1.0 deg) x 16384, the same level as roll0 -4.
   RECALIBRATE after flashing: these are only a starting point. */
EXPORT volatile IMU_CAL imu_cal = {-4, -3, 40, 645, -119, 1, -286};
EXPORT volatile UB      imu_peak_mm; /* highest hump so far (FR8) */

/* calibration in progress */
LOCAL BOOL    cal_on;
LOCAL UW      cal_t0;
LOCAL W       cal_roll_sum, cal_pitch_sum, cal_lat_sum;
LOCAL INT     cal_n;
LOCAL IMU_VEC cal_min, cal_max;

/* Smoothed roll for hump detection, 0.25 deg x 16 (fixed point).  Motor
   vibration and steering wobble add short roll spikes that are not a
   hump; a first-order low-pass removes most of them (HUMP_FILTER_SHIFT). */
LOCAL W roll_f16;

LOCAL INT tilt_q (const NAV_IMU * n)
{
    INT r;

    roll_f16 += (((W) n->roll_q << 4) - roll_f16) >> HUMP_FILTER_SHIFT;
    r = (INT) (roll_f16 >> 4);
    return (r < 0) ? -r : r; /* |roll|: hump under one wheel (FR7) */
}

/* ---- Hump height: h = W x (s_ref - s), subsystem3.h ------------------ */
LOCAL UH   hump_odo0; /* odometer when the hump started */
LOCAL UH   over_odo;  /* odometer when h first went over the trigger */
LOCAL W    ref_acc;   /* s_ref << HUMP_REF_SHIFT, Q14 */
LOCAL INT  ref_hold;  /* samples s_ref has been frozen */
LOCAL INT  med_buf[HUMP_MED_N];
LOCAL UINT med_i, med_n;
LOCAL INT  peak_x10; /* highest single h, 0.1 mm */
/* Height = the highest mean of HUMP_AVG_N consecutive on-hump samples
   (7 Nov): the maximum of a moving average.  Why this and not a median
   or a peak: see subsystem3.h step 6.  A ring buffer plus a running sum,
   so each sample costs one add, one subtract and one divide. */
LOCAL INT  avg_buf[HUMP_AVG_N];
LOCAL UINT avg_i, avg_n;
LOCAL W    avg_sum;      /* sum of the live window, 0.1 mm */
LOCAL INT  avg_peak_x10; /* highest window mean, 0.1 mm */
LOCAL UINT avg_wins;     /* full windows seen on this hump */
/* The 6 Nov method, kept only so ONE run on the track can compare the
   two: the median of every on-hump sample, histogram in 0.5 mm bins over
   0..40 mm.  It read 7.7 mm for the 10 mm hump (the ramps and the exit
   swing are in with the top, and hist_add() clamps the negative swing
   into bin 0).  Printed as "old method" in the "hump end" line; delete
   this and hist_*() once the moving average is confirmed on the track. */
#define HUMP_HIST_BINS     80
#define HUMP_HIST_STEP_X10 5
LOCAL UH      hump_hist[HUMP_HIST_BINS];
LOCAL UINT    hump_hist_n;
LOCAL BOOL    ref_ready; /* ref_acc has been set once */
LOCAL H       prev_turn_q;
LOCAL BOOL    have_turn;
LOCAL INT     lat_now; /* s of the last sample, Q14 */
LOCAL IMU_RAW raw_now; /* last good read, for the Idle readout */
LOCAL IMU_VEC acc_now; /* the same sample in the car frame, mg */
/* |a| of that sample, mg (1000 = 1 g at rest) */
LOCAL UW acc_mag;

LOCAL void hist_clear (void)
{
    UINT i;
    for (i = 0; i < HUMP_HIST_BINS; i++)
    {
        hump_hist[i] = 0;
    }
    hump_hist_n = 0;
}

LOCAL void hist_add (INT h)
{
    INT b = h / HUMP_HIST_STEP_X10;
    if (b < 0)
    {
        b = 0;
    }
    if (b >= HUMP_HIST_BINS)
    {
        b = HUMP_HIST_BINS - 1;
    }
    if (hump_hist[b] < 0xFFFFU)
    {
        hump_hist[b]++;
    }
    hump_hist_n++;
}

/* median of the hump samples, 0.1 mm (centre of the median bin) */
LOCAL INT hist_median (void)
{
    UINT i, c = 0, half = (hump_hist_n + 1U) / 2U;
    for (i = 0; i < HUMP_HIST_BINS; i++)
    {
        c += hump_hist[i];
        if (c >= half)
        {
            break;
        }
    }
    if (i >= HUMP_HIST_BINS)
    {
        i = HUMP_HIST_BINS - 1;
    }
    return (INT) (i * HUMP_HIST_STEP_X10 + HUMP_HIST_STEP_X10 / 2);
}

LOCAL void avg_clear (void)
{
    avg_i        = 0;
    avg_n        = 0;
    avg_sum      = 0;
    avg_peak_x10 = 0;
    avg_wins     = 0;
}

/* One on-hump sample into the moving average; keeps the highest mean.
   Only a FULL window counts: a partly filled one is a shorter average
   sitting on the entry ramp, which would bias the maximum downwards
   early and is not comparable with the rest. */
LOCAL void avg_add (INT h)
{
    if (avg_n >= HUMP_AVG_N)
    {
        avg_sum -= avg_buf[avg_i]; /* drop the oldest */
    }
    else
    {
        avg_n++;
    }
    avg_buf[avg_i] = h;
    avg_sum += h;
    avg_i = (avg_i + 1U) % HUMP_AVG_N;

    if (avg_n >= HUMP_AVG_N)
    {
        INT mean = (INT) (avg_sum / (W) HUMP_AVG_N);

        if (avg_wins == 0U || mean > avg_peak_x10)
        {
            avg_peak_x10 = mean;
        }
        avg_wins++;
    }
}

/* Not following the line (Idle, pivots, calibration ...).  6 Nov: the
   reference used to jump back to the CALIBRATED level here, and the
   floor where the car stood read ~2.7 mm off it, so right after START
   that offset plus the start-up jolt passed the trigger (a false 5 mm
   hump in the first 14 mm).  Now, while the car stands still, the
   reference follows the floor it stands on; while it moves (pivots) it
   is kept.  The calibrated level is only the very first value. */
LOCAL void hump_reset (INT s, H v_mm_s)
{
    if (!ref_ready)
    {
        ref_acc   = (W) imu_cal.lat0_q14 << HUMP_REF_SHIFT;
        ref_ready = TRUE;
    }
    if (v_mm_s <= 3 && v_mm_s >= -3)
    {
        ref_acc += (W) s - (ref_acc >> HUMP_REF_SHIFT);
    }
    ref_hold = 0;
    med_n    = 0;
    med_i    = 0;
    run_n    = 0;
    if (navi.terrain == TER_CLIMBING)
    {
        navi.terrain = TER_LEVEL;
    }
}

/* tilt as wheel height above the level, 0.1 mm, SIGNED:
   + = right wheel up (left side down), - = left wheel up */
LOCAL INT h_x10_of (INT s, INT ref)
{
    return (INT) (((W) HUMP_LEVER_MM * 10 * (ref - s)) / 16384);
}

LOCAL INT hump_side; /* +1 right wheel, -1 left wheel */
#define SIDE_NAME(sd) (((sd) > 0) ? "right" : "left")

LOCAL INT median (INT v)
{
    INT  t[HUMP_MED_N], x;
    UINT i, j, n;

    med_buf[med_i] = v;
    med_i          = (med_i + 1U) % HUMP_MED_N;
    if (med_n < HUMP_MED_N)
    {
        med_n++;
    }
    n = med_n;
    for (i = 0; i < n; i++)
    {
        x = med_buf[i];
        for (j = i; j > 0 && t[j - 1] > x; j--)
        {
            t[j] = t[j - 1];
        }
        t[j] = x;
    }
    return t[n / 2];
}

/* s = a_l / |a|, Q14, with the car's own sideways acceleration removed */
LOCAL INT lateral_q14 (IMU_VEC a, const NAV_STATE * ns)
{
    UW mag;

#if HUMP_STEER_COMP
    /* v x omega: omega from the encoder heading (turn_q, 0.25 deg, + =
       left) since the last 20 ms sample.  A left turn pushes the car
       (and the sensor) to the left: a_l reads + */
    if (have_turn)
    {
        H         dq  = (H) (ns->m.turn_q - prev_turn_q);
        long long amm = ((long long) ns->m.v_mm_s * dq * 355 * 1000) /
                        (113LL * 720 * PERIOD_IMU_MS); /* mm/s^2 */
        a.l -= (INT) ((amm * 1000) / 9807);            /* mg */
    }
#endif
    prev_turn_q = ns->m.turn_q;
    have_turn   = TRUE;
    mag = imu_isqrt((uint64_t) ((long long) a.f * a.f + (long long) a.l * a.l +
                                (long long) a.u * a.u));
    acc_now = a; /* steering-corrected, car frame (Idle readout) */
    acc_mag = mag;
    if (mag < 250U)
    {
        return lat_now; /* < 0.25 g: lifted / bad sample */
    }
    return (INT) (((long long) a.l * 16384) / (W) mag);
}

LOCAL void hump_step (INT s, UH odo_mm)
{
    INT ref = (INT) (ref_acc >> HUMP_REF_SHIFT);
    INT hs  = median(h_x10_of(s, ref)); /* signed: + right, - left */
    INT h   = HUMP_RIGHT_ONLY ? hs : ((hs < 0) ? -hs : hs);
    INT q   = tilt_q(&navi); /* smoothed |roll|, log only */

#if HUMP_DEBUG
    /* once a second while following: what the hump code sees */
    {
        LOCAL INT dbg_n, dbg_max = -32767, dbg_min = 32767;
        LOCAL UW  dbg_amax; /* highest |a|, mg: tunes PICKUP_LIFT_MG */
        /* vs calibration */
        INT hc = h_x10_of(lat_now, imu_cal.lat0_q14);

        /* signed: + = right wheel up, - = left wheel up */
        if (hs > dbg_max)
        {
            dbg_max = hs;
        }
        if (hs < dbg_min)
        {
            dbg_min = hs;
        }
        if (acc_mag > dbg_amax)
        {
            dbg_amax = acc_mag;
        }
        if (++dbg_n >= (1000 / PERIOD_IMU_MS))
        {
            tm_printf(
                (UB *) "[imu] hump watch: right wheel now %d, min %d max %d "
                       "(0.1 mm, last 1 s; trigger %d), vs calibration %d, %s; "
                       "accel raw x %d y %d z %d; max |a| %d mg (lift trip "
                       "%d)\n",
                hs, dbg_min, dbg_max, (INT) HUMP_ENTER_X10, hc,
                (navi.terrain == TER_CLIMBING) ? "ON HUMP" : "level",
                (INT) raw_now.ax, (INT) raw_now.ay, (INT) raw_now.az,
                (INT) dbg_amax, (INT) PICKUP_LIFT_MG);
            dbg_n    = 0;
            dbg_max  = -32767;
            dbg_min  = 32767;
            dbg_amax = 0;
        }
    }
#endif

    switch ((TERRAIN_STATE) navi.terrain)
    {
        case TER_LEVEL:
            /* s_ref follows the floor while it is level; frozen while the
               wheel goes up, unless that lasts so long it is a slope */
            /* 5 Nov fix: once frozen for HUMP_REF_HOLD_N it is a slope /
               a calibration that does not fit this floor: follow it on
               EVERY sample until it is level again (it used to update once
               per HUMP_REF_HOLD_N samples, i.e. minutes to catch up, and a
               level that is off by a few mm hides the 10 mm hump). */
            if (h <= HUMP_LEVEL_X10 && h >= -HUMP_LEVEL_X10)
            {
                ref_acc += (W) s - (ref_acc >> HUMP_REF_SHIFT);
                ref_hold = 0;
            }
            else if (ref_hold >= HUMP_REF_HOLD_N)
            {
                ref_acc += (W) s - (ref_acc >> HUMP_REF_SHIFT);
            }
            else
            {
                ref_hold++;
            }
            if (h > HUMP_ENTER_X10)
            {
                if (run_n++ == 0)
                {
                    over_odo = odo_mm; /* first sample above */
                }
            }
            else
            {
                run_n = 0;
            }
            if (run_n >= HUMP_ENTER_N &&
                (H) (odo_mm - over_odo) >= HUMP_ENTER_TRAVEL_MM)
            {
                navi.terrain = TER_CLIMBING;
                run_n        = 0;
                peak_q       = q;
                peak_x10     = h;
                avg_clear();
                avg_add(h);
                hist_clear();
                hist_add(h);
                hump_odo0 = odo_mm;
                hump_side = (hs >= 0) ? +1 : -1;
                tm_printf((UB *) "[imu] hump start: %s wheel up %d.%d mm "
                                 "(level %d, now %d, Q14)\n",
                          SIDE_NAME(hump_side), h / 10, h % 10, ref, s);
                (void) sup_post(SRC_IMU, EV_HUMP_CLIMB, (UW) (W) navi.roll_q,
                                0);
            }
            break;
        case TER_CLIMBING:
        {
            H    went     = (H) (odo_mm - hump_odo0); /* signed, wrap-safe */
            BOOL too_long = (went > HUMP_MAX_MM || went < -HUMP_MAX_MM);

            if (q > peak_q)
            {
                peak_q = q;
            }
            if (h > peak_x10)
            {
                peak_x10 = h;
            }
            /* registered once back at the calibrated / pre-hump level */
            run_n =
                (h <= HUMP_LEVEL_X10 && h >= -HUMP_LEVEL_X10) ? run_n + 1 : 0;
            if (run_n == 0)
            { /* the exit samples are not the top */
                avg_add(h);
                hist_add(h);
            }
            if (run_n >= HUMP_EXIT_N || too_long)
            {
                /* Height = the highest window mean (subsystem3.h step 6).
                   Two ways that fails, both on a feature far SHORTER
                   than the window, where every window straddles the
                   ramps and the exit swing instead of sitting on the
                   top.  Then fall back to the single peak and say so:
                     a) not one full window fitted;
                     b) the best mean is nowhere near the peak.  On a
                        real hump the two are within ~10 % of each
                        other, so half the peak is a wide margin; a
                        4-sample top read -0.2 mm without this. */
                BOOL shortie = (avg_wins == 0U || avg_peak_x10 < peak_x10 / 2);
                INT  top     = shortie ? peak_x10 : avg_peak_x10; /* 0.1 mm */
                INT  med     = hist_median(); /* old method */
                UINT mm;

                if (top < 0)
                {
                    top = 0; /* keeps the log unsigned */
                }
                mm = (UINT) ((top + 5) / 10);
                if (mm > 31U)
                {
                    mm = 31U; /* 5 bits (JY-1.11) */
                }
                navi.terrain = TER_LEVEL;
                run_n        = 0;
                ref_hold     = 0;
                tm_printf(
                    (UB *) "[imu] hump end: %s wheel, flat top %d.%d mm "
                           "(%d samples, highest %d.%d) -> %d mm, %d mm long "
                           "[max of %d-sample means, %d windows%s; "
                           "old median %d.%d]%s\n",
                    SIDE_NAME(hump_side), top / 10, top % 10, (INT) hump_hist_n,
                    peak_x10 / 10, peak_x10 % 10, (INT) mm, (INT) went,
                    (INT) HUMP_AVG_N, (INT) avg_wins,
                    shortie ? ", TOO SHORT: single peak used" : "", med / 10,
                    med % 10,
                    too_long ? " (never back at the level: recalibrate, or "
                               "raise HUMP_LEVEL_X10)"
                             : ", back at the calibrated level");
                if (mm > imu_peak_mm)
                {
                    imu_peak_mm = (UB) mm;
                }
                (void) sup_post(SRC_IMU, EV_HUMP_LEVEL, HUMP_A(mm, peak_q), 0);
                impact_hold_until = now_us() + IMPACT_HOLDOFF_MS * 1000U;
            }
            break;
        }
        default:
            break;
    }
}

/* sensor axes -> car frame (subsystem3.h) */
LOCAL INT pick (INT axis, INT sign, INT x, INT y, INT z)
{
    INT v = (axis == 0) ? x : ((axis == 1) ? y : z);
    return sign * v;
}

LOCAL void to_car (const IMU_RAW * r, IMU_VEC * a, IMU_VEC * m)
{
    /* magnetometer Z gain differs: scale Z to the X/Y gain */
    INT mz = (r->mz * IMU_MAG_XY_PER_GAUSS) / IMU_MAG_Z_PER_GAUSS;

    a->f = pick(IMU_AXIS_FWD, IMU_SIGN_FWD, r->ax, r->ay, r->az) *
           IMU_ACC_MG_PER_LSB;
    a->l = pick(IMU_AXIS_LEFT, IMU_SIGN_LEFT * IMU_ACC_SIGN_LEFT, r->ax, r->ay,
                r->az) *
           IMU_ACC_MG_PER_LSB;
    a->u = pick(IMU_AXIS_UP, IMU_SIGN_UP, r->ax, r->ay, r->az) *
           IMU_ACC_MG_PER_LSB;
    m->f = pick(IMU_AXIS_FWD, IMU_SIGN_FWD, r->mx, r->my, mz);
    m->l = pick(IMU_AXIS_LEFT, IMU_SIGN_LEFT, r->mx, r->my, mz);
    m->u = pick(IMU_AXIS_UP, IMU_SIGN_UP, r->mx, r->my, mz);
}

LOCAL B sat8 (INT q)
{
    return (B) ((q > 127) ? 127 : ((q < -127) ? -127 : q));
}

LOCAL void calib_start (void)
{
    cal_on       = TRUE;
    cal_t0       = now_us();
    cal_roll_sum = cal_pitch_sum = cal_lat_sum = 0;
    cal_n                                      = 0;
    cal_min.f = cal_min.l = cal_min.u = 0x7FFFFFFF;
    cal_max.f = cal_max.l = cal_max.u = -0x7FFFFFFF;
    tm_printf((UB *) "[imu] calibrate: keep the car flat and still for %d s, "
                     "then turn it "
                     "slowly through a full circle\n",
              (INT) (IMU_CAL_LEVEL_MS / 1000));
}

/* one sample during calibration; raw roll/pitch, lateral s (Q14), raw
   (car-frame) field */
LOCAL void calib_step (INT roll, INT pitch, INT lat, const IMU_VEC * m)
{
    UW t = (now_us() - cal_t0) / 1000U;

    if (t < IMU_CAL_LEVEL_MS)
    {
        cal_roll_sum += roll;
        cal_pitch_sum += pitch;
        cal_lat_sum += lat;
        cal_n++;
    }
    if (m->f < cal_min.f)
    {
        cal_min.f = m->f;
    }
    if (m->l < cal_min.l)
    {
        cal_min.l = m->l;
    }
    if (m->u < cal_min.u)
    {
        cal_min.u = m->u;
    }
    if (m->f > cal_max.f)
    {
        cal_max.f = m->f;
    }
    if (m->l > cal_max.l)
    {
        cal_max.l = m->l;
    }
    if (m->u > cal_max.u)
    {
        cal_max.u = m->u;
    }

    if (t >= IMU_CAL_MS)
    {
        BOOL mag_ok = (cal_max.f - cal_min.f) >= IMU_CAL_MAG_SPAN &&
                      (cal_max.l - cal_min.l) >= IMU_CAL_MAG_SPAN;
        cal_on = FALSE;
        if (cal_n > 0)
        {
            imu_cal.roll0_q  = (H) (cal_roll_sum / cal_n);
            imu_cal.pitch0_q = (H) (cal_pitch_sum / cal_n);
            imu_cal.lat0_q14 = (H) (cal_lat_sum / cal_n); /* hump level */
        }
        if (mag_ok)
        {
            imu_cal.mag_off_f = (H) ((cal_max.f + cal_min.f) / 2);
            imu_cal.mag_off_l = (H) ((cal_max.l + cal_min.l) / 2);
            imu_cal.mag_off_u = (H) ((cal_max.u + cal_min.u) / 2);
            imu_cal.mag_ok    = 1;
        }
        tm_printf(
            (UB *) "[imu] calibrate done: level roll %d pitch %d (0.25 deg), "
                   "lateral %d (Q14); "
                   "compass %s (span f %d, l %d; offsets %d %d %d)\n",
            (INT) imu_cal.roll0_q, (INT) imu_cal.pitch0_q,
            (INT) imu_cal.lat0_q14,
            mag_ok ? "OK" : "KEPT OLD - turn the car through a full circle",
            (INT) (cal_max.f - cal_min.f), (INT) (cal_max.l - cal_min.l),
            (INT) imu_cal.mag_off_f, (INT) imu_cal.mag_off_l,
            (INT) imu_cal.mag_off_u);
        (void) sup_post(SRC_IMU, EV_CALIB_DONE, CAL_A(CAL_WHO_IMU, 1), 0);
    }
}

LOCAL void check_calibrate (void)
{
    T_RFLG rflg;

    if (tk_ref_flg(flg_calib, &rflg) == E_OK && (rflg.flgptn & CAL_REQ_IMU))
    {
        (void) tk_clr_flg(flg_calib, ~CAL_REQ_IMU); /* my bit only */
        if (imu_driver_status == DRV_IMU_OK)
        {
            calib_start();
        }
        else
        {
            /* no IMU: nothing to calibrate, let the IR part finish */
            (void) sup_post(SRC_IMU, EV_CALIB_DONE, CAL_A(CAL_WHO_IMU, 1), 0);
        }
    }
    if (cal_on && sys_state_get() != SYS_CALIBRATE)
    {
        cal_on = FALSE; /* aborted with MODE */
        tm_printf((UB *) "[imu] calibrate aborted\n");
    }
}

/* Pickup (subsystem3.h PICKUP_*): the car lifted while running.  Uses
   this sample's calibrated roll / pitch (navi) and |a| (acc_mag, set by
   lateral_q14), so it runs after both.  The fault latches: put the car
   down and press MODE to clear it, as for the other trip faults. */
LOCAL void pickup_check (void)
{
    INT  r       = (navi.roll_q < 0) ? -navi.roll_q : navi.roll_q;
    INT  p       = (navi.pitch_q < 0) ? -navi.pitch_q : navi.pitch_q;
    BOOL running = !cal_on && SYS_IS_RUNNING(sys_state_get());
    BOOL hump_quiet =
        navi.terrain != TER_CLIMBING && (W) (now_us() - impact_hold_until) >= 0;

    if (!running || (fault_peek() & FAULT_PICKUP))
    {
        pickup_tilt_n = 0;
        pickup_lift_n = 0;
        return;
    }

    /* only a sample near 1 g is a tilt; any other is motion (subsystem3.h) */
    pickup_tilt_n =
        ((r > PICKUP_TILT_Q || p > PICKUP_TILT_Q) &&
         acc_mag >= (UW) PICKUP_TILT_G_LO && acc_mag <= (UW) PICKUP_TILT_G_HI)
            ? pickup_tilt_n + 1
            : 0;
    pickup_lift_n =
        (PICKUP_LIFT_MG > 0 && hump_quiet && acc_mag > (UW) PICKUP_LIFT_MG)
            ? pickup_lift_n + 1
            : 0;

    if (pickup_tilt_n >= PICKUP_TILT_N)
    {
        tm_printf((UB *) "[imu] PICKUP: tilted, roll %d pitch %d (0.25 deg, "
                         "limit %d), "
                         "|a| %d mg, %d ms\n",
                  (INT) navi.roll_q, (INT) navi.pitch_q, (INT) PICKUP_TILT_Q,
                  (INT) acc_mag, (INT) (PICKUP_TILT_N * PERIOD_IMU_MS));
        fault_raise(FAULT_PICKUP);
    }
    else if (pickup_lift_n >= PICKUP_LIFT_N)
    {
        tm_printf((UB *) "[imu] PICKUP: lifted, |a| %d mg (limit %d)\n",
                  (INT) acc_mag, (INT) PICKUP_LIFT_MG);
        fault_raise(FAULT_PICKUP);
    }
}

LOCAL void process (const IMU_RAW * raw)
{
    NAV_STATE ns;
    IMU_VEC   a, m;
    INT       roll, pitch;
    UW        horiz;

    raw_now = *raw; /* for the Idle readout (axis checks) */
    to_car(raw, &a, &m);

    /* JY-1.7: remove the car's own forward acceleration (mm/s^2 -> mg) */
    nav_read(&ns);
    a.f -= ((INT) ns.m.a_mm_s2 * 1000) / 9807;

    roll    = imu_roll_q(&a);
    pitch   = imu_pitch_q(&a);
    lat_now = lateral_q14(a, &ns); /* every sample: keeps turn_q fresh */

    if (cal_on)
    {
        calib_step(roll, pitch, lat_now, &m);
    }
    navi.roll_q  = sat8(roll - imu_cal.roll0_q);
    navi.pitch_q = sat8(pitch - imu_cal.pitch0_q);

    m.f -= imu_cal.mag_off_f;
    m.l -= imu_cal.mag_off_l;
    m.u -= imu_cal.mag_off_u;
    navi.heading_deg = (UH) imu_heading_deg(&a, &m);

    /* humps only while following the line: picking the car up in Idle,
       or the pivots at junctions / in the bypass, are not humps */
    {
        SYS_STATE st = sys_state_get();

        if (!cal_on && (st == SYS_LINE_FOLLOW || st == SYS_HUMP_TRAVERSE))
        {
            hump_step(lat_now, ns.m.odo_mm);
        }
        else
        {
            roll_f16 = (W) navi.roll_q << 4; /* start each run settled */
            hump_reset(lat_now, ns.m.v_mm_s);
        }
    }

    /* impact: a hit is a sudden HORIZONTAL acceleration; humps are
       vertical.  Skipped while turning by hand in calibration. */
    horiz = imu_isqrt((uint64_t) ((W) a.f * a.f + (W) a.l * a.l));
    if (!cal_on && (W) (now_us() - impact_hold_until) >= 0 &&
        SYS_IS_RUNNING(sys_state_get()) && horiz > IMPACT_MG)
    {
        if (++impact_n >= IMPACT_N)
        {
            tm_printf((UB *) "[imu] IMPACT: %d mg horizontal\n", (INT) horiz);
            fault_raise(FAULT_IMPACT);
            impact_n = 0;
        }
    }
    else
    {
        impact_n = 0;
    }

    pickup_check();
}

EXPORT void imu_task (INT stacd, void * exinf)
{
    IMU_RAW raw;
    UINT    k = 0;

    (void) stacd;
    (void) exinf;
    wcet_set_period(WCET_IMU, PERIOD_IMU_MS);

    for (;;)
    {
        UW  t0;
        INT r;

        tk_slp_tsk(TMO_FEVR);
        t0 = wcet_release(WCET_IMU);

        check_calibrate();

        if (imu_driver_status != DRV_IMU_OK)
        {
            navi.imu_ok = 0; /* running without IMU */
        }
        else
        {
            r = drv_imu_read(&raw);
            if (r == DRV_IMU_OK)
            {
                fails       = 0;
                navi.imu_ok = 1;
                if (navi.terrain == TER_SENSOR_FAULT)
                {
                    navi.terrain = TER_LEVEL;
                }
                process(&raw);
            }
            else
            {
                navi.imu_ok = 0;
                if (++fails >= IMU_FAIL_LIMIT)
                {
                    navi.terrain = TER_SENSOR_FAULT;
                    /* latched: Supervisor clears */
                    fault_raise(FAULT_IMU_SENSOR);
                }
            }
        }

        /* sensor check in Idle, every 2 s (car stopped: printf is harmless).
           The right wheel height is the MEAN of the last 2 s of samples
           (one raw sample jumps by +-3 mm even parked on flat ground), with
           the lowest and highest sample for the spread.  Put a spacer of
           known height under the right wheel to check it.  */
        if (sys_state_get() == SYS_IDLE && navi.imu_ok)
        {
            INT h = h_x10_of(lat_now, imu_cal.lat0_q14);

            if (idle_n == 0)
            {
                idle_lo = h;
                idle_hi = h;
            }
            if (h < idle_lo)
            {
                idle_lo = h;
            }
            if (h > idle_hi)
            {
                idle_hi = h;
            }
            idle_sum += h;
            idle_n++;
            if (++k >= (2000 / PERIOD_IMU_MS))
            {
                INT mean = (INT) (idle_sum / idle_n);
                INT ma   = (mean < 0) ? -mean : mean;
                INT la   = (idle_lo < 0) ? -idle_lo : idle_lo;
                INT ha   = (idle_hi < 0) ? -idle_hi : idle_hi;

                k = 0;
                tm_printf((UB *) "[imu] idle: heading %d deg, roll %d, pitch "
                                 "%d (0.25 deg), "
                                 "compass %s; right wheel %s%d.%d mm (2 s "
                                 "mean, range %s%d.%d..%s%d.%d); "
                                 "accel raw x %d y %d z %d (2 mg/count), car "
                                 "fwd %d left %d up %d mg, "
                                 "|a| %d mg\n",
                          (INT) navi.heading_deg, (INT) navi.roll_q,
                          (INT) navi.pitch_q,
                          imu_cal.mag_ok ? "calibrated" : "NOT calibrated",
                          (mean < 0) ? "-" : "", ma / 10, ma % 10,
                          (idle_lo < 0) ? "-" : "", la / 10, la % 10,
                          (idle_hi < 0) ? "-" : "", ha / 10, ha % 10,
                          (INT) raw_now.ax, (INT) raw_now.ay, (INT) raw_now.az,
                          (INT) acc_now.f, (INT) acc_now.l, (INT) acc_now.u,
                          (INT) acc_mag);
                idle_sum = 0;
                idle_n   = 0;
            }
        }
        else
        {
            k        = 0;
            idle_sum = 0;
            idle_n   = 0;
        }

        navi.t_us = now_us();
        nav_imu_write(&navi);
        wcet_end(WCET_IMU, t0);
    }
}

/*** end of file ***/
