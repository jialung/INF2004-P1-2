/*
 *      task_motion.c
 *      Motion Control task, 10 ms, core 1, priority 8.        Owner: Jia Lung
 *      The ONLY code that writes the motor PWM.
 *      Plan refs: JL-1.13..1.20, chain 4.2 step 3, chain 4.5 step 3, D1.
 *
 *      Every cycle, in this order:
 *        1. read the fault flag            -> any stop bit: brake NOW
 *        2. take new motion commands        (newest wins)
 *        3. read encoders, update odometry  -> stall / encoder fault check
 *        4. run the active command
 *        5. publish nav_state motion half
 *
 *      What is real here: fault cut, command handling, odometry, stall
 *      detection, open-loop line steering, distance drive, encoder pivots.
 *      What is still TODO (marked with plan IDs): calibration constants,
 *      per-wheel PID, dead-reckoned x/y, compass check after turns.
 */

#include "subsystem1.h"
#include "drv_motor.h"
#include "encoders.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../wcet.h"
#include "../hal.h"
#include "../subsystem3/imu_math.h" /* integer sin / cos table */

EXPORT volatile UW motion_cut_t_us;

/* ---- Calibration: geometry is in hw_alloc.h (JL-1.4, JL-1.5) -------- */

/* mm per count, Q16: pi * D / counts */
#define MM_PER_COUNT_Q16 \
    ((W) ((205887LL * WHEEL_DIAMETER_MM) / ENC_COUNTS_PER_REV))
/* 0.25 deg per (dR - dL) count, Q16: mm_per_count * (4 * 180 / pi) / track.
   Uses the EFFECTIVE track width (hw_alloc.h, TURN_TRACK_X10), so the
   30 Oct distance calibration left the turn scale unchanged. */
#define TURN_Q_PER_COUNT_Q16 \
    ((W) (((long long) MM_PER_COUNT_Q16 * 229 * 10) / TURN_TRACK_X10))

/* ---- Tuning: PLACEHOLDERS (JL-1.14, JL-2.6) -------------------------- */
/* 3.5x slower than { 0, 110, 150, 300 } / turn 250 / steering 3 (23 Oct).
   Steering is scaled by the same factor, so the car still turns along the
   same path, only slower.  These are wheel SPEED targets (the PI below
   raises the duty as needed to get the wheels moving at low speed). */
LOCAL const INT speed_pm[4] = {0, 31, 43, 86}; /* STOP SLOW HUMP CRUISE */
#define K_LINE_NUM 6 /* steering = err * NUM / DEN, per mille */
#define K_LINE_DEN 7 /* 6/7 = 3 / 3.5 */
/* How much slower to drive while Line Following reports an arc (LE_CURVE).
   The follow radius is base x TRACK_WIDTH_MM / (2 x corr), so x % off the
   speed takes x % off the radius: at SPD_CRUISE (86) and a full-step error
   (60 -> corr 51) that is 95 mm at full speed, 86 mm at 10 % slower.
   Raise it if the car still runs wide in the loop; the cost is lap time.
   0 = off (the behaviour before 7 Nov). */
#define CURVE_SLOW_PCT 10
#define TURN_SPEED_PM  71
/* Fast pivot, used only when the turn command carries SPD_CRUISE: the
   90 / 180 deg pivots at a junction (supervisor.c).  Line search, bypass
   and calibrate send SPD_SLOW and keep TURN_SPEED_PM.
   7 Nov.  Pivot rate = 0.8 deg/s per unit (2 wheels at +-v on a 120.8 mm
   track, CPS_PER_PM 10): 71 -> ~57 deg/s (90 deg in ~1.6 s),
   150 -> ~120 deg/s (90 deg in ~0.75 s).  The 23 Oct turn setting before
   the 3.5x slow-down was 250 (~200 deg/s).  The half-speed zone before the
   end is longer (30 deg) so the faster pivot does not overshoot.
   Raise TURN_FAST_PM for a sharper turn; if the car overshoots the 90 deg
   or the wheels slip on the floor, lower it or widen TURN_FAST_SLOW_Q. */
#define TURN_FAST_PM 150
/* MC_DRIVE with SPD_HUMP = obstacle bypass legs only: 46 mm/s (11 Nov) */
#define DRIVE_BYPASS_PM   55
#define LINE_STALE_CYCLES 3

/* ---- Wheel speed PI (JL-1.14, FR5) ------------------------------------
 * Each wheel gets a TARGET speed instead of a fixed duty.  The target is
 * the old duty value (speed_pm, steering) converted to encoder counts with
 * CPS_PER_PM, so the speed table keeps its meaning: "300" still means "as
 * fast as 300 per mille used to drive", now held by feedback.
 *
 *   duty = target_pm                    feed-forward (the old open loop)
 *        + KP * (target - counts)       this cycle's speed error
 *        + KI * sum(target - counts)    position lag: makes both wheels
 *                                       cover the same distance -> straight
 *
 * CPS_PER_PM: encoder counts per second per 1 per-mille of duty.  Bench
 * 23 Oct, wheels off the ground: 220 per mille -> ~958 counts / 400 ms
 * = ~2400 counts/s, so ~10.9.  On the floor it is lower; the integral
 * makes up the difference.  TODO JL-1.14: measure on the floor.         */
#define CPS_PER_PM  10
#define PI_KP       4 /* per mille per count/cycle of error */
#define PI_KI_NUM   1 /* per mille per count of lag ...     */
#define PI_KI_DEN   1
#define PI_I_MAX_PM 300 /* integral term limit, per mille     */
#define DUTY_MAX_PM 900 /* never ask for more than this       */
/* target counts per 10 ms cycle, Q8 */
#define TGT_Q8(pm) (((W) (pm) * CPS_PER_PM * 256 * PERIOD_MOTION_MS) / 1000)

/* Slow down near the end of a distance move or a turn (less overshoot) */
#define DRIVE_SLOW_MM    30       /* last 30 mm at half speed */
#define TURN_SLOW_Q      (15 * 4) /* last 15 deg at half speed */
#define TURN_FAST_SLOW_Q (30 * 4) /* the same for a fast pivot */

/* Speed (nav_state v_mm_s) is averaged over this many 10 ms cycles */
#define V_WIN 10
/* ---- Fault detection thresholds (JL-1.20) ---------------------------- */
#define STALL_MIN_PM 300 /* only judge a wheel driven this hard */
#define STALL_CYCLES 30  /* 300 ms with no counts */

typedef struct
{
    MOTION_STATE state;
    UW           cmd;      /* active MOTION_CMD word */
    BOOL         cmd_live; /* a command is running and owes a reply */
    BOOL         pending;  /* command held while OBST_STOP is set */
    long long    start_odo_q16;
    long long    start_turn_q16;
    W            target; /* mm (drive) or 0.25 deg (turn), signed */
    INT          stale;  /* cycles with the same line-error seq */
    UINT         last_le_seq;
} MOTION_CTX;

LOCAL MOTION_CTX mc;
LOCAL NAV_MOTION navm;
/* 64-bit: a 32-bit Q16 odometer would overflow after 32 m */
LOCAL long long odo_q16;      /* running distance, mm Q16 */
LOCAL long long turn_q16;     /* running heading change, 0.25 deg Q16 */
LOCAL long long x_q16, y_q16; /* dead-reckoned position, mm Q16 */
LOCAL W         prev_cnt[2];
LOCAL W         dcnt[2];   /* counts this cycle, forward = + */
LOCAL INT       still[2];  /* cycles with no counts while driven */
LOCAL W         lag_q8[2]; /* PI: sum of (target - counts), Q8 */

LOCAL void pi_reset (void)
{
    lag_q8[0] = 0;
    lag_q8[1] = 0;
}

LOCAL INT clamp (INT v, INT lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

/* Drive both wheels at target speeds given in per mille of the old duty
   scale (+ = forward).  Called once per cycle, after update_odometry(). */
LOCAL void wheels_set (INT tgt_l_pm, INT tgt_r_pm)
{
    const INT tgt[2] = {tgt_l_pm, tgt_r_pm};
    INT       duty[2];
    INT       i;
    const W   lag_max = ((W) PI_I_MAX_PM * 256 * PI_KI_DEN) / PI_KI_NUM;

    for (i = 0; i < 2; i++)
    {
        W e_q8 = TGT_Q8(tgt[i]) - dcnt[i] * 256;
        W p, k;

        lag_q8[i] += e_q8;
        if (lag_q8[i] > lag_max)
        {
            lag_q8[i] = lag_max; /* anti-windup */
        }
        if (lag_q8[i] < -lag_max)
        {
            lag_q8[i] = -lag_max;
        }

        p       = (PI_KP * e_q8) / 256;
        k       = (lag_q8[i] * PI_KI_NUM) / (256 * PI_KI_DEN);
        duty[i] = clamp(tgt[i] + (INT) p + (INT) k, DUTY_MAX_PM);
    }
    drv_motor_set(duty[0], duty[1]);
}

LOCAL void wheels_brake (void)
{
    drv_motor_brake();
    pi_reset();
}

LOCAL void reply (MD_RESULT r)
{
    if (mc.cmd_live)
    {
        (void) sup_post(SRC_MOTION, EV_MOTION_DONE, MD_A(MC_SEQ(mc.cmd), r),
                        navm.odo_mm);
        mc.cmd_live = FALSE;
    }
}

LOCAL void start_cmd (UW cmd)
{
    NAV_STATE ns;
    W         deg;

    if (mc.cmd_live)
    {
        reply(MD_ABORTED); /* replaced before it finished */
    }
    mc.cmd      = cmd;
    mc.cmd_live = (MC_TYPE_OF(cmd) != MC_STOP);
    mc.pending  = FALSE; /* a new command replaces any held one */
    pi_reset();          /* new targets: start the lag from zero */
    mc.start_odo_q16  = odo_q16;
    mc.start_turn_q16 = turn_q16;

    switch (MC_TYPE_OF(cmd))
    {
        case MC_STOP:
            mc.state = MS_IDLE;
            break;
        case MC_FOLLOW:
            mc.state = MS_FOLLOW;
            break;
        case MC_DRIVE:
            mc.target = MC_DIST_MM(cmd);
            mc.state  = MS_DRIVE;
            break;
        case MC_TURN_REL:
            mc.target = MC_ANGLE_DEG(cmd) * 4; /* +: left / CCW */
            mc.state  = MS_TURN;
            break;
        case MC_TURN_ABS:
            /* convert to relative using the compass now.
               TODO JY-1.8: heading convention (0 = north, clockwise?) */
            nav_read(&ns);
            deg = MC_ANGLE_DEG(cmd) - (W) ns.i.heading_deg;
            while (deg > 180)
            {
                deg -= 360;
            }
            while (deg <= -180)
            {
                deg += 360;
            }
            mc.target = -deg * 4; /* compass CW = car turning right */
            mc.state  = MS_TURN;
            break;
        default:
            mc.state = MS_IDLE;
            reply(MD_REJECTED);
            break;
    }
}

LOCAL void update_odometry (void)
{
    W   cnt[2], d[2];
    INT i;
    W   dist_q16;

    cnt[0] = ENC_SIGN_L * encoders_count(ENC_LEFT);
    cnt[1] = ENC_SIGN_R * encoders_count(ENC_RIGHT);
    for (i = 0; i < 2; i++)
    {
        d[i]        = cnt[i] - prev_cnt[i]; /* wrap-safe */
        prev_cnt[i] = cnt[i];
        dcnt[i]     = d[i];
    }

    dist_q16 = ((d[0] + d[1]) * MM_PER_COUNT_Q16) / 2;
    odo_q16 += dist_q16;
    turn_q16 += (long long) (d[1] - d[0]) * TURN_Q_PER_COUNT_Q16;

    /* Speed from the wheel encoders: the distance both wheels covered
       (average) over the last V_WIN cycles.  One cycle alone is too
       coarse: one count in 10 ms = 8.4 mm/s, so at ~70 mm/s the old
       per-cycle value jumped by +-8 mm/s every cycle, and its
       derivative (a_mm_s2, used by the IMU to remove the car's own
       acceleration from the tilt) by +-840 mm/s^2.  Over 100 ms the step
       is 0.84 mm/s, for ~50 ms of extra lag. */
    {
        LOCAL W    v_win[V_WIN]; /* dist_q16 per cycle */
        LOCAL W    v_sum;
        LOCAL UINT v_i;
        H          v;

        v_sum += dist_q16 - v_win[v_i];
        v_win[v_i] = dist_q16;
        v_i        = (v_i + 1U) % V_WIN;
        v          = (H) ((v_sum * (1000 / (PERIOD_MOTION_MS * V_WIN))) >> 16);
        navm.a_mm_s2 = (H) ((v - navm.v_mm_s) * (1000 / PERIOD_MOTION_MS));
        navm.v_mm_s  = v;
    }
    navm.odo_mm = (UH) (odo_q16 >> 16);
    navm.turn_q = (H) (turn_q16 >> 16);

    /* JL-1.18: dead-reckoned position for the course map (FR14).
       Heading from the encoders (0 = the direction at START, + = left /
       counter-clockwise), so x = forward at START, y = to its left. */
    {
        INT hdeg = (INT) ((turn_q16 >> 16) / 4);
        x_q16 += ((long long) dist_q16 * imu_cos_q14(hdeg)) >> 14;
        y_q16 += ((long long) dist_q16 * imu_sin_q14(hdeg)) >> 14;
        navm.x_mm = (W) (x_q16 >> 16);
        navm.y_mm = (W) (y_q16 >> 16);
    }

    /* Stall / encoder fault (JL-1.20): a wheel driven hard with no counts */
    for (i = 0; i < 2; i++)
    {
        INT duty = drv_motor_get((UINT) i);
        if ((duty >= STALL_MIN_PM || duty <= -STALL_MIN_PM) && d[i] == 0)
        {
            if (still[i] < STALL_CYCLES)
            {
                still[i]++;
            }
        }
        else
        {
            still[i] = 0;
        }
    }
    if (still[0] >= STALL_CYCLES && still[1] >= STALL_CYCLES)
    {
        fault_raise(FAULT_MOTOR_STALL); /* both stuck: blocked */
    }
    else if (still[0] >= STALL_CYCLES || still[1] >= STALL_CYCLES)
    {
        fault_raise(FAULT_ENC_FAULT); /* one silent: encoder */
    }
    navm.enc_ok = (UB) ((still[0] < STALL_CYCLES) && (still[1] < STALL_CYCLES));
}

LOCAL void run_follow (void)
{
    UW  le   = line_err_read();
    INT base = speed_pm[MC_SPEED(mc.cmd)];
    INT err  = 0, corr;

    if (LE_SEQ(le) == mc.last_le_seq)
    {
        if (mc.stale < LINE_STALE_CYCLES)
        {
            mc.stale++;
        }
    }
    else
    {
        mc.stale       = 0;
        mc.last_le_seq = LE_SEQ(le);
    }
    if ((LE_FLAGS(le) & LE_VALID) && mc.stale < LINE_STALE_CYCLES)
    {
        err = LE_ERR(le); /* RY-1.8: + = line toward the LEFT sensor */
        /* In an arc, go CURVE_SLOW_PCT slower.  The steering mix below
           turns a radius of base x TRACK / (2 x corr), so the arc gets
           tighter in the same proportion as the speed drops, with no
           change to the gain (which would also affect straights).
           A stale or invalid word leaves the speed alone. */
        if (LE_FLAGS(le) & LE_CURVE)
        {
            base -= (base * CURVE_SLOW_PCT) / 100;
        }
    }
    /* steering mix on the wheel speed targets; the PI holds them */
    corr = (err * K_LINE_NUM) / K_LINE_DEN;
    wheels_set(base - corr, base + corr);
}

LOCAL void run_drive (void)
{
    W   done_mm = (W) ((odo_q16 - mc.start_odo_q16) >> 16);
    INT s       = speed_pm[MC_SPEED(mc.cmd)];

    W left_mm;

    if (MC_SPEED(mc.cmd) == SPD_HUMP)
    {
        s = DRIVE_BYPASS_PM;
    }

    if (s == 0)
    {
        s = speed_pm[SPD_SLOW];
    }
    if ((mc.target >= 0 && done_mm >= mc.target) ||
        (mc.target < 0 && done_mm <= mc.target))
    {
        wheels_brake();
        mc.state = MS_IDLE;
        reply(MD_DONE);
        return;
    }
    left_mm = mc.target - done_mm;
    if (left_mm < 0)
    {
        left_mm = -left_mm;
    }
    if (left_mm < DRIVE_SLOW_MM)
    {
        s /= 2;
    }
    /* JL-2.1: equal targets + the PI's position lag term keep both
       wheels at the same distance, which holds the heading */
    if (mc.target >= 0)
    {
        wheels_set(s, s);
    }
    else
    {
        wheels_set(-s, -s);
    }
}

LOCAL void run_turn (void)
{
    W   done_q = (W) ((turn_q16 - mc.start_turn_q16) >> 16);
    W   left_q;
    INT s      = TURN_SPEED_PM;
    W   slow_q = TURN_SLOW_Q;

    if (MC_SPEED(mc.cmd) == SPD_CRUISE)
    { /* junction pivot: fast */
        s      = TURN_FAST_PM;
        slow_q = TURN_FAST_SLOW_Q;
    }

    if ((mc.target >= 0 && done_q >= mc.target) ||
        (mc.target < 0 && done_q <= mc.target))
    {
        wheels_brake();
        mc.state = MS_IDLE;
        /* TODO JL-1.28 / JY-1.20: compare with the compass, +-5 deg */
        reply(MD_DONE);
        return;
    }
    left_q = mc.target - done_q;
    if (left_q < 0)
    {
        left_q = -left_q;
    }
    if (left_q < slow_q)
    {
        s /= 2;
    }
    if (mc.target >= 0)
    {
        wheels_set(-s, s); /* left */
    }
    else
    {
        wheels_set(s, -s); /* right */
    }
}

EXPORT void motion_task (INT stacd, void * exinf)
{
    UW   cmd;
    UINT faults;

    (void) stacd;
    (void) exinf;
    LOG("motion", "task on core %d\n", (INT) hal_core_id());
    wcet_set_period(WCET_MOTION, PERIOD_MOTION_MS);
    mc.state = MS_IDLE;
    wheels_brake();
    /* start the odometer from the counts the encoders already hold, so
       the first cycle does not see a jump (odo 65535 at boot) */
    prev_cnt[0] = ENC_SIGN_L * encoders_count(ENC_LEFT);
    prev_cnt[1] = ENC_SIGN_R * encoders_count(ENC_RIGHT);

    for (;;)
    {
        UW t0;

        tk_slp_tsk(TMO_FEVR);
        t0 = wcet_release(WCET_MOTION);

        /* 1. safety first: one read, one decision, this cycle */
        faults = fault_peek();
        if (faults & FAULT_STOP_MASK)
        {
            wheels_brake();
            if (mc.state != MS_CUT)
            {
                motion_cut_t_us = now_us();
                if (faults & FAULT_TRIP_MASK)
                {
                    reply(MD_ABORTED); /* fault: command is dead */
                    mc.pending = FALSE;
                }
                else if (mc.cmd_live)
                {
                    mc.pending = TRUE; /* OBST_STOP: resume later */
                }
                mc.state = MS_CUT;
            }
        }

        /* 2. new commands (newest wins) */
        while (tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL) == (INT) sizeof(cmd))
        {
            if (faults & FAULT_TRIP_MASK)
            {
                mc.cmd      = cmd;
                mc.cmd_live = TRUE;
                reply(MD_REJECTED);
            }
            else if (faults & FAULT_OBST_STOP)
            {
                if (mc.cmd_live)
                {
                    reply(MD_ABORTED);
                }
                mc.cmd      = cmd;
                mc.cmd_live = (MC_TYPE_OF(cmd) != MC_STOP);
                mc.pending  = mc.cmd_live; /* runs when the path clears */
            }
            else
            {
                start_cmd(cmd);
            }
        }

        /* 3. odometry */
        update_odometry();

        /* 4. run */
        if (mc.state == MS_CUT && (faults & FAULT_STOP_MASK) == 0)
        {
            if (mc.pending)
            {
                mc.pending  = FALSE;
                mc.cmd_live = FALSE; /* start_cmd re-arms it */
                start_cmd(mc.cmd);
            }
            else
            {
                mc.state = MS_IDLE;
            }
        }
        switch (mc.state)
        {
            case MS_FOLLOW:
                run_follow();
                break;
            case MS_DRIVE:
                run_drive();
                break;
            case MS_TURN:
                run_turn();
                break;
            case MS_IDLE:
            case MS_CUT:
            default:
                wheels_brake();
                break;
        }

        /* 5. publish */
        navm.motion_state = (UB) mc.state;
        navm.t_us         = now_us();
        nav_motion_write(&navm);

        wcet_end(WCET_MOTION, t0);
    }
}

/*** end of file ***/
