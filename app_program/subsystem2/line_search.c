/*
 *      line_search.c
 *      LineSearch state machine.                                 Owner: Ryan
 *      Runs in the Supervisor's task.  No waiting, no motor access.
 *
 *      One search "round", repeated until the line is found or the car has
 *      travelled S (LS_S_MM) since the search began:
 *
 *          SWEEP_1   pivot ALPHA towards the side the line was last seen
 *          SWEEP_2   pivot 2 x ALPHA the other way
 *          CENTRE    pivot ALPHA back to the starting heading
 *          CREEP     drive S1 (LS_S1_MM) straight ahead at SLOW
 *
 *      Line lost starts with the sweep (the line is usually just beside the
 *      car, on the outside of a curve).  After a bypass or a turn the line is
 *      expected ahead, so those start with a creep.
 *
 *      Found: either sensor sees black for LS_CONFIRM_MM of travel while
 *      creeping, or for LS_CONFIRM_SAMPLES samples in a row (5 ms each) while
 *      pivoting (the odometer hardly moves in a pivot).  A single black
 *      sample is treated as noise.
 *
 *      Motion replies: only EV_MOTION_DONE carrying the sequence number of
 *      the command sent from here counts.  ABORTED replies are ignored: the
 *      first one is always the Supervisor's FOLLOW being replaced.
 *
 *      TODO RY-1.15 / R9: the +-beta heading check and the offset window
 *      before accepting a line found after a bypass (needs the compass).
 */

#include "line_search.h"
#include "../fault_flags.h"
#include "../msg_bus.h"

LOCAL LS_STATE  st    = LSS_IDLE; /* LSS_CONFIRM while confirming */
LOCAL LS_STATE  phase = LSS_IDLE; /* current move: CREEP / SWEEP_* / CENTRE */
LOCAL LS_REF    ref;
LOCAL LS_REASON why;
LOCAL UH        start_odo; /* where the search started */
LOCAL UH        seen_odo;  /* where the line was first seen again */
LOCAL UINT      seen_n;    /* consecutive black samples */
LOCAL BOOL      moving;
LOCAL UINT      seq;
LOCAL INT       side; /* +1: first sweep to the left, -1: right */

LOCAL LS_RESULT result (LS_KIND k)
{
    LS_RESULT r = {k, 0};
    return r;
}

LOCAL LS_RESULT send (MC_TYPE type, INT angle_deg, INT dist_mm)
{
    LS_RESULT r;

    seq    = (seq + 1U) & 0xFU;
    r.kind = LS_MOVE;
    r.cmd  = motion_cmd_make(type, SPD_SLOW, angle_deg, dist_mm, seq);
    moving = TRUE;
    return r;
}

/* Start the move for 'phase' */
LOCAL LS_RESULT start_phase (void)
{
    switch (phase)
    {
        case LSS_SWEEP_L: /* first sweep */
            return send(MC_TURN_REL, side * LS_ALPHA_DEG, 0);
        case LSS_SWEEP_R: /* second sweep, other way */
            return send(MC_TURN_REL, -side * 2 * LS_ALPHA_DEG, 0);
        case LSS_CENTRE:
            return send(MC_TURN_REL, side * LS_ALPHA_DEG, 0);
        case LSS_CREEP:
        default:
            phase = LSS_CREEP;
            return send(MC_DRIVE, 0, LS_S1_MM);
    }
}

/* The current move finished: pick the next one */
LOCAL LS_STATE next_phase (LS_STATE p)
{
    switch (p)
    {
        case LSS_SWEEP_L:
            return LSS_SWEEP_R;
        case LSS_SWEEP_R:
            return LSS_CENTRE;
        case LSS_CENTRE:
            return LSS_CREEP;
        case LSS_CREEP:
        default:
            return LSS_SWEEP_L;
    }
}

EXPORT void line_search_start (LS_REASON reason, const LS_REF * r)
{
    ref       = *r;
    why       = reason;
    start_odo = r->odo_mm;
    moving    = FALSE;
    seen_n    = 0;
    /* + offset = line expected to the LEFT: look there first */
    side  = (r->expected_offset_mm < 0) ? -1 : +1;
    phase = (reason == LS_REASON_LINE_LOST) ? LSS_SWEEP_L : LSS_CREEP;
    st    = phase;
}

EXPORT LS_RESULT line_search_step (EV_CODE code, UW a, UW odo_mm)
{
    /* Signed and wrap-safe: pivots and the car rolling back a little move
       the odometer BACKWARDS, which as an unsigned difference would look
       like 65 m travelled and fail the search at once. */
    H  d         = (H) ((UH) odo_mm - start_odo);
    UH travelled = (UH) ((d < 0) ? -d : d);

    if (st == LSS_IDLE || st == LSS_FAIL)
    {
        return result(LS_WAIT);
    }

    if (travelled >= LS_S_MM)
    {
        st    = LSS_FAIL;
        phase = LSS_FAIL;
        fault_raise(FAULT_LINE_SEARCH_FAIL); /* RY-1.16 */
        return send(MC_STOP, 0, 0);
    }

    if (code == EV_LINE_SAMPLE)
    {
        BOOL seen = LS_SEEN_L(a) || LS_SEEN_R(a);

        if (!seen)
        {
            seen_n = 0;
            st     = phase; /* noise: carry on */
        }
        else
        {
            if (seen_n == 0)
            {
                seen_odo = (UH) odo_mm;
            }
            seen_n++;
            st = LSS_CONFIRM;
            if (seen_n >= LS_CONFIRM_SAMPLES ||
                (H) ((UH) odo_mm - seen_odo) >= LS_CONFIRM_MM)
            {
                /* TODO RY-1.15 / R9: check heading within
                   +-beta of ref.heading_deg and the offset
                   window first */
                st    = LSS_IDLE;
                phase = LSS_IDLE;
                return result(LS_FOUND);
            }
        }
    }

    if (code == EV_MOTION_DONE && moving && MD_SEQ(a) == seq &&
        MD_RESULT_OF(a) != MD_ABORTED)
    {
        moving = FALSE;
        phase  = next_phase(phase);
        if (st != LSS_CONFIRM)
        {
            st = phase;
        }
    }
    if (!moving)
    {
        return start_phase();
    }
    return result(LS_WAIT);
}

EXPORT LS_STATE line_search_state (void)
{
    return st;
}

/*** end of file ***/
