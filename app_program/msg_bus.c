/*
 *      msg_bus.c
 *      Creates the shared channels and implements the send rules.
 *                                                            Owner: Christian
 */

#include "app.h"
#include "msg_bus.h"
#include "hal.h"

EXPORT ID                 mbf_sup_event;
EXPORT ID                 mbf_motion_cmd;
EXPORT ID                 mbf_scan_req;
EXPORT ID                 mbf_telem;
EXPORT ID                 flg_calib;
EXPORT volatile BUS_STATS bus_stats;

LOCAL volatile UW line_err_word;        /* written by Line Following only */
LOCAL volatile UB sys_state = SYS_INIT; /* written by Supervisor only */
LOCAL volatile UH src_seq[SRC_BUTTON + 1];

#define MBF_MSG_COST(sz) ((sz) + 4) /* kernel adds a 4-byte header */

LOCAL ID make_mbf (INT msgsz, INT depth)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) (MBF_MSG_COST(msgsz) * depth);
    cmbf.maxmsz = msgsz;
    return tk_cre_mbf(&cmbf);
}

EXPORT ER msg_bus_init (void)
{
    T_CFLG cflg = {0};

    mbf_sup_event  = make_mbf((INT) sizeof(SUP_EVENT), SUP_QUEUE_MSGS);
    mbf_motion_cmd = make_mbf((INT) sizeof(UW), MOTION_QUEUE_MSGS);
    mbf_scan_req   = make_mbf((INT) sizeof(UW), SCAN_QUEUE_MSGS);
    mbf_telem      = make_mbf((INT) sizeof(TELEM_MSG), TELEM_QUEUE_MSGS);

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    flg_calib    = tk_cre_flg(&cflg);

    if (mbf_sup_event < E_OK)
    {
        return mbf_sup_event;
    }
    if (mbf_motion_cmd < E_OK)
    {
        return mbf_motion_cmd;
    }
    if (mbf_scan_req < E_OK)
    {
        return mbf_scan_req;
    }
    if (mbf_telem < E_OK)
    {
        return mbf_telem;
    }
    if (flg_calib < E_OK)
    {
        return flg_calib;
    }
    return E_OK;
}

EXPORT RAMFUNC UW now_us (void)
{
    return hal_now_us();
}

LOCAL BOOL is_droppable (EV_CODE code)
{
    return (code == EV_LINE_SAMPLE) || (code == EV_SNAPSHOT_REQ);
}

EXPORT ER sup_post (EV_SRC src, EV_CODE code, UW a, UW b)
{
    SUP_EVENT ev;
    ER        er;

    if (is_droppable(code))
    {
        T_RMBF rmbf;
        if (tk_ref_mbf(mbf_sup_event, &rmbf) != E_OK ||
            rmbf.frbufsz <
                (SZ) (MBF_MSG_COST(sizeof(SUP_EVENT)) * (SUP_RESERVE_MSGS + 1)))
        {
            bus_stats.sup_dropped_low++;
            return E_LIMIT;
        }
    }

    ev.code = (UB) code;
    ev.src  = (UB) src;
    ev.seq  = src_seq[src <= SRC_BUTTON ? src : SRC_SYSTEM]++;
    ev.t_us = now_us();
    ev.a    = a;
    ev.b    = b;

    er = tk_snd_mbf(mbf_sup_event, &ev, (INT) sizeof(ev), TMO_POL);
    if (er == E_OK)
    {
        bus_stats.sup_sent++;
    }
    else if (is_droppable(code))
    {
        bus_stats.sup_dropped_low++;
    }
    else
    {
        bus_stats.sup_lost_keep++; /* a real bug: test RY-1.26 */
    }
    return er;
}

EXPORT UW motion_cmd_make (MC_TYPE type, SPEED_LEVEL speed, INT angle_deg,
                           INT dist_mm, UINT seq)
{
    if (angle_deg > 127 * MC_ANGLE_UNIT_DEG)
    {
        angle_deg = 127 * MC_ANGLE_UNIT_DEG;
    }
    if (angle_deg < -128 * MC_ANGLE_UNIT_DEG)
    {
        angle_deg = -128 * MC_ANGLE_UNIT_DEG;
    }
    if (dist_mm > MC_DIST_MAX_MM)
    {
        dist_mm = MC_DIST_MAX_MM;
    }
    if (dist_mm < MC_DIST_MIN_MM)
    {
        dist_mm = MC_DIST_MIN_MM;
    }
    return MC_PACK(type, speed, angle_deg, dist_mm, seq);
}

EXPORT ER motion_cmd_send (UW cmd)
{
    ER er = tk_snd_mbf(mbf_motion_cmd, &cmd, (INT) sizeof(cmd), TMO_POL);
    if (er != E_OK)
    {
        bus_stats.motion_lost++;
    }
    return er;
}

EXPORT ER scan_req_send (UW req)
{
    ER er = tk_snd_mbf(mbf_scan_req, &req, (INT) sizeof(req), TMO_POL);
    if (er != E_OK)
    {
        bus_stats.scan_lost++;
    }
    return er;
}

EXPORT ER telem_send (const TELEM_MSG * msg)
{
    ER er = tk_snd_mbf(mbf_telem, (void *) msg, (INT) sizeof(*msg), TMO_POL);
    if (er != E_OK)
    {
        bus_stats.telem_lost++;
    }
    return er;
}

EXPORT void line_err_publish (INT err, UINT flags)
{
    LOCAL UB seq;

    if (err > 127)
    {
        err = 127;
    }
    if (err < -128)
    {
        err = -128;
    }
    seq++;
    /* one aligned 32-bit store: atomic on Cortex-M0+, no lock needed */
    line_err_word = LE_PACK(err, flags, seq);
    MEM_BARRIER();
}

EXPORT UW line_err_read (void)
{
    MEM_BARRIER();
    return line_err_word;
}

EXPORT void sys_state_set (SYS_STATE s)
{
    sys_state = (UB) s;
    MEM_BARRIER();
}

EXPORT SYS_STATE sys_state_get (void)
{
    MEM_BARRIER();
    return (SYS_STATE) sys_state;
}

/*** end of file ***/
