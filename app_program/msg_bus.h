/*
 *      msg_bus.h
 *      The kernel objects behind msg_types.h, and safe send helpers.
 *                                                            Owner: Christian
 *      Plan refs: section 2.2, CH-1.19, RY-1.26 (burst test).
 *
 *      Why helpers instead of raw tk_snd_mbf:
 *      - A FIFO cannot drop selectively once it is full.  So "never drop a
 *        junction or barcode" is enforced at the SENDER: droppable events
 *        (EV_LINE_SAMPLE, EV_SNAPSHOT_REQ) are only sent while more than
 *        SUP_RESERVE_MSGS slots are free.  The reserve is kept for the rest.
 *      - Every send is TMO_POL, so it is safe from tasks, cyclic handlers and
 *        interrupt handlers, and never blocks a control loop.
 *      - Every failure is counted.  bus_stats.sup_lost_keep must stay 0.
 */

#ifndef MSG_BUS_H
#define MSG_BUS_H

#include <tk/tkernel.h>
#include "msg_types.h"

/* Queue depths (messages).  Kernel cost per message = size + 4 B header. */
#define SUP_QUEUE_MSGS    16
#define SUP_RESERVE_MSGS  6 /* free slots kept for must-keep events */
#define MOTION_QUEUE_MSGS 8
#define SCAN_QUEUE_MSGS   4
#define TELEM_QUEUE_MSGS  8

IMPORT ID mbf_sup_event;  /* SUP_EVENT,  reader: Supervisor          */
IMPORT ID mbf_motion_cmd; /* UW (MC_*),  reader: Motion Control      */
IMPORT ID mbf_scan_req;   /* UW (SCAN_*), reader: Obstacle Scan      */
IMPORT ID mbf_telem;      /* TELEM_MSG,  reader: Telemetry           */
IMPORT ID flg_calib;      /* CAL_REQ_* bits                          */

typedef struct
{
    UW sup_sent;
    UW sup_dropped_low; /* droppable events skipped (expected) */
    UW sup_lost_keep;   /* must-keep events lost: must be 0 */
    UW motion_lost;
    UW scan_lost;
    UW telem_lost;
} BUS_STATS;
IMPORT volatile BUS_STATS bus_stats;

IMPORT ER msg_bus_init(void); /* call once, before any task starts */

/* Microsecond timestamp (32-bit, wraps every 71.6 min).  Any context. */
IMPORT UW now_us(void);

/* Post to the Supervisor.  Droppable codes are dropped first. */
IMPORT ER sup_post(EV_SRC src, EV_CODE code, UW a, UW b);

/* Supervisor -> Motion Control.  Clamps angle and distance to the field. */
IMPORT UW motion_cmd_make(MC_TYPE type, SPEED_LEVEL speed, INT angle_deg,
                          INT dist_mm, UINT seq);
IMPORT ER motion_cmd_send(UW cmd);

IMPORT ER scan_req_send(UW req);
IMPORT ER telem_send(const TELEM_MSG * msg);

/* Line error latest-value word (msg_types.h section 5) */
IMPORT void line_err_publish(INT err, UINT flags);
IMPORT UW   line_err_read(void);

/* System state, published by the Supervisor only */
IMPORT void      sys_state_set(SYS_STATE s);
IMPORT SYS_STATE sys_state_get(void);

#endif /* MSG_BUS_H */

/*** end of file ***/
