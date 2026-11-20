/*
 *      telem_frame.h
 *      Binary telemetry frame (CH-1.12), version 0.         Owner: Christian
 *      Plain <stdint.h> so the PC decoder test can compile it on a laptop
 *      (tests/host/test_msg_types.c).
 *
 *      Little-endian, 14 bytes:
 *        0      header: type 3 | version 1 | state 4   (state needs 4 bits:
 *               10 leaf states -- fix for the 3-bit field in the plan)
 *        1-2    fault mask (16 bit: 9 fault bits now)
 *        3-4    odometer, mm (wraps)
 *        5-6    speed, mm/s (signed)
 *        7-8    heading 9 | terrain 3 | speed level 2 | spare 2
 *        9      roll, 0.25 deg (signed)
 *        10     pitch, 0.25 deg (signed)
 *        11     hump count
 *        12     last letter 5 | reversed 1 | spare 2
 *        13     CRC-8, polynomial 0x07, init 0, over bytes 0-12
 */

#ifndef TELEM_FRAME_H
#define TELEM_FRAME_H

#include <stdint.h>

#define TF_TYPE_STATUS 1U
#define TF_VERSION     0U
#define TF_LEN         14U

typedef struct
{
    uint8_t  state;
    uint16_t faults;
    uint16_t odo_mm;
    int16_t  v_mm_s;
    uint16_t heading_deg;
    uint8_t  terrain;
    uint8_t  speed_level;
    int8_t   roll_q;
    int8_t   pitch_q;
    uint8_t  humps;
    uint8_t  letter; /* 31 = none yet */
    uint8_t  letter_rev;
} TELEM_FIELDS;

/* ---- Hooks for the Wi-Fi task (mqtt_utk.c, SDK side) -------------------
 * Events (FR12, FR14 map): the telemetry task writes one JSON line per
 * Supervisor event into a ring; the Wi-Fi task publishes and frees them.
 * Single producer, single consumer: head is written only by the
 * producer, tail only by the consumer.                                   */
#define TELEM_EVT_SLOTS 8U
#define TELEM_EVT_LEN   96U
extern volatile uint8_t  telem_evt[TELEM_EVT_SLOTS][TELEM_EVT_LEN];
extern volatile uint32_t telem_evt_head, telem_evt_tail;

/* Snapshot (FR13): a JSON summary of the robot's state right now, for
   the reply to a snapshot request.  Task context only (reads nav_state).
   Defined in task_telemetry.c.  Returns the length. */
uint32_t telem_snapshot(char * out, uint32_t size);

uint8_t  tf_crc8(const uint8_t * p, uint32_t n);
uint32_t tf_pack(const TELEM_FIELDS * f, uint8_t out[TF_LEN]);
int32_t  tf_unpack(const uint8_t in[TF_LEN], TELEM_FIELDS * f); /* 0 = ok */

#endif /* TELEM_FRAME_H */

/*** end of file ***/
