/*
 *      subsystem4.h
 *      Subsystem 4 -- Obstacle Scan: HC-SR04 + SG90 servo.
 *      Owner: Lui Khee (Spatial Optimizer).  Public interface only.  Plan
 * section 9.
 *
 *      Inputs   mbf_scan_req (SCAN_REQ: mode decides who may stop the car)
 *      Outputs  OBST_STOP (sets AND clears it: the only clearer),
 *               US_FAULT, EV_OBST_DETECT, EV_OBST_AHEAD, EV_OBST_PROFILE,
 *               EV_OBST_SIDE, EV_OBST_NEAR
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../msg_types.h"

IMPORT ID tid_obstacle;

/* Distances, cm (LK-1.10, revised names: fix #9) */
/* 11 Nov: stop 15 cm from the car's front = reading 21 cm (sensor 51 mm behind
 * the front). Keep D_EMERG_CM < D_DETECT_CM < D_CLEAR_HYST_CM. */
#define D_DETECT_CM     21 /* 2 of 3 readings below -> stop (was 10) */
#define D_EMERG_CM      6  /* 1 reading below -> stop at once */
#define D_CLEAR_HYST_CM 26 /* 3 readings above -> path clear (was 15) */
#define D_STOP_MIN_CM   20 /* FR9 target as a sensor reading */
#define US_FAIL_LIMIT   10 /* pings with no rising edge -> US_FAULT */

typedef enum
{
    US_OK = 0,
    US_NO_ECHO,
    US_NO_RISE
} US_RESULT;

/* Servo: +1 if a HIGHER servo angle points the sensor to the LEFT.
   PLACEHOLDER (LK-1.2): check by watching the sensor during a profile. */
#define SERVO_LEFT_SIGN (+1)

/* Smooth servo moves: SERVO_RAMP_DEG every SERVO_RAMP_MS, one step per
   servo PWM frame (drv_servo.h SERVO_FRAME_MS).  16 ms steps at 50 Hz
   (until 13 Nov) put one step in some frames and two in others, so the
   servo moved in uneven jumps.  At 100 Hz: 1 deg / 10 ms = 100 deg/s for
   every move (the bypass pass guard's turn to 30 deg takes 0.3 s, as with
   2 deg / 20 ms before).  Works with SERVO_FRAME_MS from drv_servo.h. */
#define SERVO_RAMP_DEG 1
#define SERVO_RAMP_MS  SERVO_FRAME_MS
/* Width sweep: the same 1 deg steps, and a longer wait after each move so
   the sensor has stopped shaking before it pings.  About 5 s per sweep. */
#define WS_RAMP_DEG  1   /* deg per frame: 100 deg/s at 100 Hz */
#define WS_SETTLE_MS 150 /* after each move, before the ping */

/* Servo scans.  Settle = wait after the ramp has arrived, so the sensor
   stops shaking before it pings. */
#define SCAN_COARSE_SETTLE_MS 150
#define SCAN_FINE_SETTLE_MS   60
#define SCAN_PING_GAP_MS      30 /* between the two pings of one look */

/* servo never goes further than this
   from straight ahead (13 Nov: 30 -> 35,
   the mount now holds at 35).  Only the
   width sweep goes past 30. */
#define SCAN_SIDE_MAX_DEG 35

/* Width sweep (12 Nov, subsystem5/obst_width.h).  In GUARD, 2 of 3
   readings under D_WIDTH_CM send EV_OBST_AHEAD once; the Supervisor stops
   the car and asks for SCAN_PROFILE.  D_WIDTH_CM is about 35 cm so that both
   edges of a 20 cm box are inside the +-30 deg the servo can reach.  It is
   armed again after 3 readings over D_WIDTH_CLEAR_CM (or a SCAN_INHIBIT).
   Keep D_CLEAR_HYST_CM < D_WIDTH_CM < D_WIDTH_CLEAR_CM. */
#define D_WIDTH_CM       40
#define D_WIDTH_CLEAR_CM 45
#define WS_COARSE_DEG    5 /* first look each side: 5, 10 ... 30 deg */
/* straight-ahead reading beyond this = nothing there */
#define WS_FACE_MAX_MM 600
#define WS_NO_ECHO_MM  4000 /* us_ping_mm: no echo */
/* Pass along (LK-1.14) */
/* servo to the side.  Fixed at 30 (it was
   SCAN_SIDE_MAX_DEG): the bypass geometry
   and the pass guard are tuned for 30 */
#define SCAN_PASS_DEG 30
/* pass guard: closer than this
   sideways = too near (request param 0) */
#define PASS_SIDE_CM 35

typedef struct
{
    UW profiles; /* width sweeps started */
} OBST_STATS;
IMPORT volatile OBST_STATS obst_stats;

/* us_echo.c */
IMPORT ER        us_init(void);
IMPORT US_RESULT us_ping(UW * dist_cm);
IMPORT US_RESULT
us_ping_mm(UW * dist_mm); /* 12 Nov: width sweep, 4000 = none */
IMPORT volatile UW echo_isr_count;

/* Private to subsystem 4 */
IMPORT void      obstacle_task(INT stacd, void * exinf);
IMPORT SCAN_MODE obstacle_mode(void);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
