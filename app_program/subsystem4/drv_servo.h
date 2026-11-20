/*
 *      drv_servo.h
 *      SG90 scan servo on GP15, PWM slice 7B, 50 Hz.  Owner: Lui Khee.
 *      SDK side via hal.h only.  Plan refs: LK-1.2, LK-1.11.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

#define SERVO_AHEAD_DEG 90U
#define SERVO_MIN_US    500U /* TODO LK-1.2: check the end stops */
#define SERVO_MAX_US    2500U

/* Servo signal rate (13 Nov: 50 -> 100 Hz).  The servo takes a new
   position once per frame, so this sets how fast 1 deg steps can go:
   50 Hz = 1 deg / 20 ms (50 deg/s), 100 Hz = 1 deg / 10 ms (100 deg/s).
   The pulse widths are the same at either rate.  If the servo buzzes,
   jitters while holding still or gets warm, set this back to 50. */
#define SERVO_PWM_HZ   100U
#define SERVO_FRAME_MS (1000U / SERVO_PWM_HZ)

/* Straight-ahead trim (13 Nov), microseconds added to EVERY pulse, so the
   whole scan turns with it.  About 11 us per degree.  The servo has no
   memory: at power-up it goes to the pulse it is sent, wherever the horn
   was left by hand, and the horn only fits on its splines in ~14 deg
   steps, so a small tilt is trimmed here.  + = longer pulse.  Measured
   on the car; re-measure if the sensor or the horn is remounted. */
/* set on the car 20 Nov (about 7 deg; was 95) */
#define SERVO_TRIM_US     80
#define SERVO_TRIM_MAX_US 200 /* +-18 deg: more = refit the horn */

/* End-stop calibration (19 Nov), microseconds added at each side's
   end (SCAN_SIDE_MAX_DEG, subsystem4.h), scaled down linearly to 0
   straight ahead, so the trim above still sets straight ahead and each
   side's travel is set on its own.  + = further out on that side.
   0 = the plain 500..2500 us scale (no change).  The obstacle task applies
   them (task_obstacle.c servo_raw). */
/* set on the car 19 Nov: body blocked the old end */
#define SERVO_END_LEFT_US  (-50)
#define SERVO_END_RIGHT_US (-70) /* set on the car 20 Nov (was -20) */
#define SERVO_END_MAX_US   150   /* +-13 deg at the end */

int32_t drv_servo_init(void);
/* 0 = right, 90 = ahead, 180 = left */
void drv_servo_set_deg(uint32_t deg);
/* As drv_servo_set_deg, plus adj_us on the pulse.  Returns the pulse sent, us.
 */
uint32_t drv_servo_set_deg_adj(uint32_t deg, int32_t adj_us);

#endif /* DRV_SERVO_H */

/*** end of file ***/
