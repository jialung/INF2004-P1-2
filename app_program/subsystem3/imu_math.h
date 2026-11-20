/*
 *	imu_math.h
 *	Tilt and tilt-compensated compass heading, integer only (no libm,
 *	no float on the M0+).  Plain <stdint.h>: tested on a laptop.
 *	                                                       Owner: Jian Yi
 *	Plan refs: JY-1.6..JY-1.8.
 *
 *	All vectors are in the CAR frame: f = forward, l = left, u = up
 *	(right-handed).  The accelerometer reads +1 g on u when the car is
 *	level and still.
 *	  roll  = atan2(a_l, a_u)                 + = left side up
 *	  pitch = atan2(a_f, sqrt(a_l^2 + a_u^2)) + = nose up
 *	  heading: E = m x a (East), N = a x E (North), both in the car frame;
 *	           heading = atan2(E_f * |a|, N_f), clockwise from magnetic
 *	           north, 0..359.  No trig: the cross products remove the
 *	           tilt, so the car can be on a hump.
 *	Angles in "q" units = 0.25 degree.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

#define IMU_Q_PER_DEG		4

typedef struct { int32_t f, l, u; } IMU_VEC;

int32_t  imu_atan2_q(int64_t y, int64_t x);	/* -720..720 (+-180 deg) */
uint32_t imu_isqrt(uint64_t v);
int32_t  imu_roll_q(const IMU_VEC *a);
int32_t  imu_pitch_q(const IMU_VEC *a);
int32_t  imu_heading_deg(const IMU_VEC *a, const IMU_VEC *m);	/* 0..359 */

/* sin / cos of a whole-degree angle (any value), Q14 (16384 = 1.0) */
int32_t  imu_sin_q14(int32_t deg);
int32_t  imu_cos_q14(int32_t deg);

#endif /* IMU_MATH_H */
