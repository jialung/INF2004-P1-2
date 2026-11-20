/*
 *	subsystem3.h
 *	Subsystem 3 -- IMU: heading, tilt, humps, impact.
 *	Owner: Jian Yi (Physics Watchdog).  Public interface only.  Plan section 8.
 *
 *	Outputs  nav_state IMU half (heading, roll, pitch, terrain),
 *	         EV_HUMP_CLIMB, EV_HUMP_LEVEL, EV_CALIB_DONE (CAL_WHO_IMU),
 *	         IMU_SENSOR and IMPACT bits
 *	Inputs   I2C1 (sole owner), nav_state speed + acceleration, flg_calib
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

IMPORT ID tid_imu;

/* Terrain / hump statechart state (3 bits, JY-1.11; Design Review 6.3) */
typedef enum {
	TER_LEVEL = 0,
	TER_CLIMBING,		/* |tilt| > enter threshold for N samples */
	TER_SENSOR_FAULT,
	TER_STATE_COUNT
} TERRAIN_STATE;

/* Thresholds (chain 4.4), in 0.25 deg units, on the SMOOTHED roll.
   30 Oct, flat track: the raw roll passed 3 deg for 3 samples four times
   in 70 cm while driving (motor vibration, steering wobble), giving false
   humps of 13-20 mm.  So: smooth first, then a higher, longer test.  A
   real one-wheel hump at ~120 mm/s takes several hundred ms, so 120 ms
   is still early enough to slow down. */
#define HUMP_FILTER_SHIFT	2	/* roll_f += (roll - roll_f) / 4 per 20 ms
					   sample: time constant ~80 ms (log only) */

/* ---- Hump height (5 Nov): one wheel, always the RIGHT one ------------
 * The two wheel contacts stay TRACK_WIDTH_MM (W) apart, so with the right
 * wheel raised by h the axle line tilts by h / W, and gravity's share
 * along the car's left axis is exactly
 *         s = a_l / |a| = -h / W      ->      h = W x (s_ref - s)
 * (a_l: lateral acceleration, mg; |a|: the measured gravity vector, so
 * the sensor's gain error cancels).  Every 20 ms:
 *   1. a_l minus the car's own sideways acceleration when steering
 *      (v x omega, both from the wheel encoders)            HUMP_STEER_COMP
 *   2. s = a_l / |a| in Q14 (16384 = 1 g)
 *   3. h = W x (s_ref - s), 0.1 mm.  s_ref = the calibrated level
 *      (imu_cal.lat0_q14), then slowly follows the floor while level, so
 *      a slight slope where the car was calibrated does not add height
 *      (1 deg of slope = 2 mm).  Frozen while the wheel is going up.
 *   4. median of the last HUMP_MED_N values: removes vibration spikes
 *      without flattening the hump's flat top
 *   5. Level -> Climbing: h > HUMP_ENTER_X10 for HUMP_ENTER_N samples;
 *      Climbing -> Level (hump REGISTERED): |h| <= HUMP_LEVEL_X10, i.e.
 *      back at the calibrated level, for HUMP_EXIT_N samples
 *   6. height = the HIGHEST mean of any HUMP_AVG_N consecutive samples
 *      (the maximum of a moving average), 7 Nov.  A traverse is a
 *      trapezoid plus rocking: ramp up, flat top, ramp down, then a
 *      negative swing as the wheel drops off the far side.  The flat
 *      top is the only part that stays high for a whole window, so the
 *      highest window mean lands on it: the ramps are too short to fill
 *      one, and the negative swing can only pull a mean DOWN, so
 *      neither can set the maximum.  Averaging also removes the rocking
 *      the median of step 4 leaves behind (~1/sqrt(N) of it).
 * Practice hump: ~10 mm, flat top -> ~5.1 deg.  The old 5.0 deg trigger
 * would only just have caught it, hence thresholds in mm now.
 * Two earlier methods and why they failed on the 10 mm practice hump:
 *   - mean of the samples within 1 mm of the HIGHEST single sample:
 *     19.4 mm, because "highest" was the top of the rocking swing;
 *   - median of EVERY on-hump sample (6 Nov): 7.7 mm, because the two
 *     ramps and the exit swing are averaged in with the top.
 * The single highest sample read 11.9 mm on that run, i.e. the true
 * 10 mm plus the ~1.9 mm of rocking it happened to sit on.             */
/* 1: only a RIGHT-wheel-up tilt counts.  0: either side (the log and the
   UI say which).  6 Nov: back to 1.  With the axes corrected, a spacer
   under the right wheel reads POSITIVE (checked), and the course only
   has right-wheel humps; 0 let the wheel dropping off the hump (swings
   down to -15 mm) count as a "left" hump. */
#define HUMP_RIGHT_ONLY		1
/* Lever arm L in h = L x sin(tilt), mm: the track width W (113), the car
   pivoting on the left wheel.  5 Nov this was set to 61 from a 10 mm
   hump reading 18.6 mm, but that reading came from the wrong (pitch)
   axis (IMU_AXIS_* above), so it says nothing about the lever: back to
   the geometry.  If the car rocks on two casters instead (left wheel
   lifts when the right one goes up), the true value is ~half of 113.
   Fine-tune: in Idle, put a spacer of known height under the RIGHT
   wheel and read "[imu] idle: ... right wheel X mm";
       new HUMP_LEVER_MM = HUMP_LEVER_MM x spacer / X
   6 Nov, parked, right wheel on the 10 mm hump (left wheel stays down):
   2 s means 13.6..15.2, ~14.4 mm; same spot off the hump ~2.7 mm.  So
   10 mm reads 11.7 mm: 113 x 10 / 11.7 = 97.  (The wheel sits on the
   hump's edge / the tyre squashes: the lean is a bit more than 10/113.) */
#define HUMP_LEVER_MM		97
#define HUMP_STEER_COMP		1	/* subtract v x omega from a_l */
#define HUMP_MED_N		5	/* median window, samples (100 ms) */
/* Moving-average window for the height (step 6 above).  It must be
   SHORTER than the time the raised wheel spends on the flat top: a
   window that straddles a ramp reads low.  10 samples = 200 ms, and by
   then the Supervisor has already slowed the car to SPD_HUMP (50 %,
   ~60 mm/s), so the window spans only ~12 mm of travel while the 10 mm
   practice hump's top is far longer than that.
   Larger N: less noise, but needs a longer flat top.  Smaller N: the
   other way round.  Simulated traverses of a 10 mm hump (trapezoid plus
   rocking, through the same 5-median), worst error over the runs:
       N =  5   +1.1 mm on a rough surface, -0.5 on a short top
       N = 10   +0.6 mm on a rough surface, -0.9 on a short top
       N = 20   +0.2 mm on a rough surface, -0.9 on a short top
   10 is the middle: raise it if the track is rough, lower it if the log
   reports "TOO SHORT" on a real hump.  That marker means no window ever
   sat on the flat top, and the single highest sample was used instead. */
#define HUMP_AVG_N		10	/* moving-average window, samples (200 ms) */
#define HUMP_REF_SHIFT		5	/* s_ref follows the floor, ~640 ms */
#define HUMP_REF_HOLD_N		50	/* frozen at most this long (1 s), then
					   it is a slope, not a hump */
/* Enter: h above HUMP_ENTER_X10 for HUMP_ENTER_N samples AND over
   HUMP_ENTER_TRAVEL_MM of travel.  5 Nov: bumps under 5 mm on the flat
   track were taken as humps (they read ~2x too high, see HUMP_LEVER_MM,
   and were long enough in TIME at hump speed).  7 mm sits between the
   track's < 5 mm bumps and the 10 mm hump; the travel test rejects short
   ridges, since the hump's ramp + flat top is far longer. */
/* 6 Nov: 7.0 -> 5.5 mm.  On the flat part of the run the tilt stayed
   within -2.9 .. +4.1 mm; the hump read -6.7 / -7.9 mm, too close to 7. */
#define HUMP_ENTER_X10		55	/* 5.5 mm */
#define HUMP_ENTER_N		4	/* 80 ms ... */
#define HUMP_ENTER_TRAVEL_MM	10	/* ... and 10 mm of travel above it
					   (5 Nov: 15 missed the hump) */
/* 1: "[imu] hump watch" once a second while following the line: the
   right wheel height now, min / max over the last second, against the
   trigger.  Drive over the hump and read it to tune the numbers above. */
#define HUMP_DEBUG		1
#define HUMP_LEVEL_X10		20	/* 2.0 mm either side of the level */
#define HUMP_EXIT_N		5	/* 100 ms at level */
#define HUMP_MAX_MM		500	/* climbing this far without reaching level:
					   register the hump anyway, so the car
					   does not stay slow (HumpTraverse) */
#define IMU_FAIL_LIMIT		3	/* failed reads -> IMU_SENSOR (JY-1.5) */

/* Sensor axes -> car frame.  axis: 0 = sensor X, 1 = Y, 2 = Z; sign +1/-1.
   6 Nov, mount as built (team): sensor Y points BACKWARDS, sensor X
   points to the LEFT, so
       forward = -Y      left = +X      up = +Z   (X x Y = left x back = up)
   Was: forward = X, left = Y (the placeholder "X forward, Y left"), which
   swapped the two tilt axes: "roll" was really pitch and the other way
   round.  So the hump code measured the car pitching (caster over a
   bump), not one wheel going up, and the car's forward acceleration was
   taken off the sideways axis.  The 29 Oct bench result ("left side
   lifted gave roll -17..-23 deg", fixed with an extra accelerometer
   sign) fits that swap, so the extra sign is gone: the accelerometer
   and the compass now use the same axes.
   Check after flashing (A4): car flat -> "[imu] idle" roll and pitch
   near 0; lift the LEFT side -> roll +; lift the NOSE -> pitch +; spacer
   under the RIGHT wheel -> "right wheel X mm" +; turn the car RIGHT
   (clockwise from above) -> heading goes up.  Fix signs here.          */
#define IMU_AXIS_FWD		1
#define IMU_SIGN_FWD		(-1)
#define IMU_AXIS_LEFT		0
#define IMU_SIGN_LEFT		(+1)
#define IMU_AXIS_UP		2
#define IMU_SIGN_UP		(+1)
/* Extra sign for the ACCELEROMETER left axis only (was -1, see above) */
#define IMU_ACC_SIGN_LEFT	(+1)

/* Impact (JY-1.12): horizontal acceleration, mg, this many samples */
#define IMPACT_MG		2000	/* PLACEHOLDER: tune on the bench */
#define IMPACT_N		2
#define IMPACT_HOLDOFF_MS	300	/* after a hump: landing is not a hit */

/* Calibration (JY-1.13) */
#define IMU_CAL_LEVEL_MS	1000	/* still and flat */
#define IMU_CAL_MS		15000	/* total, incl. the full turn */
#define IMU_CAL_MAG_SPAN	200	/* counts on f and l, else keep old */

/* 0 (default): a missing / wrong IMU at boot is logged and the car runs
   without it (no compass check, no humps).  1: it is an INIT_FAIL. */
#define IMU_REQUIRED		0

typedef struct {
	H	roll0_q, pitch0_q;	/* level offsets, 0.25 deg */
	H	mag_off_f, mag_off_l, mag_off_u;	/* hard iron, car frame */
	UB	mag_ok;			/* compass calibrated */
	H	lat0_q14;		/* level: a_l / |a| at rest, Q14 (hump) */
} IMU_CAL;
IMPORT volatile IMU_CAL	imu_cal;
IMPORT volatile UB	imu_peak_mm;	/* highest hump (FR8) */

/* Private to subsystem 3 */
IMPORT void imu_task(INT stacd, void *exinf);
IMPORT INT  imu_driver_status;

#endif /* SUBSYSTEM3_H */
