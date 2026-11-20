/*
 *      bypass_geom.h
 *      Obstacle bypass geometry: the 50 deg side-step (FR11, 11 Nov).  Pure
 *      integer functions, no kernel calls, plain <stdint.h>: tested on a
 *      laptop (tests/host/test_logic.c, worked examples for 10 / 15 / 20 cm
 *      boxes on both sides).
 *
 *      Frame: origin = wheel axle centre when the bypass starts, along =
 * forward along the line, lat = sideways (+ = left).  The Supervisor passes
 *      lat_s = lat x side (+ = towards the bypass side) as 'lat_mm'.
 *      All distances in mm, ranges r_cm[] in cm, look i at 5 x i deg.
 */

#ifndef BYPASS_GEOM_H
#define BYPASS_GEOM_H

#include <stdint.h>
#include "../hw_alloc.h"

#define BG_TURN_DEG 50 /* every bypass pivot */
/* Why step 1 is short and step 2 is computed: when the car straightens
   after an out-step, its front corner on the obstacle side swings forward
   by up to its radius.  For a 20 cm box 2 cm off-centre that is safe only
   while 0.643 t + corner < xf - 20 (t = total diagonal: ~170 mm left,
   ~145 mm right) or once the car is beside the box, 0.766 t - half width
   >= edge + 20 (~280 / ~320 mm).  In between it could clip the box.  So
   step 1 is the longest step in the first band, scan 1 measures the edge
   from there, and step 2 goes straight to edge + need, always in the
   second band.  Re-check this before raising BG_STEP1_MAX_MM. */
#define BG_STEP1_MAX_MM 150 /* longest step 1 (see above) */
#define BG_STEP1_MIN_MM 50  /* shorter: stopped too close, fail */
/* front corner to the face while
   straightening after step 1 */
#define BG_FACE_MARGIN_MM 30
/* front-right corner radius from the
   axle: sqrt(132^2 + 75^2) */
#define BG_CORNER_R_MM 152
/* front-left corner (barcode sensor):
   sqrt(132^2 + 105^2) */
#define BG_CORNER_L_MM  169
#define BG_STEP2_MIN_MM 40 /* step 2 is never shorter */
/* largest obstacle edge from the line:
   20 cm wide -> 100, + 2 cm off-centre */
#define BG_E_MAX_MM 120
/* Obstacle depth along the line assumed for the pass (the car drives past
   the back face plus the turn room).  Set from the PC ("depth <mm>").
   12 Nov: 200 -> 140, the practice box (9 cm wide, 14 cm deep): the car
   went ~15 cm past its back face.  Shallower than the real box = the
   car's side comes closer to its back corner on the diagonal back to the
   line (0.77 mm of the 73 mm clearance per mm). */
#define BG_DEPTH_DEFAULT_MM 140 /* after power-up */
#define BG_DEPTH_MIN_MM     10  /* the PC may set this range */
#define BG_DEPTH_MAX_MM     300
#define BG_TURN_CLEAR_MM    30   /* extra margin on the pass length */
#define BG_DIST_MAX_MM      1023 /* = MC_DIST_MAX_MM (msg_types.h) */
#define BG_SIDE_RANGE_CM    40   /* side probe: at or beyond = nothing */
/* a reading is "the face" within this
   of the expected forward distance */
#define BG_FACE_TOL_MM   50
#define BG_NEED_LEFT_MM  (BYPASS_MARGIN_MM + CAR_HALF_RIGHT_MM) /* 125 */
#define BG_NEED_RIGHT_MM (BYPASS_MARGIN_MM + CAR_HALF_LEFT_MM)  /* 155 */

/* Position relative to the stop, rotated into the line frame.  h0_deg =
   encoder heading at the stop. */
void bg_pose(int32_t dx_mm, int32_t dy_mm, int32_t h0_deg, int32_t * along_mm,
             int32_t * lat_mm);
/* Step 1 length; -1 = stopped too close to turn.  corner_mm =
   BG_CORNER_R_MM for a left bypass, BG_CORNER_L_MM for right. */
int32_t bg_step1_mm(int32_t xf_mm, int32_t corner_mm);
/* Scan 1: obstacle near edge from the line, bypass side.  f_mm = sensor
   to face, forward. */
int32_t bg_edge_mm(const uint8_t r_cm[7], int32_t lat_mm, int32_t f_mm);
/* Step 2 length: lat_s ends at edge + need */
int32_t bg_step2_mm(int32_t edge_mm, int32_t need_mm, int32_t lat_mm);
/* Scan 2 and later: 1 = something in the corridor the car drives through */
int bg_blocked(const uint8_t r_cm[7], int32_t need_mm);
/* Pass length: axle past the back face plus the room to turn back, 0..1023.
   depth_mm = obstacle depth assumed (BG_DEPTH_DEFAULT_MM unless the PC
   set another). */
int32_t bg_pass_mm(int32_t xf_mm, int32_t edge_mm, int32_t need_mm,
                   int32_t lat_mm, int32_t along_mm, int32_t depth_mm);
/* Axle travel at 50 deg until the front line sensors reach the line */
int32_t bg_line_mm(int32_t lat_mm);

#endif /* BYPASS_GEOM_H */

/*** end of file ***/
