/*
 *      obst_width.h
 *      Obstacle width from the width sweep (12 Nov).  Integer only, plain
 *      <stdint.h>: tested on a laptop (tests/host).
 *
 *      The car stops about 40 cm from the obstacle and the servo swings the
 *      ultrasonic sensor sideways from straight ahead, up to +-35 deg.  A look
 *      is a "hit" while the reading is the obstacle's flat face (its forward
 *      distance is the same as straight ahead), a "miss" once the beam has
 *      left the box.
 *
 *        face   F = OW_PIVOT_MM + face reading (mm): from the servo axis
 *        edge   one side's hit / miss boundary a, in half-degrees:
 *               (last hit deg) + (first miss deg)
 *
 *      The HC-SR04 beam is wide, so the boundary is far outside the real edge,
 *      and it moves less than the edge does (12 Nov: a 53 mm and a 96 mm box
 *      both gave boundaries of 20 to 25 deg).  Each SIDE has its own two
 *      constants (13 Nov), because the sensor mount and the servo are not
 *      symmetric:
 *               tan(a) = S + K x edge / F
 *               edge   = F x (tan(a) - S) / K       (signed, + = outwards)
 *               width  = left edge + right edge
 *        edge = the distance from the car's centre line to that edge of the
 *               box, + = out on its own side (negative: past the line).
 *        S = the boundary's tan for an edge on the line (the beam spread),
 *        K = how fast it grows with edge / F (1 = a perfect narrow beam).
 *        Both x 10000.
 *
 *      Calibrate from the PC ("Edge calibration" window, or "wcal <left mm>
 *      <right mm>" in Idle): put a box in front, measure where each of its
 *      edges is from the centre line, one run per position.  Shifting the
 *      same box sideways gives each side different edge positions, so one
 *      box is enough.  The car fits S and K per side over every run since
 *      power-up ("wcal clear" starts again) and prints them to copy here.
 */

#ifndef OBST_WIDTH_H
#define OBST_WIDTH_H

#include <stdint.h>

/* ---- tuning ------------------------------------------------------------ */
/* Per side.  20 Nov, ONE-POINT CORRECTION, not a fit: trim 80 us, ends
   -50 / -70 us, a 120 mm box centred (edges 60 / 60 mm), face 394 mm,
   boundaries left 30.5 / right 29.5 deg in 3 of 4 runs (run 1: left 33.5).
   K kept from the 12 Nov fit; S moved so those runs read 60 + 60 mm.
   Only that box at that distance is checked: other widths are not, and
   the 20 Nov sweeps suggest the boundary hardly follows the edge.  For a
   real per-side fit, shift the box sideways so each edge moves 80 mm or
   more.  Before: S 3455 / K 6079 both sides (the 12 Nov two-sided fit
   split evenly). */
#define OW_SL_E4 4992 /* left:  S x 10000 */
#define OW_KL_E4 6079 /* left:  K x 10000 */
#define OW_SR_E4 4760 /* right: S x 10000 */
#define OW_KR_E4 6079 /* right: K x 10000 */
/* Servo axis to the transducer face, mm.  Measured: 12. */
#define OW_PIVOT_MM 12
/* A reading is the box face when its forward distance is this close to
   the straight-ahead one, mm. */
#define OW_FACE_TOL_MM 40

#define OW_CAL_MAX_POINTS 8 /* calibration runs kept for the fit */
/* A side's fit needs its known edge positions this far apart.  1 deg of
   boundary noise is ~7 mm of edge at 40 cm; over a 22 mm spread that
   wrecked the 12 Nov per-side fit. */
#define OW_CAL_MIN_SPREAD_MM 40

/* One calibration run: the sweep and where the box's edges really were */
typedef struct
{
    int32_t face_mm;          /* straight-ahead reading */
    int32_t el_half, er_half; /* boundaries, half-degrees */
    /* edges from the centre line, + = outwards */
    int32_t known_l_mm, known_r_mm;
} OW_POINT;

/* The calibration in use: S and K per side, x 10000 */
typedef struct
{
    int32_t sl_e4, kl_e4; /* left */
    int32_t sr_e4, kr_e4; /* right */
} OW_CAL;

#define OW_SIDE_LEFT  0
#define OW_SIDE_RIGHT 1

/* Bypass side (20 Nov): go round on the side with more room.  The box
   reaches further out on one side when that side's edge is more than
   this further from the line than the other.  Less = centred: no
   preference (the Supervisor then uses BYPASS_SIDE_DEFAULT).  1 deg of
   boundary noise is ~7 mm of edge at 40 cm, on each side. */
#define OW_SIDE_MARGIN_MM 20
#define OW_GO_LEFT        (+1)
#define OW_GO_RIGHT       (-1)
#define OW_GO_EITHER      0

/* tan of an angle in 0.1 deg (any sign, |t| < 900), Q14 (16384 = 1.0) */
int32_t ow_tan_q14(int32_t tenth_deg);

/* Is reading_mm (a reading at deg from straight ahead, from the transducer)
   the obstacle face, whose straight-ahead reading was face_mm?  0 = no. */
int ow_is_face(int32_t reading_mm, int32_t deg, int32_t face_mm);

/* One edge's distance from the centre line, mm, + = outwards on its own
   side (negative: the edge is past the line).  e_half: its boundary.
   0 if the inputs are unusable. */
int32_t ow_edge_mm(int32_t face_mm, int32_t e_half, int32_t s_e4, int32_t k_e4);

/* Width in mm (left edge + right edge), or -1 if it is not positive or
   the inputs are unusable (an edge out of view is the caller's check). */
int32_t ow_width_mm(int32_t face_mm, int32_t el_half, int32_t er_half,
                    const OW_CAL * c);

/* Which side to go round, from one width sweep: OW_GO_LEFT, OW_GO_RIGHT
   or OW_GO_EITHER (centred within OW_SIDE_MARGIN_MM, both edges out of
   view, or no face).  oov_l / oov_r: that edge was still on the box at the
   servo limit, so the box reaches far out on that side: go the other
   way.  *l_mm / *r_mm (may be NULL): each edge from the line, + = out on
   its own side, as ow_edge_mm (for an edge out of view: the least it is). */
int ow_bypass_side(int32_t face_mm, int32_t el_half, int32_t er_half, int oov_l,
                   int oov_r, const OW_CAL * c, int32_t * l_mm, int32_t * r_mm);

/* Least-squares S and K of one side (OW_SIDE_*) over n calibration runs.
   0 = done, -1 = not possible (fewer than 2 runs, that side's known edges
   less than OW_CAL_MIN_SPREAD_MM apart, or a fit with K <= 0). */
int ow_fit_side(const OW_POINT * p, int n, int side, int32_t * s_e4,
                int32_t * k_e4);

#endif /* OBST_WIDTH_H */

/*** end of file ***/
