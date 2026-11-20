/*
 *      bypass_geom.c
 *      Obstacle bypass geometry (FR11, 11 Nov), see bypass_geom.h.
 *      Integer only.  Q14 = x 16384 (imu_sin_q14 / imu_cos_q14).
 *      Worked examples are checked in tests/host/test_logic.c.
 */

#include "bypass_geom.h"
#include "../subsystem3/imu_math.h"

#define Q14 16384

/* ceil(a / b) for a >= 0, b > 0 */
static int32_t ceil_div (int32_t a, int32_t b)
{
    return (a + b - 1) / b;
}

void bg_pose (int32_t dx_mm, int32_t dy_mm, int32_t h0_deg, int32_t * along_mm,
              int32_t * lat_mm)
{
    int64_t s = imu_sin_q14(h0_deg), c = imu_cos_q14(h0_deg);

    *along_mm = (int32_t) (((int64_t) dx_mm * c + (int64_t) dy_mm * s) / Q14);
    *lat_mm   = (int32_t) ((-(int64_t) dx_mm * s + (int64_t) dy_mm * c) / Q14);
}

int32_t bg_step1_mm (int32_t xf_mm, int32_t corner_mm)
{
    int32_t v = (xf_mm - BG_FACE_MARGIN_MM - corner_mm) * Q14 /
                imu_cos_q14(BG_TURN_DEG);

    if (v < BG_STEP1_MIN_MM)
    {
        return -1;
    }
    return (v > BG_STEP1_MAX_MM) ? BG_STEP1_MAX_MM : v;
}

/* Is look i (at 5 x i deg) on the obstacle face, f_mm ahead of the sensor? */
static int is_face (const uint8_t r_cm[7], int i, int32_t f_mm)
{
    int32_t fwd, d;

    if (r_cm[i] >= BG_SIDE_RANGE_CM)
    {
        return 0;
    }
    fwd = (int32_t) r_cm[i] * 10 * imu_cos_q14(5 * i) / Q14;
    d   = fwd - f_mm;
    if (d < 0)
    {
        d = -d;
    }
    return d <= BG_FACE_TOL_MM;
}

/* lat - f x tan(a), not below 0 */
static int32_t lat_minus_tan (int32_t lat_mm, int32_t f_mm, int a_deg)
{
    int32_t e = lat_mm - f_mm * imu_sin_q14(a_deg) / imu_cos_q14(a_deg);

    return (e < 0) ? 0 : e;
}

int32_t bg_edge_mm (const uint8_t r_cm[7], int32_t lat_mm, int32_t f_mm)
{
    int k;

    if (f_mm <= 0)
    {
        return BG_E_MAX_MM;
    }
    if (is_face(r_cm, 0, f_mm))
    {
        return (lat_mm > BG_E_MAX_MM) ? lat_mm : BG_E_MAX_MM;
    }
    for (k = 1; k <= 6; k++)
    {
        if (is_face(r_cm, k, f_mm))
        {
            /* the last look that missed: the cautious end */
            return lat_minus_tan(lat_mm, f_mm, 5 * (k - 1));
        }
    }
    return lat_minus_tan(lat_mm, f_mm, 30);
}

int32_t bg_step2_mm (int32_t edge_mm, int32_t need_mm, int32_t lat_mm)
{
    int32_t x = edge_mm + need_mm - lat_mm, d;

    if (x <= 0)
    {
        return BG_STEP2_MIN_MM;
    }
    d = ceil_div(x * Q14, imu_sin_q14(BG_TURN_DEG));
    return (d < BG_STEP2_MIN_MM) ? BG_STEP2_MIN_MM : d;
}

int bg_blocked (const uint8_t r_cm[7], int32_t need_mm)
{
    int i;

    for (i = 0; i < 7; i++)
    {
        if (r_cm[i] < BG_SIDE_RANGE_CM &&
            (int32_t) r_cm[i] * 10 * imu_sin_q14(5 * i) / Q14 < need_mm)
        {
            return 1;
        }
    }
    return 0;
}

int32_t bg_pass_mm (int32_t xf_mm, int32_t edge_mm, int32_t need_mm,
                    int32_t lat_mm, int32_t along_mm, int32_t depth_mm)
{
    int32_t g = lat_mm - edge_mm;
    int32_t t = need_mm * Q14 - g * imu_cos_q14(BG_TURN_DEG);
    int32_t c = ((t > 0) ? ceil_div(t, imu_sin_q14(BG_TURN_DEG)) : 0) +
                BG_TURN_CLEAR_MM;
    int32_t p;

    if (depth_mm < BG_DEPTH_MIN_MM)
    {
        depth_mm = BG_DEPTH_MIN_MM;
    }
    if (depth_mm > BG_DEPTH_MAX_MM)
    {
        depth_mm = BG_DEPTH_MAX_MM;
    }
    p = xf_mm + depth_mm + c - along_mm;

    if (p < 0)
    {
        return 0;
    }
    return (p > BG_DIST_MAX_MM) ? BG_DIST_MAX_MM : p;
}

int32_t bg_line_mm (int32_t lat_mm)
{
    return lat_mm * Q14 / imu_sin_q14(BG_TURN_DEG) - SENSOR_TO_AXLE_MM;
}

/*** end of file ***/
