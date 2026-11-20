/*
 *      obst_width.c
 *      Obstacle width from the width sweep (12 Nov), see obst_width.h.
 *      Integer only.  Q14 = x 16384 (imu_sin_q14 / imu_cos_q14).
 *      Worked examples are checked in tests/host/test_logic.c.
 */

#include <stddef.h>
#include "obst_width.h"
#include "../subsystem3/imu_math.h"

#define Q14 16384

/* sin / cos of an angle in 0.1 deg (t >= 0): straight line between the
   two whole-degree values either side (error < 0.005 %) */
static int32_t sin_t (int32_t t)
{
    int32_t d = t / 10, f = t % 10;
    int32_t a = imu_sin_q14(d), b = imu_sin_q14(d + 1);

    return a + (b - a) * f / 10;
}

static int32_t cos_t (int32_t t)
{
    int32_t d = t / 10, f = t % 10;
    int32_t a = imu_cos_q14(d), b = imu_cos_q14(d + 1);

    return a + (b - a) * f / 10;
}

int32_t ow_tan_q14 (int32_t tenth_deg)
{
    int32_t m = (tenth_deg < 0) ? -tenth_deg : tenth_deg;
    int32_t t;

    if (m > 890)
    {
        m = 890; /* tan is 57 x at 89 deg: far outside use */
    }
    t = (int32_t) (((int64_t) sin_t(m) * Q14) / cos_t(m));
    return (tenth_deg < 0) ? -t : t;
}

int ow_is_face (int32_t reading_mm, int32_t deg, int32_t face_mm)
{
    int32_t fwd, d;

    if (reading_mm <= 0 || face_mm <= 0)
    {
        return 0;
    }
    fwd = (int32_t) (((int64_t) (OW_PIVOT_MM + reading_mm) * imu_cos_q14(deg)) /
                     Q14);
    d   = fwd - (OW_PIVOT_MM + face_mm);
    if (d < 0)
    {
        d = -d;
    }
    return d <= OW_FACE_TOL_MM;
}

/* n / d rounded to the nearest integer, d > 0 */
static int64_t div_round (int64_t n, int64_t d)
{
    return (n >= 0) ? (n + d / 2) / d : -((-n + d / 2) / d);
}

/* tan of a boundary in half-degrees, x 10000 */
static int64_t tan_e4 (int32_t half)
{
    return div_round((int64_t) ow_tan_q14(5 * half) * 10000, Q14);
}

int32_t ow_edge_mm (int32_t face_mm, int32_t e_half, int32_t s_e4, int32_t k_e4)
{
    int64_t f = OW_PIVOT_MM + face_mm;

    if (face_mm <= 0 || k_e4 <= 0)
    {
        return 0;
    }
    return (int32_t) div_round(f * (tan_e4(e_half) - s_e4), k_e4);
}

int32_t ow_width_mm (int32_t face_mm, int32_t el_half, int32_t er_half,
                     const OW_CAL * c)
{
    int32_t w;

    if (face_mm <= 0 || c->kl_e4 <= 0 || c->kr_e4 <= 0)
    {
        return -1;
    }
    w = ow_edge_mm(face_mm, el_half, c->sl_e4, c->kl_e4) +
        ow_edge_mm(face_mm, er_half, c->sr_e4, c->kr_e4);
    return (w > 0) ? w : -1;
}

int ow_bypass_side (int32_t face_mm, int32_t el_half, int32_t er_half,
                    int oov_l, int oov_r, const OW_CAL * c, int32_t * l_mm,
                    int32_t * r_mm)
{
    int32_t l = 0, r = 0;
    int     side = OW_GO_EITHER;

    if (face_mm > 0)
    {
        l = ow_edge_mm(face_mm, el_half, c->sl_e4, c->kl_e4);
        r = ow_edge_mm(face_mm, er_half, c->sr_e4, c->kr_e4);
        if (oov_l && oov_r)
        {
            side = OW_GO_EITHER; /* wider than the sweep */
        }
        else if (oov_l)
        {
            side = OW_GO_RIGHT;
        }
        else if (oov_r)
        {
            side = OW_GO_LEFT;
        }
        else if (l - r > OW_SIDE_MARGIN_MM)
        {
            side = OW_GO_RIGHT; /* the box reaches further left */
        }
        else if (r - l > OW_SIDE_MARGIN_MM)
        {
            side = OW_GO_LEFT;
        }
    }
    if (l_mm != NULL)
    {
        *l_mm = l;
    }
    if (r_mm != NULL)
    {
        *r_mm = r;
    }
    return side;
}

int ow_fit_side (const OW_POINT * p, int n, int side, int32_t * s_e4,
                 int32_t * k_e4)
{
    /* y = S + K x, with x = edge / F and y = tan(boundary), both
       x 1000000 (|x|, |y| < 3e6, so the sums fit easily in 64 bits) */
    int64_t sx = 0, sy = 0, sxx = 0, sxy = 0, num, den, k;
    int32_t lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;
    int     i;

    if (n < 2)
    {
        return -1;
    }
    for (i = 0; i < n; i++)
    {
        int64_t f = OW_PIVOT_MM + p[i].face_mm;
        int32_t e = (side == OW_SIDE_LEFT) ? p[i].known_l_mm : p[i].known_r_mm;
        int32_t h = (side == OW_SIDE_LEFT) ? p[i].el_half : p[i].er_half;
        int64_t x, y;

        if (p[i].face_mm <= 0)
        {
            return -1;
        }
        x = div_round((int64_t) e * 1000000, f);
        y = tan_e4(h) * 100;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        if (e < lo)
        {
            lo = e;
        }
        if (e > hi)
        {
            hi = e;
        }
    }
    if (hi - lo < OW_CAL_MIN_SPREAD_MM)
    {
        return -1;
    }
    num = n * sxy - sx * sy;
    den = n * sxx - sx * sx;
    if (den <= 0)
    {
        return -1;
    }
    /* num < 8 x 8e12 here; x 10000 still fits in 64 bits */
    k = div_round(num * 10000, den);
    if (k <= 0)
    {
        return -1;
    }
    *k_e4 = (int32_t) k;
    /* S = (sum y - K sum x) / n, from x 1e6 to x 1e4 */
    *s_e4 = (int32_t) div_round(sy * 10000 - k * sx, (int64_t) n * 1000000);
    return 0;
}

/*** end of file ***/
