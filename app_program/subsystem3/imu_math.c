/*
 *      imu_math.c
 *      See imu_math.h.                                        Owner: Jian Yi
 */

#include "imu_math.h"

/* sin(k deg) * 16384, k = 0..90 */
static const uint16_t sin_tab[91] = {
    0,     286,   572,   857,   1143,  1428,  1713,  1997,  2280,  2563,  2845,
    3126,  3406,  3686,  3964,  4240,  4516,  4790,  5063,  5334,  5604,  5872,
    6138,  6402,  6664,  6924,  7182,  7438,  7692,  7943,  8192,  8438,  8682,
    8923,  9162,  9397,  9630,  9860,  10087, 10311, 10531, 10749, 10963, 11174,
    11381, 11585, 11786, 11982, 12176, 12365, 12551, 12733, 12911, 13085, 13255,
    13421, 13583, 13741, 13894, 14044, 14189, 14330, 14466, 14598, 14726, 14849,
    14968, 15082, 15191, 15296, 15396, 15491, 15582, 15668, 15749, 15826, 15897,
    15964, 16026, 16083, 16135, 16182, 16225, 16262, 16294, 16322, 16344, 16362,
    16374, 16382, 16384};

int32_t imu_sin_q14 (int32_t deg)
{
    deg %= 360;
    if (deg < 0)
    {
        deg += 360;
    }
    if (deg <= 90)
    {
        return sin_tab[deg];
    }
    if (deg <= 180)
    {
        return sin_tab[180 - deg];
    }
    if (deg <= 270)
    {
        return -(int32_t) sin_tab[deg - 180];
    }
    return -(int32_t) sin_tab[360 - deg];
}

int32_t imu_cos_q14 (int32_t deg)
{
    return imu_sin_q14(deg + 90);
}

/* atan(z), z in Q15 0..1  ->  q units 0..180 (0..45 deg).
   atan(z) ~ (pi/4) z + 0.273 z (1 - z) rad, error < 0.3 deg:
   in q units: 180 z + 62.6 z (1 - z). */
static int32_t atan_q_unit (int32_t z)
{
    int32_t a = (180 * z) >> 15;
    int32_t b = (int32_t) (((int64_t) z * (32768 - z)) >> 15); /* Q15 */
    return a + (int32_t) ((63 * (int64_t) b) >> 15);
}

int32_t imu_atan2_q (int64_t y, int64_t x)
{
    int64_t ax = (x < 0) ? -x : x;
    int64_t ay = (y < 0) ? -y : y;
    int32_t a;

    if (ax == 0 && ay == 0)
    {
        return 0;
    }
    if (ax >= ay)
    {
        a = atan_q_unit((int32_t) ((ay << 15) / ax));
    }
    else
    {
        a = 360 - atan_q_unit((int32_t) ((ax << 15) / ay));
    }
    if (x < 0)
    {
        a = 720 - a;
    }
    return (y < 0) ? -a : a;
}

uint32_t imu_isqrt (uint64_t v)
{
    uint64_t r = 0, bit = (uint64_t) 1 << 62;

    while (bit > v)
    {
        bit >>= 2;
    }
    while (bit != 0)
    {
        if (v >= r + bit)
        {
            v -= r + bit;
            r = (r >> 1) + bit;
        }
        else
        {
            r >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t) r;
}

int32_t imu_roll_q (const IMU_VEC * a)
{
    return imu_atan2_q(a->l, a->u);
}

int32_t imu_pitch_q (const IMU_VEC * a)
{
    uint64_t s =
        (uint64_t) ((int64_t) a->l * a->l) + (uint64_t) ((int64_t) a->u * a->u);
    return imu_atan2_q(a->f, (int64_t) imu_isqrt(s));
}

int32_t imu_heading_deg (const IMU_VEC * a, const IMU_VEC * m)
{
    int64_t ef, el, eu, nf, an;
    int32_t h;

    /* E = m x a */
    ef = (int64_t) m->l * a->u - (int64_t) m->u * a->l;
    el = (int64_t) m->u * a->f - (int64_t) m->f * a->u;
    eu = (int64_t) m->f * a->l - (int64_t) m->l * a->f;
    /* N = a x E, forward component only */
    nf = (int64_t) a->l * eu - (int64_t) a->u * el;
    an = (int64_t) imu_isqrt((uint64_t) ((int64_t) a->f * a->f +
                                         (int64_t) a->l * a->l +
                                         (int64_t) a->u * a->u));
    h  = imu_atan2_q(ef * an, nf); /* q units */
    h  = (h + (h < 0 ? 1440 : 0)) / IMU_Q_PER_DEG;
    return (h >= 360) ? h - 360 : h;
}

/*** end of file ***/
