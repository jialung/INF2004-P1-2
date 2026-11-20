/*
 *      barcode_decode.c
 *      Code 39 decoder (see barcode_decode.h).                     Owner: Ryan
 */

#include "barcode_decode.h"

/* The Code 39 table is built, not typed in, so a typo cannot hide in
 * 27 nine-digit strings.  Each character = a bar pattern (5 bars, 2 wide)
 * plus one wide space:
 *   column (bar pattern) 0..9, the same for 1-9,0 / A-J / K-T / U-Z,*
 *   row A-J: 3rd space wide, row K-T: 4th, row U-Z and '*': 1st.
 * bar5 bit k = bar k (k = 0..4) is wide.                                */
static const uint8_t bar5[10] = {
    0x11, /* 10001  A K U */
    0x12, /* 01001  B L V */
    0x03, /* 11000  C M W */
    0x14, /* 00101  D N X */
    0x05, /* 10100  E O Y */
    0x06, /* 01100  F P Z */
    0x18, /* 00011  G Q   */
    0x09, /* 10010  H R   */
    0x0A, /* 01010  I S   */
    0x0C  /* 00110  J T * */
};
static const uint8_t space_slot[3] = {2, 3, 0}; /* per row, 0-based */

uint16_t bc_pattern (uint32_t idx)
{
    uint32_t row, col, k;
    uint16_t p = 0;

    if (idx == BC_IDX_STAR)
    {
        row = 2;
        col = 9;
    }
    else if (idx < 26U)
    {
        row = idx / 10U;
        col = idx % 10U;
    }
    else
    {
        return 0;
    }
    for (k = 0; k < 5U; k++)
    {
        if (bar5[col] & (1U << k))
        {
            p |= (uint16_t) (1U << (2U * k)); /* bar k = element 2k */
        }
    }
    /* space s = element 2s+1 */
    p |= (uint16_t) (1U << (2U * space_slot[row] + 1U));
    return p;
}

/* One character, 9 elements -> pattern index 0..26, or < 0 */
static int32_t decode_char (const uint16_t * w)
{
    uint32_t i, b1 = 0, b2 = 2, s1 = 1;
    uint32_t nb_max = 0, ns_max = 0, wb_min;
    uint16_t pat;

    /* two widest bars (elements 0, 2, 4, 6, 8) */
    if (w[b2] > w[b1])
    {
        b1 = 2;
        b2 = 0;
    }
    for (i = 4; i <= 8; i += 2)
    {
        if (w[i] > w[b1])
        {
            b2 = b1;
            b1 = i;
        }
        else if (w[i] > w[b2])
        {
            b2 = i;
        }
    }
    /* widest space (elements 1, 3, 5, 7) */
    for (i = 3; i <= 7; i += 2)
    {
        if (w[i] > w[s1])
        {
            s1 = i;
        }
    }
    for (i = 0; i < 9U; i++)
    {
        if (i == b1 || i == b2 || i == s1)
        {
            continue;
        }
        if ((i & 1U) == 0)
        {
            if (w[i] > nb_max)
            {
                nb_max = w[i];
            }
        }
        else
        {
            if (w[i] > ns_max)
            {
                ns_max = w[i];
            }
        }
    }
    wb_min = (w[b1] < w[b2]) ? w[b1] : w[b2];
    if (wb_min * 10U < nb_max * BC_RATIO_X10 ||
        (uint32_t) w[s1] * 10U < ns_max * BC_RATIO_X10)
    {
        return BCD_ERR_WIDTH;
    }

    pat = (uint16_t) ((1U << b1) | (1U << b2) | (1U << s1));
    for (i = 0; i <= BC_IDX_STAR; i++)
    {
        if (bc_pattern(i) == pat)
        {
            return (int32_t) i;
        }
    }
    return BCD_ERR_PATTERN;
}

static int32_t decode_dir (const uint16_t * w, uint32_t n, uint8_t * letter)
{
    uint32_t k     = (n + 1U) / 10U, c;
    int32_t  first = -1, second = -1, last = -1, r;

    for (c = 0; c < k; c++)
    {
        r = decode_char(&w[c * 10U]); /* element c*10+9 = gap: skipped */
        if (r < 0)
        {
            return r;
        }
        if (c == 0)
        {
            first = r;
        }
        if (c == 1)
        {
            second = r;
        }
        last = r;
    }
    if (first != (int32_t) BC_IDX_STAR || last != (int32_t) BC_IDX_STAR)
    {
        return BCD_ERR_FRAME;
    }
    if (second == (int32_t) BC_IDX_STAR)
    {
        return BCD_ERR_NOT_LETTER;
    }
    *letter = (uint8_t) second;
    return BCD_OK;
}

int32_t bc_decode (const uint16_t * w, uint32_t n, uint8_t * letter,
                   uint8_t * reversed)
{
    uint16_t rev[BC_MAX_ELEMS];
    uint32_t i;
    int32_t  r1, r2;

    if (n < 29U || n > BC_MAX_ELEMS || ((n + 1U) % 10U) != 0)
    {
        return BCD_ERR_COUNT;
    }
    r1 = decode_dir(w, n, letter);
    if (r1 == BCD_OK)
    {
        *reversed = 0;
        return BCD_OK;
    }
    for (i = 0; i < n; i++)
    {
        rev[i] = w[n - 1U - i];
    }
    r2 = decode_dir(rev, n, letter);
    if (r2 == BCD_OK)
    {
        *reversed = 1;
        return BCD_OK;
    }
    /* report the more specific failure */
    return (r1 == BCD_ERR_FRAME) ? r2 : r1;
}

int32_t bc_decode_window (const uint16_t * w, uint32_t n, uint8_t * letter,
                          uint8_t * reversed, uint32_t * start, uint32_t * len)
{
    uint32_t s, l;

    /* earliest start first, longest window first */
    for (s = 0; s + 29U <= n; s += 2U)
    { /* bars at even indices */
        for (l = BC_MAX_ELEMS; l >= 29U; l -= 10U)
        {
            if (s + l <= n && bc_decode(&w[s], l, letter, reversed) == BCD_OK)
            {
                *start = s;
                *len   = l;
                return BCD_OK;
            }
        }
    }
    return bc_decode(w, n, letter, reversed);
}

void bc_edge_reset (BC_EDGE * e, uint16_t v, uint16_t thr)
{
    e->black = (uint8_t) (v > thr);
    e->ext   = v;
}

uint8_t bc_edge_step (BC_EDGE * e, uint16_t v, uint16_t swing, uint16_t floor)
{
    if (e->black)
    {
        if (v > e->ext)
        {
            e->ext = v; /* new peak */
        }
        else if ((uint32_t) v + swing < e->ext)
        {
            e->black = 0; /* fell 'swing' below the peak */
            e->ext   = v;
        }
    }
    else
    {
        if (v < e->ext)
        {
            e->ext = v; /* new dip */
        }
        else if ((uint32_t) v > (uint32_t) e->ext + swing && v > floor)
        {
            e->black = 1; /* rose 'swing' above the dip */
            e->ext   = v;
        }
    }
    return e->black;
}

/*** end of file ***/
