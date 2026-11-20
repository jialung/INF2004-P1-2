/*
 *      barcode_decode.h
 *      Code 39 decoder, A-Z, read in either direction.             Owner: Ryan
 *      Plain <stdint.h>, no kernel calls, so the laptop tests compile it
 *      (tests/host).  Plan refs: RY-1.11, FR2.
 *
 *      Input: the widths of the bars and spaces of one barcode, in the order
 *      the sensor met them, starting and ending with a BAR.  Any unit works
 *      (ms at constant speed, mm, samples): only the ratios matter.
 *
 *      Code 39: each character is 9 elements (5 bars, 4 spaces), exactly 2
 *      bars and 1 space wide; characters are separated by a narrow space.
 *      The printed barcode is  * letter *  ('*' = start / stop), so a full
 *      read is 9 + 1 + 9 + 1 + 9 = 29 elements.
 *
 *      Wide / narrow is decided separately for bars and spaces: the 2 widest
 *      of the 5 bars, the widest of the 4 spaces.  The IR spot makes black
 *      bars look wider than white spaces; judging each group against itself
 *      cancels that bias.  A group whose wide and narrow widths are not
 *      clearly apart (BC_RATIO_X10) is rejected rather than guessed.
 *
 *      Direction: read backwards, '*' is a different character, so the
 *      decoder tries the elements as read, and reversed; whichever gives
 *      '*' ... '*' wins and sets *reversed.
 */

#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H

#include <stdint.h>

#define BC_ELEMS_PER_CHAR 9U
#define BC_IDX_STAR       26U /* bc_pattern() index of '*' */
#define BC_MAX_ELEMS      49U /* up to 5 characters */
#define BC_RATIO_X10      14U /* wide >= 1.4 x narrow */

#define BCD_OK             0
#define BCD_ERR_COUNT      (-1) /* not 10k - 1 elements, k >= 3 */
#define BCD_ERR_WIDTH      (-2) /* wide and narrow not clearly apart */
#define BCD_ERR_PATTERN    (-3) /* not a Code 39 A-Z / '*' pattern */
#define BCD_ERR_FRAME      (-4) /* no '*' at both ends */
#define BCD_ERR_NOT_LETTER (-5) /* '*' where the letter should be */

/* Wide-element pattern of A..Z (0..25) and '*' (26): bit i set = element
   i (0 = first bar) is wide. */
uint16_t bc_pattern(uint32_t idx);

/* Decode one barcode.  On BCD_OK: *letter = 0..25 (A..Z), *reversed = 1
   if the elements had to be reversed (read from the far end). */
int32_t bc_decode(const uint16_t * w, uint32_t n, uint8_t * letter,
                  uint8_t * reversed);

/* Decode a capture that may hold MORE than one barcode's elements: a
   bump from the line edge before it, or the crossing line of a junction
   after it (30 Oct: "17 elements", the last one the cross).  Tries every
   window that starts on a bar, 49 / 39 / 29 elements long, earliest
   first; '*' at both ends is required, so a window cut from the middle
   of junk is rejected.  On BCD_OK also gives the window (*start, *len).
   Otherwise returns bc_decode()'s result for the whole capture. */
int32_t bc_decode_window(const uint16_t * w, uint32_t n, uint8_t * letter,
                         uint8_t * reversed, uint32_t * start, uint32_t * len);

/* Black / white from the signal's own peaks and dips instead of one fixed
 * threshold (30 Oct traces: narrow bars at 2044-2292 and narrow gaps
 * that only dipped to 1430-1500 between bars at 2400, both lost against
 * a fixed 1713 / 1313).
 *   white -> black: the reading has RISEN 'swing' above the lowest value
 *                   seen since it went white, AND is above 'floor'.
 *   black -> white: the reading has FALLEN 'swing' below the highest
 *                   value seen since it went black.
 * A slow drift smaller than 'swing' (the sensor half over the line edge,
 * 1550-2100 for a second) is not an edge.                              */
typedef struct
{
    uint8_t  black;
    uint16_t ext; /* peak while black, dip while white */
} BC_EDGE;

void    bc_edge_reset(BC_EDGE * e, uint16_t v, uint16_t thr);
uint8_t bc_edge_step(BC_EDGE * e, uint16_t v, uint16_t swing, uint16_t floor);

#endif /* BARCODE_DECODE_H */

/*** end of file ***/
