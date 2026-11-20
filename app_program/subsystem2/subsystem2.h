/*
 *      subsystem2.h
 *      Subsystem 2 -- Line Following, Barcode Decode, LineSearch.
 *      Owner: Ryan (Track Purist).  Public interface only.  Plan section 7.
 *
 *      Pin revision 2 (AO outputs only, DO unconnected):
 *        left line sensor  GP26 = ADC0   analog, threshold set by calibration
 *        right line sensor GP17          AO into a NON-ADC pin: digital only
 *        barcode sensor    GP27 = ADC1   analog
 *
 *      Outputs  line error word (5 ms), EV_LINE_LOST, EV_JUNCTION,
 *               EV_BARCODE, EV_LINE_SAMPLE (only in SYS_LINE_SEARCH),
 *               EV_CALIB_DONE (CAL_WHO_IR), LINE_SEARCH_FAIL bit
 *      Inputs   ADC0, ADC1, GP17, nav_state odometer, sys_state_get(),
 * flg_calib
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "line_search.h"

IMPORT ID tid_line;
IMPORT ID tid_barcode;

/* GP17 level on black tape.  On most TCRT5000 modules AO rises on black
   (less reflection), so black = 1.  TODO RY-1.3: check on the bench, and
   check that white really reads 0 (AO must go below 0.8 V). */
#define LINE_R_BLACK 1U

/* IR thresholds for the two analog sensors, set by calibration (RY-1.4).
   ADC above the threshold = black.  Placeholders until measured. */
typedef struct
{
    UH thr_l;  /* left line sensor, ADC0 */
    UH thr_bc; /* barcode sensor, ADC1 */
    UB calibrated;
} IR_CAL;
IMPORT IR_CAL ir_cal;

/* Newest ADC reading of channel ADC_CH_LINE_L / ADC_CH_BARCODE, at most
   1 ms old.  The barcode task is the only ADC reader (task_barcode.c). */
IMPORT UW adc_latest(UINT ch);

/* Barcode counters (telemetry, tests) */
typedef struct
{
    UW reads;
    UW misreads;
    UB last_letter; /* 0..25, BC_LETTER_INVALID = none */
    UB last_rev;
} BC_STATS;
IMPORT volatile BC_STATS bc_stats;

/* Private to subsystem 2 */
IMPORT void line_task(INT stacd, void * exinf);
IMPORT void barcode_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
