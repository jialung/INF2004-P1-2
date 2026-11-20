/*
 *	task_barcode.c
 *	Barcode Decode task, 1 ms, core 1, priority 11.             Owner: Ryan
 *	Plan refs: RY-1.11, chain 4.3 steps 2-4, FR2, NFR3 (>= 1 ms sampling).
 *
 *	This task is the ONLY reader of the ADC: every 1 ms it converts ADC0
 *	(left line sensor) and ADC1 (barcode sensor) and leaves both in
 *	adc_last[]; Line Following reads adc_latest() instead of the ADC, so
 *	two tasks never race on the one converter (the job the DMA ring,
 *	RY-1.5, was meant to do; 2 x ~2 us per ms is cheap enough without it).
 *
 *	Barcode sensor -> black / white from peaks and dips (bc_edge_step),
 *	and a 2-sample glitch filter.  Widths are DISTANCE, in 0.1 mm: how
 *	far the barcode sensor itself moved between two edges, from both
 *	wheel encoders read every 1 ms (see "Distance" below).  They used to
 *	be time (ms), which is only right at a steady speed.
 *
 *	Statechart (Design Review 6.1):
 *	  WaitQuiet  -> first black edge             -> Capture
 *	  Capture    -> each edge: store the width
 *	             -> white longer than the quiet zone -> Decode
 *	             -> too many elements / one bar too long -> discard
 *	  Decode     -> '*' X '*' forwards or reversed -> EV_BARCODE (letter)
 *	             -> otherwise                      -> EV_BARCODE_MISREAD
 *	Only while following the line (LineFollow, HumpTraverse): pivots and
 *	searches sweep the sensor sideways over the floor.  A capture with
 *	fewer than 9 elements (the crossing line of a junction, tape edges)
 *	is dropped silently.
 */

#include "subsystem2.h"
#include "drv_adc.h"
#include "barcode_decode.h"
#include "../hw_alloc.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../wcet.h"
#include "../subsystem1/encoders.h"	/* wheel counts, for distance */
#include "../subsystem1/subsystem1.h"	/* ENC_SIGN_L / ENC_SIGN_R */

#define BC_HYST			200	/* ADC counts either side of thr_bc */
/* 1: black / white from peaks and dips (bc_edge_step), 0: the old fixed
   thr_bc +- BC_HYST.  BC_SWING: how far the reading must rise from the
   last dip / fall from the last peak to count as an edge.  30 Oct traces:
   real bar edges swing 950-3700, the sensor half over the line edge
   drifts up to ~560, paper noise ~100. */
#ifndef BC_ADAPTIVE
#define BC_ADAPTIVE		1
#endif
/* 700 -> 400 (30 Oct, evening): after the sensor height change the bars
   read only 2200-2670 and the barcode was taken as one dark stretch.
   400 may let the line edge (drift up to ~560) make extra edges; those
   give misread / dropped lines, never a letter (the '*' frame is needed). */
#define BC_SWING		400

/* ---- Distance (30 Oct: was time in ms) --------------------------------
 * Every width is the distance the barcode SENSOR moved, in 0.1 mm.  Time
 * only worked at a steady speed: while the car weaves, the wheel targets
 * swing between e.g. 35 and 137 per mille, so the sensor's speed changes
 * by more than the 1.4 x wide / narrow margin inside one character.
 * The sensor spot is BARCODE_SENSOR_Y_MM left of the centre line, so per
 * wheel distance dL, dR it moves
 *     d = dL * (1/2 + y/T) + dR * (1/2 - y/T)        T = track width
 * (on a straight, dL = dR, d is the same for any y).  The encoders are
 * read here every 1 ms; one count = pi * 65 / 2425 = 0.084 mm, so a ~3 mm
 * narrow bar is ~36 counts.  The odometer in nav_state is no use here:
 * 1 mm steps, updated every 10 ms.                                       */
#define BC_MM_PER_COUNT_Q16	((W)((205887LL * WHEEL_DIAMETER_MM) / ENC_COUNTS_PER_REV))
/* T here is the EFFECTIVE track width (TURN_TRACK_X10, 0.1 mm), the one
   Motion Control's encoder heading uses: the turn rate is what matters. */
#define BC_KL_Q16		((W)(((long long)BC_MM_PER_COUNT_Q16 * \
				 (TURN_TRACK_X10 + 20 * BARCODE_SENSOR_Y_MM)) / (2 * TURN_TRACK_X10)))
#define BC_KR_Q16		((W)(((long long)BC_MM_PER_COUNT_Q16 * \
				 (TURN_TRACK_X10 - 20 * BARCODE_SENSOR_Y_MM)) / (2 * TURN_TRACK_X10)))
#define BC_MM(mm)		((UINT)(mm) * 10U)	/* mm -> width units (0.1 mm) */

/* A 1-2 element capture longer than this is not the cross line (18 mm
   wide, ~22 mm at 35 deg) but a barcode whose bars merged: reported,
   trace kept.  Was 400 ms (~48 mm at 120 mm/s). */
#define BC_LONG_DARK		BC_MM(40)
#define BC_GLITCH		2	/* samples (ms) a new colour must last */
#define BC_QUIET_MIN		BC_MM(8)	/* end of barcode: white this far ...
						   (was 60 ms, ~7 mm) */
/* ... or this many x the widest element.  3 -> 4 (30 Oct): the course has
   141 mm and 290 mm barcodes, i.e. narrow ~3 mm / ~6.2 mm and wide ~9 /
   ~18.5 mm if wide = 3 x narrow.  '*' starts narrow bar, WIDE space, so
   right after the first bar the quiet zone was 3 x narrow = the wide
   space itself, and the capture could end inside it.  4 x leaves a 33 %
   margin; the white after a barcode must then be > 4 x wide (~36 mm /
   ~74 mm), else the next dark mark joins the capture (bc_decode_window
   still finds the barcode at its start). */
#define BC_QUIET_FACTOR		4
#define BC_ELEM_MAX		BC_MM(300)	/* one bar / space longer: not a code */
#define BC_ELEM_MAX_MS		3000U	/* ... or this long (car stopped on it) */
#define BC_TRACE_STEP_Q16	(65536 / 2)	/* trace: one value per 0.5 mm */
#define BC_TRACE_N		700U	/* 350 mm: the 290 mm barcode plus the
					   white after it */

EXPORT volatile UH	adc_last[2];
EXPORT volatile BC_STATS bc_stats;

LOCAL UH	w[BC_MAX_ELEMS];	/* widths, 0.1 mm */
LOCAL UINT	n;
LOCAL BOOL	capturing;
LOCAL BOOL	black;		/* accepted colour */
LOCAL UINT	run;		/* samples (ms) in the accepted colour */
LOCAL UINT	cand;		/* samples of the other colour, not yet accepted */
LOCAL UINT	wmax;		/* widest element so far, 0.1 mm */
LOCAL UW	pos_q16;	/* sensor travel, mm Q16; wraps, use differences */
LOCAL UW	edge_pos;	/* pos_q16 where the current element began */
LOCAL UW	cand_pos;	/* pos_q16 at the first sample of the other colour */
LOCAL W		prev_cnt[2];
LOCAL BOOL	cnt_ready;
LOCAL UB	line[300];	/* misread dump */
LOCAL BOOL	trace_rearm;	/* last capture was 1-2 elements: restart the trace */
LOCAL BC_EDGE	edge;		/* peak / dip tracker (BC_ADAPTIVE) */

EXPORT UW adc_latest(UINT ch)
{
	return adc_last[ch & 1U];
}

/* Advance pos_q16 by how far the sensor spot moved since the last call.
   Called once per sample, at the same time as the ADC read, so a sample's
   position is where the sensor was when it was converted. */
LOCAL void travel_update(void)
{
	W c[2], dl, dr;

	c[0] = ENC_SIGN_L * encoders_count(ENC_LEFT);
	c[1] = ENC_SIGN_R * encoders_count(ENC_RIGHT);
	if (!cnt_ready) {
		prev_cnt[0] = c[0];
		prev_cnt[1] = c[1];
		cnt_ready = TRUE;
	}
	dl = c[0] - prev_cnt[0];		/* wrap-safe */
	dr = c[1] - prev_cnt[1];
	prev_cnt[0] = c[0];
	prev_cnt[1] = c[1];
	pos_q16 += (UW)(dl * BC_KL_Q16 + dr * BC_KR_Q16);
}

/* Distance from one position to a later one, in 0.1 mm.  Backwards
   (the sensor rolled back) counts as 0. */
LOCAL UH width_of(UW from, UW to)
{
	W q = (W)(to - from);

	if (q <= 0) return 0;
	q = (W)(((long long)q * 10) >> 16);
	return (q > 0xFFFF) ? (UH)0xFFFFU : (UH)q;
}

/* A capture that was thrown away, with its widths, so a barcode that is
   never read still shows up in the log (30 Oct: nothing was printed). */
LOCAL void dump(const char *why)
{
	INT i, len;

	bc_stats.misreads++;
	len = tm_sprintf(line, (const UB*)"[bc] dropped (%s), %d elements, widths 0.1 mm:",
			 why, (INT)n);
	for (i = 0; i < (INT)n && len < (INT)sizeof(line) - 8; i++) {
		len += tm_sprintf(&line[len], (const UB*)" %d", (INT)w[i]);
	}
	tm_printf((UB*)"%s\n", line);
}

LOCAL void finish(void)
{
	NAV_STATE	ns;
	uint8_t		letter = 0, rev = 0;
	INT		r, i, len;

	capturing = FALSE;
	if (n < 3U) {
		UW total = 0;
		for (i = 0; i < (INT)n; i++) total += w[i];
		if (total < BC_LONG_DARK) {
			trace_rearm = TRUE;	/* the cross line, not a barcode */
			return;
		}
		/* 1-2 elements but long: a whole barcode seen as one dark
		   stretch (bars not separated).  30 Oct: silently dropped,
		   nothing to go on.  Keep the trace and say so. */
		dump("one long dark stretch: barcode bars not separated");
		return;
	}
	if (n < BC_ELEMS_PER_CHAR) {
		/* junction line, tape edge: 1-2 elements, normal.  More than
		   that is probably a barcode only partly under the sensor. */
		if (n >= 3U) dump("too few elements for a letter");
		return;
	}
	nav_read(&ns);
	{
		uint32_t ws = 0, wl = n;
		r = bc_decode_window(w, n, &letter, &rev, &ws, &wl);
		if (r == BCD_OK && (ws != 0U || wl != n)) {
			tm_printf((UB*)"[bc] barcode found in elements %d..%d of %d "
				  "(the rest: line edge / cross line)\n",
				  (INT)ws, (INT)(ws + wl - 1U), (INT)n);
		}
	}
	if (r == BCD_OK) {
		bc_stats.reads++;
		bc_stats.last_letter = letter;
		bc_stats.last_rev = rev;
		len = tm_sprintf(line, (const UB*)"[bc] read '%c' (%s), %d elements, widths 0.1 mm:",
				 (INT)('A' + letter), rev ? "reversed" : "forward", (INT)n);
		for (i = 0; i < (INT)n && len < (INT)sizeof(line) - 8; i++) {
			len += tm_sprintf(&line[len], (const UB*)" %d", (INT)w[i]);
		}
		tm_printf((UB*)"%s\n", line);
		(void)sup_post(SRC_BARCODE, EV_BARCODE, BC_A(letter, rev), ns.m.odo_mm);
	} else {
		bc_stats.misreads++;
		len = tm_sprintf(line, (const UB*)"[bc] misread (error %d), %d elements, widths 0.1 mm:",
				 r, (INT)n);
		for (i = 0; i < (INT)n && len < (INT)sizeof(line) - 8; i++) {
			len += tm_sprintf(&line[len], (const UB*)" %d", (INT)w[i]);
		}
		tm_printf((UB*)"%s\n", line);
		(void)sup_post(SRC_BARCODE, EV_BARCODE_MISREAD, bc_stats.misreads, ns.m.odo_mm);
	}
}

LOCAL void sample(UW v, BOOL active)
{
	W	thr = (W)ir_cal.thr_bc;
	BOOL	b;

#if BC_ADAPTIVE
	/* peaks and dips (barcode_decode.h); the fixed threshold only sets
	   the floor a bar must reach and the colour at the start */
	{
		W floor = thr - BC_HYST;
		if (floor < 0) floor = 0;
		b = (BOOL)bc_edge_step(&edge, (uint16_t)v, BC_SWING, (uint16_t)floor);
	}
#else
	if ((W)v > thr + BC_HYST)      b = TRUE;
	else if ((W)v < thr - BC_HYST) b = FALSE;
	else                           b = black;	/* in the band: hold */
#endif

	if (!active) {
		/* Following ended (junction, line lost, obstacle, stop) in the
		   middle of a capture: decode / report what was seen instead of
		   dropping it silently (30 Oct: a 1.6 s trace of clean DO bars,
		   then nothing). */
		if (capturing && n >= 3U) {
			if (black && run > 0U && n < BC_MAX_ELEMS) {
				w[n++] = width_of(edge_pos, pos_q16);	/* last bar */
			}
			tm_printf((UB*)"[bc] following ended during a barcode, %d elements so far:\n",
				  (INT)n);
			finish();
		}
		capturing = FALSE;
#if BC_ADAPTIVE
		bc_edge_reset(&edge, (uint16_t)v, (uint16_t)thr);
		b = (BOOL)edge.black;
#endif
		black = b; run = 0; cand = 0;
		edge_pos = pos_q16;
		return;
	}

	if (b == black) {
		run += cand + 1U;		/* a rejected glitch belongs here */
		cand = 0;			/* (its distance is inside edge_pos..) */
	} else {
		if (cand == 0U) {
			cand_pos = pos_q16;	/* where the other colour began */
		}
		if (++cand >= BC_GLITCH) {
			/* colour change accepted: the finished element ran
			   from edge_pos to where the new colour began */
			UH wd = width_of(edge_pos, cand_pos);

			if (capturing) {
				if (n >= BC_MAX_ELEMS) {
					/* full: the barcode may still be in
					   there, with junk around it
					   (bc_decode_window) */
					finish();
				} else {
					w[n++] = wd;
					if (wd > wmax) wmax = wd;
				}
			} else if (b) {
				capturing = TRUE;	/* first black edge */
				n = 0;
				wmax = 0;
			}
			black = b;
			run = cand;
			cand = 0;
			edge_pos = cand_pos;
		}
	}

	if (capturing) {
		UINT quiet = BC_QUIET_FACTOR * wmax;
		UINT cur = width_of(edge_pos, pos_q16);

		if (quiet < BC_QUIET_MIN) quiet = BC_QUIET_MIN;
		if (!black && cur > quiet) {
			finish();			/* quiet zone after the last bar */
		} else if (cur > BC_ELEM_MAX || run > BC_ELEM_MAX_MS) {
			capturing = FALSE;		/* sat on black / white too long */
			if (n >= 3U) dump("one bar longer than 300 mm / 3 s");
		}
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;
	wcet_set_period(WCET_BARCODE, PERIOD_BARCODE_MS);
	bc_stats.last_letter = BC_LETTER_INVALID;

	UH	run_min = 0xFFFFU, run_max = 0;	/* barcode ADC while following */
	UW	run_ms = 0;
	/* Raw trace of the first barcode-like stretch in each run, from the
	   first reading above black, one value every 0.5 mm of sensor travel.
	   Printed when following ends (car stopped), to see what the sensor
	   really saw: bars merging, bars too grey, or the sensor sliding off
	   the barcode sideways.  Being per distance, it also shows the bar
	   widths in mm directly (2 values = 1 mm). */
	LOCAL UH	trace[BC_TRACE_N];
	UINT		tr_n = 0;
	BOOL		tr_on = FALSE;
	UH		tr_odo0 = 0;
	UW		tr_next = 0;	/* pos_q16 of the next trace value */

	for (;;) {
		UW		t0;
		SYS_STATE	s;
		BOOL		active;
		UH		v;

		tk_slp_tsk(TMO_FEVR);
		t0 = wcet_release(WCET_BARCODE);

		adc_last[ADC_CH_LINE_L]  = (UH)drv_adc_read(ADC_CH_LINE_L);
		adc_last[ADC_CH_BARCODE] = (UH)drv_adc_read(ADC_CH_BARCODE);
		travel_update();		/* where the sensor was for this sample */

		s = sys_state_get();
		active = (s == SYS_LINE_FOLLOW || s == SYS_HUMP_TRAVERSE);
		v = adc_last[ADC_CH_BARCODE];
		sample(v, active);

		/* Per run: did the barcode sensor ever see black?  Printed when
		   the car stops following (junction, obstacle, stop, fault). */
		if (active) {
			if (v < run_min) run_min = v;
			if (v > run_max) run_max = v;
			run_ms++;
			if (trace_rearm) {
				trace_rearm = FALSE;
				tr_n = 0;		/* that was the cross line */
				tr_on = FALSE;
			}
			if (!tr_on && tr_n == 0 && (W)v > (W)ir_cal.thr_bc + BC_HYST) {
				NAV_STATE ns;
				nav_read(&ns);
				tr_on = TRUE;		/* first black of this run */
				tr_next = pos_q16;
				tr_odo0 = ns.m.odo_mm;
			}
			if (tr_on && (W)(pos_q16 - tr_next) >= 0) {
				trace[tr_n++] = v;
				tr_next += BC_TRACE_STEP_Q16;
				if ((W)(pos_q16 - tr_next) >= 0) {
					tr_next = pos_q16 + BC_TRACE_STEP_Q16;	/* fell behind */
				}
				if (tr_n >= BC_TRACE_N) tr_on = FALSE;	/* full */
			}
		} else if (run_ms > 0) {
			UINT i;

#if BC_ADAPTIVE
			tm_printf((UB*)"[bc] last %d ms of following: barcode sensor min %d max %d "
				  "(a bar: rise >= %d from a dip, above %d), %d read, "
				  "%d misread/dropped so far\n",
				  (INT)run_ms, (INT)run_min, (INT)run_max,
				  (INT)BC_SWING, (INT)ir_cal.thr_bc - BC_HYST,
				  (INT)bc_stats.reads, (INT)bc_stats.misreads);
#else
			tm_printf((UB*)"[bc] last %d ms of following: barcode sensor min %d max %d "
				  "(black needs > %d), %d read, %d misread/dropped so far\n",
				  (INT)run_ms, (INT)run_min, (INT)run_max,
				  (INT)ir_cal.thr_bc + BC_HYST, (INT)bc_stats.reads,
				  (INT)bc_stats.misreads);
#endif
			if (tr_n > 0) {
				tm_printf((UB*)"[bc] trace from the first black (odo %d mm), one value "
					  "every 0.5 mm of sensor travel, black > %d, white < %d:\n",
					  (INT)tr_odo0, (INT)ir_cal.thr_bc + BC_HYST,
					  (INT)ir_cal.thr_bc - BC_HYST);
				for (i = 0; i < tr_n; i += 20U) {
					UINT k, len = 0;
					len = (UINT)tm_sprintf(line, (const UB*)"[bc] trace %3d mm:",
							       (INT)(i / 2U));
					for (k = i; k < i + 20U && k < tr_n; k++) {
						len += (UINT)tm_sprintf(&line[len], (const UB*)" %4d",
									(INT)trace[k]);
					}
					tm_printf((UB*)"%s\n", line);
				}
			}
			run_min = 0xFFFFU; run_max = 0; run_ms = 0;
			tr_n = 0; tr_on = FALSE;
		}

		wcet_end(WCET_BARCODE, t0);
	}
}
