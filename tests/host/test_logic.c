/*
 *	test_logic.c
 *	Laptop unit tests for the pure logic in the skeleton:
 *	  msg_types.h packing, msg_bus.c send rules, fault_flags.c,
 *	  telem_frame.c (CRC-8 + frame), line_search.c (first cut).
 *	Build and run:   make -C tests/host -f host_tests.mk
 *	The kernel calls are faked below (a tiny message-buffer and event flag).
 */

#include <stdio.h>
#include <string.h>
#include "../../app_program/app.h"
#include "../../app_program/msg_bus.h"
#include "../../app_program/fault_flags.h"
#include "../../app_program/subsystem2/line_search.h"
#include "../../app_program/subsystem5/telem_frame.h"
#include "../../app_program/subsystem2/barcode_decode.h"
#include "../../app_program/subsystem5/map_parse.h"
#include "../../app_program/subsystem3/imu_math.h"
#include "../../app_program/subsystem5/bypass_geom.h"
#include "../../app_program/subsystem5/obst_width.h"

static int pass, fail;
#define T_CHECK(id, cond) do { if (cond) pass++; else { fail++; \
	printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

/* ---- fakes ------------------------------------------------------- */
#define MAXQ 8
typedef struct { INT bufsz, used, msgsz; UB last[32]; INT count; } FAKE_MBF;
static FAKE_MBF mbf[MAXQ];
static INT n_mbf;
static UINT flg[4];
static INT n_flg;

ID tk_cre_mbf(const T_CMBF *p) { mbf[n_mbf].bufsz = (INT)p->bufsz; mbf[n_mbf].msgsz = p->maxmsz; return ++n_mbf; }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t)
{
	FAKE_MBF *q = &mbf[id - 1]; (void)t;
	if (q->used + sz + 4 > q->bufsz) return -50;		/* E_TMOUT-like */
	q->used += sz + 4; q->count++; memcpy(q->last, m, (size_t)sz); return E_OK;
}
ER tk_ref_mbf(ID id, T_RMBF *r) { r->frbufsz = mbf[id - 1].bufsz - mbf[id - 1].used; return E_OK; }
ID tk_cre_flg(const T_CFLG *p) { flg[n_flg] = p->iflgptn; return ++n_flg; }
ER tk_set_flg(ID id, UINT p) { flg[id - 1] |= p; return E_OK; }
ER tk_clr_flg(ID id, UINT p) { flg[id - 1] &= p; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *r) { r->flgptn = flg[id - 1]; return E_OK; }
uint32_t hal_now_us(void) { static uint32_t t; return t += 100; }
static void drain(ID id) { mbf[id - 1].used = 0; mbf[id - 1].count = 0; }
static UW last_motion(void) { UW w; memcpy(&w, mbf[mbf_motion_cmd - 1].last, 4); return w; }

/* ---- tests -------------------------------------------------------- */
static void test_bitfields(void)
{
	INT a, d;
	UW w;

	/* signed fields: every value of an 8-bit angle and 11-bit distance */
	for (a = -128; a <= 127; a++) {
		w = MC_PACK(MC_TURN_REL, SPD_SLOW, a * 2, 0, 5);
		T_CHECK("MC-angle", MC_ANGLE_DEG(w) == a * 2);
		T_CHECK("MC-type", MC_TYPE_OF(w) == MC_TURN_REL && MC_SPEED(w) == SPD_SLOW && MC_SEQ(w) == 5);
	}
	for (d = MC_DIST_MIN_MM; d <= MC_DIST_MAX_MM; d++) {
		w = MC_PACK(MC_DRIVE, SPD_CRUISE, -90, d, 15);
		if (MC_DIST_MM(w) != d || MC_ANGLE_DEG(w) != -90) { T_CHECK("MC-dist", 0); break; }
	}
	T_CHECK("MC-dist", 1);

	/* clamping in motion_cmd_make: out-of-range never wraps sign */
	w = motion_cmd_make(MC_DRIVE, SPD_SLOW, 0, 5000, 1);
	T_CHECK("MC-clamp+", MC_DIST_MM(w) == MC_DIST_MAX_MM);
	w = motion_cmd_make(MC_DRIVE, SPD_SLOW, 0, -5000, 1);
	T_CHECK("MC-clamp-", MC_DIST_MM(w) == MC_DIST_MIN_MM);
	w = motion_cmd_make(MC_TURN_REL, SPD_SLOW, 300, 0, 1);
	T_CHECK("MC-clamp-angle", MC_ANGLE_DEG(w) == 254);
	w = motion_cmd_make(MC_TURN_REL, SPD_SLOW, 180, 0, 1);
	T_CHECK("MC-uturn", MC_ANGLE_DEG(w) == 180);

	/* profile word (LK-1.12) uses 27 of 32 bits */
	w = PROF_A(127, 100, 3, GAP_RIGHT, 15);
	T_CHECK("PROF", PROF_WIDTH(w) == 127 && PROF_GAP_L(w) == 100 && PROF_GAP_R(w) == 3 &&
		      PROF_SIDE(w) == GAP_RIGHT && PROF_STEP(w) == 15);
	w = PROF_B(400, 60, 127);
	T_CHECK("PROF_B", PROF_CLOSEST(w) == 400 && PROF_EDGE_L(w) == 60 && PROF_EDGE_R(w) == 127);

	/* line error word */
	w = LE_PACK(-128, LE_VALID | LE_SEEN_R, 200);
	T_CHECK("LE", LE_ERR(w) == -128 && LE_FLAGS(w) == (LE_VALID | LE_SEEN_R) && LE_SEQ(w) == 200);

	/* hump, barcode, scan */
	w = HUMP_A(31, -124);
	T_CHECK("HUMP", HUMP_HEIGHT_MM(w) == 31 && HUMP_PEAK_Q(w) == -124);
	w = BC_A(25, 1);
	T_CHECK("BC", BC_LETTER(w) == 25 && BC_REVERSED(w) == 1);
	w = SCAN_PACK(SCAN_PASS_LEFT, 9, 22);
	T_CHECK("SCAN", SCAN_MODE_OF(w) == SCAN_PASS_LEFT && SCAN_SEQ(w) == 9 && SCAN_PARAM(w) == 22);

	T_CHECK("SUP_EVENT 16 B", sizeof(SUP_EVENT) == 16);
	T_CHECK("TELEM_MSG 16 B", sizeof(TELEM_MSG) == 16);
	T_CHECK("states fit 4 bits", SYS_STATE_COUNT <= (1 << SYS_STATE_BITS));
}

static void test_bus_rules(void)
{
	INT i;

	/* droppable events stop at the reserve; must-keep events still fit */
	drain(mbf_sup_event);
	for (i = 0; i < 40; i++) (void)sup_post(SRC_LINE, EV_LINE_SAMPLE, 0, 0);
	T_CHECK("reserve kept", mbf[mbf_sup_event - 1].count == SUP_QUEUE_MSGS - SUP_RESERVE_MSGS);
	for (i = 0; i < SUP_RESERVE_MSGS; i++) {
		T_CHECK("keep fits", sup_post(SRC_BARCODE, EV_BARCODE, BC_A(i, 0), 0) == E_OK);
	}
	T_CHECK("no keep lost", bus_stats.sup_lost_keep == 0);
	T_CHECK("queue full", sup_post(SRC_BARCODE, EV_BARCODE, 0, 0) != E_OK);
	T_CHECK("loss counted", bus_stats.sup_lost_keep == 1);
	bus_stats.sup_lost_keep = 0;
	drain(mbf_sup_event);
}

static void test_faults(void)
{
	drain(mbf_sup_event);
	fault_raise(FAULT_OBST_STOP);
	T_CHECK("raise", fault_peek() == FAULT_OBST_STOP);
	T_CHECK("raise posts EV_FAULT", mbf[mbf_sup_event - 1].count == 1);
	fault_raise(FAULT_OBST_STOP);
	T_CHECK("no repeat post", mbf[mbf_sup_event - 1].count == 1);
	fault_raise(FAULT_IMPACT);
	fault_clear(FAULT_OBST_STOP);
	T_CHECK("clear keeps others", fault_peek() == FAULT_IMPACT);
	T_CHECK("stop mask", (FAULT_STOP_MASK & FAULT_OBST_STOP) && !(FAULT_TRIP_MASK & FAULT_OBST_STOP));
	T_CHECK("wifi is status only", !(FAULT_STOP_MASK & FAULT_WIFI_DOWN));
	fault_clear(0xFFFF);
	T_CHECK("all clear", fault_peek() == 0);
	drain(mbf_sup_event);
}

static void test_frame(void)
{
	const uint8_t v[] = "123456789";
	TELEM_FIELDS a, b;
	uint8_t fr[TF_LEN];

	T_CHECK("CRC-8/0x07 check value", tf_crc8(v, 9) == 0xF4);	/* CRC-8/SMBUS */
	memset(&a, 0, sizeof a);
	memset(&b, 0, sizeof b);	/* struct padding must match for memcmp */
	a.state = SYS_FAULT; a.faults = 0x1FF; a.odo_mm = 65535; a.v_mm_s = -32768;
	a.heading_deg = 511; a.terrain = 7; a.speed_level = 3; a.roll_q = -128;
	a.pitch_q = 127; a.humps = 255; a.letter = 31; a.letter_rev = 1;
	T_CHECK("pack len", tf_pack(&a, fr) == TF_LEN);
	T_CHECK("unpack", tf_unpack(fr, &b) == 0);
	T_CHECK("round trip", memcmp(&a, &b, sizeof a) == 0);
	fr[2] ^= 1;
	T_CHECK("crc catches", tf_unpack(fr, &b) == -1);
}

static void test_line_search(void)
{
	LS_REF ref = { 90, 1000, 0 };
	LS_RESULT r, r2;
	UW odo = 1000, last_cmd;
	int moves = 0;

	drain(mbf_sup_event);
	fault_clear(0xFFFF);

	/* line lost, last seen on the RIGHT: sweep right first, then left
	   twice as far, back to centre, then creep S1 */
	ref.expected_offset_mm = -60;
	line_search_start(LS_REASON_LINE_LOST, &ref);
	r = line_search_step(EV_NONE, 0, odo);
	T_CHECK("LS sweeps first", r.kind == LS_MOVE && MC_TYPE_OF(r.cmd) == MC_TURN_REL &&
				   MC_ANGLE_DEG(r.cmd) == -LS_ALPHA_DEG);
	last_cmd = r.cmd;
	/* a FOLLOW being replaced replies ABORTED: must not advance the search */
	r2 = line_search_step(EV_MOTION_DONE, MD_A(MC_SEQ(last_cmd), MD_ABORTED), odo);
	T_CHECK("LS ignores ABORTED", r2.kind == LS_WAIT);
	/* a reply for some other command must not advance it either */
	r2 = line_search_step(EV_MOTION_DONE, MD_A(MC_SEQ(last_cmd) ^ 1U, MD_DONE), odo);
	T_CHECK("LS ignores other seq", r2.kind == LS_WAIT);
	r = line_search_step(EV_MOTION_DONE, MD_A(MC_SEQ(last_cmd), MD_DONE), odo);
	T_CHECK("LS sweeps back", r.kind == LS_MOVE && MC_TYPE_OF(r.cmd) == MC_TURN_REL &&
				  MC_ANGLE_DEG(r.cmd) == 2 * LS_ALPHA_DEG);
	last_cmd = r.cmd;
	r = line_search_step(EV_MOTION_DONE, MD_A(MC_SEQ(last_cmd), MD_DONE), odo);
	T_CHECK("LS centres", r.kind == LS_MOVE && MC_ANGLE_DEG(r.cmd) == -LS_ALPHA_DEG);
	last_cmd = r.cmd;
	r = line_search_step(EV_MOTION_DONE, MD_A(MC_SEQ(last_cmd), MD_DONE), odo);
	T_CHECK("LS creeps", r.kind == LS_MOVE && MC_TYPE_OF(r.cmd) == MC_DRIVE &&
			    MC_DIST_MM(r.cmd) == LS_S1_MM);
	last_cmd = r.cmd;

	/* no line at all: after S it fails and raises the fault */
	while (odo < 1000 + LS_S_MM + 10 && moves < 1000) {
		odo += 5;
		r = line_search_step(EV_LINE_SAMPLE, LS_A(100, 100, 0, 0), odo);
		if (line_search_state() == LSS_FAIL) break;
		if (r.kind == LS_MOVE) last_cmd = r.cmd;
		if (odo % 50 == 0) {
			r = line_search_step(EV_MOTION_DONE,
					     MD_A(MC_SEQ(last_cmd), MD_DONE), odo);
			if (r.kind == LS_MOVE) { last_cmd = r.cmd; moves++; }
		}
	}
	T_CHECK("LS fails after S", line_search_state() == LSS_FAIL);
	T_CHECK("LS raised fault", fault_peek() & FAULT_LINE_SEARCH_FAIL);
	fault_clear(0xFFFF);

	/* line seen during a pivot (odometer not moving): 3 samples in a row */
	odo = 3000; ref.odo_mm = 3000; ref.expected_offset_mm = 60;
	line_search_start(LS_REASON_LINE_LOST, &ref);
	r = line_search_step(EV_NONE, 0, odo);
	T_CHECK("LS left first", MC_ANGLE_DEG(r.cmd) == LS_ALPHA_DEG);
	(void)line_search_step(EV_LINE_SAMPLE, LS_A(3000, 0, 1, 0), odo);
	(void)line_search_step(EV_LINE_SAMPLE, LS_A(3000, 0, 1, 0), odo);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(3000, 0, 1, 0), odo);
	T_CHECK("LS found in pivot", r.kind == LS_FOUND);

	/* the odometer moving BACKWARDS a little (pivot, roll back) must not
	   look like 65 m travelled (bug seen on the car, 23 Oct) */
	fault_clear(0xFFFF);
	odo = 0; ref.odo_mm = 0;
	line_search_start(LS_REASON_LINE_LOST, &ref);
	(void)line_search_step(EV_NONE, 0, odo);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(100, 0, 0, 0), 65530);
	T_CHECK("LS backwards is not far", line_search_state() != LSS_FAIL &&
					   (fault_peek() & FAULT_LINE_SEARCH_FAIL) == 0);

	/* line appears and stays for L_confirm -> found */
	odo = 2000; ref.odo_mm = 2000;
	line_search_start(LS_REASON_AFTER_BYPASS, &ref);
	(void)line_search_step(EV_NONE, 0, odo);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(3000, 100, 1, 0), odo += 2);
	T_CHECK("LS confirming", line_search_state() == LSS_CONFIRM);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(100, 100, 0, 0), odo += 2);
	T_CHECK("LS noise rejected", line_search_state() == LSS_CREEP);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(3000, 100, 1, 0), odo += 2);
	r = line_search_step(EV_LINE_SAMPLE, LS_A(3000, 100, 1, 0), odo += LS_CONFIRM_MM);
	T_CHECK("LS found", r.kind == LS_FOUND);
	T_CHECK("no fault", fault_peek() == 0);
}

/* Widths of "* letter *" as the sensor would see them: narrow 3, wide 8,
   bars 'bias' wider (IR blur), +-1 jitter when 'jit' */
static uint32_t make_code(uint32_t letter, uint16_t *w, int bias, int jit)
{
	const uint32_t idx[3] = { BC_IDX_STAR, letter, BC_IDX_STAR };
	uint32_t c, e, n = 0;

	for (c = 0; c < 3; c++) {
		uint16_t p = bc_pattern(idx[c]);
		for (e = 0; e < 9; e++) {
			int v = ((p >> e) & 1) ? 8 : 3;
			if ((e & 1) == 0) v += bias;
			if (jit) v += (int)(n % 3) - 1;
			w[n++] = (uint16_t)v;
		}
		if (c < 2) {
			int g = 3 + (jit ? (int)(n % 3) - 1 : 0);
			w[n++] = (uint16_t)g;
		}
	}
	return n;
}

static void test_barcode(void)
{
	uint16_t w[BC_MAX_ELEMS], r[BC_MAX_ELEMS];
	uint32_t i, j, n, bits, ok_f = 0, ok_r = 0;
	uint8_t  letter, rev;
	int      distinct = 1, shape = 1;

	/* known Code 39 patterns (element i wide = bit i) */
	T_CHECK("C39 A", bc_pattern(0) == 0x121);	/* 100001001 */
	T_CHECK("C39 M", bc_pattern(12) == 0x085);	/* 101000010 */
	T_CHECK("C39 W", bc_pattern(22) == 0x007);	/* 111000000 */
	T_CHECK("C39 Z", bc_pattern(25) == 0x016);	/* 011010000 */
	T_CHECK("C39 *", bc_pattern(BC_IDX_STAR) == 0x052);	/* 010010100 */
	for (i = 0; i <= BC_IDX_STAR; i++) {
		uint16_t p = bc_pattern(i);
		for (bits = 0, j = 0; j < 9; j++) bits += (p >> j) & 1;
		if (bits != 3 || (p & 0x0AA) == 0 || (p & 0x155) == 0) shape = 0;
		for (j = 0; j < i; j++) if (bc_pattern(j) == p) distinct = 0;
	}
	T_CHECK("C39 2 wide bars + 1 wide space", shape);
	T_CHECK("C39 patterns distinct", distinct);

	/* every letter, both directions, with bar bias and jitter */
	for (i = 0; i < 26; i++) {
		n = make_code(i, w, 2, 1);
		if (bc_decode(w, n, &letter, &rev) == BCD_OK && letter == i && rev == 0) ok_f++;
		for (j = 0; j < n; j++) r[j] = w[n - 1 - j];
		if (bc_decode(r, n, &letter, &rev) == BCD_OK && letter == i && rev == 1) ok_r++;
	}
	T_CHECK("C39 all 26 forward", ok_f == 26);
	T_CHECK("C39 all 26 reversed", ok_r == 26);

	n = make_code(0, w, 0, 0);
	T_CHECK("C39 too few elements", bc_decode(w, 28, &letter, &rev) == BCD_ERR_COUNT);
	for (j = 0; j < n; j++) w[j] = 5;
	T_CHECK("C39 flat widths rejected", bc_decode(w, n, &letter, &rev) < 0);

	/* window search: junk before, the cross line after (30 Oct) */
	{
		uint16_t big[BC_MAX_ELEMS];
		uint32_t m = 0, s = 99, l = 99, k;
		int ok_all = 1;

		for (i = 0; i < 26; i++) {
			n = make_code(i, w, 2, 1);
			m = 0;
			big[m++] = 9;  big[m++] = 40;		/* line-edge bump + gap */
			for (k = 0; k < n; k++) big[m++] = w[k];
			big[m++] = 30; big[m++] = 22;		/* gap + cross line */
			if (bc_decode_window(big, m, &letter, &rev, &s, &l) != BCD_OK ||
			    letter != i || s != 2 || l != n) ok_all = 0;
		}
		T_CHECK("C39 window: all 26 with junk before and cross after", ok_all);

		n = make_code(7, w, 2, 1);
		for (k = 0; k < n; k++) big[k] = w[k];
		big[n] = 30; big[n + 1] = 22;
		T_CHECK("C39 window: cross after only",
			bc_decode_window(big, n + 2, &letter, &rev, &s, &l) == BCD_OK &&
			letter == 7 && s == 0 && l == n);
		for (k = 0; k < 31; k++) big[k] = 5;
		T_CHECK("C39 window: flat junk not read",
			bc_decode_window(big, 31, &letter, &rev, &s, &l) != BCD_OK);
		n = make_code(3, w, 2, 1);
		T_CHECK("C39 window: exact capture unchanged",
			bc_decode_window(w, n, &letter, &rev, &s, &l) == BCD_OK &&
			letter == 3 && s == 0 && l == n);
	}
}

/* Edge detector against values copied from the 30 Oct car traces
   (barcode ADC, one value per 4 ms). */
static uint32_t edges_of(const uint16_t *v, uint32_t n, uint16_t swing,
			 uint16_t floor, uint16_t thr)
{
	BC_EDGE  e;
	uint32_t i, flips = 0;
	uint8_t  was;

	bc_edge_reset(&e, v[0], thr);
	was = e.black;
	for (i = 1; i < n; i++) {
		uint8_t b = bc_edge_step(&e, v[i], swing, floor);
		if (b != was) flips++;
		was = b;
	}
	return flips;
}

static void test_bc_edge(void)
{
	/* odo 6358: bars ~2400 with narrow gaps dipping only to ~1430: the
	   fixed 1713 / 1313 kept it black (one merged bar).  Should be:
	   white, bar, gap, bar, gap, bar, white = 6 flips. */
	static const uint16_t narrow_gaps[] = {
		250, 240, 300, 714, 1169, 1613, 1942, 2124, 2094, 2117, 2292, 2394,
		2372, 2213, 1946, 1654, 1457, 1505, 1490, 1433, 1529, 1774, 2016,
		2264, 2420, 2434, 2356, 2388, 2507, 2461, 2299, 2015, 1728, 1489,
		1422, 1497, 1480, 1496, 1636, 1940, 2269, 2577, 2765, 2915, 3070,
		3281, 3496, 3638, 3745, 3828, 2000, 1000, 400, 250, 240 };
	/* odo 7709: sensor half over the line edge, 1550-2100 for a second:
	   no edges at all once it is black. */
	static const uint16_t line_edge[] = {
		1774, 1860, 1891, 1886, 1942, 1960, 1941, 1868, 1877, 1870, 2041,
		2065, 2048, 2017, 2102, 2063, 1987, 1941, 1816, 1764, 1727, 1707,
		1636, 1605, 1650, 1602, 1568, 1608, 1640, 1547, 1549, 1569, 1624,
		1651, 1690, 1729, 1782, 1836, 1907, 1920, 1981, 2012, 1965, 1896,
		1851, 1777, 1790, 1909, 1962, 2009, 2078, 2060, 1985, 1960, 1924 };
	/* odo 10023: narrow bar peaking only 2044 between white */
	static const uint16_t weak_bar[] = {
		230, 240, 283, 319, 449, 783, 1016, 1315, 1833, 2044, 1941, 1509,
		813, 302, 272, 212, 208 };
	/* white paper noise */
	static const uint16_t paper[] = {
		168, 125, 162, 180, 161, 173, 176, 172, 190, 177, 186, 148, 200,
		185, 177, 172, 183, 163, 173, 160, 233, 269, 281, 207 };

#define NOF(a) ((uint32_t)(sizeof(a) / sizeof((a)[0])))
	T_CHECK("BC edge: narrow gaps at ~1430 split the bars (6 flips)",
		edges_of(narrow_gaps, NOF(narrow_gaps), 700, 1313, 1513) == 6);
	T_CHECK("BC edge: line-edge drift 1550-2100 gives no extra edges",
		edges_of(line_edge, NOF(line_edge), 700, 1313, 1513) == 0);
	T_CHECK("BC edge: weak narrow bar (2044) is one bar (2 flips)",
		edges_of(weak_bar, NOF(weak_bar), 700, 1313, 1513) == 2);
	T_CHECK("BC edge: paper noise is no edge",
		edges_of(paper, NOF(paper), 700, 1313, 1513) == 0);
	/* firmware setting since the height change: swing 400, floor 1023 */
	T_CHECK("BC edge 400: narrow gaps still split (6 flips)",
		edges_of(narrow_gaps, NOF(narrow_gaps), 400, 1023, 1223) == 6);
	T_CHECK("BC edge 400: weak narrow bar is one bar (2 flips)",
		edges_of(weak_bar, NOF(weak_bar), 400, 1023, 1223) == 2);
	T_CHECK("BC edge 400: paper noise is no edge",
		edges_of(paper, NOF(paper), 400, 1023, 1223) == 0);
#undef NOF
}

/* linked from map_parse.h / telem_frame.h but defined kernel side */
uint32_t telem_snapshot(char *out, uint32_t size) { (void)out; (void)size; return 0; }

static void test_imu_math(void)
{
	IMU_VEC a = { 0, 0, 1000 }, m;
	int ok = 1, k;

	T_CHECK("atan2 0", imu_atan2_q(0, 1000) == 0);
	T_CHECK("atan2 45", imu_atan2_q(1000, 1000) >= 179 && imu_atan2_q(1000, 1000) <= 181);
	T_CHECK("atan2 90", imu_atan2_q(1000, 0) == 360);
	T_CHECK("atan2 -135", imu_atan2_q(-1000, -1000) >= -541 && imu_atan2_q(-1000, -1000) <= -539);
	T_CHECK("isqrt", imu_isqrt(1000000) == 1000 && imu_isqrt(99) == 9);
	T_CHECK("level roll 0", imu_roll_q(&a) == 0 && imu_pitch_q(&a) == 0);
	a.l = 174; a.u = 985;				/* 10 deg left side up */
	T_CHECK("roll 10 deg", imu_roll_q(&a) >= 38 && imu_roll_q(&a) <= 42);
	a.l = 0; a.u = 1000;
	/* level, field towards north and down; heading for the car turned
	   to k degrees clockwise: forward points at k, north is at -k */
	for (k = 0; k < 360; k += 15) {
		int32_t h, d;
		m.f = (imu_cos_q14(k) * 300) >> 14;	/* north in the car frame */
		m.l = (imu_sin_q14(k) * 300) >> 14;	/* (left = +y) */
		m.u = -400;				/* dip */
		h = imu_heading_deg(&a, &m);
		d = h - k; if (d > 180) d -= 360; if (d < -180) d += 360;
		if (d > 2 || d < -2) { ok = 0; printf("heading %d -> %d\n", k, (int)h); }
	}
	T_CHECK("heading all round", ok);
	/* tilted 15 deg about the forward axis, still north: heading 0 */
	a.f = 0; a.l = 259; a.u = 966;
	m.f = 300; m.l = -104; m.u = -386;	/* north + 400 down, rolled 15 deg */
	{
		int32_t h = imu_heading_deg(&a, &m);
		T_CHECK("heading tilt-compensated", h <= 2 || h >= 358);
	}
	T_CHECK("sin/cos", imu_sin_q14(30) == 8192 && imu_cos_q14(60) == 8192 &&
			   imu_sin_q14(-90) == -16384 && imu_cos_q14(180) == -16384);
}

static void test_map(void)
{
	uint8_t  t[26], u[26], d = 0, du = 0;
	uint32_t m = 0, a, b, i;
	int      same = 1;
	char     out[200];
	const char *p1 = "A=forward,B=left,C=right,D=uturn";
	const char *p2 = "{\"X\":\"Left\",\"q\":\"U-turn\",\"K\":\"straight\",\"m\":\"RIGHT\"}";
	const char *p3 = "right=Z left=Y";
	const char *p4 = "A=uturn,default=left";
	const char *p5 = "B=right else=u_turn";

	T_CHECK("map 4 pairs", map_parse((const uint8_t*)p1, (uint32_t)strlen(p1), t, &m, &d) == 4);
	T_CHECK("map values", t[0] == MP_FORWARD && t[1] == MP_LEFT && t[2] == MP_RIGHT &&
			      t[3] == MP_UTURN && t[4] == MP_FORWARD && m == 0xF);
	T_CHECK("map default absent", d == MP_FORWARD);
	T_CHECK("map JSON", map_parse((const uint8_t*)p2, (uint32_t)strlen(p2), t, &m, &d) == 4 &&
			    t['X' - 'A'] == MP_LEFT && t['Q' - 'A'] == MP_UTURN &&
			    t['K' - 'A'] == MP_FORWARD && t['M' - 'A'] == MP_RIGHT &&
			    t[0] == MP_FORWARD);
	T_CHECK("map word first", map_parse((const uint8_t*)p3, (uint32_t)strlen(p3), t, &m, &d) == 2 &&
				  t[25] == MP_RIGHT && t[24] == MP_LEFT);
	/* the junction default: "default=" and the "else=" alias */
	T_CHECK("map default set", map_parse((const uint8_t*)p4, (uint32_t)strlen(p4), t, &m, &d) == 2 &&
				   t[0] == MP_UTURN && d == MP_LEFT && m == 0x1);
	T_CHECK("map default else", map_parse((const uint8_t*)p5, (uint32_t)strlen(p5), t, &m, &d) == 2 &&
				    t[1] == MP_RIGHT && d == MP_UTURN);
	T_CHECK("map typo rejected", map_parse((const uint8_t*)"A=fwd", 5, t, &m, &d) == MP_ERR_WORD);
	T_CHECK("map no partner", map_parse((const uint8_t*)"A=left B", 8, t, &m, &d) == MP_ERR_PAIR);
	T_CHECK("map default no partner",
		map_parse((const uint8_t*)"A=left default", 14, t, &m, &d) == MP_ERR_PAIR);
	T_CHECK("map empty", map_parse((const uint8_t*)" , ", 3, t, &m, &d) == MP_ERR_EMPTY);

	(void)map_parse((const uint8_t*)p4, (uint32_t)strlen(p4), t, &m, &d);
	map_pack(t, d, &a, &b);
	map_unpack(a, b, u, &du);
	for (i = 0; i < 26; i++) if (t[i] != u[i]) same = 0;
	T_CHECK("map pack round trip", same && du == d);
	/* a word packed before the default existed has 0 there = forward */
	map_unpack(a, b & 0x000FFFFFU, u, &du);
	T_CHECK("map unpack old word", du == MP_FORWARD);

	(void)map_parse((const uint8_t*)p1, (uint32_t)strlen(p1), t, &m, &d);
	(void)map_format(t, m, out, sizeof out);
	T_CHECK("map format", strcmp(out, "A=forward B=left C=right D=uturn") == 0);
}

/* Obstacle bypass (11 Nov): worked examples for flat box faces 10 / 15 /
   20 cm wide, stopped at a 20 cm reading, both sides, and the new message
   packing.  A different result for the same inputs = the geometry changed. */
typedef struct {
	uint8_t	r[7];
	int32_t	edge, step2, along2, lat2, pass, line;
} BG_ROW;

static void bg_check_side(const char *id, int32_t need, int32_t corner,
			  int32_t d1_want, int32_t along1_want, int32_t lat1_want,
			  int32_t f_want, const BG_ROW *rows, int n)
{
	const int32_t xf = 281;			/* reading 20 cm */
	const int32_t S = imu_sin_q14(BG_TURN_DEG), C = imu_cos_q14(BG_TURN_DEG);
	int32_t d1, along, lat, f;
	int i;

	d1 = bg_step1_mm(xf, corner);
	along = d1 * C / 16384;
	lat = d1 * S / 16384;
	f = xf - (US_FACE_TO_AXLE_MM + along);
	T_CHECK(id, d1 == d1_want && along == along1_want && lat == lat1_want && f == f_want);
	for (i = 0; i < n; i++) {
		const BG_ROW *w = &rows[i];
		int32_t e = bg_edge_mm(w->r, lat, f);
		int32_t d2 = bg_step2_mm(e, need, lat);
		int32_t a2 = along + d2 * C / 16384, l2 = lat + d2 * S / 16384;
		int32_t p = bg_pass_mm(xf, e, need, l2, a2, BG_DEPTH_DEFAULT_MM);
		int32_t ln = bg_line_mm(l2);
		int ok = e == w->edge && d2 == w->step2 && a2 == w->along2 &&
			 l2 == w->lat2 && p == w->pass && ln == w->line;

		if (!ok) printf("  %s row %d: edge %d step2 %d along %d lat %d pass %d line %d\n",
				id, i, (int)e, (int)d2, (int)a2, (int)l2, (int)p, (int)ln);
		T_CHECK(id, ok);
	}
}

static void test_bypass_geom(void)
{
	/* pass column: depth BG_DEPTH_DEFAULT_MM 140 (12 Nov; was 200, every pass 60 longer) */
	static const BG_ROW left[] = {
		{ { 63, 63, 63, 63, 63, 63, 63 },  54,  85, 150, 179, 360, 103 },
		{ { 63, 63, 63, 63, 63, 11, 12 },  77, 115, 169, 202, 341, 133 },
		{ { 63, 63, 10, 10, 11, 11, 12 }, 105, 152, 193, 230, 317, 170 },
		{ { 10, 10, 10, 10, 11, 11, 12 }, 120, 172, 206, 245, 304, 189 },
	};
	static const BG_ROW right[] = {
		{ { 63, 63, 63, 63, 63, 13, 13 },  54, 147, 175, 209, 349, 142 },
		{ { 63, 63, 63, 12, 12, 13, 13 },  77, 177, 194, 232, 330, 172 },
		{ { 11, 11, 12, 12, 12, 13, 13 }, 120, 233, 230, 275, 294, 228 },
	};
	static const uint8_t none[7] = { 63, 63, 63, 63, 63, 63, 63 };
	static const uint8_t ahead[7] = { 35, 63, 63, 63, 63, 63, 63 };
	static const uint8_t at30_20[7] = { 63, 63, 63, 63, 63, 63, 20 };
	static const uint8_t at30_30[7] = { 63, 63, 63, 63, 63, 63, 30 };
	const uint8_t vals[7] = { 0, 1, 17, 31, 32, 50, 63 };
	int32_t a, l;
	UW wa = 0, wb = 0, w;
	int i, ok = 1;

	T_CHECK("BG need", BG_NEED_LEFT_MM == 125 && BG_NEED_RIGHT_MM == 155);
	bg_check_side("BG left", BG_NEED_LEFT_MM, BG_CORNER_R_MM, 150, 96, 114, 104,
		      left, (int)(sizeof left / sizeof left[0]));
	bg_check_side("BG right", BG_NEED_RIGHT_MM, BG_CORNER_L_MM, 127, 81, 97, 119,
		      right, (int)(sizeof right / sizeof right[0]));

	/* the depth set from the PC: 1 mm deeper = 1 mm longer pass, clamped */
	T_CHECK("BG depth 200", bg_pass_mm(281, 54, 125, 179, 150, 200) == 420);
	T_CHECK("BG depth 100", bg_pass_mm(281, 54, 125, 179, 150, 100) == 320);
	T_CHECK("BG depth clamped", bg_pass_mm(281, 54, 125, 179, 150, 5000) ==
				    bg_pass_mm(281, 54, 125, 179, 150, BG_DEPTH_MAX_MM));
	T_CHECK("BG step1 too close", bg_step1_mm(200, 169) == -1);
	T_CHECK("BG step2 minimum", bg_step2_mm(54, 125, 200) == 40);
	T_CHECK("BG clear", bg_blocked(none, 125) == 0);
	T_CHECK("BG blocked ahead", bg_blocked(ahead, 125) == 1);
	T_CHECK("BG blocked 30 deg 20 cm", bg_blocked(at30_20, 125) == 1);
	T_CHECK("BG clear 30 deg 30 cm", bg_blocked(at30_30, 125) == 0);
	bg_pose(100, 0, 0, &a, &l);
	T_CHECK("BG pose ahead", a == 100 && l == 0);
	bg_pose(0, 100, 90, &a, &l);
	T_CHECK("BG pose turned 90", a == 100 && l == 0);
	bg_pose(0, 100, 0, &a, &l);
	T_CHECK("BG pose left", a == 0 && l == 100);

	/* packing */
	w = SCAN_PACK(SCAN_SIDE_PROBE, 3, SIDE_PROBE_LEFT);
	T_CHECK("SCAN side probe", SCAN_MODE_OF(w) == SCAN_SIDE_PROBE && SCAN_SEQ(w) == 3 &&
				   SCAN_PARAM(w) == SIDE_PROBE_LEFT);
	T_CHECK("SCAN_SIDE_PROBE = 6", SCAN_SIDE_PROBE == 6 && SCAN_MODE_COUNT <= 8);
	w = SCAN_PACK(SCAN_RAW, 15, 0);		/* TEMPORARY (20 Nov): the last 3-bit mode */
	T_CHECK("SCAN raw", SCAN_RAW == 7 && SCAN_MODE_OF(w) == SCAN_RAW && SCAN_SEQ(w) == 15);
	for (i = 0; i < SIDE_LOOKS; i++) {
		if (i < 5) wa |= SIDE_PUT(i, vals[i]); else wb |= SIDE_PUT(i, vals[i]);
	}
	for (i = 0; i < SIDE_LOOKS; i++) {
		if (SIDE_GET(wa, wb, i) != vals[i]) ok = 0;
	}
	T_CHECK("SIDE round trip", ok);
	T_CHECK("EV_OBST_SIDE / NEAR", EV_OBST_SIDE != EV_OBST_NEAR &&
				       EV_OBST_SIDE < EV_CODE_COUNT && EV_OBST_NEAR < EV_CODE_COUNT);
}

/* Width sweep (12 Nov; per side 13 Nov).  Boundaries are in half-degrees,
   like the obstacle task reports them. */
static int32_t i32abs(int32_t v) { return v < 0 ? -v : v; }

static void test_obst_width(void)
{
	/* 12 Nov calibration: 96 mm and 53 mm boxes, centred, twice each */
	static const OW_POINT real[4] = {
		{ 410, 39, 49, 48, 48 }, { 406, 43, 49, 48, 48 },
		{ 407, 41, 45, 26, 27 }, { 407, 39, 43, 26, 27 },
	};
	static const OW_CAL geo = { 0, 10000, 0, 10000 };	/* plain geometry */
	static const OW_CAL half = { 3500, 5000, 3500, 5000 };
	static const OW_CAL def = { OW_SL_E4, OW_KL_E4, OW_SR_E4, OW_KR_E4 };
	OW_POINT syn[4];
	OW_CAL c;
	int32_t s, k, i, F;
	UW w;

	/* tan at 0.1 deg: tan 20 = 0.36397, 30 = 0.57735, 45 = 1 */
	T_CHECK("OW tan 0", ow_tan_q14(0) == 0);
	T_CHECK("OW tan 20", i32abs(ow_tan_q14(200) - 5963) <= 3);
	T_CHECK("OW tan 30", i32abs(ow_tan_q14(300) - 9459) <= 3);
	T_CHECK("OW tan 35", i32abs(ow_tan_q14(350) - 11472) <= 3);	/* 0.70021 */
	T_CHECK("OW tan 45", i32abs(ow_tan_q14(450) - 16384) <= 3);
	T_CHECK("OW tan -45", i32abs(ow_tan_q14(-450) + 16384) <= 3);
	T_CHECK("OW tan 12.5", i32abs(ow_tan_q14(125) - 3632) <= 3);	/* 0.22169 */

	/* face test: face 350 mm ahead, so OW_PIVOT_MM + 350 from the servo axis */
	T_CHECK("OW face ahead", ow_is_face(350, 0, 350));
	T_CHECK("OW face at 20 deg", ow_is_face(373, 20, 350));	/* (pivot + 373) x cos 20 ~ pivot + 350 */
	T_CHECK("OW face tolerance", ow_is_face(330, 0, 350) && !ow_is_face(300, 0, 350));
	T_CHECK("OW not the face", !ow_is_face(700, 20, 350) && !ow_is_face(4000, 10, 350));
	T_CHECK("OW no reading", !ow_is_face(0, 0, 350) && !ow_is_face(350, 0, 0));

	/* S = 0, K = 1 is plain geometry: edge = F x tan a.
	   Tables: tan 20.5 = 0.37388, tan 12.5 = 0.22169 */
	F = OW_PIVOT_MM + 350;
	T_CHECK("OW edge geometry", i32abs(ow_edge_mm(350, 41, 0, 10000) - F * 37388 / 100000) <= 1);
	T_CHECK("OW width geometry", i32abs(ow_width_mm(350, 41, 25, &geo) -
					   F * (37388 + 22169) / 100000) <= 1);
	/* S 0.35, K 0.5: edge = F x (tan a - 0.35) / 0.5 */
	T_CHECK("OW calibrated edge", i32abs(ow_edge_mm(350, 41, 3500, 5000) -
					    F * 2388 * 2 / 100000) <= 1);
	T_CHECK("OW edge past line", ow_edge_mm(350, 31, 3500, 5000) < 0);
	T_CHECK("OW width impossible", ow_width_mm(350, 31, 31, &half) == -1 &&
				       ow_width_mm(0, 41, 41, &geo) == -1);
	c = half;
	c.kr_e4 = 0;
	T_CHECK("OW width bad K", ow_width_mm(350, 41, 41, &c) == -1);
	/* each side uses its own constants */
	c = half;
	c.sr_e4 = 3000;
	T_CHECK("OW sides separate", ow_width_mm(350, 41, 41, &c) ==
				     ow_edge_mm(350, 41, 3500, 5000) + ow_edge_mm(350, 41, 3000, 5000));
	/* the default constants (20 Nov one-point correction) read the 20 Nov
	   runs (120 mm box, centred, face 394, left 30.5 / right 29.5 deg)
	   as 60 + 60 mm */
	T_CHECK("OW default left edge", i32abs(ow_edge_mm(394, 61, OW_SL_E4, OW_KL_E4) - 60) <= 1);
	T_CHECK("OW default right edge", i32abs(ow_edge_mm(394, 59, OW_SR_E4, OW_KR_E4) - 60) <= 1);
	T_CHECK("OW defaults = 20 Nov runs", i32abs(ow_width_mm(394, 61, 59, &def) - 120) <= 2);

	/* per-side fit: points made from known S / K come back out.  One box
	   shifted sideways: left edge 0..120, right edge 120..0 */
	for (i = 0; i < 4; i++) {
		syn[i].face_mm = 400;
		syn[i].el_half = 30 + 6 * i;
		syn[i].er_half = 48 - 6 * i;
		syn[i].known_l_mm = ow_edge_mm(400, syn[i].el_half, 3400, 5000);
		syn[i].known_r_mm = ow_edge_mm(400, syn[i].er_half, 3600, 7000);
	}
	T_CHECK("OW fit left", ow_fit_side(syn, 4, OW_SIDE_LEFT, &s, &k) == 0 &&
			       i32abs(s - 3400) <= 15 && i32abs(k - 5000) <= 30);
	T_CHECK("OW fit right", ow_fit_side(syn, 4, OW_SIDE_RIGHT, &s, &k) == 0 &&
				i32abs(s - 3600) <= 15 && i32abs(k - 7000) <= 30);
	T_CHECK("OW fit needs 2", ow_fit_side(syn, 1, OW_SIDE_LEFT, &s, &k) == -1);
	/* the 12 Nov runs: each edge moved only 22 mm, under the spread needed */
	T_CHECK("OW fit needs spread", ow_fit_side(real, 4, OW_SIDE_LEFT, &s, &k) == -1 &&
				       ow_fit_side(real, 4, OW_SIDE_RIGHT, &s, &k) == -1);
	/* an edge further out gave a SMALLER boundary: K < 0, rejected */
	syn[0].el_half = 60;
	syn[1].el_half = 58;
	syn[2].el_half = 40;
	syn[3].el_half = 30;
	T_CHECK("OW fit rejects K <= 0", ow_fit_side(syn, 4, OW_SIDE_LEFT, &s, &k) == -1);

	/* "wcal <left> <right>" packing, signed 16 bits each */
	w = WCAL_B(-120, 250);
	T_CHECK("WCAL round trip", WCAL_L(w) == -120 && WCAL_R(w) == 250);
	w = WCAL_B(75, 75);
	T_CHECK("WCAL plain", WCAL_L(w) == 75 && WCAL_R(w) == 75 && w != 0U);
	T_CHECK("REMOTE codes", REMOTE_SERVO_TRIM == 9U && REMOTE_EDGE_SWEEP == 10U &&
				REMOTE_SERVO_END == 11U && REMOTE_RAW_SCAN == 12U);
	/* the message word */
	w = WM_PACK(4095, 127, 59, WM_F_OOV_L | WM_F_NONE);
	T_CHECK("WM round trip", WM_FACE(w) == 4095 && WM_EDGE_L(w) == 127 &&
				 WM_EDGE_R(w) == 59 && WM_FLAGS(w) == (WM_F_OOV_L | WM_F_NONE));
	w = WM_PACK(350, 45, 45, 0);
	T_CHECK("WM plain", WM_FACE(w) == 350 && WM_EDGE_L(w) == 45 &&
			    WM_EDGE_R(w) == 45 && WM_FLAGS(w) == 0);
	T_CHECK("EV_OBST_AHEAD", EV_OBST_AHEAD < EV_CODE_COUNT && EV_OBST_AHEAD != EV_OBST_NEAR &&
				 REMOTE_WIDTH_CAL == 8U && WM_F_OOV_L != WM_F_OOV_R);
}

int main(void)
{
	T_CHECK("bus init", msg_bus_init() == E_OK);
	T_CHECK("flag init", fault_flags_init() == E_OK);
	test_bitfields();
	test_bus_rules();
	test_faults();
	test_frame();
	test_line_search();
	test_barcode();
	test_bc_edge();
	test_map();
	test_imu_math();
	test_bypass_geom();
	test_obst_width();
	(void)last_motion;
	printf("host tests: %d passed, %d failed\n", pass, fail);
	return fail ? 1 : 0;
}
