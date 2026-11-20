/*
 *	barcode.c  (Buddy 3: Code 39 barcodes on GP2)
 *
 *	ISR (shared GPIO dispatcher, core 0): stamps every edge of the
 *	barcode sensor's D0 into a ring and wakes the barcode task.  Nothing
 *	else; the timestamp comes from the dispatcher (taken once on entry).
 *	Ring: the ISR owns 'head', the task owns 'tail' (one writer each).
 *
 *	barcode task (priority 15, core 0):
 *	  WAIT     for a white -> black edge
 *	  MEASURE  collect edges; drop glitches shorter than BC_MIN_EL_US
 *	  DECODE   once the sensor has seen white for 2 x the widest element
 *	           (at least BC_END_GAP_MS): widths -> code39_decode()
 *	A frame that runs longer than BC_MAX_FRAME_MS, or an element longer
 *	than BC_MAX_EL_MS, is not a barcode (the main line, a junction) and is
 *	thrown away.
 *
 *	Action of the first character (team table, CHECK against the course):
 *	  A = turn left at the next junction    B = turn right
 *	  C = U-turn                            D = end of course (stop)
 *	  anything else: just reported (console + robot/tlm/line)
 */
#include "../car.h"
#include "../gpio_irq.h"
#include "code39.h"

#define BC_RING			128	/* power of two */
#define BC_MAX_EDGES		(10 * 10)	/* up to 10 characters incl. the two '*' */
#define BC_MIN_EL_US		300	/* shorter = noise */
/* 16 Nov: was 40.  The frame ended after 2 x the widest element so far,
   which right after the start (widest = one space) cut the barcode off
   whenever the car or a hand slowed down for one narrow space (bench log:
   bar 123, space 205, bar 121 ms, then given up).  A barcode is followed
   by a white quiet zone of 10 narrow widths, so waiting 0.8 s is safe. */
#define BC_END_GAP_MS		800
/* 12 Nov: sized for line speeds down to 25 mm/s.  Before the timer fix
   (kernel cpu_clock.c) every ms here was really ~43 ms, so the old
   400 / 2500 never limited anything.  At 30 mm/s a 9 mm wide bar takes
   300 ms and a 141 mm barcode ~4.7 s, so the old values would now
   reject every barcode. */
/* 12 Nov evening: the course barcodes go up to ~333 mm long with
   21 mm wide bars (BARCODE.pptx, largest size).  At 25 mm/s that is a
   856 ms bar and a 13.3 s barcode, so 600 / 7000 rejected them.  The
   sensor sits BESIDE the line, so it never sees the main line itself. */
#define BC_MAX_EL_MS		1500
#define BC_MAX_FRAME_MS		16000
#define BC_RATIO_X10		15	/* wide must be >= 1.5 x narrow */
#define BC_BUSY_HOLD_MS		400	/* "busy" this long after the last edge */
/* 16 Nov: sensor check on the console.  1 = once a second, if the sensor
   changed at all: how many black/white changes and black bars it saw;
   and a line for every barcode attempt that is thrown away, with why.
   Set 0 once the barcodes read reliably. */
#define BC_DIAG			1
#define BC_DIAG_MS		1000
/* 16 Nov: the sensor's comparator chatters near its threshold (bench log:
   300-600 changes a second, a barcode has ~30).  A change only counts once
   the sensor has held the new colour this long; a burst of chatter becomes
   one edge in the middle of the burst, a burst that ends on the old colour
   is dropped.  10 ms is still under half the narrowest bar at 250 mm/s
   (5 mm bar = 20 ms). */
#define BC_DEBOUNCE_US		10000U

typedef struct { UW t; UB black; } EDGE;

LOCAL volatile EDGE ring[BC_RING];
LOCAL volatile UINT ring_head, ring_tail;
LOCAL volatile UW ring_lost;
LOCAL ID bc_tid;
LOCAL BARCODE_INFO info;		/* written by the barcode task only */
LOCAL volatile UW last_edge_ms;
LOCAL volatile BOOL in_frame;

EXPORT void barcode_get(BARCODE_INFO *b) { *b = info; }	/* copy; may mix one update */
EXPORT BOOL barcode_busy(void)
{
	return in_frame || (now_ms() - last_edge_ms) < BC_BUSY_HOLD_MS;
}

LOCAL void bc_isr(UINT gpio, UINT events, UW t)
{
	UINT h = ring_head, nh = (h + 1U) & (BC_RING - 1U);
	BOOL level = gpio_get_val(gpio) != 0;

	(void)events;
	if (nh == ring_tail) { ring_lost++; return; }
	ring[h].t = t;
	ring[h].black = (level == (BARCODE_BLACK_IS_HIGH != 0)) ? 1U : 0U;
	ring_head = nh;
	if (bc_tid > 0) tk_wup_tsk(bc_tid);
}

/* ---- BARCODE_ANALOG: A0 samples -> edges (ADC interrupt, core 0) ----
   A half-seen narrow bar never gets as dark as a wide one, so a fixed
   black/white level misses it (simulated: 0 of 15 reads with a 2.5 mm
   blur).  Instead each bar is found as a peak and each gap as a dip:
     - average 4 samples (4 ms) against noise
     - learn the white..black range (hi/lo jump to new extremes, drift
       together over ~2 s) and the noise (average sample-to-sample step)
     - BLACK once the signal has risen H above its lowest point since the
       last switch, WHITE once it has fallen H below its highest point;
       H = a fifth of the range, at least 3 x the noise
   Nothing switches until the range is BC_ADC_MIN_SPAN and twice H. */
#define BC_ADC_AVG		4
LOCAL UH   a_hist[BC_ADC_AVG];
LOCAL UINT a_hn, a_hi_idx;
LOCAL INT  a_prev = -1, a_nz8;		/* noise: mean |step|, x256 */
LOCAL INT  a_hi8 = -1, a_lo8;		/* black / white level, x256 */
LOCAL INT  a_ext;			/* lowest (white) / highest (black) since the switch */
LOCAL UB   a_black;			/* current colour from A0 */
LOCAL volatile UH a_now, a_min, a_max;	/* BC_DIAG: this second's range */
LOCAL volatile UH a_h;			/* BC_DIAG: current H */

EXPORT void barcode_adc_sample(UINT raw, UW t_us)
{
	INT v = (INT)(BARCODE_A0_BLACK_IS_HIGH ? raw : 4095U - raw);
	INT s, span, h, nz;
	UINT i, sum = 0;

	if (a_prev >= 0) {
		INT d = v - a_prev;
		a_nz8 += (((d < 0 ? -d : d) << 8) - a_nz8) >> 6;
	}
	a_prev = v;
	a_hist[a_hi_idx] = (UH)v;
	a_hi_idx = (a_hi_idx + 1U) % BC_ADC_AVG;
	if (a_hn < BC_ADC_AVG) a_hn++;
	for (i = 0; i < a_hn; i++) sum += a_hist[i];
	s = (INT)(sum / a_hn);

	a_now = (UH)s;
	if (s < (INT)a_min) a_min = (UH)s;
	if (s > (INT)a_max) a_max = (UH)s;
	if (a_hi8 < 0) { a_hi8 = a_lo8 = s << 8; a_ext = s; return; }
	if ((s << 8) > a_hi8) a_hi8 = s << 8;
	if ((s << 8) < a_lo8) a_lo8 = s << 8;
	span = a_hi8 - a_lo8;
	a_hi8 -= span >> 11;			/* ~2 s at 1000 samples/s */
	a_lo8 += span >> 11;
	span = (a_hi8 - a_lo8) >> 8;
	nz = a_nz8 >> 8;
	h = span / 5;
	if (h < 3 * nz) h = 3 * nz;
	if (h < 40) h = 40;
	a_h = (UH)h;

	if (!a_black) { if (s < a_ext) a_ext = s; }
	else          { if (s > a_ext) a_ext = s; }
	if (span < BC_ADC_MIN_SPAN || span < 2 * h) return;	/* no real contrast yet */
	if (!a_black ? (s > a_ext + h) : (s < a_ext - h)) {
		UINT hd = ring_head, nh = (hd + 1U) & (BC_RING - 1U);
		a_black = (UB)!a_black;
		a_ext = s;
		if (nh == ring_tail) { ring_lost++; return; }
		ring[hd].t = t_us - (BC_ADC_AVG / 2U) * (1000000U / BC_ADC_RATE_HZ);	/* average lags */
		ring[hd].black = a_black;
		ring_head = nh;
		if (bc_tid > 0) tk_wup_tsk(bc_tid);
	}
}

/* letter -> action.  The map lives in RAM so the GUI can set it over
   MQTT (robot/map, retained: the car gets it again on every reconnect).
   Until a map arrives it is BC_ACTION_MAP from car_config.h.  Letters the
   map doesn't name, digits and symbols all mean BC_ACT_NONE: drive straight
   on.  Only the comms task writes (barcode_set_map); the barcode task reads
   one byte per decode, and a byte store is atomic, so no lock is needed. */
LOCAL volatile UB act_map[BC_MAP_LETTERS];
LOCAL volatile BOOL map_from_gui;
LOCAL volatile BOOL map_ready;

LOCAL void map_boot_default(void)
{
	static const char def[] = BC_ACTION_MAP;
	UINT i;
	for (i = 0; i < BC_MAP_LETTERS; i++) {
		char m = (i < sizeof def - 1U) ? def[i] : '-';
		act_map[i] = (UB)(m == 'L' ? BC_ACT_LEFT : m == 'R' ? BC_ACT_RIGHT :
				  m == 'U' ? BC_ACT_UTURN : m == 'S' ? BC_ACT_END : BC_ACT_NONE);
	}
	map_ready = TRUE;
}

EXPORT void barcode_set_map(const UB act[BC_MAP_LETTERS])
{
	UINT i;
	if (!map_ready) map_boot_default();
	for (i = 0; i < BC_MAP_LETTERS; i++)
		act_map[i] = (act[i] <= BC_ACT_END) ? act[i] : (UB)BC_ACT_NONE;
	map_from_gui = TRUE;
}

EXPORT BOOL barcode_get_map(UB act[BC_MAP_LETTERS])
{
	UINT i;
	if (!map_ready) map_boot_default();
	for (i = 0; i < BC_MAP_LETTERS; i++) act[i] = act_map[i];
	return map_from_gui;
}

LOCAL UINT action_of(char c)
{
	if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
	if (c < 'A' || c > 'Z') return BC_ACT_NONE;
	return act_map[c - 'A'];
}
LOCAL const char *const act_name[] = { "straight on (not mapped)", "LEFT at the next junction",
	"RIGHT at the next junction", "U-TURN", "STOP (end of course)" };

LOCAL const char *err_text(INT e)
{
	switch (e) {
	case C39_ERR_LENGTH:  return "wrong number of bars";
	case C39_ERR_RATIO:   return "wide/narrow unclear";
	case C39_ERR_UNKNOWN: return "unknown pattern";
	case C39_ERR_FRAME:   return "no * start/stop";
	case C39_ERR_FULLASCII: return "bad Full ASCII pair";
	default:              return "too long";
	}
}

/* ---- debounce: raw ISR edges -> clean edges ---- */
LOCAL UB  db_level;			/* accepted colour, 1 = black */
LOCAL BOOL db_pend;			/* a burst of raw edges is settling */
LOCAL UW  db_t0, db_t1;			/* first / last raw edge of the burst */
LOCAL UB  db_last;			/* colour read at the last raw edge */
LOCAL UW  d_changes, d_bars, d_clean;	/* BC_DIAG counters */

LOCAL UB pin_black(void)
{
	if (BARCODE_READ_A0) return a_black;
	return ((gpio_get_val(PIN_BARCODE) != 0) == (BARCODE_BLACK_IS_HIGH != 0)) ? 1U : 0U;
}

LOCAL BOOL db_settle(UB level, EDGE *out)
{
	db_pend = FALSE;
	if (level == db_level) return FALSE;	/* chatter that came back: nothing */
	db_level = level;
	out->t = db_t0 + (db_t1 - db_t0) / 2U;
	out->black = level;
	d_clean++;
#if BC_DIAG
	{	/* every real change, with how long the colour before it lasted */
		static UW prev_t;
		tm_printf((UB*)"BARCODE  %s  (%s lasted %d ms)\n", level ? "-> BLACK" : "-> WHITE",
			  level ? "white" : "black", (INT)((out->t - prev_t) / 1000U));
		prev_t = out->t;
	}
#endif
	return TRUE;
}

/* next clean edge, or FALSE if none is ready yet */
LOCAL BOOL next_edge(EDGE *out)
{
	for (;;) {
		if (ring_tail != ring_head) {
			UW t = ring[ring_tail].t;
			if (db_pend && (t - db_t1) >= BC_DEBOUNCE_US) {
				/* quiet gap before this edge: the burst settled on db_last */
				if (db_settle(db_last, out)) return TRUE;
				continue;
			}
			db_last = ring[ring_tail].black;
			ring_tail = (ring_tail + 1U) & (BC_RING - 1U);
			last_edge_ms = now_ms();	/* for barcode_busy() */
			d_changes++;
			if (db_last) d_bars++;
			if (!db_pend) { db_pend = TRUE; db_t0 = t; }
			db_t1 = t;
			continue;
		}
		if (db_pend && (now_us() - db_t1) >= BC_DEBOUNCE_US)
			return db_settle(pin_black(), out);	/* quiet now: read the pin */
		return FALSE;
	}
}

LOCAL void barcode_task(INT stacd, void *exinf)
{
	static UW et[BC_MAX_EDGES];		/* edge times of the current frame */
	static uint32_t wid[BC_MAX_EDGES];
	INT n = 0;
	UB last_black = 0;
	UW widest = 0, t_start = 0;
	BOOL broken = FALSE;
#if BC_DIAG
	UW d_t = 0;
#endif

	(void)stacd; (void)exinf;
	gpio_set_pin(PIN_BARCODE, GPIO_MODE_IN);
	db_level = pin_black();
#if BARCODE_READ_A0
	a_min = 4095; a_max = 0;
	if (adc_scan_start())
		tm_printf((UB*)"BARCODE analog A0 on GP%d (ADC scan 1000/s per pin, filter %d ms)\n",
			  PIN_BARCODE_ADC, (INT)(BC_DEBOUNCE_US / 1000U));
	else
		tm_printf((UB*)"BARCODE analog A0: ADC scan did NOT start, no barcodes\n");
	(void)bc_isr;
#else
	/* no pull resistors: the input may be the module's A0 (analog) wired
	   straight to this digital pin, and a pull would shift its level;
	   Schmitt trigger on, so the pin itself has some hysteresis */
	clr_w(GPIO(PIN_BARCODE), GPIO_PUE | GPIO_PDE);
	set_w(GPIO(PIN_BARCODE), 1U << 1);	/* PADS_BANK0 SCHMITT */
	CHECK(gpio_irq_attach(PIN_BARCODE, GPIO_EV_BOTH, bc_isr), "barcode irq");	/* core 0 */
#endif
#if BC_DIAG
	tm_printf((UB*)"BARCODE sensor on GP%d reads %s now (filter %d ms)\n",
		  BARCODE_READ_A0 ? PIN_BARCODE_ADC : PIN_BARCODE,
		  pin_black() ? "BLACK" : "WHITE", (INT)(BC_DEBOUNCE_US / 1000U));
	d_t = now_ms();
#endif

	for (;;) {
		UW now;

		/* settling a burst: come back as soon as it can be decided */
		tk_slp_tsk(db_pend ? (BC_DEBOUNCE_US / 1000U + 1U) : 20);
		tk_can_wup(TSK_SELF);

		/* 1. take the (debounced) edges */
		for (;;) {
			EDGE e;
			if (!next_edge(&e)) break;

			if (!in_frame) {
				if (!e.black) continue;		/* wait for white -> black */
				in_frame = TRUE;
				n = 0; widest = 0; broken = FALSE;
				t_start = now_ms();
				et[n++] = e.t;
				last_black = 1;
				continue;
			}
			if (e.black == last_black) { broken = TRUE; continue; }	/* missed an edge */
			if (n > 0 && (e.t - et[n - 1]) < BC_MIN_EL_US) {
				n--;				/* glitch: undo the last edge */
				last_black = e.black;
				if (n == 0) in_frame = FALSE;
				continue;
			}
			if (n >= BC_MAX_EDGES) { broken = TRUE; continue; }
			if (n > 0 && (e.t - et[n - 1]) > widest) widest = e.t - et[n - 1];
			et[n++] = e.t;
			last_black = e.black;
		}

#if BC_DIAG
		if (now_ms() - d_t >= BC_DIAG_MS) {
			if (d_changes > 0)
				tm_printf((UB*)"BARCODE sensor: %d raw changes (%d black) -> %d real after filtering, in %d ms, now %s\n",
					  (INT)d_changes, (INT)d_bars, (INT)d_clean, (INT)(now_ms() - d_t),
					  pin_black() ? "BLACK" : "WHITE");
#if BARCODE_READ_A0
			{	/* A0 range this second; printed when it moved, or every 5 s */
				static UINT quiet;
				UINT mn = a_min, mx = a_max;
				INT hi = a_hi8 >> 8, lo = a_lo8 >> 8;
				a_min = 4095; a_max = 0;
				if (mx >= mn && (mx - mn > 60 || ++quiet >= 5)) {
					quiet = 0;
					tm_printf((UB*)"BARCODE A0: now %d, this second %d..%d | learned white %d black %d, noise %d, step %d%s\n",
						  (INT)a_now, (INT)mn, (INT)mx, lo, hi, (INT)(a_nz8 >> 8), (INT)a_h,
						  ((hi - lo) < BC_ADC_MIN_SPAN || (hi - lo) < 2 * (INT)a_h) ? " - not enough contrast yet" : "");
				}
			}
#endif
			d_changes = 0; d_bars = 0; d_clean = 0; d_t = now_ms();
		}
#endif

		/* 2. frame over? (white for long enough after a black -> white) */
		if (!in_frame) continue;
		now = now_ms();
		if (widest > BC_MAX_EL_MS * 1000U || (now - t_start) > BC_MAX_FRAME_MS) {
			in_frame = FALSE;			/* the line or a junction */
#if BC_DIAG
			if (n >= 3)
				tm_printf((UB*)"BARCODE ignored after %d black bars: %s\n", n / 2,
					  widest > BC_MAX_EL_MS * 1000U
					  ? "one bar/space took over 1.5 s (too slow, or the line / a junction)"
					  : "took over 16 s (too slow)");
#endif
			continue;
		}
		{
			UW quiet_us = now_us() - et[n - 1];	/* white since the last edge */
			if (last_black || quiet_us < BC_END_GAP_MS * 1000U || quiet_us < 2U * widest)
				continue;
		}

		/* 3. decode */
		in_frame = FALSE;
		if (n < 2) continue;
		{
			INT i, r;
			int32_t rev;
			char txt[BC_MAX_TEXT + 1];

			for (i = 0; i < n - 1; i++) wid[i] = et[i + 1] - et[i];
			r = broken ? C39_ERR_LENGTH
				   : code39_decode(wid, n - 1, BC_RATIO_X10, txt, sizeof txt, &rev);
			if (r > 0) r = code39_full_ascii(txt, txt, sizeof txt);	/* Full ASCII */
			if (r > 0) {
				for (i = 0; i <= r; i++) info.text[i] = txt[i];
				info.action = action_of(txt[0]);
				info.reversed = rev != 0;
				info.t_ms = now;
				info.good++;
				info.seq++;
				tm_printf((UB*)"BARCODE \"%s\"%s -> %s\n", txt,
					  rev ? " (read backwards)" : "", act_name[info.action]);
				mission_event(EV_BARCODE);
			} else if (n - 1 >= (BC_DIAG ? 3 : 9)) {	/* ignore single specks */
				info.bad++;
				tm_printf((UB*)"BARCODE not read: %s (%d black bars, *A* has 15; %d elements, need 29)%s\n",
					  err_text(r), n / 2, n - 1, broken ? " [an edge was missed]" : "");
				/* every bar / space in ms, to see what went wrong:
				   wide ones should be ~3 x the narrow ones */
				tm_printf((UB*)"BARCODE widths ms (bar space bar ...):");
				for (i = 0; i < n - 1 && i < 60; i++) tm_printf((UB*)" %d", (INT)(wid[i] / 1000U));
				tm_printf((UB*)"\n");
			}
		}
	}
}

EXPORT ER barcode_start(void)
{
	if (!map_ready) map_boot_default();
	ID tid = app_start_task((FP)barcode_task, PRI_BARCODE, CORE_BARCODE, "barcode");
	if (tid < E_OK) return tid;
	bc_tid = tid;
	return E_OK;
}
