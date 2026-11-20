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
#define BC_END_GAP_MS		40
/* 12 Nov: sized for line speeds down to 25 mm/s.  Before the timer fix
   (kernel cpu_clock.c) every ms here was really ~43 ms, so the old
   400 / 2500 never limited anything.  At 30 mm/s a 9 mm wide bar takes
   300 ms and a 141 mm barcode ~4.7 s, so the old values would now
   reject every barcode. */
#define BC_MAX_EL_MS		600
#define BC_MAX_FRAME_MS		7000
#define BC_RATIO_X10		15	/* wide must be >= 1.5 x narrow */
#define BC_BUSY_HOLD_MS		400	/* "busy" this long after the last edge */

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

LOCAL UINT action_of(char c)
{
	switch (c) {
	case 'A': return BC_ACT_LEFT;
	case 'B': return BC_ACT_RIGHT;
	case 'C': return BC_ACT_UTURN;
	case 'D': return BC_ACT_END;
	default:  return BC_ACT_NONE;
	}
}

LOCAL const char *err_text(INT e)
{
	switch (e) {
	case C39_ERR_LENGTH:  return "wrong number of bars";
	case C39_ERR_RATIO:   return "wide/narrow unclear";
	case C39_ERR_UNKNOWN: return "unknown pattern";
	case C39_ERR_FRAME:   return "no * start/stop";
	default:              return "too long";
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

	(void)stacd; (void)exinf;
	gpio_set_pin(PIN_BARCODE, GPIO_MODE_IN);
	CHECK(gpio_irq_attach(PIN_BARCODE, GPIO_EV_BOTH, bc_isr), "barcode irq");	/* core 0 */

	for (;;) {
		UW now;

		tk_slp_tsk(20);
		tk_can_wup(TSK_SELF);

		/* 1. take the edges the ISR has stamped */
		while (ring_tail != ring_head) {
			EDGE e;
			e.t = ring[ring_tail].t;
			e.black = ring[ring_tail].black;
			ring_tail = (ring_tail + 1U) & (BC_RING - 1U);
			last_edge_ms = now_ms();		/* for barcode_busy() */

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

		/* 2. frame over? (white for long enough after a black -> white) */
		if (!in_frame) continue;
		now = now_ms();
		if (widest > BC_MAX_EL_MS * 1000U || (now - t_start) > BC_MAX_FRAME_MS) {
			in_frame = FALSE;			/* the line or a junction */
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
			if (r > 0) {
				for (i = 0; i <= r; i++) info.text[i] = txt[i];
				info.action = action_of(txt[0]);
				info.reversed = rev != 0;
				info.t_ms = now;
				info.good++;
				info.seq++;
				tm_printf((UB*)"BARCODE \"%s\"%s -> action %d\n", txt,
					  rev ? " (read backwards)" : "", info.action);
				mission_event(EV_BARCODE);
			} else if (n - 1 >= 9) {		/* ignore single specks */
				info.bad++;
				tm_printf((UB*)"BARCODE not read: %s (%d elements)\n", err_text(r), n - 1);
			}
		}
	}
}

EXPORT ER barcode_start(void)
{
	ID tid = app_start_task((FP)barcode_task, PRI_BARCODE, CORE_BARCODE, "barcode");
	if (tid < E_OK) return tid;
	bc_tid = tid;
	return E_OK;
}
