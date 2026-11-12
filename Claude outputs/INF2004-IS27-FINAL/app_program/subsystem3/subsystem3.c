/*
 *	subsystem3.c  (Buddy 3: line sensors)
 *	line task: woken every 20 ms by ctl_tick (same tick as motion).
 *	  1. read both IR sensors (ADC0 = left, ADC1 = right)
 *	  2. remove each sensor's white level (learned while stopped)
 *	  3. always publish the readings (mission uses them to re-find the
 *	     line after an obstacle)
 *	  4. when following: line_ctl_step() -> motion slot LINE
 *	  5. three junction samples in a row (both black) -> EV_JUNCTION for
 *	     the mission (barcode turns), unless barcode bars are passing
 *	The ADC is only touched here.  The barcode reader is barcode.c.
 */
#include "../car.h"
#include "line_ctl.h"

/* same mode numbers as line_ctl.c */
CC_ASSERT(LINE_M_TRACK == 1 && LINE_M_CENTRED == 2 && LINE_M_RECOVER == 3 &&
	  LINE_M_JUNCTION == 4 && LINE_M_LOST == 5 && LINE_M_BENDSTOP == 6 && LINE_M_PIVOT == 7,
	  "line mode numbers differ from line_ctl.c");

#define WHITE_CAP	600	/* learned white level never above this */
#define WHITE_ALPHA	8	/* 1/8 per sample while learning */

EXPORT ID line_task_id = 0;

/* ---- mission -> line: one word ----
   bit 0 on, bits 1..2 side (0 none, 1 left, 2 right), bit 3 deep,
   bits 8..15 generation (a new command restarts the controller) */
LOCAL volatile UW line_cmd;
LOCAL UINT cmd_gen;
LOCAL volatile UINT see_bits;
/* layout found at START: 0 straddle, 1 inside, 2 edge (LINE_EDGE) */
LOCAL volatile UINT layout = LINE_LAYOUT_DEFAULT;
LOCAL INT edge_dir = 1;			/* edge: +1 line under the left sensor */
EXPORT BOOL line_inside(void) { return layout != 0; }	/* both black is not a junction */
LOCAL const char *const layout_name[] = { "STRADDLE (sensors either side)",
	"INSIDE (both sensors on the line)", "EDGE (one sensor on black, one on white)" };

LOCAL volatile INT speed_req = 0;	/* 0 = use the tuning's own speed */
LOCAL volatile INT bend_req = LINE_BEND_DEFAULT, kp_req = 0;
LOCAL volatile UINT tune_gen, tune_seen;
LOCAL volatile INT speed_now, bend_now, kp_now;
LOCAL volatile BOOL pivot_req = LINE_PIVOT_DEFAULT;
EXPORT void line_set_pivot(BOOL on) { pivot_req = on ? TRUE : FALSE; tune_gen++; }
EXPORT BOOL line_get_pivot(void) { return pivot_req; }
EXPORT void line_set_speed(INT mms) { speed_req = CLAMP(mms, LINE_SPEED_MIN, LINE_SPEED_MAX); tune_gen++; }
EXPORT INT  line_get_speed(void) { return speed_now; }
EXPORT void line_set_tuning(INT mms, INT bend, INT kp)
{
	if (mms)  speed_req = CLAMP(mms, LINE_SPEED_MIN, LINE_SPEED_MAX);
	if (bend) bend_req = CLAMP(bend, 20, 100);
	if (kp)   kp_req = CLAMP(kp, 300, 3000);
	tune_gen++;
}
EXPORT void line_get_tuning(INT *mms, INT *bend, INT *kp) { *mms = speed_now; *bend = bend_now; *kp = kp_now; }

/* working copies of the three tunings, so the speed can change live */
LOCAL LINE_TUNE tune_straddle, tune_inside, tune_edge;
LOCAL LINE_TUNE *tune_of(UINT lay)
{
	return lay == LINE_EDGE ? &tune_edge : lay == 1 ? &tune_inside : &tune_straddle;
}
LOCAL void apply_speed(void)
{
	INT sp = speed_req ? speed_req : LINE_SPEED_DEFAULT;
	/* bend speed: % of the line speed, but never below LINE_BEND_FLOOR
	   (the motors may stall) and never above the line speed */
	INT lo = sp < LINE_BEND_FLOOR ? sp : LINE_BEND_FLOOR;
	INT bend_mms = CLAMP(sp * bend_req / 100, lo, sp);
	UINT i;

	for (i = 0; i < 3; i++) {
		LINE_TUNE *k = tune_of(i);
		k->speed = sp;
		k->speed_min = bend_mms;
		if (kp_req) k->kp_x1000 = kp_req;
	}
	/* bends: stop + pivot on the inner wheel, or steer through */
	tune_inside.pivot_steer   = pivot_req ? PIVOT_STEER_INSIDE : 0;
	tune_straddle.pivot_steer = pivot_req ? PIVOT_STEER_STRADDLE : 0;
	tune_edge.pivot_steer     = pivot_req ? PIVOT_STEER_EDGE : 0;
	tune_edge.edge_dir = edge_dir;
	speed_now = sp;
	bend_now = bend_req;
	kp_now = tune_of(layout)->kp_x1000;
}
LOCAL volatile UW loop_max;
EXPORT UW line_loop_us_max(void) { UW v = loop_max; loop_max = 0; return v; }
LOCAL LINE_STATUS ls;

EXPORT void line_follow(BOOL on, INT side, BOOL deep)
{
	UW s = side < 0 ? 1U : side > 0 ? 2U : 0U;
	cmd_gen = (cmd_gen + 1U) & 0xFFU;		/* only the mission calls this */
	line_cmd = (on ? 1U : 0U) | (s << 1) | (deep ? 8U : 0U) | ((UW)cmd_gen << 8);
}

EXPORT void line_get(LINE_STATUS *st) { *st = ls; }	/* debug copy */
EXPORT UINT line_see_bits(void) { return see_bits; }

/* ---------------- ADC (direct registers) ---------------- */
#define ADC_CS		(0x4004C000U + 0x00U)
#define ADC_RESULT	(0x4004C000U + 0x04U)
#define ADC_EN		(1U << 0)
#define ADC_START_ONCE	(1U << 2)
#define ADC_READY	(1U << 8)
#define ADC_AINSEL(ch)	((UW)(ch) << 12)
#ifndef FUNCSEL_NULL
#define FUNCSEL_NULL	31U
#endif

LOCAL void analog_pin(UINT pin)
{
	out_w(GPIO_CTRL(pin), FUNCSEL_NULL);		/* no digital function */
	clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);	/* no input buffer, no pulls */
	set_w(GPIO(pin), GPIO_OD);
}

LOCAL UINT adc_read(UINT ch)
{
	UINT spin = 0;
	UINT v;

	out_w(ADC_CS, ADC_EN | ADC_AINSEL(ch) | ADC_START_ONCE);
	while (!(in_w(ADC_CS) & ADC_READY)) {
		if (++spin > 2000) return 0;		/* ~2 us normally */
	}
	v = in_w(ADC_RESULT) & 0xFFFU;
	return LINE_BLACK_IS_HIGH ? v : 4095U - v;
}

/* ---------------- line task ---------------- */
LOCAL void line_task(INT stacd, void *exinf)
{
	LINE_CTL ctl;
	LINE_OUT out = {0};
	UINT white[2] = {WHITE_CAP / 2, WHITE_CAP / 2};
	UINT seen_gen = 0xFFFFU;
	UINT junc_n = 0;
	UW t_wake;
	BOOL on = FALSE;
	UINT i;

	(void)stacd; (void)exinf;
	analog_pin(PIN_LINE_L);
	analog_pin(PIN_LINE_R);
	set_w(ADC_CS, ADC_EN);
	tune_straddle = line_tune_default;
	tune_inside = line_tune_inside;
	tune_edge = line_tune_edge;
	apply_speed();
	line_ctl_reset(&ctl, &tune_straddle, 0, 0, now_ms());

	for (;;) {
		UINT raw[2], dark[2];
		UW cmd;

		tk_slp_tsk(TMO_FEVR);		/* ctl_tick wakes us every 20 ms */
		t_wake = now_us();
		tk_can_wup(TSK_SELF);		/* drop any backlog: no catch-up bursts */

		raw[0] = adc_read(PIN_LINE_L - 26);
		raw[1] = adc_read(PIN_LINE_R - 26);

		/* command from the mission */
		cmd = line_cmd;
		if (((cmd >> 8) & 0xFFU) != seen_gen) {
			UINT s = (cmd >> 1) & 3U;
			seen_gen = (cmd >> 8) & 0xFFU;
			if (on && !(cmd & 1U))
				motion_drive(MOT_OWNER_LINE, 0, 0);	/* no stale command left in the slot */
			on = (cmd & 1U) != 0;
			out.lost = 0;
			/* START (no seed side): look where the sensors are now.
			   Both black = INSIDE layout; both white = STRADDLE;
			   one black + one white = EDGE (follow the line's edge).
			   Restarts after an obstacle or a barcode turn keep the
			   layout found at START. */
			if ((cmd & 1U) && s == 0) {
				UINT see0 = (raw[0] >= 2000U ? 1U : 0U) | (raw[1] >= 2000U ? 2U : 0U);
				UINT was = layout;
				if (see0 == 3) layout = 1;
				else if (see0 == 0) layout = 0;
				else { layout = LINE_EDGE; edge_dir = see0 == 1 ? 1 : -1; }
				tune_edge.edge_dir = edge_dir;
				tm_printf((UB*)"LINE layout: %s%s%s\n", layout_name[layout],
					  layout == LINE_EDGE ? (edge_dir > 0 ? ", line under the LEFT sensor"
									 : ", line under the RIGHT sensor") : "",
					  was != layout ? " [changed]" : "");
			}
			kp_now = tune_of(layout)->kp_x1000;
			line_ctl_reset(&ctl, tune_of(layout),
				       s == 1 ? -1 : s == 2 ? 1 : 0, (cmd & 8U) != 0, now_ms());
		}

		/* white level: learned only while NOT following (car parked
		   on white before START), so a line under a sensor during the
		   run can't be learned as "white" */
		for (i = 0; i < 2; i++) {
			if (!on && raw[i] < WHITE_CAP && !(layout == 1 && raw[i ^ 1U] >= 2000U))
				white[i] = (white[i] * (WHITE_ALPHA - 1) + raw[i]) / WHITE_ALPHA;
			dark[i] = raw[i] > white[i]
				? ((raw[i] - white[i]) * 4095U) / (4095U - white[i]) : 0U;
		}
		see_bits = (dark[0] >= (UINT)line_tune_default.see ? 1U : 0U) |
			   (dark[1] >= (UINT)line_tune_default.see ? 2U : 0U);

		if (tune_gen != tune_seen) {
			tune_seen = tune_gen;
			apply_speed();
			tm_printf((UB*)"LINE speed %d mm/s, bends %d mm/s (%d %%), steering KP %d, bends: %s\n",
				  speed_now, tune_edge.speed_min, bend_now, kp_now,
				  pivot_req ? "stop + pivot on the inner wheel" : "steer through");
		}
		if (on) {
			line_ctl_step(&ctl, (int32_t)dark[0], (int32_t)dark[1], now_ms(), &out);
			if (out.lost) {
				on = FALSE;			/* stop driving the slot */
				motion_drive(MOT_OWNER_LINE, 0, 0);
				mission_event(EV_LINE_LOST);
			} else {
				motion_drive(MOT_OWNER_LINE, out.speed, out.steer);
			}
			if (out.mode == LINE_M_JUNCTION) {
				if (++junc_n == 3 && !barcode_busy()) mission_event(EV_JUNCTION);
			} else {
				junc_n = 0;
			}
		} else {
			out.mode = LINE_M_OFF;
			out.steer = out.speed = 0;
			out.pos = (((INT)dark[1] - (INT)dark[0]) * 1000) / 4095;
		}

		for (i = 0; i < 2; i++) {
			ls.raw[i] = raw[i];
			ls.dark[i] = dark[i];
			ls.see[i] = (see_bits >> i) & 1U;
		}
		ls.pos = out.pos;
		ls.steer = out.steer;
		ls.speed = out.speed;
		ls.set_speed = speed_now;
		ls.mode = on ? (UINT)out.mode : (out.lost ? LINE_M_LOST : LINE_M_OFF);
		ls.side = ctl.side;
		{
			UW took = now_us() - t_wake;
			if (took > loop_max) loop_max = took;
		}
	}
}

IMPORT ER barcode_start(void);
EXPORT ER subsystem3_init(void) { return E_OK; }	/* no kernel objects */

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task((FP)line_task, PRI_LINE, CORE_LINE, "line");
	if (tid < E_OK) return tid;
	line_task_id = tid;
	return barcode_start();
}

#if APP_TEST
EXPORT void subsystem3_test(void) { }
#endif
