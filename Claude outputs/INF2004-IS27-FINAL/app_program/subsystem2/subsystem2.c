/*
 *	subsystem2.c  (Buddy 2: motion)
 *	motion task: the ONLY code that writes the motor PWM.
 *
 *	Every 20 ms (ctl_tick cyclic handler -> MF_TICK), or at once on
 *	MF_WAKE (owner change / STOP):
 *	  1. read the slot of the current owner (left/right mm/s)
 *	  2. ramp toward it (MOT_ACCEL_MMS2)
 *	  3. duty = (D0 + K * speed) * gain + kick + encoder trim
 *	  4. encoder health + trim learning (only if BOTH encoders healthy)
 */
#include "subsystem2.h"

EXPORT ID motion_flg;
LOCAL ID ctl_cyc;
LOCAL volatile UW slot[MOT_NUM_OWNERS];		/* (UH)left | (UH)right << 16 */
LOCAL volatile UINT owner = MOT_OWNER_NONE;
LOCAL MOTION_STATUS stat;
LOCAL volatile W odo_mm;			/* written by the motion task only */

EXPORT W motion_odometry_mm(void) { return odo_mm; }
LOCAL volatile W head_d10;		/* written by the motion task only */
LOCAL volatile BOOL head_ok = TRUE;
EXPORT W motion_heading_d10(void) { return head_d10; }
EXPORT BOOL motion_heading_ok(void) { return head_ok; }
LOCAL volatile UW loop_max;
EXPORT UW motion_loop_us_max(void) { UW v = loop_max; loop_max = 0; return v; }

EXPORT UW now_us(void) { return in_w(TIMER_TIMERAWL); }

/* now_us() / 1000 would jump from 4294967 back to 0, which breaks
   "t - t0" in ms.  Use all 64 bits (read high, low, high again). */
EXPORT UW now_ms(void)
{
	UW hi, lo, hi2;
	do {
		hi = in_w(TIMER_TIMERAWH);
		lo = in_w(TIMER_TIMERAWL);
		hi2 = in_w(TIMER_TIMERAWH);
	} while (hi != hi2);
	return (UW)((((UD)hi << 32) | lo) / 1000U);
}

/* ---------------- API (any task, any core) ---------------- */
#define PACK(l, r)	((UW)(UH)(H)(l) | ((UW)(UH)(H)(r) << 16))
#define UNPACK_L(w)	((INT)(H)(UH)((w) & 0xFFFFU))
#define UNPACK_R(w)	((INT)(H)(UH)((w) >> 16))

EXPORT void motion_wheels(UINT o, INT l, INT r)
{
	if (o == MOT_OWNER_NONE || o >= MOT_NUM_OWNERS) return;
	l = CLAMP(l, -MOT_MAX_MMS, MOT_MAX_MMS);
	r = CLAMP(r, -MOT_MAX_MMS, MOT_MAX_MMS);
	slot[o] = PACK(l, r);			/* one 32-bit store */
}

EXPORT void motion_drive(UINT o, INT speed, INT steer)
{
	INT s = ABS(CLAMP(steer, -1000, 1000));
	INT inner;

	if (speed < 0) speed = 0;
	/* inner wheel: 1 - 2s.  s = 500 -> stopped, s = 1000 -> -1/2 */
	inner = speed - (speed * 2 * s) / 1000;
	if (inner < -speed / 2) inner = -speed / 2;
	if (steer >= 0) motion_wheels(o, speed, inner);	/* + = turn right */
	else            motion_wheels(o, inner, speed);
}

EXPORT void motion_set_owner(UINT o)
{
	if (o >= MOT_NUM_OWNERS) return;
	owner = o;
	tk_set_flg(motion_flg, MF_WAKE);
}

EXPORT void motion_stop(void)
{
	owner = MOT_OWNER_NONE;
	drv_motor_brake(MOTOR_L);	/* brake now, even before the task runs */
	drv_motor_brake(MOTOR_R);
	tk_set_flg(motion_flg, MF_WAKE);
}

EXPORT void motion_get_status(MOTION_STATUS *st)
{
	*st = stat;			/* debug copy: may be one tick mixed */
}

/* ---------------- 20 ms tick: wakes motion + line ---------------- */
IMPORT ID line_task_id;
LOCAL void ctl_tick(void *exinf)
{
	(void)exinf;
	tk_set_flg(motion_flg, MF_TICK);
	if (line_task_id > 0) tk_wup_tsk(line_task_id);
}

/* ---------------- motion task ---------------- */
LOCAL INT ramp(INT cur, INT tgt, INT step)
{
	if (tgt > cur) return (tgt - cur > step) ? cur + step : tgt;
	return (cur - tgt > step) ? cur - step : tgt;
}

LOCAL void motion_task(INT stacd, void *exinf)
{
	static const INT gain[2] = { MOT_GAIN_L_X1000, MOT_GAIN_R_X1000 };
	INT cur[2] = {0, 0};		/* ramped wheel speeds, mm/s */
	INT trim[2] = {0, 0};		/* learned duty correction, pm */
	UW start_us[2] = {0, 0};	/* when this wheel last left 0 */
	UW hist_cnt[ENC_WINDOW_TICKS][2];
	UW hist_t[ENC_WINDOW_TICKS];
	UINT hi = 0, n_hist = 0;
	BOOL healthy[2] = {TRUE, TRUE};
	UW heal_t0[2] = {0, 0};
	INT boost[2] = {0, 0};		/* stall boost, pm (12 Nov evening) */
	UW boost_max_t[2] = {0, 0};	/* when the boost reached STALL_BOOST_MAX_PM */
	UW t_last = now_us();
	UINT last_owner = MOT_OWNER_NONE;
	ENC_SNAP es[2];
	W prev_pos[2] = {0, 0};
	W odo_um = 0;
	long long diff_um = 0;		/* right - left wheel travel, um */
	BOOL first = TRUE, enc_fault = FALSE;
	UINT ptn, m;
	ER er;

	(void)stacd; (void)exinf;
	CHECK(drv_encoder_init(), "encoder init");	/* we are on core 0 */

	for (;;) {
		INT tgt[2], v[2] = {0, 0};
		UINT duty[2] = {0, 0};
		UINT o;
		UW t, dt_us;
		INT step;
		BOOL trim_on;

		er = tk_wai_flg(motion_flg, MF_TICK | MF_WAKE, TWF_ORW | TWF_BITCLR,
				&ptn, 100);
		t = now_us();
		dt_us = t - t_last;
		t_last = t;
		if (dt_us > 100000U) dt_us = 100000U;
		(void)er;

		/* encoders + speed over the last ENC_WINDOW_TICKS samples */
		drv_encoder_snapshot(es);
		if (n_hist == ENC_WINDOW_TICKS) {
			UINT old = hi;			/* oldest entry */
			UW span = t - hist_t[old];
			for (m = 0; m < 2; m++) {
				UW steps = es[m].count - hist_cnt[old][m];
				v[m] = span ? (INT)((steps * ENC_UM_PER_STEP) / (span / 1000U + 1U)) : 0;
			}
		}
		for (m = 0; m < 2; m++) hist_cnt[hi][m] = es[m].count;
		hist_t[hi] = t;
		hi = (hi + 1U) % ENC_WINDOW_TICKS;
		if (n_hist < ENC_WINDOW_TICKS) n_hist++;

		o = owner;
		if (o != last_owner) {
			if (o == MOT_OWNER_NONE) { cur[0] = cur[1] = 0; }
			last_owner = o;
		}
		if (o == MOT_OWNER_NONE) {
			tgt[0] = tgt[1] = 0;
			cur[0] = cur[1] = 0;
		} else {
			UW w = slot[o];
			tgt[0] = UNPACK_L(w);
			tgt[1] = UNPACK_R(w);
		}

		step = (INT)(((UW)MOT_ACCEL_MMS2 * dt_us) / 1000000U);
		if (step < 1) step = 1;

		for (m = 0; m < 2; m++) {
			INT was = cur[m];
			INT sp;
			INT d;

			/* reversing: ramp through zero */
			if ((tgt[m] > 0 && cur[m] < 0) || (tgt[m] < 0 && cur[m] > 0))
				cur[m] = ramp(cur[m], 0, step * 2);
			else
				cur[m] = ramp(cur[m], tgt[m], step);
			if (was == 0 && cur[m] != 0) start_us[m] = t;

			sp = ABS(cur[m]);
			if (was != 0 && cur[m] != 0 && (was > 0) != (cur[m] > 0)) boost[m] = 0;
			if (sp < 5) { drv_motor_brake(m); duty[m] = 0; boost[m] = 0; continue; }

			/* stall boost (12 Nov evening): a wheel that is driven but
			   not turning (ramp, one-sided ramp, pivoting on one
			   wheel at ~15 % duty) gets more duty, step by step,
			   until it moves; the boost then fades out slowly */
			if (healthy[m] && sp >= STALL_MIN_MMS &&
			    (t - start_us[m]) > MOT_KICK_MS * 1000U &&
			    (t - es[m].last_us) > STALL_MS * 1000U) {
				if (boost[m] < STALL_BOOST_MAX_PM) {
					boost[m] += STALL_BOOST_STEP_PM;
					if (boost[m] >= STALL_BOOST_MAX_PM) {
						boost[m] = STALL_BOOST_MAX_PM;
						boost_max_t[m] = t;
					}
				}
			} else if (boost[m] > 0) {
				boost[m] -= STALL_BOOST_FADE_PM;
				if (boost[m] < 0) boost[m] = 0;
			}

			d = ((MOT_D0_PM + (sp * MOT_K_PM_X1000) / 1000) * gain[m]) / 1000;
			d += trim[m] + boost[m];
			if ((t - start_us[m]) < MOT_KICK_MS * 1000U) d += MOT_KICK_PM;
			/* trim learnt with the wheels in the air can be very
			   negative: never let it push the duty below the level
			   where the wheel stalls */
			d = CLAMP(d, MOT_D0_PM + 30, 1000);
			duty[m] = (UINT)d;
			drv_motor_set(m, cur[m] > 0 ? 1 : -1, duty[m]);
		}

		/* encoder health: driven but no steps.  12 Nov evening: a
		   silent wheel is first treated as STALLED (boost above); only
		   if it is still silent STALL_GIVEUP_MS after the boost hit its
		   maximum is the ENCODER declared dead.  Before, a wheel stuck
		   on a ramp was called a dead encoder after 0.2 s, its trim was
		   thrown away and the car never got going again. */
		BOOL fresh[2];
		for (m = 0; m < 2; m++) {
			BOOL driven = ABS(cur[m]) >= TRIM_MIN_MMS &&
				      (t - start_us[m]) > (MOT_KICK_MS + ENC_DEAD_MS) * 1000U;
			BOOL silent = (t - es[m].last_us) > ENC_DEAD_MS * 1000U;
			fresh[m] = driven && !silent;
			if (healthy[m]) {
				if (driven && silent && boost[m] >= STALL_BOOST_MAX_PM &&
				    (t - boost_max_t[m]) > STALL_GIVEUP_MS * 1000U) {
					healthy[m] = FALSE;
					trim[m] = 0;		/* learned from a dead wheel: forget it */
					boost[m] = 0;
				}
				heal_t0[m] = t;
			} else if (silent) {
				heal_t0[m] = t;			/* restart the heal timer */
			} else if ((t - heal_t0[m]) > ENC_HEAL_MS * 1000U) {
				healthy[m] = TRUE;
			}
		}

		/* speed control (trim), PER WHEEL.  12 Nov evening: it used
		   to need both wheels above 40 mm/s and no pivoting, so at the
		   line speed (25..30 mm/s, pivoting) it NEVER ran and the
		   motors were pure open loop at ~15 % duty: any slope stalled
		   them.  Now each wheel learns from TRIM_MIN_MMS up. */
		trim_on = FALSE;
		if (n_hist == ENC_WINDOW_TICKS) {
			for (m = 0; m < 2; m++) {
				INT err;
				if (!healthy[m] || !fresh[m] || boost[m] != 0 ||
				    ABS(cur[m]) < TRIM_MIN_MMS || ABS(cur[m] - tgt[m]) > 20) continue;
				err = ABS(cur[m]) - v[m];
				trim[m] += (err * TRIM_KI_X1000) / 1000;
				trim[m] = CLAMP(trim[m], -TRIM_MAX_PM, TRIM_MAX_PM);
				trim_on = TRUE;
			}
		}

		/* odometry: average of the healthy wheels; none healthy ->
		   estimate from the commanded speed */
		{
			W d0 = es[0].pos - prev_pos[0], d1 = es[1].pos - prev_pos[1];
			W step_um;
			if (first) { d0 = d1 = 0; first = FALSE; }
			prev_pos[0] = es[0].pos;
			prev_pos[1] = es[1].pos;
			/* 12 Nov: one encoder unhealthy (dropping out): estimate its
			   travel from the healthy wheel and the commanded speed ratio
			   (spin in place: -1, straight: +1) instead of giving up on
			   the heading.  Without this every obstacle turn fell back
			   to timing, and the car swung far too wide. */
			if (healthy[0] != healthy[1]) {
				UINT g = healthy[0] ? 0U : 1U;	/* the good wheel */
				W dg = g ? d1 : d0;
				W est = cur[g] ? (W)((dg * cur[g ^ 1U]) / cur[g]) : 0;
				if (g) d0 = est; else d1 = est;
			}
			if (healthy[0] || healthy[1]) step_um = ((d0 + d1) * ENC_UM_PER_STEP) / 2;
			else step_um = (W)(((cur[0] + cur[1]) / 2) * (INT)(dt_us / 1000U));
			odo_um += step_um;
			odo_mm = odo_um / 1000;
			/* heading: (right - left) / track radians, as 0.1 deg */
			diff_um += (long long)(d1 - d0) * ENC_UM_PER_STEP;
			head_d10 = (W)((diff_um * 18000LL) / (31416LL * MOT_TRACK_MM));
			head_ok = healthy[0] || healthy[1];	/* one good wheel is enough (above) */
		}
		if ((healthy[0] && healthy[1]) == enc_fault) {	/* changed */
			enc_fault = !enc_fault;
			if (enc_fault) fault_raise(FAULT_ENCODER); else fault_clear(FAULT_ENCODER);
		}

		stat.owner = o;
		stat.trim_on = trim_on;
		for (m = 0; m < 2; m++) {
			stat.tgt_mms[m] = cur[m];
			stat.duty_pm[m] = duty[m];
			stat.trim_pm[m] = trim[m];
			stat.v_mms[m]   = healthy[m] ? v[m] : 0;
			stat.count[m]   = es[m].count;
			stat.pos[m]     = es[m].pos;
			stat.glitch[m]  = es[m].glitch;
			stat.healthy[m] = healthy[m];
		}
		{
			UW took = now_us() - t;		/* this loop's CPU time */
			if (took > loop_max) loop_max = took;
		}
	}
}

/* ---------------- init / start ---------------- */
EXPORT ER subsystem2_init(void)
{
	T_CFLG cflg = {0};
	T_CCYC ccyc = {0};

	drv_motor_init();			/* braked from here on */

	cflg.flgatr = TA_TFIFO | TA_WSGL;	/* only the motion task waits */
	if ((motion_flg = tk_cre_flg(&cflg)) < E_OK) return motion_flg;

	ccyc.cycatr = TA_HLNG;			/* created stopped */
	ccyc.cychdr = (FP)ctl_tick;
	ccyc.cyctim = CTL_PERIOD_MS;
	ccyc.cycphs = CTL_PERIOD_MS;
	if ((ctl_cyc = tk_cre_cyc(&ccyc)) < E_OK) return ctl_cyc;
	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	ID tid = app_start_task((FP)motion_task, PRI_MOTION, CORE_MOTION, "motion");
	if (tid < E_OK) return tid;
	return tk_sta_cyc(ctl_cyc);
}

#if APP_TEST
EXPORT void subsystem2_test(void) { }
#endif
