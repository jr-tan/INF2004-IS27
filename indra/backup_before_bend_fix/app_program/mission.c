/*
 *	mission.c
 *	Top-level behaviour.  mission task (priority 11, core 1), every 20 ms
 *	or at once on an event.
 *
 *	  IDLE --START--> FOLLOW --obstacle--> AVOID ... --line found--> FOLLOW
 *	   ^                 |  line lost                |  any failure
 *	   +------STOP-------+-----------> FAULT <-------+
 *
 *	STOP (GP21) works in every state: brake at once, back to IDLE.
 *
 *	Going round an obstacle:
 *	  CONFIRM  wait AV_CONFIRM_MS for the car to stop (the servo swings
 *	           to the first angle meanwhile)
 *	  SCAN     fan sweep, one pass: the servo swings the ultrasonic from
 *	           24 deg left to 24 deg right (7 angles, ~0.5 s) with short
 *	           close-range pings (ranger_fast).  Straight ahead clear =
 *	           it has gone / false alarm -> FOLLOW.  scan_profile()
 *	           turns that into distance, bearing, WIDTH and both edges,
 *	           and picks the side that needs the smaller sideways move
 *	  then a box route round it, every turn MEASURED by the wheel
 *	  encoders (motion_heading_d10) and every leg by odometry:
 *	  OUT / WIDEN / ALIGN / PASS / BACK / SEEK / ADVANCE / REJOIN
 *	  (see subsystem5.h).  REJOIN turns the car back to the heading it
 *	  had when it stopped, so it faces along the line again.
 *	If an encoder has failed, turns fall back to timing (AV_SPIN_DPS).
 */
#include "car.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/scan_profile.h"

LOCAL ID mission_flg;
LOCAL OBSTACLE_INFO obs;		/* last sweep, for telemetry */
LOCAL volatile UINT state = MS_IDLE;
LOCAL const char *volatile text = "IDLE";

EXPORT void mission_event(UINT ev)
{
	if (mission_flg > 0) tk_set_flg(mission_flg, ev);
}
EXPORT UINT mission_state(void) { return state; }
EXPORT const char *mission_text(void) { return (const char *)text; }
EXPORT void mission_obstacle(OBSTACLE_INFO *o) { *o = obs; }

/* Every obstacle step goes into a small ring so the comms task can't miss
   one even if two happen within its 100 ms period.
   mission task writes obs_head, comms task writes obs_tail. */
#define OBS_RING	8
LOCAL OBSTACLE_INFO obs_ring[OBS_RING];
LOCAL volatile UINT obs_head, obs_tail;

EXPORT BOOL mission_obstacle_pop(OBSTACLE_INFO *o)
{
	UINT t = obs_tail;
	if (t == obs_head) return FALSE;
	*o = obs_ring[t];
	obs_tail = (t + 1U) % OBS_RING;
	return TRUE;
}

LOCAL void obs_note(UINT stage, INT dist_cm)
{
	UINT h = obs_head, nh = (h + 1U) % OBS_RING;
	obs.stage = stage;
	if (dist_cm >= 0) obs.dist_cm = dist_cm;
	obs.seq++;
	obs_ring[h] = obs;
	if (nh != obs_tail) obs_head = nh;	/* full: overwrite the newest slot */
}

LOCAL UINT av;			/* avoidance sub-state (below) */
enum { AV_CONFIRM, AV_SCAN, AV_BLOCKED, AV_OUT, AV_WIDEN,
       AV_ALIGN, AV_PASS, AV_BACK, AV_SEEK, AV_ADVANCE, AV_REJOIN, AV_FIND1, AV_FIND2 };

/* robot/status mode (team list: INIT IDLE LINE BARCODE AVOID RECOVERY
   PAUSED SAFE_STOP COMPLETE) */
EXPORT UINT mission_sys_mode(void)
{
	switch (state) {
	case MS_IDLE:   return 1;
	case MS_FOLLOW: return 2;
	case MS_TURN:   return 3;
	case MS_REVERSE: return 5;
	case MS_AVOID:  return (av >= AV_BACK) ? 5 : 4;
	case MS_PAUSED: return 6;
	case MS_DONE:   return 8;
	default:        return 7;		/* FAULT = safe stop */
	}
}

/* ---------------- avoidance sub-states ---------------- */
LOCAL const char *const av_text[] = { "AVOID confirm", "AVOID fan scan",
	"AVOID blocked", "AVOID turn out", "AVOID widen",
	"AVOID align", "AVOID pass", "AVOID turn back", "AVOID seek line",
	"AVOID onto line", "AVOID rejoin", "AVOID find line (1)", "AVOID find line (2)" };

LOCAL UW   t0;			/* entry time of the sub-state, ms */
LOCAL INT  dir;			/* +1 = going round on the LEFT, -1 = right */
LOCAL W    h0;			/* heading when we stopped (= along the line), 0.1 deg */
LOCAL W    h_goal;		/* heading the current spin is turning to */
LOCAL INT  spin_dir;		/* +1 left, -1 right */
LOCAL W    leg_mm;		/* length of the current straight leg */
LOCAL W    leg_odo0;		/* odometry when it started */
LOCAL BOOL leg_more;		/* PASS already extended once */
LOCAL INT  need_cm;		/* sideways move from the sweep */
LOCAL INT  line_side;		/* where the line was when we stopped */
LOCAL INT  n, best, hits;	/* small counters for the current sub-state */
LOCAL INT  sweep_k;		/* which point of the sweep we are on */
LOCAL INT  sweep_aim;
LOCAL UW   t_pt;		/* when we aimed at this point */
LOCAL int32_t sweep_deg[AV_SWEEP_PTS], sweep_cm[AV_SWEEP_PTS];
LOCAL SCAN_PROFILE prof;

LOCAL UW   t_stop;		/* when the car stopped for this obstacle */

LOCAL const SCAN_TUNE scan_tune = {
	AV_BAND_CM, AV_FAR_CM, AV_BEAM_DEG, AV_HALF_CAR_CM, AV_MARGIN_CM, AV_PREFER_LEFT
};

/* k-th angle of the fan: one pass, left to right */
LOCAL INT sweep_angle(INT k) { return AV_SWEEP_MAX_DEG - k * AV_SWEEP_STEP_DEG; }
/* table slot, left to right: +80 -> 0 ... -80 -> 16 */
#define SLOT(deg)	((AV_SWEEP_MAX_DEG - (deg)) / AV_SWEEP_STEP_DEG)

LOCAL void sweep_aim_next(UW t)
{
	sweep_aim = sweep_angle(sweep_k);
	ranger_aim(sweep_aim);
	t_pt = t;
}
LOCAL BOOL seen;
LOCAL UW   t_mark;
LOCAL UINT last_seq;
LOCAL BOOL marked;		/* t_mark is set (t_mark itself may be 0) */



LOCAL void go_av(UINT s, UW t)
{
	av = s; t0 = t; n = 0; best = 0; hits = 0; seen = FALSE; t_mark = 0; marked = FALSE;
	text = av_text[s];
	tm_printf((UB*)"MISSION %s\n", av_text[s]);
}

/* new valid reading taken at the angle we asked for? */
LOCAL BOOL ping(INT aim, INT *cm)
{
	UW w = ranger_word();
	if (RW_SEQ(w) == last_seq) return FALSE;
	last_seq = RW_SEQ(w);
	if (!RW_VALID(w) || RW_DEG(w) != aim) return FALSE;
	*cm = RW_CM(w);
	return TRUE;
}

/* spin in place: + = left (counter-clockwise) */
LOCAL void spin(INT d)
{
	motion_wheels(MOT_OWNER_MISSION, -d * AV_SPIN_MMS, d * AV_SPIN_MMS);
	motion_set_owner(MOT_OWNER_MISSION);
}
LOCAL void forward(INT mms)
{
	motion_wheels(MOT_OWNER_MISSION, mms, mms);
	motion_set_owner(MOT_OWNER_MISSION);
}

/* turn on the spot to an absolute heading (0.1 deg, + = left) */
LOCAL void turn_to(W goal)
{
	h_goal = goal;
	spin_dir = (goal - motion_heading_d10()) >= 0 ? 1 : -1;
	spin(spin_dir);
}
/* TRUE when the turn has arrived (by encoders, or by time if one failed) */
LOCAL BOOL turn_done(UW dt, W deg_d10_total)
{
	W left;
	if (!motion_heading_ok())		/* timed fallback */
		return dt >= (UW)((ABS(deg_d10_total) * 100) / AV_SPIN_DPS);
	left = (h_goal - motion_heading_d10()) * spin_dir;
	return left <= AV_SPIN_LEAD_D10;
}
LOCAL void turn_to_slow(W goal)
{
	h_goal = goal;
	spin_dir = (goal - motion_heading_d10()) >= 0 ? 1 : -1;
	motion_wheels(MOT_OWNER_MISSION, -spin_dir * AV_FIND_MMS, spin_dir * AV_FIND_MMS);
	motion_set_owner(MOT_OWNER_MISSION);
}
LOCAL void leg(W mm, INT mms)
{
	leg_mm = mm;
	leg_odo0 = motion_odometry_mm();
	forward(mms);
}
LOCAL W leg_done_mm(void) { return motion_odometry_mm() - leg_odo0; }

LOCAL void halt_all(void)
{
	ranger_fast(FALSE);
	motion_stop();
	line_follow(FALSE, 0, FALSE);
	ranger_guard(FALSE);
	ranger_aim(0);
}

#define SPIN_HIST		31	/* 30 s of one-second heading samples */
LOCAL UW   spin_t0;
LOCAL W    spin_hist[SPIN_HIST];
LOCAL UINT spin_i;

LOCAL void begin_follow(INT side, BOOL deep)
{
	spin_t0 = now_ms() - 1000U;		/* take a sample at once */
	spin_i = 0;
	if (state == MS_IDLE || state == MS_FAULT || state == MS_PAUSED || state == MS_DONE)
		fault_clear(FAULT_LINE | FAULT_AVOID | FAULT_MOTION);
	ranger_fast(FALSE);
	ranger_aim(0);
	ranger_led(FALSE);
	line_follow(TRUE, side, deep);
	motion_set_owner(MOT_OWNER_LINE);
	ranger_guard(TRUE);
	state = MS_FOLLOW;
	text = "FOLLOW";
	tm_printf((UB*)"MISSION FOLLOW (seed side %d%s)\n", side, deep ? ", turn to it" : "");
}

LOCAL void obs_note(UINT stage, INT dist_cm);
LOCAL void fault(const char *why, UINT bits)
{
	if (state == MS_AVOID) obs_note(OBS_FAILED, -1);
	fault_raise(bits);
	halt_all();
	ranger_led(TRUE);
	state = MS_FAULT;
	text = why;
	tm_printf((UB*)"MISSION FAULT: %s.  Press START to try again.\n", why);
}

LOCAL void begin_avoid(UW t)
{
	LINE_STATUS l;

	motion_stop();				/* brake first, think later */
	t_stop = t;
	h0 = motion_heading_d10();
	line_get(&l);
	line_side = l.side;
	line_follow(FALSE, 0, FALSE);
	ranger_guard(FALSE);
	ranger_aim(AV_SWEEP_MAX_DEG);		/* servo swings to the first angle while we brake */
	ranger_led(TRUE);
	state = MS_AVOID;
	tm_printf((UB*)"MISSION obstacle at %d cm\n", RW_CM(ranger_word()));
	obs.bearing_deg = 0; obs.width_cm = 0; obs.need_left_cm = 0; obs.need_right_cm = 0; obs.dir = 0;
	obs_note(OBS_DETECTED, RW_CM(ranger_word()));
	go_av(AV_CONFIRM, t);
}

/* back on the line after an obstacle: hand over to the line follower.
   We came from the 'dir' side, so the line is on the other side of the
   car: right (+1) if we went round on the left. */
LOCAL void rejoined(void)
{
	obs_note(OBS_REJOINED, -1);
	begin_follow(dir > 0 ? 1 : -1, TRUE);
}

/* ---------------- avoidance step ---------------- */
LOCAL void avoid_step(UW t)
{
	INT cm, i;
	UW dt = t - t0;

	switch (av) {
	case AV_CONFIRM:			/* let the car stop, then sweep */
		if (dt < AV_CONFIRM_MS) break;
		tm_printf((UB*)"MISSION sweep starts %d ms after the stop\n", (INT)(t - t_stop));
		go_av(AV_SCAN, t);
		sweep_k = 0;
		ranger_fast(TRUE);
		sweep_aim_next(t);
		break;

	case AV_SCAN: {				/* fan sweep, one reading per angle */
		BOOL got = ping(sweep_aim, &cm);

		if (!got && (t - t_pt) < AV_PT_TIMEOUT_MS) break;
		if (!got) cm = RANGE_FAR_CM;		/* no echo here: free */
		sweep_deg[SLOT(sweep_aim)] = sweep_aim;
		sweep_cm[SLOT(sweep_aim)] = cm;
		if (++sweep_k < AV_SWEEP_PTS) { sweep_aim_next(t); break; }

		/* sweep done: print it, then work out the obstacle */
		ranger_fast(FALSE);
		ranger_aim(0);
		tm_printf((UB*)"SCAN took %d ms (%d ms since the stop)\n", (INT)dt, (INT)(t - t_stop));
		tm_printf((UB*)"SCAN deg:cm");
		for (i = 0; i < AV_SWEEP_PTS; i++)
			tm_printf((UB*)" %d:%d", (INT)sweep_deg[i], (INT)sweep_cm[i]);
		tm_printf((UB*)"\n");
		scan_profile(&scan_tune, sweep_deg, sweep_cm, AV_SWEEP_PTS, &prof);
		/* sanity 1: the confirm step saw it straight ahead.  If the
		   sweep says straight ahead is clear, it has moved away. */
		if (sweep_cm[SLOT(0)] > AV_CLEAR_CM && sweep_cm[SLOT(AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM &&
		    sweep_cm[SLOT(-AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM) {
			tm_printf((UB*)"MISSION straight ahead is clear now: obstacle gone\n");
			obs_note(OBS_GONE, sweep_cm[SLOT(0)]);
			begin_follow(line_side, FALSE);
			break;
		}
		/* sanity 2: every angle reads the same distance -> the servo is
		   most likely NOT turning (unplugged, wrong column, no power).
		   Only meaningful for a wide sweep: across +-25 deg a flat box
		   really does read almost the same everywhere. */
#if AV_SWEEP_MAX_DEG >= 45
		{
			INT lo = 255, hi = 0;
			for (i = 0; i < AV_SWEEP_PTS; i++) {
				if (sweep_cm[i] < lo) lo = sweep_cm[i];
				if (sweep_cm[i] > hi) hi = sweep_cm[i];
			}
			if (hi - lo <= 4) {
				tm_printf((UB*)"MISSION all %d angles read %d..%d cm: is the SERVO turning? Check its plug (GP13 column, brown wire to -)\n",
					  AV_SWEEP_PTS, lo, hi);
				fault("servo not turning (sweep is flat)", FAULT_AVOID);
				break;
			}
		}
#endif
		if (!prof.found) {
			tm_printf((UB*)"MISSION nothing within %d cm after all\n", AV_FAR_CM);
			obs_note(OBS_GONE, RANGE_FAR_CM);
			begin_follow(line_side, FALSE);
			break;
		}
		tm_printf((UB*)"OBSTACLE %d cm at %d deg, width %d cm, edges L %d / R %d cm (%s/%s)"
			  " -> move L %d / R %d cm\n",
			  (INT)prof.dmin_cm, (INT)prof.bearing_deg, (INT)prof.width_cm,
			  (INT)prof.left_cm, (INT)prof.right_cm,
			  prof.open_left ? "open" : "closed", prof.open_right ? "open" : "closed",
			  (INT)prof.need_left_cm, (INT)prof.need_right_cm);
		obs.dist_cm = prof.dmin_cm;
		obs.bearing_deg = prof.bearing_deg;
		obs.width_cm = prof.width_cm;
		obs.need_left_cm = prof.need_left_cm;
		obs.need_right_cm = prof.need_right_cm;
		obs.dir = prof.dir;
		if (prof.dir == 0) {
			/* both edges are outside the narrow sweep: we can't see past
			   it either side.  Truly blocked only if both ends of the
			   sweep are close; otherwise go round on the roomier side
			   with a fixed sideways move. */
			INT l_end = sweep_cm[0], r_end = sweep_cm[AV_SWEEP_PTS - 1];
			if (l_end <= AV_CLEAR_CM && r_end <= AV_CLEAR_CM && l_end < 15 && r_end < 15) {
				obs_note(OBS_BLOCKED, -1);
				go_av(AV_BLOCKED, t);
				break;
			}
			dir = (l_end == r_end) ? (AV_PREFER_LEFT ? 1 : -1) : (l_end > r_end ? 1 : -1);
			prof.need_cm = AV_UNKNOWN_NEED_CM;
			obs.dir = dir;
			tm_printf((UB*)"MISSION obstacle wider than the %d deg view: guessing %s, %d cm sideways\n",
				  AV_SWEEP_MAX_DEG, dir > 0 ? "LEFT" : "RIGHT", AV_UNKNOWN_NEED_CM);
		} else {
			dir = prof.dir;
		}
		obs_note(OBS_SWEPT, -1);
		need_cm = (INT)prof.need_cm;
		tm_printf((UB*)"MISSION going round on the %s (%d cm sideways), heading %d%s\n",
			  dir > 0 ? "LEFT" : "RIGHT", need_cm, (INT)(motion_heading_d10() - h0),
			  motion_heading_ok() ? "" : " [encoder fault: timed turns]");
		turn_to(h0 + dir * AV_TURN_DEG * 10);
		go_av(AV_OUT, t);
		break;
	}

	case AV_BLOCKED:			/* no room either side: wait */
		/* wait (STOP to give up) until 3 pings in a row say clear */
		if (ping(0, &cm)) {
			hits = (cm > AV_CLEAR_CM) ? hits + 1 : 0;
			if (hits >= 3) begin_follow(line_side, FALSE);
		}
		break;

	case AV_OUT:				/* turn AV_TURN_DEG away */
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			/* sideways move need_cm at AV_TURN_DEG: leg = need / sin */
			W mm = ((W)need_cm * 10 * 1000) / isin_x1000(AV_TURN_DEG);
			tm_printf((UB*)"MISSION out %d deg (heading %d), widen %d mm\n",
				  AV_TURN_DEG, (INT)(motion_heading_d10() - h0), (INT)mm);
			leg(mm, AV_DRIVE_MMS);
			go_av(AV_WIDEN, t);
		}
		break;

	case AV_WIDEN:				/* move out sideways */
		if (ping(0, &cm) && cm <= AV_SEEK_STOP_CM) { fault("something in the way while going round", FAULT_AVOID); break; }
		if (leg_done_mm() >= leg_mm) {
			turn_to(h0);
			go_av(AV_ALIGN, t);
		}
		break;

	case AV_ALIGN:				/* parallel to the line again */
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			/* forward needed from the stop: its distance + depth +
			   car length; WIDEN already went need_cm forward */
			W mm = ((W)prof.dmin_cm + AV_OBS_DEPTH_CM + AV_CAR_LEN_CM - need_cm) * 10;
			if (mm < 100) mm = 100;
			ranger_aim(-dir * AV_SIDE_DEG);	/* keep an eye on it */
			leg_more = FALSE;
			seen = FALSE;
			tm_printf((UB*)"MISSION parallel (heading %d), pass %d mm\n",
				  (INT)(motion_heading_d10() - h0), (INT)mm);
			leg(mm, AV_DRIVE_MMS);
			go_av(AV_PASS, t);
		}
		break;

	case AV_PASS:				/* drive past it */
		if (ping(-dir * AV_SIDE_DEG, &cm)) seen = cm < AV_SIDE_SEE_CM;
		if (leg_done_mm() >= leg_mm) {
			if (seen && !leg_more) {	/* still beside us: a bit more */
				leg_more = TRUE;
				leg_mm += AV_PASS_MORE_CM * 10;
				tm_printf((UB*)"MISSION still beside it: %d mm more\n", AV_PASS_MORE_CM * 10);
				break;
			}
			tm_printf((UB*)"MISSION passed (%d mm)\n", (INT)leg_done_mm());
			obs_note(OBS_PASSED, -1);
			ranger_aim(0);
			turn_to(h0 - dir * AV_TURN_DEG * 10);
			go_av(AV_BACK, t);
		}
		break;

	case AV_BACK:				/* turn toward the line */
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			W mm = ((W)need_cm * 10 * 1000) / isin_x1000(AV_TURN_DEG) + AV_SEEK_EXTRA_CM * 10;
			leg(mm, AV_SEEK_MMS);
			go_av(AV_SEEK, t);
		}
		break;

	case AV_SEEK:				/* drive until a sensor sees black */
		if (line_see_bits() != 0) {
			tm_printf((UB*)"MISSION line found after %d mm\n", (INT)leg_done_mm());
			leg(AV_SENSOR_FWD_CM * 10, AV_SEEK_MMS);
			go_av(AV_ADVANCE, t);
			break;
		}
		if (ping(0, &cm) && cm <= AV_SEEK_STOP_CM) fault("blocked on the way back to the line", FAULT_AVOID);
		else if (leg_done_mm() >= leg_mm) fault("line not found after the obstacle", FAULT_AVOID);
		break;

	case AV_ADVANCE:			/* wheels onto the line */
		if (leg_done_mm() >= leg_mm) {
			turn_to(h0);			/* face along the line again */
			go_av(AV_REJOIN, t);
		}
		break;

	case AV_REJOIN:
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			tm_printf((UB*)"MISSION back at the line, heading %d (0 = as before the obstacle), sensors %d\n",
				  (INT)(motion_heading_d10() - h0), (INT)line_see_bits());
			if (line_see_bits() != 0) { rejoined(); break; }
			/* facing the right way but no sensor on the line: sweep
			   slowly back toward the side we came from, then the
			   other way, and take the line as soon as a sensor sees it
			   (instead of letting the line follower spin round) */
			turn_to_slow(h0 + dir * AV_FIND_DEG * 10);
			go_av(AV_FIND1, t);
		}
		break;

	case AV_FIND1:
	case AV_FIND2:
		if (line_see_bits() != 0) {
			motion_wheels(MOT_OWNER_MISSION, 0, 0);
			tm_printf((UB*)"MISSION line found while sweeping (heading %d)\n",
				  (INT)(motion_heading_d10() - h0));
			rejoined();
			break;
		}
		if (turn_done(dt, (av == AV_FIND1 ? 1 : 2) * AV_FIND_DEG * 10 * AV_SPIN_MMS / AV_FIND_MMS)) {
			if (av == AV_FIND1) {
				turn_to_slow(h0 - dir * AV_FIND_DEG * 10);
				go_av(AV_FIND2, t);
			} else {
				fault("line not found after the obstacle (swept both ways)", FAULT_AVOID);
			}
		}
		break;
	}
}

/* ---------------- barcode turns ---------------- */
#define TURN_SPEED_MMS		80
#define TURN_ADVANCE_MS		600	/* after the crossing: axle over the junction */
#define TURN_MIN_LR_MS		300	/* spin at least this long before looking */
#define TURN_MIN_U_MS		1500	/* ~100 deg at the slower turn speed */
#define TURN_MAX_MS		7000	/* a U-turn at 70 mm/s takes ~3 s */
#define TURN_PENDING_MS		4000	/* a turn barcode waits this long for a junction */
/* INSIDE layout: both sensors are always black on the line, so a junction
   can't be seen.  The turn starts this far after the barcode instead (CAL:
   measure barcode -> junction on your course). */
#define TURN_AFTER_MM		150

/* remote REVERSE: back up this far (odometry), slowly, then stop */
#define REVERSE_MM		200
#define REVERSE_MMS		90
#define REVERSE_MAX_MS		4000

enum { TN_CROSS, TN_ADVANCE, TN_SPIN, TN_SETTLE };
#define TURN_SPIN_MMS		70	/* slower than the avoidance spin: easier to stop on the line */
#define TURN_SETTLE_MS		250	/* brake on the line, then look where it is */

LOCAL void turn_spin(INT d)
{
	motion_wheels(MOT_OWNER_MISSION, -d * TURN_SPIN_MMS, d * TURN_SPIN_MMS);
	motion_set_owner(MOT_OWNER_MISSION);
}
LOCAL UINT pend_act;		/* BC_ACT_LEFT/RIGHT waiting for a junction */
LOCAL UW   pend_t;
LOCAL W    pend_odo;
LOCAL BOOL spin_off;		/* sensors left the old line during the spin */
LOCAL W    rev_odo0;
LOCAL UW   rev_t0;
LOCAL UINT tn, tn_act;
LOCAL UW   tn_t0;

LOCAL void begin_turn(UINT act, UW t, BOOL at_junction)
{
	line_follow(FALSE, 0, FALSE);
	ranger_guard(FALSE);
	tn_act = act;
	tn_t0 = t;
	state = MS_TURN;
	spin_off = FALSE;
	if (at_junction && !line_inside()) { tn = TN_CROSS; forward(TURN_SPEED_MMS); text = "TURN crossing"; }
	else if (at_junction)              { tn = TN_ADVANCE; forward(TURN_SPEED_MMS); text = "TURN advance"; }
	else                               { tn = TN_SPIN; turn_spin(1); text = "TURN spinning"; }	/* U-turn: left */
	tm_printf((UB*)"MISSION barcode turn %s\n",
		  act == BC_ACT_LEFT ? "LEFT" : act == BC_ACT_RIGHT ? "RIGHT" : "U-TURN");
}

LOCAL void turn_step(UW t)
{
	UINT see = line_see_bits();
	UW dt = t - tn_t0;
	INT d = (tn_act == BC_ACT_RIGHT) ? -1 : 1;	/* spin direction, + = left */

	if (dt > TURN_MAX_MS) { fault("barcode turn: line not found", FAULT_LINE); return; }
	switch (tn) {
	case TN_CROSS:					/* drive off the black crossing */
		if (see != 3) { tn = TN_ADVANCE; tn_t0 = t; text = "TURN advance"; }
		break;
	case TN_ADVANCE:
		if (dt >= TURN_ADVANCE_MS) { tn = TN_SPIN; tn_t0 = t; turn_spin(d); text = "TURN spinning"; }
		break;
	case TN_SPIN:
		/* the sensors must leave the old line first (INSIDE layout starts
		   the spin sitting on it), then the first black is the new line */
		if (see == 0) spin_off = TRUE;
		if (dt >= (tn_act == BC_ACT_UTURN ? TURN_MIN_U_MS : TURN_MIN_LR_MS) && see != 0 && spin_off) {
			/* found it: STOP turning first.  Handing over while still
			   spinning made the car slide past the line before the line
			   task had a single sample, then 'recover' the wrong way
			   round and spin a full circle, again and again. */
			motion_wheels(MOT_OWNER_MISSION, 0, 0);
			tn = TN_SETTLE;
			tn_t0 = t;
			text = "TURN settle";
		}
		break;
	case TN_SETTLE:
		if (dt >= TURN_SETTLE_MS) {
			/* now look where the line really is.  If the car slid past
			   it, the line is on the side we were turning AWAY from:
			   spinning left (d > 0) leaves it to the right (+1). */
			INT side = (see == 1) ? -1 : (see == 2) ? 1 : (see == 3) ? 0 : (d > 0 ? 1 : -1);
			tm_printf((UB*)"MISSION turn done, line %s\n",
				  see == 3 ? "under both sensors" : see == 0 ? "just missed: turning back to it" :
				  see == 1 ? "under the left sensor" : "under the right sensor");
			/* STRADDLE layout: both white can simply mean centred */
			begin_follow(side, side != 0 && (see != 0 || line_inside()));
		}
		break;
	}
}

LOCAL void on_barcode(UW t)
{
	BARCODE_INFO b;

	barcode_get(&b);
	switch (b.action) {
	case BC_ACT_END:
		halt_all();
		state = MS_DONE;
		text = "COMPLETE";
		tm_printf((UB*)"MISSION end-of-course barcode \"%s\": stopped\n", b.text);
		break;
	case BC_ACT_UTURN:
		begin_turn(BC_ACT_UTURN, t, FALSE);
		break;
	case BC_ACT_LEFT:
	case BC_ACT_RIGHT:
		pend_act = b.action;
		pend_t = t;
		pend_odo = motion_odometry_mm() + TURN_AFTER_MM;
		if (line_inside())
			tm_printf((UB*)"MISSION turn %s in %d mm\n", b.action == BC_ACT_LEFT ? "LEFT" : "RIGHT", TURN_AFTER_MM);
		else
			tm_printf((UB*)"MISSION turn %s at the next junction\n",
				  b.action == BC_ACT_LEFT ? "LEFT" : "RIGHT");
		break;
	default:
		break;				/* reported by the comms task */
	}
}

/* ---------------- buttons ---------------- */
LOCAL void button_pin(UINT pin)
{
	gpio_set_pin(pin, GPIO_MODE_IN);
	set_w(GPIO(pin), GPIO_IE | GPIO_PUE);
	clr_w(GPIO(pin), GPIO_PDE);
}

/* ---------------- mission task ---------------- */
LOCAL void mission_task(INT stacd, void *exinf)
{
	UW start_t = 0, stop_t = 0;		/* when the button went down */
	BOOL start_dn = FALSE, stop_dn = FALSE, start_armed = TRUE;
	UINT ptn;
	ER er;

	(void)stacd; (void)exinf;
	for (;;) {
		UW t;
		BOOL start_now = FALSE;

		er = tk_wai_flg(mission_flg, EV_ALL, TWF_ORW | TWF_BITCLR, &ptn, MISSION_PERIOD_MS);
		if (er != E_OK) ptn = 0;
		t = now_ms();

		/* buttons (active low): pressed = held down for 40 ms */
		if (gpio_get_val(PIN_BTN_STOP))  stop_dn = FALSE;
		else if (!stop_dn)               { stop_dn = TRUE; stop_t = t; }
		if (gpio_get_val(PIN_BTN_START)) { start_dn = FALSE; start_armed = TRUE; }
		else if (!start_dn)              { start_dn = TRUE; start_t = t; }
		if (start_dn && start_armed && (t - start_t) >= 40U) { start_now = TRUE; start_armed = FALSE; }

		if (ptn & EV_CMD_STOP) { stop_dn = TRUE; stop_t = t - 40U; }	/* remote STOP */
		if ((ptn & EV_CMD_START) && state != MS_FOLLOW && state != MS_AVOID && state != MS_TURN &&
		    state != MS_REVERSE)
			start_now = TRUE;
		if (stop_dn && (t - stop_t) >= 40U) {
			if (ptn & EV_CMD_STOP) stop_dn = FALSE;	/* one-shot */
			if (state != MS_IDLE) {
				halt_all();
				ranger_led(FALSE);
				state = MS_IDLE;
				text = "IDLE";
				pend_act = BC_ACT_NONE;
				tm_printf((UB*)"MISSION STOP\n");
			}
			continue;
		}

		/* tipped over / picked up: stop whatever we were doing */
		if ((ptn & EV_TILT) && (state == MS_FOLLOW || state == MS_AVOID || state == MS_TURN || state == MS_REVERSE)) {
			fault("tilted or picked up (IMU)", FAULT_MOTION);
			continue;
		}
		/* remote PAUSE / RESUME */
		if ((ptn & EV_CMD_PAUSE) && (state == MS_FOLLOW || state == MS_AVOID || state == MS_TURN)) {
			halt_all();
			ranger_led(FALSE);
			state = MS_PAUSED;
			text = "PAUSED";
			tm_printf((UB*)"MISSION PAUSE\n");
			continue;
		}
		if ((ptn & EV_CMD_RESUME) && state == MS_PAUSED) { begin_follow(0, FALSE); continue; }
		/* remote U-TURN: turn round on the line and follow it back */
		if ((ptn & EV_CMD_UTURN) && state != MS_AVOID && state != MS_TURN && state != MS_REVERSE) {
			pend_act = BC_ACT_NONE;
			begin_turn(BC_ACT_UTURN, t, FALSE);
			continue;
		}
		/* remote REVERSE: back up REVERSE_MM and stop (no rear sensor: short and slow) */
		if ((ptn & EV_CMD_REVERSE) && state != MS_REVERSE) {
			line_follow(FALSE, 0, FALSE);
			ranger_guard(FALSE);
			motion_wheels(MOT_OWNER_MISSION, -REVERSE_MMS, -REVERSE_MMS);
			motion_set_owner(MOT_OWNER_MISSION);
			rev_odo0 = motion_odometry_mm();
			rev_t0 = t;
			state = MS_REVERSE;
			text = "REVERSE";
			tm_printf((UB*)"MISSION reversing %d mm\n", REVERSE_MM);
			continue;
		}
		if (pend_act != BC_ACT_NONE && (t - pend_t) > TURN_PENDING_MS && !line_inside()) {
			tm_printf((UB*)"MISSION no junction after the barcode: turn dropped\n");
			pend_act = BC_ACT_NONE;
		}

		switch (state) {
		case MS_IDLE:
		case MS_FAULT:
		case MS_PAUSED:
		case MS_DONE:
			if (start_now) { pend_act = BC_ACT_NONE; begin_follow(0, FALSE); }
			break;
		case MS_FOLLOW:
			/* going round in circles = the line is lost but a sensor
			   keeps catching something now and then, so the line
			   follower never gives up.  Heading sampled once a
			   second: more than 270 deg in 10 s, or 400 deg in 30 s
			   (a real track turns far less) -> stop */
			if (motion_heading_ok() && (t - spin_t0) >= 1000U) {
				W h = motion_heading_d10();
				spin_t0 = t;
				spin_hist[spin_i % SPIN_HIST] = h;
				spin_i++;
				if ((spin_i > 10 && ABS(h - spin_hist[(spin_i - 11) % SPIN_HIST]) > 2700) ||
				    (spin_i > SPIN_HIST && ABS(h - spin_hist[spin_i % SPIN_HIST]) > 4000)) {
					fault("going round in circles: line lost", FAULT_LINE);
					break;
				}
			}
			if (ptn & EV_OBSTACLE)       begin_avoid(t);
			else if (ptn & EV_LINE_LOST) fault("line lost", FAULT_LINE);
			else if (ptn & EV_BARCODE)   on_barcode(t);
			else if (pend_act != BC_ACT_NONE && line_inside() &&
				 motion_odometry_mm() >= pend_odo) {
				UINT a = pend_act;
				pend_act = BC_ACT_NONE;
				begin_turn(a, t, TRUE);
			}
			else if ((ptn & EV_JUNCTION) && pend_act != BC_ACT_NONE) {
				UINT a = pend_act;
				pend_act = BC_ACT_NONE;
				begin_turn(a, t, TRUE);
			}
			break;
		case MS_TURN:
			turn_step(t);
			break;
		case MS_REVERSE:
			if ((rev_odo0 - motion_odometry_mm()) >= REVERSE_MM || (t - rev_t0) > REVERSE_MAX_MS) {
				halt_all();
				state = MS_IDLE;
				text = "IDLE";
				tm_printf((UB*)"MISSION reversed %d mm, stopped\n", (INT)(rev_odo0 - motion_odometry_mm()));
			}
			break;
		case MS_AVOID:
			avoid_step(t);
			break;
		}
	}
}

EXPORT ER mission_init(void)
{
	T_CFLG cflg = {0};

	button_pin(PIN_BTN_START);
	button_pin(PIN_BTN_STOP);
	cflg.flgatr = TA_TFIFO | TA_WSGL;	/* only the mission task waits */
	if ((mission_flg = tk_cre_flg(&cflg)) < E_OK) return mission_flg;
	return E_OK;
}

EXPORT ER mission_start(void)
{
	ID tid = app_start_task((FP)mission_task, PRI_MISSION, CORE_MISSION, "mission");
	return tid < E_OK ? tid : E_OK;
}
