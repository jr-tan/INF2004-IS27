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
 *	  the sweep only picks the side.  Then (see subsystem5.h):
 *	  EDGE  spin slowly that way until the ping gets past the obstacle
 *	  AIM / WIDEN  turn past that edge (+ half the car + margin), drive out
 *	  ALIGN / PASS parallel, servo watching it, until past its far end
 *	  BACK / SEEK / ADVANCE / REJOIN  back to the line
 *	  Turns MEASURED by the wheel encoders, legs by odometry.  REJOIN turns the car back to the heading it
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
enum { AV_CONFIRM, AV_BACKUP, AV_SCAN, AV_BLOCKED, AV_RECHECK, AV_EDGE, AV_AIM, AV_OUT, AV_WIDEN,
       AV_ALIGN, AV_PASS, AV_BACK, AV_SEEK, AV_ADVANCE, AV_REJOIN, AV_FIND1, AV_FIND2,
       AV_PEEK, AV_PEEK_ALIGN, AV_PEEK_FWD };

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
LOCAL const char *const av_text[] = { "AVOID confirm", "AVOID back up", "AVOID fan scan",
	"AVOID blocked", "AVOID re-check ahead", "AVOID find edge", "AVOID aim past edge", "AVOID turn out", "AVOID widen",
	"AVOID align", "AVOID pass", "AVOID turn back", "AVOID seek line",
	"AVOID onto line", "AVOID rejoin", "AVOID find line (1)", "AVOID find line (2)",
	"AVOID look toward line", "AVOID still beside it: parallel", "AVOID still beside it: on" };

LOCAL UW   t0;			/* entry time of the sub-state, ms */
LOCAL INT  dir;			/* +1 = going round on the LEFT, -1 = right */
LOCAL W    h0;			/* heading when we stopped (= along the line), 0.1 deg */
LOCAL W    h_goal;		/* heading the current spin is turning to */
LOCAL INT  spin_dir;		/* +1 left, -1 right */
LOCAL W    leg_mm;		/* length of the current straight leg */
LOCAL W    leg_odo0;		/* odometry when it started */
LOCAL W    leg_head;		/* heading held on this leg, 0.1 deg */
LOCAL INT  leg_mms;		/* speed of this leg */
LOCAL INT  blocked_n;		/* sweeps in a row that said "blocked" */
LOCAL BOOL post_avoid;		/* following again after an obstacle: watch the heading */
LOCAL W    post_odo0;		/* odometry when following restarted */
LOCAL INT  back_n;		/* times we turned the car back to h0 for this obstacle */
LOCAL INT  need_cm;		/* sideways move: where the car's centre ends up */
LOCAL INT  obs_d;		/* obstacle's front, cm from the ultrasonic */
LOCAL INT  edge_phase;		/* 1 = measuring the first side, 2 = the second */
LOCAL INT  edge_seen[2];	/* beam-clear angle per side, deg ([0] LEFT, [1] RIGHT), -1 = none */
LOCAL W    e_side_cm;		/* edge position on the side we pass, cm */
LOCAL W    far_seen;		/* farthest point of the obstacle's side seen, odometry mm */
LOCAL W    aim_d10;		/* how far out the WIDEN leg points (for timed turns) */
LOCAL W    pass_odo0, pass_end;	/* PASS: start, and odometry where it may turn back */
LOCAL W    peek_extra;		/* driven on because a look said "still there", mm */
LOCAL INT  line_side;		/* where the line was when we stopped */
LOCAL INT  n, best, hits;	/* small counters for the current sub-state */
LOCAL INT  sweep_k;		/* which point of the sweep we are on */
LOCAL INT  sweep_aim;
LOCAL UW   t_pt;		/* when we aimed at this point */
LOCAL INT  pt_n, pt_cm;		/* readings taken at this point */
LOCAL INT  pt_buf[8];		/* ... and their values (median kept) */
LOCAL INT  guard_cm;		/* what the guard saw straight ahead when we stopped */
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
	pt_n = 0;
	pt_cm = RANGE_FAR_CM;
}
/* median of n small values (sorts them in place); n = 0 -> far */
LOCAL INT median_of(INT *v, INT n)
{
	INT a, b, x;
	if (n <= 0) return RANGE_FAR_CM;
	for (a = 1; a < n; a++)
		for (b = a; b > 0 && v[b - 1] > v[b]; b--) { x = v[b]; v[b] = v[b - 1]; v[b - 1] = x; }
	return v[n / 2];
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
	leg_head = h_goal;		/* the heading the last turn aimed for */
	leg_mms = mms;
	forward(mms);
}
/* 12 Nov: straight legs steer back to leg_head with the encoder
   heading.  Open loop, a leg curved with any left/right motor
   difference and the few degrees a spin over/undershoots, so the car
   met the line far from where the box route expected (or missed it).
   Set AV_HEADING_HOLD to 0 to go back to open-loop legs. */
#ifndef AV_HEADING_HOLD
#define AV_HEADING_HOLD		1
#endif
#define AV_HOLD_DIV		300	/* correction mm/s = error (0.1 deg) * speed / this */
LOCAL void leg_hold(void)
{
	W err;
	INT corr;

	if (!AV_HEADING_HOLD || !motion_heading_ok()) return;	/* encoder fault: open loop */
	err = leg_head - motion_heading_d10();	/* + = turn left */
	corr = (INT)((err * leg_mms) / AV_HOLD_DIV);
	corr = CLAMP(corr, -leg_mms / 2, leg_mms / 2);
	motion_wheels(MOT_OWNER_MISSION, leg_mms - corr, leg_mms + corr);
}
LOCAL W leg_done_mm(void) { return motion_odometry_mm() - leg_odo0; }


/* ---------------- climb mode (12 Nov evening) ----------------
   On a slope or one-sided ramp, or with a driven wheel not turning, the
   zig-zag (pivoting on one wheel at ~15 % duty) got the car stuck.
   Climb mode: line following goes on, but faster and with both wheels
   always forward (line_climb).  Still stuck after CLIMB_STUCK_MS -> back
   off CLIMB_BACK_MM and try again with a run-up, CLIMB_RETRIES times,
   then a clean "stuck on a ramp" safe stop. */
#define CLIMB_TILT_D10		80	/* nose up or tilted sideways by 8 deg ... */
#define CLIMB_ON_MS		300	/* ... for this long -> climb mode */
#define CLIMB_OFF_MS		600	/* level and moving this long -> normal again */
#define CLIMB_STALL_WIN_MS	500	/* wheel progress checked over this window */
#define CLIMB_STALL_STEPS	36	/* < ~3 mm in the window while driven = stalled */
#define CLIMB_STUCK_MS		1500	/* stalled this long in climb mode -> back off */
#define CLIMB_RETRIES		2
#define CLIMB_BACK_MM		60
#define CLIMB_BACK_MMS		60

LOCAL UINT climb;		/* 0 off, 1 climbing, 2 backing off for a run-up */
LOCAL INT  climb_tries;
LOCAL UW   climb_t, calm_t, tilt_t, stuck_t, wpos_t;
LOCAL W    wpos[2], climb_odo;
LOCAL BOOL wstalled;

LOCAL void climb_reset(void)
{
	climb = 0;
	climb_tries = 0;
	tilt_t = calm_t = stuck_t = wpos_t = 0;
	wstalled = FALSE;
	line_climb(FALSE);
}

/* a healthy wheel driven at >= 15 mm/s that moved < ~3 mm in the window */
LOCAL BOOL wheels_stalled(UW t)
{
	MOTION_STATUS ms;
	UINT m;

	if (wpos_t != 0 && (t - wpos_t) < CLIMB_STALL_WIN_MS) return wstalled;
	motion_get_status(&ms);
	wstalled = FALSE;
	for (m = 0; m < 2; m++) {
		if (wpos_t != 0 && ms.healthy[m] && ABS(ms.tgt_mms[m]) >= 15 &&
		    ABS(ms.pos[m] - wpos[m]) < CLIMB_STALL_STEPS)
			wstalled = TRUE;
		wpos[m] = ms.pos[m];
	}
	wpos_t = t;
	return wstalled;
}

LOCAL void fault(const char *why, UINT bits);

/* called every mission tick while following; TRUE = it faulted */
LOCAL BOOL climb_step(UW t)
{
	IMU_STATUS im;
	BOOL tilt, stalled;

	imu_get(&im);
	/* nose UP or sideways: going down a slope needs no help */
	tilt = im.ok && (im.pitch_d10 >= CLIMB_TILT_D10 || ABS(im.roll_d10) >= CLIMB_TILT_D10);
	stalled = (climb == 2) ? FALSE : wheels_stalled(t);

	switch (climb) {
	case 0:
		if (tilt) { if (tilt_t == 0) tilt_t = t; } else tilt_t = 0;
		if ((tilt_t != 0 && (t - tilt_t) >= CLIMB_ON_MS) || stalled) {
			climb = 1;
			climb_t = t;
			calm_t = stuck_t = 0;
			line_climb(TRUE);
			text = "FOLLOW (climb)";
			tm_printf((UB*)"MISSION climb mode: pitch %d roll %d (0.1 deg)%s\n",
				  im.pitch_d10, im.roll_d10, stalled ? ", a wheel is stalled" : "");
		}
		break;
	case 1:
		if (!tilt && !stalled) {
			if (calm_t == 0) calm_t = t;
			if ((t - calm_t) >= CLIMB_OFF_MS) {
				tm_printf((UB*)"MISSION climb done (%d ms)\n", (INT)(t - climb_t));
				climb_reset();
				text = "FOLLOW";
				break;
			}
		} else {
			calm_t = 0;
		}
		if (stalled) { if (stuck_t == 0) stuck_t = t; } else stuck_t = 0;
		if (stuck_t != 0 && (t - stuck_t) >= CLIMB_STUCK_MS) {
			if (climb_tries >= CLIMB_RETRIES) {
				fault("stuck on a ramp", FAULT_MOTION);
				return TRUE;
			}
			climb_tries++;
			climb = 2;
			climb_t = t;
			climb_odo = motion_odometry_mm();
			motion_wheels(MOT_OWNER_MISSION, -CLIMB_BACK_MMS, -CLIMB_BACK_MMS);
			motion_set_owner(MOT_OWNER_MISSION);
			tm_printf((UB*)"MISSION still stuck: backing off %d mm for a run-up (try %d of %d)\n",
				  CLIMB_BACK_MM, climb_tries, CLIMB_RETRIES);
		}
		break;
	case 2:
		if ((climb_odo - motion_odometry_mm()) >= CLIMB_BACK_MM || (t - climb_t) > 2000U) {
			motion_set_owner(MOT_OWNER_LINE);	/* climb mode still on: go */
			climb = 1;
			climb_t = t;
			stuck_t = calm_t = wpos_t = 0;
			tm_printf((UB*)"MISSION run-up\n");
		}
		break;
	}
	return FALSE;
}

LOCAL void halt_all(void)
{
	climb_reset();
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
	climb_reset();
	spin_t0 = now_ms() - 1000U;		/* take a sample at once */
	spin_i = 0;
	if (state == MS_IDLE || state == MS_FAULT || state == MS_PAUSED || state == MS_DONE)
		fault_clear(FAULT_LINE | FAULT_AVOID | FAULT_MOTION);
	/* 12 Nov: coming back from an obstacle, the line follower can catch
	   the line facing the wrong way and follow it back to the start.
	   Watch the heading for the first AV_BACKWARD_CHECK_MM. */
	post_avoid = (state == MS_AVOID);
	post_odo0 = motion_odometry_mm();
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
	climb_reset();
	t_stop = t;
	h0 = motion_heading_d10();
	line_get(&l);
	line_side = l.side;
	line_follow(FALSE, 0, FALSE);
	ranger_guard(FALSE);
	ranger_aim(AV_SWEEP_MAX_DEG);		/* servo swings to the first angle while we brake */
	ranger_led(TRUE);
	state = MS_AVOID;
	/* the guard's last close reading: with "3 of 5" the latest ping
	   may well be a missed echo (255) */
	guard_cm = ranger_guard_cm();
	if (guard_cm > AV_CLEAR_CM) guard_cm = AV_CLEAR_CM;	/* never a far reading */
	tm_printf((UB*)"MISSION obstacle at %d cm\n", guard_cm);
	obs.bearing_deg = 0; obs.width_cm = 0; obs.need_left_cm = 0; obs.need_right_cm = 0; obs.dir = 0;
	obs_note(OBS_DETECTED, guard_cm);
	blocked_n = 0;
	back_n = 0;
	go_av(AV_CONFIRM, t);
}

/* back on the line after an obstacle: hand over to the line follower.
   We came from the 'dir' side, so the line is on the other side of the
   car: right (+1) if we went round on the left. */
LOCAL void rejoined(void)
{
	/* 12 Nov: a sensor on black says where the line really is (as at
	   the end of a barcode turn); the 'came from' guess is only used
	   when both sensors see black */
	UINT see = line_see_bits();
	INT side = (see == 1U) ? -1 : (see == 2U) ? 1 : (dir > 0 ? 1 : -1);

	obs_note(OBS_REJOINED, -1);
	tm_printf((UB*)"MISSION rejoin: sensors %d, line to the %s\n",
		  (INT)see, side > 0 ? "RIGHT" : "LEFT");
	begin_follow(side, TRUE);
}

/* start (or restart) the fan sweep */
LOCAL void start_sweep(UW t)
{
	go_av(AV_SCAN, t);
	sweep_k = 0;
	ranger_fast(TRUE);
	sweep_aim_next(t);
}

/* ---- sensor-guided way round (see subsystem5.h) ---- */
LOCAL INT icos_x1000(INT deg) { return isin_x1000(90 - deg); }
/* smallest whole degree 0..89 whose tangent is >= num / den */
LOCAL INT atan_deg(W num, W den)
{
	INT a;
	if (num <= 0) return 0;
	if (den <= 0) return 89;
	for (a = 0; a < 89; a++)
		if ((W)isin_x1000(a) * den >= (W)icos_x1000(a) * num) return a;
	return 89;
}
LOCAL void edge_spin(void)
{
	spin_dir = dir;
	motion_wheels(MOT_OWNER_MISSION, -dir * AV_EDGE_MMS, dir * AV_EDGE_MMS);
	motion_set_owner(MOT_OWNER_MISSION);
}
/* old box route with a fixed sideways move: only if the encoder
   heading is not usable (the edge spin needs it) */
LOCAL void go_round_fixed(UW t, const char *why)
{
	need_cm = AV_UNKNOWN_NEED_CM;
	aim_d10 = AV_TURN_DEG * 10;
	tm_printf((UB*)"MISSION %s: fixed route on the %s, %d cm sideways\n",
		  why, dir > 0 ? "LEFT" : "RIGHT", need_cm);
	ranger_fast(FALSE);
	turn_to(h0 + dir * AV_TURN_DEG * 10);
	go_av(AV_OUT, t);
}
/* side chosen (dir): find that edge by turning the car */
/* sideways position of an edge whose beam-clear angle was seen_deg */
LOCAL W edge_lat_cm(INT seen_deg)
{
	W dax = (W)obs_d + AV_US_FWD_CM;		/* front, from the axle */
	INT e_deg = seen_deg - AV_CONE_DEG;		/* true edge: minus the cone */
	if (e_deg < 0) e_deg = 0;
	return (dax * isin_x1000(e_deg)) / icos_x1000(e_deg);
}

/* 12 Nov evening: BOTH edges are measured (the car turns to each in
   turn), then it goes round on the side needing the smaller sideways
   move: need = edge on that side + half the car + margin.  Width =
   left edge + right edge. */
LOCAL void start_edge(UW t)
{
	obs_d = (prof.found && prof.dmin_cm <= AV_CLEAR_CM) ? (INT)prof.dmin_cm : guard_cm;
	if (obs_d < US_MIN_CM) obs_d = US_MIN_CM;
	obs.dir = dir;
	if (!motion_heading_ok()) {
		obs_note(OBS_SWEPT, -1);
		go_round_fixed(t, "encoder heading not usable");
		return;
	}
	tm_printf((UB*)"MISSION obstacle front at %d cm: measuring its edges, %s side first\n",
		  obs_d, dir > 0 ? "LEFT" : "RIGHT");
	edge_phase = 1;
	edge_seen[0] = edge_seen[1] = -1;
	ranger_aim(0);
	ranger_fast(TRUE);
	edge_spin();
	go_av(AV_EDGE, t);
}

/* both sides looked at: pick the shorter way round and aim past it */
LOCAL void edge_decide(UW t)
{
	W dax = (W)obs_d + AV_US_FWD_CM;
	W e[2] = {0, 0}, need[2] = {0, 0};
	INT k, aim, ix;

	motion_wheels(MOT_OWNER_MISSION, 0, 0);
	ranger_fast(FALSE);
	for (k = 0; k < 2; k++) {
		if (edge_seen[k] < 0) continue;
		e[k] = edge_lat_cm(edge_seen[k]);
		need[k] = e[k] + AV_HALF_CAR_CM + AV_MARGIN_CM;
	}
	if (edge_seen[0] < 0 && edge_seen[1] < 0) {
		fault("no way round: no edge found on either side", FAULT_AVOID);
		return;
	}
	if (edge_seen[0] >= 0 && edge_seen[1] >= 0) {
		dir = (need[0] < need[1]) ? 1 : (need[1] < need[0]) ? -1 : (AV_PREFER_LEFT ? 1 : -1);
		obs.width_cm = (INT)CLAMP(e[0] + e[1], 0, 254);
		obs.bearing_deg = ((edge_seen[0] - AV_CONE_DEG) - (edge_seen[1] - AV_CONE_DEG)) / 2;
		tm_printf((UB*)"OBSTACLE edges: LEFT %d cm, RIGHT %d cm -> width ~%d cm, "
			  "move L %d / R %d cm -> going %s (shorter)\n",
			  (INT)e[0], (INT)e[1], (INT)(e[0] + e[1]), (INT)need[0], (INT)need[1],
			  dir > 0 ? "LEFT" : "RIGHT");
	} else {
		dir = (edge_seen[0] >= 0) ? 1 : -1;
		obs.width_cm = 0;
		tm_printf((UB*)"OBSTACLE only the %s edge found (%d cm): going that way\n",
			  dir > 0 ? "LEFT" : "RIGHT", (INT)e[dir > 0 ? 0 : 1]);
	}
	ix = dir > 0 ? 0 : 1;
	e_side_cm = e[ix];
	need_cm = (INT)need[ix];
	if (need_cm > AV_NEED_MAX_CM) {
		tm_printf((UB*)"MISSION sideways move %d cm capped to %d cm\n", need_cm, AV_NEED_MAX_CM);
		need_cm = AV_NEED_MAX_CM;
	}
	aim = atan_deg(need_cm, dax);
	if (aim > AV_AIM_MAX_DEG) aim = AV_AIM_MAX_DEG;
	aim_d10 = (W)aim * 10;
	obs.need_left_cm  = (INT)CLAMP(need[0], 0, 254);
	obs.need_right_cm = (INT)CLAMP(need[1], 0, 254);
	obs.dir = dir;
	obs_note(OBS_SWEPT, -1);
	tm_printf((UB*)"MISSION aim %d deg %s, car centre %d cm out\n", aim, dir > 0 ? "LEFT" : "RIGHT", need_cm);
	turn_to(h0 + dir * aim_d10);
	go_av(AV_AIM, t);
}

/* this side done (found or not): the other side, or decide */
LOCAL void edge_next(UW t)
{
	if (edge_phase == 1) {
		edge_phase = 2;
		dir = -dir;
		edge_spin();
		t0 = t;
		n = 0;
		tm_printf((UB*)"MISSION now the %s edge\n", dir > 0 ? "LEFT" : "RIGHT");
	} else {
		edge_decide(t);
	}
}

/* can't measure the obstacle's edges from the sweep: pick the side whose
   end of the sweep had more room, then find the edge by turning */
LOCAL void go_round_guess(UW t, const char *why)
{
	INT l_end = sweep_cm[0], r_end = sweep_cm[AV_SWEEP_PTS - 1];

	dir = (l_end == r_end) ? (AV_PREFER_LEFT ? 1 : -1) : (l_end > r_end ? 1 : -1);
	tm_printf((UB*)"MISSION %s: going round on the %s\n", why, dir > 0 ? "LEFT" : "RIGHT");
	start_edge(t);
}

/* ---------------- avoidance step ---------------- */
LOCAL void avoid_step(UW t)
{
	INT cm, i;
	UW dt = t - t0;

	switch (av) {
	case AV_CONFIRM:			/* let the car stop, then sweep */
		if (dt < AV_CONFIRM_MS) break;
		if (guard_cm < AV_BACKUP_BELOW_CM) {
			/* too close to see it in the sweep or to turn out:
			   reverse straight back first */
			tm_printf((UB*)"MISSION stopped at %d cm: backing up to ~%d cm first\n",
				  guard_cm, AV_BACKUP_TO_CM);
			go_av(AV_BACKUP, t);
			leg_odo0 = motion_odometry_mm();
			leg_mm = (W)(AV_BACKUP_TO_CM - guard_cm) * 10;
			motion_wheels(MOT_OWNER_MISSION, -AV_BACKUP_MMS, -AV_BACKUP_MMS);
			motion_set_owner(MOT_OWNER_MISSION);
			break;
		}
		tm_printf((UB*)"MISSION sweep starts %d ms after the stop\n", (INT)(t - t_stop));
		start_sweep(t);
		break;

	case AV_BACKUP: {			/* n = 0 reversing, 1 settling */
		W back = leg_odo0 - motion_odometry_mm();	/* mm reversed so far */
		if (n == 0) {
			if (back >= leg_mm || dt >= AV_BACKUP_MAX_MS) {
				motion_wheels(MOT_OWNER_MISSION, 0, 0);
				n = 1;
				t_mark = t;
			}
			break;
		}
		if ((t - t_mark) < AV_BACKUP_SETTLE_MS) break;
		if (back < 0) back = 0;
		guard_cm += (INT)(back / 10);		/* it is that much farther now */
		if (guard_cm > AV_CLEAR_CM) guard_cm = AV_CLEAR_CM;
		tm_printf((UB*)"MISSION backed up %d mm, obstacle now ~%d cm\n", (INT)back, guard_cm);
		start_sweep(t);
		break;
	}

	case AV_SCAN: {				/* fan sweep, AV_PINGS_PER_PT readings per angle */
		/* 12 Nov: one reading per angle let a single missed echo (common
		   at 3.3 V) hide part of the obstacle, so it looked narrower and
		   off to one side.  Keep the closest of a few readings. */
		if (ping(sweep_aim, &cm) && pt_n < (INT)(sizeof pt_buf / sizeof pt_buf[0]))
			pt_buf[pt_n++] = cm;
		if (pt_n < AV_PINGS_PER_PT && (t - t_pt) < AV_PT_TIMEOUT_MS) break;
		/* 12 Nov evening: the MEDIAN, not the closest: one ghost echo
		   no longer decides the point */
		pt_cm = median_of(pt_buf, pt_n);
		sweep_deg[SLOT(sweep_aim)] = sweep_aim;
		sweep_cm[SLOT(sweep_aim)] = pt_cm;	/* RANGE_FAR_CM: nothing here */
		if (++sweep_k < AV_SWEEP_PTS) { sweep_aim_next(t); break; }

		/* sweep done: print it, then work out the obstacle */
		ranger_fast(FALSE);
		ranger_aim(0);
		tm_printf((UB*)"SCAN took %d ms (%d ms since the stop)\n", (INT)dt, (INT)(t - t_stop));
		tm_printf((UB*)"SCAN deg:cm");
		for (i = 0; i < AV_SWEEP_PTS; i++)
			tm_printf((UB*)" %d:%d", (INT)sweep_deg[i], (INT)sweep_cm[i]);
		tm_printf((UB*)"\n");
		/* sanity 1: the confirm step saw it straight ahead.  If the
		   sweep says straight ahead is clear, it has moved away. */
		if (sweep_cm[SLOT(0)] > AV_CLEAR_CM && sweep_cm[SLOT(AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM &&
		    sweep_cm[SLOT(-AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM) {
			/* 12 Nov: at 3.3 V the sensor misses echoes, so one
			   sweep saying "clear" was often wrong: the car drove
			   on, the guard stopped it again, it swept again ...
			   (the ~10 s wait in the videos).  Look straight ahead
			   with normal pings before believing it. */
			tm_printf((UB*)"MISSION sweep says ahead is clear: re-checking\n");
			go_av(AV_RECHECK, t);
			break;
		}
		/* sanity 2: every angle reads the same distance -> the servo is
		   most likely NOT turning (unplugged, wrong column, no power).
		   Only meaningful for a wide sweep: across +-25 deg a flat box
		   really does read almost the same everywhere. */
/* 12 Nov: the guard saw the obstacle straight ahead at guard_cm, so
		   it IS there: a missed echo at 0 deg must not split it or move
		   its centre off to one side */
		if (sweep_cm[SLOT(0)] > guard_cm) sweep_cm[SLOT(0)] = guard_cm;
		scan_profile(&scan_tune, sweep_deg, sweep_cm, AV_SWEEP_PTS, &prof);
		tm_printf((UB*)"SCAN used (0 deg floored to the guard's %d cm):", guard_cm);
		for (i = 0; i < AV_SWEEP_PTS; i++)
			tm_printf((UB*)" %d:%d", (INT)sweep_deg[i], (INT)sweep_cm[i]);
		tm_printf((UB*)"\n");
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
			tm_printf((UB*)"MISSION nothing within %d cm after all: re-checking\n", AV_FAR_CM);
			go_av(AV_RECHECK, t);
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
				/* 12 Nov: re-sweep a few times, then go round
				   anyway (the old code waited for the obstacle
				   to move, which a box never does) */
				if (++blocked_n < AV_BLOCKED_SWEEPS) {
					obs_note(OBS_BLOCKED, -1);
					go_av(AV_BLOCKED, t);
					break;
				}
				go_round_guess(t, "still blocked after re-sweeps");
				break;
			}
			dir = (l_end == r_end) ? (AV_PREFER_LEFT ? 1 : -1) : (l_end > r_end ? 1 : -1);
			tm_printf((UB*)"MISSION obstacle wider than the %d deg view: side %s (more room at that end)\n",
				  AV_SWEEP_MAX_DEG, dir > 0 ? "LEFT" : "RIGHT");
		} else {
			dir = prof.dir;
		}
		/* the sweep only CHOOSES the side; the edge itself is found by
		   turning the car (start_edge), so its size is not guessed */
		start_edge(t);
		break;
	}

	case AV_EDGE: {				/* slow spin toward dir, pinging ahead */
		W rel = (motion_heading_d10() - h0) * dir;	/* 0.1 deg out on this side */
		if (!motion_heading_ok()) {
			motion_wheels(MOT_OWNER_MISSION, 0, 0);
			go_round_fixed(t, "encoder heading lost while finding the edge");
			break;
		}
		if (ping(0, &cm)) {
			INT a = (INT)(rel / 10);
			INT expect;
			if (rel < 0) {
				n = 0;		/* second side: still swinging back across the front */
			} else {
				a = CLAMP(a, 0, 80);
				/* a flat front reads farther as the beam turns: d / cos */
				expect = (obs_d * 1000) / icos_x1000(a);
				if (cm >= RANGE_FAR_CM || cm > expect + AV_BAND_CM) {
					if (n == 0) best = (INT)rel;	/* first clear reading of this run */
					if (++n >= AV_EDGE_CLEAR_PINGS) {
						/* edge found: 'best' = the car's angle when the beam
						   first got past it */
						INT seen_deg = best / 10;
						edge_seen[dir > 0 ? 0 : 1] = seen_deg;
						tm_printf((UB*)"MISSION %s edge: beam clear at %d deg (%d cm) = %d cm sideways\n",
							  dir > 0 ? "LEFT" : "RIGHT", seen_deg, cm, (INT)edge_lat_cm(seen_deg));
						edge_next(t);
						break;
					}
				} else {
					n = 0;				/* still on the obstacle */
				}
			}
		}
		if (rel > AV_EDGE_MAX_DEG * 10 || dt > AV_EDGE_MAX_MS) {
			tm_printf((UB*)"MISSION no %s edge within %d deg\n", dir > 0 ? "LEFT" : "RIGHT", AV_EDGE_MAX_DEG);
			edge_next(t);
		}
		break;
	}

	case AV_AIM:				/* point past the edge, then drive out */
		if (turn_done(dt, aim_d10)) {
			/* drive until level with the obstacle's front: forward
			   distance dax along a line 'aim' out */
			INT aim = (INT)(aim_d10 / 10);
			W mm = (((W)obs_d + AV_US_FWD_CM) * 10 * 1000) / icos_x1000(aim);
			tm_printf((UB*)"MISSION aimed %d deg (heading %d), out %d mm\n",
				  aim, (INT)(motion_heading_d10() - h0), (INT)mm);
			leg(mm, AV_DRIVE_MMS);
			go_av(AV_WIDEN, t);
		}
		break;

	case AV_BLOCKED:			/* no room either side: look again */
		if (dt >= AV_BLOCKED_RETRY_MS) {
			tm_printf((UB*)"MISSION blocked (%d of %d): sweeping again\n",
				  blocked_n, AV_BLOCKED_SWEEPS);
			start_sweep(t);
		}
		break;

	case AV_RECHECK:			/* sweep said clear: make sure */
		/* AV_RECHECK_PINGS normal pings straight ahead.  Any close
		   one = it is still there: go round.  All clear = gone. */
		if (ping(0, &cm)) {
			n++;
			if (cm <= AV_CLEAR_CM) {
				tm_printf((UB*)"MISSION re-check: still there at %d cm\n", cm);
				go_round_guess(t, "sweep missed it");
				break;
			}
			if (n >= AV_RECHECK_PINGS) {
				tm_printf((UB*)"MISSION re-check: clear %d times, obstacle gone\n", n);
				obs_note(OBS_GONE, cm);
				if (line_see_bits() != 0) {
					begin_follow(line_side, FALSE);
					break;
				}
				/* 12 Nov 18:48 run: no sensor on the line here, and
				   the line follower's recovery spin has no limit: it
				   turned the car ~120 deg and picked the line up
				   facing BACKWARDS.  Instead face the old heading and
				   sweep at most AV_FIND_DEG each way (toward the side
				   the line was on first). */
				dir = (line_side > 0) ? -1 : (line_side < 0) ? 1 : (AV_PREFER_LEFT ? 1 : -1);
				tm_printf((UB*)"MISSION no sensor on the line: bounded search for it\n");
				turn_to(h0);
				go_av(AV_REJOIN, t);
				break;
			}
		}
		if (dt > AV_RECHECK_MAX_MS) go_round_guess(t, "no clear answer ahead");
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
		leg_hold();
		if (ping(0, &cm) && cm <= AV_SEEK_STOP_CM) { fault("something in the way while going round", FAULT_AVOID); break; }
		if (leg_done_mm() >= leg_mm) {
			turn_to(h0);
			go_av(AV_ALIGN, t);
		}
		break;

	case AV_ALIGN:				/* parallel to the line again */
		if (turn_done(dt, aim_d10)) {
			/* the axle is now about level with the obstacle's front.
			   Drive on until AV_PASS_CLEAR_CM past the farthest point
			   of it the side-looking sensor sees (at least
			   AV_PASS_MIN_DEPTH_CM deep), so its length is measured,
			   not assumed. */
			ranger_aim(-dir * AV_SIDE_DEG);	/* look toward it */
			leg((W)AV_PASS_MAX_CM * 10, AV_DRIVE_MMS);
			pass_odo0 = motion_odometry_mm();
			pass_end = pass_odo0 + (W)(AV_PASS_MIN_DEPTH_CM + AV_PASS_CLEAR_CM) * 10;
			peek_extra = 0;
			far_seen = pass_odo0;
			tm_printf((UB*)"MISSION parallel (heading %d), passing it\n",
				  (INT)(motion_heading_d10() - h0));
			go_av(AV_PASS, t);
		}
		break;

	case AV_PASS:				/* drive past it */
		leg_hold();
		if (ping(-dir * AV_SIDE_DEG, &cm) && cm < AV_PASS_SEE_CM) {
			/* that point is (sensor ahead of the axle) + cm x cos(side angle) ahead */
			W end = motion_odometry_mm() +
				((W)AV_US_FWD_CM * 10 + ((W)cm * 10 * icos_x1000(AV_SIDE_DEG)) / 1000) +
				(W)AV_PASS_CLEAR_CM * 10;
			if (end > pass_end) pass_end = end;
			if (end - (W)AV_PASS_CLEAR_CM * 10 > far_seen) far_seen = end - (W)AV_PASS_CLEAR_CM * 10;
			seen = TRUE;
		}
		if (motion_odometry_mm() >= pass_end || leg_done_mm() >= leg_mm) {
			if (motion_odometry_mm() < pass_end)
				tm_printf((UB*)"MISSION still seeing it after %d cm: turning back anyway\n", AV_PASS_MAX_CM);
			tm_printf((UB*)"MISSION passed (%d mm, %s)\n", (INT)leg_done_mm(),
				  seen ? "length measured" : "never saw its side: minimum length");
			obs_note(OBS_PASSED, -1);
			ranger_aim(0);
			turn_to(h0 - dir * AV_TURN_DEG * 10);
			go_av(AV_BACK, t);
		}
		break;

	case AV_BACK:				/* turn toward the line, then look */
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			motion_wheels(MOT_OWNER_MISSION, 0, 0);
			ranger_aim(0);
			pt_n = 0;
			go_av(AV_PEEK, t);
		}
		break;

	case AV_PEEK: {				/* is the way back to the line clear? */
		INT med;
		if (dt < 150U) break;			/* stop rocking first */
		if (ping(0, &cm) && pt_n < AV_PEEK_PINGS) pt_buf[pt_n++] = cm;
		if (pt_n < AV_PEEK_PINGS && dt < AV_PEEK_MAX_MS) break;
		med = median_of(pt_buf, pt_n);
		if (med > AV_PEEK_CLEAR_CM) {
			W mm = ((W)need_cm * 10 * 1000) / isin_x1000(AV_TURN_DEG) + AV_SEEK_EXTRA_CM * 10;
			/* length: from the front (PASS start, axle level with it) to
			   the farthest side point seen.  Rough: +- a few cm. */
			if (far_seen > pass_odo0)
				tm_printf((UB*)"OBSTACLE length ~%d cm (width ~%d cm)\n",
					  (INT)((far_seen - pass_odo0) / 10), obs.width_cm);
			else
				tm_printf((UB*)"OBSTACLE length: short, its side was never seen (under ~%d cm)\n",
					  AV_PASS_MIN_DEPTH_CM + AV_PASS_CLEAR_CM);
			tm_printf((UB*)"MISSION way back clear (%d cm ahead, %d readings): heading for the line\n",
				  med, pt_n);
			leg(mm, AV_SEEK_MMS);
			go_av(AV_SEEK, t);
			break;
		}
		/* still beside us: a long obstacle.  The 45 deg beam meets its
		   side about (gap to it) ahead of the sensor. */
		{
			W pt = motion_odometry_mm() + ((W)AV_US_FWD_CM + ((W)need_cm - e_side_cm)) * 10;
			if (pt > far_seen) far_seen = pt;
		}
		peek_extra += (W)AV_PEEK_STEP_CM * 10;
		if (peek_extra > (W)AV_PEEK_MAX_CM * 10) {
			fault("obstacle too long: still beside it after the extra distance", FAULT_AVOID);
			break;
		}
		tm_printf((UB*)"MISSION still beside it (%d cm ahead): %d cm further on, then look again\n",
			  med, AV_PEEK_STEP_CM);
		turn_to(h0);
		go_av(AV_PEEK_ALIGN, t);
		break;
	}

	case AV_PEEK_ALIGN:			/* parallel again */
		if (turn_done(dt, AV_TURN_DEG * 10)) {
			leg((W)AV_PEEK_STEP_CM * 10, AV_DRIVE_MMS);
			go_av(AV_PEEK_FWD, t);
		}
		break;

	case AV_PEEK_FWD:			/* a bit further along it */
		leg_hold();
		if (leg_done_mm() >= leg_mm) {
			turn_to(h0 - dir * AV_TURN_DEG * 10);
			go_av(AV_BACK, t);
		}
		break;

	case AV_SEEK:				/* drive until a sensor sees black */
		leg_hold();
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
		leg_hold();
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
#define TURN_PENDING_MS		10000	/* a turn barcode waits this long for a junction
				   (12 Nov: was 4000, ~120 mm at 30 mm/s; the old
				   slow timer made it ~170 s) */
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
	climb_reset();
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
		/* tipped over / picked up: SAFE STOP (brake, FAULT, needs START) */
		if ((ptn & (EV_TILT | EV_LIFT)) &&
		    (state == MS_FOLLOW || state == MS_AVOID || state == MS_TURN || state == MS_REVERSE)) {
			fault((ptn & EV_LIFT) ? "picked up (IMU): safe stop" : "tilted over (IMU): safe stop",
			      FAULT_MOTION);
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
			if (climb_step(t)) break;		/* faulted: stuck on a ramp */
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
			if (post_avoid) {
				W dh = motion_heading_d10() - h0;
				if (!motion_heading_ok() ||
				    (motion_odometry_mm() - post_odo0) > AV_BACKWARD_CHECK_MM) {
					post_avoid = FALSE;		/* far enough: normal following */
				} else if (ABS(dh) > AV_BACKWARD_DEG * 10) {
					if (++back_n > AV_BACKWARD_TRIES) {
						fault("keeps following the line backwards after the obstacle", FAULT_AVOID);
						break;
					}
					tm_printf((UB*)"MISSION facing %d deg off the old heading: following "
						  "the line BACKWARDS, turning back\n", (INT)(dh / 10));
					if (dir == 0) dir = AV_PREFER_LEFT ? 1 : -1;	/* FIND sweep order */
					line_follow(FALSE, 0, FALSE);
					ranger_guard(FALSE);
					state = MS_AVOID;
					turn_to(h0);
					go_av(AV_REJOIN, t);
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
