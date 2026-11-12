/*
 *	line_ctl.c  (Buddy 3)
 *	Pure line-following logic.  See line_ctl.h.  No kernel calls here.
 */
#include "line_ctl.h"

/* same numbers as LINE_M_* in car.h (checked in subsystem3.c) */
#define LC_TRACK	1
#define LC_CENTRED	2
#define LC_RECOVER	3
#define LC_JUNCTION	4
#define LC_LOST		5
#define LC_BENDSTOP	6
#define LC_PIVOT	7
#define PIVOT_STEER_PM	500	/* motion_drive(): steer 500 = inner wheel stopped */
#ifndef EDGE_KP
#define EDGE_KP		1000	/* CAL (line_sim EDGE=1) */
#endif
#ifndef EDGE_KD
#define EDGE_KD		300
#endif
#ifndef PIVOT_TIGHT_MS
#define PIVOT_TIGHT_MS	400	/* line still not found after this ... */
#endif
#ifndef PIVOT_TIGHT_PM
#define PIVOT_TIGHT_PM	900	/* ... inner wheel backs up at 40 % of the outer one */
#endif

/* CAL values: tuned in testing/line_sim.c, then on the mat */
/* Sensors STRADDLE the line (gap wider than the line) */
const LINE_TUNE line_tune_default = {
	.inside     = 0,
	.see        = 1200,
	.junction   = 3000,
	.kp_x1000   = 1000,
	.kd_x1000   = 100,
	.side_pos   = 150,
	.deep_pos   = 700,
	.fall_pos   = 100,
	.hold_small = 150,
	.wobble_ms  = 200,
	.hold_deep  = 450,
	.hold_max   = 900,
	.ramp_ms    = 600,
	.calm_ms    = 0,	/* 0: calm lasts one excursion (best in sim) */
	.lost_ms    = 2500,
	.blind_ms   = 0,	/* 0 = off.  In the simulator a well-centred car ran 3 m
				   with both sensors white, so a timeout here stops it
				   on long straights.  Try 8000 if you want it. */
	.speed      = 80,	/* 3 Nov: start slow; raise from the dashboard once bends work */
	.speed_min  = 50,
	.slow_steer = 1000,	/* straddle: slow down in proportion, as before */
	.slow_hold_ms = 0,
	.pivot_steer    = PIVOT_STEER_STRADDLE,
	.pivot_stop_ms  = 200,
	.pivot_exit_pos = 250,
	.pivot_rest_ms  = 600,
	.pivot_max_ms   = 6000,	/* a 90 deg pivot at 40 mm/s takes ~5 s */
};

/* Sensors both ON the line (gap narrower than the line), IS27 car 3 Nov.
   Simulator, 25 mm line, sensor gap 8..18 mm, tight bends R60..R120:
   the straddle tuning passed 152..202 of 216 runs (ran off the outside
   of tight bends); this one 210..216 of 216. */
const LINE_TUNE line_tune_inside = {
	.inside     = 1,
	.see        = 1200,
	.junction   = 3000,	/* unused: both black is normal here */
	.kp_x1000   = 1500,	/* stronger: only the edge of the line gives a signal */
	.kd_x1000   = 300,	/* more damping for the stronger gain */
	.side_pos   = 150,
	.deep_pos   = 700,
	.fall_pos   = 100,
	.hold_small = 150,
	.wobble_ms  = 200,
	.hold_deep  = 450,
	.hold_max   = 900,
	.ramp_ms    = 600,
	.calm_ms    = 0,
	.lost_ms    = 2500,
	.blind_ms   = 0,
	.speed      = 80,	/* 3 Nov: start slow; raise from the dashboard once bends work */
	.speed_min  = 50,
	.slow_steer = 400,	/* inside: full bend speed from 40 % steering */
	.slow_hold_ms = 400,	/* and keep it 0.4 s after the bend */
	.pivot_steer    = PIVOT_STEER_INSIDE,
	.pivot_stop_ms  = 200,
	.pivot_exit_pos = 250,
	.pivot_rest_ms  = 600,
	.pivot_max_ms   = 6000,	/* a 90 deg pivot at 40 mm/s takes ~5 s */
};

/* One sensor on the line, one on the white beside it (IS27, 3 Nov).
   Simulator, 25 mm line, sensor gap 8..18 mm, laggy car, 30 mm/s:
   see testing/line_sim (EDGE=1). */
const LINE_TUNE line_tune_edge = {
	.edge_dir   = 1,	/* set at START from which sensor is on black */
	.inside     = LINE_EDGE,
	.see        = 1200,
	.junction   = 3000,
	.kp_x1000   = EDGE_KP,
	.kd_x1000   = EDGE_KD,
	.side_pos   = 150,
	.deep_pos   = 700,
	.fall_pos   = 100,
	.hold_small = 150,
	.wobble_ms  = 200,
	.hold_deep  = 600,	/* line lost: turn back toward it, harder ... */
	.hold_max   = 900,	/* ... the longer it is missing */
	.ramp_ms    = 600,
	.calm_ms    = 0,
	.lost_ms    = 3000,
	.blind_ms   = 0,
	.speed      = 30,	/* 3 Nov: very slow while the turning is sorted out */
	.speed_min  = 25,
	.slow_steer = 400,
	.slow_hold_ms = 400,
	.pivot_steer    = PIVOT_STEER_EDGE,
	.pivot_stop_ms  = 200,
	.pivot_exit_pos = 250,
	.pivot_rest_ms  = 600,
	.pivot_max_ms   = 10000,	/* a 90 deg pivot at 25 mm/s takes ~8 s */
};

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }
static int32_t clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

void line_ctl_reset(LINE_CTL *c, const LINE_TUNE *k, int32_t seed_side,
		    int32_t seed_deep, uint32_t now_ms)
{
	c->k = k;
	c->prev_pos = 0;
	c->side = seed_side > 0 ? 1 : seed_side < 0 ? -1 : 0;
	c->deep = (seed_deep && c->side) ? 1 : 0;
	c->peak = c->deep ? k->deep_pos : 0;
	c->calm = 0;
	c->t_seen = now_ms;
	c->t_bend = now_ms - 100000U;
	c->mode = LC_CENTRED;
	c->edge = k->edge_dir >= 0 ? 1 : -1;
	c->pv = 0;
	c->pv_dir = 0;
	c->pv_seen = 0;
	c->t_pv = now_ms;
	c->t_pv_end = now_ms - 100000U;
}

void line_ctl_step(LINE_CTL *c, int32_t dl, int32_t dr, uint32_t now, LINE_OUT *o)
{
	const LINE_TUNE *k = c->k;
	int32_t see_l = dl >= k->see, see_r = dr >= k->see;
	int32_t pos = clamp(((dr - dl) * 1000) / 4095, -1000, 1000);
	int32_t steer = 0;
	int32_t slow;

	o->lost = 0;

	if (k->inside == LINE_EDGE) {
		/* edge: one sensor on black, one on white.  dl + dr = 4095 on
		   the edge; less = off the line, more = over it. */
		int32_t dir;
		/* whichever sensor is on black IS the line sensor.  If the
		   car crosses the whole line (an inside corner, or a big
		   drift), it simply follows the other edge from then on,
		   instead of steering back across the line the wrong way. */
		if (see_l && dr < k->see / 2) c->edge = 1;
		else if (see_r && dl < k->see / 2) c->edge = -1;
		dir = c->edge;
		pos = clamp((dir * (dl + dr - 4095) * 1000) / 4095, -1000, 1000);
		if (!see_l && !see_r) {
			/* both white: the line is on the black sensor's side */
			uint32_t gone = now - c->t_seen;
			int32_t g = (int32_t)(gone > (uint32_t)k->ramp_ms ? (uint32_t)k->ramp_ms : gone);
			c->side = -dir;
			steer = c->side * (k->hold_deep + ((k->hold_max - k->hold_deep) * g) / k->ramp_ms);
			c->mode = LC_RECOVER;
			if (gone > (uint32_t)k->lost_ms) {
				c->mode = LC_LOST;
				o->lost = 1;
			}
		} else {
			steer = (k->kp_x1000 * pos + k->kd_x1000 * (pos - c->prev_pos)) / 1000;
			if (iabs(pos) >= k->side_pos) c->side = pos > 0 ? 1 : -1;
			c->mode = LC_TRACK;
			c->t_seen = now;
		}
	} else if (!k->inside && see_l && see_r && dl >= k->junction && dr >= k->junction) {
		/* both black: a crossing or a wide mark.  Go straight. */
		c->mode = LC_JUNCTION;
		c->t_seen = now;
		steer = 0;
	} else if (see_l || see_r) {
		int32_t a = iabs(pos);
		int32_t side_now = a >= k->side_pos ? (pos > 0 ? 1 : -1) : 0;

		if (c->mode == LC_RECOVER) {
			/* found it again.  Same side = it really was outside
			   (a curve).  Other side = we overshot: calm down. */
			if (side_now != 0) c->calm = (side_now != c->side);
			c->peak = 0;
			c->deep = 0;
		}
		if (side_now != 0) {
			if (side_now != c->side) { c->peak = 0; c->deep = 0; }
			c->side = side_now;
			if (a > c->peak) c->peak = a;
			if (c->peak >= k->deep_pos && !c->calm) c->deep = 1;
		}
		steer = (k->kp_x1000 * pos + k->kd_x1000 * (pos - c->prev_pos)) / 1000;

		/* deep and fading on the same side: the line may be sliding
		   out past the sensor.  Don't let the steering collapse. */
		if (c->deep && side_now == c->side && side_now != 0 &&
		    a < c->peak - k->fall_pos && iabs(steer) < k->hold_deep)
			steer = c->side * k->hold_deep;

		c->mode = LC_TRACK;
		c->t_seen = now;
	} else {
		/* both white */
		uint32_t gone = now - c->t_seen;

		if (k->inside && c->side) c->deep = 1;	/* both white = off the line, for sure */
		if (c->deep && c->side) {
			int32_t g = (int32_t)(gone > (uint32_t)k->ramp_ms ? (uint32_t)k->ramp_ms : gone);
			steer = c->side * (k->hold_deep + ((k->hold_max - k->hold_deep) * g) / k->ramp_ms);
			c->mode = LC_RECOVER;
			if (gone > (uint32_t)k->lost_ms) {
				c->mode = LC_LOST;
				o->lost = 1;
			}
		} else {
			if (gone > (uint32_t)k->calm_ms) c->calm = 0;	/* settled */
			c->mode = LC_CENTRED;
			steer = (c->side && gone < (uint32_t)k->wobble_ms) ? c->side * k->hold_small : 0;
			if (k->blind_ms > 0 && gone > (uint32_t)k->blind_ms) {
				c->mode = LC_LOST;
				o->lost = 1;
			}
		}
	}

	steer = clamp(steer, -1000, 1000);
	c->prev_pos = pos;

	/* slower when steering hard, slowest while recovering.  Full bend
	   speed from |steer| = slow_steer, held slow_hold_ms after the bend
	   so the car doesn't speed up again half way round it. */
	{
		int32_t a = iabs(steer), ss = k->slow_steer > 0 ? k->slow_steer : 1000;
		if (a >= ss || c->mode == LC_RECOVER) c->t_bend = now;
		if (a > ss) a = ss;
		slow = ((k->speed - k->speed_min) * a) / ss;
		if ((now - c->t_bend) < (uint32_t)k->slow_hold_ms) slow = k->speed - k->speed_min;
	}
	o->speed = (c->mode == LC_RECOVER) ? k->speed_min : k->speed - slow;
	o->steer = steer;
	o->mode = c->mode;

	/* ---- bend: stop, then pivot on the inner wheel ---- */
	if (k->pivot_steer > 0) {
		int32_t a = iabs(pos);
		int32_t side_now = a >= k->side_pos ? (pos > 0 ? 1 : -1) : 0;

		if (c->pv == 0 && c->mode != LC_JUNCTION &&
		    (iabs(steer) >= k->pivot_steer || (c->mode == LC_RECOVER && c->side != 0))) {
			c->pv_dir = c->mode == LC_RECOVER ? c->side : (steer > 0 ? 1 : -1);
			c->pv = (now - c->t_pv_end) > (uint32_t)k->pivot_rest_ms ? 1 : 2;
			c->pv_seen = (see_l || see_r);
			c->t_pv = now;
		}
		if (c->pv) {
			int32_t done;
			if (see_l || see_r) c->pv_seen = 1;
			if (k->inside == LINE_EDGE)	/* back on the edge */
				done = (see_l || see_r) && a < k->pivot_exit_pos;
			else if (k->inside)	/* both on the line again, near the middle */
				done = see_l && see_r && a < k->pivot_exit_pos;
			else			/* line back between the sensors */
				done = c->pv_seen && !see_l && !see_r;
			/* or it swung past the middle onto the other sensor */
			if (side_now != 0 && side_now != c->pv_dir) done = 1;

			if (c->pv == 1 && (now - c->t_pv) >= (uint32_t)k->pivot_stop_ms) {
				c->pv = 2;
				c->t_pv = now;
			}
			if (done && c->pv == 2) {
				c->pv = 0;
				c->t_pv_end = now;
				c->t_seen = now;
				c->t_bend = now;		/* bend speed for slow_hold_ms */
				if (!k->inside) { c->deep = 0; c->peak = 0; }
				/* carry on with normal steering, at bend speed */
				o->speed = k->speed_min;
			} else if (c->pv == 1) {
				o->speed = 0;			/* brake first */
				o->steer = 0;
				o->mode = LC_BENDSTOP;
				c->t_bend = now;
			} else {
				o->speed = k->speed_min;	/* outer wheel at bend speed */
				o->steer = c->pv_dir * PIVOT_STEER_PM;	/* inner wheel stopped */
				/* a corner sharper than the pivot circle: the line is
				   still not under a sensor, so the inner wheel backs
				   up a little and the car turns on the spot */
				if (!see_l && !see_r && (now - c->t_pv) > (uint32_t)PIVOT_TIGHT_MS)
					o->steer = c->pv_dir * PIVOT_TIGHT_PM;
				o->mode = LC_PIVOT;
				c->t_bend = now;
			}
			/* taking too long: hand back to normal steering (which
			   still stops the car if the line is really gone) */
			if (c->pv == 2 && (now - c->t_pv) > (uint32_t)k->pivot_max_ms) {
				c->pv = 0;
				c->t_pv_end = now;
			}
		}
	}
	if (c->pv) {
		/* the pivot is looking for the line: the RECOVER timer must
		   not give up while it turns (pivot_max_ms limits it) */
		o->lost = 0;
		c->t_seen = now;
	}
	if (o->lost) o->speed = 0;
	o->pos = pos;
}
