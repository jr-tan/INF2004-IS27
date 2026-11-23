/*
 *	line_ctl.h  (Buddy 3)
 *	Two-sensor line controller as PURE logic: no kernel calls, no
 *	hardware.  The same file is compiled into the firmware and into the
 *	PC simulator (testing/line_sim.c), so what is tested is what runs.
 *
 *	Three layouts (LINE_TUNE.inside):
 *	  0 straddle  sensors either side of the line: centred = BOTH white
 *	  1 inside    both sensors over the line: centred = BOTH black
 *	  2 edge      ONE sensor on the line, the other on the white beside
 *	              it: the car follows the line's EDGE.  Steering comes
 *	              from how the light changes on each sensor:
 *	                pos = edge_dir * (dark_l + dark_r - 4095)
 *	              0 = exactly on the edge (one black + one white).
 *	              The black sensor turning grey = drifting off the line
 *	              -> steer toward it; the white sensor turning grey =
 *	              drifting over the line -> steer away.  Both white =
 *	              line lost on the black sensor's side (RECOVER).
 *	              Whichever sensor is on black is the line sensor, so
 *	              after crossing the whole line (inside corner) the car
 *	              just follows the other edge.
 *	The steering formula is the same for both; what "both black" and
 *	"both white" mean is not (see 'inside' below).
 *	  pos = right darkness - left darkness  (-1000 .. +1000, + = line
 *	        under the RIGHT sensor = car has drifted left = steer right)
 *
 *	Modes
 *	  TRACK     a sensor sees the line: steer = KP * pos + KD * dpos.
 *	            Remember which side and how DEEP the excursion went.
 *	            Deep (line got right under a sensor) and now fading:
 *	            keep at least HOLD_DEEP, because on a curve the line is
 *	            sliding OUTSIDE the sensor exactly when plain P would
 *	            straighten the car.
 *	  CENTRED   both white after a shallow excursion: small hold
 *	            toward the last side for WOBBLE_MS, then straight.
 *	  RECOVER   both white after a deep excursion: the line is outside.
 *	            Turn toward it, harder the longer it is missing
 *	            (HOLD_DEEP -> HOLD_MAX over RAMP_MS) and slow down.
 *	            If the line then shows up on the OTHER sensor, it was not
 *	            a curve but an overshoot: the next excursion is treated
 *	            as shallow ("calm"), which stops a left-right pivot loop.
 *	  JUNCTION  both black: straight on.
 *	  BEND STOP a bend is detected (|steer| >= PIVOT_STEER, or the line
 *	            slid outside): brake for PIVOT_STOP_MS so the car does
 *	            not roll on past the bend ...
 *	  PIVOT     ... then turn with the INNER wheel stopped and the outer
 *	            wheel at the bend speed (left bend: left wheel stops,
 *	            right wheel drives) until the line is back in the
 *	            middle, then normal steering again.  A bend that goes on
 *	            straight after a pivot pivots again without the stop.
 *	  LOST      RECOVER for LOST_MS, or CENTRED for BLIND_MS, without
 *	            seeing the line: stop.
 */
#ifndef LINE_CTL_H
#define LINE_CTL_H

#include <stdint.h>

/* |steer| that starts a bend stop + pivot (line_sim: inside 700 passes the
   sharp R20/15 corners with a laggy car where plain steering fails) */
#ifndef PIVOT_STEER_INSIDE
#define PIVOT_STEER_INSIDE	700
#endif
#ifndef PIVOT_STEER_STRADDLE
#define PIVOT_STEER_STRADDLE	900
#endif
#ifndef PIVOT_STEER_EDGE
#define PIVOT_STEER_EDGE	700
#endif
#define LINE_EDGE		2	/* LINE_TUNE.inside value for the edge layout */

typedef struct {
	int32_t edge_dir;	/* edge layout: +1 line under the LEFT sensor, -1 RIGHT */
	int32_t inside;		/* 2: EDGE, one sensor on black, one on white (above)
				   0: sensors STRADDLE the line (both white when centred)
				   1: both sensors sit ON the line (gap narrower than the
				      line, both black when centred): both black = centred,
				      not a junction; both white = off the line, always
				      turn back toward the side it was last seen */
	int32_t see;		/* darkness that counts as "sees the line" */
	int32_t junction;	/* both above this = junction / crossing */
	int32_t kp_x1000;	/* steer per mille per pos unit, x1000 */
	int32_t kd_x1000;
	int32_t side_pos;	/* |pos| above this sets the side */
	int32_t deep_pos;	/* |pos| above this = line was under the sensor */
	int32_t fall_pos;	/* this far below the peak = "fading" */
	int32_t hold_small;	/* CENTRED hold, per mille */
	int32_t wobble_ms;
	int32_t hold_deep;	/* RECOVER start, per mille */
	int32_t hold_max;	/* RECOVER end, per mille */
	int32_t ramp_ms;
	int32_t calm_ms;	/* centred longer than this ends 'calm' */
	int32_t lost_ms;	/* RECOVER this long = LOST */
	int32_t blind_ms;	/* CENTRED with no line this long = LOST (end of line) */
	int32_t speed;		/* mm/s on straights */
	int32_t speed_min;	/* mm/s in a bend / in RECOVER */
	int32_t slow_steer;	/* |steer| at which the bend speed is fully reached
				   (lower = slows down earlier in a bend) */
	int32_t slow_hold_ms;	/* stay at bend speed this long after the bend */
	int32_t pivot_steer;	/* |steer| at which a bend starts a stop + pivot (0 = off) */
	int32_t pivot_stop_ms;	/* brake this long before pivoting */
	int32_t pivot_exit_pos;	/* |pos| below this = back in the middle: pivot done */
	int32_t pivot_rest_ms;	/* a new bend within this long after a pivot: no stop */
	int32_t pivot_max_ms;	/* pivoting this long: back to normal steering */
} LINE_TUNE;

typedef struct {
	const LINE_TUNE *k;
	int32_t prev_pos;
	int32_t side;		/* -1 left, +1 right, 0 unknown */
	int32_t peak;		/* deepest |pos| of this excursion */
	int32_t deep;		/* 1 = this excursion went deep */
	int32_t calm;		/* 1 = last RECOVER was an overshoot: no deep logic */
	uint32_t t_seen;	/* ms: last time a sensor saw the line */
	uint32_t t_bend;	/* ms: last time we were steering hard */
	int32_t mode;
	int32_t edge;		/* edge layout: +1 line under left sensor now, -1 right */
	int32_t pv;		/* 0 none, 1 bend stop, 2 pivot */
	int32_t pv_dir;		/* -1 left, +1 right */
	int32_t pv_seen;	/* the line was seen during this pivot */
	uint32_t t_pv;		/* ms: pivot phase started */
	uint32_t t_pv_end;	/* ms: last pivot ended */
} LINE_CTL;

typedef struct {
	int32_t steer;		/* -1000 .. +1000, + = right */
	int32_t speed;		/* mm/s */
	int32_t pos;
	int32_t mode;		/* LINE_M_* values from car.h */
	int32_t lost;		/* 1 = give up */
} LINE_OUT;

extern const LINE_TUNE line_tune_default;	/* straddle layout */
extern const LINE_TUNE line_tune_inside;	/* both sensors on the line */
extern const LINE_TUNE line_tune_edge;		/* one on the line, one beside it */

void line_ctl_reset(LINE_CTL *c, const LINE_TUNE *k, int32_t seed_side,
		    int32_t seed_deep, uint32_t now_ms);
void line_ctl_step(LINE_CTL *c, int32_t dark_l, int32_t dark_r,
		   uint32_t now_ms, LINE_OUT *o);

#endif
