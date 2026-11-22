/*
 *	subsystem5.h  (Buddy 5: ultrasonic, servo, obstacle avoidance)
 *	Public API in car.h.  Tunables here.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../car.h"

/* ---- ultrasonic ---- */
#define US_PER_CM		58	/* round trip, speed of sound */
#define US_MIN_CM		2	/* closer readings are glitches */

/* ---- guard while line following ---- */
#define GUARD_STOP_CM		25	/* obstacle closer than this ... */
#define GUARD_HITS		3	/* ... on this many pings ... */
#define GUARD_WINDOW		5	/* ... out of the last this many (300 ms) */
CC_ASSERT(GUARD_HITS <= GUARD_WINDOW && GUARD_WINDOW <= 16, "guard window");

/* ---- servo (SG90 on GP13) ---- */
#define SERVO_CENTER_US		1500
#define SERVO_US_PER_DEG	11	/* ~1000 us per 90 deg */
#define SERVO_REVERSE		0	/* 1 if + angle turns the sensor RIGHT */
#define SERVO_MAX_DEG		25	/* never turn further than this (team choice) */
#define SERVO_TEST_DEG		25	/* power-up self-test swing */
/* 12 Nov evening: was 30 / 30.  An SG90 overshoots and wobbles after
   the move; pinging then measured a different spot every time.  The
   first ping after any move is also thrown away (subsystem5.c). */
#define SERVO_SETTLE_MS		60	/* per 15 deg moved */
#define SERVO_SETTLE_MIN_MS	80	/* even a small move waits this long */

/* ---- sweep mode (ranger_fast): the sweep only needs AV_FAR_CM, so the
   echo window can be short and pings closer together.  10 ms of echo
   = 172 cm.  Tick-sized (10 ms) values only. */
#define SWEEP_ECHO_WAIT_MS	10
/* 12 Nov evening: was 30.  The HC-SR04 needs ~60 ms for the last ping's
   echoes (walls, furniture) to die away; at 30 ms they came back as
   random SHORT readings in the next ping. */
#define SWEEP_PING_PERIOD_MS	60

/* ---- avoidance (mission.c).  All CAL on the real track. ---- */
#define AV_CONFIRM_MS		100	/* let the car stop rolling (the servo is already
				   swinging to the first angle meanwhile), then
				   sweep.  No separate re-check: the guard saw it
				   3 times, and the sweep's middle readings say
				   if it has gone. */
#define AV_IDLE_MAX_MS		2000	/* the fan sweep starts no later than this
				   after the stop, however patchy the echoes */
#define AV_CLEAR_CM		35	/* front farther than this = clear (> GUARD_STOP_CM,
				   or a car stopped at 26 cm would call it a false alarm) */
/* fan sweep: ONE pass +24 -> -24 in 12 deg steps, 5 points.  It only
   picks the side now (the edge is found by turning the car), so fewer,
   better readings beat many noisy ones.  AV_SWEEP_MAX_DEG must be a
   multiple of AV_SWEEP_STEP_DEG. */
#define AV_SWEEP_MAX_DEG	24
#define AV_SWEEP_STEP_DEG	12
#define AV_SWEEP_PTS		(2 * AV_SWEEP_MAX_DEG / AV_SWEEP_STEP_DEG + 1)
#define AV_PT_TIMEOUT_MS	450	/* no echo at an angle this long = free (255) */
#define AV_PINGS_PER_PT		3	/* readings per angle; the MEDIAN is kept (was the
				   closest, so one ghost echo always won) */
#define AV_BAND_CM		20	/* same obstacle if within closest + this */
#define AV_FAR_CM		100	/* farther = free space */
#define AV_BEAM_DEG		0	/* sound cone makes edges look wider: pull in.
				   12 Nov: was 8, as large as the 8 deg sweep step, so
				   a box seen at two angles shrank to 2 cm wide
				   (18:48 run).  0 = never smaller than seen. */
#define AV_HALF_CAR_CM		7	/* half the car's width incl. wheels: 12.5 cm measured 12 Nov, rounded up */
#define AV_MARGIN_CM		5	/* extra room when passing */
#define AV_PREFER_LEFT		1	/* tie -> go round on the left */
/* ---- going round: turns measured by the wheel encoders (heading),
   distances by odometry.  Route, for an obstacle needing a sideways
   move N (from the sweep), going round on side d:
     OUT    turn AV_TURN_DEG toward d
     WIDEN  drive N / sin(AV_TURN_DEG): now N to the side of the line
     ALIGN  turn back: parallel to the line
     PASS   drive past the obstacle (its distance + depth + car length)
     BACK   turn AV_TURN_DEG toward the line
     SEEK   drive until a line sensor sees black
     ADVANCE drive AV_SENSOR_FWD_CM more: the wheels are now on the line
     REJOIN turn back to the heading we had when we stopped, FOLLOW
   Facing the same way as before the obstacle is what stops the car
   from going round in circles or following the line backwards. */
#define AV_TURN_DEG		45	/* CAL: angle of the side legs */
#define AV_SPIN_MMS		80	/* wheel speed for spins in place */
#define AV_SPIN_LEAD_D10	40	/* stop spinning this early (0.1 deg): the
				   wheels coast a little after the brake */
#define AV_DRIVE_MMS		80	/* speed on every straight leg */
#define AV_SPIN_DPS		75	/* CAL: deg per second at AV_SPIN_MMS, only
				   used if an encoder fails (timed turns) */
#define AV_OBS_DEPTH_CM		20	/* CAL: assumed front-to-back size of an obstacle */
#define AV_CAR_LEN_CM		20	/* CAL: car length, MEASURE */
#define AV_SENSOR_FWD_CM	6	/* CAL: line sensors ahead of the wheel axle, MEASURE */
#define AV_SEEK_EXTRA_CM	15	/* look this far beyond where the line should be */
#define AV_FIND_DEG		35	/* back facing along the line but no sensor on it:
				   sweep this far each way to find it */
#define AV_FIND_MMS		50	/* slow spin while sweeping */
#define AV_SIDE_DEG		25	/* servo angle while passing (toward obstacle) */
/* With a narrow sweep most obstacles reach past the edge of the view.
   Then the car can't measure the width: it goes round on the side with
   more room at the sweep's ends, moving this far sideways. */
#define AV_UNKNOWN_NEED_CM	35
#define AV_SIDE_SEE_CM		30	/* side reading below this = obstacle still beside us */
#define AV_PASS_MORE_CM		20	/* still beside us at the end of PASS: up to this much more */
#define AV_SEEK_MMS		70	/* drive back toward the line */
#define AV_SEEK_STOP_CM		12	/* on the way back: something this close -> stop */
#define AV_BLOCKED_RETRY_MS	500	/* both sides blocked: sweep again after this */
#define AV_BLOCKED_SWEEPS	2	/* blocked this many sweeps in a row: go round anyway */
#define AV_RECHECK_PINGS	8	/* sweep said clear: this many clear pings IN A ROW
				   ahead to believe it.  The 18:04 run (v3_23) got 3
				   misses in a row with the box at 19 cm. */
#define AV_RECHECK_MAX_MS	1200	/* no answer in this long: assume it is still there */
#define AV_BACKWARD_DEG		90	/* after an obstacle, facing this far off the old
				   heading = following the line backwards.
				   12 Nov 18:48 run: was 120, the recovery spin had
				   already turned the car ~120 deg before it fired.
				   Too low and a real bend right after the obstacle
				   trips it: raise again if that happens. */
#define AV_BACKWARD_CHECK_MM	500	/* ... checked for this far after rejoining */
#define AV_BACKWARD_TRIES	2	/* turned back this many times: give up (fault) */
/* 12 Nov 18:48 run: the guard stopped the car at 10 cm.  That close the
   +-24 deg sweep missed the obstacle, it was called "gone", and there
   was no room to turn out.  Stopped closer than AV_BACKUP_BELOW_CM ->
   reverse straight back to about AV_BACKUP_TO_CM first, then sweep. */
#define AV_BACKUP_BELOW_CM	15
#define AV_BACKUP_TO_CM		22
#define AV_BACKUP_MMS		60
#define AV_BACKUP_MAX_MS	2000	/* give up reversing after this (wheels stuck) */
#define AV_BACKUP_SETTLE_MS	150	/* stop rolling before the sweep */
CC_ASSERT(AV_BACKUP_TO_CM > AV_BACKUP_BELOW_CM, "AV_BACKUP_TO_CM must be above AV_BACKUP_BELOW_CM");

/* ---- going round, sensor-guided (12 Nov evening).  The +-25 deg servo
   with a ~30 deg sound cone can't see past the edges of an obstacle at
   20 cm, so the old box route fell back to a fixed 35 cm guess.  Now:
     EDGE   spin slowly toward the chosen side, pinging straight ahead,
            until the reading jumps past the obstacle = the beam has
            cleared its edge (wide obstacle -> spins further)
     AIM    turn to (that angle - AV_CONE_DEG + angle for half the car
            and the margin at the obstacle's distance)
     WIDEN  drive along that line until level with the obstacle's front
     ALIGN  parallel to the line again
     PASS   drive on, servo looking AV_SIDE_DEG toward the obstacle,
            until AV_PASS_CLEAR_CM past the farthest point it saw
     BACK / SEEK / ADVANCE / REJOIN  as before
   CALIBRATE AV_CONE_DEG: put a box of known width in front, run, and
   compare the "MISSION edge ... edge X deg = Y cm sideways" console line
   with the real half-width (plus any offset from the car's centre). */
#define AV_EDGE_MMS		25	/* wheel speed for the slow edge-finding spin:
				   ~27 deg/s with the 108 mm track = ~1.6 deg per
				   60 ms ping (12 Nov: 40 gave +-3 cm width scatter) */
#define AV_EDGE_CLEAR_PINGS	3	/* readings in a row past the obstacle = edge */
#define AV_EDGE_MAX_DEG		75	/* no edge within this: try the other side */
#define AV_EDGE_MAX_MS		8000	/* per side (the 2nd side first swings back
				   across the front), in case the heading is stuck */
#define AV_CONE_DEG		16	/* CAL: the beam clears the edge this many degrees
				   after the car's centre line does.  Too big = cuts
				   too close; too small = goes too wide.
				   12 Nov: 13.5 cm box read 22 / 20 / 26 cm at 8
				   -> ~16 deg.  Re-check with the box. */
#define AV_AIM_MAX_DEG		75	/* never aim further out than this */
#define AV_NEED_MAX_CM		40	/* sideways move never more than this: a 30 cm
				   obstacle needs ~29 cm; more = a bad reading */
#define AV_US_FWD_CM		9	/* ultrasonic ahead of the wheel axle: 8.5 cm measured 12 Nov, rounded up */
#define AV_PASS_CLEAR_CM	15	/* axle this far past the obstacle's far end
				   before turning back toward the line */
#define AV_PASS_MIN_DEPTH_CM	5	/* assume at least this deep before it is seen */
#define AV_PASS_MAX_CM		100	/* PASS never longer than this */
#define AV_PASS_SEE_CM		60	/* side reading below this = obstacle
				   (at 14 cm sideways the 25 deg beam meets it at ~33 cm) */
CC_ASSERT(AV_CONE_DEG >= 0 && AV_CONE_DEG < 30, "AV_CONE_DEG");

/* ---- long obstacles: look before turning back (12 Nov evening).  The
   side-looking ping meets a flat side at a glancing angle and often gets
   no echo, so after PASS the car turns AV_TURN_DEG toward the line,
   stops and pings straight ahead.  Still there -> turn parallel again,
   drive AV_PEEK_STEP_CM more, look again. */
#define AV_PEEK_PINGS		3	/* readings per look (median) */
#define AV_PEEK_MAX_MS		700	/* give up waiting for readings after this */
#define AV_PEEK_CLEAR_CM	35	/* median farther than this = way back is clear */
#define AV_PEEK_STEP_CM		15
#define AV_PEEK_MAX_CM		120	/* extra driving allowed before "too long" fault */

/* ---- bench test: while IDLE the ranger prints straight-ahead statistics
   to the USB console every RANGE_BENCH_MS (min / median / max, valid
   pings), plus the line sensors' raw values.  0 = off. */
#define RANGE_BENCH_MS		2000

CC_ASSERT(AV_IDLE_MAX_MS > AV_CONFIRM_MS + 3 * PING_PERIOD_MS, "AV_IDLE_MAX_MS too short for 3 pings");
CC_ASSERT(SWEEP_ECHO_WAIT_MS * 1000 / US_PER_CM > AV_FAR_CM + 20, "sweep echo window too short for AV_FAR_CM");
CC_ASSERT(SWEEP_ECHO_WAIT_MS < SWEEP_PING_PERIOD_MS, "sweep ping timing");
CC_ASSERT(AV_CLEAR_CM >= GUARD_STOP_CM + 5, "AV_CLEAR_CM must be well above GUARD_STOP_CM");
CC_ASSERT(AV_SWEEP_MAX_DEG % AV_SWEEP_STEP_DEG == 0,
	  "AV_SWEEP_MAX_DEG must be a multiple of AV_SWEEP_STEP_DEG");
CC_ASSERT(AV_SWEEP_MAX_DEG <= SERVO_MAX_DEG && AV_SIDE_DEG <= SERVO_MAX_DEG,
	  "sweep / side angle beyond SERVO_MAX_DEG");

IMPORT void ranger_led(BOOL on);	/* alert LED on GP14 */

#endif
