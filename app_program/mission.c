/**
 * @file    mission.c
 *  mission.c
 *  Top-level behaviour.  mission task (priority 11, core 1), every 20 ms
 *  or at once on an event.
 *
 *    IDLE --START--> FOLLOW --obstacle--> AVOID ... --line found--> FOLLOW
 *     ^                 |  line lost                |  any failure
 *     +------STOP-------+-----------> FAULT <-------+
 *
 *  STOP (GP21) works in every state: brake at once, back to IDLE.
 *
 *  Going round an obstacle:
 *    CONFIRM  wait AV_CONFIRM_MS for the car to stop (the servo swings
 *             to the first angle meanwhile)
 *    SCAN     fan sweep, one pass: the servo swings the ultrasonic from
 *             24 deg left to 24 deg right (7 angles, ~0.5 s) with short
 *             close-range pings (ranger_fast).  Straight ahead clear =
 *             it has gone / false alarm -> FOLLOW.  scan_profile_compute()
 *             turns that into distance, bearing, WIDTH and both edges,
 *             and picks the side that needs the smaller sideways move
 *    the sweep only picks the side.  Then (see subsystem5.h):
 *    EDGE  spin slowly that way until the ping gets past the obstacle
 *    AIM / WIDEN  turn past that edge (+ half the car + margin), drive out
 *    ALIGN / PASS parallel, servo watching it, until past its far end
 *    BACK / SEEK / ADVANCE / REJOIN  back to the line
 *    Turns MEASURED by the wheel encoders, legs by odometry.  REJOIN turns the
 *car back to the heading it had when it stopped, so it faces along the line
 *again. If an encoder has failed, turns fall back to timing (AV_SPIN_DPS).
 */
#include "car.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/scan_profile.h"
#include "mission.h" /* own header (BARR-C 4.3.c) */

LOCAL ID                g_mission_flg;
LOCAL obstacle_info_t   g_obs; /* last sweep, for telemetry */
LOCAL volatile uint32_t g_state     = MS_IDLE;
LOCAL const char * volatile gp_text = "IDLE";

/**
 * @brief  Mission event.
 */
EXPORT void mission_event (uint32_t ev_val)
{
    if (g_mission_flg > 0)
    {
        tk_set_flg(g_mission_flg, ev_val);
    }
}
/**
 * @brief  Mission state.
 * @return Result as described above.
 */
EXPORT uint32_t mission_state (void)
{
    return g_state;
}
/**
 * @brief  Mission text.
 * @return Result as described above.
 */
EXPORT const char * mission_text (void)
{
    return (const char *)gp_text;
}
/**
 * @brief  Mission obstacle.
 */
EXPORT void mission_obstacle (obstacle_info_t * p_out)
{
    *p_out = g_obs;
}

/* Every obstacle step goes into a small ring so the comms task can't miss
   one even if two happen within its 100 ms period.
   mission task writes obs_head, comms task writes obs_tail. */
#define OBS_RING 8
LOCAL obstacle_info_t   g_obs_ring[OBS_RING];
LOCAL volatile uint32_t g_obs_head;
LOCAL volatile uint32_t g_obs_tail;

/**
 * @brief  Mission obstacle pop.
 * @return Result as described above.
 */
EXPORT BOOL mission_obstacle_pop (obstacle_info_t * p_out)
{
    uint32_t tval = g_obs_tail;
    if (tval == g_obs_head)
    {
        return FALSE;
    }
    *p_out     = g_obs_ring[tval];
    g_obs_tail = (tval + 1U) % OBS_RING;
    return TRUE;
}

/**
 * @brief  Obs note.
 */
LOCAL void obs_note (uint32_t stage, int32_t dist_cm)
{
    uint32_t hval      = g_obs_head;
    uint32_t next_head = (hval + 1U) % OBS_RING;
    g_obs.stage        = stage;
    if (dist_cm >= 0)
    {
        g_obs.dist_cm = dist_cm;
    }
    g_obs.seq++;
    g_obs_ring[hval] = g_obs;
    if (next_head != g_obs_tail)
    {
        g_obs_head = next_head; /* full: overwrite the newest slot */
    }
}

LOCAL uint32_t g_av_state; /* avoidance sub-state (below) */
enum
{
    AV_CONFIRM,
    AV_BACKUP,
    AV_SCAN,
    AV_BLOCKED,
    AV_RECHECK,
    AV_EDGE,
    AV_AIM,
    AV_OUT,
    AV_WIDEN,
    AV_ALIGN,
    AV_PASS,
    AV_BACK,
    AV_SEEK,
    AV_ADVANCE,
    AV_REJOIN,
    AV_FIND1,
    AV_FIND2,
    AV_PEEK,
    AV_PEEK_ALIGN,
    AV_PEEK_FWD
};

/** robot/status mode (team list: INIT IDLE LINE BARCODE AVOID RECOVERY
   PAUSED SAFE_STOP COMPLETE) */
EXPORT uint32_t mission_sys_mode (void)
{
    switch (g_state)
    {
        case MS_IDLE:
            return 1;
        case MS_FOLLOW:
            return 2;
        case MS_TURN:
            return 3;
        case MS_REVERSE:
            return 5;
        case MS_AVOID:
            return (g_av_state >= AV_BACK) ? 5 : 4;
        case MS_PAUSED:
            return 6;
        case MS_DONE:
            return 8;
        default:
            return 7; /* FAULT = safe stop */
    }
}

/* ---------------- avoidance sub-states ---------------- */
LOCAL const char * const gp_av_text[] = {"AVOID confirm",
                                         "AVOID back up",
                                         "AVOID fan scan",
                                         "AVOID blocked",
                                         "AVOID re-check ahead",
                                         "AVOID find edge",
                                         "AVOID aim past edge",
                                         "AVOID turn out",
                                         "AVOID widen",
                                         "AVOID align",
                                         "AVOID pass",
                                         "AVOID turn back",
                                         "AVOID seek line",
                                         "AVOID onto line",
                                         "AVOID rejoin",
                                         "AVOID find line (1)",
                                         "AVOID find line (2)",
                                         "AVOID look toward line",
                                         "AVOID still beside it: parallel",
                                         "AVOID still beside it: on"};

LOCAL uint32_t g_t_start; /* entry time of the sub-state, ms */
LOCAL int32_t  g_dir;     /* +1 = going round on the LEFT, -1 = right */
/* heading when we stopped (= along the line), 0.1 deg */
LOCAL int32_t g_h_start;
LOCAL int32_t g_h_goal;    /* heading the current spin is turning to */
LOCAL int32_t g_spin_dir;  /* +1 left, -1 right */
LOCAL int32_t g_leg_mm;    /* length of the current straight leg */
LOCAL int32_t g_leg_odo0;  /* odometry when it started */
LOCAL int32_t g_leg_head;  /* heading held on this leg, 0.1 deg */
LOCAL int32_t g_leg_mms;   /* speed of this leg */
LOCAL int32_t g_blocked_n; /* sweeps in a row that said "blocked" */
/* following again after an obstacle: watch the heading */
LOCAL BOOL    gb_post_avoid;
LOCAL int32_t g_post_odo0; /* odometry when following restarted */
/* times we turned the car back to h0 for this obstacle */
LOCAL int32_t g_back_n;
LOCAL int32_t g_need_cm;    /* sideways move: where the car's centre ends up */
LOCAL int32_t g_obs_d;      /* obstacle's front, cm from the ultrasonic */
LOCAL int32_t g_edge_phase; /* 1 = measuring the first side, 2 = the second */
LOCAL int32_t g_edge_seen[2]; /* beam-clear angle per side, deg ([0] LEFT, [1]
                           RIGHT), -1 = none */
LOCAL int32_t g_e_side_cm;    /* edge position on the side we pass, cm */
/* farthest point of the obstacle's side seen, odometry mm */
LOCAL int32_t g_far_seen;
/* how far out the WIDEN leg points (for timed turns) */
LOCAL int32_t g_aim_d10;
/* PASS: start, and odometry where it may turn back */
LOCAL int32_t g_pass_odo0;
LOCAL int32_t g_pass_end;
/* driven on because a look said "still there", mm */
LOCAL int32_t  g_peek_extra;
LOCAL int32_t  g_line_side; /* where the line was when we stopped */
LOCAL int32_t  g_num;       /* small counters for the current sub-state */
LOCAL int32_t  g_best;
LOCAL int32_t  g_hits;
LOCAL int32_t  g_sweep_k; /* which point of the sweep we are on */
LOCAL int32_t  g_sweep_aim;
LOCAL uint32_t g_t_pt; /* when we aimed at this point */
LOCAL int32_t  g_pt_n; /* readings taken at this point */
LOCAL int32_t  g_pt_cm;
LOCAL int32_t  g_pt_buf[8]; /* ... and their values (median kept) */
/* what the guard saw straight ahead when we stopped */
LOCAL int32_t        g_guard_cm;
LOCAL int32_t        g_sweep_deg[AV_SWEEP_PTS];
LOCAL int32_t        g_sweep_cm[AV_SWEEP_PTS];
LOCAL scan_profile_t g_prof;

LOCAL uint32_t g_t_stop; /* when the car stopped for this obstacle */

LOCAL const scan_tune_t g_scan_tune = {AV_BAND_CM,   AV_FAR_CM,
                                       AV_BEAM_DEG,  AV_HALF_CAR_CM,
                                       AV_MARGIN_CM, AV_PREFER_LEFT};

/** k-th angle of the fan: one pass, left to right */
LOCAL int32_t sweep_angle (int32_t kdx)
{
    return AV_SWEEP_MAX_DEG - (kdx * AV_SWEEP_STEP_DEG);
}
/* table slot, left to right: +80 -> 0 ... -80 -> 16 */
#define SLOT(deg) ((AV_SWEEP_MAX_DEG - (deg)) / AV_SWEEP_STEP_DEG)

/**
 * @brief  Sweep aim next.
 */
LOCAL void sweep_aim_next (uint32_t tval)
{
    g_sweep_aim = sweep_angle(g_sweep_k);
    ranger_aim(g_sweep_aim);
    g_t_pt  = tval;
    g_pt_n  = 0;
    g_pt_cm = RANGE_FAR_CM;
}
/** median of n small values (sorts them in place); n = 0 -> far */
LOCAL int32_t median_of (int32_t * p_val, int32_t num_x)
{
    int32_t aval;
    int32_t bval;
    int32_t xval;
    if (num_x <= 0)
    {
        return RANGE_FAR_CM;
    }
    for (aval = 1; aval < num_x; aval++)
    {
        for (bval = aval; (bval > 0) && (p_val[bval - 1] > p_val[bval]); bval--)
        {
            xval            = p_val[bval];
            p_val[bval]     = p_val[bval - 1];
            p_val[bval - 1] = xval;
        }
    }
    return p_val[num_x / 2];
}
LOCAL BOOL     gb_seen;
LOCAL uint32_t g_t_mark;
LOCAL uint32_t g_last_seq;
LOCAL BOOL     gb_marked; /** t_mark is set (t_mark itself may be 0) */

LOCAL void go_av (uint32_t sval, uint32_t tval)
{
    g_av_state = sval;
    g_t_start  = tval;
    g_num      = 0;
    g_best     = 0;
    g_hits     = 0;
    gb_seen    = FALSE;
    g_t_mark   = 0;
    gb_marked  = FALSE;
    gp_text    = gp_av_text[sval];
    tm_printf((uint8_t *)"MISSION %s\n", gp_av_text[sval]);
}

/** new valid reading taken at the angle we asked for? */
LOCAL BOOL ping (int32_t aim, int32_t * p_cm_val)
{
    uint32_t wval = ranger_word();
    if (RW_SEQ(wval) == g_last_seq)
    {
        return FALSE;
    }
    g_last_seq = RW_SEQ(wval);
    if ((!RW_VALID(wval)) || (RW_DEG(wval) != aim))
    {
        return FALSE;
    }
    *p_cm_val = RW_CM(wval);
    return TRUE;
}

/** spin in place: + = left (counter-clockwise) */
LOCAL void spin (int32_t dval)
{
    motion_wheels(MOT_OWNER_MISSION, -dval * AV_SPIN_MMS, dval * AV_SPIN_MMS);
    motion_set_owner(MOT_OWNER_MISSION);
}
/**
 * @brief  Forward.
 */
LOCAL void forward (int32_t mms)
{
    motion_wheels(MOT_OWNER_MISSION, mms, mms);
    motion_set_owner(MOT_OWNER_MISSION);
}

/** turn on the spot to an absolute heading (0.1 deg, + = left) */
LOCAL void turn_to (int32_t goal)
{
    g_h_goal   = goal;
    g_spin_dir = ((goal - motion_heading_d10()) >= 0) ? 1 : -1;
    spin(g_spin_dir);
}
/** TRUE when the turn has arrived (by encoders, or by time if one failed) */
LOCAL BOOL turn_done (uint32_t dt_val, int32_t deg_d10_total)
{
    int32_t left;
    if (!motion_heading_ok()) /* timed fallback */
    {
        return dt_val >= (uint32_t)((ABS(deg_d10_total) * 100) / AV_SPIN_DPS);
    }
    left = (g_h_goal - motion_heading_d10()) * g_spin_dir;
    return left <= AV_SPIN_LEAD_D10;
}
/**
 * @brief  Turn to slow.
 */
LOCAL void turn_to_slow (int32_t goal)
{
    g_h_goal   = goal;
    g_spin_dir = ((goal - motion_heading_d10()) >= 0) ? 1 : -1;
    motion_wheels(MOT_OWNER_MISSION, -g_spin_dir * AV_FIND_MMS,
                  g_spin_dir * AV_FIND_MMS);
    motion_set_owner(MOT_OWNER_MISSION);
}
/**
 * @brief  Leg.
 */
LOCAL void leg (int32_t mm_val, int32_t mms)
{
    g_leg_mm   = mm_val;
    g_leg_odo0 = motion_odometry_mm();
    g_leg_head = g_h_goal; /* the heading the last turn aimed for */
    g_leg_mms  = mms;
    forward(mms);
}
/* 12 Nov: straight legs steer back to leg_head with the encoder
   heading.  Open loop, a leg curved with any left/right motor
   difference and the few degrees a spin over/undershoots, so the car
   met the line far from where the box route expected (or missed it).
   Set AV_HEADING_HOLD to 0 to go back to open-loop legs. */
#ifndef AV_HEADING_HOLD
#define AV_HEADING_HOLD 1
#endif
/* correction mm/s = error (0.1 deg) * speed / this */
#define AV_HOLD_DIV 300
LOCAL void leg_hold (void)
{
    int32_t err;
    int32_t corr;

    if ((!AV_HEADING_HOLD) || (!motion_heading_ok()))
    {
        return; /* encoder fault: open loop */
    }
    err  = g_leg_head - motion_heading_d10(); /* + = turn left */
    corr = (int32_t)((err * g_leg_mms) / AV_HOLD_DIV);
    corr = CLAMP(corr, -g_leg_mms / 2, g_leg_mms / 2);
    motion_wheels(MOT_OWNER_MISSION, g_leg_mms - corr, g_leg_mms + corr);
}
/**
 * @brief  Leg done mm.
 * @return Result as described above.
 */
LOCAL int32_t leg_done_mm (void)
{
    return motion_odometry_mm() - g_leg_odo0;
}

/* ---------------- climb mode (12 Nov evening) ----------------
   On a slope or one-sided ramp, or with a driven wheel not turning, the
   zig-zag (pivoting on one wheel at ~15 % duty) got the car stuck.
   Climb mode: line following goes on, but faster and with both wheels
   always forward (line_climb).  Still stuck after CLIMB_STUCK_MS -> back
   off CLIMB_BACK_MM and try again with a run-up, CLIMB_RETRIES times,
   then a clean "stuck on a ramp" safe stop. */
#define CLIMB_TILT_D10     80  /* nose up or tilted sideways by 8 deg ... */
#define CLIMB_ON_MS        300 /* ... for this long -> climb mode */
#define CLIMB_OFF_MS       600 /* level and moving this long -> normal again */
#define CLIMB_STALL_WIN_MS 500 /* wheel progress checked over this window */
/* < ~3 mm in the window while driven = stalled */
#define CLIMB_STALL_STEPS 36
/* NOTE: only a wheel driven at this speed or more can count as stalled.
   Was 15 mm/s: the slow inner wheel of the zig-zag looked stalled and
   switched climb mode (70 mm/s, capped steering) on, so the car ran off
   curves after rejoining the line. */
#define CLIMB_STALL_MIN_MMS 25
/* stalled this long in climb mode -> back off */
#define CLIMB_STUCK_MS 1500
#define CLIMB_RETRIES  2
#define CLIMB_BACK_MM  60
#define CLIMB_BACK_MMS 60

LOCAL uint32_t g_climb; /* 0 off, 1 climbing, 2 backing off for a run-up */
LOCAL int32_t  g_climb_tries;
LOCAL uint32_t g_climb_t;
LOCAL uint32_t g_calm_t;
LOCAL uint32_t g_tilt_t;
LOCAL uint32_t g_stuck_t;
LOCAL uint32_t g_wpos_t;
LOCAL int32_t  g_wpos[2];
LOCAL int32_t  g_climb_odo;
LOCAL BOOL     gb_wstalled;

/**
 * @brief  Climb reset.
 */
LOCAL void climb_reset (void)
{
    g_climb       = 0;
    g_climb_tries = 0;
    g_tilt_t = g_calm_t = g_stuck_t = g_wpos_t = 0;
    gb_wstalled                                = FALSE;
    line_climb(FALSE);
}

/**
 * @brief  Is a wheel stalled?  A healthy wheel driven at
 *         CLIMB_STALL_MIN_MMS or more that moved less than about 3 mm in
 *         the last CLIMB_STALL_WIN_MS.
 * @param  tval  Current time, ms.
 * @return TRUE if a wheel is stalled.
 */
LOCAL BOOL wheels_stalled (uint32_t tval)
{
    motion_status_t ms_val;
    uint32_t        mval;

    if ((g_wpos_t != 0) && ((tval - g_wpos_t) < CLIMB_STALL_WIN_MS))
    {
        return gb_wstalled;
    }
    motion_get_status(&ms_val);
    gb_wstalled = FALSE;
    for (mval = 0; mval < 2; mval++)
    {
        if ((((g_wpos_t != 0) && (ms_val.healthy[mval])) &&
             (ABS(ms_val.tgt_mms[mval]) >= CLIMB_STALL_MIN_MMS)) &&
            (ABS(ms_val.pos[mval] - g_wpos[mval]) < CLIMB_STALL_STEPS))
        {
            gb_wstalled = TRUE;
        }
        g_wpos[mval] = ms_val.pos[mval];
    }
    g_wpos_t = tval;
    return gb_wstalled;
}

LOCAL void fault (const char * p_why, uint32_t bits);

/** called every mission tick while following; TRUE = it faulted */
LOCAL BOOL climb_step (uint32_t tval)
{
    imu_status_t im_val;
    BOOL         b_tilt;
    BOOL         b_stalled;

    imu_get(&im_val);
    /* nose UP or sideways: going down a slope needs no help */
    b_tilt    = (im_val.ok) && ((im_val.pitch_d10 >= CLIMB_TILT_D10) ||
                             (ABS(im_val.roll_d10) >= CLIMB_TILT_D10));
    b_stalled = (2 == g_climb) ? FALSE : wheels_stalled(tval);

    switch (g_climb)
    {
        case 0:
            if (b_tilt)
            {
                if (0 == g_tilt_t)
                {
                    g_tilt_t = tval;
                }
            }
            else
            {
                g_tilt_t = 0;
            }
            if (((g_tilt_t != 0) && ((tval - g_tilt_t) >= CLIMB_ON_MS)) ||
                b_stalled)
            {
                g_climb   = 1;
                g_climb_t = tval;
                g_calm_t = g_stuck_t = 0;
                line_climb(TRUE);
                gp_text = "FOLLOW (climb)";
                tm_printf((uint8_t *)"MISSION climb mode: pitch %d roll %d "
                                     "(0.1 deg)%s\n",
                          im_val.pitch_d10, im_val.roll_d10,
                          b_stalled ? ", a wheel is stalled" : "");
            }
        break;
        case 1:
            if ((!b_tilt) && (!b_stalled))
            {
                if (0 == g_calm_t)
                {
                    g_calm_t = tval;
                }
                if ((tval - g_calm_t) >= CLIMB_OFF_MS)
                {
                    tm_printf((uint8_t *)"MISSION climb done (%d ms)\n",
                              (int32_t)(tval - g_climb_t));
                    climb_reset();
                    gp_text = "FOLLOW";
                    break;
                }
            }
            else
            {
                g_calm_t = 0;
            }
            if (b_stalled)
            {
                if (0 == g_stuck_t)
                {
                    g_stuck_t = tval;
                }
            }
            else
            {
                g_stuck_t = 0;
            }
            if ((g_stuck_t != 0) && ((tval - g_stuck_t) >= CLIMB_STUCK_MS))
            {
                if (g_climb_tries >= CLIMB_RETRIES)
                {
                    fault("stuck on a ramp", FAULT_MOTION);
                    return TRUE;
                }
                g_climb_tries++;
                g_climb     = 2;
                g_climb_t   = tval;
                g_climb_odo = motion_odometry_mm();
                motion_wheels(MOT_OWNER_MISSION, -CLIMB_BACK_MMS,
                              -CLIMB_BACK_MMS);
                motion_set_owner(MOT_OWNER_MISSION);
                tm_printf(
                    (uint8_t *)"MISSION still stuck: backing off %d mm for a "
                               "run-up (try %d of %d)\n",
                    CLIMB_BACK_MM, g_climb_tries, CLIMB_RETRIES);
            }
        break;
        case 2:
            if (((g_climb_odo - motion_odometry_mm()) >= CLIMB_BACK_MM) ||
                ((tval - g_climb_t) > 2000U))
            {
                motion_set_owner(MOT_OWNER_LINE); /* climb mode still on: go */
                g_climb   = 1;
                g_climb_t = tval;
                g_stuck_t = g_calm_t = g_wpos_t = 0;
                tm_printf((uint8_t *)"MISSION run-up\n");
            }
        break;
        default:
        {
            /* No other values are expected. */
            break;
        }
    }
    return FALSE;
}

/**
 * @brief  Halt all.
 */
LOCAL void halt_all (void)
{
    climb_reset();
    ranger_fast(FALSE);
    motion_stop();
    line_follow(FALSE, 0, FALSE);
    ranger_guard(FALSE);
    ranger_aim(0);
}

#define SPIN_HIST 31 /* 30 s of one-second heading samples */
LOCAL uint32_t g_spin_t0;
LOCAL int32_t  g_spin_hist[SPIN_HIST];
LOCAL uint32_t g_spin_i;

/**
 * @brief  Begin follow.
 */
LOCAL void begin_follow (int32_t side, BOOL b_deep)
{
    climb_reset();
    g_spin_t0 = motion_now_ms() - 1000U; /* take a sample at once */
    g_spin_i  = 0;
    if ((((MS_IDLE == g_state) || (MS_FAULT == g_state)) ||
         (MS_PAUSED == g_state)) ||
        (MS_DONE == g_state))
    {
        comms_fault_clear(FAULT_LINE | FAULT_AVOID | FAULT_MOTION);
    }
    /* 12 Nov: coming back from an obstacle, the line follower can catch
       the line facing the wrong way and follow it back to the start.
       Watch the heading for the first AV_BACKWARD_CHECK_MM. */
    gb_post_avoid = (MS_AVOID == g_state);
    g_post_odo0   = motion_odometry_mm();
    ranger_fast(FALSE);
    ranger_aim(0);
    ranger_led(FALSE);
    line_follow(TRUE, side, b_deep);
    motion_set_owner(MOT_OWNER_LINE);
    ranger_guard(TRUE);
    g_state = MS_FOLLOW;
    gp_text = "FOLLOW";
    tm_printf((uint8_t *)"MISSION FOLLOW (seed side %d%s)\n", side,
              b_deep ? ", turn to it" : "");
}

LOCAL void obs_note (uint32_t stage, int32_t dist_cm);
/**
 * @brief  Fault.
 */
LOCAL void fault (const char * p_why, uint32_t bits)
{
    if (MS_AVOID == g_state)
    {
        obs_note(OBS_FAILED, -1);
    }
    comms_fault_raise(bits);
    halt_all();
    ranger_led(TRUE);
    g_state = MS_FAULT;
    gp_text = p_why;
    tm_printf((uint8_t *)"MISSION FAULT: %s.  Press START to try again.\n",
              p_why);
}

/**
 * @brief  Begin avoid.
 */
LOCAL void begin_avoid (uint32_t tval)
{
    line_status_t lval;

    motion_stop(); /* brake first, think later */
    climb_reset();
    g_t_stop  = tval;
    g_h_start = motion_heading_d10();
    line_get(&lval);
    g_line_side = lval.side;
    line_follow(FALSE, 0, FALSE);
    ranger_guard(FALSE);
    ranger_aim(
        AV_SWEEP_MAX_DEG); /* servo swings to the first angle while we brake */
    ranger_led(TRUE);
    g_state = MS_AVOID;
    /* the guard's last close reading: with "3 of 5" the latest ping
       may well be a missed echo (255) */
    g_guard_cm = ranger_guard_cm();
    if (g_guard_cm > AV_CLEAR_CM)
    {
        g_guard_cm = AV_CLEAR_CM; /* never a far reading */
    }
    tm_printf((uint8_t *)"MISSION obstacle at %d cm\n", g_guard_cm);
    g_obs.bearing_deg   = 0;
    g_obs.width_cm      = 0;
    g_obs.need_left_cm  = 0;
    g_obs.need_right_cm = 0;
    g_obs.dir           = 0;
    obs_note(OBS_DETECTED, g_guard_cm);
    g_blocked_n = 0;
    g_back_n    = 0;
    go_av(AV_CONFIRM, tval);
}

/** back on the line after an obstacle: hand over to the line follower.
   We came from the 'dir' side, so the line is on the other side of the
   car: right (+1) if we went round on the left. */
LOCAL void rejoined (void)
{
    /* 12 Nov: a sensor on black says where the line really is (as at
       the end of a barcode turn); the 'came from' guess is only used
       when both sensors see black */
    uint32_t see  = line_see_bits();
    int32_t  side = (1U == see) ? -1 : (2U == see) ? 1 : ((g_dir > 0) ? 1 : -1);

    obs_note(OBS_REJOINED, -1);
    tm_printf((uint8_t *)"MISSION rejoin: sensors %d, line to the %s\n",
              (int32_t)see, (side > 0) ? "RIGHT" : "LEFT");
    begin_follow(side, TRUE);
}

/** start (or restart) the fan sweep */
LOCAL void start_sweep (uint32_t tval)
{
    go_av(AV_SCAN, tval);
    g_sweep_k = 0;
    ranger_fast(TRUE);
    sweep_aim_next(tval);
}

/** ---- sensor-guided way round (see subsystem5.h) ---- */
LOCAL int32_t icos_x1000 (int32_t deg)
{
    return scan_profile_isin_x1000(90 - deg);
}
/** smallest whole degree 0..89 whose tangent is >= num / den */
LOCAL int32_t atan_deg (int32_t num, int32_t den)
{
    int32_t aval;
    if (num <= 0)
    {
        return 0;
    }
    if (den <= 0)
    {
        return 89;
    }
    for (aval = 0; aval < 89; aval++)
    {
        if (((int32_t)scan_profile_isin_x1000(aval) * den) >=
            ((int32_t)icos_x1000(aval) * num))
        {
            return aval;
        }
    }
    return 89;
}
/**
 * @brief  Edge spin.
 */
LOCAL void edge_spin (void)
{
    g_spin_dir = g_dir;
    motion_wheels(MOT_OWNER_MISSION, -g_dir * AV_EDGE_MMS, g_dir * AV_EDGE_MMS);
    motion_set_owner(MOT_OWNER_MISSION);
}
/** old box route with a fixed sideways move: only if the encoder
   heading is not usable (the edge spin needs it) */
LOCAL void go_round_fixed (uint32_t tval, const char * p_why)
{
    g_need_cm = AV_UNKNOWN_NEED_CM;
    g_aim_d10 = AV_TURN_DEG * 10;
    tm_printf((uint8_t *)"MISSION %s: fixed route on the %s, %d cm sideways\n",
              p_why, (g_dir > 0) ? "LEFT" : "RIGHT", g_need_cm);
    ranger_fast(FALSE);
    turn_to(g_h_start + (g_dir * AV_TURN_DEG * 10));
    go_av(AV_OUT, tval);
}
/* side chosen (dir): find that edge by turning the car */
/** sideways position of an edge whose beam-clear angle was seen_deg */
LOCAL int32_t edge_lat_cm (int32_t seen_deg)
{
    int32_t dax   = (int32_t)g_obs_d + AV_US_FWD_CM; /* front, from the axle */
    int32_t e_deg = seen_deg - AV_CONE_DEG; /* true edge: minus the cone */
    if (e_deg < 0)
    {
        e_deg = 0;
    }
    return (dax * scan_profile_isin_x1000(e_deg)) / icos_x1000(e_deg);
}

/** 12 Nov evening: BOTH edges are measured (the car turns to each in
   turn), then it goes round on the side needing the smaller sideways
   move: need = edge on that side + half the car + margin.  Width =
   left edge + right edge. */
LOCAL void start_edge (uint32_t tval)
{
    g_obs_d = ((g_prof.found) && (g_prof.dmin_cm <= AV_CLEAR_CM))
                  ? (int32_t)g_prof.dmin_cm
                  : g_guard_cm;
    if (g_obs_d < US_MIN_CM)
    {
        g_obs_d = US_MIN_CM;
    }
    g_obs.dir = g_dir;
    if (!motion_heading_ok())
    {
        obs_note(OBS_SWEPT, -1);
        go_round_fixed(tval, "encoder heading not usable");
        return;
    }
    tm_printf(
        (uint8_t *)"MISSION obstacle front at %d cm: measuring its edges, %s "
                   "side first\n",
        g_obs_d, (g_dir > 0) ? "LEFT" : "RIGHT");
    g_edge_phase   = 1;
    g_edge_seen[0] = g_edge_seen[1] = -1;
    ranger_aim(0);
    ranger_fast(TRUE);
    edge_spin();
    go_av(AV_EDGE, tval);
}

/** both sides looked at: pick the shorter way round and aim past it */
LOCAL void edge_decide (uint32_t tval)
{
    int32_t dax        = (int32_t)g_obs_d + AV_US_FWD_CM;
    int32_t ev_item[2] = {0, 0};
    int32_t need[2]    = {0, 0};
    int32_t kdx;
    int32_t aim;
    int32_t ix_val;

    motion_wheels(MOT_OWNER_MISSION, 0, 0);
    ranger_fast(FALSE);
    for (kdx = 0; kdx < 2; kdx++)
    {
        if (g_edge_seen[kdx] < 0)
        {
            continue;
        }
        ev_item[kdx] = edge_lat_cm(g_edge_seen[kdx]);
        need[kdx]    = ev_item[kdx] + AV_HALF_CAR_CM + AV_MARGIN_CM;
    }
    if ((g_edge_seen[0] < 0) && (g_edge_seen[1] < 0))
    {
        fault("no way round: no edge found on either side", FAULT_AVOID);
        return;
    }
    if ((g_edge_seen[0] >= 0) && (g_edge_seen[1] >= 0))
    {
        g_dir          = (need[0] < need[1])   ? 1
                         : (need[1] < need[0]) ? -1
                                               : (AV_PREFER_LEFT ? 1 : -1);
        g_obs.width_cm = (int32_t)CLAMP(ev_item[0] + ev_item[1], 0, 254);
        g_obs.bearing_deg =
            ((g_edge_seen[0] - AV_CONE_DEG) - (g_edge_seen[1] - AV_CONE_DEG)) /
            2;
        tm_printf((uint8_t *)"OBSTACLE edges: LEFT %d cm, RIGHT %d cm -> width "
                             "~%d cm, "
                             "move L %d / R %d cm -> going %s (shorter)\n",
                  (int32_t)ev_item[0], (int32_t)ev_item[1],
                  (int32_t)(ev_item[0] + ev_item[1]), (int32_t)need[0],
                  (int32_t)need[1], (g_dir > 0) ? "LEFT" : "RIGHT");
    }
    else
    {
        g_dir          = (g_edge_seen[0] >= 0) ? 1 : -1;
        g_obs.width_cm = 0;
        tm_printf((uint8_t *)"OBSTACLE only the %s edge found (%d cm): going "
                             "that way\n",
                  (g_dir > 0) ? "LEFT" : "RIGHT",
                  (int32_t)ev_item[(g_dir > 0) ? 0 : 1]);
    }
    ix_val      = (g_dir > 0) ? 0 : 1;
    g_e_side_cm = ev_item[ix_val];
    g_need_cm   = (int32_t)need[ix_val];
    if (g_need_cm > AV_NEED_MAX_CM)
    {
        tm_printf((uint8_t *)"MISSION sideways move %d cm capped to %d cm\n",
                  g_need_cm, AV_NEED_MAX_CM);
        g_need_cm = AV_NEED_MAX_CM;
    }
    aim = atan_deg(g_need_cm, dax);
    if (aim > AV_AIM_MAX_DEG)
    {
        aim = AV_AIM_MAX_DEG;
    }
    g_aim_d10           = (int32_t)aim * 10;
    g_obs.need_left_cm  = (int32_t)CLAMP(need[0], 0, 254);
    g_obs.need_right_cm = (int32_t)CLAMP(need[1], 0, 254);
    g_obs.dir           = g_dir;
    obs_note(OBS_SWEPT, -1);
    tm_printf((uint8_t *)"MISSION aim %d deg %s, car centre %d cm out\n", aim,
              (g_dir > 0) ? "LEFT" : "RIGHT", g_need_cm);
    turn_to(g_h_start + (g_dir * g_aim_d10));
    go_av(AV_AIM, tval);
}

/** this side done (found or not): the other side, or decide */
LOCAL void edge_next (uint32_t tval)
{
    if (1 == g_edge_phase)
    {
        g_edge_phase = 2;
        g_dir        = -g_dir;
        edge_spin();
        g_t_start = tval;
        g_num     = 0;
        tm_printf((uint8_t *)"MISSION now the %s edge\n",
                  (g_dir > 0) ? "LEFT" : "RIGHT");
    }
    else
    {
        edge_decide(tval);
    }
}

/** can't measure the obstacle's edges from the sweep: pick the side whose
   end of the sweep had more room, then find the edge by turning */
LOCAL void go_round_guess (uint32_t tval, const char * p_why)
{
    int32_t l_end = g_sweep_cm[0];
    int32_t r_end = g_sweep_cm[AV_SWEEP_PTS - 1];

    g_dir = (l_end == r_end) ? (AV_PREFER_LEFT ? 1 : -1)
                             : ((l_end > r_end) ? 1 : -1);
    tm_printf((uint8_t *)"MISSION %s: going round on the %s\n", p_why,
              (g_dir > 0) ? "LEFT" : "RIGHT");
    start_edge(tval);
}

/** ---------------- avoidance step ---------------- */
LOCAL void avoid_step (uint32_t tval)
{
    int32_t  cm_val;
    int32_t  idx;
    uint32_t dt_val = tval - g_t_start;

    switch (g_av_state)
    {
        case AV_CONFIRM: /* let the car stop, then sweep */
            if (dt_val < AV_CONFIRM_MS)
            {
                break;
            }
            if (g_guard_cm < AV_BACKUP_BELOW_CM)
            {
                /* too close to see it in the sweep or to turn out:
                   reverse straight back first */
                tm_printf(
                    (uint8_t *)"MISSION stopped at %d cm: backing up to ~%d "
                               "cm first\n",
                    g_guard_cm, AV_BACKUP_TO_CM);
                go_av(AV_BACKUP, tval);
                g_leg_odo0 = motion_odometry_mm();
                g_leg_mm   = (int32_t)(AV_BACKUP_TO_CM - g_guard_cm) * 10;
                motion_wheels(MOT_OWNER_MISSION, -AV_BACKUP_MMS,
                              -AV_BACKUP_MMS);
                motion_set_owner(MOT_OWNER_MISSION);
                break;
            }
            tm_printf((uint8_t *)"MISSION sweep starts %d ms after the stop\n",
                      (int32_t)(tval - g_t_stop));
            start_sweep(tval);
        break;

        case AV_BACKUP:
        { /* n = 0 reversing, 1 settling */
            int32_t back =
                g_leg_odo0 - motion_odometry_mm(); /* mm reversed so far */
            if (0 == g_num)
            {
                if ((back >= g_leg_mm) || (dt_val >= AV_BACKUP_MAX_MS))
                {
                    motion_wheels(MOT_OWNER_MISSION, 0, 0);
                    g_num    = 1;
                    g_t_mark = tval;
                }
                break;
            }
            if ((tval - g_t_mark) < AV_BACKUP_SETTLE_MS)
            {
                break;
            }
            if (back < 0)
            {
                back = 0;
            }
            g_guard_cm +=
                (int32_t)(back / 10); /* it is that much farther now */
            if (g_guard_cm > AV_CLEAR_CM)
            {
                g_guard_cm = AV_CLEAR_CM;
            }
            tm_printf(
                (uint8_t *)"MISSION backed up %d mm, obstacle now ~%d cm\n",
                (int32_t)back, g_guard_cm);
            start_sweep(tval);
            break;
        }

        case AV_SCAN:
        { /* fan sweep, AV_PINGS_PER_PT readings per angle */
            /* 12 Nov: one reading per angle let a single missed echo (common
               at 3.3 V) hide part of the obstacle, so it looked narrower and
               off to one side.  Keep the closest of a few readings. */
            if ((ping(g_sweep_aim, &cm_val)) &&
                (g_pt_n < (int32_t)(sizeof g_pt_buf / sizeof g_pt_buf[0])))
            {
                g_pt_buf[g_pt_n++] = cm_val;
            }
            if ((g_pt_n < AV_PINGS_PER_PT) &&
                ((tval - g_t_pt) < AV_PT_TIMEOUT_MS))
            {
                break;
            }
            /* 12 Nov evening: the MEDIAN, not the closest: one ghost echo
               no longer decides the point */
            g_pt_cm                        = median_of(g_pt_buf, g_pt_n);
            g_sweep_deg[SLOT(g_sweep_aim)] = g_sweep_aim;
            g_sweep_cm[SLOT(g_sweep_aim)] =
                g_pt_cm; /* RANGE_FAR_CM: nothing here */
            if (++g_sweep_k < AV_SWEEP_PTS)
            {
                sweep_aim_next(tval);
                break;
            }

            /* sweep done: print it, then work out the obstacle */
            ranger_fast(FALSE);
            ranger_aim(0);
            tm_printf((uint8_t *)"SCAN took %d ms (%d ms since the stop)\n",
                      (int32_t)dt_val, (int32_t)(tval - g_t_stop));
            tm_printf((uint8_t *)"SCAN deg:cm");
            for (idx = 0; idx < AV_SWEEP_PTS; idx++)
            {
                tm_printf((uint8_t *)" %d:%d", (int32_t)g_sweep_deg[idx],
                          (int32_t)g_sweep_cm[idx]);
            }
            tm_printf((uint8_t *)"\n");
            /* sanity 1: the confirm step saw it straight ahead.  If the
               sweep says straight ahead is clear, it has moved away. */
            if (((g_sweep_cm[SLOT(0)] > AV_CLEAR_CM) &&
                 (g_sweep_cm[SLOT(AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM)) &&
                (g_sweep_cm[SLOT(-AV_SWEEP_STEP_DEG)] > AV_CLEAR_CM))
            {
                /* 12 Nov: at 3.3 V the sensor misses echoes, so one
                   sweep saying "clear" was often wrong: the car drove
                   on, the guard stopped it again, it swept again ...
                   (the ~10 s wait in the videos).  Look straight ahead
                   with normal pings before believing it. */
                tm_printf((uint8_t *)"MISSION sweep says ahead is clear: "
                                     "re-checking\n");
                go_av(AV_RECHECK, tval);
                break;
            }
            /* sanity 2: every angle reads the same distance -> the servo is
               most likely NOT turning (unplugged, wrong column, no power).
               Only meaningful for a wide sweep: across +-25 deg a flat box
               really does read almost the same everywhere. */
            /* 12 Nov: the guard saw the obstacle straight ahead at guard_cm, so
                       it IS there: a missed echo at 0 deg must not split it or
               move its centre off to one side */
            if (g_sweep_cm[SLOT(0)] > g_guard_cm)
            {
                g_sweep_cm[SLOT(0)] = g_guard_cm;
            }
            scan_profile_compute(&g_scan_tune, g_sweep_deg, g_sweep_cm,
                                 AV_SWEEP_PTS, &g_prof);
            tm_printf(
                (uint8_t *)"SCAN used (0 deg floored to the guard's %d cm):",
                g_guard_cm);
            for (idx = 0; idx < AV_SWEEP_PTS; idx++)
            {
                tm_printf((uint8_t *)" %d:%d", (int32_t)g_sweep_deg[idx],
                          (int32_t)g_sweep_cm[idx]);
            }
            tm_printf((uint8_t *)"\n");
#if AV_SWEEP_MAX_DEG >= 45
            {
                int32_t lo = 255;
                int32_t hi = 0;
                for (i = 0; i < AV_SWEEP_PTS; i++)
                {
                    if (sweep_cm[i] < lo)
                    {
                        lo = sweep_cm[i];
                    }
                    if (sweep_cm[i] > hi)
                    {
                        hi = sweep_cm[i];
                    }
                }
                if (hi - lo <= 4)
                {
                    tm_printf(
                        (uint8_t *)"MISSION all %d angles read %d..%d cm: is "
                                   "the SERVO turning? Check its plug (GP13 "
                                   "column, brown wire to -)\n",
                        AV_SWEEP_PTS, lo, hi);
                    fault("servo not turning (sweep is flat)", FAULT_AVOID);
                    break;
                }
            }
#endif
            if (!g_prof.found)
            {
                tm_printf((uint8_t *)"MISSION nothing within %d cm after all: "
                                     "re-checking\n",
                          AV_FAR_CM);
                go_av(AV_RECHECK, tval);
                break;
            }
            tm_printf(
                (uint8_t *)"OBSTACLE %d cm at %d deg, width %d cm, edges L %d "
                           "/ R %d cm (%s/%s)"
                           " -> move L %d / R %d cm\n",
                (int32_t)g_prof.dmin_cm, (int32_t)g_prof.bearing_deg,
                (int32_t)g_prof.width_cm, (int32_t)g_prof.left_cm,
                (int32_t)g_prof.right_cm,
                (g_prof.open_left) ? "open" : "closed",
                (g_prof.open_right) ? "open" : "closed",
                (int32_t)g_prof.need_left_cm, (int32_t)g_prof.need_right_cm);
            g_obs.dist_cm       = g_prof.dmin_cm;
            g_obs.bearing_deg   = g_prof.bearing_deg;
            g_obs.width_cm      = g_prof.width_cm;
            g_obs.need_left_cm  = g_prof.need_left_cm;
            g_obs.need_right_cm = g_prof.need_right_cm;
            g_obs.dir           = g_prof.dir;
            if (0 == g_prof.dir)
            {
                /* both edges are outside the narrow sweep: we can't see past
                   it either side.  Truly blocked only if both ends of the
                   sweep are close; otherwise go round on the roomier side
                   with a fixed sideways move. */
                int32_t l_end = g_sweep_cm[0];
                int32_t r_end = g_sweep_cm[AV_SWEEP_PTS - 1];
                if ((((l_end <= AV_CLEAR_CM) && (r_end <= AV_CLEAR_CM)) &&
                     (l_end < 15)) &&
                    (r_end < 15))
                {
                    /* 12 Nov: re-sweep a few times, then go round
                       anyway (the old code waited for the obstacle
                       to move, which a box never does) */
                    if (++g_blocked_n < AV_BLOCKED_SWEEPS)
                    {
                        obs_note(OBS_BLOCKED, -1);
                        go_av(AV_BLOCKED, tval);
                        break;
                    }
                    go_round_guess(tval, "still blocked after re-sweeps");
                    break;
                }
                g_dir = (l_end == r_end) ? (AV_PREFER_LEFT ? 1 : -1)
                                         : ((l_end > r_end) ? 1 : -1);
                tm_printf(
                    (uint8_t *)"MISSION obstacle wider than the %d deg view: "
                               "side %s (more room at that end)\n",
                    AV_SWEEP_MAX_DEG, (g_dir > 0) ? "LEFT" : "RIGHT");
            }
            else
            {
                g_dir = g_prof.dir;
            }
            /* the sweep only CHOOSES the side; the edge itself is found by
               turning the car (start_edge), so its size is not guessed */
            start_edge(tval);
            break;
        }

        case AV_EDGE:
        { /* slow spin toward dir, pinging ahead */
            int32_t rel = (motion_heading_d10() - g_h_start) *
                          g_dir; /* 0.1 deg out on this side */
            if (!motion_heading_ok())
            {
                motion_wheels(MOT_OWNER_MISSION, 0, 0);
                go_round_fixed(tval,
                               "encoder heading lost while finding the edge");
                break;
            }
            if (ping(0, &cm_val))
            {
                int32_t aval = (int32_t)(rel / 10);
                int32_t expect;
                if (rel < 0)
                {
                    g_num =
                        0; /* second side: still swinging back across the front
                            */
                }
                else
                {
                    aval = CLAMP(aval, 0, 80);
                    /* a flat front reads farther as the beam turns: d / cos */
                    expect = (g_obs_d * 1000) / icos_x1000(aval);
                    if ((cm_val >= RANGE_FAR_CM) ||
                        (cm_val > (expect + AV_BAND_CM)))
                    {
                        if (0 == g_num)
                        {
                            g_best = (int32_t)
                                rel; /* first clear reading of this run */
                        }
                        if (++g_num >= AV_EDGE_CLEAR_PINGS)
                        {
                            /* edge found: 'best' = the car's angle when the
                               beam first got past it */
                            int32_t seen_deg                 = g_best / 10;
                            g_edge_seen[(g_dir > 0) ? 0 : 1] = seen_deg;
                            tm_printf(
                                (uint8_t *)"MISSION %s edge: beam clear at %d "
                                           "deg (%d cm) = %d cm sideways\n",
                                (g_dir > 0) ? "LEFT" : "RIGHT", seen_deg,
                                cm_val, (int32_t)edge_lat_cm(seen_deg));
                            edge_next(tval);
                            break;
                        }
                    }
                    else
                    {
                        g_num = 0; /* still on the obstacle */
                    }
                }
            }
            if ((rel > (AV_EDGE_MAX_DEG * 10)) || (dt_val > AV_EDGE_MAX_MS))
            {
                tm_printf((uint8_t *)"MISSION no %s edge within %d deg\n",
                          (g_dir > 0) ? "LEFT" : "RIGHT", AV_EDGE_MAX_DEG);
                edge_next(tval);
            }
            break;
        }

        case AV_AIM: /* point past the edge, then drive out */
            if (turn_done(dt_val, g_aim_d10))
            {
                /* drive until level with the obstacle's front: forward
                   distance dax along a line 'aim' out */
                int32_t aim = (int32_t)(g_aim_d10 / 10);
                int32_t mm_val =
                    (((int32_t)g_obs_d + AV_US_FWD_CM) * 10 * 1000) /
                    icos_x1000(aim);
                tm_printf(
                    (uint8_t *)"MISSION aimed %d deg (heading %d), out %d mm\n",
                    aim, (int32_t)(motion_heading_d10() - g_h_start),
                    (int32_t)mm_val);
                leg(mm_val, AV_DRIVE_MMS);
                go_av(AV_WIDEN, tval);
            }
        break;

        case AV_BLOCKED: /* no room either side: look again */
            if (dt_val >= AV_BLOCKED_RETRY_MS)
            {
                tm_printf(
                    (uint8_t *)"MISSION blocked (%d of %d): sweeping again\n",
                    g_blocked_n, AV_BLOCKED_SWEEPS);
                start_sweep(tval);
            }
        break;

        case AV_RECHECK: /* sweep said clear: make sure */
            /* AV_RECHECK_PINGS normal pings straight ahead.  Any close
               one = it is still there: go round.  All clear = gone. */
            if (ping(0, &cm_val))
            {
                g_num++;
                if (cm_val <= AV_CLEAR_CM)
                {
                    tm_printf(
                        (uint8_t *)"MISSION re-check: still there at %d cm\n",
                        cm_val);
                    go_round_guess(tval, "sweep missed it");
                    break;
                }
                if (g_num >= AV_RECHECK_PINGS)
                {
                    tm_printf((uint8_t *)"MISSION re-check: clear %d times, "
                                         "obstacle gone\n",
                              g_num);
                    obs_note(OBS_GONE, cm_val);
                    if (line_see_bits() != 0)
                    {
                        begin_follow(g_line_side, FALSE);
                        break;
                    }
                    /* 12 Nov 18:48 run: no sensor on the line here, and
                       the line follower's recovery spin has no limit: it
                       turned the car ~120 deg and picked the line up
                       facing BACKWARDS.  Instead face the old heading and
                       sweep at most AV_FIND_DEG each way (toward the side
                       the line was on first). */
                    g_dir = (g_line_side > 0)   ? -1
                            : (g_line_side < 0) ? 1
                                                : (AV_PREFER_LEFT ? 1 : -1);
                    tm_printf(
                        (uint8_t *)"MISSION no sensor on the line: bounded "
                                   "search for it\n");
                    turn_to(g_h_start);
                    go_av(AV_REJOIN, tval);
                    break;
                }
            }
            if (dt_val > AV_RECHECK_MAX_MS)
            {
                go_round_guess(tval, "no clear answer ahead");
            }
        break;

        case AV_OUT: /* turn AV_TURN_DEG away */
            if (turn_done(dt_val, AV_TURN_DEG * 10))
            {
                /* sideways move need_cm at AV_TURN_DEG: leg = need / sin */
                int32_t mm_val = ((int32_t)g_need_cm * 10 * 1000) /
                                 scan_profile_isin_x1000(AV_TURN_DEG);
                tm_printf(
                    (uint8_t *)"MISSION out %d deg (heading %d), widen %d mm\n",
                    AV_TURN_DEG, (int32_t)(motion_heading_d10() - g_h_start),
                    (int32_t)mm_val);
                leg(mm_val, AV_DRIVE_MMS);
                go_av(AV_WIDEN, tval);
            }
        break;

        case AV_WIDEN: /* move out sideways */
            leg_hold();
            if ((ping(0, &cm_val)) && (cm_val <= AV_SEEK_STOP_CM))
            {
                fault("something in the way while going round", FAULT_AVOID);
                break;
            }
            if (leg_done_mm() >= g_leg_mm)
            {
                turn_to(g_h_start);
                go_av(AV_ALIGN, tval);
            }
        break;

        case AV_ALIGN: /* parallel to the line again */
            if (turn_done(dt_val, g_aim_d10))
            {
                /* the axle is now about level with the obstacle's front.
                   Drive on until AV_PASS_CLEAR_CM past the farthest point
                   of it the side-looking sensor sees (at least
                   AV_PASS_MIN_DEPTH_CM deep), so its length is measured,
                   not assumed. */
                ranger_aim(-g_dir * AV_SIDE_DEG); /* look toward it */
                leg((int32_t)AV_PASS_MAX_CM * 10, AV_DRIVE_MMS);
                g_pass_odo0 = motion_odometry_mm();
                g_pass_end =
                    g_pass_odo0 +
                    ((int32_t)(AV_PASS_MIN_DEPTH_CM + AV_PASS_CLEAR_CM) * 10);
                g_peek_extra = 0;
                g_far_seen   = g_pass_odo0;
                tm_printf(
                    (uint8_t *)"MISSION parallel (heading %d), passing it\n",
                    (int32_t)(motion_heading_d10() - g_h_start));
                go_av(AV_PASS, tval);
            }
        break;

        case AV_PASS: /* drive past it */
            leg_hold();
            if ((ping(-g_dir * AV_SIDE_DEG, &cm_val)) &&
                (cm_val < AV_PASS_SEE_CM))
            {
                /* that point is (sensor ahead of the axle) + cm x cos(side
                 * angle) ahead */
                int32_t end =
                    motion_odometry_mm() +
                    (((int32_t)AV_US_FWD_CM * 10) +
                     (((int32_t)cm_val * 10 * icos_x1000(AV_SIDE_DEG)) /
                      1000)) +
                    ((int32_t)AV_PASS_CLEAR_CM * 10);
                if (end > g_pass_end)
                {
                    g_pass_end = end;
                }
                if ((end - ((int32_t)AV_PASS_CLEAR_CM * 10)) > g_far_seen)
                {
                    g_far_seen = end - ((int32_t)AV_PASS_CLEAR_CM * 10);
                }
                gb_seen = TRUE;
            }
            if ((motion_odometry_mm() >= g_pass_end) ||
                (leg_done_mm() >= g_leg_mm))
            {
                if (motion_odometry_mm() < g_pass_end)
                {
                    tm_printf((uint8_t *)"MISSION still seeing it after %d cm: "
                                         "turning back anyway\n",
                              AV_PASS_MAX_CM);
                }
                tm_printf((uint8_t *)"MISSION passed (%d mm, %s)\n",
                          (int32_t)leg_done_mm(),
                          gb_seen ? "length measured"
                                  : "never saw its side: minimum length");
                obs_note(OBS_PASSED, -1);
                ranger_aim(0);
                turn_to(g_h_start - (g_dir * AV_TURN_DEG * 10));
                go_av(AV_BACK, tval);
            }
        break;

        case AV_BACK: /* turn toward the line, then look */
            if (turn_done(dt_val, AV_TURN_DEG * 10))
            {
                motion_wheels(MOT_OWNER_MISSION, 0, 0);
                ranger_aim(0);
                g_pt_n = 0;
                go_av(AV_PEEK, tval);
            }
        break;

        case AV_PEEK:
        { /* is the way back to the line clear? */
            int32_t med;
            if (dt_val < 150U)
            {
                break; /* stop rocking first */
            }
            if ((ping(0, &cm_val)) && (g_pt_n < AV_PEEK_PINGS))
            {
                g_pt_buf[g_pt_n++] = cm_val;
            }
            if ((g_pt_n < AV_PEEK_PINGS) && (dt_val < AV_PEEK_MAX_MS))
            {
                break;
            }
            med = median_of(g_pt_buf, g_pt_n);
            if (med > AV_PEEK_CLEAR_CM)
            {
                int32_t mm_val = (((int32_t)g_need_cm * 10 * 1000) /
                                  scan_profile_isin_x1000(AV_TURN_DEG)) +
                                 (AV_SEEK_EXTRA_CM * 10);
                /* length: from the front (PASS start, axle level with it) to
                   the farthest side point seen.  Rough: +- a few cm. */
                if (g_far_seen > g_pass_odo0)
                {
                    tm_printf(
                        (uint8_t *)"OBSTACLE length ~%d cm (width ~%d cm)\n",
                        (int32_t)((g_far_seen - g_pass_odo0) / 10),
                        g_obs.width_cm);
                }
                else
                {
                    tm_printf((uint8_t *)"OBSTACLE length: short, its side was "
                                         "never seen (under ~%d cm)\n",
                              AV_PASS_MIN_DEPTH_CM + AV_PASS_CLEAR_CM);
                }
                tm_printf((uint8_t *)"MISSION way back clear (%d cm ahead, %d "
                                     "readings): heading for the line\n",
                          med, g_pt_n);
                leg(mm_val, AV_SEEK_MMS);
                go_av(AV_SEEK, tval);
                break;
            }
            /* still beside us: a long obstacle.  The 45 deg beam meets its
               side about (gap to it) ahead of the sensor. */
            {
                int32_t pt_val = motion_odometry_mm() +
                                 (((int32_t)AV_US_FWD_CM +
                                   ((int32_t)g_need_cm - g_e_side_cm)) *
                                  10);
                if (pt_val > g_far_seen)
                {
                    g_far_seen = pt_val;
                }
            }
            g_peek_extra += (int32_t)AV_PEEK_STEP_CM * 10;
            if (g_peek_extra > ((int32_t)AV_PEEK_MAX_CM * 10))
            {
                fault("obstacle too long: still beside it after the extra "
                      "distance",
                      FAULT_AVOID);
                break;
            }
            tm_printf((uint8_t *)"MISSION still beside it (%d cm ahead): %d cm "
                                 "further on, then look again\n",
                      med, AV_PEEK_STEP_CM);
            turn_to(g_h_start);
            go_av(AV_PEEK_ALIGN, tval);
            break;
        }

        case AV_PEEK_ALIGN: /* parallel again */
            if (turn_done(dt_val, AV_TURN_DEG * 10))
            {
                leg((int32_t)AV_PEEK_STEP_CM * 10, AV_DRIVE_MMS);
                go_av(AV_PEEK_FWD, tval);
            }
        break;

        case AV_PEEK_FWD: /* a bit further along it */
            leg_hold();
            if (leg_done_mm() >= g_leg_mm)
            {
                turn_to(g_h_start - (g_dir * AV_TURN_DEG * 10));
                go_av(AV_BACK, tval);
            }
        break;

        case AV_SEEK: /* drive until a sensor sees black */
            leg_hold();
            if (line_see_bits() != 0)
            {
                tm_printf((uint8_t *)"MISSION line found after %d mm\n",
                          (int32_t)leg_done_mm());
                leg(AV_SENSOR_FWD_CM * 10, AV_SEEK_MMS);
                go_av(AV_ADVANCE, tval);
                break;
            }
            if ((ping(0, &cm_val)) && (cm_val <= AV_SEEK_STOP_CM))
            {
                fault("blocked on the way back to the line", FAULT_AVOID);
            }
            else if (leg_done_mm() >= g_leg_mm)
            {
                fault("line not found after the obstacle", FAULT_AVOID);
            }
            else
            {
                /* No action is required in this case. */
            }
        break;

        case AV_ADVANCE: /* wheels onto the line */
            leg_hold();
            if (leg_done_mm() >= g_leg_mm)
            {
                turn_to(g_h_start); /* face along the line again */
                go_av(AV_REJOIN, tval);
            }
        break;

        case AV_REJOIN:
            if (turn_done(dt_val, AV_TURN_DEG * 10))
            {
                tm_printf(
                    (uint8_t *)"MISSION back at the line, heading %d (0 = as "
                               "before the obstacle), sensors %d\n",
                    (int32_t)(motion_heading_d10() - g_h_start),
                    (int32_t)line_see_bits());
                if (line_see_bits() != 0)
                {
                    rejoined();
                    break;
                }
                /* facing the right way but no sensor on the line: sweep
                   slowly back toward the side we came from, then the
                   other way, and take the line as soon as a sensor sees it
                   (instead of letting the line follower spin round) */
                turn_to_slow(g_h_start + (g_dir * AV_FIND_DEG * 10));
                go_av(AV_FIND1, tval);
            }
        break;

        case AV_FIND1:
        case AV_FIND2:
            if (line_see_bits() != 0)
            {
                motion_wheels(MOT_OWNER_MISSION, 0, 0);
                tm_printf((uint8_t *)"MISSION line found while sweeping "
                                     "(heading %d)\n",
                          (int32_t)(motion_heading_d10() - g_h_start));
                rejoined();
                break;
            }
            if (turn_done(dt_val, ((AV_FIND1 == g_av_state) ? 1 : 2) *
                                      AV_FIND_DEG * 10 * AV_SPIN_MMS /
                                      AV_FIND_MMS))
            {
                if (AV_FIND1 == g_av_state)
                {
                    turn_to_slow(g_h_start - (g_dir * AV_FIND_DEG * 10));
                    go_av(AV_FIND2, tval);
                }
                else
                {
                    fault("line not found after the obstacle (swept both ways)",
                          FAULT_AVOID);
                }
            }
        break;
        default:
        {
            /* No other values are expected. */
            break;
        }
    }
}

/* ---------------- barcode turns ---------------- */
#define TURN_SPEED_MMS  80
#define TURN_ADVANCE_MS 600  /* after the crossing: axle over the junction */
#define TURN_MIN_LR_MS  300  /* spin at least this long before looking */
#define TURN_MIN_U_MS   1500 /* ~100 deg at the slower turn speed */
#define TURN_MAX_MS     7000 /* a U-turn at 70 mm/s takes ~3 s */
/* a turn barcode waits this long for a junction (12 Nov: was 4000, ~120 mm
   at 30 mm/s; the old slow timer made it ~170 s) */
#define TURN_PENDING_MS 10000
/* INSIDE layout: both sensors are always black on the line, so a junction
   can't be seen.  The turn starts this far after the barcode instead (CAL:
   measure barcode -> junction on your course). */
#define TURN_AFTER_MM 150

/* remote REVERSE: back up this far (odometry), slowly, then stop */
#define REVERSE_MM     200
#define REVERSE_MMS    90
#define REVERSE_MAX_MS 4000

enum
{
    TN_CROSS,
    TN_ADVANCE,
    TN_SPIN,
    TN_SETTLE
};
/* slower than the avoidance spin: easier to stop on the line */
#define TURN_SPIN_MMS  70
#define TURN_SETTLE_MS 250 /** brake on the line, then look where it is */

LOCAL void turn_spin (int32_t dval)
{
    motion_wheels(MOT_OWNER_MISSION, -dval * TURN_SPIN_MMS,
                  dval * TURN_SPIN_MMS);
    motion_set_owner(MOT_OWNER_MISSION);
}
LOCAL uint32_t g_pend_act; /* BC_ACT_LEFT/RIGHT waiting for a junction */
LOCAL uint32_t g_pend_t;
LOCAL int32_t  g_pend_odo;
LOCAL BOOL     gb_spin_off; /* sensors left the old line during the spin */
LOCAL int32_t  g_rev_odo0;
LOCAL uint32_t g_rev_t0;
LOCAL uint32_t g_tn_val;
LOCAL uint32_t g_tn_act;
LOCAL uint32_t g_tn_t0;

/**
 * @brief  Begin turn.
 */
LOCAL void begin_turn (uint32_t act, uint32_t tval, BOOL b_at_junction)
{
    climb_reset();
    line_follow(FALSE, 0, FALSE);
    ranger_guard(FALSE);
    g_tn_act    = act;
    g_tn_t0     = tval;
    g_state     = MS_TURN;
    gb_spin_off = FALSE;
    if (b_at_junction && (!line_inside()))
    {
        g_tn_val = TN_CROSS;
        forward(TURN_SPEED_MMS);
        gp_text = "TURN crossing";
    }
    else if (b_at_junction)
    {
        g_tn_val = TN_ADVANCE;
        forward(TURN_SPEED_MMS);
        gp_text = "TURN advance";
    }
    else
    {
        g_tn_val = TN_SPIN;
        turn_spin(1);
        gp_text = "TURN spinning";
    } /* U-turn: left */
    tm_printf((uint8_t *)"MISSION barcode turn %s\n",
              (BC_ACT_LEFT == act)    ? "LEFT"
              : (BC_ACT_RIGHT == act) ? "RIGHT"
                                      : "U-TURN");
}

/**
 * @brief  Turn step.
 */
LOCAL void turn_step (uint32_t tval)
{
    uint32_t see    = line_see_bits();
    uint32_t dt_val = tval - g_tn_t0;
    int32_t  dval =
        (BC_ACT_RIGHT == g_tn_act) ? -1 : 1; /* spin direction, + = left */

    if (dt_val > TURN_MAX_MS)
    {
        fault("barcode turn: line not found", FAULT_LINE);
        return;
    }
    switch (g_tn_val)
    {
        case TN_CROSS: /* drive off the black crossing */
            if (see != 3)
            {
                g_tn_val = TN_ADVANCE;
                g_tn_t0  = tval;
                gp_text  = "TURN advance";
            }
        break;
        case TN_ADVANCE:
            if (dt_val >= TURN_ADVANCE_MS)
            {
                g_tn_val = TN_SPIN;
                g_tn_t0  = tval;
                turn_spin(dval);
                gp_text = "TURN spinning";
            }
        break;
        case TN_SPIN:
            /* the sensors must leave the old line first (INSIDE layout starts
               the spin sitting on it), then the first black is the new line */
            if (0 == see)
            {
                gb_spin_off = TRUE;
            }
            if (((dt_val >= ((BC_ACT_UTURN == g_tn_act) ? TURN_MIN_U_MS
                                                        : TURN_MIN_LR_MS)) &&
                 (see != 0)) &&
                gb_spin_off)
            {
                /* found it: STOP turning first.  Handing over while still
                   spinning made the car slide past the line before the line
                   task had a single sample, then 'recover' the wrong way
                   round and spin a full circle, again and again. */
                motion_wheels(MOT_OWNER_MISSION, 0, 0);
                g_tn_val = TN_SETTLE;
                g_tn_t0  = tval;
                gp_text  = "TURN settle";
            }
        break;
        case TN_SETTLE:
            if (dt_val >= TURN_SETTLE_MS)
            {
                /* now look where the line really is.  If the car slid past
                   it, the line is on the side we were turning AWAY from:
                   spinning left (d > 0) leaves it to the right (+1). */
                int32_t side = (1 == see)   ? -1
                               : (2 == see) ? 1
                               : (3 == see) ? 0
                                            : ((dval > 0) ? 1 : -1);
                tm_printf((uint8_t *)"MISSION turn done, line %s\n",
                          (3 == see)   ? "under both sensors"
                          : (0 == see) ? "just missed: turning back to it"
                          : (1 == see) ? "under the left sensor"
                                       : "under the right sensor");
                /* STRADDLE layout: both white can simply mean centred */
                begin_follow(side,
                             (side != 0) && ((see != 0) || (line_inside())));
            }
        break;
        default:
        {
            /* No other values are expected. */
            break;
        }
    }
}

/**
 * @brief  On barcode.
 */
LOCAL void on_barcode (uint32_t tval)
{
    barcode_info_t bval;

    barcode_get(&bval);
    switch (bval.action)
    {
        case BC_ACT_END:
            halt_all();
            g_state = MS_DONE;
            gp_text = "COMPLETE";
            tm_printf(
                (uint8_t *)"MISSION end-of-course barcode \"%s\": stopped\n",
                bval.text);
        break;
        case BC_ACT_UTURN:
            begin_turn(BC_ACT_UTURN, tval, FALSE);
        break;
        case BC_ACT_LEFT:
        case BC_ACT_RIGHT:
            g_pend_act = bval.action;
            g_pend_t   = tval;
            g_pend_odo = motion_odometry_mm() + TURN_AFTER_MM;
            if (line_inside())
            {
                tm_printf((uint8_t *)"MISSION turn %s in %d mm\n",
                          (BC_ACT_LEFT == bval.action) ? "LEFT" : "RIGHT",
                          TURN_AFTER_MM);
            }
            else
            {
                tm_printf((uint8_t *)"MISSION turn %s at the next junction\n",
                          (BC_ACT_LEFT == bval.action) ? "LEFT" : "RIGHT");
            }
        break;
        default:
            break; /* reported by the comms task */
    }
}

/** ---------------- buttons ---------------- */
LOCAL void button_pin (uint32_t pin)
{
    gpio_set_pin(pin, GPIO_MODE_IN);
    set_w(GPIO(pin), GPIO_IE | GPIO_PUE);
    clr_w(GPIO(pin), GPIO_PDE);
}

/** ---------------- mission task ---------------- */
LOCAL void mission_task (int32_t stacd, void * p_exinf)
{
    uint32_t start_t       = 0;
    uint32_t stop_t        = 0; /* when the button went down */
    BOOL     b_start_dn    = FALSE;
    BOOL     b_stop_dn     = FALSE;
    BOOL     b_start_armed = TRUE;
    UINT     ptn; /* T-Kernel API type (tk_wai_flg). */
    ER       err_x;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        uint32_t tval;
        BOOL     b_start_now = FALSE;

        err_x = tk_wai_flg(g_mission_flg, EV_ALL, TWF_ORW | TWF_BITCLR, &ptn,
                           MISSION_PERIOD_MS);
        if (err_x != E_OK)
        {
            ptn = 0;
        }
        tval = motion_now_ms();

        /* buttons (active low): pressed = held down for 40 ms */
        if (gpio_get_val(PIN_BTN_STOP))
        {
            b_stop_dn = FALSE;
        }
        else if (!b_stop_dn)
        {
            b_stop_dn = TRUE;
            stop_t    = tval;
        }
        else
        {
            /* No action is required in this case. */
        }
        if (gpio_get_val(PIN_BTN_START))
        {
            b_start_dn    = FALSE;
            b_start_armed = TRUE;
        }
        else if (!b_start_dn)
        {
            b_start_dn = TRUE;
            start_t    = tval;
        }
        else
        {
            /* No action is required in this case. */
        }
        if ((b_start_dn && b_start_armed) && ((tval - start_t) >= 40U))
        {
            b_start_now   = TRUE;
            b_start_armed = FALSE;
        }

        if (ptn & EV_CMD_STOP)
        {
            b_stop_dn = TRUE;
            stop_t    = tval - 40U;
        } /* remote STOP */
        if (((((ptn & EV_CMD_START) && (g_state != MS_FOLLOW)) &&
              (g_state != MS_AVOID)) &&
             (g_state != MS_TURN)) &&
            (g_state != MS_REVERSE))
        {
            b_start_now = TRUE;
        }
        if (b_stop_dn && ((tval - stop_t) >= 40U))
        {
            if (ptn & EV_CMD_STOP)
            {
                b_stop_dn = FALSE; /* one-shot */
            }
            if (g_state != MS_IDLE)
            {
                halt_all();
                ranger_led(FALSE);
                g_state    = MS_IDLE;
                gp_text    = "IDLE";
                g_pend_act = BC_ACT_NONE;
                tm_printf((uint8_t *)"MISSION STOP\n");
            }
            continue;
        }

        /* tipped over / picked up: stop whatever we were doing */
        /* tipped over / picked up: SAFE STOP (brake, FAULT, needs START) */
        if ((ptn & (EV_TILT | EV_LIFT)) &&
            ((((MS_FOLLOW == g_state) || (MS_AVOID == g_state)) ||
              (MS_TURN == g_state)) ||
             (MS_REVERSE == g_state)))
        {
            fault((ptn & EV_LIFT) ? "picked up (IMU): safe stop"
                                  : "tilted over (IMU): safe stop",
                  FAULT_MOTION);
            continue;
        }
        /* remote PAUSE / RESUME */
        if ((ptn & EV_CMD_PAUSE) &&
            (((MS_FOLLOW == g_state) || (MS_AVOID == g_state)) ||
             (MS_TURN == g_state)))
        {
            halt_all();
            ranger_led(FALSE);
            g_state = MS_PAUSED;
            gp_text = "PAUSED";
            tm_printf((uint8_t *)"MISSION PAUSE\n");
            continue;
        }
        if ((ptn & EV_CMD_RESUME) && (MS_PAUSED == g_state))
        {
            begin_follow(0, FALSE);
            continue;
        }
        /* remote U-TURN: turn round on the line and follow it back */
        if ((((ptn & EV_CMD_UTURN) && (g_state != MS_AVOID)) &&
             (g_state != MS_TURN)) &&
            (g_state != MS_REVERSE))
        {
            g_pend_act = BC_ACT_NONE;
            begin_turn(BC_ACT_UTURN, tval, FALSE);
            continue;
        }
        /* remote REVERSE: back up REVERSE_MM and stop (no rear sensor: short
         * and slow) */
        if ((ptn & EV_CMD_REVERSE) && (g_state != MS_REVERSE))
        {
            line_follow(FALSE, 0, FALSE);
            ranger_guard(FALSE);
            motion_wheels(MOT_OWNER_MISSION, -REVERSE_MMS, -REVERSE_MMS);
            motion_set_owner(MOT_OWNER_MISSION);
            g_rev_odo0 = motion_odometry_mm();
            g_rev_t0   = tval;
            g_state    = MS_REVERSE;
            gp_text    = "REVERSE";
            tm_printf((uint8_t *)"MISSION reversing %d mm\n", REVERSE_MM);
            continue;
        }
        if (((g_pend_act != BC_ACT_NONE) &&
             ((tval - g_pend_t) > TURN_PENDING_MS)) &&
            (!line_inside()))
        {
            tm_printf((uint8_t *)"MISSION no junction after the barcode: turn "
                                 "dropped\n");
            g_pend_act = BC_ACT_NONE;
        }

        switch (g_state)
        {
            case MS_IDLE:
            case MS_FAULT:
            case MS_PAUSED:
            case MS_DONE:
                if (b_start_now)
                {
                    g_pend_act = BC_ACT_NONE;
                    begin_follow(0, FALSE);
                }
            break;
            case MS_FOLLOW:
                if (climb_step(tval))
                {
                    break; /* faulted: stuck on a ramp */
                }
                /* going round in circles = the line is lost but a sensor
                   keeps catching something now and then, so the line
                   follower never gives up.  Heading sampled once a
                   second: more than 270 deg in 10 s, or 400 deg in 30 s
                   (a real track turns far less) -> stop */
                if ((motion_heading_ok()) && ((tval - g_spin_t0) >= 1000U))
                {
                    int32_t hval                      = motion_heading_d10();
                    g_spin_t0                         = tval;
                    g_spin_hist[g_spin_i % SPIN_HIST] = hval;
                    g_spin_i++;
                    if (((g_spin_i > 10) &&
                         (ABS(hval - g_spin_hist[(g_spin_i - 11) % SPIN_HIST]) >
                          2700)) ||
                        ((g_spin_i > SPIN_HIST) &&
                         (ABS(hval - g_spin_hist[g_spin_i % SPIN_HIST]) >
                          4000)))
                    {
                        fault("going round in circles: line lost", FAULT_LINE);
                        break;
                    }
                }
                if (gb_post_avoid)
                {
                    int32_t dh_val = motion_heading_d10() - g_h_start;
                    if ((!motion_heading_ok()) ||
                        ((motion_odometry_mm() - g_post_odo0) >
                         AV_BACKWARD_CHECK_MM))
                    {
                        gb_post_avoid =
                            FALSE; /* far enough: normal following */
                    }
                    else if (ABS(dh_val) > (AV_BACKWARD_DEG * 10))
                    {
                        if (++g_back_n > AV_BACKWARD_TRIES)
                        {
                            fault("keeps following the line backwards after "
                                  "the obstacle",
                                  FAULT_AVOID);
                            break;
                        }
                        tm_printf(
                            (uint8_t *)"MISSION facing %d deg off the old "
                                       "heading: following "
                                       "the line BACKWARDS, turning back\n",
                            (int32_t)(dh_val / 10));
                        if (0 == g_dir)
                        {
                            g_dir =
                                AV_PREFER_LEFT ? 1 : -1; /* FIND sweep order */
                        }
                        line_follow(FALSE, 0, FALSE);
                        ranger_guard(FALSE);
                        g_state = MS_AVOID;
                        turn_to(g_h_start);
                        go_av(AV_REJOIN, tval);
                        break;
                    }
                    else
                    {
                        /* No action is required in this case. */
                    }
                }
                if (ptn & EV_OBSTACLE)
                {
                    begin_avoid(tval);
                }
                else if (ptn & EV_LINE_LOST)
                {
                    fault("line lost", FAULT_LINE);
                }
                else if (ptn & EV_BARCODE)
                {
                    on_barcode(tval);
                }
                else if (((g_pend_act != BC_ACT_NONE) && (line_inside())) &&
                         (motion_odometry_mm() >= g_pend_odo))
                {
                    uint32_t aval = g_pend_act;
                    g_pend_act    = BC_ACT_NONE;
                    begin_turn(aval, tval, TRUE);
                }
                else if ((ptn & EV_JUNCTION) && (g_pend_act != BC_ACT_NONE))
                {
                    uint32_t aval = g_pend_act;
                    g_pend_act    = BC_ACT_NONE;
                    begin_turn(aval, tval, TRUE);
                }
                else
                {
                    /* No action is required in this case. */
                }
            break;
            case MS_TURN:
                turn_step(tval);
            break;
            case MS_REVERSE:
                if (((g_rev_odo0 - motion_odometry_mm()) >= REVERSE_MM) ||
                    ((tval - g_rev_t0) > REVERSE_MAX_MS))
                {
                    halt_all();
                    g_state = MS_IDLE;
                    gp_text = "IDLE";
                    tm_printf((uint8_t *)"MISSION reversed %d mm, stopped\n",
                              (int32_t)(g_rev_odo0 - motion_odometry_mm()));
                }
            break;
            case MS_AVOID:
                avoid_step(tval);
            break;
            default:
            {
                /* No other values are expected. */
                break;
            }
        }
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 * @return Result as described above.
 */
EXPORT ER mission_init (void)
{
    T_CFLG cflg = {0};

    button_pin(PIN_BTN_START);
    button_pin(PIN_BTN_STOP);
    cflg.flgatr   = TA_TFIFO | TA_WSGL; /* only the mission task waits */
    g_mission_flg = tk_cre_flg(&cflg);
    if (g_mission_flg < E_OK)
    {
        return g_mission_flg;
    }
    return E_OK;
}

/**
 * @brief  Start the module's task(s).
 * @return Result as described above.
 */
EXPORT ER mission_start (void)
{
    ID tid = app_main_task_start((FP)mission_task, PRI_MISSION, CORE_MISSION,
                                 "mission");
    return (tid < E_OK) ? tid : E_OK;
}

/*** end of file ***/
