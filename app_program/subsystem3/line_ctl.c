/**
 * @file    line_ctl.c
 *  line_ctl.c  (Buddy 3)
 *  Pure line-following logic.  See line_ctl.h.  No kernel calls here.
 */
#include "line_ctl.h"

/* same numbers as LINE_M_* in car.h (checked in subsystem3.c) */
#define LC_TRACK    1
#define LC_CENTRED  2
#define LC_RECOVER  3
#define LC_JUNCTION 4
#define LC_LOST     5
#define LC_BENDSTOP 6
#define LC_PIVOT    7
/* motion_drive(): steer 500 = inner wheel stopped */
#define PIVOT_STEER_PM 500
#ifndef EDGE_KP
#define EDGE_KP 1000 /* CAL (line_sim EDGE=1) */
#endif
#ifndef EDGE_KD
#define EDGE_KD 300
#endif
#ifndef PIVOT_TIGHT_MS
#define PIVOT_TIGHT_MS 400 /* line still not found after this ... */
#endif
#ifndef PIVOT_TIGHT_PM
/* ... inner wheel backs up at 40 % of the outer one */
#define PIVOT_TIGHT_PM 900
#endif

/* CAL values: tuned in testing/line_sim.c, then on the mat */
/* Sensors STRADDLE the line (gap wider than the line) */
const line_tune_t g_line_tune_default = {
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
    .calm_ms    = 0, /* 0: calm lasts one excursion (best in sim) */
    .lost_ms    = 2500,
    .blind_ms   = 0, /* 0 = off.  In the simulator a well-centred car ran 3 m
                with both sensors white, so a timeout here stops it
                on long straights.  Try 8000 if you want it. */
    .speed =
        80, /* 3 Nov: start slow; raise from the dashboard once bends work */
    .speed_min      = 50,
    .slow_steer     = 1000, /* straddle: slow down in proportion, as before */
    .slow_hold_ms   = 0,
    .pivot_steer    = PIVOT_STEER_STRADDLE,
    .pivot_stop_ms  = 200,
    .pivot_exit_pos = 250,
    .pivot_rest_ms  = 600,
    .pivot_max_ms   = 6000, /* a 90 deg pivot at 40 mm/s takes ~5 s */
};

/* Sensors both ON the line (gap narrower than the line), IS27 car 3 Nov.
   Simulator, 25 mm line, sensor gap 8..18 mm, tight bends R60..R120:
   the straddle tuning passed 152..202 of 216 runs (ran off the outside
   of tight bends); this one 210..216 of 216. */
const line_tune_t g_line_tune_inside = {
    .inside     = 1,
    .see        = 1200,
    .junction   = 3000, /* unused: both black is normal here */
    .kp_x1000   = 1500, /* stronger: only the edge of the line gives a signal */
    .kd_x1000   = 300,  /* more damping for the stronger gain */
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
    .speed =
        80, /* 3 Nov: start slow; raise from the dashboard once bends work */
    .speed_min      = 50,
    .slow_steer     = 400, /* inside: full bend speed from 40 % steering */
    .slow_hold_ms   = 400, /* and keep it 0.4 s after the bend */
    .pivot_steer    = PIVOT_STEER_INSIDE,
    .pivot_stop_ms  = 200,
    .pivot_exit_pos = 250,
    .pivot_rest_ms  = 600,
    .pivot_max_ms   = 6000, /* a 90 deg pivot at 40 mm/s takes ~5 s */
};

/* One sensor on the line, one on the white beside it (IS27, 3 Nov).
   Simulator, 25 mm line, sensor gap 8..18 mm, laggy car, 30 mm/s:
   see testing/line_sim (EDGE=1). */
const line_tune_t g_line_tune_edge = {
    .edge_dir       = 1, /* set at START from which sensor is on black */
    .inside         = LINE_EDGE,
    .see            = 1200,
    .junction       = 3000,
    .kp_x1000       = EDGE_KP,
    .kd_x1000       = EDGE_KD,
    .side_pos       = 150,
    .deep_pos       = 700,
    .fall_pos       = 100,
    .hold_small     = 150,
    .wobble_ms      = 200,
    .hold_deep      = 600, /* line lost: turn back toward it, harder ... */
    .hold_max       = 900, /* ... the longer it is missing */
    .ramp_ms        = 600,
    .calm_ms        = 0,
    .lost_ms        = 3000,
    .blind_ms       = 0,
    .speed          = 30, /* 3 Nov: very slow while the turning is sorted out */
    .speed_min      = 25,
    .slow_steer     = 400,
    .slow_hold_ms   = 400,
    .pivot_steer    = PIVOT_STEER_EDGE,
    .pivot_stop_ms  = 200,
    .pivot_exit_pos = 250,
    .pivot_rest_ms  = 600,
    .pivot_max_ms   = 10000, /* a 90 deg pivot at 25 mm/s takes ~8 s */
};

/**
 * @brief  Iabs.
 * @return Result as described above.
 */
static int32_t iabs (int32_t val)
{
    return (val < 0) ? -val : val;
}
/**
 * @brief  Clamp.
 * @return Result as described above.
 */
static int32_t clamp (int32_t val, int32_t lo_val, int32_t hi_val)
{
    return (val < lo_val) ? lo_val : (val > hi_val) ? hi_val : val;
}

/**
 * @brief  Line ctl reset.
 */
void line_ctl_reset (line_ctl_t * p_chr, const line_tune_t * p_key,
                     int32_t seed_side, int32_t seed_deep,
                     uint32_t motion_now_ms)
{
    p_chr->k        = p_key;
    p_chr->prev_pos = 0;
    p_chr->side     = (seed_side > 0) ? 1 : (seed_side < 0) ? -1 : 0;
    p_chr->deep     = (seed_deep && (p_chr->side)) ? 1 : 0;
    p_chr->peak     = (p_chr->deep) ? p_key->deep_pos : 0;
    p_chr->calm     = 0;
    p_chr->t_seen   = motion_now_ms;
    p_chr->t_bend   = motion_now_ms - 100000U;
    p_chr->mode     = LC_CENTRED;
    p_chr->edge     = (p_key->edge_dir >= 0) ? 1 : -1;
    /* 12 Nov: edge layout restarted with a known line side (after an
       obstacle or a barcode turn): side +1 = line to the right = under
       the RIGHT sensor (edge -1).  Without this, RECOVER used the edge
       found at START and could steer away from the line. */
    if ((LINE_EDGE == p_key->inside) && (p_chr->deep))
    {
        p_chr->edge = -p_chr->side;
    }
    p_chr->pv       = 0;
    p_chr->pv_dir   = 0;
    p_chr->pv_seen  = 0;
    p_chr->t_pv     = motion_now_ms;
    p_chr->t_pv_end = motion_now_ms - 100000U;
}

/**
 * @brief  Line ctl step.
 */
void line_ctl_step (line_ctl_t * p_chr, int32_t dl_val, int32_t dr_val,
                    uint32_t now, line_out_t * p_out)
{
    const line_tune_t * p_key = p_chr->k;
    int32_t             see_l = dl_val >= p_key->see;
    int32_t             see_r = dr_val >= p_key->see;
    int32_t pos   = clamp(((dr_val - dl_val) * 1000) / 4095, -1000, 1000);
    int32_t steer = 0;
    int32_t slow;

    p_out->lost = 0;

    if (LINE_EDGE == p_key->inside)
    {
        /* edge: one sensor on black, one on white.  dl + dr = 4095 on
           the edge; less = off the line, more = over it. */
        int32_t dir;
        /* whichever sensor is on black IS the line sensor.  If the
           car crosses the whole line (an inside corner, or a big
           drift), it simply follows the other edge from then on,
           instead of steering back across the line the wrong way. */
        if (see_l && (dr_val < (p_key->see / 2)))
        {
            p_chr->edge = 1;
        }
        else if (see_r && (dl_val < (p_key->see / 2)))
        {
            p_chr->edge = -1;
        }
        else
        {
            /* No action is required in this case. */
        }
        dir = p_chr->edge;
        pos =
            clamp((dir * (dl_val + dr_val - 4095) * 1000) / 4095, -1000, 1000);
        if ((!see_l) && (!see_r))
        {
            /* both white: the line is on the black sensor's side */
            uint32_t gone = now - p_chr->t_seen;
            int32_t  gval = (int32_t)((gone > (uint32_t)p_key->ramp_ms)
                                          ? (uint32_t)p_key->ramp_ms
                                          : gone);
            p_chr->side   = -dir;
            steer =
                p_chr->side * (p_key->hold_deep +
                               (((p_key->hold_max - p_key->hold_deep) * gval) /
                                p_key->ramp_ms));
            p_chr->mode = LC_RECOVER;
            if (gone > (uint32_t)p_key->lost_ms)
            {
                p_chr->mode = LC_LOST;
                p_out->lost = 1;
            }
        }
        else
        {
            steer = ((p_key->kp_x1000 * pos) +
                     (p_key->kd_x1000 * (pos - p_chr->prev_pos))) /
                    1000;
            if (iabs(pos) >= p_key->side_pos)
            {
                p_chr->side = (pos > 0) ? 1 : -1;
            }
            p_chr->mode   = LC_TRACK;
            p_chr->t_seen = now;
        }
    }
    else if (((((!p_key->inside) && see_l) && see_r) &&
              (dl_val >= p_key->junction)) &&
             (dr_val >= p_key->junction))
    {
        /* both black: a crossing or a wide mark.  Go straight. */
        p_chr->mode   = LC_JUNCTION;
        p_chr->t_seen = now;
        steer         = 0;
    }
    else if (see_l || see_r)
    {
        int32_t aval     = iabs(pos);
        int32_t side_now = (aval >= p_key->side_pos) ? ((pos > 0) ? 1 : -1) : 0;

        if (LC_RECOVER == p_chr->mode)
        {
            /* found it again.  Same side = it really was outside
               (a curve).  Other side = we overshot: calm down. */
            if (side_now != 0)
            {
                p_chr->calm = (side_now != p_chr->side);
            }
            p_chr->peak = 0;
            p_chr->deep = 0;
        }
        if (side_now != 0)
        {
            if (side_now != p_chr->side)
            {
                p_chr->peak = 0;
                p_chr->deep = 0;
            }
            p_chr->side = side_now;
            if (aval > p_chr->peak)
            {
                p_chr->peak = aval;
            }
            if ((p_chr->peak >= p_key->deep_pos) && (!p_chr->calm))
            {
                p_chr->deep = 1;
            }
        }
        steer = ((p_key->kp_x1000 * pos) +
                 (p_key->kd_x1000 * (pos - p_chr->prev_pos))) /
                1000;

        /* deep and fading on the same side: the line may be sliding
           out past the sensor.  Don't let the steering collapse. */
        if (((((p_chr->deep) && (side_now == p_chr->side)) &&
              (side_now != 0)) &&
             (aval < (p_chr->peak - p_key->fall_pos))) &&
            (iabs(steer) < p_key->hold_deep))
        {
            steer = p_chr->side * p_key->hold_deep;
        }

        p_chr->mode   = LC_TRACK;
        p_chr->t_seen = now;
    }
    else
    {
        /* both white */
        uint32_t gone = now - p_chr->t_seen;

        if ((p_key->inside) && (p_chr->side))
        {
            p_chr->deep = 1; /* both white = off the line, for sure */
        }
        if ((p_chr->deep) && (p_chr->side))
        {
            int32_t gval = (int32_t)((gone > (uint32_t)p_key->ramp_ms)
                                         ? (uint32_t)p_key->ramp_ms
                                         : gone);
            steer =
                p_chr->side * (p_key->hold_deep +
                               (((p_key->hold_max - p_key->hold_deep) * gval) /
                                p_key->ramp_ms));
            p_chr->mode = LC_RECOVER;
            if (gone > (uint32_t)p_key->lost_ms)
            {
                p_chr->mode = LC_LOST;
                p_out->lost = 1;
            }
        }
        else
        {
            if (gone > (uint32_t)p_key->calm_ms)
            {
                p_chr->calm = 0; /* settled */
            }
            p_chr->mode = LC_CENTRED;
            steer       = ((p_chr->side) && (gone < (uint32_t)p_key->wobble_ms))
                              ? p_chr->side * p_key->hold_small
                              : 0;
            if ((p_key->blind_ms > 0) && (gone > (uint32_t)p_key->blind_ms))
            {
                p_chr->mode = LC_LOST;
                p_out->lost = 1;
            }
        }
    }

    steer           = clamp(steer, -1000, 1000);
    p_chr->prev_pos = pos;

    /* slower when steering hard, slowest while recovering.  Full bend
       speed from |steer| = slow_steer, held slow_hold_ms after the bend
       so the car doesn't speed up again half way round it. */
    {
        int32_t aval   = iabs(steer);
        int32_t ss_val = (p_key->slow_steer > 0) ? p_key->slow_steer : 1000;
        if ((aval >= ss_val) || (LC_RECOVER == p_chr->mode))
        {
            p_chr->t_bend = now;
        }
        if (aval > ss_val)
        {
            aval = ss_val;
        }
        slow = ((p_key->speed - p_key->speed_min) * aval) / ss_val;
        if ((now - p_chr->t_bend) < (uint32_t)p_key->slow_hold_ms)
        {
            slow = p_key->speed - p_key->speed_min;
        }
    }
    p_out->speed =
        (LC_RECOVER == p_chr->mode) ? p_key->speed_min : p_key->speed - slow;
    p_out->steer = steer;
    p_out->mode  = p_chr->mode;

    /* ---- bend: stop, then pivot on the inner wheel ---- */
    if (p_key->pivot_steer > 0)
    {
        int32_t aval     = iabs(pos);
        int32_t side_now = (aval >= p_key->side_pos) ? ((pos > 0) ? 1 : -1) : 0;

        if (((0 == p_chr->pv) && (p_chr->mode != LC_JUNCTION)) &&
            ((iabs(steer) >= p_key->pivot_steer) ||
             ((LC_RECOVER == p_chr->mode) && (p_chr->side != 0))))
        {
            p_chr->pv_dir = (LC_RECOVER == p_chr->mode)
                                ? p_chr->side
                                : ((steer > 0) ? 1 : -1);
            p_chr->pv =
                ((now - p_chr->t_pv_end) > (uint32_t)p_key->pivot_rest_ms) ? 1
                                                                           : 2;
            p_chr->pv_seen = (see_l || see_r);
            p_chr->t_pv    = now;
        }
        if (p_chr->pv)
        {
            int32_t done;
            if (see_l || see_r)
            {
                p_chr->pv_seen = 1;
            }
            if (LINE_EDGE == p_key->inside) /* back on the edge */
            {
                done = (see_l || see_r) && (aval < p_key->pivot_exit_pos);
            }
            else if (p_key
                         ->inside) /* both on the line again, near the middle */
            {
                done = (see_l && see_r) && (aval < p_key->pivot_exit_pos);
            }
            else /* line back between the sensors */
            {
                done = ((p_chr->pv_seen) && (!see_l)) && (!see_r);
            }
            /* or it swung past the middle onto the other sensor */
            if ((side_now != 0) && (side_now != p_chr->pv_dir))
            {
                done = 1;
            }

            if ((1 == p_chr->pv) &&
                ((now - p_chr->t_pv) >= (uint32_t)p_key->pivot_stop_ms))
            {
                p_chr->pv   = 2;
                p_chr->t_pv = now;
            }
            if (done && (2 == p_chr->pv))
            {
                p_chr->pv       = 0;
                p_chr->t_pv_end = now;
                p_chr->t_seen   = now;
                p_chr->t_bend   = now; /* bend speed for slow_hold_ms */
                if (!p_key->inside)
                {
                    p_chr->deep = 0;
                    p_chr->peak = 0;
                }
                /* carry on with normal steering, at bend speed */
                p_out->speed = p_key->speed_min;
            }
            else if (1 == p_chr->pv)
            {
                p_out->speed  = 0; /* brake first */
                p_out->steer  = 0;
                p_out->mode   = LC_BENDSTOP;
                p_chr->t_bend = now;
            }
            else
            {
                p_out->speed = p_key->speed_min; /* outer wheel at bend speed */
                p_out->steer =
                    p_chr->pv_dir * PIVOT_STEER_PM; /* inner wheel stopped */
                /* a corner sharper than the pivot circle: the line is
                   still not under a sensor, so the inner wheel backs
                   up a little and the car turns on the spot */
                if (((!see_l) && (!see_r)) &&
                    ((now - p_chr->t_pv) > (uint32_t)PIVOT_TIGHT_MS))
                {
                    p_out->steer = p_chr->pv_dir * PIVOT_TIGHT_PM;
                }
                p_out->mode   = LC_PIVOT;
                p_chr->t_bend = now;
            }
            /* taking too long: hand back to normal steering (which
               still stops the car if the line is really gone) */
            if ((2 == p_chr->pv) &&
                ((now - p_chr->t_pv) > (uint32_t)p_key->pivot_max_ms))
            {
                p_chr->pv       = 0;
                p_chr->t_pv_end = now;
            }
        }
    }
    if (p_chr->pv)
    {
        /* the pivot is looking for the line: the RECOVER timer must
           not give up while it turns (pivot_max_ms limits it) */
        p_out->lost   = 0;
        p_chr->t_seen = now;
    }
    if (p_out->lost)
    {
        p_out->speed = 0;
    }
    p_out->pos = pos;
}

/*** end of file ***/
