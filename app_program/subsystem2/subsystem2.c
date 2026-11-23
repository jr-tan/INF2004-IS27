/**
 * @file    subsystem2.c
 *  subsystem2.c  (Buddy 2: motion)
 *  motion task: the ONLY code that writes the motor PWM.
 *
 *  Every 20 ms (ctl_tick_isr cyclic handler -> MF_TICK), or at once on
 *  MF_WAKE (owner change / STOP):
 *    1. read the slot of the current owner (left/right mm/s)
 *    2. ramp toward it (MOT_ACCEL_MMS2)
 *    3. duty = (D0 + K * speed) * gain + kick + encoder trim
 *    4. encoder health + trim learning (only if BOTH encoders healthy)
 */
#include "subsystem2.h"

EXPORT ID               g_motion_flg;
LOCAL ID                g_ctl_cyc;
LOCAL volatile uint32_t g_slot[MOT_NUM_OWNERS]; /* (UH)left | (UH)right << 16 */
LOCAL volatile uint32_t g_owner = MOT_OWNER_NONE;
LOCAL motion_status_t   g_stat;
LOCAL volatile int32_t  g_odo_mm; /** written by the motion task only */

EXPORT int32_t motion_odometry_mm (void)
{
    return g_odo_mm;
}
LOCAL volatile int32_t g_head_d10; /* written by the motion task only */
LOCAL volatile BOOL    gb_head_ok = TRUE;
/**
 * @brief  Motion heading d10.
 * @return Result as described above.
 */
EXPORT int32_t motion_heading_d10 (void)
{
    return g_head_d10;
}
/**
 * @brief  Motion heading ok.
 * @return Result as described above.
 */
EXPORT BOOL motion_heading_ok (void)
{
    return gb_head_ok;
}
LOCAL volatile uint32_t g_loop_max;
/**
 * @brief  Motion loop us max.
 * @return Result as described above.
 */
EXPORT uint32_t motion_loop_us_max (void)
{
    uint32_t val = g_loop_max;
    g_loop_max   = 0;
    return val;
}

/**
 * @brief  Motion now us.
 * @return Result as described above.
 */
EXPORT uint32_t motion_now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** motion_now_us() / 1000 would jump from 4294967 back to 0, which breaks
   "t - t0" in ms.  Use all 64 bits (read high, low, high again). */
EXPORT uint32_t motion_now_ms (void)
{
    uint32_t hi_val;
    uint32_t lo_val;
    uint32_t hi2;
    do
    {
        hi_val = in_w(TIMER_TIMERAWH);
        lo_val = in_w(TIMER_TIMERAWL);
        hi2    = in_w(TIMER_TIMERAWH);
    } while (hi_val != hi2);
    return (uint32_t)((((UD)hi_val << 32) | lo_val) / 1000U);
}

/* ---------------- API (any task, any core) ---------------- */
#define PACK(l, r)                      \
    ((uint32_t)(uint16_t)(int16_t)(l) | \
     ((uint32_t)(uint16_t)(int16_t)(r) << 16))
#define UNPACK_L(w) ((int32_t)(int16_t)(uint16_t)((w) & 0xFFFFU))
#define UNPACK_R(w) ((int32_t)(int16_t)(uint16_t)((w) >> 16))

/**
 * @brief  Motion wheels.
 */
EXPORT void motion_wheels (uint32_t obj, int32_t lval, int32_t res)
{
    if ((MOT_OWNER_NONE == obj) || (obj >= MOT_NUM_OWNERS))
    {
        return;
    }
    lval        = CLAMP(lval, -MOT_MAX_MMS, MOT_MAX_MMS);
    res         = CLAMP(res, -MOT_MAX_MMS, MOT_MAX_MMS);
    g_slot[obj] = PACK(lval, res); /* one 32-bit store */
}

/**
 * @brief  Motion drive.
 */
EXPORT void motion_drive (uint32_t obj, INT speed, INT steer)
{
    INT sval = ABS(CLAMP(steer, -1000, 1000));
    INT inner;

    if (speed < 0)
    {
        speed = 0;
    }
    /* inner wheel: 1 - 2s.  s = 500 -> stopped, s = 1000 -> -1/2 */
    inner = speed - ((speed * 2 * sval) / 1000);
    if (inner < (-speed / 2))
    {
        inner = -speed / 2;
    }
    if (steer >= 0)
    {
        motion_wheels(obj, speed, inner); /* + = turn right */
    }
    else
    {
        motion_wheels(obj, inner, speed);
    }
}

/**
 * @brief  Motion set owner.
 */
EXPORT void motion_set_owner (uint32_t obj)
{
    if (obj >= MOT_NUM_OWNERS)
    {
        return;
    }
    g_owner = obj;
    tk_set_flg(g_motion_flg, MF_WAKE);
}

/**
 * @brief  Motion stop.
 */
EXPORT void motion_stop (void)
{
    g_owner = MOT_OWNER_NONE;
    drv_motor_brake(MOTOR_L); /* brake now, even before the task runs */
    drv_motor_brake(MOTOR_R);
    tk_set_flg(g_motion_flg, MF_WAKE);
}

/**
 * @brief  Motion get status.
 */
EXPORT void motion_get_status (motion_status_t * p_state)
{
    *p_state = g_stat; /* debug copy: may be one tick mixed */
}

/* ---------------- 20 ms tick: wakes motion + line ---------------- */
IMPORT ID g_line_task_id;
/**
 * @brief  Interrupt handler.
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void ctl_tick_isr (void * p_exinf)
{
    (void)p_exinf;
    tk_set_flg(g_motion_flg, MF_TICK);
    if (g_line_task_id > 0)
    {
        tk_wup_tsk(g_line_task_id);
    }
}

/** ---------------- motion task ---------------- */
LOCAL int32_t ramp (int32_t cur, int32_t tgt, int32_t step)
{
    if (tgt > cur)
    {
        return ((tgt - cur) > step) ? cur + step : tgt;
    }
    return ((cur - tgt) > step) ? cur - step : tgt;
}

/**
 * @brief  Task body (micro T-Kernel task entry).
 * @param  stacd  Required by the micro T-Kernel signature; not used.
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void motion_task (int32_t stacd, void * p_exinf)
{
    static const int32_t gain[2] = {MOT_GAIN_L_X1000, MOT_GAIN_R_X1000};
    /* INT (T-Kernel, 32-bit) kept for cur, trim and boost: see
       docs/barr_c_conformance.md, D-5. */
    INT      cur[2]      = {0, 0}; /* ramped wheel speeds, mm/s */
    INT      trim[2]     = {0, 0}; /* learned duty correction, pm */
    uint32_t start_us[2] = {0, 0}; /* when this wheel last left 0 */
    uint32_t hist_cnt[ENC_WINDOW_TICKS][2];
    uint32_t hist_t[ENC_WINDOW_TICKS];
    uint32_t hi_val         = 0;
    uint32_t n_hist         = 0;
    BOOL     healthy[2]     = {TRUE, TRUE};
    uint32_t heal_t0[2]     = {0, 0};
    INT      boost[2]       = {0, 0}; /* stall boost, pm (12 Nov evening) */
    uint32_t boost_max_t[2] = {
        0, 0}; /* when the boost reached STALL_BOOST_MAX_PM */
    uint32_t   t_last     = motion_now_us();
    uint32_t   last_owner = MOT_OWNER_NONE;
    enc_snap_t es_val[2];
    int32_t    prev_pos[2] = {0, 0};
    int32_t    odo_um      = 0;
    int64_t    diff_um     = 0; /* right - left wheel travel, um */
    BOOL       b_first     = TRUE;
    BOOL       b_enc_fault = FALSE;
    UINT       ptn; /* T-Kernel API type (tk_wai_flg). */
    uint32_t   mval;
    ER         err_x;

    (void)stacd;
    (void)p_exinf;
    CHECK(drv_encoder_init(), "encoder init"); /* we are on core 0 */

    for (;;)
    {
        int32_t  tgt[2];
        int32_t  val[2]  = {0, 0};
        uint32_t duty[2] = {0, 0};
        uint32_t obj;
        uint32_t tval;
        uint32_t dt_us;
        int32_t  step;
        BOOL     b_trim_on;

        err_x  = tk_wai_flg(g_motion_flg, MF_TICK | MF_WAKE,
                            TWF_ORW | TWF_BITCLR, &ptn, 100);
        tval   = motion_now_us();
        dt_us  = tval - t_last;
        t_last = tval;
        if (dt_us > 100000U)
        {
            dt_us = 100000U;
        }
        (void)err_x;

        /* encoders + speed over the last ENC_WINDOW_TICKS samples */
        drv_encoder_snapshot(es_val);
        if (ENC_WINDOW_TICKS == n_hist)
        {
            uint32_t old  = hi_val; /* oldest entry */
            uint32_t span = tval - hist_t[old];
            for (mval = 0; mval < MOT_WHEELS; mval++)
            {
                uint32_t steps = es_val[mval].count - hist_cnt[old][mval];
                val[mval]      = span ? (int32_t)((steps * ENC_UM_PER_STEP) /
                                             ((span / 1000U) + 1U))
                                      : 0;
            }
        }
        for (mval = 0; mval < MOT_WHEELS; mval++)
        {
            hist_cnt[hi_val][mval] = es_val[mval].count;
        }
        hist_t[hi_val] = tval;
        hi_val         = (hi_val + 1U) % ENC_WINDOW_TICKS;
        if (n_hist < ENC_WINDOW_TICKS)
        {
            n_hist++;
        }

        obj = g_owner;
        if (obj != last_owner)
        {
            if (MOT_OWNER_NONE == obj)
            {
                cur[0] = cur[1] = 0;
            }
            last_owner = obj;
        }
        if (MOT_OWNER_NONE == obj)
        {
            tgt[0] = tgt[1] = 0;
            cur[0] = cur[1] = 0;
        }
        else
        {
            uint32_t wval = g_slot[obj];
            tgt[0]        = UNPACK_L(wval);
            tgt[1]        = UNPACK_R(wval);
        }

        step = (int32_t)(((uint32_t)MOT_ACCEL_MMS2 * dt_us) / 1000000U);
        if (step < 1)
        {
            step = 1;
        }

        for (mval = 0; mval < MOT_WHEELS; mval++)
        {
            int32_t was = cur[mval];
            int32_t sp_val;
            int32_t dval;

            /* reversing: ramp through zero */
            if (((tgt[mval] > 0) && (cur[mval] < 0)) ||
                ((tgt[mval] < 0) && (cur[mval] > 0)))
            {
                cur[mval] = ramp(cur[mval], 0, step * 2);
            }
            else
            {
                cur[mval] = ramp(cur[mval], tgt[mval], step);
            }
            if ((0 == was) && (cur[mval] != 0))
            {
                start_us[mval] = tval;
            }

            sp_val = ABS(cur[mval]);
            if (((was != 0) && (cur[mval] != 0)) &&
                ((was > 0) != (cur[mval] > 0)))
            {
                boost[mval] = 0;
            }
            if (sp_val < 5)
            {
                drv_motor_brake(mval);
                duty[mval]  = 0;
                boost[mval] = 0;
                continue;
            }

            /* stall boost (12 Nov evening): a wheel that is driven but
               not turning (ramp, one-sided ramp, pivoting on one
               wheel at ~15 % duty) gets more duty, step by step,
               until it moves; the boost then fades out slowly */
            if ((((healthy[mval]) && (sp_val >= STALL_MIN_MMS)) &&
                 ((tval - start_us[mval]) > (MOT_KICK_MS * 1000U))) &&
                ((tval - es_val[mval].last_us) > (STALL_MS * 1000U)))
            {
                if (boost[mval] < STALL_BOOST_MAX_PM)
                {
                    boost[mval] += STALL_BOOST_STEP_PM;
                    if (boost[mval] >= STALL_BOOST_MAX_PM)
                    {
                        boost[mval]       = STALL_BOOST_MAX_PM;
                        boost_max_t[mval] = tval;
                    }
                }
            }
            else if (boost[mval] > 0)
            {
                boost[mval] -= STALL_BOOST_FADE_PM;
                if (boost[mval] < 0)
                {
                    boost[mval] = 0;
                }
            }
            else
            {
                /* No action is required in this case. */
            }

            dval = ((MOT_D0_PM + ((sp_val * MOT_K_PM_X1000) / 1000)) *
                    gain[mval]) /
                   1000;
            dval += trim[mval] + boost[mval];
            if ((tval - start_us[mval]) < (MOT_KICK_MS * 1000U))
            {
                dval += MOT_KICK_PM;
            }
            /* trim learnt with the wheels in the air can be very
               negative: never let it push the duty below the level
               where the wheel stalls */
            dval       = CLAMP(dval, MOT_D0_PM + 30, 1000);
            duty[mval] = (uint32_t)dval;
            drv_motor_set(mval, (cur[mval] > 0) ? 1 : -1, duty[mval]);
        }

        /* encoder health: driven but no steps.  12 Nov evening: a
           silent wheel is first treated as STALLED (boost above); only
           if it is still silent STALL_GIVEUP_MS after the boost hit its
           maximum is the ENCODER declared dead.  Before, a wheel stuck
           on a ramp was called a dead encoder after 0.2 s, its trim was
           thrown away and the car never got going again. */
        BOOL fresh[2];
        for (mval = 0; mval < MOT_WHEELS; mval++)
        {
            BOOL b_driven = (ABS(cur[mval]) >= TRIM_MIN_MMS) &&
                            ((tval - start_us[mval]) >
                             ((MOT_KICK_MS + ENC_DEAD_MS) * 1000U));
            BOOL b_silent =
                (tval - es_val[mval].last_us) > (ENC_DEAD_MS * 1000U);
            fresh[mval] = b_driven && (!b_silent);
            if (healthy[mval])
            {
                if (((b_driven && b_silent) &&
                     (boost[mval] >= STALL_BOOST_MAX_PM)) &&
                    ((tval - boost_max_t[mval]) > (STALL_GIVEUP_MS * 1000U)))
                {
                    healthy[mval] = FALSE;
                    trim[mval]  = 0; /* learned from a dead wheel: forget it */
                    boost[mval] = 0;
                }
                heal_t0[mval] = tval;
            }
            else if (b_silent)
            {
                heal_t0[mval] = tval; /* restart the heal timer */
            }
            else if ((tval - heal_t0[mval]) > (ENC_HEAL_MS * 1000U))
            {
                healthy[mval] = TRUE;
            }
            else
            {
                /* No action is required in this case. */
            }
        }

        /* speed control (trim), PER WHEEL.  12 Nov evening: it used
           to need both wheels above 40 mm/s and no pivoting, so at the
           line speed (25..30 mm/s, pivoting) it NEVER ran and the
           motors were pure open loop at ~15 % duty: any slope stalled
           them.  Now each wheel learns from TRIM_MIN_MMS up. */
        b_trim_on = FALSE;
        if (ENC_WINDOW_TICKS == n_hist)
        {
            for (mval = 0; mval < MOT_WHEELS; mval++)
            {
                int32_t err;
                if (((((!healthy[mval]) || (!fresh[mval])) ||
                      (boost[mval] != 0)) ||
                     (ABS(cur[mval]) < TRIM_MIN_MMS)) ||
                    (ABS(cur[mval] - tgt[mval]) > 20))
                {
                    continue;
                }
                err = ABS(cur[mval]) - val[mval];
                trim[mval] += (err * TRIM_KI_X1000) / 1000;
                trim[mval] = CLAMP(trim[mval], -TRIM_MAX_PM, TRIM_MAX_PM);
                b_trim_on  = TRUE;
            }
        }

        /* odometry: average of the healthy wheels; none healthy ->
           estimate from the commanded speed */
        {
            int32_t d_first  = es_val[0].pos - prev_pos[0];
            int32_t d_second = es_val[1].pos - prev_pos[1];
            int32_t step_um;
            if (b_first)
            {
                d_first = d_second = 0;
                b_first            = FALSE;
            }
            prev_pos[0] = es_val[0].pos;
            prev_pos[1] = es_val[1].pos;
            /* 12 Nov: one encoder unhealthy (dropping out): estimate its
               travel from the healthy wheel and the commanded speed ratio
               (spin in place: -1, straight: +1) instead of giving up on
               the heading.  Without this every obstacle turn fell back
               to timing, and the car swung far too wide. */
            if (healthy[0] != healthy[1])
            {
                uint32_t gval   = (healthy[0]) ? 0U : 1U; /* the good wheel */
                int32_t  dg_val = gval ? d_second : d_first;
                int32_t  est =
                    (cur[gval])
                         ? (int32_t)((dg_val * cur[gval ^ 1U]) / cur[gval])
                         : 0;
                if (gval)
                {
                    d_first = est;
                }
                else
                {
                    d_second = est;
                }
            }
            if ((healthy[0]) || (healthy[1]))
            {
                step_um = ((d_first + d_second) * ENC_UM_PER_STEP) / 2;
            }
            else
            {
                step_um = (int32_t)(((cur[0] + cur[1]) / 2) *
                                    (int32_t)(dt_us / 1000U));
            }
            odo_um += step_um;
            g_odo_mm = odo_um / 1000;
            /* heading: (right - left) / track radians, as 0.1 deg */
            diff_um += (int64_t)(d_second - d_first) * ENC_UM_PER_STEP;
            g_head_d10 =
                (int32_t)((diff_um * 18000LL) / (31416LL * MOT_TRACK_MM));
            gb_head_ok = (healthy[0]) ||
                         (healthy[1]); /* one good wheel is enough (above) */
        }
        if (((healthy[0]) && (healthy[1])) == b_enc_fault)
        { /* changed */
            b_enc_fault = !b_enc_fault;
            if (b_enc_fault)
            {
                comms_fault_raise(FAULT_ENCODER);
            }
            else
            {
                comms_fault_clear(FAULT_ENCODER);
            }
        }

        g_stat.owner   = obj;
        g_stat.trim_on = b_trim_on;
        for (mval = 0; mval < MOT_WHEELS; mval++)
        {
            g_stat.tgt_mms[mval] = cur[mval];
            g_stat.duty_pm[mval] = duty[mval];
            g_stat.trim_pm[mval] = trim[mval];
            g_stat.v_mms[mval]   = (healthy[mval]) ? val[mval] : 0;
            g_stat.count[mval]   = es_val[mval].count;
            g_stat.pos[mval]     = es_val[mval].pos;
            g_stat.glitch[mval]  = es_val[mval].glitch;
            g_stat.healthy[mval] = healthy[mval];
        }
        {
            uint32_t took = motion_now_us() - tval; /* this loop's CPU time */
            if (took > g_loop_max)
            {
                g_loop_max = took;
            }
        }
    }
}

/** ---------------- init / start ---------------- */
EXPORT ER subsystem2_init (void)
{
    T_CFLG cflg = {0};
    T_CCYC ccyc = {0};

    drv_motor_init(); /* braked from here on */

    cflg.flgatr  = TA_TFIFO | TA_WSGL; /* only the motion task waits */
    g_motion_flg = tk_cre_flg(&cflg);
    if (g_motion_flg < E_OK)
    {
        return g_motion_flg;
    }

    ccyc.cycatr = TA_HLNG; /* created stopped */
    ccyc.cychdr = (FP)ctl_tick_isr;
    ccyc.cyctim = CTL_PERIOD_MS;
    ccyc.cycphs = CTL_PERIOD_MS;
    g_ctl_cyc   = tk_cre_cyc(&ccyc);
    if (g_ctl_cyc < E_OK)
    {
        return g_ctl_cyc;
    }
    return E_OK;
}

/**
 * @brief  Start the module's task(s).
 * @return Result as described above.
 */
EXPORT ER subsystem2_start (void)
{
    ID tid =
        app_main_task_start((FP)motion_task, PRI_MOTION, CORE_MOTION, "motion");
    if (tid < E_OK)
    {
        return tid;
    }
    return tk_sta_cyc(g_ctl_cyc);
}

#if APP_TEST
/**
 * @brief  On-target self-test of this subsystem (built only when
 *         APP_TEST = 1).
 */
EXPORT void subsystem2_test (void)
{
}
#endif

/*** end of file ***/
