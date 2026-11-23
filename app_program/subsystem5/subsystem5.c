/**
 * @file    subsystem5.c
 *  subsystem5.c  (Buddy 5: ranger)
 *  ranger task: owns the HC-SR04 (3.3 V type), the SG90 servo and the
 *  alert LED.  One ping every 60 ms.
 *
 *  Echo timing is done by HARDWARE: GP17 is PWM slice 0 input B, in
 *  "count while B is high" mode at 1 MHz, so the counter holds the echo
 *  width in us.  Encoder interrupt bursts can't shift the reading and
 *  the task just sleeps 40 ms instead of busy-waiting.
 *
 *  Each reading is tagged with the servo angle and is only marked
 *  "valid" if the servo had settled before the ping.
 */
#include "subsystem5.h"

#define SLICE_REG(s)    (PWM_BASE + 0x14U * (uint32_t)(s))
#define CSR_EN          (1U << 0)
#define CSR_LEVEL       (1U << 4)   /* DIVMODE 1: count while B high */
#define DIV_125         (125U << 4) /* 125 MHz / 125 = 1 MHz */
#define FUNCSEL_PWM     4U
#define SERVO_PERIOD_US 20000U

LOCAL volatile int32_t  g_aim_req;
LOCAL volatile BOOL     gb_guard_on;
LOCAL volatile BOOL     gb_fast_on;
LOCAL volatile uint32_t g_last_word = RANGE_FAR_CM | (128U << 8);
LOCAL volatile uint32_t g_no_echo;

/**
 * @brief  Ranger aim.
 */
EXPORT void ranger_aim (int32_t deg)
{
    g_aim_req = CLAMP(deg, -SERVO_MAX_DEG, SERVO_MAX_DEG);
}
/**
 * @brief  Ranger guard.
 */
EXPORT void ranger_guard (BOOL b_on)
{
    gb_guard_on = b_on;
}
/** last close reading the guard counted */
LOCAL volatile int32_t g_guard_last = RANGE_FAR_CM;
EXPORT int32_t         ranger_guard_cm (void)
{
    return g_guard_last;
}
/**
 * @brief  Ranger fast.
 */
EXPORT void ranger_fast (BOOL b_on)
{
    gb_fast_on = b_on;
}
/**
 * @brief  Ranger word.
 * @return Result as described above.
 */
EXPORT uint32_t ranger_word (void)
{
    return g_last_word;
}
/**
 * @brief  Ranger no echo count.
 * @return Result as described above.
 */
EXPORT uint32_t ranger_no_echo_count (void)
{
    return g_no_echo;
}
/**
 * @brief  Ranger led.
 */
EXPORT void ranger_led (BOOL b_on)
{
    gpio_set_val(PIN_ALERT_LED, b_on ? 1 : 0);
}

/** ---------------- drivers ---------------- */
LOCAL void pwm_unreset (void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0 == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 */
LOCAL void us_init (void)
{
    uint32_t base = SLICE_REG(SLICE_US);
    uint32_t pad;

    gpio_set_pin(PIN_US_TRIG, GPIO_MODE_OUT);
    gpio_set_val(PIN_US_TRIG, 0);
    out_w(base + PWM_CHx_CSR, 0);
    out_w(base + PWM_CHx_DIV, DIV_125);
    out_w(base + PWM_CHx_TOP, 0xFFFFU);
    out_w(base + PWM_CHx_CTR, 0);
    out_w(GPIO_CTRL(PIN_US_ECHO), FUNCSEL_PWM); /* echo -> PWM 0 B input */
    pad = in_w(GPIO(PIN_US_ECHO));
    pad |= GPIO_IE | GPIO_OD | GPIO_PDE; /* input only, pulled low */
    pad &= ~GPIO_PUE;
    out_w(GPIO(PIN_US_ECHO), pad);
}

/**
 * @brief  Us trigger.
 */
LOCAL void us_trigger (void)
{
    uint32_t base = SLICE_REG(SLICE_US);
    uint32_t t_start;

    out_w(base + PWM_CHx_CSR, 0);
    out_w(base + PWM_CHx_CTR, 0);
    out_w(base + PWM_CHx_CSR,
          CSR_LEVEL | CSR_EN); /* armed: counts while echo high */
    gpio_set_val(PIN_US_TRIG, 1);
    t_start = motion_now_us();
    while ((uint32_t)(motion_now_us() - t_start) < 12U)
    {
    } /* 12 us trigger pulse */
    gpio_set_val(PIN_US_TRIG, 0);
}

/** echo width in us; 0 = echo never rose (no sensor), 0xFFFF = still high */
LOCAL uint32_t us_read (void)
{
    uint32_t base = SLICE_REG(SLICE_US);
    uint32_t wval = in_w(base + PWM_CHx_CTR) & 0xFFFFU;

    if (gpio_get_val(PIN_US_ECHO))
    {
        return 0xFFFFU;
    }
    out_w(base + PWM_CHx_CSR, 0);
    return wval;
}

/**
 * @brief  Initialise the module (create its kernel objects).
 */
LOCAL void servo_init (void)
{
    uint32_t base = SLICE_REG(SLICE_SERVO);

    out_w(base + PWM_CHx_CSR, 0);
    out_w(base + PWM_CHx_DIV, DIV_125); /* 1 count = 1 us */
    out_w(base + PWM_CHx_CTR, 0);
    pwm_set_wrap(PIN_SERVO, SERVO_PERIOD_US - 1U);
    pwm_set_cc(PIN_SERVO, SERVO_CENTER_US);
    pwm_set_pin(PIN_SERVO);
    pwm_set_enabled(PIN_SERVO, TRUE);
}

/** move, return settle time in ms */
LOCAL uint32_t servo_set (int32_t deg, int32_t from)
{
    int32_t us_val =
        SERVO_CENTER_US + ((SERVO_REVERSE ? -deg : deg) * SERVO_US_PER_DEG);
    int32_t  moved  = ABS(deg - from);
    uint32_t ms_val = (uint32_t)(((moved * SERVO_SETTLE_MS) + 14) / 15);

    pwm_set_cc(PIN_SERVO, (uint32_t)us_val);
    return (ms_val < SERVO_SETTLE_MIN_MS) ? SERVO_SETTLE_MIN_MS : ms_val;
}

/** ---------------- ranger task ---------------- */
LOCAL void ranger_task (int32_t stacd, void * p_exinf)
{
    int32_t  cur        = 0;
    uint32_t settled_at = motion_now_ms() + 500U; /* servo power-up */
    uint32_t seq        = 0;
    uint32_t hits       = 0;
    uint32_t stuck      = 0;
    uint32_t silent     = 0;
    BOOL     b_moved    = TRUE; /* servo moved since the last ping */
#if RANGE_BENCH_MS
    int32_t  bench_cm[32];
    uint32_t bench_n   = 0;
    uint32_t bench_all = 0;
    uint32_t bench_t   = motion_now_ms();
#endif

    (void)stacd;
    (void)p_exinf;

    /* Servo self-test: swing right, left, centre, so you can SEE it work
       (and prove the PWM is set up, whatever the hardware does). */
    {
        uint32_t bval   = SLICE_REG(SLICE_SERVO);
        uint32_t cc_val = in_w(bval + PWM_CHx_CC);
        tm_printf(
            (uint8_t *)"SERVO GP%d: PWM slice %d %s, %d Hz, pulse %d us, pin "
                       "function %d (4 = PWM)\n",
            PIN_SERVO, SLICE_SERVO,
            (in_w(bval + PWM_CHx_CSR) & CSR_EN) ? "on" : "OFF",
            (int32_t)(1000000U / (in_w(bval + PWM_CHx_TOP) + 1U)),
            (int32_t)((PIN_SERVO & 1) ? (cc_val >> 16) : (cc_val & 0xFFFFU)),
            (int32_t)(in_w(GPIO_CTRL(PIN_SERVO)) & 0x1FU));
        tm_printf(
            (uint8_t *)"SERVO self-test: the sensor should turn RIGHT, then "
                       "LEFT, then back to the front (%d deg)\n",
            SERVO_TEST_DEG);
        tk_dly_tsk(300);
        (void)servo_set(-SERVO_TEST_DEG, 0);
        tk_dly_tsk(700);
        (void)servo_set(SERVO_TEST_DEG, -SERVO_TEST_DEG);
        tk_dly_tsk(1000);
        (void)servo_set(0, SERVO_TEST_DEG);
        tk_dly_tsk(700);
        settled_at = motion_now_ms();
    }

    for (;;)
    {
        int32_t  want = g_aim_req;
        uint32_t echo;
        uint32_t tval;
        int32_t  cm_val;
        BOOL     b_valid;
        BOOL     b_fast;

        if (want != cur)
        {
            settled_at = motion_now_ms() + servo_set(want, cur);
            cur        = want;
            b_moved    = TRUE;
        }
        tval = motion_now_ms();
        if ((int32_t)(settled_at - tval) > 0)
        { /* still moving: wait, no ping */
            uint32_t wait = settled_at - tval;
            tk_dly_tsk((wait > 200U) ? 200U : wait);
            continue;
        }

        /* Echo still high from the last ping (some modules hold it up
           to ~200 ms when nothing answers): a trigger now would be
           ignored and the counter would time the old pulse's tail. */
        if (gpio_get_val(PIN_US_ECHO))
        {
            if (++stuck >= 50)
            { /* ~1 s high: sensor fault */
                g_last_word = (uint32_t)RANGE_FAR_CM |
                              ((uint32_t)(cur + 128) << 8) |
                              ((uint32_t)((seq = (seq + 1U) & 0xFFU))
                               << 16); /* invalid */
                stuck = 0;
            }
            tk_dly_tsk(
                gb_fast_on ? 10 : 20); /* poll fast: the sweep waits on this */
            continue;
        }
        stuck = 0;

        b_fast = gb_fast_on;
        us_trigger();
        tk_dly_tsk(b_fast ? SWEEP_ECHO_WAIT_MS : ECHO_WAIT_MS);
        echo = us_read();

        /* 10 pings in a row with no echo pulse at all: sensor missing */
        if (0 == echo)
        {
            if (10 == ++silent)
            {
                comms_fault_raise(FAULT_NO_ECHO);
            }
        }
        else if (silent)
        {
            if (silent >= 10)
            {
                comms_fault_clear(FAULT_NO_ECHO);
            }
            silent = 0;
        }
        else
        {
            /* No action is required in this case. */
        }

        if (0 == echo)
        { /* no echo pulse at all */
            g_no_echo++;
            cm_val  = RANGE_FAR_CM;
            b_valid = FALSE;
        }
        else if (0xFFFFU == echo)
        { /* still high: nothing in range */
            cm_val  = RANGE_FAR_CM;
            b_valid = TRUE;
        }
        else
        {
            cm_val = (int32_t)(echo / US_PER_CM);
            if (cm_val > RANGE_FAR_CM)
            {
                cm_val = RANGE_FAR_CM;
            }
            b_valid = cm_val >= US_MIN_CM;
        }
        b_valid = b_valid && (g_aim_req == cur); /* not if a new aim arrived */
        /* 12 Nov evening: the first ping after a servo move often
           measured the wobble, not the target: throw it away */
        if (b_moved)
        {
            b_valid = FALSE;
            b_moved = FALSE;
        }

        seq         = (seq + 1U) & 0xFFU;
        g_last_word = (uint32_t)cm_val | ((uint32_t)(cur + 128) << 8) |
                      ((uint32_t)seq << 16) | (b_valid ? (1U << 24) : 0U);
        mission_event(EV_PING);

        /* guard: GUARD_HITS close pings out of the last GUARD_WINDOW,
           sensor looking straight ahead.  12 Nov 19:40 / 19:47 runs:
           "N in a row" let every missed echo (common at 3.3 V) reset
           the count, so the car only stopped at 5..9 cm. */
        if (gb_guard_on && (0 == cur))
        {
            BOOL     b_close = b_valid && (cm_val <= GUARD_STOP_CM);
            uint32_t hval;
            uint32_t kdx;
            hits = ((hits << 1) | (b_close ? 1U : 0U)) &
                   ((1U << GUARD_WINDOW) - 1U);
            if (b_close)
            {
                g_guard_last = cm_val;
            }
            kdx = 0;
            for (hval = hits; 0U != hval; hval >>= 1)
            {
                kdx += hval & 1U;
            }
            if (kdx >= GUARD_HITS)
            {
                mission_event(EV_OBSTACLE);
            }
        }
        else
        {
            hits = 0;
        }

#if RANGE_BENCH_MS
        /* bench test: parked (IDLE), sensor straight ahead */
        if ((MS_IDLE == mission_state()) && (0 == cur))
        {
            bench_all++;
            if ((b_valid && (cm_val < RANGE_FAR_CM)) && (bench_n < 32U))
            {
                bench_cm[bench_n++] = cm_val;
            }
            if ((motion_now_ms() - bench_t) >= RANGE_BENCH_MS)
            {
                /* Change Requirement 1: one packed word, a consistent pair */
                uint32_t raw_word = line_raw_word();
                if (bench_n)
                {
                    uint32_t aval;
                    uint32_t bval;
                    for (aval = 1; aval < bench_n;
                         aval++) /* insertion sort, n <= 32 */
                    {
                        for (bval = aval; (bval > 0) &&
                                          (bench_cm[bval - 1] > bench_cm[bval]);
                             bval--)
                        {
                            int32_t xval       = bench_cm[bval];
                            bench_cm[bval]     = bench_cm[bval - 1];
                            bench_cm[bval - 1] = xval;
                        }
                    }
                    tm_printf(
                        (uint8_t *)"RANGE bench: median %d cm, min %d, max %d, "
                                   "spread %d cm (%d of %d pings had an echo)"
                                   " | line raw L %d R %d\n",
                        bench_cm[bench_n / 2], bench_cm[0],
                        bench_cm[bench_n - 1],
                        bench_cm[bench_n - 1] - bench_cm[0], (int32_t)bench_n,
                        (int32_t)bench_all, (int32_t)LINE_WORD_LEFT(raw_word),
                        (int32_t)LINE_WORD_RIGHT(raw_word));
                }
                else
                {
                    tm_printf(
                        (uint8_t *)"RANGE bench: no echo in %d pings (nothing "
                                   "within ~4 m, or sensor/power problem)"
                                   " | line raw L %d R %d\n",
                        (int32_t)bench_all, (int32_t)LINE_WORD_LEFT(raw_word),
                        (int32_t)LINE_WORD_RIGHT(raw_word));
                }
                bench_n   = 0;
                bench_all = 0;
                bench_t   = motion_now_ms();
            }
        }
        else
        {
            bench_n   = 0;
            bench_all = 0;
            bench_t   = motion_now_ms();
        }
#endif

        tk_dly_tsk(b_fast ? SWEEP_PING_PERIOD_MS - SWEEP_ECHO_WAIT_MS
                          : PING_PERIOD_MS - ECHO_WAIT_MS);
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 * @return Result as described above.
 */
EXPORT ER subsystem5_init (void)
{
    pwm_unreset();
    us_init();
    servo_init();
    gpio_set_pin(PIN_ALERT_LED, GPIO_MODE_OUT);
    gpio_set_val(PIN_ALERT_LED, 0);
    return E_OK; /* no kernel objects */
}

/**
 * @brief  Start the module's task(s).
 * @return Result as described above.
 */
EXPORT ER subsystem5_start (void)
{
    ID tid =
        app_main_task_start((FP)ranger_task, PRI_RANGER, CORE_RANGER, "ranger");
    return (tid < E_OK) ? tid : E_OK;
}

#if APP_TEST
/**
 * @brief  On-target self-test of this subsystem (built only when
 *         APP_TEST = 1).
 */
EXPORT void subsystem5_test (void)
{
}
#endif

/*** end of file ***/
