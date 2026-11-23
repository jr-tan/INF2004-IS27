/**
 * @file    subsystem3.c
 *  subsystem3.c  (Buddy 3: line sensors)
 *  line task: woken every 20 ms by ctl_tick_isr (same tick as motion).
 *    1. read both IR sensors (ADC0 = left, ADC1 = right)
 *    2. remove each sensor's white level (learned while stopped)
 *    3. always publish the readings (mission uses them to re-find the
 *       line after an obstacle)
 *    4. when following: line_ctl_step() -> motion slot LINE
 *    5. three junction samples in a row (both black) -> EV_JUNCTION for
 *       the mission (barcode turns), unless barcode bars are passing
 *  The ADC is only touched here.  The barcode reader is barcode.c.
 *  16 Nov, BARCODE_ANALOG: the ADC free-runs round-robin over the analog
 *  pins in use (line L/R + barcode A0), BC_ADC_RATE_HZ samples a second
 *  each, into its FIFO; the FIFO interrupt keeps the newest line readings
 *  for this task and hands every barcode sample to barcode.c.
 */
#include "../car.h"
#include "line_ctl.h"
#include "subsystem3.h" /* own header (BARR-C 4.3.c) */

/* same mode numbers as line_ctl.c */
CC_ASSERT((LINE_M_TRACK == 1) && (LINE_M_CENTRED == 2) &&
              (LINE_M_RECOVER == 3) && (LINE_M_JUNCTION == 4) &&
              (LINE_M_LOST == 5) && (LINE_M_BENDSTOP == 6) &&
              (LINE_M_PIVOT == 7),
          "line mode numbers differ from line_ctl.c");

#define WHITE_CAP   600 /* learned white level never above this */
#define WHITE_ALPHA 8   /* 1/8 per sample while learning */

EXPORT ID g_line_task_id = 0;

/* ---- mission -> line: one word ----
   bit 0 on, bits 1..2 side (0 none, 1 left, 2 right), bit 3 deep,
   bits 8..15 generation (a new command restarts the controller) */
LOCAL volatile uint32_t g_line_cmd;
LOCAL uint32_t          g_cmd_gen;
LOCAL volatile uint32_t g_see_bits;
/* layout found at START: 0 straddle, 1 inside, 2 edge (LINE_EDGE) */
LOCAL volatile uint32_t g_layout = LINE_LAYOUT_DEFAULT;
LOCAL int32_t g_edge_dir         = 1; /** edge: +1 line under the left sensor */
EXPORT BOOL   line_inside (void)
{
    return g_layout != 0;
} /* both black is not a junction */
LOCAL const char * const gp_layout_name[] = {
    "STRADDLE (sensors either side)", "INSIDE (both sensors on the line)",
    "EDGE (one sensor on black, one on white)"};

LOCAL volatile int32_t  g_speed_req = 0; /* 0 = use the tuning's own speed */
LOCAL volatile int32_t  g_bend_req  = LINE_BEND_DEFAULT;
LOCAL volatile int32_t  g_kp_req    = 0;
LOCAL volatile uint32_t g_tune_gen;
LOCAL volatile uint32_t g_tune_seen;
LOCAL volatile int32_t  g_speed_now;
LOCAL volatile int32_t  g_bend_now;
LOCAL volatile int32_t  g_kp_now;
LOCAL volatile BOOL     gb_pivot_req = LINE_PIVOT_DEFAULT;
/**
 * @brief  Line set pivot.
 */
EXPORT void line_set_pivot (BOOL b_on)
{
    gb_pivot_req = b_on ? TRUE : FALSE;
    g_tune_gen++;
}
/**
 * @brief  Line get pivot.
 * @return Result as described above.
 */
EXPORT BOOL line_get_pivot (void)
{
    return gb_pivot_req;
}
/**
 * @brief  Line set speed.
 */
EXPORT void line_set_speed (int32_t mms)
{
    g_speed_req = CLAMP(mms, LINE_SPEED_MIN, LINE_SPEED_MAX);
    g_tune_gen++;
}
/**
 * @brief  Line get speed.
 * @return Result as described above.
 */
EXPORT int32_t line_get_speed (void)
{
    return g_speed_now;
}
/**
 * @brief  Line set tuning.
 */
EXPORT void line_set_tuning (int32_t mms, int32_t bend, int32_t kp_val)
{
    if (mms)
    {
        g_speed_req = CLAMP(mms, LINE_SPEED_MIN, LINE_SPEED_MAX);
    }
    if (bend)
    {
        g_bend_req = CLAMP(bend, 20, 100);
    }
    if (kp_val)
    {
        g_kp_req = CLAMP(kp_val, 300, 3000);
    }
    g_tune_gen++;
}
/**
 * @brief  Line get tuning.
 */
EXPORT void line_get_tuning (int32_t * p_mms, int32_t * p_bend,
                             int32_t * p_kp_val)
{
    *p_mms    = g_speed_now;
    *p_bend   = g_bend_now;
    *p_kp_val = g_kp_now;
}

/* working copies of the three tunings, so the speed can change live */
LOCAL line_tune_t g_tune_straddle;
LOCAL line_tune_t g_tune_inside;
LOCAL line_tune_t g_tune_edge;
/**
 * @brief  Tune of.
 * @return Result as described above.
 */
LOCAL line_tune_t * tune_of (uint32_t lay)
{
    return (LINE_EDGE == lay) ? &g_tune_edge
           : (1 == lay)       ? &g_tune_inside
                              : &g_tune_straddle;
}
/**
 * @brief  Apply speed.
 */
LOCAL void apply_speed (void)
{
    int32_t sp_val = g_speed_req ? g_speed_req : LINE_SPEED_DEFAULT;
    /* bend speed: % of the line speed, but never below LINE_BEND_FLOOR
       (the motors may stall) and never above the line speed */
    int32_t  lo_val   = (sp_val < LINE_BEND_FLOOR) ? sp_val : LINE_BEND_FLOOR;
    int32_t  bend_mms = CLAMP(sp_val * g_bend_req / 100, lo_val, sp_val);
    uint32_t idx;

    for (idx = 0; idx < 3; idx++)
    {
        line_tune_t * p_key = tune_of(idx);
        p_key->speed        = sp_val;
        p_key->speed_min    = bend_mms;
        if (g_kp_req)
        {
            p_key->kp_x1000 = g_kp_req;
        }
    }
    /* bends: stop + pivot on the inner wheel, or steer through */
    g_tune_inside.pivot_steer   = gb_pivot_req ? PIVOT_STEER_INSIDE : 0;
    g_tune_straddle.pivot_steer = gb_pivot_req ? PIVOT_STEER_STRADDLE : 0;
    g_tune_edge.pivot_steer     = gb_pivot_req ? PIVOT_STEER_EDGE : 0;
    g_tune_edge.edge_dir        = g_edge_dir;
    g_speed_now                 = sp_val;
    g_bend_now                  = g_bend_req;
    g_kp_now                    = tune_of(g_layout)->kp_x1000;
}
LOCAL volatile uint32_t g_loop_max;
/**
 * @brief  Line loop us max.
 * @return Result as described above.
 */
EXPORT uint32_t line_loop_us_max (void)
{
    uint32_t val = g_loop_max;
    g_loop_max   = 0;
    return val;
}
LOCAL line_status_t g_ls_val;

/**
 * @brief  Line follow.
 */
EXPORT void line_follow (BOOL b_on, int32_t side, BOOL b_deep)
{
    uint32_t sval = (side < 0) ? 1U : (side > 0) ? 2U : 0U;
    g_cmd_gen     = (g_cmd_gen + 1U) & 0xFFU; /* only the mission calls this */
    g_line_cmd    = (b_on ? 1U : 0U) | (sval << 1) | (b_deep ? 8U : 0U) |
                 ((uint32_t)g_cmd_gen << 8);
}

/**
 * @brief  Line get.
 */
EXPORT void line_get (line_status_t * p_state)
{
    *p_state = g_ls_val;
} /** debug copy */
EXPORT uint32_t line_see_bits (void)
{
    return g_see_bits;
}

/* Change Requirement 1: packed readings (LINE_WORD_* in car.h).  Only
   the line task writes them, each with one aligned 32-bit store. */
LOCAL volatile uint32_t g_line_raw_word;
LOCAL volatile uint32_t g_line_dark_word;
#define LINE_WORD_MASK12      0xFFFU
#define LINE_WORD_SHIFT_RIGHT 12U
#define LINE_WORD_SHIFT_SEQ   24U
#define LINE_WORD_MASK_SEQ    0xFFU

/**
 * @brief  Pack a left/right reading pair and a sequence count.
 * @param  left   Left reading, 0..4095.
 * @param  right  Right reading, 0..4095.
 * @param  seq    Sequence count (low 8 bits used).
 * @return The packed word (layout: LINE_WORD_* in car.h).
 */
LOCAL uint32_t line_word_pack (uint32_t left, uint32_t right, uint32_t seq)
{
    return (left & LINE_WORD_MASK12) |
           ((right & LINE_WORD_MASK12) << LINE_WORD_SHIFT_RIGHT) |
           ((seq & LINE_WORD_MASK_SEQ) << LINE_WORD_SHIFT_SEQ);
}

/**
 * @brief  Latest raw (ADC) readings, packed.
 * @return Packed word; LINE_WORD_LEFT/RIGHT/SEQ in car.h unpack it.
 */
EXPORT uint32_t line_raw_word (void)
{
    return g_line_raw_word;
}

/**
 * @brief  Latest readings with the white level removed, packed.
 * @return Packed word; LINE_WORD_LEFT/RIGHT/SEQ in car.h unpack it.
 */
EXPORT uint32_t line_dark_word (void)
{
    return g_line_dark_word;
}
/* climb mode (mission, on a slope or when stalled): no pivoting, both
   wheels always forward, a bit faster for momentum */
LOCAL volatile BOOL gb_climb_on;
/**
 * @brief  Line climb.
 */
EXPORT void line_climb (BOOL b_on)
{
    gb_climb_on = b_on;
}

/* ---------------- ADC (direct registers) ---------------- */
#define ADC_CS          (0x4004C000U + 0x00U)
#define ADC_RESULT      (0x4004C000U + 0x04U)
#define ADC_EN          (1U << 0)
#define ADC_START_ONCE  (1U << 2)
#define ADC_READY       (1U << 8)
#define ADC_AINSEL(ch)  ((uint32_t)(ch) << 12)
#define ADC_RESULT_MASK 0xFFFU /* 12-bit result */
#define ADC_FULL_SCALE  4095U  /* largest 12-bit result */
#define ADC_SPIN_MAX    2000U  /* busy-wait limit, ~2 us is normal */
#define ADC_PIN0        26U    /* GP26 = ADC channel 0 */
#ifndef FUNCSEL_NULL
#define FUNCSEL_NULL 31U
#endif

/**
 * @brief  Analog pin.
 */
LOCAL void analog_pin (uint32_t pin)
{
    out_w(GPIO_CTRL(pin), FUNCSEL_NULL); /* no digital function */
    clr_w(GPIO(pin),
          GPIO_IE | GPIO_PUE | GPIO_PDE); /* no input buffer, no pulls */
    set_w(GPIO(pin), GPIO_OD);
}

/* ---------------- ADC scan (BARCODE_ANALOG) ---------------- */
#define ADC_FCS             (0x4004C000U + 0x08U)
#define ADC_FIFO            (0x4004C000U + 0x0CU)
#define ADC_DIV             (0x4004C000U + 0x10U)
#define ADC_INTE            (0x4004C000U + 0x18U)
#define ADC_START_MANY      (1U << 3)
#define ADC_RROBIN(m)       ((uint32_t)(m) << 16)
#define FCS_EN              (1U << 0)
#define FCS_UNDER           (1U << 10)
#define FCS_OVER            (1U << 11)
#define FCS_LEVEL(f)        (((f) >> 16) & 0xFU)
#define FCS_THRESH(n)       ((uint32_t)(n) << 24)
#define ADC_IRQ_NO          22        /* ADC_IRQ_FIFO */
#define ADC_IRQ_PRI         2         /* same as the GPIO edges */
#define ADC_CLK_HZ          48000000U /* CLK_ADC_FREQ */
#define ADC_CHANNELS        4U        /* RP2040 ADC inputs 0..3 */
#define ADC_GPIO_CHANNELS   3U        /* inputs 0..2 are GP26..GP28 */
#define ADC_PER_CH_HZ       ((uint32_t)BC_ADC_RATE_HZ) /* per channel */
#define US_PER_S            1000000U
#define ADC_SCAN_WAIT_TRIES 200U /* line task waits up to 200 x 10 ms */
#define ADC_SCAN_WAIT_MS    10

LOCAL volatile uint16_t g_adc_last[ADC_CHANNELS]; /* newest raw sample */
LOCAL volatile BOOL     gb_adc_scan_on; /* line task reads g_adc_last[] */
LOCAL volatile BOOL     gb_adc_single; /* line task fell back to single reads */
LOCAL uint8_t           g_adc_ch[ADC_CHANNELS]; /* round-robin order */
LOCAL uint32_t          g_adc_nch;
LOCAL uint32_t          g_adc_idx;
LOCAL uint32_t          g_adc_conv_us; /* time between two conversions */
LOCAL volatile uint32_t g_adc_resync;  /* FIFO over/underflows (stays 0) */

/**
 * @brief  (Re)start the round-robin scan from the first channel, with
 *         an empty FIFO and its error flags cleared.
 */
LOCAL void adc_scan_restart (void)
{
    uint32_t spin = 0U;

    clr_w(ADC_CS, ADC_START_MANY);
    while ((0U == (in_w(ADC_CS) & ADC_READY)) && (spin < ADC_SPIN_MAX))
    {
        spin++; /* let the conversion in progress finish */
    }
    while (0U != FCS_LEVEL(in_w(ADC_FCS)))
    {
        (void)in_w(ADC_FIFO); /* drain the FIFO */
    }
    out_w(ADC_FCS, FCS_EN | FCS_THRESH(1) | FCS_UNDER |
                       FCS_OVER); /* write 1 to clear the errors */
    g_adc_idx = 0U;
    out_w(ADC_CS, ADC_EN | ADC_AINSEL(g_adc_ch[0]) |
                      ADC_RROBIN((1U << (PIN_LINE_L - ADC_PIN0)) |
                                 (1U << (PIN_LINE_R - ADC_PIN0)) |
                                 (1U << (PIN_BARCODE_ADC - ADC_PIN0))) |
                      ADC_START_MANY);
}

/**
 * @brief  ADC FIFO interrupt (core 0): keep the newest sample of each
 *         channel and pass every barcode sample, with its time, on.
 * @param  intno  Interrupt number; not used.
 */
LOCAL void adc_isr (uint32_t intno)
{
    uint32_t now = in_w(TIMER_TIMERAWL);
    uint32_t fcs = in_w(ADC_FCS);
    uint32_t num;
    uint32_t left;

    (void)intno;
    if (0U != (fcs & (FCS_OVER | FCS_UNDER)))
    {
        /* lost our place in the rotation: start it again */
        g_adc_resync++;
        adc_scan_restart();
    }
    else
    {
        num = FCS_LEVEL(fcs);
        for (left = num; left > 0U; left--) /* oldest first */
        {
            uint32_t val = in_w(ADC_FIFO) & ADC_RESULT_MASK;
            uint32_t chn = g_adc_ch[g_adc_idx];

            g_adc_idx = ((g_adc_idx + 1U) < g_adc_nch) ? (g_adc_idx + 1U) : 0U;
            g_adc_last[chn] = (uint16_t)val; /* 12 bits: fits */
            if ((PIN_BARCODE_ADC - ADC_PIN0) == chn)
            {
                barcode_adc_sample(val, now - ((left - 1U) * g_adc_conv_us));
            }
        }
    }
}

/**
 * @brief  Start the ADC scan (core 0, the barcode task).
 * @return TRUE if it started; FALSE if BARCODE_ANALOG is 0, the line
 *         task already fell back to single reads, or the interrupt could
 *         not be attached.
 */
EXPORT BOOL line_adc_scan_start (void)
{
    BOOL b_started = FALSE;
#if BARCODE_ANALOG
    T_DINT   dint;
    uint32_t chn;
    uint32_t total;

    if (!gb_adc_single)
    {
        analog_pin(PIN_LINE_L);
        analog_pin(PIN_LINE_R);
        analog_pin(PIN_BARCODE_ADC);

        /* the hardware visits the channels in rising order */
        g_adc_nch = 0U;
        for (chn = 0U; chn < ADC_GPIO_CHANNELS; chn++)
        {
            if (((PIN_LINE_L - ADC_PIN0) == chn) ||
                ((PIN_LINE_R - ADC_PIN0) == chn) ||
                ((PIN_BARCODE_ADC - ADC_PIN0) == chn))
            {
                g_adc_ch[g_adc_nch] = (uint8_t)chn; /* 0..2: fits */
                g_adc_nch++;
            }
        }
        total         = ADC_PER_CH_HZ * g_adc_nch;
        g_adc_conv_us = US_PER_S / total;
        out_w(ADC_INTE, 0);
        out_w(ADC_CS, ADC_EN);
        /* integer part of the clock divider, no fraction */
        out_w(ADC_DIV, ((ADC_CLK_HZ / total) - 1U) << 8);

        /* NOTE: replaces the kernel's unused "adca" driver handler. */
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)adc_isr; /* kernel handler type */
        if (E_OK <= tk_def_int(ADC_IRQ_NO, &dint))
        {
            ClearInt(ADC_IRQ_NO);
            adc_scan_restart();
            out_w(ADC_INTE, 1U); /* FIFO level >= threshold */
            EnableInt(ADC_IRQ_NO, ADC_IRQ_PRI);
            gb_adc_scan_on = TRUE;
            b_started      = TRUE;
        }
    }
#endif
    return b_started;
}

/**
 * @brief  Adc read.
 * @return Result as described above.
 */
LOCAL uint32_t adc_read (uint32_t chr)
{
    uint32_t spin   = 0U;
    uint32_t val    = 0U;
    uint32_t result = 0U; /* 0 if the conversion never finishes */

    if (gb_adc_scan_on)
    {
        /* scan running: take its newest sample */
        val    = g_adc_last[chr];
        result = LINE_BLACK_IS_HIGH ? val : (ADC_FULL_SCALE - val);
    }
    else
    {
        out_w(ADC_CS, ADC_EN | ADC_AINSEL(chr) | ADC_START_ONCE);
        while ((0U == (in_w(ADC_CS) & ADC_READY)) && (spin <= ADC_SPIN_MAX))
        {
            spin++; /* ~2 us normally */
        }
        if (spin <= ADC_SPIN_MAX)
        {
            val    = in_w(ADC_RESULT) & ADC_RESULT_MASK;
            result = LINE_BLACK_IS_HIGH ? val : (ADC_FULL_SCALE - val);
        }
    }
    return result;
}

/** ---------------- line task ---------------- */
LOCAL void line_task (int32_t stacd, void * p_exinf)
{
    line_ctl_t ctl;
    line_out_t out      = {0};
    uint32_t   white[2] = {WHITE_CAP / 2, WHITE_CAP / 2};
    uint32_t   seen_gen = 0xFFFFU;
    uint32_t   junc_n   = 0;
    uint32_t   t_wake;
    BOOL       b_on     = FALSE;
    uint32_t   line_seq = 0U; /* sequence count of the packed readings */
    uint32_t   idx;

    (void)stacd;
    (void)p_exinf;
#if BARCODE_ANALOG
    /* The barcode task (core 0) starts the ADC scan; never touch the ADC
       registers while it may be doing so.  Give it 2 s, then fall back
       to single reads. */
    for (idx = 0U; (idx < ADC_SCAN_WAIT_TRIES) && (!gb_adc_scan_on); idx++)
    {
        tk_dly_tsk(ADC_SCAN_WAIT_MS);
    }
    if (!gb_adc_scan_on)
    {
        gb_adc_single = TRUE;
        tm_printf((uint8_t *)"LINE: ADC scan not started, single reads "
                             "(barcode A0 off)\n");
    }
#endif
    if (!gb_adc_scan_on)
    {
        analog_pin(PIN_LINE_L);
        analog_pin(PIN_LINE_R);
        set_w(ADC_CS, ADC_EN);
    }
    g_tune_straddle = g_line_tune_default;
    g_tune_inside   = g_line_tune_inside;
    g_tune_edge     = g_line_tune_edge;
    apply_speed();
    line_ctl_reset(&ctl, &g_tune_straddle, 0, 0, motion_now_ms());

    for (;;)
    {
        /* UINT (T-Kernel, 32-bit) kept: see docs/barr_c_conformance.md, D-5. */
        UINT     raw[2];
        UINT     dark[2];
        uint32_t cmd;

        tk_slp_tsk(TMO_FEVR); /* ctl_tick_isr wakes us every 20 ms */
        t_wake = motion_now_us();
        tk_can_wup(TSK_SELF); /* drop any backlog: no catch-up bursts */

        raw[0] = adc_read(PIN_LINE_L - 26);
        raw[1] = adc_read(PIN_LINE_R - 26);

        /* command from the mission */
        cmd = g_line_cmd;
        if (((cmd >> 8) & 0xFFU) != seen_gen)
        {
            uint32_t sval = (cmd >> 1) & 3U;
            seen_gen      = (cmd >> 8) & 0xFFU;
            if (b_on && (!(cmd & 1U)))
            {
                motion_drive(MOT_OWNER_LINE, 0,
                             0); /* no stale command left in the slot */
            }
            b_on     = (cmd & 1U) != 0;
            out.lost = 0;
            /* START (no seed side): look where the sensors are now.
               Both black = INSIDE layout; both white = STRADDLE;
               one black + one white = EDGE (follow the line's edge).
               Restarts after an obstacle or a barcode turn keep the
               layout found at START. */
            if ((cmd & 1U) && (0 == sval))
            {
                uint32_t see0 = ((raw[0] >= 2000U) ? 1U : 0U) |
                                ((raw[1] >= 2000U) ? 2U : 0U);
                uint32_t was = g_layout;
                if (3 == see0)
                {
                    g_layout = 1;
                }
                else if (0 == see0)
                {
                    g_layout = 0;
                }
                else
                {
                    g_layout   = LINE_EDGE;
                    g_edge_dir = (1 == see0) ? 1 : -1;
                }
                g_tune_edge.edge_dir = g_edge_dir;
                tm_printf((uint8_t *)"LINE layout: %s%s%s\n",
                          gp_layout_name[g_layout],
                          (LINE_EDGE == g_layout)
                              ? ((g_edge_dir > 0)
                                     ? ", line under the LEFT sensor"
                                     : ", line under the RIGHT sensor")
                              : "",
                          (was != g_layout) ? " [changed]" : "");
            }
            g_kp_now = tune_of(g_layout)->kp_x1000;
            line_ctl_reset(&ctl, tune_of(g_layout),
                           (1 == sval)   ? -1
                           : (2 == sval) ? 1
                                         : 0,
                           (cmd & 8U) != 0, motion_now_ms());
        }

        /* white level: learned only while NOT following (car parked
           on white before START), so a line under a sensor during the
           run can't be learned as "white" */
        for (idx = 0; idx < 2; idx++)
        {
            if (((!b_on) && (raw[idx] < WHITE_CAP)) &&
                (!((1 == g_layout) && (raw[idx ^ 1U] >= 2000U))))
            {
                white[idx] =
                    ((white[idx] * (WHITE_ALPHA - 1)) + raw[idx]) / WHITE_ALPHA;
            }
            dark[idx] =
                (raw[idx] > white[idx])
                    ? ((raw[idx] - white[idx]) * 4095U) / (4095U - white[idx])
                    : 0U;
        }
        g_see_bits =
            ((dark[0] >= (uint32_t)g_line_tune_default.see) ? 1U : 0U) |
            ((dark[1] >= (uint32_t)g_line_tune_default.see) ? 2U : 0U);

        /* Change Requirement 1: publish each reading pair with ONE store;
           both words carry the same sequence count. */
        line_seq         = (line_seq + 1U) & LINE_WORD_MASK_SEQ;
        g_line_raw_word  = line_word_pack(raw[0], raw[1], line_seq);
        g_line_dark_word = line_word_pack(dark[0], dark[1], line_seq);

        if (g_tune_gen != g_tune_seen)
        {
            g_tune_seen = g_tune_gen;
            apply_speed();
            tm_printf((uint8_t *)"LINE speed %d mm/s, bends %d mm/s (%d %%), "
                                 "steering KP %d, bends: %s\n",
                      g_speed_now, g_tune_edge.speed_min, g_bend_now, g_kp_now,
                      gb_pivot_req ? "stop + pivot on the inner wheel"
                                   : "steer through");
        }
        if (b_on)
        {
            line_ctl_step(&ctl, (int32_t)dark[0], (int32_t)dark[1],
                          motion_now_ms(), &out);
            if (out.lost)
            {
                b_on = FALSE; /* stop driving the slot */
                motion_drive(MOT_OWNER_LINE, 0, 0);
                mission_event(EV_LINE_LOST);
            }
            else if ((barcode_busy()) && (!gb_climb_on))
            {
                /* 12 Nov evening: bars passing under the barcode sensor:
                   steady speed, gentle steering (no pivot / bend stop),
                   so the zig-zag doesn't stretch a narrow bar into a
                   wide one */
                motion_drive(
                    MOT_OWNER_LINE, g_speed_now,
                    CLAMP(out.steer, -LINE_CLIMB_STEER, LINE_CLIMB_STEER));
            }
            else if (gb_climb_on)
            {
                /* |steer| LINE_CLIMB_STEER keeps the inner wheel at
                   (1 - 2 x steer/1000) of the outer one, never stopped */
                motion_drive(
                    MOT_OWNER_LINE, LINE_CLIMB_MMS,
                    CLAMP(out.steer, -LINE_CLIMB_STEER, LINE_CLIMB_STEER));
            }
            else
            {
                motion_drive(MOT_OWNER_LINE, out.speed, out.steer);
            }
            if (LINE_M_JUNCTION == out.mode)
            {
                if ((3 == ++junc_n) && (!barcode_busy()))
                {
                    mission_event(EV_JUNCTION);
                }
            }
            else
            {
                junc_n = 0;
            }
        }
        else
        {
            out.mode  = LINE_M_OFF;
            out.steer = out.speed = 0;
            out.pos = (((int32_t)dark[1] - (int32_t)dark[0]) * 1000) / 4095;
        }

        for (idx = 0; idx < 2; idx++)
        {
            g_ls_val.raw[idx]  = raw[idx];
            g_ls_val.dark[idx] = dark[idx];
            g_ls_val.see[idx]  = (g_see_bits >> idx) & 1U;
        }
        g_ls_val.pos       = out.pos;
        g_ls_val.steer     = out.steer;
        g_ls_val.speed     = out.speed;
        g_ls_val.set_speed = g_speed_now;
        g_ls_val.mode =
            b_on ? (uint32_t)out.mode : ((out.lost) ? LINE_M_LOST : LINE_M_OFF);
        g_ls_val.side = ctl.side;
        {
            uint32_t took = motion_now_us() - t_wake;
            if (took > g_loop_max)
            {
                g_loop_max = took;
            }
        }
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 * @return Result as described above.
 */
EXPORT ER subsystem3_init (void)
{
    return E_OK;
} /** no kernel objects */

EXPORT ER subsystem3_start (void)
{
    ID tid = app_main_task_start((FP)line_task, PRI_LINE, CORE_LINE, "line");
    if (tid < E_OK)
    {
        return tid;
    }
    g_line_task_id = tid;
    return barcode_start();
}

#if APP_TEST
/**
 * @brief  On-target self-test of this subsystem (built only when
 *         APP_TEST = 1).
 */
EXPORT void subsystem3_test (void)
{
}
#endif

/*** end of file ***/
