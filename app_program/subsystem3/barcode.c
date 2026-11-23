/**
 * @file    barcode.c
 *  barcode.c  (Buddy 3: Code 39 Full ASCII barcodes)
 *
 *  Two ways in, chosen by BARCODE_ANALOG (car_config.h):
 *    0: ISR (shared GPIO dispatcher, core 0) stamps every edge of the
 *       barcode sensor's D0 (GP2) into a ring and wakes the barcode task.
 *    1: the ADC interrupt (subsystem3.c, core 0) hands every A0 sample to
 *       barcode_adc_sample(), which finds the bars as peaks and the gaps
 *       as dips and puts the edges into the same ring.
 *  Ring: the producer owns 'head', the task owns 'tail' (one writer each).
 *
 *  barcode task (priority 15, core 0):
 *    DEBOUNCE a burst of edges closer than BC_DEBOUNCE_US becomes one edge
 *    WAIT     for a white -> black edge
 *    MEASURE  collect edges; drop glitches shorter than BC_MIN_EL_US
 *    DECODE   once the sensor has seen white for BC_END_GAP_MS and for
 *             2 x the widest element: widths -> code39_decode()
 *  A frame that runs longer than BC_MAX_FRAME_MS, or an element longer
 *  than BC_MAX_EL_MS, is not a barcode (a junction) and is thrown away.
 *
 *  Action of the first character: from the letter -> action map, set by
 *  the GUI over MQTT (robot/map), BC_ACTION_MAP until one arrives.
 */
#include "../car.h"
#include "../gpio_irq.h"
#include "code39.h"
#include "barcode.h" /* own header (BARR-C 4.3.c) */

#define BC_RING      128       /* power of two */
#define BC_MAX_EDGES (10 * 10) /* up to 10 characters incl. the two '*' */
#define BC_MIN_EL_US 300       /* shorter = noise */
/* 16 Nov: was 40.  The frame ended after 2 x the widest element so far,
   which right after the start (widest = one space) cut the barcode off
   whenever the car or a hand slowed down for one narrow space (bench log:
   bar 123, space 205, bar 121 ms, then given up).  A barcode is followed
   by a white quiet zone of 10 narrow widths, so waiting 0.8 s is safe. */
#define BC_END_GAP_MS 800
/* 12 Nov: sized for line speeds down to 25 mm/s.  Before the timer fix
   (kernel cpu_clock.c) every ms here was really ~43 ms, so the old
   400 / 2500 never limited anything.
   12 Nov evening: the course barcodes go up to ~333 mm long with 21 mm
   wide bars (BARCODE.pptx, largest size).  At 25 mm/s that is a 856 ms
   bar and a 13.3 s barcode.  The sensor sits BESIDE the line, so it never
   sees the main line itself. */
#define BC_MAX_EL_MS    1500
#define BC_MAX_FRAME_MS 16000
#define BC_RATIO_X10    15  /* wide must be >= 1.5 x narrow */
#define BC_BUSY_HOLD_MS 400 /* "busy" this long after the last edge */
/* 16 Nov: sensor check on the console.  1 = once a second, if the sensor
   changed at all: how many black/white changes and black bars it saw;
   every real change; and a line for every barcode attempt that is thrown
   away, with why.  Debug output only, so it follows NDEBUG (BARR-C
   2.1.c.ii): define NDEBUG in the build to switch it off. */
#ifndef NDEBUG
#define BC_DIAG 1
#else
#define BC_DIAG 0
#endif
#define BC_DIAG_MS 1000
/* 16 Nov: the sensor's comparator chatters near its threshold (bench log:
   300-600 changes a second, a barcode has ~30).  A change only counts once
   the sensor has held the new colour this long; a burst of chatter becomes
   one edge in the middle of the burst, a burst that ends on the old colour
   is dropped.  10 ms is still under half the narrowest bar at 250 mm/s
   (5 mm bar = 20 ms). */
#define BC_DEBOUNCE_US   10000U
#define US_PER_MS        1000U
#define BC_TASK_SLEEP_MS 20 /* task period when no burst is settling */
/* failed attempts with fewer elements are specks, not barcodes */
#define BC_MIN_REPORT_ELEMENTS ((0 != BC_DIAG) ? 3 : 9)
#define BC_MAX_WIDTHS_SHOWN    60        /* widths printed for a failed read */
#define BC_PADS_SCHMITT        (1U << 1) /* PADS_BANK0 GPIOx: SCHMITT */

typedef struct
{
    uint32_t t;
    uint8_t  black;
} edge_t;

LOCAL volatile edge_t   g_ring[BC_RING];
LOCAL volatile uint32_t g_ring_head;
LOCAL volatile uint32_t g_ring_tail;
LOCAL volatile uint32_t g_ring_lost;
LOCAL ID                g_bc_tid;
LOCAL barcode_info_t    g_info; /* written by the barcode task only */
LOCAL volatile uint32_t g_last_edge_ms;
LOCAL volatile BOOL     gb_in_frame;

/**
 * @brief  Copy the latest barcode result.
 * @param  p_buf  Output; a copy that may mix one update.
 */
EXPORT void barcode_get (barcode_info_t * p_buf)
{
    *p_buf = g_info;
}

/**
 * @brief  Are bars passing the sensor right now (or did just now)?
 * @return TRUE while a frame is open or for BC_BUSY_HOLD_MS after an edge.
 */
EXPORT BOOL barcode_busy (void)
{
    return gb_in_frame ||
           ((motion_now_ms() - g_last_edge_ms) < BC_BUSY_HOLD_MS);
}

/**
 * @brief  Put one edge into the ring and wake the barcode task.  Called
 *         by the producer only (D0 interrupt or A0 sample handler).
 * @param  t_us     Time of the edge, us.
 * @param  b_black  1 = the sensor now sees black.
 */
LOCAL void ring_put (uint32_t t_us, uint8_t b_black)
{
    uint32_t hval      = g_ring_head;
    uint32_t next_head = (hval + 1U) & (BC_RING - 1U);

    if (next_head == g_ring_tail)
    {
        g_ring_lost++;
    }
    else
    {
        g_ring[hval].t     = t_us;
        g_ring[hval].black = b_black;
        g_ring_head        = next_head;
        if (g_bc_tid > 0)
        {
            tk_wup_tsk(g_bc_tid);
        }
    }
}

/**
 * @brief  D0 edge interrupt (shared GPIO dispatcher, core 0).
 * @param  gpio    Pin that changed.
 * @param  events  Edge events; not used.
 * @param  tval    Dispatcher time stamp, us.
 */
LOCAL void bc_isr (uint32_t gpio, uint32_t events, uint32_t tval)
{
    BOOL b_level = gpio_get_val(gpio) != 0;

    (void)events;
    ring_put(tval, (b_level == (BARCODE_BLACK_IS_HIGH != 0)) ? 1U : 0U);
}

/* ---- BARCODE_ANALOG: A0 samples -> edges (ADC interrupt, core 0) ----
   A half-seen narrow bar never gets as dark as a wide one, so a fixed
   black/white level misses it (simulated: 0 of 15 reads with a 2.5 mm
   blur).  Instead each bar is found as a peak and each gap as a dip:
     - average BC_ADC_AVG samples (4 ms) against noise
     - learn the white..black range (hi/lo jump to new extremes, drift
       together over ~2 s) and the noise (average sample-to-sample step)
     - BLACK once the signal has risen STEP above its lowest point since
       the last switch, WHITE once it has fallen STEP below its highest
       point; STEP = a fifth of the range, at least 3 x the noise
   Nothing switches until the range is BC_ADC_MIN_SPAN and twice STEP.
   Levels are kept x256 (fixed point) so slow drift still moves them. */
#define BC_ADC_AVG        4
#define BC_ADC_SCALE      256   /* fixed point: levels and noise x256 */
#define BC_ADC_NOISE_DIV  64    /* noise average over ~64 samples */
#define BC_ADC_DRIFT_DIV  2048  /* levels drift together over ~2 s */
#define BC_ADC_STEP_DIV   5     /* STEP = range / 5 ... */
#define BC_ADC_STEP_NOISE 3     /* ... at least 3 x the noise ... */
#define BC_ADC_STEP_MIN   40    /* ... and at least this */
#define ADC_FULL_SCALE_A0 4095U /* largest 12-bit sample */
#define US_PER_S_A0       1000000U

LOCAL uint16_t g_adc_hist[BC_ADC_AVG];
LOCAL uint32_t g_adc_hist_num;
LOCAL uint32_t g_adc_hist_idx;
LOCAL int32_t  g_adc_prev = -1;       /* -1 = no sample yet */
LOCAL int32_t  g_adc_noise8;          /* mean |step|, x256 */
LOCAL int32_t  g_adc_black8 = -1;     /* black level, x256; -1 = not learnt */
LOCAL int32_t  g_adc_white8;          /* white level, x256 */
LOCAL int32_t  g_adc_extreme;         /* lowest (white) / highest (black) since
                                         the last switch */
LOCAL uint8_t           gb_adc_black; /* current colour from A0 */
LOCAL volatile uint16_t g_adc_now;    /* BC_DIAG: latest average */
LOCAL volatile uint16_t g_adc_min;    /* BC_DIAG: this second's range */
LOCAL volatile uint16_t g_adc_max;
LOCAL volatile uint16_t g_adc_step; /* BC_DIAG: current STEP */

/**
 * @brief  Divide by a power of two, rounding towards minus infinity: the
 *         same result as an arithmetic right shift, without using a
 *         bitwise operator on signed data (BARR-C 5.3.b).
 * @param  num  Value to divide.
 * @param  den  Positive divisor.
 * @return floor(num / den).
 */
LOCAL int32_t floor_div (int32_t num, int32_t den)
{
    int32_t quot = num / den;

    if (((num % den) != 0) && (num < 0))
    {
        quot--; /* C division rounds towards zero */
    }
    return quot;
}

/**
 * @brief  One A0 sample (ADC interrupt, core 0): track the levels and put
 *         an edge into the ring when the colour changes.
 * @param  raw   12-bit ADC sample.
 * @param  t_us  Time of the sample, us.
 */
EXPORT void barcode_adc_sample (uint32_t raw, uint32_t t_us)
{
    /* 12-bit sample: fits in int32_t */
    int32_t val =
        (int32_t)(BARCODE_A0_BLACK_IS_HIGH ? raw : (ADC_FULL_SCALE_A0 - raw));
    int32_t  avg;
    int32_t  span;
    int32_t  step;
    int32_t  noise;
    uint32_t idx;
    uint32_t sum = 0U;

    /* Step 1: noise = running mean of |sample - previous sample|. */
    if (g_adc_prev >= 0)
    {
        int32_t diff = val - g_adc_prev;
        int32_t mag  = (diff < 0) ? -diff : diff;

        g_adc_noise8 +=
            floor_div((mag * BC_ADC_SCALE) - g_adc_noise8, BC_ADC_NOISE_DIV);
    }
    g_adc_prev = val;

    /* Step 2: average the last BC_ADC_AVG samples. */
    g_adc_hist[g_adc_hist_idx] = (uint16_t)val; /* 0..4095: fits */
    g_adc_hist_idx             = (g_adc_hist_idx + 1U) % BC_ADC_AVG;
    if (g_adc_hist_num < BC_ADC_AVG)
    {
        g_adc_hist_num++;
    }
    for (idx = 0U; idx < g_adc_hist_num; idx++)
    {
        sum += g_adc_hist[idx];
    }
    avg = (int32_t)(sum / g_adc_hist_num); /* <= 4095: fits */

    g_adc_now = (uint16_t)avg;
    if (avg < (int32_t)g_adc_min)
    {
        g_adc_min = (uint16_t)avg;
    }
    if (avg > (int32_t)g_adc_max)
    {
        g_adc_max = (uint16_t)avg;
    }

    /* Step 3: learn the black and white levels and the switching STEP. */
    if (g_adc_black8 < 0)
    {
        g_adc_black8  = avg * BC_ADC_SCALE; /* first sample */
        g_adc_white8  = avg * BC_ADC_SCALE;
        g_adc_extreme = avg;
    }
    else
    {
        if ((avg * BC_ADC_SCALE) > g_adc_black8)
        {
            g_adc_black8 = avg * BC_ADC_SCALE;
        }
        if ((avg * BC_ADC_SCALE) < g_adc_white8)
        {
            g_adc_white8 = avg * BC_ADC_SCALE;
        }
        span = g_adc_black8 - g_adc_white8; /* >= 0 */
        g_adc_black8 -= span / BC_ADC_DRIFT_DIV;
        g_adc_white8 += span / BC_ADC_DRIFT_DIV;
        span  = (g_adc_black8 - g_adc_white8) / BC_ADC_SCALE;
        noise = g_adc_noise8 / BC_ADC_SCALE; /* >= 0 */
        step  = span / BC_ADC_STEP_DIV;
        if (step < (BC_ADC_STEP_NOISE * noise))
        {
            step = BC_ADC_STEP_NOISE * noise;
        }
        if (step < BC_ADC_STEP_MIN)
        {
            step = BC_ADC_STEP_MIN;
        }
        g_adc_step = (uint16_t)step; /* <= 4095: fits */

        /* Step 4: follow the extreme since the last switch. */
        if (!gb_adc_black)
        {
            if (avg < g_adc_extreme)
            {
                g_adc_extreme = avg;
            }
        }
        else
        {
            if (avg > g_adc_extreme)
            {
                g_adc_extreme = avg;
            }
        }

        /* Step 5: switch colour once the signal has moved STEP away from
           that extreme, if there is real contrast yet. */
        if ((span >= BC_ADC_MIN_SPAN) && (span >= (2 * step)) &&
            ((!gb_adc_black) ? (avg > (g_adc_extreme + step))
                             : (avg < (g_adc_extreme - step))))
        {
            gb_adc_black  = (uint8_t)(!gb_adc_black);
            g_adc_extreme = avg;
            /* the average lags by half its length */
            ring_put(t_us -
                         ((BC_ADC_AVG / 2U) * (US_PER_S_A0 / BC_ADC_RATE_HZ)),
                     gb_adc_black);
        }
    }
}

/* Letter -> action.  The map lives in RAM so the GUI can set it over MQTT
   (robot/map, retained: the car gets it again on every reconnect).  Until
   a map arrives it is BC_ACTION_MAP from car_config.h.  Letters the map
   doesn't name, digits and symbols all mean BC_ACT_NONE: drive straight
   on.  NOTE: only the comms task writes (barcode_set_map); the barcode
   task reads one byte per decode, and a byte store is atomic, so no lock
   is needed. */
LOCAL volatile uint8_t g_act_map[BC_MAP_LETTERS];
LOCAL volatile BOOL    gb_map_from_gui;
LOCAL volatile BOOL    gb_map_ready;

/**
 * @brief  Load BC_ACTION_MAP from car_config.h into the map.
 */
LOCAL void map_boot_default (void)
{
    static const char def[] = BC_ACTION_MAP;
    uint32_t          idx;

    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        char letter = (idx < (sizeof def - 1U)) ? def[idx] : '-';

        switch (letter)
        {
            case 'L':
                g_act_map[idx] = BC_ACT_LEFT;
            break;
            case 'R':
                g_act_map[idx] = BC_ACT_RIGHT;
            break;
            case 'U':
                g_act_map[idx] = BC_ACT_UTURN;
            break;
            case 'S':
                g_act_map[idx] = BC_ACT_END;
            break;
            default:
                g_act_map[idx] = BC_ACT_NONE;
            break;
        }
    }
    gb_map_ready = TRUE;
}

/**
 * @brief  Replace the letter -> action map (comms task only).
 * @param  p_act  BC_MAP_LETTERS actions; out-of-range ones mean straight.
 */
EXPORT void barcode_set_map (const uint8_t p_act[BC_MAP_LETTERS])
{
    uint32_t idx;

    if (!gb_map_ready)
    {
        map_boot_default();
    }
    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        g_act_map[idx] =
            (p_act[idx] <= BC_ACT_END) ? p_act[idx] : (uint8_t)BC_ACT_NONE;
    }
    gb_map_from_gui = TRUE;
}

/**
 * @brief  Copy the letter -> action map in use.
 * @param  p_act  Output, BC_MAP_LETTERS actions.
 * @return TRUE once a map has come from the GUI.
 */
EXPORT BOOL barcode_get_map (uint8_t p_act[BC_MAP_LETTERS])
{
    uint32_t idx;

    if (!gb_map_ready)
    {
        map_boot_default();
    }
    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        p_act[idx] = g_act_map[idx];
    }
    return gb_map_from_gui;
}

/**
 * @brief  Action for the first character of a barcode.
 * @param  cval  Character (lower case acts like upper case).
 * @return BC_ACT_*; BC_ACT_NONE for anything but a letter.
 */
LOCAL uint32_t action_of (char cval)
{
    uint32_t action = BC_ACT_NONE;

    if ((cval >= 'a') && (cval <= 'z'))
    {
        cval = (char)(cval - 'a' + 'A');
    }
    if ((cval >= 'A') && (cval <= 'Z'))
    {
        action = g_act_map[cval - 'A'];
    }
    return action;
}

LOCAL const char * const gp_act_name[] = {
    "straight on (not mapped)", "LEFT at the next junction",
    "RIGHT at the next junction", "U-TURN", "STOP (end of course)"};

/**
 * @brief  Words for a decoder error, for the console.
 * @param  err  C39_ERR_* value.
 * @return Text.
 */
LOCAL const char * err_text (int32_t err)
{
    switch (err)
    {
        case C39_ERR_LENGTH:
            return "wrong number of bars";
        case C39_ERR_RATIO:
            return "wide/narrow unclear";
        case C39_ERR_UNKNOWN:
            return "unknown pattern";
        case C39_ERR_FRAME:
            return "no * start/stop";
        case C39_ERR_FULLASCII:
            return "bad Full ASCII pair";
        default:
            return "too long";
    }
}

/* ---- debounce: raw edges -> clean edges ---- */
LOCAL uint8_t  gb_db_black;      /* accepted colour, 1 = black */
LOCAL BOOL     gb_db_pending;    /* a burst of raw edges is settling */
LOCAL uint32_t g_db_t_first;     /* first raw edge of the burst, us */
LOCAL uint32_t g_db_t_last;      /* last raw edge of the burst, us */
LOCAL uint8_t  gb_db_last_black; /* colour read at the last raw edge */
LOCAL uint32_t g_diag_changes;   /* BC_DIAG counters */
LOCAL uint32_t g_diag_bars;
LOCAL uint32_t g_diag_clean;

/**
 * @brief  Colour the sensor sees right now.
 * @return 1 = black.
 */
LOCAL uint8_t pin_black (void)
{
#if BARCODE_ANALOG
    return gb_adc_black;
#else
    return ((gpio_get_val(PIN_BARCODE) != 0) == (BARCODE_BLACK_IS_HIGH != 0))
               ? 1U
               : 0U;
#endif
}

/**
 * @brief  A burst has settled on 'level': make it a clean edge if the
 *         colour really changed.
 * @param  level  Colour the burst ended on, 1 = black.
 * @param  p_out  Output edge, written only on a real change.
 * @return TRUE if p_out holds a new edge.
 */
LOCAL BOOL db_settle (uint8_t level, edge_t * p_out)
{
    BOOL b_changed = FALSE;

    gb_db_pending = FALSE;
    if (level != gb_db_black) /* else: chatter that came back, nothing */
    {
        gb_db_black  = level;
        p_out->t     = g_db_t_first + ((g_db_t_last - g_db_t_first) / 2U);
        p_out->black = level;
        g_diag_clean++;
        b_changed = TRUE;
#if BC_DIAG
        {
            /* every real change, with how long the colour before it lasted */
            static uint32_t prev_t;

            tm_printf((uint8_t *)"BARCODE  %s  (%s lasted %d ms)\n",
                      level ? "-> BLACK" : "-> WHITE",
                      level ? "white" : "black",
                      (int32_t)((p_out->t - prev_t) / US_PER_MS));
            prev_t = p_out->t;
        }
#endif
    }
    return b_changed;
}

/**
 * @brief  Next clean (debounced) edge.
 * @param  p_out  Output edge.
 * @return TRUE if p_out holds an edge; FALSE if none is ready yet.
 */
LOCAL BOOL next_edge (edge_t * p_out)
{
    BOOL b_found = FALSE;
    BOOL b_done  = FALSE;

    while (!b_done)
    {
        if (g_ring_tail != g_ring_head)
        {
            uint32_t tval = g_ring[g_ring_tail].t;

            if (gb_db_pending && ((tval - g_db_t_last) >= BC_DEBOUNCE_US))
            {
                /* a quiet gap before this edge: the burst settled */
                if (db_settle(gb_db_last_black, p_out))
                {
                    b_found = TRUE;
                    b_done  = TRUE;
                }
            }
            else
            {
                gb_db_last_black = g_ring[g_ring_tail].black;
                g_ring_tail      = (g_ring_tail + 1U) & (BC_RING - 1U);
                g_last_edge_ms   = motion_now_ms(); /* for barcode_busy() */
                g_diag_changes++;
                if (gb_db_last_black)
                {
                    g_diag_bars++;
                }
                if (!gb_db_pending)
                {
                    gb_db_pending = TRUE;
                    g_db_t_first  = tval;
                }
                g_db_t_last = tval;
            }
        }
        else
        {
            if (gb_db_pending &&
                ((motion_now_us() - g_db_t_last) >= BC_DEBOUNCE_US))
            {
                /* quiet now: read the pin */
                b_found = db_settle(pin_black(), p_out);
            }
            b_done = TRUE;
        }
    }
    return b_found;
}

#if BC_DIAG
/**
 * @brief  Once a second: how busy the sensor was (and, with
 *         BARCODE_ANALOG, the A0 range and learnt levels).
 * @param  p_diag_t  Start of the current second, ms; restarted here.
 */
LOCAL void diag_report (uint32_t * p_diag_t)
{
    if ((motion_now_ms() - *p_diag_t) >= BC_DIAG_MS)
    {
        if (g_diag_changes > 0U)
        {
            tm_printf((uint8_t *)"BARCODE sensor: %d raw changes (%d black) "
                                 "-> %d real after filtering, in %d ms, "
                                 "now %s\n",
                      (int32_t)g_diag_changes, (int32_t)g_diag_bars,
                      (int32_t)g_diag_clean,
                      (int32_t)(motion_now_ms() - *p_diag_t),
                      pin_black() ? "BLACK" : "WHITE");
        }
#if BARCODE_ANALOG
        {
            /* A0 range this second; printed when it moved, or every 5 s */
            static uint32_t quiet;
            uint32_t        min_val = g_adc_min;
            uint32_t        max_val = g_adc_max;
            int32_t         black   = floor_div(g_adc_black8, BC_ADC_SCALE);
            int32_t         white   = floor_div(g_adc_white8, BC_ADC_SCALE);

            g_adc_min = (uint16_t)ADC_FULL_SCALE_A0;
            g_adc_max = 0U;
            if (max_val >= min_val)
            {
                quiet++;
                if (((max_val - min_val) > 60U) || (quiet >= 5U))
                {
                    quiet = 0U;
                    tm_printf(
                        (uint8_t *)"BARCODE A0: now %d, this second %d..%d "
                                   "| learned white %d black %d, noise %d, "
                                   "step %d%s\n",
                        (int32_t)g_adc_now, (int32_t)min_val, (int32_t)max_val,
                        white, black, (int32_t)(g_adc_noise8 / BC_ADC_SCALE),
                        (int32_t)g_adc_step,
                        (((black - white) < BC_ADC_MIN_SPAN) ||
                         ((black - white) < (2 * (int32_t)g_adc_step)))
                            ? " - not enough contrast yet"
                            : "");
                }
            }
        }
#endif
        g_diag_changes = 0U;
        g_diag_bars    = 0U;
        g_diag_clean   = 0U;
        *p_diag_t      = motion_now_ms();
    }
}
#endif

/**
 * @brief  Set up the sensor input (D0 interrupt, or the A0 ADC scan).
 */
LOCAL void sensor_start (void)
{
    gpio_set_pin(PIN_BARCODE, GPIO_MODE_IN);
    gb_db_black = pin_black();
#if BARCODE_ANALOG
    g_adc_min = (uint16_t)ADC_FULL_SCALE_A0;
    g_adc_max = 0U;
    if (line_adc_scan_start())
    {
        tm_printf((uint8_t *)"BARCODE analog A0 on GP%d (ADC scan %d/s per "
                             "pin, filter %d ms)\n",
                  PIN_BARCODE_ADC, BC_ADC_RATE_HZ,
                  (int32_t)(BC_DEBOUNCE_US / US_PER_MS));
    }
    else
    {
        tm_printf((uint8_t *)"BARCODE analog A0: ADC scan did NOT start, "
                             "no barcodes\n");
    }
    (void)bc_isr;
#else
    /* No pull resistors: the input may be the module's A0 (analog) wired
       straight to this digital pin, and a pull would shift its level.
       Schmitt trigger on, so the pin itself has some hysteresis. */
    clr_w(GPIO(PIN_BARCODE), GPIO_PUE | GPIO_PDE);
    set_w(GPIO(PIN_BARCODE), BC_PADS_SCHMITT);
    CHECK(gpio_irq_attach(PIN_BARCODE, GPIO_EV_BOTH, bc_isr),
          "barcode irq"); /* core 0 */
#endif
#if BC_DIAG
    tm_printf((uint8_t *)"BARCODE sensor on GP%d reads %s now (filter %d "
                         "ms)\n",
              BARCODE_ANALOG ? PIN_BARCODE_ADC : PIN_BARCODE,
              pin_black() ? "BLACK" : "WHITE",
              (int32_t)(BC_DEBOUNCE_US / US_PER_MS));
#endif
}

/**
 * @brief  Barcode task (micro T-Kernel task entry).
 * @param  stacd    Required by the micro T-Kernel signature; not used.
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void barcode_task (int32_t stacd, void * p_exinf)
{
    static uint32_t et_val[BC_MAX_EDGES]; /* edge times of the current frame */
    static uint32_t wid[BC_MAX_EDGES];
    int32_t         num        = 0;
    uint8_t         last_black = 0;
    uint32_t        widest     = 0;
    uint32_t        t_start    = 0;
    BOOL            b_broken   = FALSE;
#if BC_DIAG
    uint32_t diag_t;
#endif

    (void)stacd;
    (void)p_exinf;
    sensor_start();
#if BC_DIAG
    diag_t = motion_now_ms();
#endif

    for (;;)
    {
        uint32_t now;
        edge_t   ev_item;

        /* settling a burst: come back as soon as it can be decided */
        tk_slp_tsk(gb_db_pending ? ((BC_DEBOUNCE_US / US_PER_MS) + 1U)
                                 : BC_TASK_SLEEP_MS);
        tk_can_wup(TSK_SELF);

        /* 1. take the (debounced) edges */
        while (next_edge(&ev_item))
        {
            if (!gb_in_frame)
            {
                if (!ev_item.black)
                {
                    continue; /* wait for white -> black */
                }
                gb_in_frame   = TRUE;
                num           = 0;
                widest        = 0;
                b_broken      = FALSE;
                t_start       = motion_now_ms();
                et_val[num++] = ev_item.t;
                last_black    = 1;
                continue;
            }
            if (ev_item.black == last_black)
            {
                b_broken = TRUE; /* missed an edge */
                continue;
            }
            if ((num > 0) && ((ev_item.t - et_val[num - 1]) < BC_MIN_EL_US))
            {
                num--; /* glitch: undo the last edge */
                last_black = ev_item.black;
                if (0 == num)
                {
                    gb_in_frame = FALSE;
                }
                continue;
            }
            if (num >= BC_MAX_EDGES)
            {
                b_broken = TRUE;
                continue;
            }
            if ((num > 0) && ((ev_item.t - et_val[num - 1]) > widest))
            {
                widest = ev_item.t - et_val[num - 1];
            }
            et_val[num++] = ev_item.t;
            last_black    = ev_item.black;
        }

#if BC_DIAG
        diag_report(&diag_t);
#endif

        /* 2. frame over? (white for long enough after a black -> white) */
        if (!gb_in_frame)
        {
            continue;
        }
        now = motion_now_ms();
        if ((widest > (BC_MAX_EL_MS * US_PER_MS)) ||
            ((now - t_start) > BC_MAX_FRAME_MS))
        {
            gb_in_frame = FALSE; /* a junction, or far too slow */
#if BC_DIAG
            if (num >= 3)
            {
                tm_printf((uint8_t *)"BARCODE ignored after %d black bars: "
                                     "%s\n",
                          num / 2,
                          (widest > (BC_MAX_EL_MS * US_PER_MS))
                              ? "one bar/space took over 1.5 s (too slow, "
                                "or the line / a junction)"
                              : "took over 16 s (too slow)");
            }
#endif
            continue;
        }
        {
            /* white since the last edge */
            uint32_t quiet_us = motion_now_us() - et_val[num - 1];

            if ((last_black || (quiet_us < (BC_END_GAP_MS * US_PER_MS))) ||
                (quiet_us < (2U * widest)))
            {
                continue;
            }
        }

        /* 3. decode */
        gb_in_frame = FALSE;
        if (num < 2)
        {
            continue;
        }
        {
            int32_t idx;
            int32_t res;
            int32_t rev;
            char    txt[BC_MAX_TEXT + 1];

            for (idx = 0; idx < (num - 1); idx++)
            {
                wid[idx] = et_val[idx + 1] - et_val[idx];
            }
            res = b_broken ? C39_ERR_LENGTH
                           : code39_decode(wid, num - 1, BC_RATIO_X10, txt,
                                           sizeof txt, &rev);
            if (res > 0)
            {
                res = code39_full_ascii(txt, txt, sizeof txt); /* Full ASCII */
            }
            if (res > 0)
            {
                for (idx = 0; idx <= res; idx++)
                {
                    g_info.text[idx] = txt[idx];
                }
                g_info.action   = action_of(txt[0]);
                g_info.reversed = rev != 0;
                g_info.t_ms     = now;
                g_info.good++;
                g_info.seq++;
                tm_printf((uint8_t *)"BARCODE \"%s\"%s -> %s\n", txt,
                          rev ? " (read backwards)" : "",
                          gp_act_name[g_info.action]);
                mission_event(EV_BARCODE);
            }
            else if ((num - 1) >= BC_MIN_REPORT_ELEMENTS)
            {
                g_info.bad++;
                tm_printf((uint8_t *)"BARCODE not read: %s (%d black bars, "
                                     "*A* has 15; %d elements, need 29)%s\n",
                          err_text(res), num / 2, num - 1,
                          b_broken ? " [an edge was missed]" : "");
                /* every bar / space in ms, to see what went wrong:
                   wide ones should be ~3 x the narrow ones */
                tm_printf((uint8_t *)"BARCODE widths ms (bar space bar ...):");
                for (idx = 0; (idx < (num - 1)) && (idx < BC_MAX_WIDTHS_SHOWN);
                     idx++)
                {
                    tm_printf((uint8_t *)" %d",
                              (int32_t)(wid[idx] / US_PER_MS));
                }
                tm_printf((uint8_t *)"\n");
            }
            else
            {
                /* a speck, not a barcode: nothing to report */
            }
        }
    }
}

/**
 * @brief  Load the default barcode map and start the barcode task.
 * @return E_OK, or the kernel error.
 */
EXPORT ER barcode_start (void)
{
    ER result;
    ID tid;

    if (!gb_map_ready)
    {
        map_boot_default();
    }
    tid = app_main_task_start((FP)barcode_task, PRI_BARCODE, CORE_BARCODE,
                              "barcode");
    if (tid < E_OK)
    {
        result = tid;
    }
    else
    {
        g_bc_tid = tid;
        result   = E_OK;
    }
    return result;
}

/*** end of file ***/
