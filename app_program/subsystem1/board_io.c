/**
 * @file    board_io.c
 *  board_io.c  (Buddy 1)
 *  The Robo Pico's two RGB LEDs and its buzzer.  Used by the status task
 *  only (one owner, no locking needed).
 *
 *  RGB LEDs: 2 x WS2812 in a chain on GP18.  Their 800 kHz one-wire
 *  protocol is clocked out by PIO0, state machine 3, with the standard
 *  4-instruction ws2812 program loaded at instruction address 0.
 *  The Pico W radio driver claims its PIO through the SDK allocator,
 *  which tries the LAST PIO first (pio.c: "while (pio_num--)"), so the
 *  radio sits on PIO1.  v3.18-v3.20 put the LEDs on PIO1 as well: they
 *  worked at power-up and then stopped updating once WiFi was running
 *  (stuck on red).  On PIO0 they share nothing with the radio.
 *  board_io_rgb_set() also resends the colour every second and re-initialises
 *  the state machine if it ever stops taking data.
 *
 *  Buzzer: GP22 = PWM slice 3, channel A.  A tone is a 50 % square wave:
 *  1 MHz counter clock, TOP = 1e6 / f - 1.  The board's mute switch must
 *  be off to hear it.
 */
#include "../car.h"
#include "board_io.h"

/* ---------------- RGB LEDs (PIO0, SM3) ---------------- */
#define RGB_PIO_BASE     0x50200000U /* PIO0 */
#define RGB_PIO_RESET    RESETS_RESET_PIO0
#define PIO_CTRL         (RGB_PIO_BASE + 0x000U)
#define PIO_FSTAT        (RGB_PIO_BASE + 0x004U)
#define PIO_TXF(sm)      (RGB_PIO_BASE + 0x010U + 4U * (sm))
#define PIO_INSTR_MEM(i) (RGB_PIO_BASE + 0x048U + 4U * (i))
#define PIO_SM(sm)       (RGB_PIO_BASE + 0x0C8U + 0x18U * (sm))
#define SM_CLKDIV        0x00U
#define SM_EXECCTRL      0x04U
#define SM_SHIFTCTRL     0x08U
#define SM_ADDR          0x0CU
#define SM_INSTR         0x10U
#define SM_PINCTRL       0x14U
#define RGB_SM           3U
#define FUNCSEL_RGB      6U /* PIO0 */

/* pico-examples ws2812.pio, side-set 1, T1 = 2, T2 = 5, T3 = 3 cycles:
     0: out x, 1        side 0 [2]
     1: jmp !x, 3       side 1 [1]
     2: jmp 0           side 1 [4]
     3: nop             side 0 [4]      (10 cycles per bit) */
/* WS2812 PIO program length in instructions (loop bound, BARR-C 8.4.a). */
#define WS2812_PROG_LEN 4
LOCAL const uint16_t g_ws2812_prog[WS2812_PROG_LEN] = {0x6221, 0x1123, 0x1400,
                                                       0xa442};

LOCAL BOOL     gb_rgb_ready;
LOCAL uint32_t g_rgb_last = 0xFFFFFFFFU;

/**
 * @brief  Initialise the module (create its kernel objects).
 */
LOCAL void rgb_init (void)
{
    uint32_t sm_val = PIO_SM(RGB_SM);
    uint32_t idx;

    clr_w(RESETS_RESET,
          RGB_PIO_RESET); /* (already out of reset on WiFi builds) */
    while (0 == (in_w(RESETS_RESET_DONE) & RGB_PIO_RESET))
    {
    }

    clr_w(PIO_CTRL, 1U << RGB_SM); /* SM off while configuring */
    for (idx = 0; idx < WS2812_PROG_LEN; idx++)
    {
        out_w(PIO_INSTR_MEM(idx), g_ws2812_prog[idx]);
    }
    /* 125 MHz / 15.625 = 8 MHz = 10 cycles per 1.25 us bit (800 kHz) */
    out_w(sm_val + SM_CLKDIV, (15U << 16) | (160U << 8));
    out_w(sm_val + SM_EXECCTRL, (3U << 12) | (0U << 7)); /* wrap 0..3 */
    /* TX FIFO joined (8 deep), autopull at 24 bits, shift out MSB first */
    out_w(sm_val + SM_SHIFTCTRL,
          (1U << 30) | (24U << 25) | (1U << 18) | (1U << 17));
    /* side-set 1 pin and set 1 pin, both GP18 */
    out_w(sm_val + SM_PINCTRL, (1U << 29) | (1U << 26) |
                                   ((uint32_t)PIN_RGB << 10) |
                                   ((uint32_t)PIN_RGB << 5));

    out_w(GPIO_CTRL(PIN_RGB), FUNCSEL_RGB);
    clr_w(GPIO(PIN_RGB), GPIO_OD);

    set_w(PIO_CTRL, (1U << (4 + RGB_SM)) |
                        (1U << (8 + RGB_SM))); /* restart SM + clock divider */
    out_w(sm_val + SM_INSTR, 0xE081U);         /* set pindirs, 1 */
    out_w(sm_val + SM_INSTR, 0x0000U);         /* jmp 0 */
    set_w(PIO_CTRL, 1U << RGB_SM);             /* go */
    gb_rgb_ready = TRUE;
}

/**
 * @brief  Rgb send.
 * @return Result as described above.
 */
LOCAL BOOL rgb_send (uint32_t grb)
{
    uint32_t idx;
    uint32_t spin;
    for (idx = 0; idx < RGB_COUNT; idx++)
    {
        spin = 0;
        while (in_w(PIO_FSTAT) & (1U << (16 + RGB_SM)))
        { /* TX full */
            if (++spin > 100000U)
            {
                return FALSE;
            }
        }
        out_w(PIO_TXF(RGB_SM), grb << 8); /* 24 bits, MSB first */
    }
    return TRUE;
}

LOCAL uint32_t g_rgb_sent_ms;
/**
 * @brief  Board io rgb set.
 */
EXPORT void board_io_rgb_set (uint32_t res, uint32_t gval, uint32_t bval)
{
    uint32_t grb = ((uint32_t)(gval & 0xFFU) << 16) |
                   ((uint32_t)(res & 0xFFU) << 8) | (bval & 0xFFU);
    uint32_t now = motion_now_ms();

    if (!gb_rgb_ready)
    {
        return;
    }
    /* send on a change, and again every second (heals any glitch) */
    if ((grb == g_rgb_last) && ((now - g_rgb_sent_ms) < 1000U))
    {
        return;
    }
    /* the SM must be enabled and pointing to the ws2812 loop */
    if (((!(in_w(PIO_CTRL) & (1U << RGB_SM))) ||
         ((in_w(PIO_SM(RGB_SM) + SM_ADDR) & 0x1FU) > 3U)) ||
        ((in_w(GPIO_CTRL(PIN_RGB)) & 0x1FU) != FUNCSEL_RGB))
    {
        tm_printf(
            (uint8_t *)"RGB LEDs: PIO state machine disturbed (ctrl 0x%x, pc "
                       "%d, GP%d func %d): re-initialising\n",
            (int32_t)in_w(PIO_CTRL),
            (int32_t)(in_w(PIO_SM(RGB_SM) + SM_ADDR) & 0x1FU), PIN_RGB,
            (int32_t)(in_w(GPIO_CTRL(PIN_RGB)) & 0x1FU));
        rgb_init();
    }
    if (!rgb_send(grb))
    {
        tm_printf((uint8_t *)"RGB LEDs: not taking data: re-initialising\n");
        rgb_init();
        if (!rgb_send(grb))
        {
            return;
        }
    }
    g_rgb_last    = grb;
    g_rgb_sent_ms = now;
}

/* ---------------- buzzer (PWM slice 3 A) ---------------- */
#define BZ_SLICE    ((PIN_BUZZER >> 1) & 7U)
#define BZ_REG(o)   (PWM_BASE + 0x14U * BZ_SLICE + (o))
#define FUNCSEL_PWM 4U

/**
 * @brief  Buzz tone.
 */
LOCAL void buzz_tone (uint32_t hz_val)
{
    if (0 == hz_val)
    {
        out_w(BZ_REG(PWM_CHx_CC), 0); /* silent */
        return;
    }
    {
        uint32_t top = (1000000U / hz_val) - 1U;
        out_w(BZ_REG(PWM_CHx_TOP), top);
        out_w(BZ_REG(PWM_CHx_CC), (top + 1U) / 2U); /* channel A = low half */
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 */
LOCAL void buzz_init (void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0 == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
    }
    out_w(BZ_REG(PWM_CHx_CSR), 0);
    out_w(BZ_REG(PWM_CHx_DIV), 125U << 4); /* 1 MHz */
    out_w(BZ_REG(PWM_CHx_CC), 0);
    out_w(BZ_REG(PWM_CHx_TOP), 999U);
    out_w(GPIO_CTRL(PIN_BUZZER), FUNCSEL_PWM);
    out_w(BZ_REG(PWM_CHx_CSR), 1U); /* enable */
}

/* tunes: {Hz, ms} steps, ended by {0, 0} */
typedef struct
{
    uint16_t hz;
    uint16_t ms;
} note_t;
/* the status task steps every 100 ms, so notes are multiples of 100 ms */
LOCAL const note_t g_tune_wifi[] = {
    {1319, 100}, {1760, 100}, {2637, 200}, {0, 0}}; /* rising */
LOCAL const note_t g_tune_lost[] = {
    {1760, 200}, {880, 300}, {0, 0}};                        /* falling */
LOCAL const note_t g_tune_barcode[] = {{2093, 100}, {0, 0}}; /* one beep */
LOCAL const note_t g_tune_action[]  = {
    {2093, 100}, {0, 100}, {2093, 100}, {0, 0}}; /* two beeps */
LOCAL const note_t         g_tune_boot[] = {{1047, 100}, {0, 0}};
LOCAL const note_t * const gp_tunes[]    = {
    g_tune_wifi, g_tune_lost, g_tune_barcode, g_tune_action, g_tune_boot};

LOCAL const note_t * gp_playing;
LOCAL uint32_t       g_note_left_ms;

/**
 * @brief  Board io buzz play.
 */
EXPORT void board_io_buzz_play (uint32_t tune)
{
    if (tune >= (sizeof gp_tunes / sizeof gp_tunes[0]))
    {
        return;
    }
    gp_playing     = gp_tunes[tune];
    g_note_left_ms = gp_playing->ms;
    buzz_tone(gp_playing->hz);
}

/** call every 'ms' (the status task's step) */
EXPORT void board_io_buzz_step (uint32_t ms_val)
{
    if (!gp_playing)
    {
        return;
    }
    if (g_note_left_ms > ms_val)
    {
        g_note_left_ms -= ms_val;
        return;
    }
    gp_playing++;
    if (0 == gp_playing->ms)
    {
        gp_playing = NULL;
        buzz_tone(0);
        return;
    }
    g_note_left_ms = gp_playing->ms;
    buzz_tone(gp_playing->hz);
}

/**
 * @brief  Initialise the module (create its kernel objects).
 */
EXPORT void board_io_init (void)
{
    buzz_init();
    rgb_init();
    board_io_rgb_set(RGB_RED);
}

/*** end of file ***/
