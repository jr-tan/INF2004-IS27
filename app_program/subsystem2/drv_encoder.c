/**
 * @file    drv_encoder.c
 *  drv_encoder.c  (Buddy 2)
 *  Quadrature (A/B) encoders inside the two motors, full x4 decoding.
 *  Pins in car_config.h: left Grove 7 (GP7/GP28), right Grove 1 (GP0/GP1).
 *
 *  Interrupts come through the shared GPIO dispatcher (gpio_irq.c) on
 *  BOTH edges of A and B.  The handler only decodes and counts.
 *
 *  A noise spike is one step forward and one straight back: it cancels.
 *  'count' only goes up: a single step against the current direction is
 *  held back; if the next step returns, both were a bounce; if it goes
 *  the same way, the wheel really reversed and both are counted.
 */
#include "subsystem2.h"
#include <tk/syslib.h>
#include "../gpio_irq.h"
#include "drv_encoder.h" /* own header (BARR-C 4.3.c) */

typedef struct
{
    volatile uint32_t count;
    volatile int32_t  pos;
    volatile uint32_t last_us;
    volatile uint32_t glitch;
    volatile uint8_t  state; /* (A << 1) | B */
    volatile int8_t   hint;  /* direction being counted */
    volatile uint8_t  held;  /* one opposite step held back */
} enc_t;

static enc_t         g_enc[2];
static const uint8_t g_pin_a[2] = {PIN_ENC_L_A, PIN_ENC_R_A};
static const uint8_t g_pin_b[2] = {PIN_ENC_L_B, PIN_ENC_R_B};
static const uint8_t g_rev[2]   = {ENC_L_REVERSE, ENC_R_REVERSE};

/* forward sequence 00 -> 10 -> 11 -> 01 -> 00;  [old << 2 | new] */
static const int8_t g_qdec[16] = {0,  -1, +1, 0,  +1, 0,  0,  -1,
                                  -1, 0,  0,  +1, 0,  +1, -1, 0};

/**
 * @brief  Interrupt handler.
 */
LOCAL void enc_isr (uint32_t gpio, uint32_t events, uint32_t tval)
{
    uint32_t wval   = ((gpio == g_pin_a[0]) || (gpio == g_pin_b[0])) ? 0U : 1U;
    enc_t *  p_ent  = &g_enc[wval];
    uint32_t in_val = in_w(GPIO_IN);
    uint32_t ns_val = (((in_val >> g_pin_a[wval]) & 1U) << 1) |
                      ((in_val >> g_pin_b[wval]) & 1U);
    uint32_t os_val = p_ent->state;
    int32_t  step;

    (void)events;
    if (ns_val == os_val)
    {
        return; /* spike already gone */
    }
    p_ent->state = (uint8_t)ns_val;
    step         = g_qdec[(os_val << 2) | ns_val];
    if (0 == step)
    {
        p_ent->glitch++;
        return;
    } /* two bits at once: resync */
    if (g_rev[wval])
    {
        step = -step;
    }
    p_ent->pos += step;

    if (step == p_ent->hint)
    {
        if (p_ent->held)
        {
            p_ent->held = 0;
            p_ent->glitch++;
            return;
        } /* bounce */
    }
    else if (!p_ent->held)
    {
        p_ent->held = 1; /* wait and see */
        return;
    }
    else
    {
        p_ent->held = 0; /* real reversal */
        p_ent->hint = (int8_t)step;
        p_ent->count++;
    }
    p_ent->count++;
    p_ent->last_us = tval;
}

#define PAD_SCHMITT (1U << 1)

/**
 * @brief  Pin setup.
 */
LOCAL void pin_setup (uint32_t pin)
{
    uint32_t pad;

    gpio_set_pin(pin, GPIO_MODE_IN);
    pad = in_w(GPIO(pin));
    pad |= GPIO_IE | GPIO_PUE | PAD_SCHMITT; /* pull-up + hysteresis */
    pad &= ~GPIO_PDE;
    out_w(GPIO(pin), pad);
}

/**
 * @brief  Initialise the module (create its kernel objects).
 * @return Result as described above.
 */
EXPORT ER drv_encoder_init (void)
{
    ER       err;
    uint32_t idx;

    for (idx = 0; idx < MOT_WHEELS; idx++)
    {
        pin_setup(g_pin_a[idx]);
        pin_setup(g_pin_b[idx]);
        g_enc[idx].state = (uint8_t)((gpio_get_val(g_pin_a[idx]) << 1) |
                                     gpio_get_val(g_pin_b[idx]));
        g_enc[idx].hint  = 1;
    }
    for (idx = 0; idx < MOT_WHEELS; idx++)
    {
        err = gpio_irq_attach(g_pin_a[idx], GPIO_EV_BOTH, enc_isr);
        if (err < E_OK)
        {
            return err;
        }
        err = gpio_irq_attach(g_pin_b[idx], GPIO_EV_BOTH, enc_isr);
        if (err < E_OK)
        {
            return err;
        }
    }
    return E_OK;
}

/** DI only masks THIS core; the encoder interrupt runs on core 0, so the
   caller must be on core 0 (the motion task is pinned there). */
EXPORT void drv_encoder_snapshot (enc_snap_t p_str[2])
{
    uint32_t imask;
    uint32_t idx;

    DI(imask);
    for (idx = 0; idx < MOT_WHEELS; idx++)
    {
        p_str[idx].count   = g_enc[idx].count;
        p_str[idx].pos     = g_enc[idx].pos;
        p_str[idx].last_us = g_enc[idx].last_us;
        p_str[idx].glitch  = g_enc[idx].glitch;
    }
    EI(imask);
}

/*** end of file ***/
