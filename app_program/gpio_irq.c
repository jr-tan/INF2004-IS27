/**
 * @file    gpio_irq.c
 *  gpio_irq.c
 *  Shared GPIO interrupt dispatcher.  See gpio_irq.h.
 */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include "gpio_irq.h"

/* IO_BANK0 interrupt status: 4 registers, 8 pins each, 4 event bits per
   pin (RP2040 datasheet 2.19.6); loop bounds (BARR-C 8.4.a). */
#define IRQ_REG_COUNT 4
#define PINS_PER_REG  8

#define NGPIO          30
#define IO_INTR0       (IO_BANK0_BASE + 0x0F0U) /* raw, W1C */
#define IO_PROC0_INTE0 (IO_BANK0_BASE + 0x100U)
#define IO_PROC0_INTS0 (IO_BANK0_BASE + 0x120U)
#define EDGE_BITS_ALL  0xCCCCCCCCU /* bits 2,3 of every nibble */

static gpio_irq_cb_t     g_handler[NGPIO];
static BOOL              gb_installed;
EXPORT volatile uint32_t g_gpio_irq_max_us;

/**
 * @brief  Interrupt handler.
 * @param  intno  Required by the micro T-Kernel signature; not used.
 */
LOCAL void gpio_irq_dispatch_isr (uint32_t intno)
{
    uint32_t now = in_w(TIMER_TIMERAWL); /* one timestamp for everyone */
    uint32_t stat_val;
    uint32_t ev_val;
    uint32_t took;
    uint32_t res;
    uint32_t kdx;
    uint32_t pin;

    (void)intno;
    for (res = 0; res < IRQ_REG_COUNT; res++)
    {
        stat_val = in_w(IO_PROC0_INTS0 + (4U * res)) & EDGE_BITS_ALL;
        if (0 == stat_val)
        {
            continue;
        }
        out_w(IO_INTR0 + (4U * res), stat_val); /* clear what we saw */
        for (kdx = 0; kdx < PINS_PER_REG; kdx++)
        {
            ev_val = (stat_val >> (4U * kdx)) & 0xFU;
            if (0 == ev_val)
            {
                continue;
            }
            pin = (res * 8U) + kdx;
            if ((pin < NGPIO) && (g_handler[pin] != NULL))
            {
                g_handler[pin](pin, ev_val, now);
            }
        }
    }
    took = in_w(TIMER_TIMERAWL) - now;
    if (took > g_gpio_irq_max_us)
    {
        g_gpio_irq_max_us = took;
    }
}

/**
 * @brief  Gpio irq attach.
 * @return Result as described above.
 */
EXPORT ER gpio_irq_attach (uint32_t gpio, uint32_t events, gpio_irq_cb_t cb_val)
{
    T_DINT   dint;
    uint32_t imask;
    uint32_t bits;
    uint32_t res;
    ER       err = E_OK;

    if ((((gpio >= NGPIO) || (NULL == cb_val)) || (events & ~GPIO_EV_BOTH)) ||
        (!events))
    {
        return E_PAR;
    }
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1)
    { /* processor ID 1 = core 0 */
        tm_printf((uint8_t *)"gpio_irq_attach(GP%d): must run on core 0\n",
                  gpio);
        return E_CTX;
    }
#endif
    res  = gpio / 8U;
    bits = events << (4U * (gpio % 8U));

    /* First user installs the dispatcher (all users attach during
       start-up on core 0, so there is no race on 'installed'). */
    if (!gb_installed)
    {
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)gpio_irq_dispatch_isr;
        err         = tk_def_int(GPIO_IRQ_NO, &dint);
        if (err < E_OK)
        {
            return err;
        }
        ClearInt(GPIO_IRQ_NO);
        EnableInt(GPIO_IRQ_NO, GPIO_IRQ_PRI);
        gb_installed = TRUE;
    }
    DI(imask);
    g_handler[gpio] = cb_val;
    out_w(IO_INTR0 + (4U * res), bits);       /* drop stale edges */
    set_w(IO_PROC0_INTE0 + (4U * res), bits); /* atomic set alias */
    EI(imask);
    return err;
}

/*** end of file ***/
