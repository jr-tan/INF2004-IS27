/*
 *	gpio_irq.c
 *	Shared GPIO interrupt dispatcher.  See gpio_irq.h.
 */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include "gpio_irq.h"

#define NGPIO			30
#define IO_INTR0		(IO_BANK0_BASE + 0x0F0U)	/* raw, W1C */
#define IO_PROC0_INTE0		(IO_BANK0_BASE + 0x100U)
#define IO_PROC0_INTS0		(IO_BANK0_BASE + 0x120U)
#define EDGE_BITS_ALL		0xCCCCCCCCU	/* bits 2,3 of every nibble */

static GPIO_IRQ_CB	handler[NGPIO];
static BOOL		installed;
EXPORT volatile UW	gpio_irq_max_us;

LOCAL void gpio_irq_dispatch(UINT intno)
{
	UW	now = in_w(TIMER_TIMERAWL);	/* one timestamp for everyone */
	UW	st, ev, took;
	UINT	r, k, pin;

	(void)intno;
	for (r = 0; r < 4; r++) {
		st = in_w(IO_PROC0_INTS0 + 4U * r) & EDGE_BITS_ALL;
		if (st == 0) continue;
		out_w(IO_INTR0 + 4U * r, st);		/* clear what we saw */
		for (k = 0; k < 8; k++) {
			ev = (st >> (4U * k)) & 0xFU;
			if (ev == 0) continue;
			pin = r * 8U + k;
			if (pin < NGPIO && handler[pin] != NULL) {
				handler[pin](pin, ev, now);
			}
		}
	}
	took = in_w(TIMER_TIMERAWL) - now;
	if (took > gpio_irq_max_us) gpio_irq_max_us = took;
}

EXPORT ER gpio_irq_attach(UINT gpio, UINT events, GPIO_IRQ_CB cb)
{
	T_DINT	dint;
	UINT	imask;
	UW	bits;
	UINT	r;
	ER	er = E_OK;

	if (gpio >= NGPIO || cb == NULL || (events & ~GPIO_EV_BOTH) || !events)
		return E_PAR;
#if TK_SUPPORT_SMP
	if (tk_get_prc() != 1) {		/* processor ID 1 = core 0 */
		tm_printf((UB*)"gpio_irq_attach(GP%d): must run on core 0\n", gpio);
		return E_CTX;
	}
#endif
	r    = gpio / 8U;
	bits = events << (4U * (gpio % 8U));

	/* First user installs the dispatcher (all users attach during
	   start-up on core 0, so there is no race on 'installed'). */
	if (!installed) {
		dint.intatr = TA_HLNG;
		dint.inthdr = (FP)gpio_irq_dispatch;
		er = tk_def_int(GPIO_IRQ_NO, &dint);
		if (er < E_OK) return er;
		ClearInt(GPIO_IRQ_NO);
		EnableInt(GPIO_IRQ_NO, GPIO_IRQ_PRI);
		installed = TRUE;
	}
	DI(imask);
	handler[gpio] = cb;
	out_w(IO_INTR0 + 4U * r, bits);		/* drop stale edges */
	set_w(IO_PROC0_INTE0 + 4U * r, bits);	/* atomic set alias */
	EI(imask);
	return er;
}
