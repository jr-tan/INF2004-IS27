/*
 *	drv_encoder.c  (Buddy 2)
 *	Quadrature (A/B) encoders inside the two motors, full x4 decoding.
 *	Pins in car_config.h: left Grove 7 (GP7/GP28), right Grove 1 (GP0/GP1).
 *
 *	Interrupts come through the shared GPIO dispatcher (gpio_irq.c) on
 *	BOTH edges of A and B.  The handler only decodes and counts.
 *
 *	A noise spike is one step forward and one straight back: it cancels.
 *	'count' only goes up: a single step against the current direction is
 *	held back; if the next step returns, both were a bounce; if it goes
 *	the same way, the wheel really reversed and both are counted.
 */
#include "subsystem2.h"
#include <tk/syslib.h>
#include "../gpio_irq.h"

typedef struct {
	volatile UW	count;
	volatile W	pos;
	volatile UW	last_us;
	volatile UW	glitch;
	volatile UB	state;		/* (A << 1) | B */
	volatile B	hint;		/* direction being counted */
	volatile UB	held;		/* one opposite step held back */
} ENC;

static ENC enc[2];
static const UB pin_a[2] = { PIN_ENC_L_A, PIN_ENC_R_A };
static const UB pin_b[2] = { PIN_ENC_L_B, PIN_ENC_R_B };
static const UB rev[2]   = { ENC_L_REVERSE, ENC_R_REVERSE };

/* forward sequence 00 -> 10 -> 11 -> 01 -> 00;  [old << 2 | new] */
static const B qdec[16] = {
	 0, -1, +1,  0,
	+1,  0,  0, -1,
	-1,  0,  0, +1,
	 0, +1, -1,  0
};

LOCAL void enc_isr(UINT gpio, UINT events, UW t)
{
	UINT w = (gpio == pin_a[0] || gpio == pin_b[0]) ? 0U : 1U;
	ENC *e = &enc[w];
	UW in = in_w(GPIO_IN);
	UINT ns = (((in >> pin_a[w]) & 1U) << 1) | ((in >> pin_b[w]) & 1U);
	UINT os = e->state;
	INT step;

	(void)events;
	if (ns == os) return;			/* spike already gone */
	e->state = (UB)ns;
	step = qdec[(os << 2) | ns];
	if (step == 0) { e->glitch++; return; }	/* two bits at once: resync */
	if (rev[w]) step = -step;
	e->pos += step;

	if (step == e->hint) {
		if (e->held) { e->held = 0; e->glitch++; return; }	/* bounce */
	} else if (!e->held) {
		e->held = 1;				/* wait and see */
		return;
	} else {
		e->held = 0;				/* real reversal */
		e->hint = (B)step;
		e->count++;
	}
	e->count++;
	e->last_us = t;
}

#define PAD_SCHMITT	(1U << 1)

LOCAL void pin_setup(UINT pin)
{
	UW pad;

	gpio_set_pin(pin, GPIO_MODE_IN);
	pad = in_w(GPIO(pin));
	pad |= GPIO_IE | GPIO_PUE | PAD_SCHMITT;	/* pull-up + hysteresis */
	pad &= ~GPIO_PDE;
	out_w(GPIO(pin), pad);
}

EXPORT ER drv_encoder_init(void)
{
	ER er;
	UINT i;

	for (i = 0; i < 2; i++) {
		pin_setup(pin_a[i]);
		pin_setup(pin_b[i]);
		enc[i].state = (UB)((gpio_get_val(pin_a[i]) << 1) | gpio_get_val(pin_b[i]));
		enc[i].hint = 1;
	}
	for (i = 0; i < 2; i++) {
		if ((er = gpio_irq_attach(pin_a[i], GPIO_EV_BOTH, enc_isr)) < E_OK) return er;
		if ((er = gpio_irq_attach(pin_b[i], GPIO_EV_BOTH, enc_isr)) < E_OK) return er;
	}
	return E_OK;
}

/* DI only masks THIS core; the encoder interrupt runs on core 0, so the
   caller must be on core 0 (the motion task is pinned there). */
EXPORT void drv_encoder_snapshot(ENC_SNAP s[2])
{
	UINT imask, i;

	DI(imask);
	for (i = 0; i < 2; i++) {
		s[i].count   = enc[i].count;
		s[i].pos     = enc[i].pos;
		s[i].last_us = enc[i].last_us;
		s[i].glitch  = enc[i].glitch;
	}
	EI(imask);
}
