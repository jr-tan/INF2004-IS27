/*
 *	board_io.c  (Buddy 1)
 *	The Robo Pico's two RGB LEDs and its buzzer.  Used by the status task
 *	only (one owner, no locking needed).
 *
 *	RGB LEDs: 2 x WS2812 in a chain on GP18.  Their 800 kHz one-wire
 *	protocol is clocked out by PIO0, state machine 3, with the standard
 *	4-instruction ws2812 program loaded at instruction address 0.
 *	The Pico W radio driver claims its PIO through the SDK allocator,
 *	which tries the LAST PIO first (pio.c: "while (pio_num--)"), so the
 *	radio sits on PIO1.  v3.18-v3.20 put the LEDs on PIO1 as well: they
 *	worked at power-up and then stopped updating once WiFi was running
 *	(stuck on red).  On PIO0 they share nothing with the radio.
 *	rgb_set() also resends the colour every second and re-initialises
 *	the state machine if it ever stops taking data.
 *
 *	Buzzer: GP22 = PWM slice 3, channel A.  A tone is a 50 % square wave:
 *	1 MHz counter clock, TOP = 1e6 / f - 1.  The board's mute switch must
 *	be off to hear it.
 */
#include "../car.h"
#include "board_io.h"

/* ---------------- RGB LEDs (PIO0, SM3) ---------------- */
#define RGB_PIO_BASE	0x50200000U	/* PIO0 */
#define RGB_PIO_RESET	RESETS_RESET_PIO0
#define PIO_CTRL	(RGB_PIO_BASE + 0x000U)
#define PIO_FSTAT	(RGB_PIO_BASE + 0x004U)
#define PIO_TXF(sm)	(RGB_PIO_BASE + 0x010U + 4U * (sm))
#define PIO_INSTR_MEM(i) (RGB_PIO_BASE + 0x048U + 4U * (i))
#define PIO_SM(sm)	(RGB_PIO_BASE + 0x0C8U + 0x18U * (sm))
#define SM_CLKDIV	0x00U
#define SM_EXECCTRL	0x04U
#define SM_SHIFTCTRL	0x08U
#define SM_ADDR		0x0CU
#define SM_INSTR	0x10U
#define SM_PINCTRL	0x14U
#define RGB_SM		3U
#define FUNCSEL_RGB	6U		/* PIO0 */

/* pico-examples ws2812.pio, side-set 1, T1 = 2, T2 = 5, T3 = 3 cycles:
     0: out x, 1        side 0 [2]
     1: jmp !x, 3       side 1 [1]
     2: jmp 0           side 1 [4]
     3: nop             side 0 [4]      (10 cycles per bit) */
LOCAL const UH ws2812_prog[4] = { 0x6221, 0x1123, 0x1400, 0xa442 };

LOCAL BOOL rgb_ready;
LOCAL UW rgb_last = 0xFFFFFFFFU;

LOCAL void rgb_init(void)
{
	UW sm = PIO_SM(RGB_SM);
	UINT i;

	clr_w(RESETS_RESET, RGB_PIO_RESET);		/* (already out of reset on WiFi builds) */
	while ((in_w(RESETS_RESET_DONE) & RGB_PIO_RESET) == 0) { }

	clr_w(PIO_CTRL, 1U << RGB_SM);			/* SM off while configuring */
	for (i = 0; i < 4; i++) out_w(PIO_INSTR_MEM(i), ws2812_prog[i]);
	/* 125 MHz / 15.625 = 8 MHz = 10 cycles per 1.25 us bit (800 kHz) */
	out_w(sm + SM_CLKDIV, (15U << 16) | (160U << 8));
	out_w(sm + SM_EXECCTRL, (3U << 12) | (0U << 7));		/* wrap 0..3 */
	/* TX FIFO joined (8 deep), autopull at 24 bits, shift out MSB first */
	out_w(sm + SM_SHIFTCTRL, (1U << 30) | (24U << 25) | (1U << 18) | (1U << 17));
	/* side-set 1 pin and set 1 pin, both GP18 */
	out_w(sm + SM_PINCTRL, (1U << 29) | (1U << 26) | ((UW)PIN_RGB << 10) | ((UW)PIN_RGB << 5));

	out_w(GPIO_CTRL(PIN_RGB), FUNCSEL_RGB);
	clr_w(GPIO(PIN_RGB), GPIO_OD);

	set_w(PIO_CTRL, (1U << (4 + RGB_SM)) | (1U << (8 + RGB_SM)));	/* restart SM + clock divider */
	out_w(sm + SM_INSTR, 0xE081U);			/* set pindirs, 1 */
	out_w(sm + SM_INSTR, 0x0000U);			/* jmp 0 */
	set_w(PIO_CTRL, 1U << RGB_SM);			/* go */
	rgb_ready = TRUE;
}

LOCAL BOOL rgb_send(UW grb)
{
	UINT i, spin;
	for (i = 0; i < RGB_COUNT; i++) {
		spin = 0;
		while (in_w(PIO_FSTAT) & (1U << (16 + RGB_SM))) {	/* TX full */
			if (++spin > 100000U) return FALSE;
		}
		out_w(PIO_TXF(RGB_SM), grb << 8);		/* 24 bits, MSB first */
	}
	return TRUE;
}

LOCAL UW rgb_sent_ms;
EXPORT void rgb_set(UINT r, UINT g, UINT b)
{
	UW grb = ((UW)(g & 0xFFU) << 16) | ((UW)(r & 0xFFU) << 8) | (b & 0xFFU);
	UW now = now_ms();

	if (!rgb_ready) return;
	/* send on a change, and again every second (heals any glitch) */
	if (grb == rgb_last && (now - rgb_sent_ms) < 1000U) return;
	/* the SM must be enabled and pointing to the ws2812 loop */
	if (!(in_w(PIO_CTRL) & (1U << RGB_SM)) || (in_w(PIO_SM(RGB_SM) + SM_ADDR) & 0x1FU) > 3U ||
	    (in_w(GPIO_CTRL(PIN_RGB)) & 0x1FU) != FUNCSEL_RGB) {
		tm_printf((UB*)"RGB LEDs: PIO state machine disturbed (ctrl 0x%x, pc %d, GP%d func %d): re-initialising\n",
			  (INT)in_w(PIO_CTRL), (INT)(in_w(PIO_SM(RGB_SM) + SM_ADDR) & 0x1FU), PIN_RGB,
			  (INT)(in_w(GPIO_CTRL(PIN_RGB)) & 0x1FU));
		rgb_init();
	}
	if (!rgb_send(grb)) {
		tm_printf((UB*)"RGB LEDs: not taking data: re-initialising\n");
		rgb_init();
		if (!rgb_send(grb)) return;
	}
	rgb_last = grb;
	rgb_sent_ms = now;
}

/* ---------------- buzzer (PWM slice 3 A) ---------------- */
#define BZ_SLICE	((PIN_BUZZER >> 1) & 7U)
#define BZ_REG(o)	(PWM_BASE + 0x14U * BZ_SLICE + (o))
#define FUNCSEL_PWM	4U

LOCAL void buzz_tone(UINT hz)
{
	if (hz == 0) {
		out_w(BZ_REG(PWM_CHx_CC), 0);			/* silent */
		return;
	}
	{
		UW top = 1000000U / hz - 1U;
		out_w(BZ_REG(PWM_CHx_TOP), top);
		out_w(BZ_REG(PWM_CHx_CC), (top + 1U) / 2U);	/* channel A = low half */
	}
}

LOCAL void buzz_init(void)
{
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0) { }
	out_w(BZ_REG(PWM_CHx_CSR), 0);
	out_w(BZ_REG(PWM_CHx_DIV), 125U << 4);			/* 1 MHz */
	out_w(BZ_REG(PWM_CHx_CC), 0);
	out_w(BZ_REG(PWM_CHx_TOP), 999U);
	out_w(GPIO_CTRL(PIN_BUZZER), FUNCSEL_PWM);
	out_w(BZ_REG(PWM_CHx_CSR), 1U);				/* enable */
}

/* tunes: {Hz, ms} steps, ended by {0, 0} */
typedef struct { UH hz, ms; } NOTE;
/* the status task steps every 100 ms, so notes are multiples of 100 ms */
LOCAL const NOTE tune_wifi[]    = { {1319, 100}, {1760, 100}, {2637, 200}, {0, 0} };	/* rising */
LOCAL const NOTE tune_lost[]    = { {1760, 200}, {880, 300}, {0, 0} };			/* falling */
LOCAL const NOTE tune_barcode[] = { {2093, 100}, {0, 0} };				/* one beep */
LOCAL const NOTE tune_action[]  = { {2093, 100}, {0, 100}, {2093, 100}, {0, 0} };	/* two beeps */
LOCAL const NOTE tune_boot[]    = { {1047, 100}, {0, 0} };
LOCAL const NOTE *const tunes[] = { tune_wifi, tune_lost, tune_barcode, tune_action, tune_boot };

LOCAL const NOTE *playing;
LOCAL UINT note_left_ms;

EXPORT void buzz_play(UINT tune)
{
	if (tune >= sizeof tunes / sizeof tunes[0]) return;
	playing = tunes[tune];
	note_left_ms = playing->ms;
	buzz_tone(playing->hz);
}

/* call every 'ms' (the status task's step) */
EXPORT void buzz_step(UINT ms)
{
	if (!playing) return;
	if (note_left_ms > ms) { note_left_ms -= ms; return; }
	playing++;
	if (playing->ms == 0) { playing = NULL; buzz_tone(0); return; }
	note_left_ms = playing->ms;
	buzz_tone(playing->hz);
}

EXPORT void board_io_init(void)
{
	buzz_init();
	rgb_init();
	rgb_set(RGB_RED);
}
