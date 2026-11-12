/*
 *	subsystem5.c  (Buddy 5: ranger)
 *	ranger task: owns the HC-SR04 (3.3 V type), the SG90 servo and the
 *	alert LED.  One ping every 60 ms.
 *
 *	Echo timing is done by HARDWARE: GP17 is PWM slice 0 input B, in
 *	"count while B is high" mode at 1 MHz, so the counter holds the echo
 *	width in us.  Encoder interrupt bursts can't shift the reading and
 *	the task just sleeps 40 ms instead of busy-waiting.
 *
 *	Each reading is tagged with the servo angle and is only marked
 *	"valid" if the servo had settled before the ping.
 */
#include "subsystem5.h"

#define SLICE_REG(s)	(PWM_BASE + 0x14U * (UW)(s))
#define CSR_EN		(1U << 0)
#define CSR_LEVEL	(1U << 4)	/* DIVMODE 1: count while B high */
#define DIV_125		(125U << 4)	/* 125 MHz / 125 = 1 MHz */
#define FUNCSEL_PWM	4U
#define SERVO_PERIOD_US	20000U

LOCAL volatile INT aim_req;
LOCAL volatile BOOL guard_on;
LOCAL volatile BOOL fast_on;
LOCAL volatile UW last_word = RANGE_FAR_CM | (128U << 8);
LOCAL volatile UW no_echo;

EXPORT void ranger_aim(INT deg)    { aim_req = CLAMP(deg, -SERVO_MAX_DEG, SERVO_MAX_DEG); }
EXPORT void ranger_guard(BOOL on)  { guard_on = on; }
LOCAL volatile INT guard_last = RANGE_FAR_CM;	/* last close reading the guard counted */
EXPORT INT  ranger_guard_cm(void)  { return guard_last; }
EXPORT void ranger_fast(BOOL on)   { fast_on = on; }
EXPORT UW   ranger_word(void)      { return last_word; }
EXPORT UW   ranger_no_echo_count(void) { return no_echo; }
EXPORT void ranger_led(BOOL on)    { gpio_set_val(PIN_ALERT_LED, on ? 1 : 0); }

/* ---------------- drivers ---------------- */
LOCAL void pwm_unreset(void)
{
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0) { }
}

LOCAL void us_init(void)
{
	UW base = SLICE_REG(SLICE_US), pad;

	gpio_set_pin(PIN_US_TRIG, GPIO_MODE_OUT);
	gpio_set_val(PIN_US_TRIG, 0);
	out_w(base + PWM_CHx_CSR, 0);
	out_w(base + PWM_CHx_DIV, DIV_125);
	out_w(base + PWM_CHx_TOP, 0xFFFFU);
	out_w(base + PWM_CHx_CTR, 0);
	out_w(GPIO_CTRL(PIN_US_ECHO), FUNCSEL_PWM);	/* echo -> PWM 0 B input */
	pad = in_w(GPIO(PIN_US_ECHO));
	pad |= GPIO_IE | GPIO_OD | GPIO_PDE;		/* input only, pulled low */
	pad &= ~GPIO_PUE;
	out_w(GPIO(PIN_US_ECHO), pad);
}

LOCAL void us_trigger(void)
{
	UW base = SLICE_REG(SLICE_US), t0;

	out_w(base + PWM_CHx_CSR, 0);
	out_w(base + PWM_CHx_CTR, 0);
	out_w(base + PWM_CHx_CSR, CSR_LEVEL | CSR_EN);	/* armed: counts while echo high */
	gpio_set_val(PIN_US_TRIG, 1);
	t0 = now_us();
	while ((UW)(now_us() - t0) < 12U) { }		/* 12 us trigger pulse */
	gpio_set_val(PIN_US_TRIG, 0);
}

/* echo width in us; 0 = echo never rose (no sensor), 0xFFFF = still high */
LOCAL UW us_read(void)
{
	UW base = SLICE_REG(SLICE_US);
	UW w = in_w(base + PWM_CHx_CTR) & 0xFFFFU;

	if (gpio_get_val(PIN_US_ECHO)) return 0xFFFFU;
	out_w(base + PWM_CHx_CSR, 0);
	return w;
}

LOCAL void servo_init(void)
{
	UW base = SLICE_REG(SLICE_SERVO);

	out_w(base + PWM_CHx_CSR, 0);
	out_w(base + PWM_CHx_DIV, DIV_125);		/* 1 count = 1 us */
	out_w(base + PWM_CHx_CTR, 0);
	pwm_set_wrap(PIN_SERVO, SERVO_PERIOD_US - 1U);
	pwm_set_cc(PIN_SERVO, SERVO_CENTER_US);
	pwm_set_pin(PIN_SERVO);
	pwm_set_enabled(PIN_SERVO, TRUE);
}

/* move, return settle time in ms */
LOCAL UINT servo_set(INT deg, INT from)
{
	INT us = SERVO_CENTER_US + (SERVO_REVERSE ? -deg : deg) * SERVO_US_PER_DEG;
	INT moved = ABS(deg - from);
	UINT ms = (UINT)((moved * SERVO_SETTLE_MS + 14) / 15);

	pwm_set_cc(PIN_SERVO, (UW)us);
	return ms < SERVO_SETTLE_MIN_MS ? SERVO_SETTLE_MIN_MS : ms;
}

/* ---------------- ranger task ---------------- */
LOCAL void ranger_task(INT stacd, void *exinf)
{
	INT cur = 0;
	UW settled_at = now_ms() + 500U;	/* servo power-up */
	UINT seq = 0, hits = 0, stuck = 0, silent = 0;

	(void)stacd; (void)exinf;

	/* Servo self-test: swing right, left, centre, so you can SEE it work
	   (and prove the PWM is set up, whatever the hardware does). */
	{
		UW b = SLICE_REG(SLICE_SERVO), cc = in_w(b + PWM_CHx_CC);
		tm_printf((UB*)"SERVO GP%d: PWM slice %d %s, %d Hz, pulse %d us, pin function %d (4 = PWM)\n",
			  PIN_SERVO, SLICE_SERVO, (in_w(b + PWM_CHx_CSR) & CSR_EN) ? "on" : "OFF",
			  (INT)(1000000U / (in_w(b + PWM_CHx_TOP) + 1U)),
			  (INT)((PIN_SERVO & 1) ? (cc >> 16) : (cc & 0xFFFFU)),
			  (INT)(in_w(GPIO_CTRL(PIN_SERVO)) & 0x1FU));
		tm_printf((UB*)"SERVO self-test: the sensor should turn RIGHT, then LEFT, then back to the front (%d deg)\n",
			  SERVO_TEST_DEG);
		tk_dly_tsk(300);
		(void)servo_set(-SERVO_TEST_DEG, 0);              tk_dly_tsk(700);
		(void)servo_set(SERVO_TEST_DEG, -SERVO_TEST_DEG); tk_dly_tsk(1000);
		(void)servo_set(0, SERVO_TEST_DEG);               tk_dly_tsk(700);
		settled_at = now_ms();
	}

	for (;;) {
		INT want = aim_req;
		UW echo, t;
		INT cm;
		BOOL valid, fast;

		if (want != cur) {
			settled_at = now_ms() + servo_set(want, cur);
			cur = want;
		}
		t = now_ms();
		if ((W)(settled_at - t) > 0) {		/* still moving: wait, no ping */
			UW wait = settled_at - t;
			tk_dly_tsk(wait > 200U ? 200U : wait);
			continue;
		}

		/* Echo still high from the last ping (some modules hold it up
		   to ~200 ms when nothing answers): a trigger now would be
		   ignored and the counter would time the old pulse's tail. */
		if (gpio_get_val(PIN_US_ECHO)) {
			if (++stuck >= 50) {		/* ~1 s high: sensor fault */
				last_word = (UW)RANGE_FAR_CM | ((UW)(cur + 128) << 8) |
					    ((UW)((seq = (seq + 1U) & 0xFFU)) << 16);	/* invalid */
				stuck = 0;
			}
			tk_dly_tsk(fast_on ? 10 : 20);	/* poll fast: the sweep waits on this */
			continue;
		}
		stuck = 0;

		fast = fast_on;
		us_trigger();
		tk_dly_tsk(fast ? SWEEP_ECHO_WAIT_MS : ECHO_WAIT_MS);
		echo = us_read();

		/* 10 pings in a row with no echo pulse at all: sensor missing */
		if (echo == 0) { if (++silent == 10) fault_raise(FAULT_NO_ECHO); }
		else if (silent) { if (silent >= 10) fault_clear(FAULT_NO_ECHO); silent = 0; }

		if (echo == 0) {			/* no echo pulse at all */
			no_echo++;
			cm = RANGE_FAR_CM;
			valid = FALSE;
		} else if (echo == 0xFFFFU) {		/* still high: nothing in range */
			cm = RANGE_FAR_CM;
			valid = TRUE;
		} else {
			cm = (INT)(echo / US_PER_CM);
			if (cm > RANGE_FAR_CM) cm = RANGE_FAR_CM;
			valid = cm >= US_MIN_CM;
		}
		valid = valid && (aim_req == cur);	/* not if a new aim arrived */

		seq = (seq + 1U) & 0xFFU;
		last_word = (UW)cm | ((UW)(cur + 128) << 8) | ((UW)seq << 16) |
			    (valid ? (1U << 24) : 0U);
		mission_event(EV_PING);

		/* guard: GUARD_HITS close pings out of the last GUARD_WINDOW,
		   sensor looking straight ahead.  12 Nov 19:40 / 19:47 runs:
		   "N in a row" let every missed echo (common at 3.3 V) reset
		   the count, so the car only stopped at 5..9 cm. */
		if (guard_on && cur == 0) {
			BOOL close = valid && cm <= GUARD_STOP_CM;
			UINT h, k;
			hits = ((hits << 1) | (close ? 1U : 0U)) & ((1U << GUARD_WINDOW) - 1U);
			if (close) guard_last = cm;
			for (h = hits, k = 0; h; h >>= 1) k += h & 1U;
			if (k >= GUARD_HITS) mission_event(EV_OBSTACLE);
		} else {
			hits = 0;
		}

		tk_dly_tsk(fast ? SWEEP_PING_PERIOD_MS - SWEEP_ECHO_WAIT_MS : PING_PERIOD_MS - ECHO_WAIT_MS);
	}
}

EXPORT ER subsystem5_init(void)
{
	pwm_unreset();
	us_init();
	servo_init();
	gpio_set_pin(PIN_ALERT_LED, GPIO_MODE_OUT);
	gpio_set_val(PIN_ALERT_LED, 0);
	return E_OK;				/* no kernel objects */
}

EXPORT ER subsystem5_start(void)
{
	ID tid = app_start_task((FP)ranger_task, PRI_RANGER, CORE_RANGER, "ranger");
	return tid < E_OK ? tid : E_OK;
}

#if APP_TEST
EXPORT void subsystem5_test(void) { }
#endif
