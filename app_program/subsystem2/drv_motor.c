/*
 *	drv_motor.c  (Buddy 2)
 *	Robo Pico H-bridge (MX1508-type) on PWM slices 4 and 5.
 *	Owner: motion task only.
 *
 *	Truth table (Robo Pico datasheet)
 *	  A = PWM, B = 0  forward       A = 0, B = PWM  backward
 *	  A = 0,   B = 0  BRAKE         A = 1, B = 1    coast
 *	PWM clock 125 MHz, divider 1, TOP = 125 MHz / 10 kHz - 1 = 12499.
 */
#include "subsystem2.h"

#define CLK_SYS_HZ	125000000UL
#define PWM_TOP		((UW)(CLK_SYS_HZ / MOT_PWM_HZ) - 1U)
#define SLICE_BASE(s)	(PWM_BASE + 0x14U * (UW)(s))
#define DIV_1		(1U << 4)	/* integer 1, fraction 0 */

static const UB pin_fwd[2] = {
	MOTOR_L_INVERT ? PIN_MOTOR_L_B : PIN_MOTOR_L_A,
	MOTOR_R_INVERT ? PIN_MOTOR_R_B : PIN_MOTOR_R_A
};
static const UB pin_rev[2] = {
	MOTOR_L_INVERT ? PIN_MOTOR_L_A : PIN_MOTOR_L_B,
	MOTOR_R_INVERT ? PIN_MOTOR_R_A : PIN_MOTOR_R_B
};

LOCAL void slice_setup(UINT slice, UINT pin_a)
{
	UW base = SLICE_BASE(slice);

	out_w(base + PWM_CHx_CSR, 0);		/* off while we set it up */
	out_w(base + PWM_CHx_DIV, DIV_1);
	out_w(base + PWM_CHx_CTR, 0);
	out_w(base + PWM_CHx_CC, 0);		/* A = B = 0: brake */
	pwm_set_wrap(pin_a, PWM_TOP);
}

EXPORT void drv_motor_init(void)
{
	UINT m;

	/* This kernel build does not release the PWM block from reset
	   (no physical timers), and writes to a block in reset are lost. */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0) { }

	slice_setup(SLICE_MOTOR_L, PIN_MOTOR_L_A);
	slice_setup(SLICE_MOTOR_R, PIN_MOTOR_R_A);
	for (m = 0; m < 2; m++) {		/* braked BEFORE the pins switch */
		pwm_set_pin(pin_fwd[m]);
		pwm_set_pin(pin_rev[m]);
	}
	pwm_set_enabled(PIN_MOTOR_L_A, TRUE);
	pwm_set_enabled(PIN_MOTOR_R_A, TRUE);
}

LOCAL UW duty_cc(UINT pm)
{
	if (pm >= 1000) return PWM_TOP + 1U;	/* 100 %: never low */
	return ((PWM_TOP + 1U) * pm) / 1000U;
}

/* One 32-bit store per motor: CC holds channel A in bits 0..15 and B in
   bits 16..31, and both channels of a slice drive the same motor.  So
   forward/back/brake switch in one write, with no read-modify-write that
   another core could interleave with (motion_stop() brakes from core 1). */
LOCAL UW cc_reg(UINT m)
{
	return SLICE_BASE(m == MOTOR_L ? SLICE_MOTOR_L : SLICE_MOTOR_R) + PWM_CHx_CC;
}

EXPORT void drv_motor_set(UINT m, INT dir, UINT duty_pm)
{
	UW cc, on_pin;

	if (m > MOTOR_R) return;
	if (duty_pm == 0 || dir == 0) { drv_motor_brake(m); return; }
	cc = duty_cc(duty_pm);
	on_pin = (dir > 0) ? pin_fwd[m] : pin_rev[m];	/* the other input stays 0 */
	out_w(cc_reg(m), (on_pin & 1U) ? (cc << 16) : cc);
}

EXPORT void drv_motor_brake(UINT m)
{
	if (m > MOTOR_R) return;
	out_w(cc_reg(m), 0);				/* A = B = 0 */
}
