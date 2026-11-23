/**
 * @file    drv_motor.c
 *  drv_motor.c  (Buddy 2)
 *  Robo Pico H-bridge (MX1508-type) on PWM slices 4 and 5.
 *  Owner: motion task only.
 *
 *  Truth table (Robo Pico datasheet)
 *    A = PWM, B = 0  forward       A = 0, B = PWM  backward
 *    A = 0,   B = 0  BRAKE         A = 1, B = 1    coast
 *  PWM clock 125 MHz, divider 1, TOP = 125 MHz / 10 kHz - 1 = 12499.
 */
#include "subsystem2.h"
#include "drv_motor.h" /* own header (BARR-C 4.3.c) */

#define CLK_SYS_HZ    125000000UL
#define PWM_TOP       ((uint32_t)(CLK_SYS_HZ / MOT_PWM_HZ) - 1U)
#define SLICE_BASE(s) (PWM_BASE + 0x14U * (uint32_t)(s))
#define DIV_1         (1U << 4) /* integer 1, fraction 0 */

static const uint8_t g_pin_fwd[2] = {
    MOTOR_L_INVERT ? PIN_MOTOR_L_B : PIN_MOTOR_L_A,
    MOTOR_R_INVERT ? PIN_MOTOR_R_B : PIN_MOTOR_R_A};
static const uint8_t g_pin_rev[2] = {
    MOTOR_L_INVERT ? PIN_MOTOR_L_A : PIN_MOTOR_L_B,
    MOTOR_R_INVERT ? PIN_MOTOR_R_A : PIN_MOTOR_R_B};

/**
 * @brief  Slice setup.
 */
LOCAL void slice_setup (uint32_t slice, uint32_t pin_a)
{
    uint32_t base = SLICE_BASE(slice);

    out_w(base + PWM_CHx_CSR, 0); /* off while we set it up */
    out_w(base + PWM_CHx_DIV, DIV_1);
    out_w(base + PWM_CHx_CTR, 0);
    out_w(base + PWM_CHx_CC, 0); /* A = B = 0: brake */
    pwm_set_wrap(pin_a, PWM_TOP);
}

/**
 * @brief  Initialise the module (create its kernel objects).
 */
EXPORT void drv_motor_init (void)
{
    uint32_t mval;

    /* This kernel build does not release the PWM block from reset
       (no physical timers), and writes to a block in reset are lost. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0 == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
    }

    slice_setup(SLICE_MOTOR_L, PIN_MOTOR_L_A);
    slice_setup(SLICE_MOTOR_R, PIN_MOTOR_R_A);
    for (mval = 0; mval < MOT_WHEELS; mval++)
    { /* braked BEFORE the pins switch */
        pwm_set_pin(g_pin_fwd[mval]);
        pwm_set_pin(g_pin_rev[mval]);
    }
    pwm_set_enabled(PIN_MOTOR_L_A, TRUE);
    pwm_set_enabled(PIN_MOTOR_R_A, TRUE);
}

/**
 * @brief  Duty cc.
 * @return Result as described above.
 */
LOCAL uint32_t duty_cc (uint32_t pm_val)
{
    if (pm_val >= 1000)
    {
        return PWM_TOP + 1U; /* 100 %: never low */
    }
    return ((PWM_TOP + 1U) * pm_val) / 1000U;
}

/** One 32-bit store per motor: CC holds channel A in bits 0..15 and B in
   bits 16..31, and both channels of a slice drive the same motor.  So
   forward/back/brake switch in one write, with no read-modify-write that
   another core could interleave with (motion_stop() brakes from core 1). */
LOCAL uint32_t cc_reg (uint32_t mval)
{
    return SLICE_BASE(mval == MOTOR_L ? SLICE_MOTOR_L : SLICE_MOTOR_R) +
           PWM_CHx_CC;
}

/**
 * @brief  Drv motor set.
 */
EXPORT void drv_motor_set (uint32_t mval, int32_t dir, uint32_t duty_pm)
{
    uint32_t cc_val;
    uint32_t on_pin;

    if (mval > MOTOR_R)
    {
        return;
    }
    if ((0 == duty_pm) || (0 == dir))
    {
        drv_motor_brake(mval);
        return;
    }
    cc_val = duty_cc(duty_pm);
    on_pin = (dir > 0) ? g_pin_fwd[mval]
                       : g_pin_rev[mval]; /* the other input stays 0 */
    out_w(cc_reg(mval), (on_pin & 1U) ? (cc_val << 16) : cc_val);
}

/**
 * @brief  Drv motor brake.
 */
EXPORT void drv_motor_brake (uint32_t mval)
{
    if (mval > MOTOR_R)
    {
        return;
    }
    out_w(cc_reg(mval), 0); /* A = B = 0 */
}

/*** end of file ***/
