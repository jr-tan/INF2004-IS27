/*
 *	task_led.c
 *	Subsystem 1 -- comms status LED on GP19 (the Pico W's on-board LED is on
 *	the radio chip and may not be driven from a student task).
 *
 *	  solid on        MQTT session up, link healthy            (Operational)
 *	  steady blink    connecting / reconnecting / link lost    (2 Hz)
 *	  short flash     comms fault: no MQTT session for > 20 s  (1 tick in 4)
 *
 *	Paced by a cyclic handler, as in the original template example.
 */

#include "subsystem1.h"

#define FAULT_PHASES	(4U)

EXPORT void led_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_led);	/* wake the LED task (requests queue up) */
}

EXPORT void led_task(INT stacd, void *exinf)
{
	UINT	phase = 0U;
	UINT	level = 0U;

	(void)stacd; (void)exinf;

	gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);

	for (;;) {
		T_RFLG	rflg;
		UINT	ptn = 0U;

		tk_slp_tsk(TMO_FEVR);
		tk_can_wup(TSK_SELF);	/* do not "catch up" after being suspended */

		if (tk_ref_flg(flg_comms, &rflg) == E_OK) {
			ptn = rflg.flgptn;
		}

		if ((ptn & CF_COMMS_FAULT) != 0U) {
			phase = (phase + 1U) % FAULT_PHASES;
			level = (phase == 0U) ? 1U : 0U;
		} else if (((ptn & CF_MQTT_UP) != 0U) && ((ptn & CF_CONN_LOST) == 0U)) {
			level = 1U;
		} else {
			level = (level == 0U) ? 1U : 0U;
		}
		gpio_set_val(S1_LED_PIN, level);
	}
}
