/*
 *	task_wifi_link.c
 *	Subsystem 1 -- wifi_link_task: watches the connection and publishes it as
 *	state (CF_WIFI_UP, CF_COMMS_FAULT, FAULT_WIFI_LOSS, FAULT_COMMS).
 *
 *	It does NOT touch the radio or lwIP (TEAM.md section 3): the join, the
 *	1 s / 2 s / 4 s ... 10 s reconnect backoff and the MQTT session all live in
 *	the kernel's WiFi service task.  This task only reads that task's status
 *	with cyw43_utk_get_status(), which is safe from any task.
 */

#include "subsystem1.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

#define WIFI_POLL_MS		(500)		/* design: periodic 500 ms */
#define LINK_UP_VALUE		(3)		/* CYW43_LINK_UP in cyw43.h (not includable here) */
#define LOSS_HOLD_MS		(5000U)		/* keep FAULT_WIFI_LOSS this long after recovery so the
						   dashboard sees that an outage happened */
#define COMMS_FAULT_AFTER_MS	(20000U)	/* no MQTT session for this long -> comms fault */

static BOOL link_is_up(void)
{
#if TM_WIFI_CYW43
	T_CYW43_UTK_STATUS st;

	cyw43_utk_get_status(&st);
	return (st.ready != 0U) && (st.link_status == LINK_UP_VALUE);
#else
	return FALSE;
#endif
}

EXPORT void wifi_link_task(INT stacd, void *exinf)
{
	BOOL	was_up = FALSE;
	BOOL	loss_pending = FALSE;	/* FAULT_WIFI_LOSS still set, waiting for hold-off */
	UW	up_since = 0U;
	UW	no_session_since = comms_uptime_ms();
	BOOL	comms_fault = FALSE;

	(void)stacd; (void)exinf;

	for (;;) {
		BOOL	up  = link_is_up();
		UW	now = comms_uptime_ms();
		T_RFLG	rflg;
		BOOL	mqtt_up = FALSE;

		if (up && !was_up) {
			(void)tk_set_flg(flg_comms, CF_WIFI_UP);
			up_since = now;
			tm_printf((UB*)"comms: WiFi up\n");
		} else if (!up && was_up) {
			(void)tk_clr_flg(flg_comms, ~(UINT)CF_WIFI_UP);
			fault_raise(FAULT_WIFI_LOSS);
			loss_pending = TRUE;
			tm_printf((UB*)"comms: WiFi LOST\n");
		} else {
			/* no change */
		}
		was_up = up;

		if (up && loss_pending && ((now - up_since) >= LOSS_HOLD_MS)) {
			fault_clear(FAULT_WIFI_LOSS);
			loss_pending = FALSE;
		}

		if (tk_ref_flg(flg_comms, &rflg) == E_OK) {
			mqtt_up = ((rflg.flgptn & CF_MQTT_UP) != 0U);
		}

		/* Comms fault: the robot has had no MQTT session for a long time.  The
		 * mission is NOT affected (NFR-B1-4); this only drives the LED pattern
		 * and a bit in the status/fault data. */
		if (mqtt_up) {
			no_session_since = now;
			if (comms_fault) {
				comms_fault = FALSE;
				(void)tk_clr_flg(flg_comms, ~(UINT)CF_COMMS_FAULT);
				fault_clear(FAULT_COMMS);
			}
		} else if (!comms_fault && ((now - no_session_since) >= COMMS_FAULT_AFTER_MS)) {
			comms_fault = TRUE;
			(void)tk_set_flg(flg_comms, CF_COMMS_FAULT);
			fault_raise(FAULT_COMMS);
			tm_printf((UB*)"comms: COMMS FAULT (no MQTT session for %u s)\n",
				  (UINT)(COMMS_FAULT_AFTER_MS / 1000U));
		} else {
			/* waiting */
		}

		(void)tk_dly_tsk(WIFI_POLL_MS);
	}
}
