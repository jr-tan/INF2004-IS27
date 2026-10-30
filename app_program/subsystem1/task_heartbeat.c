/*
 *	task_heartbeat.c
 *	Subsystem 1 -- heartbeat_task: publishes robot/status every 1 s (NFR-B1-2),
 *	and immediately on a mode change, on MQTT (re)connect, or on a PING command.
 *	It is also the only place that decides CONN_LOST.
 *
 *	Single-writer rule: the 4-bit status sequence number is written only here.
 */

#include "subsystem1.h"

#define HB_PERIOD_MS		(1000)
#define MISSED_FOR_LOST		(3U)		/* NFR-B1-2: 3 missed heartbeats -> CONN_LOST */
#define LOSS_HOLD_TICKS		(5U)		/* keep FAULT_MQTT_LOSS 5 good beats after recovery */
#define SEQ_MASK		(0x0FU)		/* 4-bit sequence number */

static volatile UW	s_hb_count;		/* writer: this task only */

EXPORT UW comms_hb_count(void)
{
	return s_hb_count;
}

static BOOL link_healthy(void)
{
	T_RFLG rflg;

	if (tk_ref_flg(flg_comms, &rflg) != E_OK) {
		return FALSE;
	}
	return ((rflg.flgptn & (CF_WIFI_UP | CF_MQTT_UP)) == (CF_WIFI_UP | CF_MQTT_UP)) &&
	       (comms_pub_fail_streak() == 0U);
}

EXPORT void heartbeat_task(INT stacd, void *exinf)
{
	status_frame_t	st;
	UB		buf[COMMS_FRAME_MAX];
	UB		seq = 0U;
	UB		missed = 0U;
	UB		good_since_lost = 0U;
	BOOL		loss_pending = FALSE;
	UW		next_due;

	(void)stacd; (void)exinf;

	next_due = comms_uptime_ms() + (UW)HB_PERIOD_MS;

	for (;;) {
		UINT	ptn;
		UINT	len;
		UW	now;
		W	wait;
		BOOL	periodic;
		ER	er;

		now = comms_uptime_ms();
		wait = (W)(next_due - now);
		if ((wait < 1) || (wait > HB_PERIOD_MS)) {
			wait = 1;	/* overdue (or clock hiccup): run now */
		}

		/* Wake early on a status request; otherwise time out when due. */
		er = tk_wai_flg(flg_comms, CF_STATUS_NOW, TWF_ORW | TWF_BITCLR, &ptn, wait);
		periodic = (er == E_TMOUT);
		if ((er != E_OK) && (er != E_TMOUT)) {
			(void)tk_dly_tsk(HB_PERIOD_MS);	/* object problem: do not spin */
			continue;
		}

		if (periodic) {
			now = comms_uptime_ms();
			next_due += (UW)HB_PERIOD_MS;
			if ((W)(next_due - now) < 1) {
				next_due = now + (UW)HB_PERIOD_MS;
			}

			/* CONN_LOST bookkeeping, once per second */
			if (link_healthy()) {
				missed = 0U;
				if (loss_pending) {
					good_since_lost++;
					if (good_since_lost >= LOSS_HOLD_TICKS) {
						fault_clear(FAULT_MQTT_LOSS);
						loss_pending = FALSE;
					}
				}
				(void)tk_clr_flg(flg_comms, ~(UINT)CF_CONN_LOST);
			} else if (missed < UINT8_MAX) {
				missed++;
				if (missed == MISSED_FOR_LOST) {
					(void)tk_set_flg(flg_comms, CF_CONN_LOST);
					fault_raise(FAULT_MQTT_LOSS);
					loss_pending = TRUE;
					good_since_lost = 0U;
					tm_printf((UB*)"comms: CONN_LOST (3 missed heartbeats)\n");
				}
			} else {
				/* saturated */
			}
		}

		st.seq          = seq;
		st.up_s         = comms_uptime_ms() / 1000U;
		st.rssi_neg_dbm = 0U;	/* 0 = unknown: RSSI needs a field in cyw43_utk status */
		st.mode         = comms_get_mode();
		st.faults       = fault_get();
		/* Read-and-clear in one atomic step, so a drop is reported exactly once. */
		st.tx_drop      = (tk_wai_flg(flg_comms, CF_TX_DROP, TWF_ORW | TWF_BITCLR,
					      &ptn, TMO_POL) == E_OK);

		len = comms_enc_status(buf, sizeof buf, &st);
		if (len != 0U) {
			(void)comms_publish(COMMS_TOPIC_STATUS, buf, len);
			seq = (UB)((seq + 1U) & SEQ_MASK);
			s_hb_count++;
		}
	}
}
