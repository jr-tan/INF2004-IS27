/*
 *	task_cmd_dispatch.c
 *	Subsystem 1 -- cmd_dispatch_task: validates inbound robot/cmd frames and
 *	passes START / STOP / PAUSE / RESUME on to mission control as event-flag
 *	bits.  Mission control, not this task, decides whether a command applies in
 *	the current mode (Design Review section 3).
 *
 *	A command only reaches this task while the MQTT session is up, because the
 *	frames come from the MQTT client itself.  Commands are never queued across
 *	an outage: a stale START must not fire late.
 */

#include "subsystem1.h"

#define CMD_WAIT_MS	(500)	/* wake at least every 500 ms (watchdog alive, NFR-SYS-1) */

EXPORT void cmd_dispatch_task(INT stacd, void *exinf)
{
	UB		buf[COMMS_RX_MAX];
	cmd_frame_t	cmd;
	INT		n;

	(void)stacd; (void)exinf;

	for (;;) {
		n = tk_rcv_mbf(mbf_rx, buf, CMD_WAIT_MS);
		if (n == E_TMOUT) {
			continue;		/* idle: nothing to do */
		}
		if (n < 0) {
			(void)tk_dly_tsk(CMD_WAIT_MS);	/* object error: do not spin */
			continue;
		}

		if (!comms_dec_cmd(buf, (uint16_t)n, &cmd)) {
			comms_cmd_note(FALSE);
			tm_printf((UB*)"comms: bad command frame (%d bytes, first=0x%02x) discarded\n",
				  n, (UINT)buf[0]);
			continue;
		}

		comms_cmd_note(TRUE);
		tm_printf((UB*)"comms: cmd %u id=%u\n", (UINT)cmd.code, (UINT)cmd.id);

		switch (cmd.code) {
		case CMD_START:
			(void)tk_set_flg(flg_cmd, CMDF_START);
			break;
		case CMD_STOP:
			(void)tk_set_flg(flg_cmd, CMDF_STOP);
			break;
		case CMD_PAUSE:
			(void)tk_set_flg(flg_cmd, CMDF_PAUSE);
			break;
		case CMD_RESUME:
			(void)tk_set_flg(flg_cmd, CMDF_RESUME);
			break;
		case CMD_PING:
			(void)tk_set_flg(flg_comms, CF_STATUS_NOW);	/* heartbeat_task answers */
			break;
		default:
			/* comms_dec_cmd() already rejected everything else */
			break;
		}
	}
}
