/*
 *	comms.c
 *	Subsystem 1 -- communication core: kernel objects, the public API, and the
 *	bridge functions the MQTT client calls (comms_port.h).
 *
 *	Race-safety rule (Change Requirement 11): several flags/bits share one
 *	word, so a plain "x |= bit" from two tasks (or a task and an ISR) can lose
 *	an update.  Here every shared packed word is a kernel EVENT FLAG, whose
 *	set/clear/wait operations the kernel performs atomically, on either core.
 *	Single bytes and counters that have exactly ONE writing task are plain
 *	volatile variables; their comments name the writer.
 */

#include "subsystem1.h"
#include "comms_port.h"

EXPORT ID	flg_comms;
EXPORT ID	flg_fault;
EXPORT ID	flg_cmd;
EXPORT ID	mbf_pub;
EXPORT ID	mbf_rx;

/* Sizes.  A message-buffer entry costs its length rounded up to 4 bytes plus a
 * 4-byte header, so a 13-byte entry (topic + 12-byte frame) costs 20 bytes. */
#define PUB_SLOTS	(16U)
#define PUB_ENTRY_COST	(20U)
#define RX_SLOTS	(8U)
#define RX_ENTRY_COST	(16U)	/* 8-byte payload + 4-byte header, rounded */

#define OTM_MS_MASK	(0xFFFFU)	/* fault frames carry the low 16 bits of ms */
#define MS_PER_S	(1000U)

static volatile UB	s_mode = SYS_MODE_INIT;	/* single byte; writer: mission_ctrl_task */
#if APP_TEST
static volatile BOOL	s_tx_hold = FALSE;	/* writer: the test task only */
#endif

/* ---------------------------------------------------------------------- */

EXPORT ER comms_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_comms = tk_cre_flg(&cflg);
	if (flg_comms < E_OK) {
		return flg_comms;
	}
	flg_fault = tk_cre_flg(&cflg);
	if (flg_fault < E_OK) {
		return flg_fault;
	}
	flg_cmd = tk_cre_flg(&cflg);
	if (flg_cmd < E_OK) {
		return flg_cmd;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (INT)(PUB_SLOTS * PUB_ENTRY_COST);
	cmbf.maxmsz = (INT)(COMMS_FRAME_MAX + 1U);
	mbf_pub = tk_cre_mbf(&cmbf);
	if (mbf_pub < E_OK) {
		return mbf_pub;
	}

	cmbf.bufsz  = (INT)(RX_SLOTS * RX_ENTRY_COST);
	cmbf.maxmsz = (INT)COMMS_RX_MAX;
	mbf_rx = tk_cre_mbf(&cmbf);
	return (mbf_rx < E_OK) ? mbf_rx : E_OK;
}

/* ---------------------------------------------------------------------- */
/*  Public API                                                             */
/* ---------------------------------------------------------------------- */

EXPORT ER comms_publish(UB topic, const UB *frame, UINT len)
{
	UB	msg[COMMS_FRAME_MAX + 1U];
	UINT	i;
	ER	er;

	if ((topic >= COMMS_TOPIC_COUNT) || (frame == NULL) ||
	    (len == 0U) || (len > COMMS_FRAME_MAX)) {
		return E_PAR;
	}

	msg[0] = topic;
	for (i = 0U; i < len; i++) {
		msg[i + 1U] = frame[i];
	}

	er = tk_snd_mbf(mbf_pub, msg, (INT)(len + 1U), TMO_POL);
	if (er == E_TMOUT) {
		/* Queue full: never wait (NFR-B1-4).  Remember it so the next status
		 * frame tells the dashboard that telemetry was lost. */
		(void)tk_set_flg(flg_comms, CF_TX_DROP);
	}
	return er;
}

EXPORT UW comms_uptime_ms(void)
{
	SYSTIM	otm;

	(void)tk_get_otm(&otm);		/* operating time: ms since boot, as hi:lo */
	return otm.lo;
}

EXPORT BOOL comms_is_up(void)
{
	T_RFLG	rflg;

	if (tk_ref_flg(flg_comms, &rflg) != E_OK) {
		return FALSE;
	}
	return ((rflg.flgptn & CF_MQTT_UP) != 0U) && ((rflg.flgptn & CF_CONN_LOST) == 0U);
}

EXPORT void comms_set_mode(UB mode)
{
	if (mode > SYS_MODE_MAX) {
		return;
	}
	s_mode = mode;
	(void)tk_set_flg(flg_comms, CF_STATUS_NOW);	/* wakes heartbeat_task */
}

EXPORT UB comms_get_mode(void)
{
	return s_mode;
}

EXPORT void fault_raise(UH mask)
{
	T_RFLG		rflg;
	UINT		fresh;
	fault_frame_t	ff;
	UB		buf[COMMS_FRAME_MAX];
	UINT		len;

	mask &= FAULT_MASK_ALL;
	if (mask == 0U) {
		return;
	}

	/* Read first so we can tell which bits are new.  Two tasks raising the same
	 * bit at the same instant may both publish an event: harmless.  The set
	 * itself is atomic, so no bit is ever lost. */
	fresh = mask;
	if (tk_ref_flg(flg_fault, &rflg) == E_OK) {
		fresh = mask & ~rflg.flgptn;
	}
	(void)tk_set_flg(flg_fault, mask);

	if (fresh != 0U) {
		ff.t_ms   = (UH)(comms_uptime_ms() & OTM_MS_MASK);
		ff.raised = (UH)fresh;
		len = comms_enc_fault(buf, sizeof buf, &ff);
		if (len != 0U) {
			(void)comms_publish(COMMS_TOPIC_FAULT, buf, len);
		}
	}
}

EXPORT void fault_clear(UH mask)
{
	/* tk_clr_flg ANDs the pattern with its argument, atomically. */
	(void)tk_clr_flg(flg_fault, ~(UINT)mask);
}

EXPORT UH fault_get(void)
{
	T_RFLG	rflg;

	if (tk_ref_flg(flg_fault, &rflg) != E_OK) {
		return 0U;
	}
	return (UH)(rflg.flgptn & FAULT_MASK_ALL);
}

/* ---------------------------------------------------------------------- */
/*  Bridge to the MQTT client (called only from cyw43_utk_task)            */
/* ---------------------------------------------------------------------- */

static volatile UB	s_pub_fail_streak;	/* writer: cyw43_utk_task only */
static volatile UW	s_cmd_ok;		/* writer: cmd_dispatch_task only */
static volatile UW	s_cmd_bad;		/* writer: cmd_dispatch_task only */

static INT tx_pop_raw(UB *topic, UB *frame, UINT cap)
{
	UB	msg[COMMS_FRAME_MAX + 1U];
	INT	n;
	UINT	i;

	n = tk_rcv_mbf(mbf_pub, msg, TMO_POL);
	if (n < 2) {		/* empty (E_TMOUT) or an impossible 0/1-byte entry */
		return 0;
	}
	if (((UINT)n - 1U) > cap) {
		return 0;	/* cannot happen with COMMS_FRAME_MAX buffers; drop it */
	}
	*topic = msg[0];
	for (i = 0U; i < ((UINT)n - 1U); i++) {
		frame[i] = msg[i + 1U];
	}
	return n - 1;
}

uint16_t comms_port_tx_pop(uint8_t *topic, uint8_t *frame, uint16_t cap)
{
#if APP_TEST
	if (s_tx_hold) {
		return 0U;
	}
#endif
	return (uint16_t)tx_pop_raw(topic, frame, cap);
}

void comms_port_rx_push(const uint8_t *frame, uint16_t len)
{
	if ((frame == NULL) || (len == 0U) || (len > COMMS_RX_MAX)) {
		return;
	}
	/* Non-blocking; if the dispatcher is behind, the newest command is dropped
	 * rather than stalling the network task. */
	(void)tk_snd_mbf(mbf_rx, frame, (INT)len, TMO_POL);
}

void comms_port_mqtt_state(uint8_t up)
{
	if (up != 0U) {
		UB	topic;
		UB	frame[COMMS_FRAME_MAX];
		UINT	i;

		/* Frames queued before the session existed are stale (the queue fills
		 * from boot, but the MQTT pump only starts once WiFi is up).  Publishing
		 * them in one burst also overflows lwIP's small MQTT output buffer
		 * (publish err=-1).  Drop them; one fresh status follows below. */
		for (i = 0U; i < PUB_SLOTS; i++) {
			if (tx_pop_raw(&topic, frame, sizeof frame) <= 0) {
				break;
			}
		}

		s_pub_fail_streak = 0U;
		(void)tk_set_flg(flg_comms, CF_MQTT_UP | CF_STATUS_NOW);	/* report at once */
	} else {
		(void)tk_clr_flg(flg_comms, ~(UINT)CF_MQTT_UP);
	}
}

void comms_port_publish_result(uint8_t ok)
{
	if (ok != 0U) {
		s_pub_fail_streak = 0U;
	} else if (s_pub_fail_streak < UINT8_MAX) {
		s_pub_fail_streak++;
	} else {
		/* saturated */
	}
}

/* Used by heartbeat_task: how many publishes in a row failed. */
EXPORT UB comms_pub_fail_streak(void)
{
	return s_pub_fail_streak;
}

/* Counters written by cmd_dispatch_task (single writer each) */
EXPORT void comms_cmd_note(BOOL accepted)
{
	if (accepted) {
		s_cmd_ok++;
	} else {
		s_cmd_bad++;
	}
}
EXPORT UW comms_cmd_count(void)		{ return s_cmd_ok; }
EXPORT UW comms_cmd_reject_count(void)	{ return s_cmd_bad; }

#if APP_TEST
EXPORT INT comms_test_pop(UB *topic, UB *frame, UINT cap)
{
	return tx_pop_raw(topic, frame, cap);
}

EXPORT void comms_test_hold(BOOL hold)
{
	s_tx_hold = hold;
}
#endif
