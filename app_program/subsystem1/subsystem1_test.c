/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 *
 *	These tests need NO network: they exercise the queues, flags and tasks
 *	directly.  The WiFi/MQTT behaviour is tested by hand (S1-M01...).
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#define HAMMER_ROUNDS	(500U)
#define FLG_DONE_A	(1U << 0)
#define FLG_DONE_B	(1U << 1)

static ID	flg_done;	/* hammer tasks report completion here */

/* Two tasks, one on each core, repeatedly raise/clear DIFFERENT bits of the
 * shared fault mask at the same time.  If the update were a plain
 * read-modify-write, bits would be lost.  Task A raises FAULT_IMU once and
 * leaves it; task B toggles FAULT_NO_ECHO and ends cleared. */
static void hammer_a(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;
	fault_raise(FAULT_IMU);
	{
		UINT i;
		for (i = 0U; i < HAMMER_ROUNDS; i++) {
			fault_raise(FAULT_MOTOR_STALL);
			fault_clear(FAULT_MOTOR_STALL);
		}
	}
	(void)tk_set_flg(flg_done, FLG_DONE_A);
	tk_exd_tsk();
}

static void hammer_b(INT stacd, void *exinf)
{
	UINT i;
	(void)stacd; (void)exinf;
	for (i = 0U; i < HAMMER_ROUNDS; i++) {
		fault_raise(FAULT_NO_ECHO);
		fault_clear(FAULT_NO_ECHO);
	}
	(void)tk_set_flg(flg_done, FLG_DONE_B);
	tk_exd_tsk();
}

static void drain_queue(void)
{
	UB t;
	UB f[COMMS_FRAME_MAX];
	while (comms_test_pop(&t, f, sizeof f) > 0) {
		/* discard */
	}
}

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	T_RFLG	rflg;
	T_CFLG	cflg = {0};
	ER	er;

	TEST_INIT();

	/* S1-U01: the cyclic handler exists and is running */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U01", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "cyclic handler is running");

	/* S1-U02: between ticks the LED task is asleep, not busy-looping. */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);	/* not a multiple of the tick */
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9, "LED task sleeps between ticks");
	}

	/* From here on the MQTT pump must not steal frames from the queue. */
	comms_test_hold(TRUE);
	tk_dly_tsk(50);
	drain_queue();

	/* S1-U10: frame loopback  publish -> queue -> pop -> decode */
	{
		status_frame_t in = { 9U, 1234U, 61U, SYS_MODE_AVOID, FAULT_AVOID | FAULT_IMU, TRUE };
		status_frame_t out;
		UB buf[COMMS_FRAME_MAX], topic = 0xFFU, rx[COMMS_FRAME_MAX];
		UINT n = comms_enc_status(buf, sizeof buf, &in);
		INT got;

		er = comms_publish(COMMS_TOPIC_STATUS, buf, n);
		got = comms_test_pop(&topic, rx, sizeof rx);
		TEST_CHECK("S1-U10", er == E_OK && got == (INT)n && topic == COMMS_TOPIC_STATUS &&
			   comms_dec_status(rx, (uint16_t)got, &out) &&
			   out.seq == in.seq && out.up_s == in.up_s && out.mode == in.mode &&
			   out.faults == in.faults && out.tx_drop == in.tx_drop,
			   "status frame survives publish -> queue -> pop -> decode");
	}

	/* S1-U11: bad arguments are rejected, nothing queued */
	{
		UB b[2] = { 1U, 2U };
		UB t, f[COMMS_FRAME_MAX];
		BOOL bad = comms_publish(COMMS_TOPIC_COUNT, b, 2U) == E_PAR &&
			   comms_publish(COMMS_TOPIC_STATUS, b, 0U) == E_PAR &&
			   comms_publish(COMMS_TOPIC_STATUS, b, COMMS_FRAME_MAX + 1U) == E_PAR &&
			   comms_publish(COMMS_TOPIC_STATUS, NULL, 2U) == E_PAR;
		TEST_CHECK("S1-U11", bad && comms_test_pop(&t, f, sizeof f) == 0,
			   "bad topic / length / pointer rejected, queue untouched");
	}

	/* S1-U12: a full queue never blocks the caller; the drop is recorded */
	{
		UB b[ST_FRAME_BYTES] = {0};
		UINT i, accepted = 0U;
		UW t0, t1;
		(void)tk_clr_flg(flg_comms, ~(UINT)CF_TX_DROP);
		t0 = comms_uptime_ms();
		for (i = 0U; i < 100U; i++) {
			if (comms_publish(COMMS_TOPIC_STATUS, b, ST_FRAME_BYTES) == E_OK) {
				accepted++;
			}
		}
		t1 = comms_uptime_ms();
		tk_ref_flg(flg_comms, &rflg);
		TEST_CHECK("S1-U12", accepted > 0U && accepted < 100U && (t1 - t0) < 50U &&
			   (rflg.flgptn & CF_TX_DROP) != 0U,
			   "100 publishes into a full queue return at once, CF_TX_DROP set");
		drain_queue();
	}

	/* S1-U13: fault bitmask semantics */
	{
		fault_clear(FAULT_MASK_ALL);
		fault_raise(FAULT_IMU | FAULT_NO_ECHO);
		BOOL a = (fault_get() == (FAULT_IMU | FAULT_NO_ECHO));
		fault_clear(FAULT_IMU);
		BOOL b = (fault_get() == FAULT_NO_ECHO);
		fault_raise(0xF000U);	/* bits outside the 12 valid ones are ignored */
		BOOL c = (fault_get() == FAULT_NO_ECHO);
		TEST_CHECK("S1-U13", a && b && c, "fault_raise / fault_clear / fault_get, invalid bits ignored");
		fault_clear(FAULT_MASK_ALL);
		drain_queue();
	}

	/* S1-U14: raising a NEW fault publishes one event frame; re-raising does not */
	{
		UB t, f[COMMS_FRAME_MAX];
		fault_frame_t ev;
		INT n1, n2;
		fault_raise(FAULT_AVOID);
		n1 = comms_test_pop(&t, f, sizeof f);
		BOOL ok1 = (n1 == (INT)FE_FRAME_BYTES) && (t == COMMS_TOPIC_FAULT) &&
			   comms_dec_fault(f, (uint16_t)n1, &ev) && ev.raised == FAULT_AVOID;
		fault_raise(FAULT_AVOID);	/* already set */
		n2 = comms_test_pop(&t, f, sizeof f);
		TEST_CHECK("S1-U14", ok1 && n2 == 0, "new fault -> exactly one event frame with the right bit");
		fault_clear(FAULT_MASK_ALL);
		drain_queue();
	}

	/* S1-U15: shared fault bitmask is race-free across both cores */
	{
		UINT ptn = 0U;
		T_CTSK ct = {0};
		ID ta, tb;
		cflg.flgatr = TA_TFIFO | TA_WMUL;
		flg_done = tk_cre_flg(&cflg);
		fault_clear(FAULT_MASK_ALL);

		ct.itskpri = S1_PRI_HB;
		ct.stksz   = TASK_STACK_SIZE;
		ct.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
		ct.tskatr |= TA_ASSPRC;
		ct.assprc  = TP_PRC1;
#endif
		ct.task = (FP)hammer_a;
		ta = tk_cre_tsk(&ct);
#if TK_SUPPORT_SMP
		ct.assprc  = TP_PRC2;
#endif
		ct.task = (FP)hammer_b;
		tb = tk_cre_tsk(&ct);
		tk_sta_tsk(ta, 0);
		tk_sta_tsk(tb, 0);
		er = tk_wai_flg(flg_done, FLG_DONE_A | FLG_DONE_B, TWF_ANDW, &ptn, 5000);
		TEST_CHECK("S1-U15", er == E_OK && fault_get() == FAULT_IMU,
			   "concurrent raise/clear of different bits from both cores loses no update");
		tk_del_flg(flg_done);
		fault_clear(FAULT_MASK_ALL);
		drain_queue();
	}

	/* S1-U16: valid commands reach mission control; invalid ones are rejected */
	{
		cmd_frame_t c;
		UB b[1];
		UINT ptn = 0U;
		UW ok0 = comms_cmd_count(), bad0 = comms_cmd_reject_count();
		(void)tk_clr_flg(flg_cmd, 0U);

		c.code = CMD_START; c.id = 3U;
		comms_enc_cmd(b, sizeof b, &c);
		comms_port_rx_push(b, 1U);
		er = tk_wai_flg(flg_cmd, CMDF_START, TWF_ORW | TWF_BITCLR, &ptn, 300);
		BOOL a = (er == E_OK);

		b[0] = 0x07U;	/* code 7: invalid */
		comms_port_rx_push(b, 1U);
		b[0] = 0x00U;	/* code 0: invalid */
		comms_port_rx_push(b, 1U);
		{
			UB two[2] = { 0x39U, 0x39U };	/* wrong length */
			comms_port_rx_push(two, 2U);
		}
		tk_dly_tsk(150);
		tk_ref_flg(flg_cmd, &rflg);
		BOOL b2 = (rflg.flgptn == 0U) && (comms_cmd_count() == ok0 + 1U) &&
			  (comms_cmd_reject_count() == bad0 + 3U);
		TEST_CHECK("S1-U16", a && b2, "START forwarded; codes 0 and 7 and a 2-byte frame rejected");
	}

	/* S1-U17: heartbeat_task keeps running (>= 2 beats in 2.5 s) */
	{
		UW h0 = comms_hb_count();
		tk_dly_tsk(2500);
		TEST_CHECK("S1-U17", (comms_hb_count() - h0) >= 2U, "heartbeat_task builds a status frame every ~1 s");
		drain_queue();
	}

	/* S1-U18: a mode change and a PING each produce a status frame within 100 ms */
	{
		UB t, f[COMMS_FRAME_MAX];
		status_frame_t s;
		INT n;
		BOOL mode_ok = FALSE, ping_ok = FALSE;
		UINT tries;
		cmd_frame_t c = { CMD_PING, 1U };
		UB b[1];

		drain_queue();
		comms_set_mode(SYS_MODE_LINE);
		tk_dly_tsk(100);
		for (tries = 0U; tries < 4U; tries++) {
			n = comms_test_pop(&t, f, sizeof f);
			if (n > 0 && t == COMMS_TOPIC_STATUS && comms_dec_status(f, (uint16_t)n, &s) &&
			    s.mode == SYS_MODE_LINE) {
				mode_ok = TRUE;
			}
		}
		drain_queue();
		comms_enc_cmd(b, sizeof b, &c);
		comms_port_rx_push(b, 1U);
		tk_dly_tsk(100);
		ping_ok = (comms_test_pop(&t, f, sizeof f) == (INT)ST_FRAME_BYTES) && (t == COMMS_TOPIC_STATUS);
		TEST_CHECK("S1-U18", mode_ok && ping_ok, "mode change and PING each trigger a status frame within 100 ms");
		comms_set_mode(SYS_MODE_INIT);
		drain_queue();
	}

	comms_test_hold(FALSE);
	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
