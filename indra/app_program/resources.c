/*
 *	resources.c
 *	Run-time proof of the resource allocation.  At boot, asks the kernel
 *	which object IDs really exist (tk_ref_*), compares with the budget in
 *	car_config.h and with the CNF_MAX_* limits, and prints a table.
 *	Also re-checks the task priorities and cores that were created.
 */
#include "car.h"

typedef ER (*REF_FN)(ID id, void *pk);

LOCAL INT count_objects(REF_FN fn, INT max, void *buf)
{
	INT id, n = 0;
	for (id = 1; id <= max; id++) {
		if (fn(id, buf) == E_OK) n++;
	}
	return n;
}

LOCAL BOOL row(const char *what, INT used, INT ours, INT max)
{
	BOOL ok = used <= max && ours <= used;
	tm_printf((UB*)"  %-15s %3d used (ours %2d, kernel %2d) of %3d  %s\n",
		  what, used, ours, used - ours, max, ok ? "OK" : "OVER");
	return ok;
}

EXPORT void resources_report(void)
{
	union {
		T_RTSK t; T_RSEM s; T_RFLG f; T_RMTX x; T_RMBX b;
		T_RMBF m; T_RMPF p; T_RMPL l; T_RCYC c; T_RALM a;
	} u;
	BOOL ok = TRUE;
	INT id;

	tm_printf((UB*)"\n=== IS27 resource check (kernel's own view) ===\n");
	ok &= row("tasks",         count_objects((REF_FN)tk_ref_tsk, CNF_MAX_TSKID, &u), APP_NUM_TASKS, CNF_MAX_TSKID);
	ok &= row("event flags",   count_objects((REF_FN)tk_ref_flg, CNF_MAX_FLGID, &u), APP_NUM_FLAGS, CNF_MAX_FLGID);
	ok &= row("semaphores",    count_objects((REF_FN)tk_ref_sem, CNF_MAX_SEMID, &u), APP_NUM_SEMS,  CNF_MAX_SEMID);
	ok &= row("mutexes",       count_objects((REF_FN)tk_ref_mtx, CNF_MAX_MTXID, &u), APP_NUM_MTXS,  CNF_MAX_MTXID);
	ok &= row("mailboxes",     count_objects((REF_FN)tk_ref_mbx, CNF_MAX_MBXID, &u), APP_NUM_MBXS,  CNF_MAX_MBXID);
	ok &= row("msg buffers",   count_objects((REF_FN)tk_ref_mbf, CNF_MAX_MBFID, &u), APP_NUM_MBFS,  CNF_MAX_MBFID);
	ok &= row("fixed pools",   count_objects((REF_FN)tk_ref_mpf, CNF_MAX_MPFID, &u), APP_NUM_MPFS,  CNF_MAX_MPFID);
	ok &= row("var pools",     count_objects((REF_FN)tk_ref_mpl, CNF_MAX_MPLID, &u), APP_NUM_MPLS,  CNF_MAX_MPLID);
	ok &= row("cyclic hdlrs",  count_objects((REF_FN)tk_ref_cyc, CNF_MAX_CYCID, &u), APP_NUM_CYCS,  CNF_MAX_CYCID);
	ok &= row("alarm hdlrs",   count_objects((REF_FN)tk_ref_alm, CNF_MAX_ALMID, &u), APP_NUM_ALMS,  CNF_MAX_ALMID);

	tm_printf((UB*)"  tasks running (id: priority):");
	for (id = 1; id <= CNF_MAX_TSKID; id++) {
		if (tk_ref_tsk(id, &u.t) == E_OK)
			tm_printf((UB*)" %d:%d", id, (INT)u.t.tskpri);
	}
	tm_printf((UB*)"\n  ours (priority/core): motion %d/0  mission %d/1  line %d/1  ranger %d/1"
		  "  imu %d/1  barcode %d/0  comms %d/0  status %d/0\n",
		  PRI_MOTION, PRI_MISSION, PRI_LINE, PRI_RANGER, PRI_IMU, PRI_BARCODE, PRI_COMMS, PRI_STATUS);
	tm_printf((UB*)"  PWM slices: 0 = ultrasonic echo, 4 = left motor, 5 = right motor, 6 = servo, 3 = buzzer; PIO0 SM3 = RGB LEDs (radio uses PIO1)\n");
#if BARCODE_ANALOG
	tm_printf((UB*)"  ADC scan (FIFO IRQ 22, core 0): GP%d GP%d = line sensors, GP%d = barcode A0   I2C0 = IMU (imu task)\n",
		  PIN_LINE_L, PIN_LINE_R, PIN_BARCODE_ADC);
	tm_printf((UB*)"  GPIO IRQ 13: shared dispatcher (encoders GP%d GP%d GP%d GP%d)\n",
		  PIN_ENC_R_A, PIN_ENC_R_B, PIN_ENC_L_A, PIN_ENC_L_B);
#else
	tm_printf((UB*)"  ADC0/1 = line sensors (line task)   I2C0 = IMU (imu task)\n");
	tm_printf((UB*)"  GPIO IRQ 13: shared dispatcher (encoders GP%d GP%d GP%d GP%d, barcode GP%d)\n",
		  PIN_ENC_R_A, PIN_ENC_R_B, PIN_ENC_L_A, PIN_ENC_L_B, PIN_BARCODE);
#endif
#if TM_NET_LWIP
	tm_printf((UB*)"  network build: WiFi + MQTT run in the kernel's WiFi task (core 0)\n");
#else
	tm_printf((UB*)"  offline build (NET=none): telemetry is not sent\n");
#endif
	tm_printf((UB*)"=== %s ===\n", ok ? "ALL WITHIN LIMITS" : "OVER A LIMIT: check car_config.h");
}

/* ------------------------------------------------------------------
 *  Live usage for the dashboard (robot/tlm/resources)
 * ------------------------------------------------------------------ */
EXPORT void resources_usage(RES_USAGE *r);

/* CPU load meters: one task per core at the LOWEST priority.  It only
   runs when nothing else wants that core.  It adds up the time it gets
   in unbroken stretches (a gap > 50 us = something else ran), so
   load = 100 % - its share of the last second.  No calibration needed. */
LOCAL volatile UW idle_us[2], idle_win_us[2];
LOCAL volatile UINT cpu_pct[2] = {100, 100};

LOCAL void idle_task(INT core, void *exinf)
{
	UW last = now_us(), win0 = last, acc = 0;
	(void)exinf;
	for (;;) {
		UW t = now_us(), d = t - last;
		if (d < 50U) acc += d;			/* we had the CPU all along */
		last = t;
		if ((t - win0) >= 1000000U) {		/* every second */
			UW win = t - win0;
			cpu_pct[core] = (UINT)(100U - (acc > win ? 100U : (acc * 100U) / win));
			acc = 0;
			win0 = t;
		}
	}
}

EXPORT ER resources_start(void)
{
	T_CTSK c = {0};
	INT core;
	ID tid;
	ER er;

	for (core = 0; core < 2; core++) {
		c.task = (FP)idle_task;
		c.itskpri = PRI_IDLE;
		c.stksz = 512;				/* no printing here */
		c.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
		c.tskatr |= TA_ASSPRC;
		c.assprc = core ? TP_PRC2 : TP_PRC1;
#endif
		if ((tid = tk_cre_tsk(&c)) < E_OK) return tid;
		if ((er = tk_sta_tsk(tid, core)) < E_OK) return er;	/* stacd = core */
	}
	return E_OK;
}

EXPORT void resources_usage(RES_USAGE *r)
{
	union { T_RTSK t; T_RSEM s; T_RFLG f; T_RMBF m; T_RCYC c; } u;
	r->tasks = (UINT)count_objects((REF_FN)tk_ref_tsk, CNF_MAX_TSKID, &u);
	r->flags = (UINT)count_objects((REF_FN)tk_ref_flg, CNF_MAX_FLGID, &u);
	r->sems  = (UINT)count_objects((REF_FN)tk_ref_sem, CNF_MAX_SEMID, &u);
	r->mbfs  = (UINT)count_objects((REF_FN)tk_ref_mbf, CNF_MAX_MBFID, &u);
	r->cycs  = (UINT)count_objects((REF_FN)tk_ref_cyc, CNF_MAX_CYCID, &u);
	r->cpu_pct[0] = cpu_pct[0];
	r->cpu_pct[1] = cpu_pct[1];
}
