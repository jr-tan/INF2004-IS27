/**
 * @file    resources.c
 *  resources.c
 *  Run-time proof of the resource allocation.  At boot, asks the kernel
 *  which object IDs really exist (tk_ref_*), compares with the budget in
 *  car_config.h and with the CNF_MAX_* limits, and prints a table.
 *  Also re-checks the task priorities and cores that were created.
 */
#include "car.h"
#include "resources.h" /* own header (BARR-C 4.3.c) */

/* micro T-Kernel object IDs start at 1; the RP2040 has 2 cores
   (loop bounds, BARR-C 8.4.a). */
#define FIRST_OBJ_ID 1
#define CORE_COUNT   2

typedef ER (*ref_fn_t)(ID id, void * pk);

/**
 * @brief  Count objects.
 * @return Result as described above.
 */
LOCAL int32_t count_objects (ref_fn_t fn_val, int32_t max, void * p_buf)
{
    int32_t obj_id;
    int32_t num = 0;
    for (obj_id = FIRST_OBJ_ID; obj_id <= max; obj_id++)
    {
        if (E_OK == fn_val(obj_id, p_buf))
        {
            num++;
        }
    }
    return num;
}

/**
 * @brief  Row.
 * @return Result as described above.
 */
LOCAL BOOL row (const char * p_what, int32_t used, int32_t ours, int32_t max)
{
    BOOL b_ok = (used <= max) && (ours <= used);
    tm_printf((uint8_t *)"  %-15s %3d used (ours %2d, kernel %2d) of %3d  %s\n",
              p_what, used, ours, used - ours, max, b_ok ? "OK" : "OVER");
    return b_ok;
}

/**
 * @brief  Resources report.
 */
EXPORT void resources_report (void)
{
    union
    {
        T_RTSK t;
        T_RSEM s;
        T_RFLG f;
        T_RMTX x;
        T_RMBX b;
        T_RMBF m;
        T_RMPF p;
        T_RMPL l;
        T_RCYC c;
        T_RALM a;
    } uval;
    BOOL    b_ok = TRUE;
    int32_t obj_id;

    tm_printf((uint8_t *)"\n=== IS27 resource check (kernel's own view) ===\n");
    b_ok &=
        row("tasks", count_objects((ref_fn_t)tk_ref_tsk, CNF_MAX_TSKID, &uval),
            APP_NUM_TASKS, CNF_MAX_TSKID);
    b_ok &= row("event flags",
                count_objects((ref_fn_t)tk_ref_flg, CNF_MAX_FLGID, &uval),
                APP_NUM_FLAGS, CNF_MAX_FLGID);
    b_ok &= row("semaphores",
                count_objects((ref_fn_t)tk_ref_sem, CNF_MAX_SEMID, &uval),
                APP_NUM_SEMS, CNF_MAX_SEMID);
    b_ok &= row("mutexes",
                count_objects((ref_fn_t)tk_ref_mtx, CNF_MAX_MTXID, &uval),
                APP_NUM_MTXS, CNF_MAX_MTXID);
    b_ok &= row("mailboxes",
                count_objects((ref_fn_t)tk_ref_mbx, CNF_MAX_MBXID, &uval),
                APP_NUM_MBXS, CNF_MAX_MBXID);
    b_ok &= row("msg buffers",
                count_objects((ref_fn_t)tk_ref_mbf, CNF_MAX_MBFID, &uval),
                APP_NUM_MBFS, CNF_MAX_MBFID);
    b_ok &= row("fixed pools",
                count_objects((ref_fn_t)tk_ref_mpf, CNF_MAX_MPFID, &uval),
                APP_NUM_MPFS, CNF_MAX_MPFID);
    b_ok &= row("var pools",
                count_objects((ref_fn_t)tk_ref_mpl, CNF_MAX_MPLID, &uval),
                APP_NUM_MPLS, CNF_MAX_MPLID);
    b_ok &= row("cyclic hdlrs",
                count_objects((ref_fn_t)tk_ref_cyc, CNF_MAX_CYCID, &uval),
                APP_NUM_CYCS, CNF_MAX_CYCID);
    b_ok &= row("alarm hdlrs",
                count_objects((ref_fn_t)tk_ref_alm, CNF_MAX_ALMID, &uval),
                APP_NUM_ALMS, CNF_MAX_ALMID);

    tm_printf((uint8_t *)"  tasks running (id: priority):");
    for (obj_id = FIRST_OBJ_ID; obj_id <= CNF_MAX_TSKID; obj_id++)
    {
        if (E_OK == tk_ref_tsk(obj_id, &uval.t))
        {
            tm_printf((uint8_t *)" %d:%d", obj_id, (int32_t)uval.t.tskpri);
        }
    }
    tm_printf(
        (uint8_t *)"\n  ours (priority/core): motion %d/0  mission %d/1  line "
                   "%d/1  ranger %d/1"
                   "  imu %d/1  barcode %d/0  comms %d/0  status %d/0\n",
        PRI_MOTION, PRI_MISSION, PRI_LINE, PRI_RANGER, PRI_IMU, PRI_BARCODE,
        PRI_COMMS, PRI_STATUS);
    tm_printf(
        (uint8_t *)"  PWM slices: 0 = ultrasonic echo, 4 = left motor, 5 = "
                   "right motor, 6 = servo, 3 = buzzer; PIO0 SM3 = RGB LEDs "
                   "(radio uses PIO1)\n");
    tm_printf((uint8_t *)"  ADC0/1 = line sensors (line task)   I2C0 = IMU "
                         "(imu task)\n");
    tm_printf(
        (uint8_t *)"  GPIO IRQ 13: shared dispatcher (encoders GP0 GP1 GP7 "
                   "GP28, barcode GP2)\n");
#if TM_NET_LWIP
    tm_printf(
        (uint8_t *)"  network build: WiFi + MQTT run in the kernel's WiFi "
                   "task (core 0)\n");
#else
    tm_printf((uint8_t *)"  offline build (NET=none): telemetry is not sent\n");
#endif
    tm_printf((uint8_t *)"=== %s ===\n",
              b_ok ? "ALL WITHIN LIMITS" : "OVER A LIMIT: check car_config.h");
}

/* ------------------------------------------------------------------
 *  Live usage for the dashboard (robot/tlm/resources)
 * ------------------------------------------------------------------ */
EXPORT void resources_usage (res_usage_t * p_res);

/* CPU load meters: one task per core at the LOWEST priority.  It only
   runs when nothing else wants that core.  It adds up the time it gets
   in unbroken stretches (a gap > 50 us = something else ran), so
   load = 100 % - its share of the last second.  No calibration needed. */
LOCAL volatile uint32_t g_idle_us[2];
LOCAL volatile uint32_t g_idle_win_us[2];
LOCAL volatile uint32_t g_cpu_pct[2] = {100, 100};

/**
 * @brief  Task body (micro T-Kernel task entry).
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void idle_task (int32_t core, void * p_exinf)
{
    uint32_t last = motion_now_us();
    uint32_t win0 = last;
    uint32_t acc  = 0;
    (void)p_exinf;
    for (;;)
    {
        uint32_t tval = motion_now_us();
        uint32_t dval = tval - last;
        if (dval < 50U)
        {
            acc += dval; /* we had the CPU all along */
        }
        last = tval;
        if ((tval - win0) >= 1000000U)
        { /* every second */
            uint32_t win = tval - win0;
            g_cpu_pct[core] =
                (uint32_t)(100U - ((acc > win) ? 100U : (acc * 100U) / win));
            acc  = 0;
            win0 = tval;
        }
    }
}

/**
 * @brief  Start the module's task(s).
 * @return Result as described above.
 */
EXPORT ER resources_start (void)
{
    T_CTSK  cval = {0};
    int32_t core;
    ID      tid;
    ER      err;

    for (core = 0; core < CORE_COUNT; core++)
    {
        cval.task    = (FP)idle_task;
        cval.itskpri = PRI_IDLE;
        cval.stksz   = 512; /* no printing here */
        cval.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
        cval.tskatr |= TA_ASSPRC;
        cval.assprc = core ? TP_PRC2 : TP_PRC1;
#endif
        tid = tk_cre_tsk(&cval);
        if (tid < E_OK)
        {
            return tid;
        }
        err = tk_sta_tsk(tid, core);
        if (err < E_OK)
        {
            return err; /* stacd = core */
        }
    }
    return E_OK;
}

/**
 * @brief  Resources usage.
 */
EXPORT void resources_usage (res_usage_t * p_res)
{
    union
    {
        T_RTSK t;
        T_RSEM s;
        T_RFLG f;
        T_RMBF m;
        T_RCYC c;
    } uval;
    p_res->tasks =
        (uint32_t)count_objects((ref_fn_t)tk_ref_tsk, CNF_MAX_TSKID, &uval);
    p_res->flags =
        (uint32_t)count_objects((ref_fn_t)tk_ref_flg, CNF_MAX_FLGID, &uval);
    p_res->sems =
        (uint32_t)count_objects((ref_fn_t)tk_ref_sem, CNF_MAX_SEMID, &uval);
    p_res->mbfs =
        (uint32_t)count_objects((ref_fn_t)tk_ref_mbf, CNF_MAX_MBFID, &uval);
    p_res->cycs =
        (uint32_t)count_objects((ref_fn_t)tk_ref_cyc, CNF_MAX_CYCID, &uval);
    p_res->cpu_pct[0] = g_cpu_pct[0];
    p_res->cpu_pct[1] = g_cpu_pct[1];
}

/*** end of file ***/
