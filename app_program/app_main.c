/**
 * @file    app_main.c
 *  app_main.c
 *  START HERE.  usermain() below is your program's entry point.
 *
 *  How micro T-Kernel starts your code
 *  -----------------------------------
 *    reset -> kernel initialises itself -> creates the "initial task"
 *          -> initial task calls usermain()
 *
 *  usermain() runs inside that initial task at priority 1 (the highest).
 *  On a single core nothing you create runs until usermain() blocks, but
 *  on the dual-core build the OTHER core may start a new task at once.
 *  So usermain() works in two phases:
 *    1. every subsystem creates its kernel objects  (subsystemN_init)
 *    2. every subsystem starts its tasks             (subsystemN_start)
 *  and then puts the initial task to sleep forever.
 *  If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *  The IS27 car (see car.h for the interfaces, car_config.h for pins,
 *  priorities and the resource budget):
 *    subsystem1/  comms    MQTT telemetry + commands, fault bitmask (core 0)
 *                 status   console print + status LED          (core 0)
 *    subsystem2/  motion   motors, encoders, odometry, 20 ms    (core 0)
 *    subsystem3/  line     IR sensors -> steering, 20 ms        (core 1)
 *                 barcode  Code 39 decoder on GP2               (core 0)
 *    subsystem4/  imu      GY-511: tilt, humps, impacts, 50 Hz  (core 1)
 *    subsystem5/  ranger   ultrasonic + servo, 60 ms pings      (core 1)
 *    mission.c    mission  buttons/commands, FOLLOW / TURN / AVOID (core 1)
 *    resources.c  boot-time check of every kernel object in use
 *
 *  Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "car.h"
#include "app_main.h" /* own header (BARR-C 4.3.c) */

/**
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_main_task_start (FP entry, PRI pri, uint32_t prc,
                               const char * p_name)
{
    T_CTSK ctsk = {0};
    ID     tid;
    ER     err;

    ctsk.task    = entry;           /* function the task runs */
    ctsk.itskpri = pri;             /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG          /* written in a high-level language (C) */
                  | TA_RNG3;        /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
        ctsk.assprc = prc;        /* which core(s) it may run on */
    }
#else
    (void)prc; /* single-core build: no affinity */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, p_name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    err = tk_sta_tsk(tid, 0);
    CHECK(err, p_name);
    return tid;
}

/**
 * @brief  Usermain.
 * @return Result as described above.
 */
EXPORT int32_t usermain (void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf((uint8_t *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                         " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((uint8_t *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    tm_printf((uint8_t *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem5_init(), "subsystem5_init");
    CHECK(mission_init(), "mission_init");

    /* Phase 2: every subsystem starts its tasks */
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");
    CHECK(mission_start(), "mission_start");
    CHECK(resources_start(), "resources_start"); /* CPU-load meters */

    tm_printf((uint8_t *)"all subsystems started\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
       here in the initial task, while the system is running.  Tests may
       call tk_dly_tsk() to let the other tasks work for a while. */
    tm_printf((uint8_t *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((uint8_t *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
