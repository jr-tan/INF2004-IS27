/**
 * @file task_monitor.c
 * @brief Event-driven acquisition -> filter -> terrain pipeline for GY-511.
 *
 * Stage 1 deliberately uses a timed semaphore wait as a polling fallback:
 * - If the confirmed GY-511 DRDY ISR signals the semaphore, acquisition wakes
 *   immediately.
 * - If no ISR is connected, the wait times out every 10 ms and acquisition
 *   polls the sensor. This keeps the pipeline testable without guessing a pin.
 *
 * The GY-511 (LSM303DLHC) has accelerometer + magnetometer, no gyro and no FIFO.
 */
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"
#include "subsystem4_pipeline.h"

#define S4_TASK_PRIORITY             (12)
#define S4_SAMPLE_PERIOD_MS          (10U)
#define S4_QUEUE_WAIT_MS             (20)
#define S4_PRINT_EVERY_N_SAMPLES     (10U)
#define S4_RAW_QUEUE_BYTES           (sizeof(imu_raw_sample_t) * 8U)
#define S4_FILTERED_QUEUE_BYTES      (sizeof(imu_filtered_data_t) * 8U)

static ID g_raw_mbf_id = 0;
static ID g_filtered_mbf_id = 0;
static ID g_data_ready_sem_id = 0;
static ID g_acquire_task_id = 0;
static ID g_filter_task_id = 0;
static ID g_terrain_task_id = 0;
static terrain_state_t g_current_terrain_state;
static imu_filtered_data_t g_last_filtered_data;
static bool g_pipeline_started = false;

/*
 * Integration hook for a verified DRDY ISR.
 * tk_sig_sem is intended to be the only RTOS action in this ISR hook.
 * Do not call this from a GPIO interrupt until ISR API rules are confirmed.
 */
void Subsystem4_Notify_Data_Ready_From_ISR(void)
{
    if (g_data_ready_sem_id > 0) {
        (void)tk_sig_sem(g_data_ready_sem_id, 1);
    }
}

static void subsystem4_acquisition_task(INT stacd, void *exinf)
{
    gy511_raw_sample_t gy_sample;
    imu_raw_sample_t sample;
    INT rc;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (!GY511_IsReady()) {
            if (!GY511_Init()) {
                tm_printf((UB *)"S4: GY-511 init failed, stage=%d; retrying\n",
                          (INT)GY511_Get_Error_Stage());
                (void)tk_dly_tsk(250U);
                continue;
            }

            tm_printf((UB *)"S4: GY-511 ready; keep board level/still for calibration\n");
            if (!Subsystem4_Calibrate_From_Sensor(SUBSYSTEM4_IMU_INIT_TIMEOUT_MS)) {
                tm_printf((UB *)"S4: calibration failed; retrying\n");
                (void)tk_dly_tsk(250U);
                continue;
            }
            tm_printf((UB *)"S4: calibration complete\n");
        }

        /*
         * Event-driven when a DRDY ISR is connected; timed polling fallback
         * otherwise. Timeout is expected, not an error.
         */
        rc = tk_wai_sem(g_data_ready_sem_id, 1, (TMO)S4_SAMPLE_PERIOD_MS);
        if ((rc != E_OK) && (rc != E_TMOUT)) {
            tm_printf((UB *)"S4: data-ready semaphore wait error=%d\n", rc);
            (void)tk_dly_tsk(S4_SAMPLE_PERIOD_MS);
        }

        if (Subsystem4_Get_Operational_State() == SUBSYSTEM4_STATE_ABORT_RESET) {
            Subsystem4_Reset_Watchdog_Recover();
        }

        if (!GY511_ReadRawSample(&gy_sample)) {
            tm_printf((UB *)"S4: sensor read failed; reinitializing\n");
            (void)tk_dly_tsk(100U);
            continue;
        }

        sample.accel_x = gy_sample.accel_x;
        sample.accel_y = gy_sample.accel_y;
        sample.accel_z = gy_sample.accel_z;
        sample.mag_x = gy_sample.mag_x;
        sample.mag_y = gy_sample.mag_y;
        sample.mag_z = gy_sample.mag_z;
        sample.timestamp_ms = gy_sample.timestamp_ms;

        rc = tk_snd_mbf(g_raw_mbf_id, &sample, (INT)sizeof(sample), TMO_POL);
        if (rc < E_OK) {
            /* Queue full: drop this sample instead of blocking acquisition. */
            continue;
        }
    }
}

static void subsystem4_filter_task(INT stacd, void *exinf)
{
    imu_raw_sample_t raw_sample;
    imu_filtered_data_t filtered_data;
    INT rc;

    (void)stacd;
    (void)exinf;

    for (;;) {
        rc = tk_rcv_mbf(g_raw_mbf_id, &raw_sample, (TMO)TMO_FEVR);
        if (rc < E_OK) {
            continue;
        }

        Subsystem4_Filter_Step(&raw_sample, &filtered_data);
        rc = tk_snd_mbf(g_filtered_mbf_id, &filtered_data, (INT)sizeof(filtered_data), TMO_POL);
        if (rc < E_OK) {
            /* Preserve freshness: don't stall the filter task on a full queue. */
            continue;
        }
    }
}

static void subsystem4_terrain_task(INT stacd, void *exinf)
{
    imu_filtered_data_t filtered_data;
    uint32_t sample_count = 0U;
    INT rc;

    (void)stacd;
    (void)exinf;

    for (;;) {
        rc = tk_rcv_mbf(g_filtered_mbf_id, &filtered_data, (TMO)TMO_FEVR);
        if (rc < E_OK) {
            continue;
        }

        g_last_filtered_data = filtered_data;
        /* No wheel odometry interface is available in this template snapshot. */
        Subsystem4_Terrain_Step(&filtered_data, 0U, &g_current_terrain_state);

        if ((sample_count % S4_PRINT_EVERY_N_SAMPLES) == 0U) {
            tm_printf((UB *)"S4 pitch_cdeg=%d roll_cdeg=%d heading_cdeg=%d accel_mg=%d state=%d impact=%d\n",
                      (INT)(filtered_data.pitch_deg * 100.0f),
                      (INT)(filtered_data.roll_deg * 100.0f),
                      (INT)(filtered_data.heading_deg * 100.0f),
                      (INT)(filtered_data.accel_magnitude_g * 1000.0f),
                      (INT)g_current_terrain_state.motion_state,
                      (INT)(g_current_terrain_state.sudden_impact_flag ? 1 : 0));
        }
        ++sample_count;
    }
}

const imu_filtered_data_t *Subsystem4_Get_Last_Filtered(void)
{
    return &g_last_filtered_data;
}

const terrain_state_t *Subsystem4_Get_Terrain_State(void)
{
    return &g_current_terrain_state;
}

ER Subsystem4_Start_Pipeline_Tasks(void)
{
    T_CMBF mbf_cfg;
    T_CSEM sem_cfg;

    if (g_pipeline_started) {
        return E_OK;
    }

    /* Queue IDs and semaphore are created locally to avoid guessed project IDs. */
    mbf_cfg.exinf = NULL;
    mbf_cfg.mbfatr = TA_TFIFO;
    mbf_cfg.bufsz = (INT)S4_RAW_QUEUE_BYTES;
    mbf_cfg.maxmsz = (INT)sizeof(imu_raw_sample_t);
    mbf_cfg.bufptr = NULL;
    g_raw_mbf_id = tk_cre_mbf(&mbf_cfg);
    if (g_raw_mbf_id < E_OK) {
        return (ER)g_raw_mbf_id;
    }

    mbf_cfg.bufsz = (INT)S4_FILTERED_QUEUE_BYTES;
    mbf_cfg.maxmsz = (INT)sizeof(imu_filtered_data_t);
    g_filtered_mbf_id = tk_cre_mbf(&mbf_cfg);
    if (g_filtered_mbf_id < E_OK) {
        return (ER)g_filtered_mbf_id;
    }

    sem_cfg.exinf = NULL;
    sem_cfg.sematr = TA_TFIFO;
    sem_cfg.isemcnt = 0;
    sem_cfg.maxsem = 1;
    g_data_ready_sem_id = tk_cre_sem(&sem_cfg);
    if (g_data_ready_sem_id < E_OK) {
        return (ER)g_data_ready_sem_id;
    }

    g_acquire_task_id = app_start_task((FP)subsystem4_acquisition_task,
                                       S4_TASK_PRIORITY, TP_PRC1, "s4_acq");
    if (g_acquire_task_id < E_OK) {
        return (ER)g_acquire_task_id;
    }

    g_filter_task_id = app_start_task((FP)subsystem4_filter_task,
                                      S4_TASK_PRIORITY, TP_PRC1, "s4_filter");
    if (g_filter_task_id < E_OK) {
        return (ER)g_filter_task_id;
    }

    g_terrain_task_id = app_start_task((FP)subsystem4_terrain_task,
                                       S4_TASK_PRIORITY, TP_PRC1, "s4_terrain");
    if (g_terrain_task_id < E_OK) {
        return (ER)g_terrain_task_id;
    }

    g_pipeline_started = true;
    return E_OK;
}

/* Compatibility entry point used by subsystem4.c. */
ID Subsystem4_Start_Sensor_Task(void)
{
    ER rc = Subsystem4_Start_Pipeline_Tasks();
    return (rc == E_OK) ? g_acquire_task_id : (ID)rc;
}
