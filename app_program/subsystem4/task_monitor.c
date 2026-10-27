/**
 * @file task_monitor.c
 * @brief Subsystem 4 pipeline and live MPU6050 acquisition task.
 *
 * Hardware wiring from IS27_Wiring_Guide.md: MPU6050 SDA=GP4, SCL=GP5,
 * VCC=3V3 and GND=Grove 3 GND. This driver polls at 100 Hz.
 */
#include <stdint.h>
#include "subsystem4.h"
#include "subsystem4_mpu6050.h"

#define SUBSYSTEM4_SENSOR_TASK_PRIORITY (12)
#define SUBSYSTEM4_SAMPLE_DELAY_MS      (10U)
#define SUBSYSTEM4_PRINT_EVERY_N_SAMPLES (10U)

static terrain_state_t g_current_terrain_state = {0};
static imu_filtered_data_t g_last_filtered_data = {0};
static ID g_sensor_task_id = 0;

void Subsystem4_Pipeline_Tick(const imu_raw_sample_t *p_raw_sample,
                              uint32_t delta_distance_mm)
{
    if (p_raw_sample == (const imu_raw_sample_t *)0) {
        return;
    }
    Subsystem4_Filter_Step(p_raw_sample, &g_last_filtered_data);
    Subsystem4_Terrain_Step(&g_last_filtered_data,
                            delta_distance_mm,
                            &g_current_terrain_state);
}

const imu_filtered_data_t *Subsystem4_Get_Last_Filtered(void)
{
    return &g_last_filtered_data;
}

const terrain_state_t *Subsystem4_Get_Terrain_State(void)
{
    return &g_current_terrain_state;
}

static void subsystem4_print_live_data(void)
{
    /* Print scaled integers: T-Monitor builds may not support %f formatting. */
    INT pitch_cdeg = (INT)(g_last_filtered_data.pitch_deg * 100.0f);
    INT roll_cdeg = (INT)(g_last_filtered_data.roll_deg * 100.0f);
    INT yaw_cdeg_s = (INT)(g_last_filtered_data.yaw_rate_deg_s * 100.0f);
    INT accel_mg = (INT)(g_last_filtered_data.accel_magnitude_g * 1000.0f);

    tm_printf((UB *)"S4 pitch_cdeg=%d roll_cdeg=%d yaw_cdeg_s=%d accel_mg=%d state=%d impact=%d\n",
              pitch_cdeg, roll_cdeg, yaw_cdeg_s, accel_mg,
              (INT)g_current_terrain_state.motion_state,
              (INT)(g_current_terrain_state.sudden_impact_flag ? 1 : 0));
}

static void subsystem4_sensor_task(INT stacd, void *exinf)
{
    imu_raw_sample_t sample;
    uint32_t sample_count = 0U;
    (void)stacd;
    (void)exinf;


    for (;;) {
        if (!MPU6050_Init()) {
            tm_printf(
				(UB *)"S4: MPU6050 init failed, stage=%lu, primary_abort=0x%08lX, abort=0x%08lX\n",
				(unsigned long)MPU6050_Get_I2C_Error_Stage(),
				(unsigned long)MPU6050_Get_Primary_I2C_Abort(),
				(unsigned long)MPU6050_Get_Last_I2C_Abort()
			);

            /* Do not calibrate unless initialization succeeds. */
            (void)tk_dly_tsk(1000U);
            continue;
        }

        tm_printf(
            (UB *)"S4: MPU6050 detected at I2C address 0x%02x. Keep robot level and still for calibration.\n",
            (UW)MPU6050_GetAddress()
        );

        if (!Subsystem4_Calibrate_From_Sensor(1500U)) {
            tm_printf(
                (UB *)"S4: sensor calibration failed; keep robot still and retrying.\n"
            );
            (void)tk_dly_tsk(1000U);
            continue;
        }

        tm_printf(
            (UB *)"S4: calibration complete; live tilt output follows (pitch/roll in centidegrees).\n"
        );
        tm_printf((UB *)"S4: calibration complete; live tilt output follows (pitch/roll in centidegrees).\n");
        sample_count = 0U;

        for (;;) {
            if (!MPU6050_ReadRawSample(&sample)) {
                tm_printf((UB *)"S4: MPU6050 read failed; reconnect/check wiring.\n");
                break;
            }
            /* No odometry increment is supplied here, so hump-height integration stays disabled. */
            Subsystem4_Pipeline_Tick(&sample, 0U);
            if ((sample_count % SUBSYSTEM4_PRINT_EVERY_N_SAMPLES) == 0U) {
                subsystem4_print_live_data();
            }
            ++sample_count;
            (void)tk_dly_tsk(SUBSYSTEM4_SAMPLE_DELAY_MS);
        }
        (void)tk_dly_tsk(250U);
    }
}

ID Subsystem4_Start_Sensor_Task(void)
{
    if (g_sensor_task_id > 0) {
        return g_sensor_task_id;
    }
    g_sensor_task_id = app_start_task((FP)subsystem4_sensor_task,
                                      SUBSYSTEM4_SENSOR_TASK_PRIORITY,
                                      TP_PRC1,
                                      "s4_imu");
    return g_sensor_task_id;
}
