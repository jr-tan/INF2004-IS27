/**
 * @file subsystem4.c
 * @brief Buddy 4 IMU filtering, motion classification and terrain estimation.
 */
#include <stdbool.h>
#include <stdint.h>

#include "subsystem4.h"

/* Math library functions supplied by the linker (-lm). */
extern float atan2f(float y, float x);
extern float sqrtf(float x);
extern float sinf(float x);
extern float fabsf(float x);

/* Compatibility symbols used by older template code. */
ID mbf_summary = 0;
ID flg_events = 0;

static imu_calibration_offsets_t g_calibration = {0};
static float g_filter_alpha = SUBSYSTEM4_DEFAULT_FILTER_ALPHA;
static float g_pitch_filtered_deg = 0.0f;
static float g_roll_filtered_deg = 0.0f;
static float g_integrated_height_cm = 0.0f;
static float g_peak_hump_height_cm = 0.0f;
static bool g_hump_active = false;

static bool is_finite_sample(const imu_raw_sample_t *p_sample)
{
    return (p_sample != NULL);
}

ER subsystem4_init(void)
{
    Subsystem4_Init();
    return E_OK;
}

/*
 * The application template calls subsystemN_start() after all subsystem init
 * functions have completed. No sensor-driver API was included with the
 * supplied Subsystem 4 files, so task creation/acquisition is intentionally
 * left to the integration layer rather than creating a task that cannot read
 * the physical IMU yet.
 */
ER subsystem4_start(void)
{
    return E_OK;
}

void Subsystem4_Init(void)
{
    g_filter_alpha = SUBSYSTEM4_DEFAULT_FILTER_ALPHA;
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_integrated_height_cm = 0.0f;
    g_peak_hump_height_cm = 0.0f;
    g_hump_active = false;

    if (!g_calibration.is_calibrated) {
        g_calibration.accel_x_offset = 0.0f;
        g_calibration.accel_y_offset = 0.0f;
        g_calibration.accel_z_offset = 0.0f;
        g_calibration.gyro_x_offset = 0.0f;
        g_calibration.gyro_y_offset = 0.0f;
        g_calibration.gyro_z_offset = 0.0f;
        g_calibration.is_calibrated = false;
    }
}

bool Subsystem4_Calibrate(uint32_t timeout_ms)
{
    if (timeout_ms > SUBSYSTEM4_IMU_INIT_TIMEOUT_MS) {
        return false;
    }

    /*
     * Placeholder/nominal calibration for the current software tests.
     * Replace this with averaging of real sensor samples before hardware use.
     * MPU6050 accelerometer Z is treated as +1 g when level.
     */
    g_calibration.accel_x_offset = 0.0f;
    g_calibration.accel_y_offset = 0.0f;
    g_calibration.accel_z_offset = 1.0f;
    g_calibration.gyro_x_offset = 0.0f;
    g_calibration.gyro_y_offset = 0.0f;
    g_calibration.gyro_z_offset = 0.0f;
    g_calibration.is_calibrated = true;

    return true;
}

void Subsystem4_Set_Filter_Alpha(float alpha)
{
    if ((alpha >= 0.0f) && (alpha <= 1.0f)) {
        g_filter_alpha = alpha;
    }
}

const imu_calibration_offsets_t *Subsystem4_Get_Calibration(void)
{
    return &g_calibration;
}

void Subsystem4_Filter_Step(const imu_raw_sample_t *p_raw_sample,
                            imu_filtered_data_t *p_filtered_data)
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float pitch_accel_deg;
    float roll_accel_deg;

    if ((p_raw_sample == NULL) || (p_filtered_data == NULL)) {
        return;
    }

    accel_x_g = ((float)p_raw_sample->accel_x * SUBSYSTEM4_RAW_TO_G) -
                g_calibration.accel_x_offset;
    accel_y_g = ((float)p_raw_sample->accel_y * SUBSYSTEM4_RAW_TO_G) -
                g_calibration.accel_y_offset;
    accel_z_g = ((float)p_raw_sample->accel_z * SUBSYSTEM4_RAW_TO_G) -
                (g_calibration.is_calibrated ? (g_calibration.accel_z_offset - 1.0f) : 0.0f);

    gyro_x_dps = ((float)p_raw_sample->gyro_x * SUBSYSTEM4_RAW_TO_DEG_S) -
                 g_calibration.gyro_x_offset;
    gyro_y_dps = ((float)p_raw_sample->gyro_y * SUBSYSTEM4_RAW_TO_DEG_S) -
                 g_calibration.gyro_y_offset;
    gyro_z_dps = ((float)p_raw_sample->gyro_z * SUBSYSTEM4_RAW_TO_DEG_S) -
                 g_calibration.gyro_z_offset;

    pitch_accel_deg = atan2f(-accel_x_g,
                             sqrtf((accel_y_g * accel_y_g) +
                                   (accel_z_g * accel_z_g))) *
                      SUBSYSTEM4_RAD_TO_DEG;
    roll_accel_deg = atan2f(accel_y_g, accel_z_g) * SUBSYSTEM4_RAD_TO_DEG;

    g_pitch_filtered_deg =
        (g_filter_alpha *
         (g_pitch_filtered_deg +
          (gyro_y_dps * SUBSYSTEM4_IMU_SAMPLE_PERIOD_S))) +
        ((1.0f - g_filter_alpha) * pitch_accel_deg);

    g_roll_filtered_deg =
        (g_filter_alpha *
         (g_roll_filtered_deg +
          (gyro_x_dps * SUBSYSTEM4_IMU_SAMPLE_PERIOD_S))) +
        ((1.0f - g_filter_alpha) * roll_accel_deg);

    p_filtered_data->pitch_deg = g_pitch_filtered_deg;
    p_filtered_data->roll_deg = g_roll_filtered_deg;
    p_filtered_data->yaw_rate_deg_s = gyro_z_dps;
    p_filtered_data->accel_magnitude_g =
        sqrtf((accel_x_g * accel_x_g) +
              (accel_y_g * accel_y_g) +
              (accel_z_g * accel_z_g));
    p_filtered_data->timestamp_ms = p_raw_sample->timestamp_ms;
}

void Subsystem4_Terrain_Step(const imu_filtered_data_t *p_filtered_data,
                             uint32_t delta_distance_mm,
                             terrain_state_t *p_terrain_state)
{
    float delta_distance_cm;
    float pitch_rad;

    if ((p_filtered_data == NULL) || (p_terrain_state == NULL)) {
        return;
    }

    p_terrain_state->current_pitch = p_filtered_data->pitch_deg;
    p_terrain_state->yaw_rate_deg_s = p_filtered_data->yaw_rate_deg_s;
    p_terrain_state->sudden_impact_flag = false;

    if (p_filtered_data->accel_magnitude_g >
        SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G) {
        p_terrain_state->sudden_impact_flag = true;
        p_terrain_state->motion_state = MOTION_CLASS_SUDDEN_IMPACT;
        p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
        p_terrain_state->hump_active = g_hump_active;
        return;
    }

    if (fabsf(p_filtered_data->yaw_rate_deg_s) > 15.0f) {
        p_terrain_state->motion_state = MOTION_CLASS_TURNING;
    } else if (p_filtered_data->pitch_deg > SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        p_terrain_state->motion_state = MOTION_CLASS_CLIMBING;
    } else if (p_filtered_data->pitch_deg < -SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        p_terrain_state->motion_state = MOTION_CLASS_DESCENDING;
    } else {
        p_terrain_state->motion_state = MOTION_CLASS_STATIONARY;
    }

    delta_distance_cm = (float)delta_distance_mm / 10.0f;
    pitch_rad = p_filtered_data->pitch_deg * SUBSYSTEM4_DEG_TO_RAD;

    if (p_filtered_data->pitch_deg >= SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        if (!g_hump_active) {
            g_hump_active = true;
            g_integrated_height_cm = 0.0f;
        }
        g_integrated_height_cm += delta_distance_cm * sinf(pitch_rad);
        if (g_integrated_height_cm > g_peak_hump_height_cm) {
            g_peak_hump_height_cm = g_integrated_height_cm;
        }
    } else if ((p_filtered_data->pitch_deg <= -SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) &&
               g_hump_active) {
        /* Integrate the descent until the robot returns to level ground. */
        g_integrated_height_cm += delta_distance_cm * sinf(pitch_rad);
    } else if (g_hump_active &&
               (fabsf(p_filtered_data->pitch_deg) <
                SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG)) {
        g_hump_active = false;
    }

    p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
    p_terrain_state->hump_active = g_hump_active;
}

void Subsystem4_Reset_Watchdog_Recover(void)
{
    /* Calibration is intentionally preserved across a software recovery. */
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_integrated_height_cm = 0.0f;
    g_hump_active = false;
}

/* Implemented in task_monitor.c. */
