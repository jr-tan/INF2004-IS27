/**
 * @file subsystem4.c
 * @brief Sensor-independent filtering and terrain-state logic for Buddy 4.
 */
#include <stdbool.h>
#include <stdint.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"

extern float atan2f(float y, float x);
extern float sqrtf(float x);
extern float sinf(float x);
extern float fabsf(float x);

static imu_calibration_offsets_t g_calibration = {0};
static float g_filter_alpha = SUBSYSTEM4_DEFAULT_FILTER_ALPHA;
static float g_pitch_filtered_deg = 0.0f;
static float g_roll_filtered_deg = 0.0f;
static float g_previous_heading_deg = 0.0f;
static bool g_heading_initialized = false;
static float g_integrated_height_cm = 0.0f;
static float g_peak_hump_height_cm = 0.0f;
static bool g_hump_active = false;
static bool g_abort_requested = false;
static subsystem4_operational_state_t g_operational_state = SUBSYSTEM4_STATE_INIT;

static float wrap_degrees(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

ER subsystem4_init(void)
{
    Subsystem4_Init();
    return E_OK;
}

ER subsystem4_start(void)
{
    extern ID Subsystem4_Start_Sensor_Task(void);
    ID task_id = Subsystem4_Start_Sensor_Task();

    return (task_id > 0) ? E_OK : (ER)task_id;
}

void Subsystem4_Init(void)
{
    g_filter_alpha = SUBSYSTEM4_DEFAULT_FILTER_ALPHA;
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_previous_heading_deg = 0.0f;
    g_heading_initialized = false;
    g_integrated_height_cm = 0.0f;
    g_peak_hump_height_cm = 0.0f;
    g_hump_active = false;
    g_abort_requested = false;
    g_operational_state = SUBSYSTEM4_STATE_INIT;

    if (!g_calibration.is_calibrated) {
        g_calibration.accel_x_offset_g = 0.0f;
        g_calibration.accel_y_offset_g = 0.0f;
        g_calibration.accel_z_offset_g = 0.0f;
    }
}

bool Subsystem4_Calibrate(uint32_t timeout_ms)
{
    if ((timeout_ms == 0U) ||
        (timeout_ms > SUBSYSTEM4_IMU_INIT_TIMEOUT_MS)) {
        return false;
    }

    /* Nominal calibration for deterministic software-only tests. */
    g_calibration.accel_x_offset_g = 0.0f;
    g_calibration.accel_y_offset_g = 0.0f;
    g_calibration.accel_z_offset_g = 0.0f;
    g_calibration.is_calibrated = true;
    g_operational_state = SUBSYSTEM4_STATE_FLAT_TRACKING;
    return true;
}

bool Subsystem4_Calibrate_From_Sensor(uint32_t timeout_ms)
{
    gy511_raw_sample_t sample;
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;
    uint32_t sample_count;
    uint32_t index;

    if ((timeout_ms < 100U) ||
        (timeout_ms > SUBSYSTEM4_IMU_INIT_TIMEOUT_MS) ||
        !GY511_IsReady()) {
        return false;
    }

    sample_count = timeout_ms / SUBSYSTEM4_SAMPLE_PERIOD_MS;
    if (sample_count > 100U) {
        sample_count = 100U;
    }
    if (sample_count < 10U) {
        sample_count = 10U;
    }

    for (index = 0U; index < sample_count; ++index) {
        if (!GY511_ReadRawSample(&sample)) {
            return false;
        }

        ax_sum += (float)sample.accel_x * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;
        ay_sum += (float)sample.accel_y * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;
        az_sum += (float)sample.accel_z * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;

        if ((index + 1U) < sample_count) {
            (void)tk_dly_tsk(SUBSYSTEM4_SAMPLE_PERIOD_MS);
        }
    }

    /*
     * Calibration assumes the board is level and +Z points upward.
     * Store only bias; retain the expected 1 g gravity vector at runtime.
     */
    g_calibration.accel_x_offset_g = ax_sum / (float)sample_count;
    g_calibration.accel_y_offset_g = ay_sum / (float)sample_count;
    g_calibration.accel_z_offset_g = (az_sum / (float)sample_count) - 1.0f;
    g_calibration.is_calibrated = true;

    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_heading_initialized = false;
    g_operational_state = SUBSYSTEM4_STATE_FLAT_TRACKING;
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
    float pitch_accel_deg;
    float roll_accel_deg;
    float heading_deg;
    float mag_x;
    float mag_y;

    if ((p_raw_sample == (const imu_raw_sample_t *)0) ||
        (p_filtered_data == (imu_filtered_data_t *)0)) {
        return;
    }

    accel_x_g = ((float)p_raw_sample->accel_x *
                 SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
                g_calibration.accel_x_offset_g;
    accel_y_g = ((float)p_raw_sample->accel_y *
                 SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
                g_calibration.accel_y_offset_g;
    accel_z_g = ((float)p_raw_sample->accel_z *
                 SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
                g_calibration.accel_z_offset_g;

    pitch_accel_deg = atan2f(-accel_x_g,
        sqrtf((accel_y_g * accel_y_g) + (accel_z_g * accel_z_g))) *
        SUBSYSTEM4_RAD_TO_DEG;
    roll_accel_deg = atan2f(accel_y_g, accel_z_g) * SUBSYSTEM4_RAD_TO_DEG;

    g_pitch_filtered_deg = (g_filter_alpha * g_pitch_filtered_deg) +
        ((1.0f - g_filter_alpha) * pitch_accel_deg);
    g_roll_filtered_deg = (g_filter_alpha * g_roll_filtered_deg) +
        ((1.0f - g_filter_alpha) * roll_accel_deg);

    mag_x = (float)p_raw_sample->mag_x;
    mag_y = (float)p_raw_sample->mag_y;
    heading_deg = atan2f(mag_y, mag_x) * SUBSYSTEM4_RAD_TO_DEG;

    p_filtered_data->pitch_deg = g_pitch_filtered_deg;
    p_filtered_data->roll_deg = g_roll_filtered_deg;
    p_filtered_data->accel_magnitude_g = sqrtf(
        (accel_x_g * accel_x_g) +
        (accel_y_g * accel_y_g) +
        (accel_z_g * accel_z_g));
    p_filtered_data->heading_deg = heading_deg;
    p_filtered_data->heading_valid =
        !((p_raw_sample->mag_x == 0) &&
          (p_raw_sample->mag_y == 0) &&
          (p_raw_sample->mag_z == 0));

    if (p_filtered_data->heading_valid && g_heading_initialized) {
        p_filtered_data->yaw_rate_deg_s =
            wrap_degrees(heading_deg - g_previous_heading_deg) /
            ((float)SUBSYSTEM4_SAMPLE_PERIOD_MS / 1000.0f);
    } else {
        p_filtered_data->yaw_rate_deg_s = 0.0f;
    }

    if (p_filtered_data->heading_valid) {
        g_previous_heading_deg = heading_deg;
        g_heading_initialized = true;
    }
    p_filtered_data->timestamp_ms = p_raw_sample->timestamp_ms;
}

void Subsystem4_Terrain_Step(const imu_filtered_data_t *p_filtered_data,
                             uint32_t delta_distance_mm,
                             terrain_state_t *p_terrain_state)
{
    float delta_distance_cm;
    float pitch_rad;
    motion_class_t previous_motion_state;

    if ((p_filtered_data == (const imu_filtered_data_t *)0) ||
        (p_terrain_state == (terrain_state_t *)0)) {
        return;
    }

    previous_motion_state = p_terrain_state->motion_state;
    p_terrain_state->current_pitch = p_filtered_data->pitch_deg;
    p_terrain_state->current_roll = p_filtered_data->roll_deg;
    p_terrain_state->yaw_rate_deg_s = p_filtered_data->yaw_rate_deg_s;
    p_terrain_state->sudden_impact_flag = false;
    p_terrain_state->event_flags = 0U;

    if (p_filtered_data->accel_magnitude_g >
        SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G) {
        p_terrain_state->sudden_impact_flag = true;
        p_terrain_state->motion_state = MOTION_CLASS_SUDDEN_IMPACT;
        p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
        p_terrain_state->hump_active = g_hump_active;
        p_terrain_state->event_flags |= SUBSYSTEM4_EVT_SUDDEN_IMPACT;
        g_operational_state = SUBSYSTEM4_STATE_ABORT_RESET;
        return;
    }

    if (p_filtered_data->heading_valid &&
        (fabsf(p_filtered_data->yaw_rate_deg_s) > 15.0f)) {
        p_terrain_state->motion_state = MOTION_CLASS_TURNING;
    } else if (p_filtered_data->pitch_deg > SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        p_terrain_state->motion_state = MOTION_CLASS_CLIMBING;
    } else if (p_filtered_data->pitch_deg < -SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        p_terrain_state->motion_state = MOTION_CLASS_DESCENDING;
    } else if (fabsf(p_filtered_data->accel_magnitude_g - 1.0f) > 0.15f) {
        p_terrain_state->motion_state = MOTION_CLASS_ACCELERATING;
    } else {
        p_terrain_state->motion_state = MOTION_CLASS_STATIONARY;
    }

    delta_distance_cm = (float)delta_distance_mm / 10.0f;
    pitch_rad = p_filtered_data->pitch_deg * SUBSYSTEM4_DEG_TO_RAD;

    if (p_filtered_data->pitch_deg >= SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
        if (!g_hump_active) {
            g_hump_active = true;
            g_integrated_height_cm = 0.0f;
            g_operational_state = SUBSYSTEM4_STATE_HUMP_PROCESSING;
        }
        g_integrated_height_cm += delta_distance_cm * sinf(pitch_rad);
        if (g_integrated_height_cm > g_peak_hump_height_cm) {
            g_peak_hump_height_cm = g_integrated_height_cm;
        }
    } else if ((p_filtered_data->pitch_deg <= -SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) &&
               g_hump_active) {
        g_integrated_height_cm += delta_distance_cm * sinf(pitch_rad);
    } else if (g_hump_active &&
               (fabsf(p_filtered_data->pitch_deg) <
                SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG)) {
        g_hump_active = false;
        g_operational_state = SUBSYSTEM4_STATE_TELEMETRY_DISPATCH;
        p_terrain_state->event_flags |= SUBSYSTEM4_EVT_HUMP_EXIT;
    } else if (!g_hump_active) {
        g_operational_state = SUBSYSTEM4_STATE_FLAT_TRACKING;
    }

    if (p_terrain_state->motion_state != previous_motion_state) {
    p_terrain_state->event_flags |= SUBSYSTEM4_EVT_STATE_CHANGE;

    if (!g_hump_active) {
        g_operational_state = SUBSYSTEM4_STATE_TELEMETRY_DISPATCH;
    }
    }

    p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
    p_terrain_state->hump_active = g_hump_active;
}

void Subsystem4_Reset_Watchdog_Recover(void)
{
    /* Static calibration deliberately survives this software recovery. */
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_heading_initialized = false;
    g_integrated_height_cm = 0.0f;
    g_hump_active = false;
    g_abort_requested = false;
    g_operational_state = g_calibration.is_calibrated ?
        SUBSYSTEM4_STATE_FLAT_TRACKING : SUBSYSTEM4_STATE_INIT;
}

void Subsystem4_Request_Abort_Reset(void)
{
    g_abort_requested = true;
    g_operational_state = SUBSYSTEM4_STATE_ABORT_RESET;
}

subsystem4_operational_state_t Subsystem4_Get_Operational_State(void)
{
    return g_operational_state;
}
