/**
 * @file subsystem4.h
 * @brief Buddy 4 terrain monitoring interface for RP2040 and GY-511.
 *
 * Barr-C style notes:
 * - Private state and helper functions are file-local.
 * - Public interfaces use explicit types and pointer validation.
 * - Hardware and application integration are separated from algorithms.
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define SUBSYSTEM4_IMU_INIT_TIMEOUT_MS       (3000U)
#define SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX (2.0f)
#define SUBSYSTEM4_DEFAULT_FILTER_ALPHA      (0.80f)
#define SUBSYSTEM4_SAMPLE_PERIOD_MS          (10U)
#define SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG      (5.0f)
#define SUBSYSTEM4_ALERT_PITCH_DEG           (15.0f)
#define SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G  (2.5f)
#define SUBSYSTEM4_DEG_TO_RAD                (0.017453292519943295f)
#define SUBSYSTEM4_RAD_TO_DEG                (57.29577951308232f)
#define SUBSYSTEM4_GY511_ACCEL_RAW_TO_G       (0.001f) /* 1 mg/LSB in HR mode */

#define SUBSYSTEM4_EVT_TILT          (0x0001U)
#define SUBSYSTEM4_EVT_SUDDEN_IMPACT (0x0002U)
#define SUBSYSTEM4_EVT_INIT_DONE     (0x0004U)
#define SUBSYSTEM4_EVT_STATE_CHANGE  (0x0008U)
#define SUBSYSTEM4_EVT_HUMP_EXIT     (0x0010U)

typedef enum {
    MOTION_CLASS_STATIONARY = 0U,
    MOTION_CLASS_ACCELERATING,
    MOTION_CLASS_TURNING,
    MOTION_CLASS_CLIMBING,
    MOTION_CLASS_DESCENDING,
    MOTION_CLASS_SUDDEN_IMPACT
} motion_class_t;

typedef enum {
    SUBSYSTEM4_STATE_INIT = 0U,
    SUBSYSTEM4_STATE_FLAT_TRACKING,
    SUBSYSTEM4_STATE_HUMP_PROCESSING,
    SUBSYSTEM4_STATE_TELEMETRY_DISPATCH,
    SUBSYSTEM4_STATE_ABORT_RESET
} subsystem4_operational_state_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    uint32_t timestamp_ms;
} imu_raw_sample_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    /* Not a gyro rate: magnetometer-derived heading delta, when available. */
    float yaw_rate_deg_s;
    float accel_magnitude_g;
    float heading_deg;
    bool heading_valid;
    uint32_t timestamp_ms;
} imu_filtered_data_t;

typedef struct {
    float accel_x_offset_g;
    float accel_y_offset_g;
    float accel_z_offset_g;
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
    bool is_calibrated;
} imu_calibration_offsets_t;

typedef struct {
    float current_pitch;
    float current_roll;
    float peak_height_cm;
    float yaw_rate_deg_s;
    motion_class_t motion_state;
    bool sudden_impact_flag;
    bool hump_active;
    uint32_t event_flags;
} terrain_state_t;

ER subsystem4_init(void);
ER subsystem4_start(void);
void subsystem4_test(void);

void Subsystem4_Init(void);
bool Subsystem4_Calibrate(uint32_t timeout_ms);
bool Subsystem4_Calibrate_From_Sensor(uint32_t timeout_ms);
void Subsystem4_Filter_Step(const imu_raw_sample_t *p_raw_sample,
                            imu_filtered_data_t *p_filtered_data);
void Subsystem4_Terrain_Step(const imu_filtered_data_t *p_filtered_data,
                             uint32_t delta_distance_mm,
                             terrain_state_t *p_terrain_state);
void Subsystem4_Reset_Watchdog_Recover(void);
void Subsystem4_Set_Filter_Alpha(float alpha);
const imu_calibration_offsets_t *Subsystem4_Get_Calibration(void);
void Subsystem4_Request_Abort_Reset(void);
subsystem4_operational_state_t Subsystem4_Get_Operational_State(void);

void Subsystem4_Pipeline_Tick(const imu_raw_sample_t *p_raw_sample,
                              uint32_t delta_distance_mm);
const imu_filtered_data_t *Subsystem4_Get_Last_Filtered(void);
const terrain_state_t *Subsystem4_Get_Terrain_State(void);

#endif /* SUBSYSTEM4_H */
