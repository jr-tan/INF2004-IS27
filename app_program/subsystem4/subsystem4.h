/**
 * @file subsystem4.h
 * @brief Buddy 4 IMU filtering and terrain-monitoring interface.
 *
 * The module is written for the INF2004 μT-Kernel application template.
 * Sensor acquisition is deliberately kept separate: the board/sensor driver
 * supplies imu_raw_sample_t values to Subsystem4_Pipeline_Tick().
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

/* Legacy identifiers retained for compatibility with existing subsystem code.
 * Do not use these as application data channels without defining their owner. */
#define S4_SUMMARY_MAX                 (64U)
#define FLG_BATCH                      (0x0001U)

extern ID mbf_summary;
extern ID flg_events;

/* Timing and algorithm parameters. */
#define SUBSYSTEM4_IMU_INIT_TIMEOUT_MS       (3000U)
#define SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX (2.0f)
#define SUBSYSTEM4_DEFAULT_FILTER_ALPHA      (0.96f)
#define SUBSYSTEM4_IMU_SAMPLE_PERIOD_S      (0.01f)  /* 100 Hz */
#define SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG     (5.0f)
#define SUBSYSTEM4_ALERT_PITCH_DEG          (15.0f)
#define SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G  (2.5f)
#define SUBSYSTEM4_DEG_TO_RAD               (0.017453292519943295f)
#define SUBSYSTEM4_RAD_TO_DEG               (57.29577951308232f)
#define SUBSYSTEM4_RAW_TO_DEG_S             (1.0f / 131.0f) /* MPU6050, ±250 dps */
#define SUBSYSTEM4_RAW_TO_G                  (1.0f / 16384.0f) /* MPU6050, ±2 g */

/* Event bits reserved for integration with the rest of the application. */
#define SUBSYSTEM4_EVT_TILT          (0x0001U)
#define SUBSYSTEM4_EVT_SUDDEN_IMPACT (0x0002U)
#define SUBSYSTEM4_EVT_INIT_DONE     (0x0004U)

typedef enum {
    MOTION_CLASS_STATIONARY = 0U,
    MOTION_CLASS_ACCELERATING,
    MOTION_CLASS_TURNING,
    MOTION_CLASS_CLIMBING,
    MOTION_CLASS_DESCENDING,
    MOTION_CLASS_SUDDEN_IMPACT
} motion_class_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t timestamp_ms;
} imu_raw_sample_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    float yaw_rate_deg_s;
    float accel_magnitude_g;
    uint32_t timestamp_ms;
} imu_filtered_data_t;

typedef struct {
    float accel_x_offset;
    float accel_y_offset;
    float accel_z_offset;
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
    bool is_calibrated;
} imu_calibration_offsets_t;

typedef struct {
    float current_pitch;
    float peak_height_cm;
    float yaw_rate_deg_s;
    motion_class_t motion_state;
    bool sudden_impact_flag;
    bool hump_active;
} terrain_state_t;

/* μT-Kernel application lifecycle hooks called by usermain(). */
ER subsystem4_init(void);
ER subsystem4_start(void);
void subsystem4_test(void);

/* Algorithm API. */
void Subsystem4_Init(void);
bool Subsystem4_Calibrate(uint32_t timeout_ms);
void Subsystem4_Filter_Step(const imu_raw_sample_t *p_raw_sample,
                            imu_filtered_data_t *p_filtered_data);
void Subsystem4_Terrain_Step(const imu_filtered_data_t *p_filtered_data,
                             uint32_t delta_distance_mm,
                             terrain_state_t *p_terrain_state);
void Subsystem4_Reset_Watchdog_Recover(void);
void Subsystem4_Set_Filter_Alpha(float alpha);
const imu_calibration_offsets_t *Subsystem4_Get_Calibration(void);

/* Process one acquired sensor sample. This is called by the sensor/task
 * integration layer; this module does not assume a particular I2C driver. */
void Subsystem4_Pipeline_Tick(const imu_raw_sample_t *p_raw_sample,
                              uint32_t delta_distance_mm);
const imu_filtered_data_t *Subsystem4_Get_Last_Filtered(void);
const terrain_state_t *Subsystem4_Get_Terrain_State(void);

#endif /* SUBSYSTEM4_H */
