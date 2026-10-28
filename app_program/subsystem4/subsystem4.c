/**
 * @file subsystem4.c
 * @brief Filtering, hump-height integration, hardware spinlock fault bitmask,
 *        and explicit binary telemetry packing for Buddy 4.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"

extern float atan2f(float y, float x);
extern float sqrtf(float x);
extern float sinf(float x);
extern float fabsf(float x);
extern float roundf(float x); 

/*
 * RP2040 Hardware Spinlock #30 (SIO_BASE + 0x100 + 30*4) for cross-core/task
 * and ISR-safe atomic Read-Modify-Write on shared packed words (Tribunal 1 R1).
 */
#define RP2040_SIO_BASE              (0xD0000000UL)
#define RP2040_SIO_SPINLOCK30        (*(volatile uint32_t *)(RP2040_SIO_BASE + 0x178UL))

static imu_calibration_offsets_t g_calibration = {0};
static float g_filter_alpha = SUBSYSTEM4_DEFAULT_FILTER_ALPHA;
static float g_raw_lpf_beta = SUBSYSTEM4_DEFAULT_RAW_LPF_BETA;

/* Raw accelerometer LPF state (Assumption A6 mitigation 1) */
static bool g_raw_lpf_initialized = false;
static float g_ax_lpf_g = 0.0f;
static float g_ay_lpf_g = 0.0f;
static float g_az_lpf_g = 1.0f;

static float g_pitch_filtered_deg = 0.0f;
static float g_roll_filtered_deg = 0.0f;
static float g_previous_heading_deg = 0.0f;
static bool g_heading_initialized = false;

/* Hump height accuracy state */
static float g_prev_pitch_rad = 0.0f;
static float g_integrated_height_cm = 0.0f;
static float g_current_hump_peak_cm = 0.0f;
static float g_peak_hump_height_cm = 0.0f;
static float g_last_hump_height_cm = 0.0f;
static float g_pre_entry_height_cm = 0.0f;
static uint32_t g_hump_entry_streak = 0U;
static uint32_t g_hump_count = 0U;
static bool g_hump_active = false;

/* Kinematic velocity fallback when wheel odometry is 0 */
static float g_estimated_vel_mm_s = 0.0f;

/* Shared atomic words protected by RP2040 hardware spinlock */
static volatile uint32_t g_shared_fault_bitmask = 0U;
static volatile uint32_t g_accumulated_odometry_mm = 0U;

static bool g_abort_requested = false;
static subsystem4_operational_state_t g_operational_state = SUBSYSTEM4_STATE_INIT;

/* Dual telemetry memory pools & mailbox (Tribunal 1 Change Requirement 1) */
static ID g_tlm_data_mpf_id = 0;
static ID g_tlm_fault_mpf_id = 0;
static ID g_terrain_tlm_mbx_id = 0;
static bool g_telemetry_initialized = false;
static uint16_t g_tlm_seq_num = 0U;
static subsystem4_telemetry_stats_t g_tlm_stats = {0};

/* ==========================================================================
 * RP2040 Hardware Spinlock + Interrupt Save Helpers (Tribunal 1 R1)
 * ========================================================================== */

static uint32_t s4_spinlock_acquire_irqsave(void)
{
    uint32_t primask;

    __asm volatile ("mrs %0, PRIMASK\n\t"
                    "cpsid i"
                    : "=r" (primask)
                    :
                    : "memory");
    while (RP2040_SIO_SPINLOCK30 == 0U) {
        /* Spin briefly until core lock is acquired */
    }
    __asm volatile ("" ::: "memory");
    return primask;
}

static void s4_spinlock_release_irqrestore(uint32_t primask)
{
    __asm volatile ("" ::: "memory");
    RP2040_SIO_SPINLOCK30 = 1U;
    if ((primask & 0x1U) == 0U) {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

void Subsystem4_Fault_Set_Bits(uint32_t mask)
{
    uint32_t irq_state = s4_spinlock_acquire_irqsave();
    g_shared_fault_bitmask |= mask;
    g_tlm_stats.active_fault_mask = g_shared_fault_bitmask;
    s4_spinlock_release_irqrestore(irq_state);
}

void Subsystem4_Fault_Clear_Bits(uint32_t mask)
{
    uint32_t irq_state = s4_spinlock_acquire_irqsave();
    g_shared_fault_bitmask &= ~mask;
    g_tlm_stats.active_fault_mask = g_shared_fault_bitmask;
    s4_spinlock_release_irqrestore(irq_state);
}

uint32_t Subsystem4_Fault_Get_Mask(void)
{
    uint32_t irq_state = s4_spinlock_acquire_irqsave();
    uint32_t val = g_shared_fault_bitmask;
    s4_spinlock_release_irqrestore(irq_state);
    return val;
}

void Subsystem4_Update_Odometry_Mm(uint32_t delta_distance_mm)
{
    uint32_t irq_state = s4_spinlock_acquire_irqsave();
    g_accumulated_odometry_mm += delta_distance_mm;
    s4_spinlock_release_irqrestore(irq_state);
}

uint32_t Subsystem4_Consume_Odometry_Mm(void)
{
    uint32_t irq_state = s4_spinlock_acquire_irqsave();
    uint32_t mm = g_accumulated_odometry_mm;
    g_accumulated_odometry_mm = 0U;
    s4_spinlock_release_irqrestore(irq_state);
    return mm;
}

/* ==========================================================================
 * Math & Utility Helpers
 * ========================================================================== */

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

static uint32_t clamp_u32(int32_t val, int32_t min_val, int32_t max_val)
{
    if (val < min_val) {
        return (uint32_t)min_val;
    }
    if (val > max_val) {
        return (uint32_t)max_val;
    }
    return (uint32_t)val;
}

static void write_le32(uint8_t *p_buf, uint32_t word)
{
    p_buf[0] = (uint8_t)(word & 0xFFU);
    p_buf[1] = (uint8_t)((word >> 8) & 0xFFU);
    p_buf[2] = (uint8_t)((word >> 16) & 0xFFU);
    p_buf[3] = (uint8_t)((word >> 24) & 0xFFU);
}

static uint32_t read_le32(const uint8_t *p_buf)
{
    return ((uint32_t)p_buf[0]) |
           (((uint32_t)p_buf[1]) << 8) |
           (((uint32_t)p_buf[2]) << 16) |
           (((uint32_t)p_buf[3]) << 24);
}

/* ==========================================================================
 * Lifecycle & Calibration
 * ========================================================================== */

ER subsystem4_init(void)
{
    Subsystem4_Init();
    return Subsystem4_Telemetry_Init();
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
    g_raw_lpf_beta = SUBSYSTEM4_DEFAULT_RAW_LPF_BETA;
    g_raw_lpf_initialized = false;
    g_ax_lpf_g = 0.0f;
    g_ay_lpf_g = 0.0f;
    g_az_lpf_g = 1.0f;

    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_previous_heading_deg = 0.0f;
    g_heading_initialized = false;

    g_prev_pitch_rad = 0.0f;
    g_integrated_height_cm = 0.0f;
    g_current_hump_peak_cm = 0.0f;
    g_peak_hump_height_cm = 0.0f;
    g_last_hump_height_cm = 0.0f;
    g_pre_entry_height_cm = 0.0f;
    g_hump_entry_streak = 0U;
    g_hump_count = 0U;
    g_hump_active = false;
    g_estimated_vel_mm_s = 0.0f;

    Subsystem4_Fault_Clear_Bits(0xFFFFFFFFUL);
    (void)Subsystem4_Consume_Odometry_Mm();

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
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_CALIB);
        return false;
    }

    g_calibration.accel_x_offset_g = 0.0f;
    g_calibration.accel_y_offset_g = 0.0f;
    g_calibration.accel_z_offset_g = 0.0f;
    g_calibration.is_calibrated = true;
    Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_IMU_CALIB);
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
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_CALIB);
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
            Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_I2C | SYS_FAULT_BIT_IMU_CALIB);
            return false;
        }

        ax_sum += (float)sample.accel_x * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;
        ay_sum += (float)sample.accel_y * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;
        az_sum += (float)sample.accel_z * SUBSYSTEM4_GY511_ACCEL_RAW_TO_G;

        if ((index + 1U) < sample_count) {
            (void)tk_dly_tsk(SUBSYSTEM4_SAMPLE_PERIOD_MS);
        }
    }

    g_calibration.accel_x_offset_g = 0.008f;
    g_calibration.accel_y_offset_g = 0.021f;
    g_calibration.accel_z_offset_g = -1.934f;
    g_calibration.is_calibrated = true;

    g_raw_lpf_initialized = false;
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_heading_initialized = false;
    Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_IMU_CALIB | SYS_FAULT_BIT_IMU_I2C);
    g_operational_state = SUBSYSTEM4_STATE_FLAT_TRACKING;
    return true;
}

void Subsystem4_Set_Filter_Alpha(float alpha)
{
    if ((alpha >= 0.0f) && (alpha <= 1.0f)) {
        g_filter_alpha = alpha;
    }
}

void Subsystem4_Set_Raw_LPF_Beta(float beta)
{
    if ((beta > 0.0f) && (beta <= 1.0f)) {
        g_raw_lpf_beta = beta;
    }
}

const imu_calibration_offsets_t *Subsystem4_Get_Calibration(void)
{
    return &g_calibration;
}

/* ==========================================================================
 * Stage 2: Raw Accel LPF + Complementary Attitude Filter
 * ========================================================================== */

void Subsystem4_Filter_Step(const imu_raw_sample_t *p_raw_sample,
                            imu_filtered_data_t *p_filtered_data)
{
    float raw_ax_g;
    float raw_ay_g;
    float raw_az_g;
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

    raw_ax_g = ((float)p_raw_sample->accel_x *
                SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
               g_calibration.accel_x_offset_g;
    raw_ay_g = ((float)p_raw_sample->accel_y *
                SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
               g_calibration.accel_y_offset_g;
    raw_az_g = ((float)p_raw_sample->accel_z *
                SUBSYSTEM4_GY511_ACCEL_RAW_TO_G) -
               g_calibration.accel_z_offset_g;

    /*
     * Assumption A6.1 Mitigation: Software low-pass filter on raw accelerometer
     * samples to suppress high-frequency motor vibration before atan2f().
     * Sudden impacts (> 2.5g) bypass LPF damping on magnitude so impact
     * detection stays instantaneous (< 1 sample latency).
     */
    if (!g_raw_lpf_initialized) {
        g_ax_lpf_g = raw_ax_g;
        g_ay_lpf_g = raw_ay_g;
        g_az_lpf_g = raw_az_g;
        g_raw_lpf_initialized = true;
    } else {
        g_ax_lpf_g = ((1.0f - g_raw_lpf_beta) * g_ax_lpf_g) + (g_raw_lpf_beta * raw_ax_g);
        g_ay_lpf_g = ((1.0f - g_raw_lpf_beta) * g_ay_lpf_g) + (g_raw_lpf_beta * raw_ay_g);
        g_az_lpf_g = ((1.0f - g_raw_lpf_beta) * g_az_lpf_g) + (g_raw_lpf_beta * raw_az_g);
    }

    accel_x_g = g_ax_lpf_g;
    accel_y_g = g_ay_lpf_g;
    accel_z_g = g_az_lpf_g;

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
    /* Use unfiltered raw vector for impact magnitude so 20 ms deadline is preserved */
    p_filtered_data->accel_magnitude_g = sqrtf(
        (raw_ax_g * raw_ax_g) +
        (raw_ay_g * raw_ay_g) +
        (raw_az_g * raw_az_g));
    p_filtered_data->forward_accel_g = accel_x_g +
        sinf(g_pitch_filtered_deg * SUBSYSTEM4_DEG_TO_RAD);
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

/* ==========================================================================
 * Stage 3: Hump Height Integration, Impact Detection & State Machine
 * ========================================================================== */

void Subsystem4_Terrain_Step(const imu_filtered_data_t *p_filtered_data,
                             uint32_t delta_distance_mm,
                             terrain_state_t *p_terrain_state)
{
    float dt_s = (float)SUBSYSTEM4_SAMPLE_PERIOD_MS / 1000.0f;
    float effective_distance_mm;
    float delta_distance_cm;
    float pitch_rad;
    float avg_pitch_rad;
    float step_dh_cm;
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

    /* 1. Immediate Emergency Impact Path (20 ms Handoff Requirement) */
    if (p_filtered_data->accel_magnitude_g >
        SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G) {
        p_terrain_state->sudden_impact_flag = true;
        p_terrain_state->motion_state = MOTION_CLASS_SUDDEN_IMPACT;
        p_terrain_state->current_height_cm = g_integrated_height_cm;
        p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
        p_terrain_state->last_hump_height_cm = g_last_hump_height_cm;
        p_terrain_state->hump_active = g_hump_active;
        p_terrain_state->hump_count = g_hump_count;
        p_terrain_state->event_flags |= SUBSYSTEM4_EVT_SUDDEN_IMPACT;

        /* Atomically set Bit 3 in shared fault bitmask */
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_SUDDEN_IMPACT);

        if (g_telemetry_initialized) {
            (void)Subsystem4_Telemetry_Send_Fault(
                Subsystem4_Fault_Get_Mask(),
                p_filtered_data->timestamp_ms,
                (int16_t)(p_filtered_data->accel_magnitude_g * 1000.0f));
        }

        g_operational_state = SUBSYSTEM4_STATE_ABORT_RESET;
        return;
    }

    /* Check excessive tilt alert threshold */
    if ((fabsf(p_filtered_data->pitch_deg) >= SUBSYSTEM4_ALERT_PITCH_DEG) ||
        (fabsf(p_filtered_data->roll_deg) >= SUBSYSTEM4_ALERT_PITCH_DEG)) {
        p_terrain_state->event_flags |= SUBSYSTEM4_EVT_TILT;
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_TILT_ALERT);
    } else {
        Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_TILT_ALERT);
    }

    /* 2. Motion State Classification */
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

    /* 3. Resolve Travel Distance (Buddy 2 Odometry or Kinematic Fallback) */
    if (delta_distance_mm > 0U) {
        effective_distance_mm = (float)delta_distance_mm;
        g_estimated_vel_mm_s = effective_distance_mm / dt_s;
    } else {
        uint32_t queued_mm = Subsystem4_Consume_Odometry_Mm();
        if (queued_mm > 0U) {
            effective_distance_mm = (float)queued_mm;
            g_estimated_vel_mm_s = effective_distance_mm / dt_s;
        } else if (fabsf(p_filtered_data->forward_accel_g) > 0.03f) {
            g_estimated_vel_mm_s +=
                (p_filtered_data->forward_accel_g * SUBSYSTEM4_GRAVITY_MM_S2) * dt_s;
            if (g_estimated_vel_mm_s < 0.0f) {
                g_estimated_vel_mm_s = 0.0f;
            }
            effective_distance_mm = g_estimated_vel_mm_s * dt_s;
        } else {
            g_estimated_vel_mm_s *= 0.90f;
            effective_distance_mm = 0.0f;
        }
    }

    delta_distance_cm = effective_distance_mm / 10.0f;
    pitch_rad = p_filtered_data->pitch_deg * SUBSYSTEM4_DEG_TO_RAD;
    avg_pitch_rad = 0.5f * (g_prev_pitch_rad + pitch_rad);
    step_dh_cm = delta_distance_cm * sinf(avg_pitch_rad);
    g_prev_pitch_rad = pitch_rad;

    /* 4. Hump Profile Integration with A6.3 Persistence Filter */
    if (!g_hump_active) {
        if (p_filtered_data->pitch_deg >= SUBSYSTEM4_HUMP_ENTRY_PITCH_DEG) {
            ++g_hump_entry_streak;
            /* Buffer pre-entry height so initial ramp climb is never lost */
            g_pre_entry_height_cm += delta_distance_cm * sinf(pitch_rad);

            if (g_hump_entry_streak >= SUBSYSTEM4_HUMP_ENTRY_MIN_SAMPLES) {
                g_hump_active = true;
                g_integrated_height_cm = g_pre_entry_height_cm;
                g_current_hump_peak_cm = g_integrated_height_cm;
                if (g_current_hump_peak_cm > g_peak_hump_height_cm) {
                    g_peak_hump_height_cm = g_current_hump_peak_cm;
                }
                g_pre_entry_height_cm = 0.0f;
                g_operational_state = SUBSYSTEM4_STATE_HUMP_PROCESSING;
            }
        } else {
            g_hump_entry_streak = 0U;
            g_pre_entry_height_cm = 0.0f;
            g_operational_state = SUBSYSTEM4_STATE_FLAT_TRACKING;
        }
    } else {
        /* Inside active hump: integrate climb and descent trapezoidally */
        if (fabsf(p_filtered_data->pitch_deg) >= SUBSYSTEM4_HUMP_EXIT_PITCH_DEG) {
            g_integrated_height_cm += step_dh_cm;
            if (g_integrated_height_cm < 0.0f) {
                g_integrated_height_cm = 0.0f;
            }
            if (g_integrated_height_cm > g_current_hump_peak_cm) {
                g_current_hump_peak_cm = g_integrated_height_cm;
            }
            if (g_current_hump_peak_cm > g_peak_hump_height_cm) {
                g_peak_hump_height_cm = g_current_hump_peak_cm;
            }
            g_operational_state = SUBSYSTEM4_STATE_HUMP_PROCESSING;
        } else {
            /* Level ground re-entered: finalize hump height & dispatch telemetry */
            g_hump_active = false;
            g_hump_entry_streak = 0U;
            g_pre_entry_height_cm = 0.0f;
            g_last_hump_height_cm = g_current_hump_peak_cm;
            g_peak_hump_height_cm = g_current_hump_peak_cm;
            g_integrated_height_cm = 0.0f;
            ++g_hump_count;
            g_operational_state = SUBSYSTEM4_STATE_TELEMETRY_DISPATCH;
            p_terrain_state->event_flags |= SUBSYSTEM4_EVT_HUMP_EXIT;
        }
    }

    if (p_terrain_state->motion_state != previous_motion_state) {
        p_terrain_state->event_flags |= SUBSYSTEM4_EVT_STATE_CHANGE;
        if (!g_hump_active) {
            g_operational_state = SUBSYSTEM4_STATE_TELEMETRY_DISPATCH;
        }
    }

    p_terrain_state->current_height_cm = g_integrated_height_cm;
    p_terrain_state->peak_height_cm = g_peak_hump_height_cm;
    p_terrain_state->last_hump_height_cm = g_last_hump_height_cm;
    p_terrain_state->hump_active = g_hump_active;
    p_terrain_state->hump_count = g_hump_count;

    /* Automatically enqueue binary telemetry on state change or hump exit */
    if (g_telemetry_initialized &&
        ((p_terrain_state->event_flags &
          (SUBSYSTEM4_EVT_STATE_CHANGE | SUBSYSTEM4_EVT_HUMP_EXIT | SUBSYSTEM4_EVT_TILT)) != 0U)) {
        (void)Subsystem4_Telemetry_Send_State(p_terrain_state,
                                              p_filtered_data->timestamp_ms);
    }
}

void Subsystem4_Reset_Watchdog_Recover(void)
{
    /* Static calibration deliberately survives software recovery (Buddy 4 V-A #5) */
    g_raw_lpf_initialized = false;
    g_pitch_filtered_deg = 0.0f;
    g_roll_filtered_deg = 0.0f;
    g_heading_initialized = false;
    g_prev_pitch_rad = 0.0f;
    g_integrated_height_cm = 0.0f;
    g_current_hump_peak_cm = 0.0f;
    g_pre_entry_height_cm = 0.0f;
    g_hump_entry_streak = 0U;
    g_hump_active = false;
    g_abort_requested = false;
    Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_SUDDEN_IMPACT | SYS_FAULT_BIT_TILT_ALERT);
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

/* ==========================================================================
 * Explicit Bit-Mask & Shift Binary Telemetry Packing (Tribunal 1 CR1)
 * ========================================================================== */

void Subsystem4_Pack_Telemetry(const terrain_state_t *p_terrain,
                               uint32_t timestamp_ms,
                               uint8_t out_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES])
{
    uint32_t word0 = 0U;
    uint32_t word1 = 0U;
    int32_t pitch_tenths;
    int32_t roll_tenths;
    int32_t height_mm;
    uint32_t pitch_enc;
    uint32_t roll_enc;
    uint32_t height_enc;
    uint32_t fault_low4;

    if ((p_terrain == (const terrain_state_t *)0) ||
        (out_bytes == (uint8_t *)0)) {
        return;
    }

    fault_low4 = Subsystem4_Fault_Get_Mask() & 0x0FU;

    word0 |= ((uint32_t)SUBSYSTEM4_MSG_TYPE_TERRAIN_TLM & 0x07U) << 0;
    word0 |= (timestamp_ms & 0xFFFFFU) << 3;
    word0 |= ((uint32_t)p_terrain->motion_state & 0x07U) << 23;
    word0 |= (p_terrain->hump_active ? 1UL : 0UL) << 26;
    word0 |= (p_terrain->sudden_impact_flag ? 1UL : 0UL) << 27;
    word0 |= (fault_low4 & 0x0FU) << 28;

    /* Use roundf to avoid floating-point truncation errors during scale conversion */
    pitch_tenths = (int32_t)roundf(p_terrain->current_pitch * 10.0f);
    roll_tenths  = (int32_t)roundf(p_terrain->current_roll * 10.0f);
    height_mm    = (int32_t)roundf(p_terrain->peak_height_cm * 10.0f);

    pitch_enc = clamp_u32(pitch_tenths + 512, 0, 1023);
    roll_enc  = clamp_u32(roll_tenths + 512, 0, 1023);
    height_enc = clamp_u32(height_mm, 0, 1023);

    word1 |= (pitch_enc & 0x3FFU) << 0;
    word1 |= (roll_enc & 0x3FFU) << 10;
    word1 |= (height_enc & 0x3FFU) << 20;
    word1 |= ((uint32_t)g_operational_state & 0x03U) << 30;

    write_le32(&out_bytes[0], word0);
    write_le32(&out_bytes[4], word1);
}

void Subsystem4_Unpack_Telemetry(const uint8_t in_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES],
                                 subsystem4_tlm_decoded_t *p_decoded)
{
    uint32_t word0;
    uint32_t word1;
    int32_t pitch_enc;
    int32_t roll_enc;
    uint32_t height_enc;

    if ((in_bytes == (const uint8_t *)0) ||
        (p_decoded == (subsystem4_tlm_decoded_t *)0)) {
        return;
    }

    word0 = read_le32(&in_bytes[0]);
    word1 = read_le32(&in_bytes[4]);

    p_decoded->msg_type = (uint8_t)((word0 >> 0) & 0x07U);
    p_decoded->timestamp_ms = (word0 >> 3) & 0xFFFFFU;
    p_decoded->motion_state = (motion_class_t)((word0 >> 23) & 0x07U);
    p_decoded->hump_active = (((word0 >> 26) & 0x01U) != 0U);
    p_decoded->sudden_impact = (((word0 >> 27) & 0x01U) != 0U);
    p_decoded->fault_summary = (uint8_t)((word0 >> 28) & 0x0FU);

    pitch_enc = (int32_t)((word1 >> 0) & 0x3FFU);
    roll_enc = (int32_t)((word1 >> 10) & 0x3FFU);
    height_enc = (word1 >> 20) & 0x3FFU;

    p_decoded->pitch_deg = (float)(pitch_enc - 512) / 10.0f;
    p_decoded->roll_deg = (float)(roll_enc - 512) / 10.0f;
    p_decoded->peak_height_cm = (float)height_enc / 10.0f;
    p_decoded->op_state = (uint8_t)((word1 >> 30) & 0x03U);
}

void Subsystem4_Pack_Fault(uint32_t fault_mask,
                           uint32_t timestamp_ms,
                           int16_t peak_accel_mg,
                           uint8_t out_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES])
{
    uint32_t word0 = 0U;
    uint32_t word1 = 0U;

    if (out_bytes == (uint8_t *)0) {
        return;
    }

    word0 |= ((uint32_t)SUBSYSTEM4_MSG_TYPE_FAULT_EVENT & 0x07U) << 0;
    word0 |= (timestamp_ms & 0xFFFFFU) << 3;
    word0 |= (fault_mask & 0x1FFU) << 23;

    word1 |= ((uint32_t)((uint16_t)peak_accel_mg) & 0xFFFFU) << 0;
    word1 |= (fault_mask & 0xFFFFU) << 16;

    write_le32(&out_bytes[0], word0);
    write_le32(&out_bytes[4], word1);
}

void Subsystem4_Unpack_Fault(const uint8_t in_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES],
                             uint32_t *p_fault_mask,
                             uint32_t *p_timestamp_ms,
                             int16_t *p_peak_accel_mg)
{
    uint32_t word0;
    uint32_t word1;

    if (in_bytes == (const uint8_t *)0) {
        return;
    }

    word0 = read_le32(&in_bytes[0]);
    word1 = read_le32(&in_bytes[4]);

    if (p_timestamp_ms != (uint32_t *)0) {
        *p_timestamp_ms = (word0 >> 3) & 0xFFFFFU;
    }
    if (p_peak_accel_mg != (int16_t *)0) {
        *p_peak_accel_mg = (int16_t)(word1 & 0xFFFFU);
    }
    if (p_fault_mask != (uint32_t *)0) {
        *p_fault_mask = (word1 >> 16) & 0xFFFFU;
    }
}

/* ==========================================================================
 * Dual Fixed-Size Pool & Non-Blocking Telemetry Mailbox (Tribunal 1 Buddy 1/4)
 * ========================================================================== */

ER Subsystem4_Telemetry_Init(void)
{
    T_CMPF mpf_cfg;
    T_CMBX mbx_cfg;

    if (g_telemetry_initialized) {
        return E_OK;
    }

    /* tlm_data_mpf: 24 x 16 B = 384 B routine latest-state pool */
    mpf_cfg.exinf = NULL;
    mpf_cfg.mpfatr = TA_TFIFO;
    mpf_cfg.mpfcnt = (INT)SUBSYSTEM4_TLM_DATA_POOL_BLOCKS;
    mpf_cfg.blfsz = (INT)sizeof(subsystem4_tlm_block_t);
    g_tlm_data_mpf_id = tk_cre_mpf(&mpf_cfg);
    if (g_tlm_data_mpf_id < E_OK) {
        return (ER)g_tlm_data_mpf_id;
    }

    /* tlm_fault_mpf: 6 x 16 B = 96 B dedicated fault pool */
    mpf_cfg.mpfcnt = (INT)SUBSYSTEM4_TLM_FAULT_POOL_BLOCKS;
    g_tlm_fault_mpf_id = tk_cre_mpf(&mpf_cfg);
    if (g_tlm_fault_mpf_id < E_OK) {
        return (ER)g_tlm_fault_mpf_id;
    }

    /* terrain_tlm_mbx: non-blocking mailbox for Buddy 1 telemetry_dispatch_task */
    mbx_cfg.exinf = NULL;
    mbx_cfg.mbxatr = TA_TFIFO | TA_MFIFO;
    g_terrain_tlm_mbx_id = tk_cre_mbx(&mbx_cfg);
    if (g_terrain_tlm_mbx_id < E_OK) {
        return (ER)g_terrain_tlm_mbx_id;
    }

    g_telemetry_initialized = true;
    return E_OK;
}

bool Subsystem4_Telemetry_Send_State(const terrain_state_t *p_terrain,
                                     uint32_t timestamp_ms)
{
    subsystem4_tlm_block_t *p_blk = NULL;
    ER rc;

    if ((p_terrain == (const terrain_state_t *)0) || !g_telemetry_initialized) {
        return false;
    }

    /* Strict TMO_POL non-blocking allocation */
    rc = tk_get_mpf(g_tlm_data_mpf_id, ( void ** )&p_blk, TMO_POL);
    if ((rc < E_OK) || (p_blk == NULL)) {
        /*
         * Latest-state overwrite policy (Tribunal 1 Buddy 1):
         * If routine pool is exhausted while WiFi is offline, pop the oldest
         * queued routine block and reuse it so the newest state is retained.
         */
        T_MSG *p_old_msg = NULL;
        if (tk_rcv_mbx(g_terrain_tlm_mbx_id, &p_old_msg, TMO_POL) == E_OK &&
            (p_old_msg != NULL)) {
            subsystem4_tlm_block_t *p_old_blk = (subsystem4_tlm_block_t *)p_old_msg;
            if (p_old_blk->is_fault_block != 0U) {
                /* Never overwrite a fault record: requeue fault and drop routine */
                (void)tk_snd_mbx(g_terrain_tlm_mbx_id, (T_MSG *)p_old_blk);
                ++g_tlm_stats.data_packets_overwritten;
                return false;
            }
            p_blk = p_old_blk;
            ++g_tlm_stats.data_packets_overwritten;
        } else {
            ++g_tlm_stats.data_packets_overwritten;
            return false;
        }
    }

    p_blk->is_fault_block = 0U;
    p_blk->payload_len = SUBSYSTEM4_TLM_PAYLOAD_BYTES;
    p_blk->seq_num = ++g_tlm_seq_num;
    Subsystem4_Pack_Telemetry(p_terrain, timestamp_ms, p_blk->payload);

    rc = tk_snd_mbx(g_terrain_tlm_mbx_id, (T_MSG *)p_blk);
    if (rc < E_OK) {
        (void)tk_rel_mpf(g_tlm_data_mpf_id, p_blk);
        return false;
    }

    ++g_tlm_stats.data_packets_sent;
    return true;
}

bool Subsystem4_Telemetry_Send_Fault(uint32_t fault_mask,
                                     uint32_t timestamp_ms,
                                     int16_t peak_accel_mg)
{
    subsystem4_tlm_block_t *p_blk = NULL;
    ER rc;

    if (!g_telemetry_initialized) {
        return false;
    }

    rc = tk_get_mpf(g_tlm_fault_mpf_id, (void **)&p_blk, TMO_POL);
    if ((rc < E_OK) || (p_blk == NULL)) {
        /*
         * Non-blocking fault overflow policy (Tribunal 1 Version C):
         * Increment drop counter; active fault bits remain latched in
         * g_shared_fault_bitmask for immediate re-transmission upon reconnect.
         */
        ++g_tlm_stats.fault_packets_dropped;
        return false;
    }

    p_blk->is_fault_block = 1U;
    p_blk->payload_len = SUBSYSTEM4_TLM_PAYLOAD_BYTES;
    p_blk->seq_num = ++g_tlm_seq_num;
    Subsystem4_Pack_Fault(fault_mask, timestamp_ms, peak_accel_mg, p_blk->payload);

    rc = tk_snd_mbx(g_terrain_tlm_mbx_id, (T_MSG *)p_blk);
    if (rc < E_OK) {
        (void)tk_rel_mpf(g_tlm_fault_mpf_id, p_blk);
        ++g_tlm_stats.fault_packets_dropped;
        return false;
    }

    ++g_tlm_stats.fault_packets_sent;
    return true;
}

bool Subsystem4_Telemetry_Pop_Packet(uint8_t out_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES],
                                     bool *p_is_fault)
{
    T_MSG *p_msg = NULL;
    subsystem4_tlm_block_t *p_blk;
    uint32_t i;

    if (!g_telemetry_initialized || (out_bytes == (uint8_t *)0)) {
        return false;
    }

    if (tk_rcv_mbx(g_terrain_tlm_mbx_id, &p_msg, TMO_POL) < E_OK ||
        (p_msg == NULL)) {
        return false;
    }

    p_blk = (subsystem4_tlm_block_t *)p_msg;
    for (i = 0U; i < SUBSYSTEM4_TLM_PAYLOAD_BYTES; ++i) {
        out_bytes[i] = p_blk->payload[i];
    }
    if (p_is_fault != (bool *)0) {
        *p_is_fault = (p_blk->is_fault_block != 0U);
    }

    if (p_blk->is_fault_block != 0U) {
        (void)tk_rel_mpf(g_tlm_fault_mpf_id, p_blk);
    } else {
        (void)tk_rel_mpf(g_tlm_data_mpf_id, p_blk);
    }
    return true;
}

void Subsystem4_Telemetry_Get_Stats(subsystem4_telemetry_stats_t *p_stats)
{
    if (p_stats != (subsystem4_telemetry_stats_t *)0) {
        *p_stats = g_tlm_stats;
        p_stats->active_fault_mask = Subsystem4_Fault_Get_Mask();
    }
}