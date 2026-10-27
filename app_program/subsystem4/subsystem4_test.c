/**
 * @file subsystem4_test.c
 * @brief On-target software tests for Buddy 4.
 *
 * Uses μT-Kernel's tm_printf() console API rather than Newlib printf(), which
 * avoids introducing a dependency on the bare-metal Newlib syscall stubs.
 */
#include <stdbool.h>
#include <stdint.h>
#include "subsystem4.h"
#include "subsystem4_mpu6050.h"

extern float fabsf(float x);

static uint32_t g_tests_passed = 0U;
static uint32_t g_tests_failed = 0U;

#define TEST_ASSERT(condition, message)                                      \
    do {                                                                      \
        if (condition) {                                                      \
            ++g_tests_passed;                                                 \
            tm_printf((UB *)"  [PASS] %s\n", (message));                      \
        } else {                                                              \
            ++g_tests_failed;                                                 \
            tm_printf((UB *)"  [FAIL] %s (line %d)\n",                        \
                      (message), (INT)__LINE__);                              \
        }                                                                     \
    } while (0)

static void test_init_and_calibration(void)
{
    imu_raw_sample_t sample = {0};
    imu_filtered_data_t filtered = {0};

    tm_printf((UB *)"\n=== Test 1: Initialization and calibration ===\n");
    Subsystem4_Init();
    TEST_ASSERT(Subsystem4_Calibrate(SUBSYSTEM4_IMU_INIT_TIMEOUT_MS),
                "Nominal calibration succeeds within configured limit");

    sample.accel_z = 16384;
    sample.timestamp_ms = 100U;
    Subsystem4_Filter_Step(&sample, &filtered);
    TEST_ASSERT(fabsf(filtered.pitch_deg) <= SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX,
                "Flat-ground pitch remains within +/- 2 degrees");
    TEST_ASSERT(fabsf(filtered.roll_deg) <= SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX,
                "Flat-ground roll remains within +/- 2 degrees");
}

static void test_pipeline_execution(void)
{
    imu_raw_sample_t sample = {0};
    terrain_state_t terrain = {0};

    tm_printf((UB *)"\n=== Test 2: Pipeline execution ===\n");
    sample.accel_z = 16384;
    sample.timestamp_ms = 10U;
    Subsystem4_Pipeline_Tick(&sample, 10U);
    terrain = *Subsystem4_Get_Terrain_State();
    TEST_ASSERT(terrain.motion_state == MOTION_CLASS_STATIONARY,
                "Level sample completes pipeline and classifies as stationary");
}

static void test_motion_and_impact(void)
{
    imu_raw_sample_t sample = {0};
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};

    tm_printf((UB *)"\n=== Test 3: Motion classification and impact ===\n");
    sample.accel_z = 16384;
    Subsystem4_Filter_Step(&sample, &filtered);
    Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    TEST_ASSERT(terrain.motion_state == MOTION_CLASS_STATIONARY,
                "Level sample is classified as stationary");

    sample.accel_x = 32767;
    sample.accel_y = 32767;
    sample.accel_z = 32767;
    Subsystem4_Filter_Step(&sample, &filtered);
    Subsystem4_Terrain_Step(&filtered, 0U, &terrain);
    TEST_ASSERT(terrain.sudden_impact_flag,
                "Acceleration magnitude above 2.5 g triggers impact");
    TEST_ASSERT(terrain.motion_state == MOTION_CLASS_SUDDEN_IMPACT,
                "Impact classification is reported");
}

static void test_hump_height(void)
{
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};
    uint32_t step;

    tm_printf((UB *)"\n=== Test 4: Hump height estimate ===\n");
    Subsystem4_Init();
    filtered.pitch_deg = 10.0f;
    filtered.accel_magnitude_g = 1.0f;

    for (step = 0U; step < 20U; ++step) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }

    TEST_ASSERT(terrain.hump_active, "Hump is marked active on a positive slope");
    TEST_ASSERT(fabsf(terrain.peak_height_cm - 3.47f) <= 1.0f,
                "Estimated height is close to 20 cm * sin(10 degrees)");
}

static void test_watchdog_recovery(void)
{
    const imu_calibration_offsets_t *p_calibration;

    tm_printf((UB *)"\n=== Test 5: Watchdog recovery ===\n");
    p_calibration = Subsystem4_Get_Calibration();
    TEST_ASSERT(p_calibration->is_calibrated,
                "Calibration is present before recovery");

    Subsystem4_Reset_Watchdog_Recover();
    p_calibration = Subsystem4_Get_Calibration();
    TEST_ASSERT(p_calibration->is_calibrated,
                "Recovery preserves calibration");
}

static void test_yaw_rate(void)
{
    imu_raw_sample_t sample = {0};
    imu_filtered_data_t filtered = {0};

    tm_printf((UB *)"\n=== Test 6: Yaw-rate conversion ===\n");
    sample.accel_z = 16384;
    sample.gyro_z = 11790; /* approximately 90 degrees/second */
    Subsystem4_Filter_Step(&sample, &filtered);
    TEST_ASSERT(fabsf(filtered.yaw_rate_deg_s - 90.0f) <= 1.0f,
                "Raw gyro value converts to approximately 90 degrees/second");
}


static void test_real_mpu6050(void)
{
    imu_raw_sample_t sample = {0};
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};

    tm_printf((UB *)"\n=== Test 7: Physical MPU6050 integration (GP4/GP5) ===\n");
    if (!MPU6050_Init()) {
        TEST_ASSERT(false, "MPU6050 responds to WHO_AM_I over I2C0");
        tm_printf((UB *)"  Connect the sensor to Grove 3 (SDA GP4, SCL GP5, 3V3, GND), then rerun.\n");
        return;
    }
    TEST_ASSERT(true, "MPU6050 responds to WHO_AM_I over I2C0");

    if (!MPU6050_ReadRawSample(&sample)) {
        TEST_ASSERT(false, "A 14-byte accelerometer/gyroscope sample is readable");
        return;
    }
    TEST_ASSERT(true, "A 14-byte accelerometer/gyroscope sample is readable");

    Subsystem4_Init();
    if (!Subsystem4_Calibrate_From_Sensor(1500U)) {
        TEST_ASSERT(false, "Real-sensor calibration completes while robot is still");
        return;
    }
    TEST_ASSERT(Subsystem4_Get_Calibration()->is_calibrated,
                "Real-sensor calibration completes while robot is still");

    if (!MPU6050_ReadRawSample(&sample)) {
        TEST_ASSERT(false, "A post-calibration sample enters the filter/terrain pipeline");
        return;
    }
    Subsystem4_Filter_Step(&sample, &filtered);
    Subsystem4_Terrain_Step(&filtered, 0U, &terrain);
    TEST_ASSERT((filtered.accel_magnitude_g > 0.5f) &&
                (filtered.accel_magnitude_g < 1.5f),
                "Stationary accelerometer magnitude is approximately 1 g");
    tm_printf((UB *)"  Live sample: pitch_cdeg=%d roll_cdeg=%d accel_mg=%d\n",
              (INT)(filtered.pitch_deg * 100.0f),
              (INT)(filtered.roll_deg * 100.0f),
              (INT)(filtered.accel_magnitude_g * 1000.0f));
    tm_printf((UB *)"  For slope validation, run normal mode and slowly tilt the robot forward/backward.\n");
}

void subsystem4_test(void)
{
    g_tests_passed = 0U;
    g_tests_failed = 0U;

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" BUDDY 4: IMU AND TERRAIN MONITORING TESTS\n");
    tm_printf((UB *)"===============================================\n");

    test_init_and_calibration();
    test_pipeline_execution();
    test_motion_and_impact();
    test_hump_height();
    test_watchdog_recovery();
    test_yaw_rate();
    test_real_mpu6050();

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" TOTAL PASSED: %u\n", (UW)g_tests_passed);
    tm_printf((UB *)" TOTAL FAILED: %u\n", (UW)g_tests_failed);
    tm_printf((UB *)"===============================================\n");
}
