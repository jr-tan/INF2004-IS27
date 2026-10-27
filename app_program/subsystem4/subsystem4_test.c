/**
 * @file subsystem4_test.c
 * @brief On-target algorithm and GY-511 integration tests for Buddy 4.
 */
#include <stdbool.h>
#include <stdint.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"

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
                "Nominal calibration accepts configured timeout");

    sample.accel_z = 1000;
    sample.timestamp_ms = 100U;
    Subsystem4_Filter_Step(&sample, &filtered);
    TEST_ASSERT(fabsf(filtered.pitch_deg) <= SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX,
                "Flat-ground pitch remains within +/- 2 degrees");
    TEST_ASSERT(fabsf(filtered.roll_deg) <= SUBSYSTEM4_TILT_STEADY_STATE_ERR_MAX,
                "Flat-ground roll remains within +/- 2 degrees");
}

static void test_impact_and_recovery(void)
{
    imu_raw_sample_t sample = {0};
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};

    tm_printf((UB *)"\n=== Test 2: Impact and recovery ===\n");
    sample.accel_x = 32767;
    sample.accel_y = 32767;
    sample.accel_z = 32767;
    Subsystem4_Filter_Step(&sample, &filtered);
    Subsystem4_Terrain_Step(&filtered, 0U, &terrain);
    TEST_ASSERT(terrain.sudden_impact_flag,
                "Acceleration magnitude above 2.5 g triggers impact");
    TEST_ASSERT(terrain.motion_state == MOTION_CLASS_SUDDEN_IMPACT,
                "Impact classification is reported");

    Subsystem4_Reset_Watchdog_Recover();
    TEST_ASSERT(Subsystem4_Get_Calibration()->is_calibrated,
                "Recovery preserves calibration");
}

static void test_hump_state(void)
{
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};
    uint32_t index;

    tm_printf((UB *)"\n=== Test 3: Hump state ===\n");
    Subsystem4_Init();
    filtered.pitch_deg = 10.0f;
    filtered.accel_magnitude_g = 1.0f;

    for (index = 0U; index < 20U; ++index) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }
    TEST_ASSERT(terrain.hump_active, "Positive slope enters hump processing");
    TEST_ASSERT(Subsystem4_Get_Operational_State() ==
                SUBSYSTEM4_STATE_HUMP_PROCESSING,
                "Operational state reports hump processing");
}

static void test_gy511_hardware(void)
{
    gy511_raw_sample_t sample;

    tm_printf((UB *)"\n=== Test 4: Physical GY-511 integration (I2C0 GP4/GP5) ===\n");
    if (!GY511_Init()) {
        TEST_ASSERT(false, "GY-511 accelerometer and magnetometer are identified");
        tm_printf((UB *)"  GY-511 init error stage=%lu\n",
                  (UW)GY511_Get_Error_Stage());
        return;
    }
    TEST_ASSERT(GY511_Get_Accel_WhoAmI() == GY511_ACCEL_WHO_AM_I_VALUE,
                "LSM303DLHC accelerometer WHO_AM_I is 0x33");

    if (!GY511_ReadRawSample(&sample)) {
        TEST_ASSERT(false, "Accelerometer and magnetometer samples are readable");
        return;
    }
    TEST_ASSERT(true, "Accelerometer and magnetometer samples are readable");
}

void subsystem4_test(void)
{
    g_tests_passed = 0U;
    g_tests_failed = 0U;

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" BUDDY 4: RP2040 GY-511 TERRAIN TESTS\n");
    tm_printf((UB *)"===============================================\n");

    test_init_and_calibration();
    test_impact_and_recovery();
    test_hump_state();
    test_gy511_hardware();

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" TOTAL PASSED: %u\n", (UW)g_tests_passed);
    tm_printf((UB *)" TOTAL FAILED: %u\n", (UW)g_tests_failed);
    tm_printf((UB *)"===============================================\n");
}
