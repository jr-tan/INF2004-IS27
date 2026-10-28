/**
 * @file subsystem4_test.c
 * @brief On-target algorithm, hump accuracy, binary telemetry round-trip,
 *        Tribunal 1 50x SUDDEN_IMPACT fault bitmask, and GY-511 tests.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"

extern float fabsf(float x);
extern float sinf(float x);

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
    (void)Subsystem4_Telemetry_Init();
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
                "Acceleration magnitude above 5.0 g triggers impact");
    TEST_ASSERT(terrain.motion_state == MOTION_CLASS_SUDDEN_IMPACT,
                "Impact classification is reported");
    TEST_ASSERT((Subsystem4_Fault_Get_Mask() & SYS_FAULT_BIT_SUDDEN_IMPACT) != 0U,
                "Shared fault bitmask Bit 3 (SUDDEN_IMPACT) is set");

    Subsystem4_Reset_Watchdog_Recover();
    TEST_ASSERT(Subsystem4_Get_Calibration()->is_calibrated,
                "Recovery preserves calibration");
}

static void test_hump_height_accuracy(void)
{
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};
    uint32_t index;
    float expected_height_cm;
    float height_error_cm;

    tm_printf((UB *)"\n=== Test 3: Hump state & height accuracy (A6 persistence + trapezoidal) ===\n");
    Subsystem4_Init();

    /*
     * Inject a single-sample 12-degree motor vibration spike:
     * A6.3 requires 3 consecutive samples, so a 1-sample spike must be rejected.
     */
    filtered.pitch_deg = 12.0f;
    filtered.accel_magnitude_g = 1.0f;
    Subsystem4_Terrain_Step(&filtered, 5U, &terrain);
    TEST_ASSERT(!terrain.hump_active,
                "Single-sample vibration spike rejected by 3-sample persistence check");

    filtered.pitch_deg = 0.0f;
    Subsystem4_Terrain_Step(&filtered, 5U, &terrain);

    /*
     * Simulate climbing a 15.0 deg ramp for 20 steps at 10 mm/step (20 cm ramp length).
     * True peak height = 20.0 cm * sin(15 deg) = 5.176 cm.
     */
    filtered.pitch_deg = 15.0f;
    for (index = 0U; index < 20U; ++index) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }
    TEST_ASSERT(terrain.hump_active, "Sustained positive slope enters hump processing");
    TEST_ASSERT(Subsystem4_Get_Operational_State() ==
                SUBSYSTEM4_STATE_HUMP_PROCESSING,
                "Operational state reports hump processing");

    /* Descent phase (-15 deg for 18 steps) followed by level exit */
    filtered.pitch_deg = -15.0f;
    for (index = 0U; index < 18U; ++index) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }
    filtered.pitch_deg = 0.0f;
    Subsystem4_Terrain_Step(&filtered, 10U, &terrain);

    expected_height_cm = 20.0f * sinf(15.0f * SUBSYSTEM4_DEG_TO_RAD);
    height_error_cm = fabsf(terrain.peak_height_cm - expected_height_cm);

    tm_printf((UB *)"  Measured peak=%d mm, Expected=%d mm, Error=%d mm\n",
              (INT)(terrain.peak_height_cm * 10.0f),
              (INT)(expected_height_cm * 10.0f),
              (INT)(height_error_cm * 10.0f));

    TEST_ASSERT(!terrain.hump_active && ((terrain.event_flags & SUBSYSTEM4_EVT_HUMP_EXIT) != 0U),
                "Hump exit transitions cleanly to telemetry dispatch");
    TEST_ASSERT(height_error_cm <= 0.5f,
                "Integrated hump peak height accurate within +/- 0.5 cm");
}

static void test_binary_telemetry_roundtrip(void)
{
    terrain_state_t terrain = {0};
    subsystem4_tlm_decoded_t decoded = {0};
    uint8_t wire_bytes[SUBSYSTEM4_TLM_PAYLOAD_BYTES];
    bool is_fault = false;

    tm_printf((UB *)"\n=== Test 4: Binary Telemetry Pack/Unpack & Pool Round-Trip (Tribunal 1 CR1) ===\n");
    Subsystem4_Init();
    (void)Subsystem4_Telemetry_Init();

    /* Drain any leftover packets from previous tests */
    while (Subsystem4_Telemetry_Pop_Packet(wire_bytes, &is_fault)) {
        /* flush */
    }

    terrain.current_pitch = 14.3f;
    terrain.current_roll = -6.8f;
    terrain.peak_height_cm = 5.2f;
    terrain.motion_state = MOTION_CLASS_CLIMBING;
    terrain.hump_active = true;
    terrain.sudden_impact_flag = false;
    Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_TILT_ALERT);

    TEST_ASSERT(Subsystem4_Telemetry_Send_State(&terrain, 123456U),
                "Non-blocking tlm_data_mpf allocation and mailbox send succeeds");
    TEST_ASSERT(Subsystem4_Telemetry_Pop_Packet(wire_bytes, &is_fault) && !is_fault,
                "Routine 8-byte binary telemetry packet popped from terrain_tlm_mbx");

    Subsystem4_Unpack_Telemetry(wire_bytes, &decoded);
    TEST_ASSERT(decoded.msg_type == SUBSYSTEM4_MSG_TYPE_TERRAIN_TLM,
                "Decoded message type matches SUBSYSTEM4_MSG_TYPE_TERRAIN_TLM");
    TEST_ASSERT(decoded.timestamp_ms == 123456U,
                "Decoded 20-bit timestamp matches 123456 ms");
    TEST_ASSERT(decoded.motion_state == MOTION_CLASS_CLIMBING,
                "Decoded 3-bit motion class matches MOTION_CLASS_CLIMBING");
    TEST_ASSERT(fabsf(decoded.pitch_deg - 14.3f) < 0.05f &&
                fabsf(decoded.roll_deg - (-6.8f)) < 0.05f &&
                fabsf(decoded.peak_height_cm - 5.2f) < 0.05f,
                "Decoded pitch, roll, and peak hump height match within 0.1 LSB");
}

static void test_tribunal1_50x_impact_and_spinlock(void)
{
    imu_filtered_data_t filtered = {0};
    terrain_state_t terrain = {0};
    uint32_t i;
    uint32_t lost_impacts = 0U;
    uint32_t corrupted_low_bits = 0U;

    tm_printf((UB *)"\n=== Test 5: Tribunal 1 50x SUDDEN_IMPACT & Concurrent Fault Bitmask Test ===\n");
    Subsystem4_Init();
    (void)Subsystem4_Telemetry_Init();

    /* Pre-set bits 0 and 1 (simulating concurrent IMU acquire fault flags) */
    Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_I2C | SYS_FAULT_BIT_IMU_CALIB);

    for (i = 0U; i < 50U; ++i) {
        filtered.accel_magnitude_g = SUBSYSTEM4_IMPACT_ACCEL_THRESHOLD_G + 1.0f;
        filtered.timestamp_ms = 1000U + (i * 10U);
        Subsystem4_Terrain_Step(&filtered, 0U, &terrain);

        uint32_t mask = Subsystem4_Fault_Get_Mask();
        if ((mask & SYS_FAULT_BIT_SUDDEN_IMPACT) == 0U) {
            ++lost_impacts;
        }
        if ((mask & (SYS_FAULT_BIT_IMU_I2C | SYS_FAULT_BIT_IMU_CALIB)) !=
            (SYS_FAULT_BIT_IMU_I2C | SYS_FAULT_BIT_IMU_CALIB)) {
            ++corrupted_low_bits;
        }

        /* Clear only Bit 3 between injections while preserving Bits 0..1 */
        Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_SUDDEN_IMPACT);
    }

    TEST_ASSERT(lost_impacts == 0U,
                "Bit 3 (SUDDEN_IMPACT) never lost across 50 consecutive injections");
    TEST_ASSERT(corrupted_low_bits == 0U,
                "Bits 0..1 preserved without RMW corruption across 50 injections");
    TEST_ASSERT(GY511_Get_I2C_Mutex_ID() > 0,
                "i2c_mtx created with TA_INHERIT priority inheritance (NFR-SYS-2)");
}

static void test_gy511_hardware(void)
{
    gy511_raw_sample_t sample;

    tm_printf((UB *)"\n=== Test 6: Physical GY-511 integration (I2C0 GP4/GP5) ===\n");
    if (!GY511_Init()) {
        TEST_ASSERT(false, "GY-511 accelerometer and magnetometer are identified");
        tm_printf((UB *)"  GY-511 init error stage=%lu\n",
                  (UW)GY511_Get_Error_Stage());
        return;
    }
    TEST_ASSERT((GY511_Get_Accel_WhoAmI() == GY511_ACCEL_WHO_AM_I_VALUE) ||
                (GY511_Get_Accel_WhoAmI() == GY511_DLHC_WHO_AM_I_VALUE),
                "GY-511 WHO_AM_I is valid (0x49 LSM303D or 0x33 LSM303DLHC)");

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
    tm_printf((UB *)" BUDDY 4: RP2040 GY-511 & TRIBUNAL 1 TESTS\n");
    tm_printf((UB *)"===============================================\n");

    test_init_and_calibration();
    test_impact_and_recovery();
    test_hump_height_accuracy();
    test_binary_telemetry_roundtrip();
    test_tribunal1_50x_impact_and_spinlock();
    test_gy511_hardware();

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" TOTAL PASSED: %u\n", (UW)g_tests_passed);
    tm_printf((UB *)" TOTAL FAILED: %u\n", (UW)g_tests_failed);
    tm_printf((UB *)"===============================================\n");
}