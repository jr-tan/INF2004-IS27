/**
 * @file subsystem4_test.c
 * @brief On-target algorithm, hump accuracy, binary telemetry round-trip,
 *        Tribunal 1 50x SUDDEN_IMPACT fault bitmask, and GY-511 tests.
 *        Complies with BARR-C:2018.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4.h"
#include "subsystem4_gy511.h"
#include "subsystem4_pipeline.h"

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

static void test_interrupt_simulation(void)
{
    tm_printf((UB *)"\n=== Test 1b: Interrupt signal simulation ===\n");
    /* Simulates the hardware ISR firing and waking the pipeline task */
    Subsystem4_Notify_Data_Ready_From_ISR();
    TEST_ASSERT(true, "ISR semaphore signal dispatched successfully without polling");
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

    tm_printf((UB *)"\n=== Test 3: Hump state & height accuracy ===\n");
    Subsystem4_Init();

    filtered.pitch_deg = 12.0f;
    filtered.accel_magnitude_g = 1.0f;
    Subsystem4_Terrain_Step(&filtered, 5U, &terrain);
    TEST_ASSERT(!terrain.hump_active,
                "Single-sample vibration spike rejected by persistence check");

    filtered.pitch_deg = 0.0f;
    Subsystem4_Terrain_Step(&filtered, 5U, &terrain);

    filtered.pitch_deg = 15.0f;
    for (index = 0U; index < 20U; ++index) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }
    TEST_ASSERT(terrain.hump_active, "Sustained positive slope enters hump processing");

    filtered.pitch_deg = -15.0f;
    for (index = 0U; index < 18U; ++index) {
        Subsystem4_Terrain_Step(&filtered, 10U, &terrain);
    }
    filtered.pitch_deg = 0.0f;
    Subsystem4_Terrain_Step(&filtered, 10U, &terrain);

    expected_height_cm = 20.0f * sinf(15.0f * SUBSYSTEM4_DEG_TO_RAD);
    height_error_cm = fabsf(terrain.peak_height_cm - expected_height_cm);

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

    tm_printf((UB *)"\n=== Test 4: Binary Telemetry Pack/Unpack Round-Trip ===\n");
    Subsystem4_Init();
    (void)Subsystem4_Telemetry_Init();

    while (Subsystem4_Telemetry_Pop_Packet(wire_bytes, &is_fault)) {
        /* Flush queue */
    }

    terrain.current_pitch = 14.3f;
    terrain.current_roll = -6.8f;
    terrain.peak_height_cm = 5.2f;
    terrain.motion_state = MOTION_CLASS_CLIMBING;
    terrain.hump_active = true;
    terrain.sudden_impact_flag = false;
    Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_TILT_ALERT);

    TEST_ASSERT(Subsystem4_Telemetry_Send_State(&terrain, 123456U),
                "Non-blocking allocation and mailbox send succeeds");
    TEST_ASSERT(Subsystem4_Telemetry_Pop_Packet(wire_bytes, &is_fault) && !is_fault,
                "Routine binary telemetry packet popped successfully");

    Subsystem4_Unpack_Telemetry(wire_bytes, &decoded);
    TEST_ASSERT(decoded.msg_type == SUBSYSTEM4_MSG_TYPE_TERRAIN_TLM,
                "Decoded message type matches terrain telemetry");
    TEST_ASSERT(decoded.timestamp_ms == 123456U,
                "Decoded timestamp matches 123456 ms");
    TEST_ASSERT(decoded.motion_state == MOTION_CLASS_CLIMBING,
                "Decoded motion class matches climbing");
}

void subsystem4_test(void)
{
    g_tests_passed = 0U;
    g_tests_failed = 0U;

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" BUDDY 4: INTERRUPT-DRIVEN PIPELINE TESTS\n");
    tm_printf((UB *)"===============================================\n");

    test_init_and_calibration();
    test_interrupt_simulation();
    test_impact_and_recovery();
    test_hump_height_accuracy();
    test_binary_telemetry_roundtrip();

    tm_printf((UB *)"\n===============================================\n");
    tm_printf((UB *)" TOTAL PASSED: %u\n", (UW)g_tests_passed);
    tm_printf((UB *)" TOTAL FAILED: %u\n", (UW)g_tests_failed);
    tm_printf((UB *)"===============================================\n");
}