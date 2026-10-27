/**
 * @file task_monitor.c
 * @brief Subsystem 4 pipeline coordinator.
 *
 * The caller provides a raw IMU sample and odometry distance increment.
 * Physical sensor reads and μT-Kernel task scheduling belong in the board
 * integration layer because no sensor-driver interface was provided here.
 */
#include <stdint.h>
#include "subsystem4.h"

static terrain_state_t g_current_terrain_state = {0};
static imu_filtered_data_t g_last_filtered_data = {0};

void Subsystem4_Pipeline_Tick(const imu_raw_sample_t *p_raw_sample,
                              uint32_t delta_distance_mm)
{
    if (p_raw_sample == NULL) {
        return;
    }

    /* Keep this pipeline step bounded; do not block or perform I/O here. */
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
