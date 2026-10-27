/**
 * @file subsystem4_gy511.h
 * @brief RP2040 I2C0 driver interface for the GY-511 (LSM303DLHC).
 *
 * The GY-511 contains an accelerometer and a magnetometer. It does not
 * contain a gyroscope or a hardware FIFO.
 */
#ifndef SUBSYSTEM4_GY511_H
#define SUBSYSTEM4_GY511_H

#include <stdbool.h>
#include <stdint.h>
#include "subsystem4.h"

#define GY511_ACCEL_I2C_ADDRESS       (0x1DU)  /* Test 0x1D if 0x19 continues to NACK */
#define GY511_MAG_I2C_ADDRESS         (0x1EU)
#define GY511_ACCEL_WHO_AM_I_VALUE    (0x49U)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    uint32_t timestamp_ms;
} gy511_raw_sample_t;

bool GY511_Init(void);
bool GY511_ReadRawSample(gy511_raw_sample_t *p_sample);
bool GY511_IsReady(void);
uint8_t GY511_Get_Accel_WhoAmI(void);
uint32_t GY511_Get_Error_Stage(void);

#endif /* SUBSYSTEM4_GY511_H */
