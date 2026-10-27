/**
 * @file subsystem4_MPU6050.h
 * @brief RP2040 I2C0 driver interface for the GY-511 (LSM303DLHC).
 *
 * The GY-511 contains an accelerometer and a magnetometer. It does not
 * contain a gyroscope or a hardware FIFO.
 */
#ifndef SUBSYSTEM4_MPU6050_H
#define SUBSYSTEM4_MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "subsystem4.h"

#define MPU6050_ACCEL_I2C_ADDRESS       (0x19U)
#define MPU6050_MAG_I2C_ADDRESS         (0x1EU)
#define MPU6050_ACCEL_WHO_AM_I_VALUE    (0x33U)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    uint32_t timestamp_ms;
} MPU6050_raw_sample_t;

bool MPU6050_Init(void);
bool MPU6050_ReadRawSample(MPU6050_raw_sample_t *p_sample);
bool MPU6050_IsReady(void);
uint8_t MPU6050_Get_Accel_WhoAmI(void);
uint32_t MPU6050_Get_Error_Stage(void);

#endif /* SUBSYSTEM4_MPU6050_H */
