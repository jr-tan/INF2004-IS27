/**
 * @file subsystem4_gy511.h
 * @brief RP2040 I2C0 driver interface for the GY-511 (LSM303D / LSM303DLHC)
 *        with TA_INHERIT mutex protection (Tribunal 1 i2c_mtx).
 */
#ifndef SUBSYSTEM4_GY511_H
#define SUBSYSTEM4_GY511_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4.h"

#define GY511_ACCEL_I2C_ADDRESS       (0x1DU)  /* Primary LSM303D address; fallback 0x19 supported */
#define GY511_ACCEL_ALT_I2C_ADDRESS   (0x19U)  /* LSM303DLHC accelerometer address */
#define GY511_MAG_I2C_ADDRESS         (0x1EU)
#define GY511_ACCEL_WHO_AM_I_VALUE    (0x49U)  /* LSM303D WHO_AM_I */
#define GY511_DLHC_WHO_AM_I_VALUE     (0x33U)  /* LSM303DLHC WHO_AM_I */

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
ID GY511_Get_I2C_Mutex_ID(void);

#endif /* SUBSYSTEM4_GY511_H */