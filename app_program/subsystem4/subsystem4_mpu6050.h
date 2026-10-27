#ifndef SUBSYSTEM4_MPU6050_H
#define SUBSYSTEM4_MPU6050_H

#include <stdbool.h>
#include "subsystem4.h"

/* MPU6050 is wired to Robo Pico Grove 3: SDA=GP4, SCL=GP5, VCC=3V3. */
#define MPU6050_I2C_ADDRESS_PRIMARY   (0x68U)
#define MPU6050_I2C_ADDRESS_SECONDARY (0x69U)
#define MPU6050_WHO_AM_I_VALUE        (0x68U)

bool MPU6050_Init(void);
bool MPU6050_ReadRawSample(imu_raw_sample_t *p_sample);
bool MPU6050_IsReady(void);
uint8_t MPU6050_GetAddress(void);
uint32_t MPU6050_Get_I2C_Error_Stage(void);
uint32_t MPU6050_Get_Primary_I2C_Abort(void);

#endif /* SUBSYSTEM4_MPU6050_H */
