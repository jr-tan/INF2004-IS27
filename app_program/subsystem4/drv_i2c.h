/**
 * @file    drv_i2c.h
 * drv_i2c.h  (Buddy 4): polled I2C0 master, IMU task only */
#ifndef DRV_I2C_H
#define DRV_I2C_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
IMPORT void drv_i2c_init (void);
IMPORT ER   drv_i2c_write_reg (uint8_t addr, uint8_t reg, uint8_t val);
IMPORT ER   drv_i2c_read_regs (uint8_t addr, uint8_t reg, uint8_t * p_buf,
                               uint32_t len);

#endif

/*** end of file ***/
