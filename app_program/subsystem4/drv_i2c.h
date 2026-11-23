/* drv_i2c.h  (Buddy 4): polled I2C0 master, IMU task only */
#ifndef DRV_I2C_H
#define DRV_I2C_H

IMPORT void i2c_init(void);
IMPORT ER   i2c_write_reg(UB addr, UB reg, UB val);
IMPORT ER   i2c_read_regs(UB addr, UB reg, UB *buf, UINT len);

#endif
