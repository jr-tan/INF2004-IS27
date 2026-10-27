/**
 * @file subsystem4_mpu6050.c
 * @brief Small RP2040 I2C0/MPU6050 driver for the μT-Kernel template.
 *
 * Uses RP2040 MMIO rather than Pico SDK headers because this project is built
 * by the μT-Kernel makefile. I2C0 is GP4 SDA / GP5 SCL. The board/system clock
 * assumption is 125 MHz; override S4_SYS_CLOCK_HZ if the BSP uses another rate.
 */
#include <stdbool.h>
#include <stdint.h>
#include "subsystem4_mpu6050.h"
#include <tk/tkernel.h>

#ifndef S4_SYS_CLOCK_HZ
#define S4_SYS_CLOCK_HZ 125000000UL
#endif

#define RP2040_RESETS_BASE       (0x4000C000UL)
#define RP2040_IO_BANK0_BASE     (0x40014000UL)
#define RP2040_PADS_BANK0_BASE   (0x4001C000UL)
#define RP2040_I2C0_BASE         (0x40044000UL)

#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define I2C_REG(offset) REG32(RP2040_I2C0_BASE + (offset))

#define GPIO_PAD(gpio)           REG32(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(gpio) * 4UL))
#define GPIO_FUNC_I2C            (3UL)
#define GPIO_PAD_IE              (1UL << 6)
#define GPIO_PAD_PUE             (1UL << 3)
#define GPIO_PAD_PDE             (1UL << 2)

#define I2C_IC_CON                (0x00UL)
#define I2C_IC_TAR                (0x04UL)
#define I2C_IC_DATA_CMD           (0x10UL)
#define I2C_IC_SS_SCL_HCNT        (0x14UL)
#define I2C_IC_SS_SCL_LCNT        (0x18UL)
#define I2C_IC_INTR_STAT          (0x2CUL)
#define I2C_IC_RAW_INTR_STAT      (0x34UL)
#define I2C_IC_RX_TL              (0x38UL)
#define I2C_IC_TX_TL              (0x3CUL)
#define I2C_IC_CLR_TX_ABRT        (0x54UL)
#define I2C_IC_ENABLE             (0x6CUL)
#define I2C_IC_STATUS             (0x70UL)
#define I2C_IC_TXFLR              (0x74UL)
#define I2C_IC_RXFLR              (0x78UL)
#define I2C_IC_TX_ABRT_SOURCE     (0x80UL)

#define I2C_CON_MASTER            (1UL << 0)
#define I2C_CON_SPEED_STANDARD    (1UL << 1)
#define I2C_CON_RESTART_EN        (1UL << 5)
#define I2C_CON_SLAVE_DISABLE     (1UL << 6)
#define I2C_CMD_READ              (1UL << 8)
#define I2C_CMD_STOP              (1UL << 9)
#define I2C_INTR_TX_ABRT          (1UL << 6)
#define I2C_STATUS_TFNF           (1UL << 1)
#define I2C_STATUS_TFE            (1UL << 2)
#define I2C_STATUS_RFNE           (1UL << 3)
#define I2C_STATUS_ACTIVITY       (1UL << 0)
#define I2C_STATUS_MST_ACTIVITY   (1UL << 5)
#define I2C_WAIT_LIMIT            (200000UL)

#define MPU_REG_SMPLRT_DIV        (0x19U)
#define MPU_REG_CONFIG            (0x1AU)
#define MPU_REG_GYRO_CONFIG       (0x1BU)
#define MPU_REG_ACCEL_CONFIG      (0x1CU)
#define MPU_REG_ACCEL_XOUT_H      (0x3BU)
#define MPU_REG_PWR_MGMT_1        (0x6BU)
#define MPU_REG_WHO_AM_I          (0x75U)

static bool g_i2c_initialized = false;
static bool g_mpu_ready = false;
static uint8_t g_mpu_address = MPU6050_I2C_ADDRESS_PRIMARY;
static uint32_t g_sample_timestamp_ms = 0U;
static volatile uint32_t g_i2c_last_abort_source = 0U;
static volatile uint32_t g_i2c_error_stage = 0U;
static volatile uint32_t g_i2c_primary_abort = 0U;


uint32_t MPU6050_Get_Primary_I2C_Abort(void)
{
    return g_i2c_primary_abort;
}

//Debugging for I2C Logging
uint32_t MPU6050_Get_Last_I2C_Abort(void)
{
    return g_i2c_primary_abort;
}

static bool i2c_wait_tx_ready(void)
{
    uint32_t wait;
    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
            g_i2c_last_abort_source = I2C_REG(I2C_IC_TX_ABRT_SOURCE);
            (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
            return false;
        }
        if ((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_TFNF) != 0U) {
            return true;
        }
    }
    return false;
}

static bool i2c_wait_complete(void)
{
    uint32_t wait;
    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
            g_i2c_last_abort_source = I2C_REG(I2C_IC_TX_ABRT_SOURCE);
            (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
            return false;
        }
        if (((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_TFE) != 0U) &&
            ((I2C_REG(I2C_IC_STATUS) & (I2C_STATUS_ACTIVITY | I2C_STATUS_MST_ACTIVITY)) == 0U)) {
            return true;
        }
    }
    return false;
}

static bool i2c0_init(void)
{
    uint32_t wait;
    uint32_t high_count;
    uint32_t low_count;

    /* Release I2C0 from reset and wait for the reset controller. */
    REG32(RESETS_RESET) = REG32(RESETS_RESET) & ~RESETS_RESET_I2C0;
    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((REG32(RESETS_RESET_DONE) & RESETS_RESET_I2C0) != 0U)
            break;
    }
    if (wait >= I2C_WAIT_LIMIT) {
        return false;
    }

    /* Select I2C function on GP4/GP5 and enable input/pull-ups. */
    GPIO_PAD(4U) = (GPIO_PAD(4U) | GPIO_PAD_IE | GPIO_PAD_PUE) & ~GPIO_PAD_PDE;
    GPIO_PAD(5U) = (GPIO_PAD(5U) | GPIO_PAD_IE | GPIO_PAD_PUE) & ~GPIO_PAD_PDE;
    REG32(GPIO_CTRL(4U)) = (REG32(GPIO_CTRL(4U)) & ~0x1FUL) | GPIO_FUNC_I2C;
    REG32(GPIO_CTRL(5U)) = (REG32(GPIO_CTRL(5U)) & ~0x1FUL) | GPIO_FUNC_I2C;

    I2C_REG(I2C_IC_ENABLE) = 0U;
    I2C_REG(I2C_IC_CON) = I2C_CON_MASTER | I2C_CON_SPEED_STANDARD |
                          I2C_CON_RESTART_EN | I2C_CON_SLAVE_DISABLE;

    /* Standard-mode I2C, approximately 100 kHz at 125 MHz system clock. */
    high_count = (uint32_t)((S4_SYS_CLOCK_HZ / 1000000UL) * 4UL);
    low_count = (uint32_t)(((S4_SYS_CLOCK_HZ / 1000000UL) * 47UL) / 10UL);
    if (high_count > 8U) { high_count -= 8U; }
    if (low_count > 1U) { low_count -= 1U; }
    I2C_REG(I2C_IC_SS_SCL_HCNT) = high_count;
    I2C_REG(I2C_IC_SS_SCL_LCNT) = low_count;
    I2C_REG(I2C_IC_RX_TL) = 0U;
    I2C_REG(I2C_IC_TX_TL) = 0U;
    I2C_REG(I2C_IC_ENABLE) = 1U;
    g_i2c_initialized = true;
    return true;
}

static bool i2c0_set_target(uint8_t address)
{
    I2C_REG(I2C_IC_ENABLE) = 0U;
    I2C_REG(I2C_IC_TAR) = (uint32_t)(address & 0x7FU);
    I2C_REG(I2C_IC_ENABLE) = 1U;
    g_mpu_address = address;
    return true;
}

static bool i2c0_write_reg(uint8_t reg, uint8_t value)
{
    if (!i2c_wait_tx_ready()) { return false; }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)reg;
    if (!i2c_wait_tx_ready()) { return false; }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)value | I2C_CMD_STOP;
    return i2c_wait_complete();
}

static bool i2c0_read_regs(uint8_t reg, uint8_t *p_data, uint32_t length)
{
    uint32_t index;
    uint32_t wait;
    if ((p_data == (uint8_t *)0) || (length == 0U) || (length > 16U)) {
        return false;
    }
    if (!i2c_wait_tx_ready()) { return false; }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)reg;

    for (index = 0U; index < length; ++index) {
        if (!i2c_wait_tx_ready()) { return false; }
        I2C_REG(I2C_IC_DATA_CMD) = I2C_CMD_READ |
            ((index == (length - 1U)) ? I2C_CMD_STOP : 0U);
    }

    for (index = 0U; index < length; ++index) {
        for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
            if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
                g_i2c_last_abort_source = I2C_REG(I2C_IC_TX_ABRT_SOURCE);
                (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
                return false;
            }
            if ((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_RFNE) != 0U) {
                break;
            }
        }
        if (wait >= I2C_WAIT_LIMIT) { return false; }
        p_data[index] = (uint8_t)(I2C_REG(I2C_IC_DATA_CMD) & 0xFFU);
    }
    return i2c_wait_complete();
}

static int16_t signed_be16(const uint8_t high, const uint8_t low)
{
    return (int16_t)((((uint16_t)high) << 8) | (uint16_t)low);
}


bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;
    uint32_t primary_abort = 0U;

    g_i2c_error_stage = 0U;
    g_i2c_last_abort_source = 0U;
    g_mpu_ready = false;

    /* Stage 1: initialize I2C0. */
    if (!g_i2c_initialized) {
        if (!i2c0_init()) {
            g_i2c_error_stage = 1U;
            return false;
        }
    }

    /* Stage 2 or 3: probe the primary address, 0x68. */
    if (!i2c0_set_target(MPU6050_I2C_ADDRESS_PRIMARY)) {
        g_i2c_error_stage = 2U;
        return false;
    }

    if (!i2c0_read_regs(MPU_REG_WHO_AM_I, &who_am_i, 1U)) {
        primary_abort = g_i2c_last_abort_source;
        g_i2c_error_stage = 2U;
    } else if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        g_i2c_error_stage = 3U;
    } else {
        g_i2c_error_stage = 0U;
    }

    /* If primary probing failed, try the secondary address, 0x69. */
    if (g_i2c_error_stage != 0U) {
        who_am_i = 0U;

        if (!i2c0_set_target(MPU6050_I2C_ADDRESS_SECONDARY)) {
            g_i2c_error_stage = 4U;
            return false;
        }

        if (!i2c0_read_regs(MPU_REG_WHO_AM_I, &who_am_i, 1U)) {
            g_i2c_error_stage = 4U;
            return false;
        }

        if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
            g_i2c_error_stage = 5U;
            return false;
        }
    }

    /* Stage 6: configure the sensor. */
    if (!i2c0_write_reg(MPU_REG_PWR_MGMT_1, 0x00U) ||
        !i2c0_write_reg(MPU_REG_CONFIG, 0x03U) ||
        !i2c0_write_reg(MPU_REG_SMPLRT_DIV, 0x09U) ||
        !i2c0_write_reg(MPU_REG_GYRO_CONFIG, 0x00U) ||
        !i2c0_write_reg(MPU_REG_ACCEL_CONFIG, 0x00U)) {
        g_i2c_error_stage = 6U;
        return false;
    }

    g_i2c_error_stage = 0U;
    g_mpu_ready = true;
    return true;
}


bool MPU6050_ReadRawSample(imu_raw_sample_t *p_sample)
{
    uint8_t bytes[14];

    if ((p_sample == (imu_raw_sample_t *)0) || !g_mpu_ready) {
        return false;
    }

    if (!i2c0_read_regs(MPU_REG_ACCEL_XOUT_H, bytes, 14U)) {
        return false;
    }

    p_sample->accel_x = signed_be16(bytes[0], bytes[1]);
    p_sample->accel_y = signed_be16(bytes[2], bytes[3]);
    p_sample->accel_z = signed_be16(bytes[4], bytes[5]);

    /* Bytes 6 and 7 contain temperature data. */
    p_sample->gyro_x = signed_be16(bytes[8], bytes[9]);
    p_sample->gyro_y = signed_be16(bytes[10], bytes[11]);
    p_sample->gyro_z = signed_be16(bytes[12], bytes[13]);

    g_sample_timestamp_ms += 10U;
    p_sample->timestamp_ms = g_sample_timestamp_ms;

    return true;
}



bool MPU6050_IsReady(void)
{
    return g_mpu_ready;
}

uint8_t MPU6050_GetAddress(void)
{
    return g_mpu_address;
}

uint32_t MPU6050_Get_I2C_Error_Stage(void)
{
    return g_i2c_error_stage;
}