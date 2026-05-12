#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <Arduino.h>

// Type aliases used by inv_mpu.c
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int16_t  s16;
typedef int32_t  s32;
typedef uint64_t u64;

// MPU6050 I2C address
#define MPU_ADDR  0x68

// Register definitions (subset needed by inv_mpu.c)
#define MPU_SAMPLE_RATE_REG   0x19
#define MPU_CFG_REG           0x1A
#define GYRO_CONFIG           0x1B
#define ACCEL_CONFIG          0x1C
#define MPU_FIFO_EN_REG       0x23
#define MPU_I2CMST_STA_REG    0x36
#define MPU_INTBP_CFG_REG     0x37
#define MPU_INT_EN_REG        0x38
#define MPU_INT_STA_REG       0x3A
#define MPU_USER_CTRL_REG     0x6A
#define PWR_MGMT_1            0x6B
#define PWR_MGMT_2            0x6C
#define MPU_DEVICE_ID_REG     0x75

#ifdef __cplusplus
extern "C" {
#endif

// I2C write/read (Arduino Wire-based)
u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8 *buf);
u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf);
void mpu6050_write_reg(u8 reg, u8 dat);
u8   mpu6050_read_reg(u8 reg);

// DMP init and data retrieval
void MPU6050_IIC_IO_Init(void);
u8   MPU6050_DMP_Init(void);
u8   MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);

// Timing
void delay_ms(unsigned long ms);

#ifdef __cplusplus
}
#endif

#endif
