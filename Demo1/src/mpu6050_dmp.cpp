#include "mpu6050_dmp.h"
#include <Wire.h>

// ===== I2C via Wire =====
extern "C" u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8 *buf) {
    Wire.beginTransmission(addr);  // 8-bit to 7-bit address
    Wire.write(reg);
    for (u8 i = 0; i < len; i++) {
        Wire.write(buf[i]);
    }
    u8 result = Wire.endTransmission();
    return (result == 0) ? 0 : 1;
}

extern "C" u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    u8 result = Wire.endTransmission(false);  // repeated start
    if (result != 0) return 1;

    Wire.requestFrom(addr, len);
    for (u8 i = 0; i < len; i++) {
        if (Wire.available()) {
            buf[i] = Wire.read();
        } else {
            return 1;
        }
    }
    return 0;
}

extern "C" void mpu6050_write_reg(u8 reg, u8 dat) {
    mpu6050_write(MPU_ADDR, reg, 1, &dat);
}

extern "C" u8 mpu6050_read_reg(u8 reg) {
    u8 dat;
    mpu6050_read(MPU_ADDR, reg, 1, &dat);
    return dat;
}

// ===== Timing adapter =====

// ===== Timing adapter =====
extern "C" void delay_ms(unsigned long ms) {
    delay(ms);
}

// ===== I2C IO init (no-op: Wire.begin() handles this) =====
extern "C" void MPU6050_IIC_IO_Init(void) {
    // Wire.begin() already called in setup()
}
