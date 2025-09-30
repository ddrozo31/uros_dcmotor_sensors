// imu/mpu9250.cpp

#include "mpu9250.hpp"
#include "pico/stdlib.h"

#define MPU_PWR_MGMT_1   0x6B
#define MPU_ACCEL_XOUT_H 0x3B

MPU9250::MPU9250(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t address)
    : i2c_inst(i2c), dev_addr(address), ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {

    i2c_init(i2c, 400 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C); // SDA
    gpio_set_function(scl_pin, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    }

bool MPU9250::init() {
    // Wake up the MPU9250
    return write_byte(MPU_PWR_MGMT_1, 0x00);
}

bool MPU9250::read_accel_gyro() {
    uint8_t buf[14];
    if (!read_bytes(MPU_ACCEL_XOUT_H, buf, 14)) return false;

    // Convert raw bytes to int16_t
    for (int i = 0; i < 3; i++) {
        accel_raw[i] = (int16_t)((buf[i * 2] << 8) | buf[i * 2 + 1]);
        gyro_raw[i]  = (int16_t)((buf[8 + i * 2] << 8) | buf[8 + i * 2 + 1]);
    }

    // Scale: defaults to ±2g and ±250 dps
    ax = accel_raw[0] / 16384.0f;
    ay = accel_raw[1] / 16384.0f;
    az = accel_raw[2] / 16384.0f;

    gx = gyro_raw[0] / 131.0f;
    gy = gyro_raw[1] / 131.0f;
    gz = gyro_raw[2] / 131.0f;

    return true;
}

bool MPU9250::write_byte(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(i2c_inst, dev_addr, buf, 2, false) == 2;
}

bool MPU9250::read_bytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (i2c_write_blocking(i2c_inst, dev_addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c_inst, dev_addr, buf, len, false) == len;
}
