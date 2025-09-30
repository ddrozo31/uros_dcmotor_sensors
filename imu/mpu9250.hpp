// imu/mpu9250.hpp

#ifndef MPU9250_HPP
#define MPU9250_HPP

#include "hardware/i2c.h"

class MPU9250 {
public:
    MPU9250(i2c_inst_t* i2c, uint sda_pin = 12, uint scl_pin = 13, uint8_t address = 0x68);

    bool init();
    bool read_accel_gyro();

    float ax, ay, az;  // m/s² or g (depends on config)
    float gx, gy, gz;  // deg/s

private:
    i2c_inst_t* i2c_inst;
    uint8_t dev_addr;

    bool write_byte(uint8_t reg, uint8_t val);
    bool read_bytes(uint8_t reg, uint8_t* buf, uint8_t len);

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
};

#endif // MPU9250_HPP
