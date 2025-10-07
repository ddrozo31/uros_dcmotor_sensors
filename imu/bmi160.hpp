#pragma once
#include "hardware/i2c.h"
#include <array>

class BMI160 {
public:
    BMI160(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t address = 0x68);

    bool init();
    bool read_accel_gyro();

    float ax, ay, az;
    float gx, gy, gz;

private:
    i2c_inst_t* i2c_inst;
    uint8_t dev_addr;

    std::array<int16_t, 3> accel_raw{};
    std::array<int16_t, 3> gyro_raw{};

    bool write_byte(uint8_t reg, uint8_t val);
    bool read_bytes(uint8_t reg, uint8_t* buf, uint8_t len);

    static constexpr float ACCEL_SCALE = 16384.0f; // ±2g
    static constexpr float GYRO_SCALE  = 131.0f;   // ±250°/s
};
