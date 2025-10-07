#include "bmi160.hpp"
#include "pico/stdlib.h"

// BMI160 register definitions
#define BMI160_CMD_REG       0x7E
#define BMI160_ACC_CONF      0x40
#define BMI160_ACC_RANGE     0x41
#define BMI160_GYR_CONF      0x42
#define BMI160_GYR_RANGE     0x43
#define BMI160_CHIP_ID_REG   0x00
#define BMI160_DATA_START    0x12
#define BMI160_EXPECTED_ID   0xD1

BMI160::BMI160(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t address)
    : i2c_inst(i2c), dev_addr(address), ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {

    i2c_init(i2c, 400 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    bool init_success = init();
}

bool BMI160::init() {
    uint8_t id;
    if (!read_bytes(BMI160_CHIP_ID_REG, &id, 1) || id != BMI160_EXPECTED_ID) {
        return false;
    }

    // Soft reset
    write_byte(BMI160_CMD_REG, 0xB6);
    sleep_ms(100);

    // Accelerometer: normal mode
    write_byte(BMI160_CMD_REG, 0x11);
    sleep_ms(100);

    // Gyroscope: normal mode
    write_byte(BMI160_CMD_REG, 0x15);
    sleep_ms(100);

    // Configure ranges
    write_byte(BMI160_ACC_RANGE, 0x03); // ±2g
    write_byte(BMI160_GYR_RANGE, 0x00); // ±250°/s

    return true;
}

bool BMI160::read_accel_gyro() {
    uint8_t buf[12];
    if (!read_bytes(BMI160_DATA_START, buf, 12)) return false;

    for (int i = 0; i < 3; ++i) {
        accel_raw[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]);
        gyro_raw[i]  = (int16_t)((buf[6 + i * 2 + 1] << 8) | buf[6 + i * 2]);
    }

    ax = accel_raw[0] / ACCEL_SCALE;
    ay = accel_raw[1] / ACCEL_SCALE;
    az = accel_raw[2] / ACCEL_SCALE;

    gx = gyro_raw[0] / GYRO_SCALE;
    gy = gyro_raw[1] / GYRO_SCALE;
    gz = gyro_raw[2] / GYRO_SCALE;

    return true;
}

bool BMI160::write_byte(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(i2c_inst, dev_addr, buf, 2, false) == 2;
}

bool BMI160::read_bytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (i2c_write_blocking(i2c_inst, dev_addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c_inst, dev_addr, buf, len, false) == len;
}
