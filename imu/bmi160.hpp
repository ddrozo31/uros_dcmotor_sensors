#ifndef BMI160_HPP
#define BMI160_HPP

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <cstdint>

// BMI160 Register addresses
#define BMI160_REG_CHIP_ID          0x00
#define BMI160_REG_ACC_X_L          0x12
#define BMI160_REG_ACC_X_H          0x13
#define BMI160_REG_ACC_Y_L          0x14
#define BMI160_REG_ACC_Y_H          0x15
#define BMI160_REG_ACC_Z_L          0x16
#define BMI160_REG_ACC_Z_H          0x17
#define BMI160_REG_GYR_X_L          0x0C
#define BMI160_REG_GYR_X_H          0x0D
#define BMI160_REG_GYR_Y_L          0x0E
#define BMI160_REG_GYR_Y_H          0x0F
#define BMI160_REG_GYR_Z_L          0x10
#define BMI160_REG_GYR_Z_H          0x11
#define BMI160_REG_CMD              0x7E
#define BMI160_REG_ACC_CONF         0x40
#define BMI160_REG_GYR_CONF         0x42

// BMI160 Commands
#define BMI160_CMD_ACCEL_NORMAL     0x11
#define BMI160_CMD_GYRO_NORMAL      0x15
#define BMI160_CMD_SOFT_RESET       0xB6

// BMI160 Chip ID
#define BMI160_CHIP_ID              0xD1

// SPI Read bit
#define BMI160_SPI_READ_BIT         7

// Default I2C address
#define BMI160_I2C_ADDR_DEFAULT     0x68

class BMI160 {
public:
    enum Mode {
        INVALID_MODE = -1,
        I2C_MODE = 1,
        SPI_MODE = 2
    };

    // Constructors
    BMI160();
    
    // Initialization methods
    bool begin(Mode mode, i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint8_t i2c_addr = BMI160_I2C_ADDR_DEFAULT);
    bool begin(Mode mode, spi_inst_t* spi_port, uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin);
    
    // Data reading methods
    bool readAccel(int16_t& ax, int16_t& ay, int16_t& az);
    bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz);
    bool readAccelGyro(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
    
    // Utility methods
    uint8_t getChipID();
    bool isConnected();
    
    // Public data members for direct access (like the original library)
    float ax, ay, az;  // Accelerometer data in m/s²
    float gx, gy, gz;  // Gyroscope data in rad/s
    
    // Method to update all data and store in public members
    bool read_accel_gyro();

private:
    Mode mode;
    
    // I2C configuration
    i2c_inst_t* i2c_port;
    uint8_t i2c_addr;
    uint sda_pin, scl_pin;
    
    // SPI configuration
    spi_inst_t* spi_port;
    uint cs_pin, sck_pin, mosi_pin, miso_pin;
    
    // Communication methods
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // I2C specific methods
    void init_i2c();
    bool i2c_write(uint8_t reg, uint8_t value);
    bool i2c_read(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // SPI specific methods
    void init_spi();
    bool spi_write(uint8_t reg, uint8_t value);
    bool spi_read(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Initialization helper
    bool initialize_sensor();
    
    // Scale factors for converting raw data
    static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // ±2g range
    static constexpr float GYRO_SCALE = 3.14159f / (180.0f * 16.4f);  // ±2000°/s range
};

#endif // BMI160_HPP