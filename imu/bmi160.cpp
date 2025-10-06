#include "bmi160.hpp"
#include <cstring>

BMI160::BMI160() : mode(INVALID_MODE), i2c_port(nullptr), spi_port(nullptr) {
    ax = ay = az = 0.0f;
    gx = gy = gz = 0.0f;
}

bool BMI160::begin(Mode mode, i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint8_t i2c_addr) {
    this->mode = I2C_MODE;
    this->i2c_port = i2c_port;
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
    this->i2c_addr = i2c_addr;
    
    init_i2c();
    sleep_ms(100);
    
    return initialize_sensor();
}

bool BMI160::begin(Mode mode, spi_inst_t* spi_port, uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin) {
    this->mode = SPI_MODE;
    this->spi_port = spi_port;
    this->cs_pin = cs_pin;
    this->sck_pin = sck_pin;
    this->mosi_pin = mosi_pin;
    this->miso_pin = miso_pin;
    
    init_spi();
    sleep_ms(100);
    
    return initialize_sensor();
}

void BMI160::init_i2c() {
    // Initialize I2C
    ::i2c_init(i2c_port, 400 * 1000); // 400kHz
    
    // Set up pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

void BMI160::init_spi() {
    // Initialize SPI
    ::spi_init(spi_port, 1000 * 1000); // 1MHz
    
    // Set up pins
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    
    // Chip select pin
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
}

bool BMI160::initialize_sensor() {
    // Check chip ID
    uint8_t chip_id = getChipID();
    if (chip_id != BMI160_CHIP_ID) {
        return false;
    }
    
    // Soft reset
    if (!write_register(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET)) {
        return false;
    }
    sleep_ms(100);
    
    // Enable accelerometer
    if (!write_register(BMI160_REG_CMD, BMI160_CMD_ACCEL_NORMAL)) {
        return false;
    }
    sleep_ms(10);
    
    // Enable gyroscope
    if (!write_register(BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL)) {
        return false;
    }
    sleep_ms(100);
    
    return true;
}

uint8_t BMI160::getChipID() {
    uint8_t chip_id = 0;
    read_register(BMI160_REG_CHIP_ID, chip_id);
    return chip_id;
}

bool BMI160::isConnected() {
    return (getChipID() == BMI160_CHIP_ID);
}

bool BMI160::write_register(uint8_t reg, uint8_t value) {
    switch (mode) {
        case I2C_MODE:
            return i2c_write(reg, value);
        case SPI_MODE:
            return spi_write(reg, value);
        default:
            return false;
    }
}

bool BMI160::read_register(uint8_t reg, uint8_t& value) {
    return read_registers(reg, &value, 1);
}

bool BMI160::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    switch (mode) {
        case I2C_MODE:
            return i2c_read(reg, buffer, length);
        case SPI_MODE:
            return spi_read(reg, buffer, length);
        default:
            return false;
    }
}

bool BMI160::i2c_write(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    int result = i2c_write_blocking(i2c_port, i2c_addr, data, 2, false);
    return (result == 2);
}

bool BMI160::i2c_read(uint8_t reg, uint8_t* buffer, uint8_t length) {
    // Write register address
    int result = i2c_write_blocking(i2c_port, i2c_addr, &reg, 1, true);
    if (result != 1) {
        return false;
    }
    
    // Read data
    result = i2c_read_blocking(i2c_port, i2c_addr, buffer, length, false);
    return (result == length);
}

bool BMI160::spi_write(uint8_t reg, uint8_t value) {
    uint8_t reg_write = static_cast<uint8_t>(reg & 0x7F); // Clear read bit for write
    uint8_t data[2] = {reg_write, value};
    
    gpio_put(cs_pin, 0); // CS low (active)
    int result = spi_write_blocking(spi_port, data, 2);
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    return (result == 2);
}

bool BMI160::spi_read(uint8_t reg, uint8_t* buffer, uint8_t length) {
    uint8_t tx_data = reg | (1 << BMI160_SPI_READ_BIT); // Set read bit
    
    gpio_put(cs_pin, 0); // CS low (active)
    
    // Send register address
    spi_write_blocking(spi_port, &tx_data, 1);
    
    // Read data
    int result = spi_read_blocking(spi_port, 0, buffer, length);
    
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    return (result == length);
}

bool BMI160::readAccel(int16_t& ax, int16_t& ay, int16_t& az) {
    uint8_t data[6];
    
    if (!read_registers(BMI160_REG_ACC_X_L, data, 6)) {
        return false;
    }
    
    // Combine low and high bytes
    ax = (int16_t)((data[1] << 8) | data[0]);
    ay = (int16_t)((data[3] << 8) | data[2]);
    az = (int16_t)((data[5] << 8) | data[4]);
    
    return true;
}

bool BMI160::readGyro(int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t data[6];
    
    if (!read_registers(BMI160_REG_GYR_X_L, data, 6)) {
        return false;
    }
    
    // Combine low and high bytes
    gx = (int16_t)((data[1] << 8) | data[0]);
    gy = (int16_t)((data[3] << 8) | data[2]);
    gz = (int16_t)((data[5] << 8) | data[4]);
    
    return true;
}

bool BMI160::readAccelGyro(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    return readGyro(gx, gy, gz) && readAccel(ax, ay, az);
}

bool BMI160::read_accel_gyro() {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    
    if (!readAccelGyro(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)) {
        return false;
    }
    
    // Convert to physical units
    ax = raw_ax * ACCEL_SCALE;
    ay = raw_ay * ACCEL_SCALE;
    az = raw_az * ACCEL_SCALE;
    
    gx = raw_gx * GYRO_SCALE;
    gy = raw_gy * GYRO_SCALE;
    gz = raw_gz * GYRO_SCALE;
    
    return true;
}