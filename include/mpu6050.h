#pragma once
#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

#define MPU6050_I2C_ADDR        0x68  // MPU6050 I2C address
#define I2C_MASTER_NUM          I2C_NUM_0

class MPU6050 {
private:
    i2c_port_t i2c_port;

public:
    MPU6050(i2c_port_t port);
    bool init();
    bool init(bool);

    bool writeRegister(uint8_t *data, size_t len);
    bool readRegister(uint8_t reg, uint8_t *data, size_t len);
};
#endif