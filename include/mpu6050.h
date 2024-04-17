#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <cmath>

#define MPU6050_I2C_ADDR        0x68  // MPU6050 I2C address
#define I2C_MASTER_NUM          I2C_NUM_0

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1  0x6B  // Power management register 1
#define MPU6050_REG_ACCEL_CONFIG 0x1C // Accelerometer configuration
#define MPU6050_REG_GYRO_CONFIG 0x1B // Gyroscope configuration
#define MPU6050_REG_ACCEL_XOUT_H 0x3B // Accelerometer readings
#define MPU6050_REG_GYRO_XOUT_H 0x43 // Gyroscope readings

// constants
#define G 9.81
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define PI 3.1415926535897932384626433832795



typedef enum{
    AFS_SEL_2G = 0,
    AFS_SEL_4G,
    AFS_SEL_8G,
    AFS_SEL_16G
} accel_fs_t;

typedef enum{
    GFS_SEL_250 = 0,
    GFS_SEL_500,
    GFS_SEL_1000,
    GFS_SEL_2000
} gyro_fs_t;



typedef struct {
    double x;
    double y;
    double z;
} vec3_t;



class MPU6050 {
private:
    i2c_port_t i2c_port;
    uint8_t _accel_config;
    uint8_t _gyro_config;
    

public:
    MPU6050(i2c_port_t port);
    MPU6050(int port);
    ~MPU6050();
    bool init();
    bool init(bool);

    bool writeRegister(uint8_t *data, size_t len);
    bool readRegister(uint8_t reg, uint8_t *data, size_t len);

    bool setAccelConfig(accel_fs_t afs_sel);
    bool setAccelConfig(int afs_sel);

    bool setGyroConfig(gyro_fs_t gfs_sel);
    bool setGyroConfig(int gfs_sel);

    vec3_t readAccel();
    vec3_t readRawAccel();

    vec3_t readGyro();
    vec3_t readRawGyro();
};
