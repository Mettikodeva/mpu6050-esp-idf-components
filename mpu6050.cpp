#include "mpu6050.h"

MPU6050::MPU6050(i2c_port_t port){
    i2c_port = port;
}
MPU6050::MPU6050(int port) : MPU6050((i2c_port_t)port) {}

bool MPU6050::init() {
    return init(true);
}

bool MPU6050::init(bool init_i2c) {
    if(init_i2c){
        // Initialize I2C bus
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = 21;    // Adjust these pins as per your setup
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = 22;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 400000;
        i2c_param_config(i2c_port, &conf);
        i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    }

    // Initialize MPU6050
    uint8_t data[2] = {0x6B, 0x00}; // Power management register 1, reset value
    if (writeRegister(data, 2)) {
        return true;
    } else {
        return false;
    }
}

bool MPU6050::writeRegister(uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

bool MPU6050::readRegister(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        return true;
    } else {
        return false;
    }
}



bool MPU6050::setAccelConfig(int afs_sel){
    return this->setAccelConfig((accel_fs_t)afs_sel);
}

bool MPU6050::setAccelConfig(accel_fs_t afs_sel)
{
    uint8_t acc_conf[4] = {0x00, 0x08, 0x10, 0x18};
    uint8_t data[2] = {MPU6050_REG_ACCEL_CONFIG, acc_conf[(int)afs_sel]};
    _accel_config = (uint8_t)afs_sel;
    if (writeRegister(data, 2)) {
        return true;
    } else {
        return false;
    }
}

bool MPU6050::setGyroConfig(int gfs_sel) {
    return this->setGyroConfig((gyro_fs_t)gfs_sel);
}

bool MPU6050::setGyroConfig(gyro_fs_t gfs_sel) {
    uint8_t gyro_conf[4] = {0x00, 0x08, 0x10, 0x18};
    uint8_t data[2] = {MPU6050_REG_GYRO_CONFIG, gyro_conf[(int)gfs_sel]};
    _gyro_config = (uint8_t)gfs_sel ;
    if (writeRegister(data, 2)) {
        return true;
    } else {
        return false;
    }
}

vec3_t MPU6050::readAccel() {
    float SF[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    uint8_t data[6];
    if (readRegister(MPU6050_REG_ACCEL_XOUT_H, data, 6)) {
        vec3_t accel;
        accel.x = (data[0] << 8) | data[1];
        accel.y = (data[2] << 8) | data[3];
        accel.z = (data[4] << 8) | data[5];
        accel.x = accel.x / SF[_accel_config] * G;
        accel.y = accel.y / SF[_accel_config] * G;
        accel.z = accel.z / SF[_accel_config] * G;

        return accel;

    } else {
        ESP_LOGE("MPU6050", "Failed to read accel data");
        return vec3_t(0, 0, 0);
    }
}

vec3_t MPU6050::readRawAccel() {
    float SF[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    uint8_t data[6];
    if (readRegister(MPU6050_REG_ACCEL_XOUT_H, data, 6)) {
        vec3_t accel;
        accel.x = (data[0] << 8) | data[1];
        accel.y = (data[2] << 8) | data[3];
        accel.z = (data[4] << 8) | data[5];
        return accel;

    } else {
        ESP_LOGE("MPU6050", "Failed to read accel data");
        return vec3_t(0, 0, 0);
    }
}

vec3_t MPU6050::readGyro() {
    float SF[4] = {131.0, 65.5, 32.8, 16.4};
    uint8_t data[6];
    if (readRegister(MPU6050_REG_GYRO_XOUT_H, data, 6)) {
        vec3_t gyro;
        gyro.x = (data[0] << 8) | data[1];
        gyro.y = (data[2] << 8) | data[3];
        gyro.z = (data[4] << 8) | data[5];
        gyro.x = gyro.x / SF[_gyro_config];
        gyro.y = gyro.y / SF[_gyro_config];
        gyro.z = gyro.z / SF[_gyro_config];
        return gyro;
    } else {
        vec3_t gyro;
        gyro.x = 0;
        gyro.y = 0;
        gyro.z = 0;
        return gyro;
    }
}

vec3_t MPU6050::readRawGyro() {
    float SF[4] = {131.0, 65.5, 32.8, 16.4};
    uint8_t data[6];
    if (readRegister(MPU6050_REG_GYRO_XOUT_H, data, 6)) {
        vec3_t gyro;
        gyro.x = (data[0] << 8) | data[1];
        gyro.y = (data[2] << 8) | data[3];
        gyro.z = (data[4] << 8) | data[5];
        
        return gyro;
    } else {
        vec3_t gyro;
        gyro.x = 0;
        gyro.y = 0;
        gyro.z = 0;
        return gyro;
    }
}

