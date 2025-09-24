#pragma once

#include <cstdint>
#include <array>
#include "hardware/i2c.h"


class MPU9250 {
public:
    bool init();

    void read_accel_gyro_raw();
    void read_mag_raw();
    void convert_accel_raw();
    void convert_gyro_raw();
    void convert_mag_raw();
    void convert_temp_raw();
    void calibrate();
    void calibrate_mag();

    const std::array<float, 3>& get_accel() const { return accel; }
    const std::array<float, 3>& get_gyro()  const { return gyro; }
    const std::array<float, 3>& get_mag()   const { return mag; }
    float get_temp() const { return temperature; }


private:
    bool reset();
    bool configure();
    bool configure_accel_gyro();
    bool configure_mag();
    void push_gyro_biases();
    void write_register(uint8_t reg, uint8_t value);

    std::array<float, 3> mag_adjust     = {0.0f, 0.0f, 0.0f};

    std::array<float, 3> accel_bias     = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_bias      = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_bias_raw  = {0, 0, 0};
    std::array<int32_t, 3> mag_bias_raw = {0, 0, 0};
    std::array<float, 3> mag_scale      = {0.0f, 0.0f, 0.0f};


    std::array<int16_t, 3> accel_raw    = {0, 0, 0};
    std::array<int16_t, 3> gyro_raw     = {0, 0, 0};
    std::array<int16_t, 3> mag_raw      = {0, 0, 0};
    int16_t temp_raw = 0;

    std::array<float, 3> accel          = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro           = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> mag            = {0.0f, 0.0f, 0.0f};
    float temperature = 0;

    
    // I2C configuration
    static constexpr uint8_t MPU9250_ADDR           = 0x68;
    static constexpr uint8_t AK8963_ADDR            = 0x0C;
    static constexpr i2c_inst_t* I2C_PORT           = i2c0;
    static constexpr uint SDA_PIN                   = 4;
    static constexpr uint SCL_PIN                   = 5;

    static constexpr uint8_t SMPLRT_DIV             = 0x19;
    static constexpr uint8_t INT_PIN_CFG            = 0x37;
    static constexpr uint8_t WHO_AM_I               = 0x75; // MPU9250 should return 0x71

    // Accel + gyro registers
    static constexpr uint8_t PWR_MGMT_1             = 0x6B;
    static constexpr uint8_t ACCEL_XOUT_H           = 0x3B;
    static constexpr uint8_t ACCEL_XOUT_L           = 0x3C;
    static constexpr uint8_t ACCEL_YOUT_H           = 0x3D;
    static constexpr uint8_t ACCEL_YOUT_L           = 0x3E;
    static constexpr uint8_t ACCEL_ZOUT_H           = 0x3F;
    static constexpr uint8_t ACCEL_ZOUT_L           = 0x40;
    static constexpr uint8_t ACCEL_CONFIG           = 0x1C;

    // Gyro registers
    static constexpr uint8_t GYRO_XOUT_H            = 0x43;
    static constexpr uint8_t GYRO_XOUT_L            = 0x44;
    static constexpr uint8_t GYRO_YOUT_H            = 0x45;
    static constexpr uint8_t GYRO_YOUT_L            = 0x46;
    static constexpr uint8_t GYRO_ZOUT_H            = 0x47;
    static constexpr uint8_t GYRO_ZOUT_L            = 0x48;
    static constexpr uint8_t GYRO_CONFIG            = 0x1B;
    static constexpr uint8_t XG_OFFSET_H            = 0x13;
    static constexpr uint8_t XG_OFFSET_L            = 0x14;
    static constexpr uint8_t YG_OFFSET_H            = 0x15;
    static constexpr uint8_t YG_OFFSET_L            = 0x16;
    static constexpr uint8_t ZG_OFFSET_H            = 0x17;
    static constexpr uint8_t ZG_OFFSET_L            = 0x18;
    
    // Magnetometer registers
    static constexpr uint8_t AK8963_WHO_AM_I        = 0x00;
    static constexpr uint8_t AK8963_ST1             = 0x02;
    static constexpr uint8_t AK8963_XOUT_L          = 0x03;
    static constexpr uint8_t AK8963_XOUT_H          = 0x04;
    static constexpr uint8_t AK8963_YOUT_L          = 0x05;
    static constexpr uint8_t AK8963_YOUT_H          = 0x06;
    static constexpr uint8_t AK8963_ZOUT_L          = 0x07;
    static constexpr uint8_t AK8963_ZOUT_H          = 0x08;
    static constexpr uint8_t AK8963_ST2             = 0x09;
    static constexpr uint8_t AK8963_CNTL1           = 0x0A;
    static constexpr uint8_t AK8963_CNTL2           = 0x0B;
    static constexpr uint8_t AK8963_ASAX            = 0x10;
    static constexpr uint8_t AK8963_ASAY            = 0x11;
    static constexpr uint8_t AK8963_ASAZ            = 0x12;
    


    // Accelerometer scale factors
    static constexpr float ACCEL_SCALE_FACTOR_2G        = 16384.0f;
    static constexpr float ACCEL_SCALE_FACTOR_4G        = 8192.0f;
    static constexpr float ACCEL_SCALE_FACTOR_8G        = 4096.0f;
    static constexpr float ACCEL_SCALE_FACTOR_16G       = 2048.0f;

    
    // Gyro scale factors
    static constexpr float GYRO_SCALE_FACTOR_250DPS     = 131.0f; 
    static constexpr float GYRO_SCALE_FACTOR_500DPS     = 65.5f; 
    static constexpr float GYRO_SCALE_FACTOR_1000DPS    = 32.8f;
    static constexpr float GYRO_SCALE_FACTOR_2000DPS    = 16.4f;


    // Default configuration:
    // +/-2g, +/-250 deg/sec
    static constexpr float ACCEL_SCALE_FACTOR       = ACCEL_SCALE_FACTOR_2G; 
    static constexpr float GYRO_SCALE_FACTOR        = GYRO_SCALE_FACTOR_250DPS;

    static constexpr uint8_t ACCEL_CONFIG_VALUE     = 0x00; // +/-2g
    static constexpr uint8_t GYRO_CONFIG_VALUE      = 0x00; // +/-250 degrees/sec
    static constexpr uint8_t SAMPLE_RATE_DIV        = 1; // sample Rate = gyroscope output rate / (1 + SAMPLE_RATE_DIV)
};
