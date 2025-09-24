#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

#define ACCEL_SCALE_FACTOR_2G 16384.0 // +/-2g
#define ACCEL_SCALE_FACTOR_4G 8192.0 // +/-4g
#define ACCEL_SCALE_FACTOR_8G 4096.0 // +/-8g
#define ACCEL_SCALE_FACTOR_16G 2048.0 // +/-16g

#define GYRO_SCALE_FACTOR_250DPS 131.0 // +/-250 degrees/sec
#define GYRO_SCALE_FACTOR_500DPS 65.5 // +/-500 degrees/sec
#define GYRO_SCALE_FACTOR_1000DPS 32.8 // +/-1000 degrees/sec
#define GYRO_SCALE_FACTOR_2000DPS 16.4 // +/-2000 degrees/sec

// set to desired scales
#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G 
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS

#define ACCEL_CONFIG_VALUE 0x08 // +/-4g
#define GYRO_CONFIG_VALUE 0x00 // +/-250 degrees/sec
#define SAMPLE_RATE_DIV 1 // sample Rate = gyroscope output rate / (1 + SAMPLE_RATE_DIV)

#ifdef __cplusplus
extern "C" {
#endif

extern float accel_offset[3];
extern float gyro_offset[3];

// function prototypes
void mpu6050_reset(void);
void mpu6050_configure(void);
void mpu6050_read(float accel[3], float gyro[3], float *temp);
void mpu6050_calibrate(void);

#ifdef __cplusplus
}
#endif

#endif
