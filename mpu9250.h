#ifndef MPU9250_H
#define MPU9250_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define MPU9250_ADDR        0x68
#define AK8963_ADDR         0x0C

#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_SMPLRT_DIV      0x19
#define REG_INT_PIN_CFG     0x37
#define WHO_AM_I_REG        0x75 // mpu9250 should return 0x71


// AK8963 registers (magnetometer)
#define AK8963_WHO_AM_I     0x00
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09
#define AK8963_CNTL1        0x0A
#define AK8963_CNTL2        0x0B
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12

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
extern float mag_offset[3];
extern float mag_adjust[3];
extern float mag_raw_initial[3];

// function prototypes
void mpu9250_reset(void);
void mpu9250_configure(void);
void mpu9250_read_raw(int16_t accel_raw[3], int16_t gyro_raw[3], int16_t *temp_raw);
void mpu9250_read_mag_raw(int16_t mag_raw[3]);
void mpu9250_convert_accel(const int16_t accel_raw[3], float accel[3]);
void mpu9250_convert_gyro(const int16_t gyro_raw[3], float gyro[3]);
void mpu9250_convert_temp(const int16_t temp_raw, float *temp);
void mpu9250_convert_mag(const int16_t mag_raw[3], float mag[3]);
void mpu9250_calibrate(void);


#ifdef __cplusplus
}
#endif

#endif
