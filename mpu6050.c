#include "mpu6050.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

float accel_offset[3] = {0.0f};
float gyro_offset[3] = {0.0f};

void mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu6050_configure() {
    // set accel range
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    // set gyro range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    // set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}


void mpu6050_read(float accel[3], float gyro[3], float *temp) {
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel_raw[0] = (buffer[0] << 8) | buffer[1];
    accel_raw[1] = (buffer[2] << 8) | buffer[3];
    accel_raw[2] = (buffer[4] << 8) | buffer[5];
    temp_raw = (buffer[6] << 8) | buffer[7];
    gyro_raw[0] = (buffer[8] << 8) | buffer[9];
    gyro_raw[1] = (buffer[10] << 8) | buffer[11];
    gyro_raw[2] = (buffer[12] << 8) | buffer[13];

    accel[0] = (accel_raw[0] / ACCEL_SCALE_FACTOR) - accel_offset[0];
    accel[1] = (accel_raw[1] / ACCEL_SCALE_FACTOR) - accel_offset[1];
    accel[2] = (accel_raw[2] / ACCEL_SCALE_FACTOR) - accel_offset[2];

    gyro[0] = (gyro_raw[0] / GYRO_SCALE_FACTOR) - gyro_offset[0];
    gyro[1] = (gyro_raw[1] / GYRO_SCALE_FACTOR) - gyro_offset[1];
    gyro[2] = (gyro_raw[2] / GYRO_SCALE_FACTOR) - gyro_offset[2];

    *temp = (temp_raw / 340.0f) + 36.54f;
}


void mpu6050_calibrate() {
    float accel_sum[3] = {0}, gyro_sum[3] = {0};
    float accel[3], gyro[3], temp;

    for (int i = 0; i < 2000; i++) {
        mpu6050_read(accel, gyro, &temp);

        accel_sum[0] += accel[0];
        accel_sum[1] += accel[1];
        accel_sum[2] += accel[2];

        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];

        sleep_ms(5);
    }

    accel_offset[0] = accel_sum[0] / 2000.0f;
    accel_offset[1] = accel_sum[1] / 2000.0f;
    accel_offset[2] = (accel_sum[2] / 2000.0f) + 1.0f; // assuming static position with Z axis pointing down

    gyro_offset[0] = gyro_sum[0] / 2000.0f;
    gyro_offset[1] = gyro_sum[1] / 2000.0f;
    gyro_offset[2] = gyro_sum[2] / 2000.0f;
}
