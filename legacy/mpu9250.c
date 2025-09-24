#include "mpu9250.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

float accel_offset[3] = {0.0f};
float gyro_offset[3] = {0.0f};
float mag_adjust[3] = {1.0f, 1.0f, 1.0f};

void mpu9250_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu9250_configure() {
    // set accel range
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, accel_config, 2, false);

    // set gyro range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, gyro_config, 2, false);

    // set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, sample_rate, 2, false);

    // enable i2c bypass to access magnetometer directly
    uint8_t bypass[] = {REG_INT_PIN_CFG, 0x02};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, bypass, 2, false);

    // --- magnetometer configuration ---
    // reset magnetometer
    uint8_t mag_reset[] = {AK8963_CNTL2, 0x01};
    i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_reset, 2, false);
    sleep_ms(20);

    // enter fuse ROM access mode to read sensitivity adjustment values
    uint8_t mag_fuse_rom_mod[] = {AK8963_CNTL1, 0x0F};
    i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_fuse_rom_mod, 2, false);
    sleep_ms(100);

    // read sensitivity adjustment values
    uint8_t asa_reg = AK8963_ASAX;
    uint8_t asa_values[3];
    i2c_write_blocking(I2C_PORT, AK8963_ADDR, &asa_reg, 1, true);
    i2c_read_blocking(I2C_PORT, AK8963_ADDR, asa_values, 3, false);

    // calculate magnetometer adjustment values
    mag_adjust[0] = ((asa_values[0] - 128) / 256.0f) + 1.0f;
    mag_adjust[1] = ((asa_values[1] - 128) / 256.0f) + 1.0f;
    mag_adjust[2] = ((asa_values[2] - 128) / 256.0f) + 1.0f;

    // need to power down first before changing modes
    uint8_t power_down[] = {AK8963_CNTL1, 0x00};
    i2c_write_blocking(I2C_PORT, AK8963_ADDR, power_down, 2, false);
    sleep_ms(100);

    // set to continuous measurement mode 2, 16-bit output
    uint8_t mag_config[] = {AK8963_CNTL1, 0x16}; 
    i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_config, 2, false);
    sleep_ms(100);
}

void mpu9250_read_raw(int16_t accel_raw[3], int16_t gyro_raw[3], int16_t *temp_raw) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU9250_ADDR, buffer, 14, false);

    accel_raw[0] = (buffer[0] << 8) | buffer[1];
    accel_raw[1] = (buffer[2] << 8) | buffer[3];
    accel_raw[2] = (buffer[4] << 8) | buffer[5];
    *temp_raw = (buffer[6] << 8) | buffer[7];
    gyro_raw[0] = (buffer[8] << 8) | buffer[9];
    gyro_raw[1] = (buffer[10] << 8) | buffer[11];
    gyro_raw[2] = (buffer[12] << 8) | buffer[13];
}


void mpu9250_read_mag_raw(int16_t mag_raw[3]) {
    uint8_t buffer[7];
    uint8_t reg = AK8963_XOUT_L;

    i2c_write_blocking(I2C_PORT, AK8963_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, AK8963_ADDR, buffer, 7, false);

    mag_raw[0] = buffer[0] | (buffer[1] << 8);
    mag_raw[1] = buffer[2] | (buffer[3] << 8);
    mag_raw[2] = buffer[4] | (buffer[5] << 8);
}

void mpu9250_calibrate() {
    float accel_sum[3] = {0}, gyro_sum[3] = {0}; 
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float accel[3], gyro[3], temp;

    for (int i = 0; i < 2000; i++) {
        mpu9250_read_raw(accel_raw, gyro_raw, &temp_raw);
        
        mpu9250_convert_accel(accel_raw, accel);
        mpu9250_convert_gyro(gyro_raw, gyro);
        

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
    accel_offset[2] = accel_sum[2] / 2000.0f - 1.0f; // adjust for gravity and current orientation of sensor (assuming Z axis is positive up)

    gyro_offset[0] = gyro_sum[0] / 2000.0f;
    gyro_offset[1] = gyro_sum[1] / 2000.0f;
    gyro_offset[2] = gyro_sum[2] / 2000.0f;
}


void mpu9250_convert_mag(const int16_t mag_raw[3], float mag[3]) {
    mag[0] = (mag_raw[0] * mag_adjust[0] * 0.15f);
    mag[1] = (mag_raw[1] * mag_adjust[1] * 0.15f);
    mag[2] = (mag_raw[2] * mag_adjust[2] * 0.15f);
}


void mpu9250_convert_accel(const int16_t accel_raw[3], float accel[3]) {
    accel[0] = (accel_raw[0] / ACCEL_SCALE_FACTOR) - accel_offset[0];
    accel[1] = (accel_raw[1] / ACCEL_SCALE_FACTOR) - accel_offset[1];
    accel[2] = (accel_raw[2] / ACCEL_SCALE_FACTOR) - accel_offset[2];
}

void mpu9250_convert_gyro(const int16_t gyro_raw[3], float gyro[3]) {
    gyro[0] = (gyro_raw[0] / GYRO_SCALE_FACTOR) - gyro_offset[0];
    gyro[1] = (gyro_raw[1] / GYRO_SCALE_FACTOR) - gyro_offset[1];
    gyro[2] = (gyro_raw[2] / GYRO_SCALE_FACTOR) - gyro_offset[2];
}

void mpu9250_convert_temp(const int16_t temp_raw, float *temp) {
    *temp = (temp_raw / 340.0f) + 36.54f;
}