#include <stdio.h>
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
#define SAMPLE_RATE_DIV 1 // Sample Rate = Gyroscope Output Rate / (1 + SAMPLE_RATE_DIV)


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


void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    *temp = (buffer[6] << 8) | buffer[7];
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}


int main() {
    stdio_init_all();

    // initialize i2c
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    mpu6050_reset();
    mpu6050_configure();

    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I_REG;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    printf("WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        printf("MPU6050 not detected!\n");
        while (1);
    }

    int16_t accel[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        float accel_g[3];
        accel_g[0] = accel[0] / ACCEL_SCALE_FACTOR;
        accel_g[1] = accel[1] / ACCEL_SCALE_FACTOR;
        accel_g[2] = accel[2] / ACCEL_SCALE_FACTOR;

        float gyro_dps[3];
        gyro_dps[0] = gyro[0] / GYRO_SCALE_FACTOR;
        gyro_dps[1] = gyro[1] / GYRO_SCALE_FACTOR;
        gyro_dps[2] = gyro[2] / GYRO_SCALE_FACTOR;


        printf("Accel (g): ax=%.2f ay=%.2f az=%.2f | Gyro (dps): gx=%.2f gy=%.2f gz=%.2f | Temp: %.2f C\n",
               accel_g[0], accel_g[1], accel_g[2],
               gyro_dps[0], gyro_dps[1], gyro_dps[2],
               (temp / 340.0) + 36.53);

        sleep_ms(200);
    }
    return 0;
}