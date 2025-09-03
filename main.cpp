#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "kalman.h"



Kalman_t kf_pitch, kf_roll;


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

    printf("Calibrating... Keep the sensor still.\n");
    mpu6050_calibrate();
    printf("Calibration done.\n");

    kalman_init(&kf_pitch, 0.001f, 0.003f, 0.03f, 0.0f);
    kalman_init(&kf_roll, 0.001f, 0.003f, 0.03f, 0.0f);
    absolute_time_t last_time = get_absolute_time();
    float yaw = 0.0f;

    while (1) {
        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1e6f;
        last_time = current_time;

        float accel[3], gyro[3], temp;
        mpu6050_read(accel, gyro, &temp);

        float roll_accel = atan2f(accel[1], accel[2]) * 180.0f / M_PI;
        float pitch_accel = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0f / M_PI;

        float roll = kalman_update(&kf_roll, roll_accel, gyro[0], dt);
        float pitch = kalman_update(&kf_pitch, pitch_accel, gyro[1], dt);

        yaw += gyro[2] * dt;

        printf("Roll: %7.2f, Pitch: %7.2f, Yaw: %7.2f\n", roll, pitch, yaw);

        // printf("ax=%6.2f ay=%6.2f az=%6.2f gx=%6.2f gy=%6.2f gz=%6.2f T=%5.2f degC\n",
        //         accel[0], accel[1], accel[2],
        //         gyro[0], gyro[1], gyro[2],
        //         temp);
            
        sleep_ms(50);
    }
    return 0;
}