#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu9250.h"
#include "kalman.h"



Kalman_t kf_pitch, kf_roll; //, kf_yaw;


int main() {
    stdio_init_all();
    sleep_ms(2000); 

    // initialize i2c
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    mpu9250_reset();
    mpu9250_configure();

    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I_REG;
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU9250_ADDR, &who_am_i, 1, false);
    printf("WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x71) {
        printf("MPU9250 not detected!\n");
        while (1);
    }

    printf("Calibrating... Keep the sensor still.\n");
    mpu9250_calibrate();
    printf("Calibration done.\n");

    kalman_init(&kf_pitch, 0.001f, 0.003f, 0.03f, 0.0f);
    kalman_init(&kf_roll, 0.001f, 0.003f, 0.03f, 0.0f);
    // kalman_init(&kf_yaw, 0.001f, 0.003f, 0.03f, 0.0f);
    absolute_time_t last_time = get_absolute_time();

    float yaw = 0.0f;
    float accel[3], gyro[3], mag[3], temp;
    int16_t accel_raw[3], gyro_raw[3], mag_raw[3], temp_raw;

    while (1) {
        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1e6f;
        last_time = current_time;

        mpu9250_read_raw(accel_raw, gyro_raw, &temp_raw);
        mpu9250_read_mag_raw(mag_raw);

        // convert data from raw to physical units
        mpu9250_convert_accel(accel_raw, accel);
        mpu9250_convert_gyro(gyro_raw, gyro);
        mpu9250_convert_temp(temp_raw, &temp);
        mpu9250_convert_mag(mag_raw, mag);

        float roll_accel = atan2f(accel[1], accel[2]) * 180.0f / M_PI;
        float pitch_accel = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0f / M_PI;
        float yaw_mag = atan2f(-mag[1], mag[0]);

        float roll = kalman_update(&kf_roll, roll_accel, gyro[0], dt);
        float pitch = kalman_update(&kf_pitch, pitch_accel, gyro[1], dt);
        // float yaw = kalman_update(&kf_yaw, yaw_mag, gyro[2], dt);

        // yaw += gyro[2] * dt;

        // float yaw_error = yaw_mag - yaw;
        // if (yaw_error > 180.0f) yaw_error -= 360.0f;
        // if (yaw_error < -180.0f) yaw_error += 360.0f;
        // yaw += yaw_error * 0.1f;

        // printf("Roll: %7.2f, Pitch: %7.2f, Yaw: %7.2f\n", roll, pitch, yaw);

        printf("ax=%6.2f ay=%6.2f az=%6.2f gx=%6.2f gy=%6.2f gz=%6.2f mx=%6.2f, my=%6.2f, mz=%6.2f, T=%5.2f degC\n",
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                mag[0], mag[1], mag[2],
                temp);
            
        sleep_ms(50);
    }
    return 0;
}