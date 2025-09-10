#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu9250.h"
#include "mekf.h"


MEKF mekf;
sigma noise = {
    .theta = 0.1f, 
    .bias = 0.01f, 
    .accel = 0.01f, 
    .mag = 1.0f};


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

    int16_t mag_raw_initial[3], accel_raw_initial[3], gyro_raw_initial[3], temp_raw_initial;
    float mag_initial[3], accel_initial[3];

    mpu9250_read_raw(accel_raw_initial, gyro_raw_initial, &temp_raw_initial);
    mpu9250_read_mag_raw(mag_raw_initial);
    mpu9250_convert_mag(mag_raw_initial, mag_initial);
    mpu9250_convert_accel(accel_raw_initial, accel_initial);

    float mag_ref[3] = {mag_initial[0], mag_initial[1], mag_initial[2]};

    mekf_init(&mekf, &noise);
    float q_initial[4];
    init_quat_from_accel_mag(accel_initial, mag_ref, q_initial, 1); // 1 = zero yaw
    memcpy(mekf.q, q_initial, sizeof(q_initial));

    absolute_time_t last_time = get_absolute_time();

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

        
        mekf_predict(&mekf, gyro, dt);
        mekf_update(&mekf, accel, G_INERTIAL, mekf.R_accel);
        mekf_update(&mekf, mag, mag_ref, mekf.R_mag);
       

        float roll, pitch, yaw;
        quat_to_euler(&mekf, &roll, &pitch, &yaw);

        printf("Roll: %7.2f, Pitch: %7.2f, Yaw: %7.2f\n", roll, pitch, yaw);
        // printf("q = [%.3f %.3f %.3f %.3f]\n", mekf.q[0], mekf.q[1], mekf.q[2], mekf.q[3]);

        // printf("ax=%6.2f ay=%6.2f az=%6.2f gx=%6.2f gy=%6.2f gz=%6.2f mx=%6.2f, my=%6.2f, mz=%6.2f, T=%5.2f degC\n",
        //         accel[0], accel[1], accel[2],
        //         gyro[0], gyro[1], gyro[2],
        //         mag[0], mag[1], mag[2],
        //         temp);

            
        sleep_ms(50);
    }
    return 0;
}