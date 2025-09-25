#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "mpu9250.hpp"
#include "mekf.hpp"

MEKF filter(0.1f, 0.1f, 0.1f, 0.1f, 0.1f);


int main() {
    stdio_init_all();
    sleep_ms(2000); 

    printf("Initializing MPU9250.\n");
    MPU9250 imu;
    while (!imu.init()) {
        printf("MPU9250 could not be initialized.\n");
        sleep_ms(1000);
    };
    printf("MPU9250 initialized.\n");
    printf("Calibrating accelerometer. Keep sensor still.\n");
    imu.calibrate();
    printf("Accelerometer calibration done.\n");

    printf("Calibrating magnetometer. Rotate sensor smoothly.\n");
    sleep_ms(1500);
    imu.calibrate_mag();
    printf("Magnetometer calibration done.\n");

    absolute_time_t last_time = get_absolute_time();
    while (true) {

        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1e6f;

        if (dt < 1e-6f) continue;
        last_time = current_time;

        imu.read_accel_gyro_raw();
        imu.read_mag_raw();

        imu.convert_accel_raw();
        imu.convert_gyro_raw();
        imu.convert_temp_raw();
        imu.convert_mag_raw();

        Vector3f accel = imu.get_accel();
        Vector3f gyro  = imu.get_gyro();
        Vector3f mag   = imu.get_mag();
        float temp = imu.get_temp();

        filter.predict(gyro, dt);
        filter.update(accel, mag);

        printf("%.2f, %.2f, %.2f, %.2f\n", filter.get_quat().w, filter.get_quat().x, filter.get_quat().y, filter.get_quat().z);

        // printf("ax=%6.2f ay=%6.2f az=%6.2f gx=%6.2f gy=%6.2f gz=%6.2f mx=%6.2f, my=%6.2f, mz=%6.2f, T=%5.2f degC\n",
        //         accel.x, accel.y, accel.z,
        //         gyro.x, gyro.y, gyro.z,
        //         mag.x, mag.y, mag.z,
        //         temp);

            
        sleep_ms(50);
    }
    return 0;
}