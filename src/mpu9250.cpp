#include <stdio.h>
#include "mpu9250.hpp"
#include "pico/stdlib.h"


bool MPU9250::init() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    if (!reset()) return false;
    if (!configure()) return false;

    return true;
}


bool MPU9250::reset() {
    uint8_t reset[] = {PWR_MGMT_1, 0x80};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, reset, 2, false) != 2) return false;
    sleep_ms(200);

    uint8_t wake[] = {PWR_MGMT_1, 0x00};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, wake, 2, false) != 2) return false;
    sleep_ms(200);

    return true;
}

bool MPU9250::configure() {
    return configure_accel_gyro() && configure_mag();
}

bool MPU9250::configure_accel_gyro() {
    // Set sample rate
    uint8_t sample_rate[] = {SMPLRT_DIV, SAMPLE_RATE_DIV};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, sample_rate, 2, false) != 2) return false;

    // Set accel range
    uint8_t accel_config[] = {ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, accel_config, 2, false) != 2) return false;

    // Set gyro range
    uint8_t gyro_config[] = {GYRO_CONFIG, GYRO_CONFIG_VALUE};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, gyro_config, 2, false) != 2) return false;
    
    return true;
}


bool MPU9250::configure_mag() {
    // Enable I2C bypass to access magnetometer directly
    uint8_t bypass[] = {INT_PIN_CFG, 0x02};
    if (i2c_write_blocking(I2C_PORT, MPU9250_ADDR, bypass, 2, false) !=2) return false;

    // Reset magnetometer before continuing
    uint8_t mag_reset[] = {AK8963_CNTL2, 0x01};
    if (i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_reset, 2, false) != 2) return false;
    sleep_ms(20);

    // Enter fuse ROM access mode to read sensitivity adjustment values
    uint8_t mag_fuse_rom_mod[] = {AK8963_CNTL1, 0x0F};
    if (i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_fuse_rom_mod, 2, false) != 2) return false;
    sleep_ms(100);

    // Read sensitivity adjustment values
    uint8_t asa_reg = AK8963_ASAX;
    uint8_t asa_values[3];
    if (i2c_write_blocking(I2C_PORT, AK8963_ADDR, &asa_reg, 1, true) != 1) return false;
    if (i2c_read_blocking(I2C_PORT, AK8963_ADDR, asa_values, 3, false) != 3) return false;

    // Mag adjustment values according to datasheet: https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
    mag_adjust[0] = ((asa_values[0] - 128) / 256.0f) + 1.0f;
    mag_adjust[1] = ((asa_values[1] - 128) / 256.0f) + 1.0f;
    mag_adjust[2] = ((asa_values[2] - 128) / 256.0f) + 1.0f;

    // Need to power down first before changing modes
    uint8_t power_down[] = {AK8963_CNTL1, 0x00};
    if (i2c_write_blocking(I2C_PORT, AK8963_ADDR, power_down, 2, false) != 2) return false;
    sleep_ms(100);

    // Set to continuous measurement mode 2: 100Hz, 16-bit output
    uint8_t mag_config[] = {AK8963_CNTL1, 0x16}; 
    if (i2c_write_blocking(I2C_PORT, AK8963_ADDR, mag_config, 2, false) != 2) return false;
    sleep_ms(100);

    return true;
}

void MPU9250::read_accel_gyro_raw() {
    uint8_t buffer[14];
    uint8_t reg = ACCEL_XOUT_H;

    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU9250_ADDR, buffer, 14, false);

    accel_raw[0] = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    accel_raw[1] = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    accel_raw[2] = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

    temp_raw = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);

    gyro_raw[0] = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    gyro_raw[1] = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    gyro_raw[2] = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);
}

void MPU9250::read_mag_raw() {
    uint8_t buffer[7];
    uint8_t reg = AK8963_XOUT_L;

    i2c_write_blocking(I2C_PORT, AK8963_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, AK8963_ADDR, buffer, 7, false);

    mag_raw[0] = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    mag_raw[1] = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    mag_raw[2] = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));
}

// in g
void MPU9250::convert_accel_raw() {
    accel[0] = (accel_raw[0] / ACCEL_SCALE_FACTOR) - accel_bias[0];
    accel[1] = (accel_raw[1] / ACCEL_SCALE_FACTOR) - accel_bias[1];
    accel[2] = (accel_raw[2] / ACCEL_SCALE_FACTOR) - accel_bias[2];
}

// in degrees/sec
void MPU9250::convert_gyro_raw() {
    gyro[0] = (gyro_raw[0] / GYRO_SCALE_FACTOR) - gyro_bias[0];
    gyro[1] = (gyro_raw[1] / GYRO_SCALE_FACTOR) - gyro_bias[1];
    gyro[2] = (gyro_raw[2] / GYRO_SCALE_FACTOR) - gyro_bias[2];
}

// in uT
void MPU9250::convert_mag_raw() {
    const float MAG_LSB_TO_UT = 0.15f;
    mag[0] = (mag_raw[0] - mag_bias_raw[0]) * mag_adjust[0] * mag_scale[0] * MAG_LSB_TO_UT;
    mag[1] = (mag_raw[1] - mag_bias_raw[1]) * mag_adjust[1] * mag_scale[1] * MAG_LSB_TO_UT;
    mag[2] = (mag_raw[2] - mag_bias_raw[2]) * mag_adjust[2] * mag_scale[2] * MAG_LSB_TO_UT;
}

// degrees C
void MPU9250::convert_temp_raw() {
    temperature = (temp_raw / 340.0f) + 36.54f; 
}


void MPU9250::calibrate() {
    std::array<float, 3> accel_sum = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_sum = {0.0f, 0.0f, 0.0f};

    const int samples = 1000;
    for (int i = 0; i < samples; i++) {
        
        read_accel_gyro_raw();
        
        convert_accel_raw();
        
        accel_sum[0] += accel[0];
        accel_sum[1] += accel[1];
        accel_sum[2] += accel[2];

        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];

        sleep_ms(5);

        if (i % 100 == 0) {  
        float percent = (i / (float)samples) * 100.0f;
            printf("Accelerometer calibration progress: %2.2f%%\n", percent);
        }
    }

    accel_bias[0] = accel_sum[0] / samples;
    accel_bias[1] = accel_sum[1] / samples;
    accel_bias[2] = accel_sum[2] / samples - 1;// adjust for gravity and current orientation of sensor (assuming Z axis is positive up)

    gyro_bias_raw[0] = gyro_sum[0] / samples;
    gyro_bias_raw[1] = gyro_sum[1] / samples;
    gyro_bias_raw[2] = gyro_sum[2] / samples;

    push_gyro_biases();
}


void MPU9250::push_gyro_biases() {
    std::array<uint8_t, 6> bias_lsb;
    for (int i = 0; i < 3; i++) {
        int16_t offset_lsb = static_cast<int16_t>(-gyro_bias_raw[i] / 4); // OFFSET_LSB = X_OFFS_USR * 4 / 2^FS_SEL. Nominal FS_SEL = 0

        bias_lsb[i * 2] = (offset_lsb >> 8) & 0xFF; // high byte
        bias_lsb[i * 2 + 1] = offset_lsb & 0xFF;    // low byte
    }

    // push offset to hardware registers
    write_register(XG_OFFSET_H, bias_lsb[0]);
    write_register(XG_OFFSET_L, bias_lsb[1]);
    write_register(YG_OFFSET_H, bias_lsb[2]);
    write_register(YG_OFFSET_L, bias_lsb[3]);
    write_register(ZG_OFFSET_H, bias_lsb[4]);
    write_register(ZG_OFFSET_L, bias_lsb[5]);
}


void MPU9250::calibrate_mag() {
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};
    int32_t mx_rad, my_rad, mz_rad;

    const int samples = 1500;
    for (int i=0; i<samples; i++) {
        read_mag_raw();

        for (int j=0; j<3; j++) {
            if (mag_raw[j] > mag_max[j]) mag_max[j] = mag_raw[j];
            if (mag_raw[j] < mag_min[j]) mag_min[j] = mag_raw[j];
        }

        sleep_ms(12);
        if (i % 75 == 0) { 
        float percent = (i / (float)samples) * 100.0f;
            printf("Magnetometer calibration progress: %2.2f%%\n", percent);
        }
    }

    // Hard iron correction
    mag_bias_raw[0]  = (mag_max[0] + mag_min[0])/2; 
    mag_bias_raw[1]  = (mag_max[1] + mag_min[1])/2;  
    mag_bias_raw[2]  = (mag_max[2] + mag_min[2])/2;  


    // Soft iron correction
    mx_rad  = (mag_max[0] - mag_min[0])/2; 
    my_rad  = (mag_max[1] - mag_min[1])/2;  
    mz_rad  = (mag_max[2] - mag_min[2])/2; 

    float avg_rad = mx_rad + my_rad + mz_rad;
    avg_rad /= 3.0f;

    mag_scale[0] = avg_rad/((float)mx_rad);
    mag_scale[1] = avg_rad/((float)my_rad);
    mag_scale[2] = avg_rad/((float)mz_rad);
}


void MPU9250::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, MPU9250_ADDR, buffer, 2, false);
}