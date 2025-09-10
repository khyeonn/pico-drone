#ifndef MEKF_H
#define MEKF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus 
extern "C" {
#endif


// assuming gravity points along +Z in inertial frame
static const float G_INERTIAL[3] = {0.0f, 0.0f, -1.0f};

static float mag_reference_body[3];


typedef struct {
    float q[4]; // quaternions representing orientation [w, x, y, z]
    float gyro_bias[3];

    // covariance of 6D error state:
    // [ dTheta [3], dBias[3] ]
    float P[6][6]; // error state prediction covariance (attitude error + gyro bias error)
    float Q[6][6]; // process noise covariance for error state


    float R_accel[3][3]; // accelerometer measurement noise covariance
    float R_mag[3][3]; // magnetometer measurement noise covariance
} MEKF;


typedef struct {
    float theta;
    float bias;
    float accel;
    float mag;
} sigma;


void mekf_init(MEKF *params, const sigma *sigma);
void init_quat_from_accel_mag(const float accel[3], const float mag[3], float q[4], int zero_yaw);
void mekf_predict(MEKF *params, const float gyro[3], float dt);
void mekf_update(MEKF *params, const float meas_body[3], const float ref_inertial[3], const float R[3][3]);
void mekf_update_mag_yaw(MEKF *params, const float mag_meas[3], const float mag_ref[3], const float R[3][3]);
void mekf_update_(MEKF *params, const float mag[3], const float accel[3]);
void get_mag_reference(float ref[3]);
void vec_normalize3(float v[3]);
void quat_to_euler(const MEKF *params , float *roll_deg, float *pitch_deg, float *yaw_deg);


#ifdef __cplusplus
}
#endif


#endif