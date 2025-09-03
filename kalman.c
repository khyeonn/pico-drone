#include "kalman.h"


void kalman_init(Kalman_t *k, float q_angle, float q_bias, float r_measure, float initial_angle) {
    k->Q_angle = q_angle;
    k->Q_bias = q_bias;
    k->R_measure = r_measure;

    k->angle = initial_angle;
    k->bias = 0.0f;

    k->P[0][0] = 1.0f; // P[0][0] is the variance of the angle
    k->P[0][1] = 0.0f; // P[0][1] is the covariance between angle and bias
    k->P[1][0] = 0.0f; // P[1][0] is the covariance between bias and angle
    k->P[1][1] = 1.0f; // P[1][1] is the variance of the bias
}


float kalman_update(Kalman_t *k, float measured_angle, float gyro_rate, float dt) {
    // predict
    float rate = gyro_rate - k->bias;
    k->angle += dt * rate;

    // update covariance matrix
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += dt * k->Q_bias;

    // kalman gain
    float S = k->P[0][0] + k->R_measure;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    // update estimate
    float y = measured_angle - k->angle; // measurement residual
    k->angle += K0 * y;
    k->bias += K1 * y;

    // update covariance matrix
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}
