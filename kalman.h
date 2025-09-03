#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

void kalman_init(Kalman_t *k, float q_angle, float q_bias, float r_measure, float initial_angle);
float kalman_update(Kalman_t *k, float measured_angle, float gyro_rate, float dt);


#ifdef __cplusplus
}
#endif

#endif