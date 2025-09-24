#include "mekf.hpp"
#include <cmath>
#include <stdio.h>

Matrix<3, 3> MEKF::skew3(const Vector3f& v) {
    Matrix<3, 3> skew;
    skew(0, 0) = 0;    skew(0, 1) = -v.z; skew(0, 2) = v.y;
    skew(1, 0) = v.z;  skew(1, 1) = 0;    skew(1, 2) = -v.x;
    skew(2, 0) = -v.y; skew(2, 1) = v.x;  skew(2, 2) = 0;
    return skew;
}


Vector3f MEKF::projectToHorizontal(const Vector3f& accel, const Vector3f& mag) {
    Vector3f gravity_norm = accel.normalized();

    Vector3f projection = gravity_norm * mag.dot(gravity_norm);
    Vector3f horizontal = mag - projection; 

    float magnitude = horizontal.norm();

    if (magnitude > 1e-6f) {
        horizontal = horizontal / magnitude;
    } else {
        // Fallback: use x-axis as horizontal reference
        horizontal = Vector3f(1.0f, 0.0f, 0.0f);
    }

    return horizontal;
}


void MEKF::processCov(float dt) {
    Matrix<3, 3> cov_gyro(sigma_gyro * sigma_gyro);
    Matrix<3, 3> cov_bias(sigma_bias * sigma_bias);

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            Q(i, j) = cov_gyro(i, j)*dt + cov_bias(i, j)*(dt*dt*dt)/3.0f;
        }
    }

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            float val = cov_bias(i, j) * (dt*dt)/2.0f;
            Q(i, j+3) = -val;
            Q(i+3, j) = -val;
        }
    }

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            Q(i+3, j+3) = cov_bias(i, j) * dt;
        }
    }
}


Matrix<6, 6> MEKF::computeH() const {
    Matrix<6, 6> H;

    Vector3f g_b = q.conj().rotate(G_REF);
    Vector3f m_b = q.conj().rotate(M_REF);


    Matrix<3, 3> skew_g = skew3(g_b);
    Matrix<3, 3> skew_m = skew3(m_b);

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            H(i, j) = skew_g(i, j);
            H(i+3, j) = skew_m(i, j);
        }
    }

    return H;
}


// make sure gyro_meas is converted to rad/s
void MEKF::predict(const Vector3f& gyro_meas, float dt) {
    const float DEG2RAD = 3.141592653589793f / 180.0f;
    Vector3f gyro_rad = gyro_meas * DEG2RAD;

    Vector3f gyro_pred = gyro_rad - b;

    Quaternion q_dot = {0, gyro_pred.x , gyro_pred.y, gyro_pred.z};
    q = q + (q * q_dot) * (0.5f * dt);
    q.normalize();

    Matrix<3, 3> skew_gyro = skew3(gyro_pred);
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            G(i, j) = -skew_gyro(i, j);
        }
    }


    Matrix<6, 6> F = I + G*dt;
    processCov(dt);
    P = F * P * F.transpose() + Q;
}


void MEKF::update(const Vector3f& accel_in, const Vector3f& mag_in) {
    Vector3f accel_meas = accel_in.normalized();
    Vector3f mag_norm = mag_in.normalized();
    Vector3f mag_meas = projectToHorizontal(accel_meas, mag_norm);

    Matrix<6, 6> H = computeH();
    Matrix<6, 6> PH_t = P * H.transpose();
    Matrix<6, 6> S = H * PH_t + R;
    Matrix<6, 6> K = PH_t * S.inverted();

    Vector3f g_b = q.conj().rotate(G_REF);
    Vector3f m_b = q.conj().rotate(M_REF);

    Matrix<6, 1> z;
    Matrix<6, 1> z_pred;
    Matrix<6, 1> innovation;
    Matrix<6, 1> dx;

    z(0) = accel_meas.x; z(1) = accel_meas.y; z(2) = accel_meas.z;
    z(3) = mag_meas.x; z(4) = mag_meas.y; z(5) = mag_meas.z;
    z_pred(0) = g_b.x; z_pred(1) = g_b.y; z_pred(2) = g_b.z;
    z_pred(3) = m_b.x; z_pred(4) = m_b.y; z_pred(5) = m_b.z;

    innovation = z - z_pred;
    dx = K * innovation;

    Vector3f delta_theta = {dx(0), dx(1), dx(2)};
    q = q * Quaternion(1, 0.5f*delta_theta.x, 0.5f*delta_theta.y, 0.5f*delta_theta.z);
    q.normalize();

    b.x = b.x + dx(3);
    b.y = b.y + dx(4);
    b.z = b.z + dx(5);

    P = (I - K*H) * P;
}