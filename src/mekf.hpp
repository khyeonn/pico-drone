#pragma once
#include "quaternion.hpp"
#include "vector3f.hpp"
#include "matrix.hpp"


class MEKF {
public:

    MEKF(float P0, float sigma_gyro, float sigma_bias, float sigma_accel, float sigma_mag): 
    sigma_gyro(sigma_gyro), sigma_bias(sigma_bias), sigma_accel(sigma_accel), sigma_mag(sigma_mag) {
        q = Quaternion();
        b = Vector3f(0.0f, 0.0f, 0.0f);
        P = Matrix<6, 6>(P0);
        
        // Measurement noise
        R(0, 0) = R(1, 1) = R(2, 2) = this->sigma_accel * this->sigma_accel;
        R(3, 3) = R(4, 4) = R(5, 5) = this->sigma_mag * this->sigma_mag;

        G(0,3) = G(1,4) = G(2,5) = -1.0f;

        I = Matrix<6, 6>(1.0f);
    }

    const Quaternion& get_quat() const { return q; }
    const Vector3f& get_bias() const { return b; }

    void predict(const Vector3f& gyro_meas, float dt);
    void update(const Vector3f& accel_in, const Vector3f& mag_in);
    

private:
    Vector3f G_REF;
    Vector3f M_REF;
    bool mag_initialized = false;

    Quaternion q = {1, 0, 0, 0};        // [w, x, y, z]
    Vector3f b = {0.0f, 0.0f, 0.0f};    // gyro bias
    float sigma_gyro;
    float sigma_bias;
    float sigma_accel;
    float sigma_mag;

    Matrix<6, 6> P;     // covariance
    Matrix<6, 6> Q;     // process noise
    Matrix<6, 6> R;     // accel + mag noise
    Matrix<6,6> G;      // process model

    Matrix<6, 6> I;     // identity matrix for ease of operations

    // helpers
    void processCov(float dt);
    Matrix<6,6> computeH() const;
    static Matrix<3, 3> skew3(const Vector3f& v);
    static Vector3f projectToHorizontal(const Vector3f& accel, const Vector3f& mag);
};


