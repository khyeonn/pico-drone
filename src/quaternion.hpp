#pragma once

#include <cmath>
#include "vector3f.hpp"
#include "matrix.hpp"
#include <stdio.h>


struct Quaternion {
    float w, x, y, z;
    
    Quaternion(): w(1), x(0), y(0), z(0) {};
    Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {};

    Quaternion operator+(const Quaternion& other) const {
        return {w + other.w, x + other.x, y + other.y, z + other.z};
    }

    Quaternion operator-(const Quaternion& other) const {
        return {w - other.w, x - other.x, y - other.y, z - other.z};
    }

    Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }
    
    Quaternion operator*(const Quaternion& other) const {
        return {
            w*other.w - x*other.x - y*other.y - z*other.z,
            w*other.x + x*other.w + y*other.z - z*other.y,
            w*other.y - x*other.z + y*other.w + z*other.x,
            w*other.z + x*other.y - y*other.x + z*other.w};
    }

    Quaternion operator/(float scalar) const {
        return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
    }

    Quaternion conj() const {
        return Quaternion(w, -x, -y, -z);
    }

    Vector3f rotate(const Vector3f& v) const {
        Quaternion vq(0, v.x, v.y, v.z);

        // q' = q * v * conj(q);
        Quaternion qr = (*this) * vq * this->conj();

        return Vector3f(qr.x, qr.y, qr.z);
    }

    float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    Quaternion normalized() const {
        float n = norm();
        return (n > 1e-8) ? *this/n : Quaternion();
    }

    void normalize() {
        float n = norm();
        if (n > 1e-8) {
            w = w / n;
            x = x / n;
            y = y / n;
            z = z / n;
        } else {
            w = 1.0f;
            x = y = z = 0.0f;
        }
    }
};


// i should have put this in its own header but it's here now and im too lazy to fix it
struct EulerAngles {
    float yaw;
    float pitch;
    float roll;
};

inline Vector3f quat2euler(const Quaternion& q) {
    Vector3f euler;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.x = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        euler.y = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        euler.y = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.z = atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

inline EulerAngles rad2deg(const Vector3f& in) {
    EulerAngles euler;
    const float RAD2DEG = 180.0f / 3.141592653589f;

    euler.roll = in.x * RAD2DEG;
    euler.pitch = in.y * RAD2DEG;
    euler.yaw = in.z * RAD2DEG;

    return euler;
}


// From Blender's implementation
// https://github.com/blender/blender/blob/756538b4a117cb51a15e848fa6170143b6aafcd8/source/blender/blenlib/intern/math_rotation.c#L272
inline Quaternion mat2quat(const Matrix<3, 3>& R) {
    Quaternion q;
    float trace;

    if (R(2, 2) < 0.0f) {
        if (R(0, 0) > R(1, 1)) {
            trace = 1.0f + R(0, 0) - R(1, 1) - R(2, 2);
            float s = 2.0f * sqrtf(trace);
            if (R(1, 2) < R(2, 1)) {
                s = -s; // Ensure w ≥ 0
            }
            q.x = 0.25f * s;
            s = 1.0f / s;
            q.w = (R(1, 2) - R(2, 1)) * s;
            q.y = (R(0, 1) + R(1, 0)) * s;
            q.z = (R(2, 0) + R(0, 2)) * s;

            if (trace == 1.0f && (q.w == 0.0f && q.y == 0.0f && q.z == 0.0f)) {
                q.x = 1.0f;
            }
        }
        else {
            trace = 1.0f - R(0, 0) + R(1, 1) - R(2, 2);
            float s = 2.0f * sqrtf(trace);
            if (R(2, 0) < R(0, 2)) {
                s = -s;
            }
            q.y = 0.25f * s;
            s = 1.0f / s;
            q.w = (R(2, 0) - R(0, 2)) * s;
            q.x = (R(0, 1) + R(1, 0)) * s;
            q.z = (R(1, 2) + R(2, 1)) * s;

            if (trace == 1.0f && (q.w == 0.0f && q.x == 0.0f && q.z == 0.0f)) {
                q.y = 1.0f;
            }
        }
    }
    else {
        if (R(0, 0) < -R(1, 1)) {
            trace = 1.0f - R(0, 0) - R(1, 1) + R(2, 2);
            float s = 2.0f * sqrtf(trace);
            if (R(0, 1) < R(1, 0)) {
                s = -s;
            }
            q.z = 0.25f * s;
            s = 1.0f / s;
            q.w = (R(0, 1) - R(1, 0)) * s;
            q.x = (R(2, 0) + R(0, 2)) * s;
            q.y = (R(1, 2) + R(2, 1)) * s;

            if (trace == 1.0f && (q.w == 0.0f && q.x == 0.0f && q.y == 0.0f)) {
                q.z = 1.0f;
            }
        }
        else {
            trace = 1.0f + R(0, 0) + R(1, 1) + R(2, 2);
            float s = 2.0f * sqrtf(trace);
            q.w = 0.25f * s;
            s = 1.0f / s;
            q.x = (R(1, 2) - R(2, 1)) * s;
            q.y = (R(2, 0) - R(0, 2)) * s;
            q.z = (R(0, 1) - R(1, 0)) * s;

            if (trace == 1.0f && (q.x == 0.0f && q.y == 0.0f && q.z == 0.0f)) {
                q.w = 1.0f;
            }
        }
    }

    // Normalize just in case
    q.normalize();
    return q;
}
