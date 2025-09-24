#pragma once
#include <cmath>
#include <array>


struct Vector3f {
    float x, y, z;

    Vector3f(): x(0), y(0), z(0) {};
    Vector3f(float x, float y, float z): x(x), y(y), z(z) {};
    Vector3f(const std::array<float, 3>& arr): x(arr[0]), y(arr[1]), z(arr[2]) {}

    Vector3f operator+(const Vector3f& other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }

    Vector3f operator-(const Vector3f& other) const {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }

    Vector3f operator-() const {
        return Vector3f(-x, -y, -z);
    }

    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }

    Vector3f operator/(float scalar) const {
        return Vector3f(x / scalar, y / scalar, z / scalar);
    }

    float dot(const Vector3f& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3f cross(const Vector3f& other) const {
        return Vector3f(y * other.z - z * other.y,
                        z * other.x - x * other.z,
                        x * other.y - y * other.x);
    }

    float norm() const {
        return sqrtf(x*x + y*y + z*z);
    }

    Vector3f normalized() const {
        float n = norm();
        return (n > 1e-8) ? *this/n : Vector3f(0, 0, 0);
    }

    void normalize() {
        float n = norm();
        if (n > 1e-8f) {
            x /= n;
            y /= n;
            z /= n;
        } else {
            x = y = z = 0;
        }
    }
};