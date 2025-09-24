#pragma once

#include <cmath>
#include <cstring>


template<int ROWS, int COLS>
class Matrix { 
private:
    float data[ROWS][COLS];

public:
    Matrix() {
        memset(data, 0, sizeof(data));
    }

    Matrix(float val) {
        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                data[i][j] = (i == j) ? val : 0.0f;
            }
        }
    }

    float& operator()(int i, int j) { return data[i][j]; }
    const float& operator()(int i, int j) const {return data[i][j]; }

    float& operator()(int i) { return data[i][0]; }
    const float& operator()(int i) const {return data[i][0]; }

    constexpr int rows() const { return ROWS; }
    constexpr int cols() const { return COLS; }
    constexpr std::array<int, 2> dim() const { return {ROWS, COLS}; }

    
    template<int OTHER_COLS>
    Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS>& other) const {
        Matrix<ROWS, OTHER_COLS> result;

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<OTHER_COLS; j++) {
                result(i, j) = 0.0f;
                for (int k=0; k<COLS; k++) {
                    result(i, j) += data[i][k] * other(k, j);
                }
            }
        }
        return result;
    }

    Matrix<ROWS, COLS> operator*(float scalar) const {
        Matrix<ROWS, COLS> result;
        for (int i = 0; i < ROWS; i++) {
            for (int k = 0; k < COLS; k++) {
                result(i, k) = data[i][k] * scalar;
            }
        }
        return result;
    }

    Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& other) const {
        Matrix<ROWS, COLS> result;

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                result(i, j) = data[i][j] + other(i, j);
            }
        }
        return result;
    }

    Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS>& other) const {
        Matrix<ROWS, COLS> result;

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                result(i, j) = data[i][j] - other(i, j);
            }
        }
        return result;
    }

    Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> result;

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                result(j, i) = data[i][j];
            }
        }
        return result;
    }

    // In-place matrix inversion using Gauss-Jordan (square matrices ONLY)
    void invert() {
        float augmented[ROWS][2 * COLS];

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                augmented[i][j] = data[i][j];
                augmented[i][j + COLS] = (i == j) ? 1.0f : 0.0f;
            }
        }

        for (int i=0; i<ROWS; i++) {
            float max_val = fabs(augmented[i][i]);
            int max_row = i;

            for (int k=i+1; k<ROWS; k++) {
                if (fabs(augmented[k][i]) > max_val) {
                    max_val = fabs(augmented[k][i]);
                    max_row = k;
                }
            }

            if (max_row != i) {
                for (int k=0; k<2*COLS; k++) {
                    float tmp = augmented[i][k];
                    augmented[i][k] = augmented[max_row][k];
                    augmented[max_row][k] = tmp;
                }
            }

            float pivot = augmented[i][i];
            if (fabs(pivot) < 1e-8) continue;


            for (int k=0; k<2*COLS; k++) {
                augmented[i][k] /= pivot;
            }


            for (int k=0; k<ROWS; k++) {
                if (k != i) {
                    float factor = augmented[k][i];
                    for (int j=0; j<2*COLS; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }

        for (int i=0; i<ROWS; i++) {
            for (int j=0; j<COLS; j++) {
                data[i][j] = augmented[i][j + COLS];
            }
        }
    }

    // Non-destructive invert
    Matrix<ROWS, COLS> inverted() const {
        Matrix<ROWS, COLS> result = *this;
        result.invert();
        return result;
    }


    // ------- Vector operations
    float dot(const Matrix<ROWS, 1>& other) const {
        float result = 0.0f;
        for (int i = 0; i < ROWS; i++) {
            result += data[i][0] * other(i,0);
        }
        return result;
    }

    Matrix<3, 1> cross(const Matrix<3, 1>& other) const {
        Matrix<3, 1> result;
        result(0, 0) = data[1][0]*other(2, 0) - data[2][0]*other(1, 0);
        result(1, 0) = data[2][0]*other(0, 0) - data[0][0]*other(2, 0);
        result(2, 0) = data[0][0]*other(1, 0) - data[1][0]*other(0, 0);
        return result;
    }

    float norm() const {
        return sqrtf(dot(*this));
    }

    Matrix<ROWS, 1> normalized() const {
        Matrix<ROWS, 1> result;
        float n = norm();
        if (n > 1e-8f) {
            for (int i=0; i<ROWS; i++) result(i, 0) = data[i][0] / n;
        }
        return result;
    }
};