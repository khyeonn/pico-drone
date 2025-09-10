#include "mekf.h"
#include <math.h>
#include <string.h>
#include <stdio.h>


// ----- vector helpers
static void vec_copy3(const float a[3], float b[3]) {
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
}

static void vec_zero3(float v[3]) {
    v[0] = v[1] = v[2] = 0.0f;
}

static float vec_norm3(const float v[3]) {
    return sqrtf( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

void vec_normalize3(float v[3]) {
    float norm = vec_norm3(v);

    if (norm > 1e-6f) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

// ----- matrix helpers
static void skew3(const float v[3], float S[3][3]) {
    S[0][0] =  0.0f;    S[0][1] = -v[2];    S[0][2] =  v[1];
    S[1][0] =  v[2];    S[1][1] =  0.0f;    S[1][2] = -v[0];
    S[2][0] = -v[1];    S[2][1] =  v[0];    S[2][2] =  0.0f;
}


static void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}


static int inv3(const float A[3][3], float invA[3][3]) {
    float a=A[0][0], b=A[0][1], c=A[0][2];
    float d=A[1][0], e=A[1][1], f=A[1][2];
    float g=A[2][0], h=A[2][1], i=A[2][2];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);

    if (fabsf(det) < 1e-12f) return 0;

    float inv_det = 1.0f / det;

    invA[0][0] =  (e*i - f*h) * inv_det;
    invA[0][1] = -(b*i - c*h) * inv_det;
    invA[0][2] =  (b*f - c*e) * inv_det;
    invA[1][0] = -(d*i - f*g) * inv_det;
    invA[1][1] =  (a*i - c*g) * inv_det;
    invA[1][2] = -(a*f - c*d) * inv_det;
    invA[2][0] =  (d*h - e*g) * inv_det;
    invA[2][1] = -(a*h - b*g) * inv_det;
    invA[2][2] =  (a*e - b*d) * inv_det;

    return 1;
}

static void mat6_zero(float A[6][6]) {
    memset(A, 0, sizeof(float)*6*6);
}

static void mat6_eye(float A[6][6]) {
    mat6_zero(A);

    for (int i=0; i<6; i++) {
        A[i][i] = 1.0f;
    }
}

static void mat6_make_symmetric(float A[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=i+1; j<6; j++) {
            float avg = 0.5f * (A[i][j] + A[j][i]);
            A[i][j] = avg;
            A[j][i] = avg;
        }
    }
}

static void mat6_add(const float A[6][6], const float B[6][6], float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

static void mat6_subtract(const float A[6][6], const float B[6][6], float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}

static void mat6_mult(const float A[6][6], const float B[6][6], float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<6; k++) {
                result[i][j] += A[i][k] * B[k][j]; 
            }
        }
    }
}


static void mat3x6_mult_6x6(const float A[3][6], const float B[6][6], float result[3][6]) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<6; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat3x6_mult_6x3(const float A[3][6], const float B[6][3], float result[3][3]) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<6; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


static void mat6x3_mult_3x3(const float A[6][3], const float B[3][3], float result[6][3]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat6x3_mult_3x6(const float A[6][3], const float B[3][6], float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat6x6_mult_6x3(const float A[6][6], const float B[6][3], float result[6][3]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<6; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat3_mult(const float A[3][3], const float B[3][3], float result[3][3]) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = 0.0f;
            for (int k=0; k<3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat6_transpose(const float A[6][6], float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = A[j][i];
        }
    }
}

static void mat3x6_transpose(const float A[3][6], float result[6][3]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = A[j][i];
        }
    }
}


static void mat6_scale(const float A[6][6], const float scalar, float result[6][6]) {
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            result[i][j] = A[i][j] * scalar;
        }
    }
}

static void mat3_add(const float A[3][3], const float B[3][3], float result[3][3]) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}


static void mat6x3_mult_vec(const float A[6][3], const float v[3], float result[6]) {
    for (int i=0; i<6; i++) {
        float sum = 0.0f;
        for (int j=0; j<3; j++) {
            sum += A[i][j] * v[j];
        }
        result[i] = sum;
    }
}

// ----- quaternion helpers
static void quat_normalize(float quat[4]) {
    float norm = sqrtf(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
    if (norm > 1e-6f) {
        quat[0] /= norm;
        quat[1] /= norm;
        quat[2] /= norm;
        quat[3] /= norm;
    }
}

static void quat_mult(const float q1[4], const float q2[4], float q_out[4]) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quat_from_small_angle(const float theta[3], float quat[4]) {
    quat[0] = 1.0f;
    quat[1] = 0.5f * theta[0];
    quat[2] = 0.5f * theta[1];
    quat[3] = 0.5f * theta[2];
    quat_normalize(quat);
}

static void quat_rotate_vector(const float quat[4], const float v[3], float v_out[3]) {
    // q = [w, x, y, z]
    float qv[3] = { quat[1], quat[2], quat[3] };
    float tmp[3], cross_tmp[3];

    cross3(qv, v, tmp);

    float tmp2[3] = { tmp[0] + quat[0]*v[0],
                        tmp[1] + quat[0]*v[1],
                        tmp[2] + quat[0]*v[2] };
                        
    cross3(qv, tmp2, cross_tmp);

    v_out[0] = v[0] + 2.0f * cross_tmp[0];
    v_out[1] = v[1] + 2.0f * cross_tmp[1];
    v_out[2] = v[2] + 2.0f * cross_tmp[2];
}

static void quat_rotate_vector_inverse(const float quat[4], const float v[3], float v_out[3]) {
    float quat_conj[4] = { quat[0], -quat[1], -quat[2], -quat[3] };
    quat_rotate_vector(quat_conj, v, v_out);
}


void quat_to_euler(const MEKF *params , float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float w = params->q[0];
    float x = params->q[1];
    float y = params->q[2];
    float z = params->q[3];

    float t0 = 2.0f*(w*x + y*z);
    float t1 = 1.0f - 2.0f*(x*x + y*y);
    float t2 = 2.0f*(w*y - z*x);
    float t3 = 2.0f*(w*z + x*y);
    float t4 = 1.0f - 2.0f*(y*y + z*z);

    // clamp t2
    if (t2 > 1.0f) {
        t2 = 1.0f;
    }
    if (t2 < -1.0f) {
        t2 = -1.0f;
    }

    float roll = atan2f(t0, t1);
    float pitch = asinf(t2);
    float yaw = atan2f(t3, t4);

    *roll_deg = roll * 180.0f / M_PI;
    *pitch_deg = pitch * 180.0f / M_PI;
    *yaw_deg = yaw * 180.0f / M_PI;
}


void euler_to_quat(float roll, float pitch, float yaw, float q[4]) {
    float half_roll = roll*0.5f;
    float half_pitch = pitch*0.5f;
    float half_yaw = yaw*0.5f;

    float cos_r = cosf(half_roll);
    float sin_r = sinf(half_roll);
    float cos_p = cosf(half_pitch);
    float sin_p = sinf(half_pitch);
    float cos_y = cosf(half_yaw);
    float sin_y = sinf(half_yaw);

    q[0] = cos_r*cos_p*cos_y + sin_r*sin_p*sin_y;
    q[1] = sin_r*cos_p*cos_y - cos_r*sin_p*sin_y;
    q[2] = cos_r*sin_p*cos_y + sin_r*cos_p*sin_y;
    q[3] = cos_r*cos_p*sin_y - sin_r*sin_p*cos_y;
}


// ------------- MEKF
void mekf_init(MEKF *params, const sigma *noise) {
    // init quaternions
    params->q[0] = 1.0f;
    params->q[1] = 0.0f;
    params->q[2] = 0.0f;
    params->q[3] = 0.0f;

    // init matrices and vectors
    vec_zero3(params->gyro_bias);
    mat6_zero(params->P);
    mat6_zero(params->Q);
    memset(params->R_accel, 0, sizeof(params->R_accel));
    memset(params->R_mag, 0, sizeof(params->R_mag));

    // diag fill 
    for (int i=0; i<3; i++) {
        // initial error state covariance
        params->P[i][i] = noise->theta * noise->theta;
        params->P[i+3][i+3] = noise->bias * noise->bias;

        // process noise covariance
        params->Q[i][i] = noise->theta * noise->theta;
        params->Q[i+3][i+3] = noise->bias * noise->bias;

        // measurement noise 
        params->R_accel[i][i] = noise->accel * noise->accel;
        params->R_mag[i][i] = noise->mag * noise->mag;
    }
}


// Convert DCM → quaternion (w, x, y, z)
static void dcm_to_quat(const float R[3][3], float q[4]) {
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f) {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * S;
        q[1] = (R[2][1] - R[1][2]) / S;
        q[2] = (R[0][2] - R[2][0]) / S;
        q[3] = (R[1][0] - R[0][1]) / S;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        float S = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / S;
        q[1] = 0.25f * S;
        q[2] = (R[0][1] + R[1][0]) / S;
        q[3] = (R[0][2] + R[2][0]) / S;
    } else if (R[1][1] > R[2][2]) {
        float S = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / S;
        q[1] = (R[0][1] + R[1][0]) / S;
        q[2] = 0.25f * S;
        q[3] = (R[1][2] + R[2][1]) / S;
    } else {
        float S = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / S;
        q[1] = (R[0][2] + R[2][0]) / S;
        q[2] = (R[1][2] + R[2][1]) / S;
        q[3] = 0.25f * S;
    }
}

/**
 * Initialize quaternion from accel + mag readings.
 * accel: body-frame accel (gravity direction)
 * mag:   body-frame magnetometer (calibrated)
 * q:     output quaternion [w,x,y,z]
 * zero_yaw: if true, subtract initial yaw so startup heading = 0°
 */
void init_quat_from_accel_mag(const float accel[3], const float mag[3], float q[4], int zero_yaw) {
    float a[3] = { accel[0], accel[1], accel[2] };
    float m[3] = { mag[0], mag[1], mag[2] };

    vec_normalize3(a);
    vec_normalize3(m);

    // Z
    float z_body[3] = { a[0], a[1], a[2] };

    // Y
    float y_body[3];
    cross3(z_body, m, y_body);
    vec_normalize3(y_body);

    // X
    float x_body[3];
    cross3(y_body, z_body, x_body);

    // DCM: FRD
    float R[3][3] = {
        { x_body[0], y_body[0], z_body[0] },
        { x_body[1], y_body[1], z_body[1] },
        { x_body[2], y_body[2], z_body[2] }
    };

    dcm_to_quat(R, q);
    quat_normalize(q);

    if (zero_yaw) {
        // Compute yaw angle from quaternion
        float w = q[0], x = q[1], y = q[2], z = q[3];
        float yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

        // Build a rotation about Z by -yaw
        float half = -0.5f * yaw;
        float qz[4] = { cosf(half), 0.0f, 0.0f, sinf(half) };

        // Multiply: q = q * qz
        float qtmp[4];
        qtmp[0] = q[0]*qz[0] - q[1]*qz[1] - q[2]*qz[2] - q[3]*qz[3];
        qtmp[1] = q[0]*qz[1] + q[1]*qz[0] + q[2]*qz[3] - q[3]*qz[2];
        qtmp[2] = q[0]*qz[2] - q[1]*qz[3] + q[2]*qz[0] + q[3]*qz[1];
        qtmp[3] = q[0]*qz[3] + q[1]*qz[2] - q[2]*qz[1] + q[3]*qz[0];

        memcpy(q, qtmp, sizeof(qtmp));
        quat_normalize(q);
    }
}


void mekf_predict(MEKF *params, const float gyro[3], float dt) {
    float omega[3];

    vec_normalize3(gyro);

    for (int i=0; i<3; i++) {
        omega[i] = (gyro[i] - params->gyro_bias[i]) * (M_PI/180.0f);
    }

    float dq[4];
    float omega_norm = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);

    if (omega_norm > 1e-6f) {
        float half_angle = 0.5f * omega_norm * dt;
        float sin_half = sinf(half_angle);
        float cos_half = cosf(half_angle);

        dq[0] = cos_half;
        dq[1] = sin_half * omega[0]/omega_norm;
        dq[2] = sin_half * omega[1]/omega_norm;
        dq[3] = sin_half * omega[2]/omega_norm;
    } else {  
        // small angle approx
        dq[0] = 1.0f;
        dq[1] = 0.5f*omega[0]*dt;
        dq[2] = 0.5f*omega[1]*dt;
        dq[3] = 0.5f*omega[2]*dt;
    }

    float qtmp[4];
    quat_mult(params->q, dq, qtmp);
    memcpy(params->q, qtmp, sizeof(qtmp));
    quat_normalize(params->q);

    // float skew_mat[3][3];
    // skew3(omega, skew_mat);

    // discrete state transition matrix: Phi = I + F*dt
    float Phi[6][6];
    mat6_eye(Phi);
    for (int i=0; i<3; i++) {
        Phi[i][i+3] = -dt;
    }

     // Covariance propagation: P = Phi*P*Phi' + Q*dt
    float PhiP[6][6], Phi_transpose[6][6], PhiPPhi[6][6], Q_dt[6][6];
    
    mat6_mult(Phi, params->P, PhiP);
    mat6_transpose(Phi, Phi_transpose);
    mat6_mult(PhiP, Phi_transpose, PhiPPhi);
    
    mat6_scale(params->Q, dt, Q_dt);
    mat6_add(PhiPPhi, Q_dt, params->P);
}



void mekf_update(MEKF *params, const float meas_body[3], const float ref_inertial[3], const float R[3][3]) {
    vec_normalize3(meas_body);

    float v_pred[3];
    quat_rotate_vector_inverse(params->q, ref_inertial, v_pred);
    
    float y[3];
    y[0] = meas_body[0] - v_pred[0];
    y[1] = meas_body[1] - v_pred[1];
    y[2] = meas_body[2] - v_pred[2];

    float H[3][6];
    memset(H, 0, sizeof(H));

    float skew_mat[3][3];
    skew3(v_pred, skew_mat);

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            H[i][j] = -skew_mat[i][j];
        }
    }

    // S = H*P*H' + R
    float S[3][3];
    float H_transpose[6][3], HP[3][6], HPH_transpose[3][3];
    mat3x6_mult_6x6(H, params->P, HP);
    mat3x6_transpose(H, H_transpose);
    mat3x6_mult_6x3(HP, H_transpose, HPH_transpose);
    mat3_add(HPH_transpose, R, S);


    float S_inv[3][3];
    inv3(S, S_inv);

    // K = P*H'*inv(S)
    float PH_transpose[6][3], K[6][3];
    mat6x6_mult_6x3(params->P, H_transpose, PH_transpose);
    mat6x3_mult_3x3(PH_transpose, S_inv, K);

    // dx = K*y
    float dx[6], dq[4];
    mat6x3_mult_vec(K, y, dx);


    float dtheta[3] = { dx[0], dx[1], dx[2] };
    float theta_norm = sqrt( dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2] );

    if (theta_norm > 1e-6f) {
        float half_angle = 0.5f * theta_norm;
        float sin_half = sinf(half_angle);
        float cos_half = cosf(half_angle);

        dq[0] = cos_half;
        dq[1] = sin_half * dtheta[0]/theta_norm;
        dq[2] = sin_half * dtheta[1]/theta_norm;
        dq[3] = sin_half * dtheta[2]/theta_norm;
    } else {  
        // small angle approx
        dq[0] = 1.0f;
        dq[1] = 0.5f*dtheta[0];
        dq[2] = 0.5f*dtheta[1];
        dq[3] = 0.5f*dtheta[2];
    }

    float qtmp[4];
    quat_mult(params->q, dq, qtmp);
    memcpy(params->q, qtmp, sizeof(qtmp));
    quat_normalize(params->q);

    // apply bias correction
    for (int i=0; i<3; i++) {
        params->gyro_bias[i] += dx[i+3];
    }

    // covariance update: P = (I - KH)P
    //consider joseph form for numerical stability
    // joseph form: P = (I - KH)P(I-KH)' + KRK'
    float KH[6][6], I_KH[6][6];
    mat6x3_mult_3x6(K, H, KH);
    mat6_eye(I_KH);
    mat6_subtract(I_KH, KH, I_KH);

    float P_new[6][6];
    mat6_mult(I_KH, params->P, P_new);
    mat6_make_symmetric(P_new);
    memcpy(params->P, P_new, sizeof(params->P));
}


