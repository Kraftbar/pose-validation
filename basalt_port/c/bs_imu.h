/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt IMU preintegration (basalt-headers IntegratedImuMeasurement<float>, imu_types.h PoseVelState, and the IMU part of
 * basalt/linearization/imu_block.hpp ImuBlock::linearizeImu), C99, bit-exact against g++ -O2 -ffp-contract=off -fno-fast-math
 * (SSE2, no FMA, Eigen 3.4.0, Sophus 1.24.6). Scalar = float: the estimator runs SqrtKeypointVioEstimator<float> (PLAN.md 4a);
 * IMU samples are cast double -> float in popFromImuDataQueue() before integrate().
 *
 * Executed path: integrate (36,810 calls per MH_01 run), predictState (once per frame pair), residual with all four Jacobians
 * (linearisation point) and without (current point), get_sqrt_cov_inv (lazy, the float `1/sqrtf(d)` copy, PLAN.md 4d), the
 * bias random-walk rows of linearizeImu. get_cov_inv(), the double instantiation and propagateState's extra callers are not on the path.
 *
 * Layouts: column-major like Eigen. Quaternions are Eigen coefficient order (x, y, z, w). 9x9 = pose(3) rot(3) vel(3).
 */
#ifndef BS_IMU_H
#define BS_IMU_H

#include <stdint.h>

typedef struct bs_pvstate {       /* PoseVelState<float> */
    int64_t t_ns;
    float q[4];                   /* T_w_i.so3() unit quaternion x y z w */
    float p[3];                   /* T_w_i.translation() */
    float v[3];                   /* vel_w_i */
} bs_pvstate;

typedef struct bs_pvbstate {      /* PoseVelBiasState<float> */
    bs_pvstate s;
    float bg[3];                  /* bias_gyro */
    float ba[3];                  /* bias_accel */
} bs_pvbstate;

typedef struct bs_pvb_with_lin {  /* PoseVelBiasStateWithLin<float>, only what linearizeImu reads */
    int linearized;
    bs_pvbstate lin;              /* getStateLin() */
    bs_pvbstate cur;              /* getState() when linearized (state_current) */
} bs_pvb_with_lin;

typedef struct bs_imudata {       /* ImuData<float> */
    int64_t t_ns;
    float accel[3];
    float gyro[3];
} bs_imudata;

typedef struct bs_imu_meas {      /* IntegratedImuMeasurement<float> */
    int64_t start_t_ns;
    bs_pvstate delta;             /* delta_state_ */
    float cov[81];                /* cov_ 9x9 */
    float sqrt_cov_inv[81];       /* sqrt_cov_inv_ 9x9 (valid when sqrt_cov_inv_computed) */
    int sqrt_cov_inv_computed;
    float d_state_d_ba[27];       /* 9x3 */
    float d_state_d_bg[27];       /* 9x3 */
    float bias_gyro_lin[3];
    float bias_accel_lin[3];
} bs_imu_meas;

/* IntegratedImuMeasurement() */
void bs_imu_init_default(bs_imu_meas* m);
/* IntegratedImuMeasurement(start_t_ns, bias_gyro_lin, bias_accel_lin) */
void bs_imu_init(bs_imu_meas* m, int64_t start_t_ns, const float bias_gyro_lin[3], const float bias_accel_lin[3]);

/* PoseVelState() : identity pose, zero velocity, t_ns 0 */
void bs_pvstate_default(bs_pvstate* s);

/* static propagateState; F (9x9), A (9x3, accel), G (9x3, gyro) may be NULL */
void bs_imu_propagate_state(const bs_pvstate* curr, const bs_imudata* data, bs_pvstate* next, float* F, float* A, float* G);

/* integrate(data, accel_cov, gyro_cov) (diagonals) */
void bs_imu_integrate(bs_imu_meas* m, const bs_imudata* data, const float accel_cov[3], const float gyro_cov[3]);

/* predictState: writes only so3, vel_w_i and translation of *state1 (the other members keep their value, as in C++) */
void bs_imu_predict_state(const bs_imu_meas* m, const bs_pvstate* state0, const float g[3], bs_pvstate* state1);

/* residual(...) -> res[9]; any Jacobian pointer may be NULL (d_res_d_state0/1: 9x9, d_res_d_bg/ba: 9x3) */
void bs_imu_residual(const bs_imu_meas* m, const bs_pvstate* state0, const float g[3], const bs_pvstate* state1,
                     const float curr_bg[3], const float curr_ba[3], float res[9],
                     float* d_res_d_state0, float* d_res_d_state1, float* d_res_d_bg, float* d_res_d_ba);

/* get_sqrt_cov_inv(): computes (compute_sqrt_cov_inv, float sqrt copy) on first use after integrate; returns m->sqrt_cov_inv (9x9) */
const float* bs_imu_sqrt_cov_inv(bs_imu_meas* m);

/* ImuBlock::linearizeImu: Jp (15x30, col-major) and r (15) of the IMU factor + bias random walk; returns imu_error + bg_error + ba_error.
 * g, gyro_bias_weight_sqrt, accel_bias_weight_sqrt = ImuLinData members. */
float bs_imu_linearize(bs_imu_meas* m, const float g[3], const float gyro_bias_weight_sqrt[3], const float accel_bias_weight_sqrt[3],
                       const bs_pvb_with_lin* start, const bs_pvb_with_lin* end, float Jp[450], float r[15]);

/* test hooks: Eigen LDLT<Matrix<float,9,9>> unblocked on a full 9x9 (lower triangle used; mat becomes matrixLDLT()), transpositions;
 * and triangular_solve_matrix<OnTheLeft, UnitLower> of a 9x9 right-hand side in place */
void bs_imu_dbg_gemm(int rows, int cols, int depth, const float* A, const float* B, float* out); /* column-major GEBP model */
void bs_imu_ldlt9(float mat[81], int trans[9]);
void bs_imu_trisolve_unit_lower9(const float L[81], float S[81]);

#endif
