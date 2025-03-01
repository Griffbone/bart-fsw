#include "qmekf.h"
#include "gnc/gnc_lib/gnc_lib.h"

/**
 * @brief get state-transition matrix for the QMEKF
 *
 * Compute the state-transition matrix for the attitude Kalman filter given
 * angular rate and time step
 * @param omega_hat pointer to an array representing angular velocity with
 * estimated bias removed
 * @param dt time step
 * @param phi arm matrix instance to store computed STM
 */
enum qmekf_err qmekf_get_stm(float64_t *omega_hat, float64_t dt,
                             arm_matrix_instance_f64 *phi) {
  // Extract components of angular velocity
  float64_t wx = omega_hat[0];
  float64_t wy = omega_hat[1];
  float64_t wz = omega_hat[2];

  // Set all components to zero
  for (uint8_t i = 0; i < QMEKF_PHI_MATRIX_SIZE; i++) {
    phi->pData[i] = 0.0;
  }

  // Compute phi matrix and store in matrix data
  phi->pData[0] = 1.0;
  phi->pData[1] = dt * wz;
  phi->pData[2] = -dt * wy;
  phi->pData[3] = -dt;
  phi->pData[6] = -dt * wz;
  phi->pData[7] = 1.0;
  phi->pData[8] = dt * wx;
  phi->pData[10] = -dt;
  phi->pData[12] = dt * wy;
  phi->pData[13] = -dt * wx;
  phi->pData[14] = 1.0;
  phi->pData[17] = -dt;
  phi->pData[21] = 1.0;
  phi->pData[28] = 1.0;
  phi->pData[35] = 1.0;

  return QMEKF_ERR_OK;
}

/**
 * @brief get the sensitivity matrix for the QMEKF
 *
 * Compute the measurement sensitivity matrix for the attitude Kalman filter
 * given estimated quaternion and known inertial magnetic field/gravity vectors
 * @param q_hat pointer to an array representing quaternion
 * @param b_inertial pointer to an array representing inertial magnetic field
 * @param g_inertial pointer to an array representing inertial gravity vector
 * @param h_matrix pointer to arm matrix instance to store computed sensitivity
 * matrix
 */
enum qmekf_err qmekf_get_sens(arm_matrix_instance_f64 *attitude_matrix,
                              float64_t *b_inertial, float64_t *g_inertial,
                              arm_matrix_instance_f64 *h_matrix) {
  // Apply attitude matrix to inertial vectors
  float64_t b_body[3];
  float64_t g_body[3];
  apply_rotation(attitude_matrix, b_inertial, b_body);
  apply_rotation(attitude_matrix, g_inertial, g_body);

  // Generate skew-symmetric matrices
  float64_t b_body_skew[9];
  float64_t g_body_skew[9];
  skew_symmetric(b_body, b_body_skew);
  skew_symmetric(g_body, g_body_skew);

  // Put into the h matrix instance
  h_matrix->pData[0] = b_body_skew[0];
  h_matrix->pData[1] = b_body_skew[1];
  h_matrix->pData[2] = b_body_skew[2];
  h_matrix->pData[3] = 0.0;
  h_matrix->pData[4] = 0.0;
  h_matrix->pData[5] = 0.0;

  h_matrix->pData[6] = b_body_skew[3];
  h_matrix->pData[7] = b_body_skew[4];
  h_matrix->pData[8] = b_body_skew[5];
  h_matrix->pData[9] = 0.0;
  h_matrix->pData[10] = 0.0;
  h_matrix->pData[11] = 0.0;

  h_matrix->pData[12] = b_body_skew[6];
  h_matrix->pData[13] = b_body_skew[7];
  h_matrix->pData[14] = b_body_skew[8];
  h_matrix->pData[15] = 0.0;
  h_matrix->pData[16] = 0.0;
  h_matrix->pData[17] = 0.0;

  h_matrix->pData[18] = g_body_skew[0];
  h_matrix->pData[19] = g_body_skew[1];
  h_matrix->pData[20] = g_body_skew[2];
  h_matrix->pData[21] = 0.0;
  h_matrix->pData[22] = 0.0;
  h_matrix->pData[23] = 0.0;

  h_matrix->pData[24] = g_body_skew[3];
  h_matrix->pData[25] = g_body_skew[4];
  h_matrix->pData[26] = g_body_skew[5];
  h_matrix->pData[27] = 0.0;
  h_matrix->pData[28] = 0.0;
  h_matrix->pData[29] = 0.0;

  h_matrix->pData[30] = g_body_skew[6];
  h_matrix->pData[31] = g_body_skew[7];
  h_matrix->pData[32] = g_body_skew[8];
  h_matrix->pData[33] = 0.0;
  h_matrix->pData[34] = 0.0;
  h_matrix->pData[35] = 0.0;

  return QMEKF_ERR_OK;
}

/**
 * @brief measurement residual for the QMEKF
 *
 * Given expected inertial gravity and magnetometer values, current attitude
 * estimate, and measured body-frame values, determine the measurement residual
 * yr = y - y_hat
 * @param attitude_matrix pointer to arm matrix storing current attitude matrix
 * estimate
 * @param b_inertial pointer to array representing expected inertial magnetic
 * field vector
 * @param g_inertial pointer to array representing expected inertial gravity
 * vector
 * @param b_meas pointer to aray with current body-frame magnetic field
 * measurement
 * @param g_meas pointer to array with current body-frame gravity measurement
 * @param residual pointer to arm matrix to store measurement residual
 */
enum qmekf_err qmekf_get_residual(arm_matrix_instance_f64 *attitude_matrix,
                                  float64_t *b_inertial, float64_t *g_inertial,
                                  float64_t *b_meas, float64_t *g_meas,
                                  arm_matrix_instance_f64 *residual) {
  // Apply attitude matrix to inertial vectors
  float64_t b_body[3];
  float64_t g_body[3];
  apply_rotation(attitude_matrix, b_inertial, b_body);
  apply_rotation(attitude_matrix, g_inertial, g_body);

  // Calculate residual
  residual->pData[0] = b_meas[0] - b_body[0];
  residual->pData[1] = b_meas[1] - b_body[1];
  residual->pData[2] = b_meas[2] - b_body[2];
  residual->pData[3] = g_meas[0] - g_body[0];
  residual->pData[4] = g_meas[1] - g_body[1];
  residual->pData[5] = g_meas[2] - g_body[2];

  return QMEKF_ERR_OK;
}

/**
 * @brief update step for the QMEKF
 *
 * Given measured magnetic field and gravitational acceleration, update the
 * state estimate and covariance matrices in in the QMEKF. This function updates
 * the following elements of the QMEKF struct:
 *  - qmekf.attitude_matrix
 *  - qmekf.quaternion
 *  - qmekf.gyro_bias
 *  - qmekf.covariance
 * The function requires the above information to be populated in the Kalman
 * filter struct, in addition to the following:
 *  - qmekf.measurement_covariance
 *  - qmekf.mag_vec_inertial
 *  - qmekf.g_vec_inertial
 * @param qmekf pointer to a qmekf structure
 * @param b_meas measured magnetic field (same units as the inertial field)
 * @param g_meas measured gravity vector (same units as the inertial vector)
 */
enum qmekf_err qmekf_update(struct qmekf_instance *qmekf, float64_t *b_meas,
                            float64_t *g_meas) {
  // Initialize intermediate matrices
  arm_matrix_instance_f64 temp_mat_1;
  float64_t temp_mat_1_buf[36];
  temp_mat_1.numCols = 6;
  temp_mat_1.numRows = 6;
  temp_mat_1.pData = temp_mat_1_buf;

  arm_matrix_instance_f64 temp_mat_2;
  float64_t temp_mat_2_buf[36];
  temp_mat_2.numCols = 6;
  temp_mat_2.numRows = 6;
  temp_mat_2.pData = temp_mat_2_buf;

  // Initialize measurement residual matrix (vector) and temporary matrix
  // (vector)
  arm_matrix_instance_f64 temp_vec;
  float64_t temp_vec_buf[6];
  temp_vec.numCols = 1;
  temp_vec.numRows = 6;
  temp_vec.pData = temp_vec_buf;

  // ========== Compute measurement residual ========== //
  arm_matrix_instance_f64 residual_vec;
  float64_t residual_vec_buf[6];
  residual_vec.numCols = 1;
  residual_vec.numRows = 6;
  residual_vec.pData = residual_vec_buf;
  qmekf_get_residual(qmekf->attitude_matrix, qmekf->mag_vec_inertial,
                     qmekf->g_vec_inertial, b_meas, g_meas, &residual_vec);

  // ========== Compute Kalman Gain ========== //
  // Compute measurement sensitivity
  arm_matrix_instance_f64 meas_sens;
  float64_t meas_sens_buf[36];
  meas_sens.numCols = 6;
  meas_sens.numRows = 6;
  meas_sens.pData = meas_sens_buf;
  qmekf_get_sens(qmekf->attitude_matrix, qmekf->mag_vec_inertial,
                 qmekf->g_vec_inertial, &meas_sens);

  // Compute H transpose
  arm_matrix_instance_f64 meas_sens_trans;
  float64_t meas_sens_trans_buf[36];
  meas_sens_trans.numCols = 6;
  meas_sens_trans.numRows = 6;
  meas_sens_trans.pData = meas_sens_trans_buf;
  arm_mat_trans_f64(&meas_sens, &meas_sens_trans);

  // Compute P*HT: temp_mat_1 --> P*HT
  arm_mat_mult_f64(qmekf->covariance, &meas_sens_trans, &temp_mat_1);

  // Compute H*P*HT: temp_mat_2 --> H*P*HT
  arm_mat_mult_f64(&meas_sens, &temp_mat_1, &temp_mat_2);

  // Compute H*P*HT + R: temp_mat_1 --> H*P*HT + R
  vec_add(temp_mat_2.pData, qmekf->measurement_covariance->pData,
          QMEKF_PHI_MATRIX_SIZE, temp_mat_1.pData);

  // Compute inv(H*P*HT + R): temp_mat_2 --> inv(H*P*HT + R)
  arm_mat_inverse_f64(&temp_mat_1, &temp_mat_2);

  // Compute HT*inv(H*P*HT + R): temp_mat_1 --> HT*inv(H*P*HT + R)
  arm_mat_mult_f64(&meas_sens_trans, &temp_mat_2, &temp_mat_1);

  // Compute K = P*HT*inv(H*P*HT + R): temp_mat_2 --> K
  arm_mat_mult_f64(qmekf->covariance, &temp_mat_1, &temp_mat_2);

  // ========== Calculate correction ========== //
  // Calculate delta-x vector: temp_vec --> delta-x
  arm_mat_mult_f64(&temp_mat_2, &residual_vec, &temp_vec);

  // Calculate error quaternion
  float64_t delta_q[4];
  delta_q[0] = temp_vec.pData[0] / 2.0;
  delta_q[1] = temp_vec.pData[1] / 2.0;
  delta_q[2] = temp_vec.pData[2] / 2.0;
  delta_q[3] = 1.0;

  // Update quaternion
  float64_t temp_q[4];
  memcpy(temp_q, qmekf->quaternion, sizeof(float64_t) * 4);

  quaternion_multiplication(delta_q, temp_q, qmekf->quaternion);

  quaternion_norm(qmekf->quaternion, qmekf->quaternion);

  // Update bias
  qmekf->gyro_bias[0] += temp_vec.pData[3];
  qmekf->gyro_bias[1] += temp_vec.pData[4];
  qmekf->gyro_bias[2] += temp_vec.pData[5];

  // Update attitude matrix
  quaternion_to_rotation(qmekf->quaternion, qmekf->attitude_matrix);

  // ========== Update Covariance ========== //
  // Compute K*H: temp_mat_1 --> K*H
  arm_mat_mult_f64(&temp_mat_2, &meas_sens, &temp_mat_1);

  // Compute eye(6): temp_mat_2 --> eye(6)
  memset(temp_mat_2.pData, 0.0, QMEKF_PHI_MATRIX_SIZE * sizeof(float64_t));
  temp_mat_2.pData[0] = 1.0;
  temp_mat_2.pData[7] = 1.0;
  temp_mat_2.pData[14] = 1.0;
  temp_mat_2.pData[21] = 1.0;
  temp_mat_2.pData[28] = 1.0;
  temp_mat_2.pData[35] = 1.0;

  // Compute eye(6) - K*H: meas_sens --> eye(6) - K*H
  arm_mat_sub_f64(&temp_mat_2, &temp_mat_1, &meas_sens);

  // Copute P+ = (eye(6) - K*G)*P-: temp_mat_1 --> P+
  arm_mat_mult_f64(&meas_sens, qmekf->covariance, &temp_mat_1);

  // Move updated covariance to structure
  memcpy(qmekf->covariance->pData, temp_mat_1.pData,
         sizeof(float64_t) * QMEKF_PHI_MATRIX_SIZE);

  return QMEKF_ERR_OK;
}

/**
 * @brief prediction step for the QMEKF
 *
 * Given measured body-frame angular rates and time since last prediction or
 * update, propagate the estimated quaternion, biases, and covariances. The
 * function updates the following elements ofthe QMEKF structl
 *  - qmekf.quaternion
 *  - qmekf.covariance
 *  - qmekf.attitude_matrix
 * The function requires the above information to be populated in the QMEKF
 * struct, in addition to the following:
 *  - qmekf.process_noise
 *  - qmekf.gyro_bias
 * @param qmekf pointer to a qmekf structure
 * @param omega_meas measured body-frame angular velocity (rad/s)
 * @param dt time step (s)
 */
enum qmekf_err qmekf_predict(struct qmekf_instance *qmekf,
                             float64_t *omega_meas, float64_t dt) {
  // Initialize intermediate matrices
  arm_matrix_instance_f64 temp_mat_1;
  float64_t temp_mat_1_buf[36];
  temp_mat_1.numCols = 6;
  temp_mat_1.numRows = 6;
  temp_mat_1.pData = temp_mat_1_buf;

  arm_matrix_instance_f64 temp_mat_2;
  float64_t temp_mat_2_buf[36];
  temp_mat_2.numCols = 6;
  temp_mat_2.numRows = 6;
  temp_mat_2.pData = temp_mat_2_buf;

  // Calculte estimated angular rate omega_meas - bias_hat
  float64_t omega_hat[3];
  vec_subtract(omega_meas, qmekf->gyro_bias, 3, omega_hat);

  // ========== Propagate quaternion ========== //
  float64_t temp_quat[4];
  temp_quat[0] = qmekf->quaternion[0];
  temp_quat[1] = qmekf->quaternion[1];
  temp_quat[2] = qmekf->quaternion[2];
  temp_quat[3] = qmekf->quaternion[3];
  quaternion_propagate_omega(temp_quat, omega_hat, dt, qmekf->quaternion);

  // Update attitude matrix
  quaternion_to_rotation(qmekf->quaternion, qmekf->attitude_matrix);

  // ========== Compute state-transition matrix ========== //
  arm_matrix_instance_f64 phi;
  float64_t phi_buf[36];
  phi.numCols = 6;
  phi.numRows = 6;
  phi.pData = phi_buf;
  qmekf_get_stm(omega_hat, dt, &phi);

  // ========== Update Covariance ========== //
  // Compute PhiT: temp_mat_1 --> PhiT
  arm_mat_trans_f64(&phi, &temp_mat_1);

  // Comptue P*PhiT: temp_mat
  arm_mat_mult_f64(qmekf->covariance, &temp_mat_1, &temp_mat_2);

  // Compute Phi*(P*PhiT): temp_mat_1 --> Phi*(P*PhiT)
  arm_mat_mult_f64(&phi, &temp_mat_2, &temp_mat_1);

  // Compute P_prior = Phi*(P*PhiT) + G*Q*GT
  vec_add(temp_mat_1.pData, qmekf->process_noise->pData, QMEKF_PHI_MATRIX_SIZE,
          qmekf->covariance->pData);

  return QMEKF_ERR_OK;
}