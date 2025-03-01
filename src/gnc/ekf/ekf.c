#include "ekf.h"
#include "gnc/altimetry/altimetry.h"
#include "gnc/gnc_lib/gnc_lib.h"
#include "gnc/test_utils/test_utils.h"

/**
 * @brief get state-transition matrix for the position EKF
 *
 * Compute the state-transition matrix for the position Kalman filter from
 * time step (assumes constant acceleration/attitude dynamics over dt).
 * @param dt time step (s)
 * @param phi pointer to arm matrix instance to store computed STM
 */
enum ekf_err ekf_get_stm(float64_t dt, arm_matrix_instance_f64 *phi) {
  memset(phi->pData, 0, EKF_COV_MATRIX_SIZE * sizeof(float64_t));

  phi->pData[0] = 1.0;
  phi->pData[7] = 1.0;
  phi->pData[14] = 1.0;
  phi->pData[21] = 1.0;
  phi->pData[28] = 1.0;
  phi->pData[35] = 1.0;

  phi->pData[3] = dt;
  phi->pData[10] = dt;
  phi->pData[17] = dt;

  return EKF_ERR_OK;
}

/**
 * @brief get the sensitivity matrix for the position EKF
 *
 * Compute the measurement sensitivity matrix for the position Kalman filter
 * from estimted position in the NED frame.
 * @param r_enu pointer to array with estimated position in NED frame (m)
 * @param h_matrix pointer to arm matrix instance to store computed sensitivity
 * matrix
 */
enum ekf_err ekf_get_sens(float64_t *r_enu, arm_matrix_instance_f64 *h_matrix) {
  memset(h_matrix->pData, 0, EKF_MEAS_SENS_SIZE * sizeof(float64_t));

  h_matrix->pData[0] = 1.0;
  h_matrix->pData[7] = 1.0;
  h_matrix->pData[14] = 1.0;

  uint8_t atm_layer = isa_get_layer(-1.0 * r_enu[2]);
  float64_t baro_sens =
      isa_sensitivity_from_alt(atm_layer, r_enu[2] * -1.0) * -1.0;

  h_matrix->pData[20] = baro_sens;

  return EKF_ERR_OK;
}

/**
 * @brief get the measurement residual for the position EKF
 *
 * Given estimated position and measured position and barometric pressure,
 * determine the measurement residual yr = y - yhat
 * @param r_enu pointer to array with estimated position in NED frame (m)
 * @param r_ned_meas pointer array with measured positin in NED frame (m)
 * @param baro_meas measured barometric pressure (Pa)
 * @param residual pointer to arm matrix instance to store computed measurement
 * residual
 */
enum ekf_err ekf_get_residual(float64_t *r_enu, float64_t *r_ned_meas,
                              float64_t baro_meas,
                              arm_matrix_instance_f64 *residual) {
  residual->pData[0] = r_ned_meas[0] - r_enu[0];
  residual->pData[1] = r_ned_meas[1] - r_enu[1];
  residual->pData[2] = r_ned_meas[2] - r_enu[2];

  uint8_t atm_layer = isa_get_layer(r_enu[2] * -1.0);
  float64_t baro_expected = isa_pressure_from_alt(atm_layer, r_enu[2] * -1.0);

  residual->pData[3] = baro_meas - baro_expected;

  return EKF_ERR_OK;
}
/**
 * @brief update step for the position EKF
 *
 * Given measured NED position and barometric pressure, updat the estimated
 * state and covariance. This function updates the following elements of the EKF
 * struct:
 *  - ekf.r_ned
 *  - ekf.v_ned
 *  - ekf.covariance
 * The function requires the above elements to be allocated in the EKF struct,
 * in addition to:
 *  - ekf.measurement_covariance
 * @param ekf pointer to an EKF instance structure
 * @param r_ned_meas pointer to measured NED position (m)
 * @param baro_meas measured barometric pressure (Pa)
 */
enum ekf_err ekf_update(struct ekf_instance *ekf, float64_t *r_ned_meas,
                        float64_t baro_meas) {
  // Initialize intermediate matrices
  arm_matrix_instance_f64 temp_mat_1;
  float64_t temp_mat_1_buf[EKF_MEAS_SENS_SIZE];
  temp_mat_1.numCols = EKF_MEAS_SENS_ROWS;
  temp_mat_1.numRows = EKF_MEAS_SENS_COLS;
  temp_mat_1.pData = temp_mat_1_buf;

  arm_matrix_instance_f64 temp_mat_2;
  float64_t temp_mat_2_buf[16];
  temp_mat_2.numCols = EKF_MEAS_SENS_ROWS;
  temp_mat_2.numRows = EKF_MEAS_SENS_ROWS;
  temp_mat_2.pData = temp_mat_2_buf;

  arm_matrix_instance_f64 temp_mat_3;
  float64_t temp_mat_3_buf[16];
  temp_mat_3.numCols = EKF_MEAS_SENS_ROWS;
  temp_mat_3.numRows = EKF_MEAS_SENS_ROWS;
  temp_mat_3.pData = temp_mat_3_buf;

  arm_matrix_instance_f64 temp_cov_1;
  float64_t temp_cov_1_buf[EKF_COV_MATRIX_SIZE];
  temp_cov_1.numCols = EKF_COV_MATRIX_COLS;
  temp_cov_1.numRows = EKF_COV_MATRIX_ROWS;
  temp_cov_1.pData = temp_cov_1_buf;

  arm_matrix_instance_f64 temp_cov_2;
  float64_t temp_cov_2_buf[EKF_COV_MATRIX_SIZE];
  temp_cov_2.numCols = EKF_COV_MATRIX_COLS;
  temp_cov_2.numRows = EKF_COV_MATRIX_ROWS;
  temp_cov_2.pData = temp_cov_2_buf;

  arm_matrix_instance_f64 temp_cov_3;
  float64_t temp_cov_3_buf[EKF_COV_MATRIX_SIZE];
  temp_cov_3.numCols = EKF_COV_MATRIX_COLS;
  temp_cov_3.numRows = EKF_COV_MATRIX_ROWS;
  temp_cov_3.pData = temp_cov_3_buf;

  // IInitialize temporary vector (matrix)
  arm_matrix_instance_f64 temp_vec;
  float64_t temp_vec_buf[EKF_NUM_STATES];
  temp_vec.numCols = 1;
  temp_vec.numRows = EKF_NUM_STATES;
  temp_vec.pData = temp_vec_buf;

  // ========== Compute measurement residual ========== //
  arm_matrix_instance_f64 residual_vec;
  float64_t residual_vec_buf[EKF_MEAS_SIZE];
  residual_vec.numCols = EKF_MEAS_COLS;
  residual_vec.numRows = EKF_MEAS_ROWS;
  residual_vec.pData = residual_vec_buf;
  ekf_get_residual(ekf->r_enu, r_ned_meas, baro_meas, &residual_vec);

  // ========== Compute Kalman Gain ========== //
  // Compute measurement sensitivity
  arm_matrix_instance_f64 meas_sens;
  float64_t meas_sens_buf[EKF_MEAS_SENS_SIZE];
  meas_sens.numCols = EKF_MEAS_SENS_COLS;
  meas_sens.numRows = EKF_MEAS_SENS_ROWS;
  meas_sens.pData = meas_sens_buf;
  ekf_get_sens(ekf->r_enu, &meas_sens);

  // Compute H transpose
  arm_matrix_instance_f64 meas_sens_trans;
  float64_t meas_sens_trans_buf[EKF_MEAS_SENS_SIZE];
  meas_sens_trans.numCols = EKF_MEAS_SENS_ROWS;
  meas_sens_trans.numRows = EKF_MEAS_SENS_COLS;
  meas_sens_trans.pData = meas_sens_trans_buf;
  arm_mat_trans_f64(&meas_sens, &meas_sens_trans);

  // Compute P*HT: temp_mat_1 --> P*HT
  arm_mat_mult_f64(ekf->covariance, &meas_sens_trans, &temp_mat_1);

  // Compute H*P*HT: temp_mat_2 --> H*P*HT
  arm_mat_mult_f64(&meas_sens, &temp_mat_1, &temp_mat_2);

  // Compute H*P*HT + R: temp_mat_3 --> H*P*HT + R
  vec_add(temp_mat_2.pData, ekf->measurement_covariance->pData, 16,
          temp_mat_3.pData);

  // Compute inv(H*P*HT + R): temp_mat_2 --> inv(H*P*HT + R)
  arm_mat_inverse_f64(&temp_mat_3, &temp_mat_2);

  // Compute HT*inv(H*P*HT + R): temp_mat_1 --> HT*inv(H*P*HT + R)
  arm_mat_mult_f64(&meas_sens_trans, &temp_mat_2, &temp_mat_1);

  // Compute K = P*HT*inv(H*P*HT + R): meas_sens_trans --> K
  arm_mat_mult_f64(ekf->covariance, &temp_mat_1, &meas_sens_trans);

  // ========== Calculate correction ========== //
  // Calculate delta-x vector: temp_vec --> delta-x
  arm_mat_mult_f64(&meas_sens_trans, &residual_vec, &temp_vec);

  // Update position
  ekf->r_enu[0] += temp_vec.pData[0];
  ekf->r_enu[1] += temp_vec.pData[1];
  ekf->r_enu[2] += temp_vec.pData[2];

  // Update velocity
  ekf->v_enu[0] += temp_vec.pData[3];
  ekf->v_enu[1] += temp_vec.pData[4];
  ekf->v_enu[2] += temp_vec.pData[5];

  // ========== Update Covariance ========== //
  // Compute K*H: temp_cov_1 --> K*H
  arm_mat_mult_f64(&meas_sens_trans, &meas_sens, &temp_cov_1);

  // Compute eye(6): temp_cov_2 --> eye(6)
  memset(temp_cov_2.pData, 0.0, EKF_COV_MATRIX_SIZE * sizeof(float64_t));
  temp_cov_2.pData[0] = 1.0;
  temp_cov_2.pData[7] = 1.0;
  temp_cov_2.pData[14] = 1.0;
  temp_cov_2.pData[21] = 1.0;
  temp_cov_2.pData[28] = 1.0;
  temp_cov_2.pData[35] = 1.0;

  // Compute eye(6) - K*H: temp_cov_3 --> eye(6) - K*H
  arm_mat_sub_f64(&temp_cov_2, &temp_cov_1, &temp_cov_3);

  // Copute P+ = (eye(6) - K*H)*P-: temp_cov_1 --> P+
  arm_mat_mult_f64(&temp_cov_3, ekf->covariance, &temp_cov_1);

  // Move updated covariance to structure
  memcpy(ekf->covariance->pData, temp_cov_1.pData,
         sizeof(float64_t) * EKF_COV_MATRIX_SIZE);

  return EKF_ERR_OK;
}

/**
 * @brief prediction step for the position EKF
 *
 * Given measured body-frame accelration and time since last prediction or
 * update, propagate the estimated position and velocity. This function updates
 * the following elements of the EKF struct:
 *  - ekf.r_enu
 *  - ekf.v_enu
 *  - ekf.covariance
 * The function requires the above elements to be allocated in the EKF struct,
 * in addition to the following:
 *  - ekf.process_noise
 * @param ekf pointer to an EKF instance structure
 * @param accel pointer to a vector of measured body-frame accelerations (m/s^2)
 * @param attitude_matrix matrix pointer to an arm matrix instance representing
 * vehicle attitude
 * @param dt time step (s)
 */
enum ekf_err ekf_predict(struct ekf_instance *ekf, float64_t *accel,
                         arm_matrix_instance_f64 *attitude_matrix,
                         float64_t dt) {
  // Calculate body-to-inertial rotation matrix
  arm_matrix_instance_f64 rotation_body_to_inertial;
  float64_t rotation_body_to_inertial_buf[9];
  rotation_body_to_inertial.numCols = 3;
  rotation_body_to_inertial.numRows = 3;
  rotation_body_to_inertial.pData = rotation_body_to_inertial_buf;
  arm_mat_trans_f64(attitude_matrix, &rotation_body_to_inertial);

  // Calculate inertial acceleration
  float64_t accel_inertial[3];
  apply_rotation(&rotation_body_to_inertial, accel, accel_inertial);
  vec_add(accel_inertial, ekf->g_inertial, 3, accel_inertial);

  // Square the time step
  float64_t dt2 = dt * dt;

  // Update position estimate
  ekf->r_enu[0] = ekf->r_enu[0] + ekf->v_enu[0] * dt + accel_inertial[0] * dt2;
  ekf->r_enu[1] = ekf->r_enu[1] + ekf->v_enu[1] * dt + accel_inertial[1] * dt2;
  ekf->r_enu[2] = ekf->r_enu[2] + ekf->v_enu[2] * dt + accel_inertial[2] * dt2;

  // Update velocity estimate
  ekf->v_enu[0] = ekf->v_enu[0] + accel_inertial[0] * dt;
  ekf->v_enu[1] = ekf->v_enu[1] + accel_inertial[1] * dt;
  ekf->v_enu[2] = ekf->v_enu[2] + accel_inertial[2] * dt;

  // ========== Update covariance estimate ========== //
  // Allocate temporary matrices
  arm_matrix_instance_f64 temp_mat_1;
  float64_t temp_mat_1_buf[EKF_COV_MATRIX_SIZE];
  temp_mat_1.numCols = EKF_COV_MATRIX_COLS;
  temp_mat_1.numRows = EKF_COV_MATRIX_ROWS;
  temp_mat_1.pData = temp_mat_1_buf;

  arm_matrix_instance_f64 temp_mat_2;
  float64_t temp_mat_2_buf[EKF_COV_MATRIX_SIZE];
  temp_mat_2.numCols = EKF_COV_MATRIX_COLS;
  temp_mat_2.numRows = EKF_COV_MATRIX_ROWS;
  temp_mat_2.pData = temp_mat_2_buf;

  // Allocate state transition matrix
  arm_matrix_instance_f64 phi;
  float64_t phi_buf[EKF_COV_MATRIX_SIZE];
  phi.numCols = EKF_COV_MATRIX_COLS;
  phi.numRows = EKF_COV_MATRIX_ROWS;
  phi.pData = phi_buf;

  // Compute state-transition matrix
  ekf_get_stm(dt, &phi);

  // Compute phiT: temp_mat_1 --> phiT
  arm_mat_trans_f64(&phi, &temp_mat_1);

  // Compute P*PhiT: temp_mat_2 --> P*PhiT
  arm_mat_mult_f64(ekf->covariance, &temp_mat_1, &temp_mat_2);

  // Compute Phi*P*PhiT: temp_mat_1 --> Phi*P*PhiT
  arm_mat_mult_f64(&phi, &temp_mat_2, &temp_mat_1);

  // Compute Phi*P*PhiT + R: P --> Phi*P*PhiT + R
  vec_add(temp_mat_1.pData, ekf->process_noise->pData, EKF_COV_MATRIX_SIZE,
          ekf->covariance->pData);

  return EKF_ERR_OK;
}
