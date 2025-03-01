#include "dsp/arm_math.h"
#include "gnc/ekf/ekf.h"
#include "gnc/test_utils/test_utils.h"

/**
 * @brief test ekf state transition matrix function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ekf_get_stm() {
  arm_matrix_instance_f64 stm_out;
  float64_t stm_out_buf[EKF_COV_MATRIX_SIZE];
  stm_out.numCols = EKF_COV_MATRIX_COLS;
  stm_out.numRows = EKF_COV_MATRIX_ROWS;
  stm_out.pData = stm_out_buf;

  // ========== Case 1 ========== //
  float64_t dt_1 = 1.0000e-03;
  float64_t stm_1_true_buf[EKF_COV_MATRIX_SIZE] = {
      1, 0, 0, 0.001, 0, 0, 0, 1, 0, 0, 0.001, 0, 0, 0, 1, 0, 0, 0.001,
      0, 0, 0, 1,     0, 0, 0, 0, 0, 0, 1,     0, 0, 0, 0, 0, 0, 1};
  ekf_get_stm(dt_1, &stm_out);
  if (!compare(stm_out.pData, stm_1_true_buf, EKF_COV_MATRIX_SIZE, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t dt_2 = 1;
  float64_t stm_2_true_buf[EKF_COV_MATRIX_SIZE] = {
      1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  ekf_get_stm(dt_2, &stm_out);
  if (!compare(stm_out.pData, stm_2_true_buf, EKF_COV_MATRIX_SIZE, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t dt_3 = 0;
  float64_t stm_3_true_buf[EKF_COV_MATRIX_SIZE] = {
      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  ekf_get_stm(dt_3, &stm_out);
  if (!compare(stm_out.pData, stm_3_true_buf, EKF_COV_MATRIX_SIZE, 1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test ekf sensitivity matrix function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ekf_get_sens() {
  arm_matrix_instance_f64 meas_sens;
  float64_t meas_sens_buf[EKF_MEAS_SENS_SIZE];
  meas_sens.numCols = EKF_MEAS_SENS_COLS;
  meas_sens.numRows = EKF_MEAS_SENS_ROWS;
  meas_sens.pData = meas_sens_buf;

  // ========== Case 1 ========== //
  float64_t r_enu_1[3] = {1000, -1500, 1000};
  float64_t meas_sens_1_true_buf[EKF_MEAS_SENS_SIZE] = {
      1, 0, 0, 0, 0, 0, 0, 1, 0,        0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 13.20952, 0, 0, 0};
  ekf_get_sens(r_enu_1, &meas_sens);
  if (!compare(meas_sens.pData, meas_sens_1_true_buf, EKF_MEAS_SENS_SIZE,
               1e-3)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t r_enu_2[3] = {1000, -1500, -5000};
  float64_t meas_sens_2_true_buf[EKF_MEAS_SENS_SIZE] = {
      1, 0, 0, 0, 0, 0, 0, 1, 0,        0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 7.218828, 0, 0, 0};
  ekf_get_sens(r_enu_2, &meas_sens);
  if (!compare(meas_sens.pData, meas_sens_2_true_buf, EKF_MEAS_SENS_SIZE,
               1e-3)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t r_enu_3[3] = {1000, -1500, -11000};
  float64_t meas_sens_3_true_buf[EKF_MEAS_SENS_SIZE] = {
      1, 0, 0, 0, 0, 0, 0, 1, 0,        0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 3.568813, 0, 0, 0};
  ekf_get_sens(r_enu_3, &meas_sens);
  if (!compare(meas_sens.pData, meas_sens_3_true_buf, EKF_MEAS_SENS_SIZE,
               1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test ekf residual vector function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ekf_get_residual() {
  arm_matrix_instance_f64 residual;
  float64_t residual_buf[EKF_MEAS_SIZE];
  residual.numCols = EKF_MEAS_COLS;
  residual.numRows = EKF_MEAS_ROWS;
  residual.pData = residual_buf;

  // ========== Case 1 ========== //
  float64_t r_enu_1[3] = {1482.369, 1009.19, -9237.851};
  float64_t r_enu_meas_1[3] = {1428.277, 757.2916, -9130.319};
  float64_t baro_meas_1 = 31573.13;
  float64_t residual_1_true_buf[EKF_MEAS_SIZE] = {-54.09193, -251.8982, 107.532,
                                                  1902.96};
  ekf_get_residual(r_enu_1, r_enu_meas_1, baro_meas_1, &residual);
  if (!compare(residual.pData, residual_1_true_buf, EKF_MEAS_SIZE, 0.1)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t r_enu_2[3] = {392.4172, 2383.833, -3053.826};
  float64_t r_enu_meas_2[3] = {271.623, 2174.098, -2967.232};
  float64_t baro_meas_2 = 70004.58;
  float64_t residual_2_true_buf[EKF_MEAS_SIZE] = {-120.7942, -209.7351,
                                                  86.59402, 374.5873};
  ekf_get_residual(r_enu_2, r_enu_meas_2, baro_meas_2, &residual);
  if (!compare(residual.pData, residual_2_true_buf, EKF_MEAS_SIZE, 0.1)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t r_enu_3[3] = {-82.13775, 334.5468, 156.4737};
  float64_t r_enu_meas_3[3] = {-112.6775, 32.08586, 107.7535};
  float64_t baro_meas_3 = {102589.9};
  float64_t residual_3_true_buf[EKF_MEAS_SIZE] = {-30.53974, -302.4609,
                                                  -48.72022, -628.9776};
  ekf_get_residual(r_enu_3, r_enu_meas_3, baro_meas_3, &residual);
  if (!compare(residual.pData, residual_3_true_buf, EKF_MEAS_SIZE, 0.1)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test setting measurement covariance from GPS covariance
 * @return 1 if all tests pass, 0 otherwise
 */
/*
uint8_t test_ekf_set_gps_covariance() {
  // Initialize EKF structure
  struct ekf_instance ekf;

  // Initialize EKF covariance
  arm_matrix_instance_f64 ekf_meas_cov;
  float64_t ekf_meas_cov_buf[EKF_MEAS_COV_MATRIX_SIZE] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000};
  ekf_meas_cov.numCols = EKF_MEAS_COV_MATRIX_COLS;
  ekf_meas_cov.numRows = EKF_MEAS_COV_MATRIX_ROWS;
  ekf_meas_cov.pData = ekf_meas_cov_buf;
  ekf.measurement_covariance = &ekf_meas_cov;

  // ========== Case 1 ========== //
  float32_t gps_covariance_buf[6] = {1.61, 3.075, 4.74, 9.014, 8.649, 0.972};
  float64_t meas_cov_true_buf[EKF_MEAS_COV_MATRIX_SIZE] = {
      1.61, 3.075, 4.74,  0, 3.075, 9.014, 8.649, 0,
      4.74, 8.649, 0.972, 0, 0,     0,     0,     1000};
  ekf_set_gps_covariance(&ekf, gps_covariance_buf);
  if (!compare(meas_cov_true_buf, ekf.measurement_covariance->pData,
               EKF_MEAS_COV_MATRIX_SIZE, 1e-6)) {
    return 0;
  }

  return 1;
}
*/

/**
 * @brief test ekf update function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ekf_update() {
  // ========== Initial EKF Instance ========== //
  float64_t r_enu[3] = {300, -10, -1000};
  float64_t v_enu[3] = {10, -130, -500};

  arm_matrix_instance_f64 covariance;
  float64_t covariance_buf[EKF_COV_MATRIX_SIZE] = {
      10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0,
      0,  0, 0, 1, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0, 0,  0, 0, 1};
  covariance.numCols = EKF_COV_MATRIX_COLS;
  covariance.numRows = EKF_COV_MATRIX_ROWS;
  covariance.pData = covariance_buf;

  arm_matrix_instance_f64 measurement_covariance;
  float64_t measurement_covariance_buf[EKF_MEAS_COV_MATRIX_SIZE] = {
      25, 0, 0, 0, 0, 25, 0, 0, 0, 0, 25, 0, 0, 0, 0, 10000};
  measurement_covariance.numCols = EKF_MEAS_COV_MATRIX_COLS;
  measurement_covariance.numRows = EKF_MEAS_COV_MATRIX_ROWS;
  measurement_covariance.pData = measurement_covariance_buf;

  struct ekf_instance ekf;
  ekf.r_enu = r_enu;
  ekf.v_enu = v_enu;
  ekf.covariance = &covariance;
  ekf.measurement_covariance = &measurement_covariance;

  // ========== Measurement ========== //
  float64_t r_enu_meas[3] = {360.8047, -23.07562, -1501.195};
  float64_t baro_meas = 84605.1060910608;

  // ========== Perform Update ========== //
  ekf_update(&ekf, r_enu_meas, baro_meas);

  // ========== Verify Update ========== //
  float64_t r_post_true_buf[3] = {317.3728, -13.73589, -1169.816};
  float64_t v_post_true_buf[3] = {10, -130, -500};
  float64_t cov_post_true_buf[EKF_COV_MATRIX_SIZE] = {
      7.142857, 0, 0,        0, 0, 0, 0, 7.142857, 0, 0, 0, 0,
      0,        0, 6.583961, 0, 0, 0, 0, 0,        0, 1, 0, 0,
      0,        0, 0,        0, 1, 0, 0, 0,        0, 0, 0, 1};

  if (!compare(r_post_true_buf, ekf.r_enu, 3, 1e-3)) {
    return 0;
  }

  if (!compare(v_post_true_buf, ekf.v_enu, 3, 1e-3)) {
    return 0;
  }

  if (!compare(cov_post_true_buf, ekf.covariance->pData, EKF_COV_MATRIX_SIZE,
               1e-4)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test EKF prediction function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ekf_predict() {
  // ========== Initial EKF Instance ========== //
  float64_t r_enu[3] = {300, -10, -1000};
  float64_t v_enu[3] = {10, -130, -500};
  float64_t g_inertial[3] = {0, 0, 9.80665};

  arm_matrix_instance_f64 covariance;
  float64_t covariance_buf[EKF_COV_MATRIX_SIZE] = {
      1, 0, 0, 0,   0, 0, 0, 1, 0, 0, 0,   0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1};
  covariance.numCols = EKF_COV_MATRIX_COLS;
  covariance.numRows = EKF_COV_MATRIX_ROWS;
  covariance.pData = covariance_buf;

  arm_matrix_instance_f64 process_noise;
  float64_t process_noise_buf[EKF_COV_MATRIX_SIZE] = {
      1.111111e-05, 0, 0, 0, 0, 0, 0, 1.111111e-05, 0, 0, 0, 0, 0, 0,
      1.111111e-05, 0, 0, 0, 0, 0, 0, 1.111111e-06, 0, 0, 0, 0, 0, 0,
      1.111111e-06, 0, 0, 0, 0, 0, 0, 1.111111e-06};
  process_noise.numCols = EKF_COV_MATRIX_COLS;
  process_noise.numRows = EKF_COV_MATRIX_ROWS;
  process_noise.pData = process_noise_buf;

  struct ekf_instance ekf;
  ekf.r_enu = r_enu;
  ekf.v_enu = v_enu;
  ekf.covariance = &covariance;
  ekf.process_noise = &process_noise;
  ekf.g_inertial = g_inertial;

  // ========== Perform Prediction ========== //
  float64_t accel_meas[3] = {1.1, -1.2, 10};
  float64_t dt = 0.0033;

  arm_matrix_instance_f64 attitude_matrix;
  float64_t attitude_matrix_buf[9] = {
      6.123234e-17, 0.8660254, -0.5,          0,           0.5,
      0.8660254,    1,         -5.302876e-17, 3.061617e-17};
  attitude_matrix.numCols = 3;
  attitude_matrix.numRows = 3;
  attitude_matrix.pData = attitude_matrix_buf;

  ekf_predict(&ekf, accel_meas, &attitude_matrix, dt);

  // ========== Verify Prediction ========== //
  float64_t v_enu_buf_true[3] = {10.03333, -129.9988, -499.9726};
  float64_t r_enu_buf_true[3] = {300.0334, -10.43333, -1001.667};
  float64_t covariance_buf_true[EKF_COV_MATRIX_SIZE] = {1.000012,
                                                        0,
                                                        0,
                                                        0.0003333333,
                                                        0,
                                                        0,
                                                        0,
                                                        1.000012,
                                                        0,
                                                        0,
                                                        0.0003333333,
                                                        0,
                                                        0,
                                                        0,
                                                        1.000012,
                                                        0,
                                                        0,
                                                        0.0003333333,
                                                        0.0003333333,
                                                        0,
                                                        0,
                                                        0.1000011,
                                                        0,
                                                        0,
                                                        0,
                                                        0.0003333333,
                                                        0,
                                                        0,
                                                        0.1000011,
                                                        0,
                                                        0,
                                                        0,
                                                        0.0003333333,
                                                        0,
                                                        0,
                                                        0.1000011};

  if (!compare(ekf.r_enu, r_enu_buf_true, 3, 0.1)) {
    return 0;
  }

  if (!compare(ekf.v_enu, v_enu_buf_true, 3, 0.1)) {
    return 0;
  }

  if (!compare(ekf.covariance->pData, covariance_buf_true, EKF_COV_MATRIX_SIZE,
               1e-3)) {
    return 0;
  }

  // ========== Test prediction ========== //
  float64_t accel_meas_2[3] = {0.0, 0.0, 0.0};
  ekf_predict(&ekf, accel_meas_2, &attitude_matrix, 0.0);

  return 1;
}
