#include "qmekf_tests.h"
#include "gnc/gnc_lib/gnc_lib.h"
#include "gnc/qmekf/qmekf.h"
#include "gnc/test_utils/test_utils.h"

/**
 * @brief test QMKEF state-transition matrix function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_qmekf_get_stm() {
  float64_t stm_out_buf[QMEKF_PHI_MATRIX_SIZE];
  arm_matrix_instance_f64 stm_out;
  stm_out.numCols = 6;
  stm_out.numRows = 6;
  stm_out.pData = stm_out_buf;

  // ========== Case 1 ========== //
  float64_t w_1[3] = {0.0, 0.0, 0.0};
  float64_t dt_1 = 0.001;
  float64_t stm_true_1_buf[36] = {
      1, 0, 0, -0.001, 0, 0, 0, 1, 0, 0, -0.001, 0, 0, 0, 1, 0, 0, -0.001,
      0, 0, 0, 1,      0, 0, 0, 0, 0, 0, 1,      0, 0, 0, 0, 0, 0, 1};
  qmekf_get_stm(w_1, dt_1, &stm_out);
  if (!compare(stm_true_1_buf, stm_out.pData, 36, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t w_2[3] = {-6.093683, 0.9012933, -1.616919};
  float64_t dt_2 = 0.001;
  float64_t stm_true_2_buf[36] = {1,
                                  -0.001616919,
                                  -0.0009012933,
                                  -0.001,
                                  0,
                                  0,
                                  0.001616919,
                                  1,
                                  -0.006093683,
                                  0,
                                  -0.001,
                                  0,
                                  0.0009012933,
                                  0.006093683,
                                  1,
                                  0,
                                  0,
                                  -0.001,
                                  0,
                                  0,
                                  0,
                                  1,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  1,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  1};
  qmekf_get_stm(w_2, dt_2, &stm_out);
  if (!compare(stm_true_2_buf, stm_out.pData, 36, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t w_3[3] = {-0.01505417, 0.01027405, -0.0130678};
  float64_t dt_3 = 0.0030;
  float64_t stm_true_3_buf[36] = {1,
                                  -3.920341e-05,
                                  -3.082215e-05,
                                  -0.003,
                                  0,
                                  0,
                                  3.920341e-05,
                                  1,
                                  -4.516252e-05,
                                  0,
                                  -0.003,
                                  0,
                                  3.082215e-05,
                                  4.516252e-05,
                                  1,
                                  0,
                                  0,
                                  -0.003,
                                  0,
                                  0,
                                  0,
                                  1,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  1,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  1};
  qmekf_get_stm(w_3, dt_3, &stm_out);
  if (!compare(stm_true_3_buf, stm_out.pData, 36, 1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test QMEKF sensitivity matrix function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_qmekf_get_sens() {
  arm_matrix_instance_f64 attitude_matrix;
  float64_t attitude_matrix_buf[9];
  attitude_matrix.numCols = 3;
  attitude_matrix.numRows = 3;
  attitude_matrix.pData = attitude_matrix_buf;

  arm_matrix_instance_f64 sens_matrix;
  float64_t sens_matrix_buf[36];
  sens_matrix.numCols = 6;
  sens_matrix.numRows = 6;
  sens_matrix.pData = sens_matrix_buf;

  float64_t b_inertial[3] = {22789.9, -2191.8, 42663.2};
  float64_t g_inertial[3] = {0, 0, 9.80665};

  // ========== Case 1 ========== //
  float64_t q_est_1[4] = {-0.5152053, -0.3531716, 0.5135744, -0.5882811};
  quaternion_to_rotation(q_est_1, &attitude_matrix);
  float64_t sens_true_1_buf[36] = {
      0, -8904.987, 32577.04, 0,         0,         0, 8904.987, 0, 34695.38,
      0, 0,         0,        -32577.04, -34695.38, 0, 0,        0, 0,
      0, -2.154194, 2.387049, 0,         0,         0, 2.154194, 0, 9.264547,
      0, 0,         0,        -2.387049, -9.264547, 0, 0,        0, 0};
  qmekf_get_sens(&attitude_matrix, b_inertial, g_inertial, &sens_matrix);
  if (!compare(sens_matrix.pData, sens_true_1_buf, 36, 0.01)) {
    return 0;
  }

  // ========== Case 2 =========== //
  float64_t q_est_2[4] = {0, 0, 0, 1};
  quaternion_to_rotation(q_est_2, &attitude_matrix);
  float64_t sens_true_2_buf[36] = {
      0, -42663.2, -2191.8, 0,      0,       0, 42663.2, 0, -22789.9,
      0, 0,        0,       2191.8, 22789.9, 0, 0,       0, 0,
      0, -9.80665, 0,       0,      0,       0, 9.80665, 0, 0,
      0, 0,        0,       0,      0,       0, 0,       0, 0};
  qmekf_get_sens(&attitude_matrix, b_inertial, g_inertial, &sens_matrix);
  if (!compare(sens_matrix.pData, sens_true_2_buf, 36, 0.01)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t q_est_3[4] = {0.6580238, 0.2470479, -0.3719492, 0.6063215};
  quaternion_to_rotation(q_est_3, &attitude_matrix);
  float64_t sens_true_3_buf[36] = {
      0, 1666.985,  44203.97, 0,         0,         0, -1666.985, 0, 19686.61,
      0, 0,         0,        -44203.97, -19686.61, 0, 0,         0, 0,
      0, -0.117131, 6.022945, 0,         0,         0, 0.117131,  0, 7.738268,
      0, 0,         0,        -6.022945, -7.738268, 0, 0,         0, 0};
  qmekf_get_sens(&attitude_matrix, b_inertial, g_inertial, &sens_matrix);
  if (!compare(sens_matrix.pData, sens_true_3_buf, 36, 0.01)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test QMEKF get residual function
 * @return uint8_t 1 if all tests pass, 0 otherwise
 */
uint8_t test_qmekf_get_resisdual() {

  arm_matrix_instance_f64 attitude_matrix;
  attitude_matrix.numCols = 3;
  attitude_matrix.numRows = 3;

  arm_matrix_instance_f64 residual;
  float64_t residual_buf[6];
  residual.numCols = 6;
  residual.numRows = 1;
  residual.pData = residual_buf;

  float64_t g_inertial[3] = {0, 0, 9.80665};
  float64_t b_inertial[3] = {22789.9, -2191.8, 42663.2};

  // ========== Case 1 ========== //
  float64_t dcm_1_buf[9] = {0.3323561,  -0.8675651, -0.3699599,
                            -0.9045539, -0.4042782, 0.13543,
                            -0.267061,  0.2896377,  -0.9191237};
  attitude_matrix.pData = dcm_1_buf;
  float64_t b_meas_1[3] = {34621.72, 25864.7, -21022.51};
  float64_t g_meas_1[3] = {7.929227, 5.736021, 0.3602769};
  float64_t residual_1_true_buf[6] = {40929.5,  39815.43, 24911.37,
                                      11.55729, 4.407907, 9.373802};
  qmekf_get_residual(&attitude_matrix, b_inertial, g_inertial, b_meas_1,
                     g_meas_1, &residual);
  if (!compare(residual_1_true_buf, residual.pData, 6, 0.1)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t dcm_2_buf[9] = {-0.03659358, -0.5846826, 0.8104364,
                            0.1159004,   0.8030238,  0.5845681,
                            -0.9925865,  0.1153213,  0.03837941};
  attitude_matrix.pData = dcm_2_buf;
  float64_t b_meas_2[3] = {34621.72, 25864.7, -21022.51};
  float64_t g_meas_2[3] = {7.929227, 5.736021, 0.3602769};
  float64_t residual_2_true_buf[6] = {-401.6358,   43.86664,   213.8137,
                                      -0.01843893, 0.00336632, -0.01609656};
  qmekf_get_residual(&attitude_matrix, b_inertial, g_inertial, b_meas_2,
                     g_meas_2, &residual);
  if (!compare(residual_2_true_buf, residual.pData, 6, 0.1)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t dcm_3_buf[9] = {-0.3783613, 0.925267,   0.02690379,
                            -0.8050671, -0.3432753, 0.4837655,
                            0.4568476,  0.1613788,  0.8747841};
  attitude_matrix.pData = dcm_3_buf;
  float64_t b_meas_3[3] = {-18284.49, 10946.47, 43448.73};
  float64_t g_meas_3[3] = {-1.791692, 6.271169, 7.304967};
  float64_t residual_3_true_buf[6] = {-8781.481, 7902.498, -3930.156,
                                      -2.055528, 1.52705,  -1.273734};
  qmekf_get_residual(&attitude_matrix, b_inertial, g_inertial, b_meas_3,
                     g_meas_3, &residual);
  if (!compare(residual_3_true_buf, residual.pData, 6, 0.1)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test QMEKF prediction step
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_qmekf_predict() {
  // ========== Initial QMEKF object ========== //
  float64_t q_prior[4] = {0, 0, 0, 1};
  float64_t b_prior[3] = {0, 0, 0};

  arm_matrix_instance_f64 cov_prior;
  float64_t cov_prior_buf[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  cov_prior.numCols = 6;
  cov_prior.numRows = 6;
  cov_prior.pData = cov_prior_buf;

  arm_matrix_instance_f64 process_noise;
  float64_t process_noise_buf[36] = {
      7.615435e-05, 0, 0, 0, 0, 0, 0, 7.615435e-05, 0, 0, 0, 0, 0, 0,
      7.615435e-05, 0, 0, 0, 0, 0, 0, 0.0001,       0, 0, 0, 0, 0, 0,
      0.0001,       0, 0, 0, 0, 0, 0, 0.0001};
  process_noise.numCols = 6;
  process_noise.numRows = 6;
  process_noise.pData = process_noise_buf;

  arm_matrix_instance_f64 attitude_matrix;
  float64_t attitude_matrix_buf[9];
  attitude_matrix.numCols = 3;
  attitude_matrix.numCols = 3;
  attitude_matrix.pData = attitude_matrix_buf;

  struct qmekf_instance qmekf;
  qmekf.gyro_bias = b_prior;
  qmekf.quaternion = q_prior;
  qmekf.covariance = &cov_prior;
  qmekf.process_noise = &process_noise;
  qmekf.attitude_matrix = &attitude_matrix;

  // ========== Case 1 ========== //
  // Call update function
  float64_t omega_1[3] = {0.8726646, -0.03490659, 0.05235988};
  float64_t dt = 0.001;

  qmekf_predict(&qmekf, omega_1, dt);

  // Verify correct values
  float64_t q_prior_true[4] = {0.0004363323, -1.745329e-05, 2.617994e-05,
                               0.9999999};
  float64_t b_prior_true[3] = {0, 0, 0};
  float64_t cov_prior_true[36] = {1.000076,
                                  3.046174e-08,
                                  -4.569261e-08,
                                  0,
                                  0,
                                  0,
                                  3.046174e-08,
                                  1.000077,
                                  1.827705e-09,
                                  0,
                                  0,
                                  0,
                                  -4.569261e-08,
                                  1.827705e-09,
                                  1.000077,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0.0001,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0.0001,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0.0001};
  uint8_t c1 = compare(q_prior_true, qmekf.quaternion, 4, 1e-6);
  uint8_t c2 = compare(b_prior_true, qmekf.gyro_bias, 3, 1e-6);
  uint8_t c3 = compare(cov_prior_true, qmekf.covariance->pData, 36, 1e-6);

  if (!(c1 && (c2 && c3))) {
    return 0;
  }

  // Flags
  return 1;
};

/**
 * @brief test QMEKF update step
 * @return 1 if test passes, 0 otherwise
 */
uint8_t test_qmekf_update() {
  // Initialize QMEKF variables
  arm_matrix_instance_f64 cov;
  float64_t cov_buf[QMEKF_PHI_MATRIX_SIZE] = {
      1, 0, 0, 0,   0, 0, 0, 1, 0, 0, 0,   0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1};
  cov.numCols = 6;
  cov.numRows = 6;
  cov.pData = cov_buf;

  arm_matrix_instance_f64 cov_meas;
  float64_t cov_meas_buf[QMEKF_PHI_MATRIX_SIZE] = {
      40000, 0, 0,     0, 0,      0, 0, 40000, 0, 0,      0, 0,
      0,     0, 40000, 0, 0,      0, 0, 0,     0, 0.0001, 0, 0,
      0,     0, 0,     0, 0.0001, 0, 0, 0,     0, 0,      0, 0.0001};
  cov_meas.numCols = 6;
  cov_meas.numRows = 6;
  cov_meas.pData = cov_meas_buf;

  arm_matrix_instance_f64 attitude_matrix;
  float64_t attitude_matrix_buf[9];
  attitude_matrix.numCols = 3;
  attitude_matrix.numRows = 3;
  attitude_matrix.pData = attitude_matrix_buf;

  float64_t q_prior[4] = {0.4295436, 0.1989121, 0.4705624, 0.7446458};
  float64_t b_prior[3] = {0.001745329, 0.01745329, -0.01745329};
  float64_t b_inertial[3] = {22789.9, -2191.8, 42663.2};
  float64_t g_inertial[3] = {0, 0, 9.80665};

  struct qmekf_instance qmekf;
  qmekf.covariance = &cov;
  qmekf.measurement_covariance = &cov_meas;
  qmekf.attitude_matrix = &attitude_matrix;
  qmekf.mag_vec_inertial = b_inertial;
  qmekf.g_vec_inertial = g_inertial;
  qmekf.quaternion = q_prior;
  qmekf.gyro_bias = b_prior;
  quaternion_to_rotation(q_prior, qmekf.attitude_matrix);

  // ========== Update the Filter ========== //
  float64_t b_meas[3] = {11104.4, 30896.23, 35770};
  float64_t g_meas[3] = {-0.07845827, 8.927737, 4.032118};

  qmekf_update(&qmekf, b_meas, g_meas);

  // ========== Confirm the update was correct //
  float64_t q_post_true_buf[4] = {0.4754889, 0.2539742, 0.3846095, 0.7493216};
  float64_t b_post_true_buf[4] = {0.001745329, 0.01745329, -0.01745329};
  float64_t cov_post_tue_buf[36] = {2.074496e-06,
                                    7.552018e-06,
                                    5.296987e-06,
                                    0,
                                    0,
                                    0,
                                    7.552015e-06,
                                    5.322385e-05,
                                    3.659927e-05,
                                    0,
                                    0,
                                    0,
                                    5.296982e-06,
                                    3.659927e-05,
                                    2.663497e-05,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0.1,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0.1,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0.1};

  if (!compare(q_post_true_buf, qmekf.quaternion, 4, 1e-6)) {
    return 0;
  }

  if (!compare(b_post_true_buf, qmekf.gyro_bias, 3, 1e-6)) {
    return 0;
  }

  if (!compare(cov_post_tue_buf, qmekf.covariance->pData, 36, 1e-6)) {
    return 0;
  }

  //  print_matrix(qmekf.quaternion, 4);
  //  print_matrix(qmekf.gyro_bias, 3);
  //  print_matrix(qmekf.covariance->pData, 36);

  return 1;
}