#include "gnc_tests.h"
#include "cli/cli.h"
#include "dsp/arm_math.h"
#include "gnc/gnc_lib/gnc_lib.h"
#include "gnc/test_utils/test_utils.h"

#include <math.h>

extern struct cli_handle cli;

/**
 * @brief test quaternion multiplication function
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_quaternion_multiplication() {
  float64_t q_out[4];

  // ========== Case 1 ========== //
  float64_t qa_1[4] = {0, 0, 0, 1};
  float64_t qb_1[4] = {0, 0, 0, 1};
  float64_t q_true_1[4] = {0, 0, 0, 1};
  quaternion_multiplication(qa_1, qb_1, q_out);
  if (!compare(q_true_1, q_out, 4, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t qa_2[4] = {0.3258873, 0.4669675, 0.06537574, -0.8194296};
  float64_t qb_2[4] = {0.1960502, -0.5557167, -0.5441063, -0.5972367};
  float64_t q_true_2[4] = {-0.1375316, -0.01365364, 0.6794621, 0.7205761};
  quaternion_multiplication(qa_2, qb_2, q_out);
  if (!compare(q_true_2, q_out, 4, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t qa_3[4] = {-0.5935134, -0.5500343, -0.5036147, -0.3026158};
  float64_t qb_3[4] = {-0.3196351, -0.4917807, -0.4334876, 0.6841591};
  float64_t q_true_3[4] = {-0.3000961, -0.1311826, -0.3294406, -0.885552};
  quaternion_multiplication(qa_3, qb_3, q_out);
  if (!compare(q_true_3, q_out, 4, 1e-6)) {
    return 0;
  }

  return 1;
}

uint8_t test_quaternion_propagate_delta_angle() { return 0; }

/**
 * @brief test quaternion propagation from angular velocity
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_quaternion_propagate_omega() {
  float64_t q_out[4];
  float64_t dt = 0.001;

  // ========== Case 1 ========== //
  float64_t q_in_1[4] = {0.613956440086224, 0.572726129568905,
                         0.359714023326419, 0.406998884037773};
  float64_t q_out_1_true[4] = {0.614126117342133, 0.572872785841017,
                               0.359716507366873, 0.406534030025540};
  float64_t w_1[3] = {0.811580458282477, 0.532825588799455, 0.350727103576883};
  quaternion_propagate_omega(q_in_1, w_1, dt, q_out);
  if (!compare(q_out, q_out_1_true, 4, 1e-9)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t q_in_2[4] = {0.8775741605, 0.2736148568, 0.1448824663,
                         0.3660704494};
  float64_t q_out_2_true[4] = {0.877746439671354, 0.273441399152745,
                               0.145026462504705, 0.365729837518183};
  float64_t w_2[3] = {0.5948960740, 0.2622117478, 0.6028430894};
  quaternion_propagate_omega(q_in_2, w_2, dt, q_out);
  if (!compare(q_out, q_out_2_true, 4, 1e-9)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t q_in_3[4] = {0.1008732732, 0.3096204804, 0.9448659739,
                         0.0344678427};
  float64_t q_out_3_true[4] = {0.041366211768167, 0.370192841351976,
                               0.919433856293492, -0.126045549626221};
  float64_t w_3[3] = {0.3187783019, 0.4241667597, 0.5078582847};
  dt = 0.5;
  quaternion_propagate_omega(q_in_3, w_3, dt, q_out);
  if (!compare(q_out, q_out_3_true, 4, 1e-9)) {
    return 0;
  }

  // ========== Case 4 ========== //
  float64_t q_in_4[4] = {0.3868354817, 0.3712851653, 0.6999259191,
                         0.4718149466};
  float64_t q_out_4_true[4] = {0.3868354817, 0.3712851653, 0.6999259191,
                               0.4718149466};
  float64_t w_4[3] = {0.0, 0.0, 0.0};
  quaternion_propagate_omega(q_in_4, w_4, dt, q_out);
  if (!compare(q_out, q_out_4_true, 4, 1e-9)) {
    return 0;
  }

  // ========== Case 5 ========== //
  float64_t q_in_5[4] = {0.3868354817, 0.3712851653, 0.6999259191,
                         0.4718149466};
  float64_t q_out_5_true[4] = {0.3868354817, 0.3712851653, 0.6999259191,
                               0.4718149466};
  float64_t w_5[3] = {0.0, 0.0, 0.0};
  quaternion_propagate_omega(q_in_5, w_5, dt, q_out);
  if (!compare(q_out, q_out_5_true, 4, 1e-9)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test quaternion to rotation matrix
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_quaternion_to_rotation() {
  float64_t rotation_out_buf[9];
  arm_matrix_instance_f64 rotation_out;
  rotation_out.numCols = 3;
  rotation_out.numRows = 3;
  rotation_out.pData = rotation_out_buf;

  // ========== Case 1 ========== //
  float64_t q_1[4] = {0, 0, 0, 1};
  float64_t rotation_true_buf_1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  quaternion_to_rotation(q_1, &rotation_out);
  if (!compare(rotation_true_buf_1, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t q_2[4] = {0.3247871, -0.7190222, -0.4081213, -0.459301};
  float64_t rotation_true_buf_2[9] = {-0.3671119, -0.09215724, -0.9256003,
                                      -0.8419593, 0.4559008,   0.2885465,
                                      0.3953903,  0.8852466,   -0.2449593};
  quaternion_to_rotation(q_2, &rotation_out);
  if (!compare(rotation_true_buf_2, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t q_3[4] = {0.335896, 0.6892573, -0.3112888, 0.5614246};
  float64_t rotation_true_buf_3[9] = {-0.1439527, 0.1135071,  -0.9830533,
                                      0.8125678,  0.5805464,  -0.05195566,
                                      0.5648107,  -0.8062766, -0.1758035};
  quaternion_to_rotation(q_3, &rotation_out);
  if (!compare(rotation_true_buf_3, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 4 ========== //
  float64_t q_4[4] = {0.3215448, -0.9046441, 0.1873724, -0.2076525};
  float64_t rotation_true_buf_4[9] = {-0.7069788, -0.6595839, -0.2552059,
                                      -0.5039505, 0.7230011,  -0.4725498,
                                      0.4962004,  -0.2054715, -0.8435441};
  quaternion_to_rotation(q_4, &rotation_out);
  if (!compare(rotation_true_buf_4, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test axis-1 rotation matrix
 * @brief 1 if all tests pass, 0 otherwise
 */
uint8_t test_rotation_1() {
  float64_t rotation_out_buf[9];
  arm_matrix_instance_f64 rotation_out;
  rotation_out.numCols = 3;
  rotation_out.numRows = 3;
  rotation_out.pData = rotation_out_buf;

  // ========== Case 1 ========== //
  float64_t theta_1 = 4.418482;
  float64_t rotation_true_buf_1[9] = {
      1, 0, 0, 0, -0.289694, -0.9571193, 0, 0.9571193, -0.289694};
  rotation_1(theta_1, &rotation_out);
  if (!compare(rotation_true_buf_1, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t theta_2 = 3.491804;
  float64_t rotation_true_buf_2[9] = {
      1, 0, 0, 0, -0.9393001, -0.3430968, 0, 0.3430968, -0.9393001};
  rotation_1(theta_2, &rotation_out);
  if (!compare(rotation_true_buf_2, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t theta_3 = 0;
  float64_t rotation_true_buf_3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  rotation_1(theta_3, &rotation_out);
  if (!compare(rotation_true_buf_3, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test axis-2 rotation matrix
 * @brief 1 if all tests pass, 0 otherwise
 */
uint8_t test_rotation_2() {
  float64_t rotation_out_buf[9];
  arm_matrix_instance_f64 rotation_out;
  rotation_out.numCols = 3;
  rotation_out.numRows = 3;
  rotation_out.pData = rotation_out_buf;

  // ========== Case 1 ========== //
  float64_t theta_1 = 0;
  float64_t rotation_true_buf_1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  rotation_2(theta_1, &rotation_out);
  if (!compare(rotation_true_buf_1, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t theta_2 = 1.158831;
  float64_t rotation_true_buf_2[9] = {0.4004111, 0, -0.9163356, 0, 1, 0,
                                      0.9163356, 0, 0.4004111};
  rotation_2(theta_2, &rotation_out);
  if (!compare(rotation_true_buf_2, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t theta_3 = 1.332229;
  float64_t rotation_true_buf_3[9] = {0.2363107, 0, -0.9716775, 0, 1, 0,
                                      0.9716775, 0, 0.2363107};
  rotation_2(theta_3, &rotation_out);
  if (!compare(rotation_true_buf_3, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  return 1;
}

/**
 * @brief test axis-3 rotation matrix
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_rotation_3() {
  float64_t rotation_out_buf[9];
  arm_matrix_instance_f64 rotation_out;
  rotation_out.numCols = 3;
  rotation_out.numRows = 3;
  rotation_out.pData = rotation_out_buf;

  // ========== Case 1 ========== //
  float64_t theta_1 = 0;
  float64_t rotation_true_buf_1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  rotation_3(theta_1, &rotation_out);
  if (!compare(rotation_true_buf_1, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 2 ========== //
  float64_t theta_2 = 0.4859843;
  float64_t rotation_true_buf_2[9] = {
      0.8842156, 0.4670789, 0, -0.4670789, 0.8842156, 0, 0, 0, 1};
  rotation_3(theta_2, &rotation_out);
  if (!compare(rotation_true_buf_2, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  // ========== Case 3 ========== //
  float64_t theta_3 = 5.741577;
  float64_t rotation_true_buf_3[9] = {
      0.8568808, -0.5155145, 0, 0.5155145, 0.8568808, 0, 0, 0, 1};
  rotation_3(theta_3, &rotation_out);
  if (!compare(rotation_true_buf_3, rotation_out.pData, 9, 1e-6)) {
    return 0;
  }

  return 1;
}

uint8_t test_skew_symmetric() { return 0; }

uint8_t test_apply_rotation() { return 0; }

uint8_t test_vec_subtract() { return 0; }

uint8_t test_vec_add() { return 0; }

/**
 * @brief test transformation from ECEF to NED
 * @return 1 if all tests pass, 0 otherwise
 */
uint8_t test_ecef_to_ned() {
  float64_t lat_ref = 33.28 * PI_F / 180;
  float64_t lon_ref = -84.4 * PI_F / 180;
  float64_t r_ref_ecef[3] = {520873.008878507, -5312273.88842034,
                             3480125.8458558};

  // ========== Case 1 NE ========== //
  float64_t r_ecef_1[3] = {522717.61567579, -5311993.22436066, 3480458.8923068};
  float64_t r_ned_1[3];
  float64_t r_ned_1_true[3] = {332.92781341386, 1863.19123965172,
                               -99.7194316272092};
  gnc_ecef_to_ned(lat_ref, lon_ref, r_ref_ecef, r_ecef_1, r_ned_1);
  if (!compare(r_ned_1, r_ned_1_true, 3, 1e-6)) {
    return 0;
  }

  // ========== Case 2 SW ========== //
  float64_t r_ecef_2[3] = {511714.974683035, -5314363.20393616,
                           3479582.55746385};
  float64_t r_ned_2[3];
  float64_t r_ned_2_true[3] = {-1104.80602718832, -9318.20802784908,
                               -693.105128329149};
  gnc_ecef_to_ned(lat_ref, lon_ref, r_ref_ecef, r_ecef_2, r_ned_2);
  if (!compare(r_ned_2, r_ned_2_true, 3, 1e-6)) {
    return 0;
  }

  // ========== Case 3 SE ========== //
  float64_t r_ecef_3[3] = {524106.96617365, -5316590.92614286,
                           3472540.26302639};
  float64_t r_ned_3[3];
  float64_t r_ned_3_true[3] = {-8872.29406636718, 2797.25385116441,
                               306.80651621803};
  gnc_ecef_to_ned(lat_ref, lon_ref, r_ref_ecef, r_ecef_3, r_ned_3);
  if (!compare(r_ned_3, r_ned_3_true, 3, 1e-6)) {
    return 0;
  }

  // ========== Case 4 NW ========== //
  float64_t r_ecef_4[3] = {512319.35383272, -5320639.92139259,
                           3486377.14535461};
  float64_t r_ned_4[3];
  float64_t r_ned_4_true[3] = {1115.3080828019, -9329.21362847258,
                               -9693.09696194151};
  gnc_ecef_to_ned(lat_ref, lon_ref, r_ref_ecef, r_ecef_4, r_ned_4);
  if (!compare(r_ned_4, r_ned_4_true, 3, 1e-6)) {
    return 0;
  }

  return 1;
}