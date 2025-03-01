#include "gnc_lib.h"
#include <math.h>

/**
 * @brief compute quaternion product
 *
 * Compute the non-Hamiltonian product qc = qa * qb between two four-element
 * floating-point buffers representing quaternions
 * @param qa pointer to first quaternion
 * @param qb pointer to second quaternion
 * @param qc pointer to resulting quaternion
 * @return function exit status
 */
enum gnc_err quaternion_multiplication(float64_t *qa, float64_t *qb,
                                       float64_t *qc) {
  float64_t qa1 = qa[0];
  float64_t qa2 = qa[1];
  float64_t qa3 = qa[2];
  float64_t qa4 = qa[3];

  float64_t qb1 = qb[0];
  float64_t qb2 = qb[1];
  float64_t qb3 = qb[2];
  float64_t qb4 = qb[3];

  qc[0] = qa1 * qb4 - qa2 * qb3 + qa3 * qb2 + qa4 * qb1;
  qc[1] = qa1 * qb3 - qa3 * qb1 + qa2 * qb4 + qa4 * qb2;
  qc[2] = qa2 * qb1 - qa1 * qb2 + qa3 * qb4 + qa4 * qb3;
  qc[3] = qa4 * qb4 - qa2 * qb2 - qa3 * qb3 - qa1 * qb1;

  return GNC_ERR_OK;
}

/**
 * @brief normalize an input quaternion
 *
 * Brute-force normalize a quaternion by dividing each element by the L-2 norm
 * of the input quaternion
 * @param qa pointer to input non-normalized quaternion
 * @param qb pointer to array to store output unit-normalized quaternion
 * @return gnc_err
 */
enum gnc_err quaternion_norm(float64_t *qa, float64_t *qb) {
  float64_t qn =
      sqrt(qa[0] * qa[0] + qa[1] * qa[1] + qa[2] * qa[2] + qa[3] * qa[3]);
  qb[0] = qa[0] / qn;
  qb[1] = qa[1] / qn;
  qb[2] = qa[2] / qn;
  qb[3] = qa[3] / qn;

  return GNC_ERR_OK;
}

enum gnc_err quaternion_propagate_delta_angle() { return GNC_ERR_OK; }

/**
 * @brief Propagate quaternion from an angular velocity
 *
 * This function uses the Euler-rodrigues rotation formula to propagate a
 * quaternion given angular rate and a time step. The function also
 * re-normalizes the quaternion after computations are complete.
 * @param q0 initial quaternion array
 * @param w angular velocity array (rad/s)
 * @param dt propagation time step (s)
 * @param qf to final quaternion array
 */
enum gnc_err quaternion_propagate_omega(float64_t *q0, float64_t *omega,
                                        float64_t dt, float64_t *qf) {
  // Separate out omega
  float64_t w1 = omega[0];
  float64_t w2 = omega[1];
  float64_t w3 = omega[2];
  float64_t wn = sqrt(w1 * w1 + w2 * w2 + w3 * w3);

  if (wn == 0) {
    qf[0] = q0[0];
    qf[1] = q0[1];
    qf[2] = q0[2];
    qf[3] = q0[3];
  } else {
    // Calculate sine and cosine constants
    float64_t c = cos(wn * dt / 2);
    float64_t s = (1.0 / wn) * sin(wn * dt / 2);

    // Separate out initial quaternion
    float64_t q1 = q0[0];
    float64_t q2 = q0[1];
    float64_t q3 = q0[2];
    float64_t qs = q0[3];

    // Propagate quaternion
    float64_t qf1 = q1 * c + qs * w1 * s + q2 * w3 * s - q3 * w2 * s;
    float64_t qf2 = q2 * c + qs * w2 * s - q1 * w3 * s + q3 * w1 * s;
    float64_t qf3 = q3 * c + qs * w3 * s + q1 * w2 * s - q2 * w1 * s;
    float64_t qfs = qs * c - q1 * s * w1 - q2 * s * w2 - q3 * s * w3;

    // Re-normalize quaternion and store to qf
    float64_t qn = sqrt(qf1 * qf1 + qf2 * qf2 + qf3 * qf3 + qfs * qfs);
    qf[0] = qf1 / qn;
    qf[1] = qf2 / qn;
    qf[2] = qf3 / qn;
    qf[3] = qfs / qn;
  }

  return GNC_ERR_OK;
}
/**
 * @brief convert a quaternion to a rotation matrix
 *
 * Compute a rotation matrix and store in an arm matrix instance given a
 * four-element input buffer representing a quaternion.
 * @param q pointer to the quaternion
 * @param rotation_matrix arm matrix instance for the resulting rotation matrix
 */
enum gnc_err quaternion_to_rotation(float64_t *q,
                                    arm_matrix_instance_f64 *rotation_matrix) {
  // Extract quaternion
  float64_t q1 = q[0];
  float64_t q2 = q[1];
  float64_t q3 = q[2];
  float64_t q4 = q[3];

  // Pre-compute products
  float64_t q12 = q1 * q1;
  float64_t q22 = q2 * q2;
  float64_t q32 = q3 * q3;
  float64_t q42 = q4 * q4;
  float64_t q1q2 = q1 * q2;
  float64_t q3q4 = q3 * q4;
  float64_t q1q3 = q1 * q3;
  float64_t q2q4 = q2 * q4;
  float64_t q2q3 = q2 * q3;
  float64_t q1q4 = q1 * q4;

  // Compute rotation matrix and store in
  rotation_matrix->pData[0] = q42 + q12 - q22 - q32;
  rotation_matrix->pData[1] = 2.0 * (q1q2 + q3q4);
  rotation_matrix->pData[2] = 2.0 * (q1q3 - q2q4);
  rotation_matrix->pData[3] = 2.0 * (q1 * q2 - q3 * q4);
  rotation_matrix->pData[4] = q42 - q12 + q22 - q32;
  rotation_matrix->pData[5] = 2.0 * (q2q3 + q1q4);
  rotation_matrix->pData[6] = 2.0 * (q1q3 + q2q4);
  rotation_matrix->pData[7] = 2.0 * (q2q3 - q1q4);
  rotation_matrix->pData[8] = q42 - q12 - q22 + q32;

  return GNC_ERR_OK;
}

/**
 * @brief compute first axis rotation matrix
 *
 * Compute a rotation matrix for a passive counterclockwise rotation about the
 * first axis
 * @param theta rotation angle (rad)
 * @param rotation_matrix arm matrix instance to store resulting rotation matrix
 */
enum gnc_err rotation_1(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix) {
  // Pre-compute sine and cosine
  float64_t c = cos(theta);
  float64_t s = sin(theta);

  // Compute rotation matrix and store
  rotation_matrix->pData[0] = 1;
  rotation_matrix->pData[1] = 0;
  rotation_matrix->pData[2] = 0;
  rotation_matrix->pData[3] = 0;
  rotation_matrix->pData[4] = c;
  rotation_matrix->pData[5] = s;
  rotation_matrix->pData[6] = 0;
  rotation_matrix->pData[7] = -s;
  rotation_matrix->pData[8] = c;

  return GNC_ERR_OK;
}

/**
 * @brief compute second axis rotation matrix
 *
 * Compute a rotation matrix for a passive counterclockwise rotation about the
 * second axis
 * @param theta rotation angle (rad)
 * @param rotation_matrix arm matrix instance to store resulting rotation matrix
 */
enum gnc_err rotation_2(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix) {
  // Pre-compute sine and cosine
  float64_t c = cos(theta);
  float64_t s = sin(theta);

  // Compute rotation matrix and store
  rotation_matrix->pData[0] = c;
  rotation_matrix->pData[1] = 0;
  rotation_matrix->pData[2] = -s;
  rotation_matrix->pData[3] = 0;
  rotation_matrix->pData[4] = 1;
  rotation_matrix->pData[5] = 0;
  rotation_matrix->pData[6] = s;
  rotation_matrix->pData[7] = 0;
  rotation_matrix->pData[8] = c;

  return GNC_ERR_OK;
}

/**
 * @brief compute third axis rotation matrix
 *
 * Compute a rotation matrix for a passive counterclockwise rotation about the
 * third axis
 * @param theta rotation angle (rad)
 * @param rotation_matrix arm matrix instance to store resulting rotation matrix
 */
enum gnc_err rotation_3(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix) {
  // Pre-compute sine and cosine
  float64_t c = cos(theta);
  float64_t s = sin(theta);

  // Compute rotation matrix and store
  rotation_matrix->pData[0] = c;
  rotation_matrix->pData[1] = s;
  rotation_matrix->pData[2] = 0;
  rotation_matrix->pData[3] = -s;
  rotation_matrix->pData[4] = c;
  rotation_matrix->pData[5] = 0;
  rotation_matrix->pData[6] = 0;
  rotation_matrix->pData[7] = 0;
  rotation_matrix->pData[8] = 1;

  return GNC_ERR_OK;
}

/**
 * @brief compute the skew-symmetric matrix of a vector
 *
 * Computes the skew-symmetric (cross-product) matrix of a vector and stores it
 * in a float array using row-major format to be consistent with ARM_MATH
 * libraries
 * @param vector pointer to three-element vector
 * @param vector_skew pointer to array to store skew-symmetric matrix
 */
enum gnc_err skew_symmetric(float64_t *vector, float64_t *vector_skew) {

  memset(vector_skew, 0, 9 * sizeof(float64_t));

  vector_skew[1] = -vector[2];
  vector_skew[2] = vector[1];
  vector_skew[3] = vector[2];
  vector_skew[5] = -vector[0];
  vector_skew[6] = -vector[1];
  vector_skew[7] = vector[0];

  return GNC_ERR_OK;
}

/**
 * @brief transform a vector by applying a 3x3 transformation matrix
 *
 * Find a transformed vector by calculating the product of the vector with a
 * given rotation matrix
 * @param rotation_matrix pointer to arm matrix instance of rotation matrix
 * @param vector pointer to array representing un-transformed rotation matrix
 * @param vector_transformed pointer to array to store transformed vector
 */
enum gnc_err apply_rotation(arm_matrix_instance_f64 *rotation_matrix,
                            float64_t *vector, float64_t *vector_transformed) {
  vector_transformed[0] = rotation_matrix->pData[0] * vector[0] +
                          rotation_matrix->pData[1] * vector[1] +
                          rotation_matrix->pData[2] * vector[2];
  vector_transformed[1] = rotation_matrix->pData[3] * vector[0] +
                          rotation_matrix->pData[4] * vector[1] +
                          rotation_matrix->pData[5] * vector[2];
  vector_transformed[2] = rotation_matrix->pData[6] * vector[0] +
                          rotation_matrix->pData[7] * vector[1] +
                          rotation_matrix->pData[8] * vector[2];

  return GNC_ERR_OK;
}

/**
 * @brief subtract a vector from another vector
 *
 * Given vector_1 and vector_2, calculate the subtraction vector_3 = vector_1 -
 * vector_2
 * @param vector_1 pointer to first vector
 * @param vector_2 pointer to second vector
 * @param len length of vectors
 * @param vector_3 pointer to store resulting vector
 */
enum gnc_err vec_subtract(float64_t *vector_1, float64_t *vector_2, uint8_t len,
                          float64_t *vector_3) {
  for (uint8_t i = 0; i < len; i++) {
    vector_3[i] = vector_1[i] - vector_2[i];
  }

  return GNC_ERR_OK;
}

/**
 * @brief add two vectors
 *
 * Given vector_1 and vector_2, calculate the addition vector_3 = vector_1 +
 * vector_2
 * @param vector_1 pointer to first vector
 * @param vector_2 pointer to second vector
 * @param len length of vectors
 * @param vector_3 pointer to store resulting vector
 */
enum gnc_err vec_add(float64_t *vector_1, float64_t *vector_2, uint8_t len,
                     float64_t *vector_3) {
  for (uint8_t i = 0; i < len; i++) {
    vector_3[i] = vector_1[i] + vector_2[i];
  }

  return GNC_ERR_OK;
}

/**
 * @brief Convert ECEF position vector to NED position centered at a given
 * reference point.
 *
 * This function determines the position vector in a topocentric North-East-Down
 * frame centered about a reference point. Reference point is specified by its
 * latitude, longitude, and ECEF position vector for computational efficiency.
 * @param lat reference point latitude (rad)
 * @param lon reference point longitude (rad)
 * @param r_ref_ecef reference point ECEF vector (m)
 * @param r_ecef position vector in ECEF frame (m)
 * @param r_ned NED position vector centered at reference point (m)
 */
enum gnc_err gnc_ecef_to_ned(float64_t lat_ref, float64_t lon_ref,
                             float64_t *r_ref_ecef, float64_t *r_ecef,
                             float64_t *r_ned) {
  // Pre-compute trig values
  float64_t s_lat_ref = sin(lat_ref);
  float64_t c_lat_ref = cos(lat_ref);
  float64_t s_lon_ref = sin(lon_ref);
  float64_t c_lon_ref = cos(lon_ref);

  // Compute ECEF range vector
  float64_t x_diff = r_ecef[0] - r_ref_ecef[0];
  float64_t y_diff = r_ecef[1] - r_ref_ecef[1];
  float64_t z_diff = r_ecef[2] - r_ref_ecef[2];

  // Transform to ENU frame
  float64_t x = -s_lon_ref * x_diff + c_lon_ref * y_diff;
  float64_t y = -s_lat_ref * c_lon_ref * x_diff -
                s_lat_ref * s_lon_ref * y_diff + c_lat_ref * z_diff;
  float64_t z = c_lat_ref * c_lon_ref * x_diff +
                c_lat_ref * s_lon_ref * y_diff + s_lat_ref * z_diff;

  // Transform to NED and store in output array
  r_ned[0] = y;
  r_ned[1] = x;
  r_ned[2] = -1.0 * z;

  return GNC_ERR_OK;
}