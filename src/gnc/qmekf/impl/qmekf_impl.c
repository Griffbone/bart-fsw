/**
 * @file ekfk_impl.c
 * @brief intialization of flight attitude EKF and wrappers for running
 *
 * This file contains intializations for the QMEKF instance to be flown on
 * Strange Magic, in addition to wrappers around the QMEKF source code to
 * directly update the Kalman filter from sensor measurements. The file also
 * introduces the following struct:
 *  - \c qmekf_data: a struct to store number of predict and updates performed
 * by the QMEKF
 *
 * @author Griffin Jourda
 */
#include "qmekf_impl.h"
#include "gnc/qmekf/qmekf.h"

// ======================= INITIALIZE HISTORY STRUCTS ====================== //
struct qmekf_hist_instance qmekf_data = {
    .num_updates = 0,
    .num_predicts = 0,
};

// ========================================================================= //
// ================== INITIALIZE FLIGHT QMEKF PARAMETERS =================== //
// ========================================================================= //
struct qmekf_instance qmekf;

// Covariance
arm_matrix_instance_f64 qmekf_covariance;
float64_t qmekf_covariance_buf[QMEKF_PHI_MATRIX_SIZE] = {
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Attitude matrix
arm_matrix_instance_f64 qmekf_attitude_mat;
float64_t qmekf_attitude_mat_buf[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// Gyro bias
float64_t qmekf_gyro_bias[3] = {0, 0, 0};

// Quaternion
float64_t qmekf_quaternion[4] = {0, 0, 0, 1};

// Measurement noise
arm_matrix_instance_f64 qmekf_meas_noise;
float64_t qmekf_meas_noise_buf[36] = {
    400, 0, 0, 0,     0, 0, 0, 400, 0, 0, 0,     0, 0, 0, 400, 0, 0, 0,
    0,   0, 0, 1e-06, 0, 0, 0, 0,   0, 0, 1e-06, 0, 0, 0, 0,   0, 0, 1e-06};

// Process noise
arm_matrix_instance_f64 qmekf_process_noise;
float64_t qmekf_process_noise_buf[36] = {
    1.949551e-12, 0, 0, 0, 0, 0, 0, 1.949551e-12, 0, 0, 0, 0, 0, 0,
    1.949551e-12, 0, 0, 0, 0, 0, 0, 1e-11,        0, 0, 0, 0, 0, 0,
    1e-11,        0, 0, 0, 0, 0, 0, 1e-11};

// Inertial magnetic field (from
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm for FAR
// on 7/12/2024)
float64_t mag_vec_inertial[3] = {22972.1, 4724.9, 40753.5};

// Inertial gravity vector
float64_t g_vec_inertial[3] = {0, 0, 9.80665};

// ========================================================================= //
// ======================== FLIGHT QMEKF FUNCTIONS ========================= //
// ========================================================================= //

/**
 * @brief assign globally-defined QMEKF parameters to the flight QMEKF structure
 */
void flight_qmekf_init() {
  // Covariance
  qmekf_covariance.numCols = 6;
  qmekf_covariance.numRows = 6;
  qmekf_covariance.pData = qmekf_covariance_buf;
  qmekf.covariance = &qmekf_covariance;

  // Attitude matrix
  qmekf_attitude_mat.numCols = 3;
  qmekf_attitude_mat.numRows = 3;
  qmekf_attitude_mat.pData = qmekf_attitude_mat_buf;
  qmekf.attitude_matrix = &qmekf_attitude_mat;

  // Quaternion
  qmekf.quaternion = qmekf_quaternion;

  // Gyro bias
  qmekf.gyro_bias = qmekf_gyro_bias;

  // Measurement noise
  qmekf_meas_noise.numCols = 6;
  qmekf_meas_noise.numRows = 6;
  qmekf_meas_noise.pData = qmekf_meas_noise_buf;
  qmekf.measurement_covariance = &qmekf_meas_noise;

  // Process noise
  qmekf_process_noise.numCols = 6;
  qmekf_process_noise.numRows = 6;
  qmekf_process_noise.pData = qmekf_process_noise_buf;
  qmekf.process_noise = &qmekf_process_noise;

  // Inertial magnetic field
  qmekf.mag_vec_inertial = mag_vec_inertial;

  // Inertial gravity vector
  qmekf.g_vec_inertial = g_vec_inertial;

  // Time
  qmekf.time_prev = 0.0;
}

/**
 * @brief Update the QMEKF from sensor readings
 *
 * Given magnetometer and accelerometer readings, update the QMEKF attitude
 * and bias estimates.
 * @param mag pointer to array with vector magnetometer reading (nT)
 * @param accel pointer to array with vector accelerometer measurment (m/s^2)
 */
void flight_qmekf_update(float64_t *mag, float64_t *accel) {
  qmekf_update(&qmekf, mag, accel);
  qmekf_data.num_updates++;
}

/**
 * @brief Predict the next state of the attitude Kalman filter
 *
 * From gyroscope measurements and a time step, propagate the vehicle attitude
 * usingthe QMEKF instance
 * @param omega pointer to measured angular velocity vector (rad/s)
 * @param dt integration time step (s)
 */
void flight_qmekf_predict(float64_t *omega, float64_t dt) {
  qmekf_predict(&qmekf, omega, dt);
  qmekf_data.num_predicts++;
}