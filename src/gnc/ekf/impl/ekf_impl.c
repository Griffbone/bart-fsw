/**
 * @file ekfk_impl.c
 * @brief intialization of flight positioning EKF and wrappers for running.
 *
 * This file contains intializations for the EKF instance to be flown on Strange
 * Magic, in addition to wrappers aroundthe EKF source code to directly update
 * the Kalman filter from sensor measurements, including frame transformations
 * and unit conversions between the two. The file also introduces the following
 * structs:
 *  - \c ground_station: a struct to store information on the location of the
 * ground station, which is the NED frame origin
 *  - \c ekf_housekeeping: a struct of other parameters related to the EKF that
 * are important for flight
 *
 * @author Griffin Jourda
 */
#include "ekf_impl.h"
#include "dsp/arm_math.h"
#include "gnc/gnc_lib/gnc_lib.h"
#include "gnc/qmekf/qmekf.h"

// ========== INITIALIZE Flight EKF Parameters========== //
struct ground_station_instance ground_station = {
    .lat = 35.34765999092926 * PI_F / 180.0,
    .lon = -117.80837401117485 * PI_F / 810.0,
    .alt = 630,
    .r_ecef = {-2429951.616, -4607181.056, 3669759.331},
};

struct flight_hist_data_instance position_flight_data = {
    .max_alt = 0,
    .num_updates = 0,
    .num_predicts = 0,
};

// ========== INITIALIZE EKF Parameters========== //
struct ekf_instance ekf;

// Covariance
arm_matrix_instance_f64 ekf_covariance;
float64_t ekf_covariance_buf[EKF_COV_MATRIX_SIZE] = {
    10, 0, 0, 0,  0, 0, 0, 10, 0, 0, 0,  0, 0, 0, 10, 0, 0, 0,
    0,  0, 0, 10, 0, 0, 0, 0,  0, 0, 10, 0, 0, 0, 0,  0, 0, 10};

// Measurement noise
arm_matrix_instance_f64 ekf_meas_noise;
float64_t ekf_meas_noise_buf[EKF_MEAS_COV_MATRIX_SIZE] = {
    100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 1000};

// Process noise
arm_matrix_instance_f64 ekf_process_noise;
float64_t ekf_process_noise_buf[EKF_COV_MATRIX_SIZE] = {4.771375e-07,
                                                        4.771375e-07,
                                                        4.771375e-07,
                                                        0,
                                                        0,
                                                        0,
                                                        4.771375e-07,
                                                        4.771375e-07,
                                                        4.771375e-07,
                                                        0,
                                                        0,
                                                        0,
                                                        4.771375e-07,
                                                        4.771375e-07,
                                                        4.771375e-07,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        6.773958e-05,
                                                        6.773958e-05,
                                                        6.773958e-05,
                                                        0,
                                                        0,
                                                        0,
                                                        6.773958e-05,
                                                        6.773958e-05,
                                                        6.773958e-05,
                                                        0,
                                                        0,
                                                        0,
                                                        6.773958e-05,
                                                        6.773958e-05,
                                                        6.773958e-05};

// Position and velocity
float64_t ekf_r_enu[3] = {0, 0, 0};
float64_t ekf_v_enu[3] = {0, 0, 0};

// Inertial gravity vector
float64_t ekf_g_inertial[3] = {0.0, 0.0, 9.80665};

// ========== Flight EKF Functions ========== //

/**
 * @brief assign globally-defined EKF parameters to the flight EKF structure
 */
void flight_ekf_init() {
  // Assign all the buffers and variables to the EKF instance
  ekf_covariance.numCols = EKF_COV_MATRIX_COLS;
  ekf_covariance.numRows = EKF_COV_MATRIX_ROWS;
  ekf_covariance.pData = ekf_covariance_buf;
  ekf.covariance = &ekf_covariance;

  ekf_meas_noise.numCols = EKF_MEAS_COV_MATRIX_COLS;
  ekf_meas_noise.numRows = EKF_MEAS_COV_MATRIX_ROWS;
  ekf_meas_noise.pData = ekf_meas_noise_buf;
  ekf.measurement_covariance = &ekf_meas_noise;

  ekf_process_noise.numCols = EKF_COV_MATRIX_COLS;
  ekf_process_noise.numRows = EKF_COV_MATRIX_ROWS;
  ekf_process_noise.pData = ekf_process_noise_buf;
  ekf.process_noise = &ekf_process_noise;

  ekf.r_enu = ekf_r_enu;
  ekf.v_enu = ekf_v_enu;

  ekf.r_enu = ekf_r_enu;
  ekf.v_enu = ekf_v_enu;

  ekf.g_inertial = ekf_g_inertial;

  ekf.time_prev = 0.0;
}

/**
 * @brief update EKF measurement covariance from GPS
 *
 * Given six-element (upper triangular matrix) GPS position covariance, update
 * the measurement covaraince in the position EKF structure.
 * @param ekf pointer to an EKF structure instance
 * @param gps_cov upper triangular part of GPS covariance (m^2)
 */
void flight_ekf_set_gps_covariance(struct ekf_instance *ekf,
                                   float32_t *gps_cov) {
  float64_t gps_cov_nn = (float64_t)gps_cov[0];
  float64_t gps_cov_ne = (float64_t)gps_cov[1];
  float64_t gps_cov_nd = (float64_t)gps_cov[2];
  float64_t gps_cov_ee = (float64_t)gps_cov[3];
  float64_t gps_cov_ed = (float64_t)gps_cov[4];
  float64_t gps_cov_dd = (float64_t)gps_cov[5];

  ekf->measurement_covariance->pData[0] = gps_cov_nn;
  ekf->measurement_covariance->pData[1] = gps_cov_ne;
  ekf->measurement_covariance->pData[2] = gps_cov_nd;
  ekf->measurement_covariance->pData[4] = gps_cov_ne;
  ekf->measurement_covariance->pData[5] = gps_cov_ee;
  ekf->measurement_covariance->pData[6] = gps_cov_ed;
  ekf->measurement_covariance->pData[8] = gps_cov_nd;
  ekf->measurement_covariance->pData[9] = gps_cov_ed;
  ekf->measurement_covariance->pData[10] = gps_cov_dd;
}

/**
 * @brief Update the flight Kalman filter
 *
 * From raw sensor readings, peform the update step of the flight positioning
 * extended Kalman filter.
 * @param r_ecef_gps pointer to raw GPS reading in ECEF frame (m)
 * @param baro raw barometric pressure reading (Pa)
 */
void flight_ekf_update(float32_t *r_ecef_gps, float32_t *gps_cov,
                       float64_t baro) {
  // Convert GPS from ECEF to NED
  float64_t gps_ecef[3] = {(float64_t)r_ecef_gps[0], (float64_t)r_ecef_gps[1],
                           (float64_t)r_ecef_gps[2]};
  float64_t gps_ned[3];
  gnc_ecef_to_ned(ground_station.lat, ground_station.lon, ground_station.r_ecef,
                  gps_ecef, gps_ned);

  // Set covariance from the gps
  flight_ekf_set_gps_covariance(&ekf, gps_cov);

  // Run the update step
  ekf_update(&ekf, gps_ned, baro);

  // Update flight data log
  position_flight_data.num_updates++;
  if (ekf.r_enu[2] * -1.0 > position_flight_data.max_alt) {
    position_flight_data.max_alt = ekf.r_enu[2] * -1.0;
  }
}

/**
 * @brief Predict the next state for the flight Kalman filter
 *
 * From raw sensor readings and an attitude estimate, predict the propagated
 * translational state using the position extended Kalman filter
 * @param accel measured body-frame acceleration (m/s^2)
 * @param attitude_matrix estimated inertial-to-body transformation matrix
 * @param dt integration time step
 */
void flight_ekf_predict(float64_t *accel,
                        arm_matrix_instance_f64 *attitude_matrix,
                        float64_t dt) {
  ekf_predict(&ekf, accel, attitude_matrix, dt);

  // Update flight data log
  position_flight_data.num_predicts++;
  if (ekf.r_enu[2] * -1.0 > position_flight_data.max_alt) {
    position_flight_data.max_alt = ekf.r_enu[2] * -1.0;
  }
}