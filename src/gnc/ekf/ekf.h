#ifndef __EKF_H__
#define __EKF_H__

#include "dsp/arm_math.h"

#define EKF_NUM_STATES 6

#define EKF_COV_MATRIX_SIZE 36
#define EKF_COV_MATRIX_COLS 6
#define EKF_COV_MATRIX_ROWS 6

#define EKF_MEAS_SENS_SIZE 24
#define EKF_MEAS_SENS_COLS 6
#define EKF_MEAS_SENS_ROWS 4

#define EKF_MEAS_SIZE 4
#define EKF_MEAS_COLS 1
#define EKF_MEAS_ROWS 4

#define EKF_MEAS_COV_MATRIX_SIZE 16
#define EKF_MEAS_COV_MATRIX_COLS 4
#define EKF_MEAS_COV_MATRIX_ROWS 4

enum ekf_err {
  EKF_ERR_OK,
};

struct ekf_instance {
  // State variables
  float64_t *r_enu;
  float64_t *v_enu;
  arm_matrix_instance_f64 *covariance;

  // Constants/tuning
  arm_matrix_instance_f64 *measurement_covariance;
  arm_matrix_instance_f64 *process_noise;

  float64_t *g_inertial;

  // Timekeeping
  float64_t time_prev;
};

enum ekf_err ekf_get_stm(float64_t dt, arm_matrix_instance_f64 *phi);

enum ekf_err ekf_get_sens(float64_t *r_enu, arm_matrix_instance_f64 *h_matrix);

enum ekf_err ekf_get_residual(float64_t *r_enu, float64_t *r_enu_meas,
                              float64_t baro_meas,
                              arm_matrix_instance_f64 *residual);

enum ekf_err ekf_update(struct ekf_instance *ekf, float64_t *r_enu_meas,
                        float64_t baro_meas);

enum ekf_err ekf_predict(struct ekf_instance *ekf, float64_t *accel,
                         arm_matrix_instance_f64 *attitude_matrix,
                         float64_t dt);

#endif /* __EKF_H__ */