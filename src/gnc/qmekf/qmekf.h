#ifndef __QMEKF_H__
#define __QMEKF_H__

#include "dsp/arm_math.h"
#define QMEKF_PHI_MATRIX_SIZE 36

enum qmekf_err {
  QMEKF_ERR_OK,
};

struct qmekf_instance {
  // State variables
  arm_matrix_instance_f64 *covariance;
  arm_matrix_instance_f64 *attitude_matrix;
  float64_t *quaternion;
  float64_t *gyro_bias;

  // Constants/tuning
  arm_matrix_instance_f64 *measurement_covariance;
  arm_matrix_instance_f64 *process_noise;

  // Environment variables
  float64_t *mag_vec_inertial;
  float64_t *g_vec_inertial;

  // Timekeeping
  float64_t time_prev;
};

enum qmekf_err qmekf_get_stm(float64_t *omega_hat, float64_t dt,
                             arm_matrix_instance_f64 *phi);

enum qmekf_err qmekf_get_sens(arm_matrix_instance_f64 *attitude_matrix,
                              float64_t *b_inertial, float64_t *g_inertial,
                              arm_matrix_instance_f64 *h_matrix);

enum qmekf_err qmekf_get_residual(arm_matrix_instance_f64 *attitude_matrix,
                                  float64_t *b_inertial, float64_t *g_inertial,
                                  float64_t *b_meas, float64_t *g_meas,
                                  arm_matrix_instance_f64 *residual);

enum qmekf_err qmekf_update(struct qmekf_instance *qmekf, float64_t *b_meas,
                            float64_t *g_meas);

enum qmekf_err qmekf_predict(struct qmekf_instance *qmekf,
                             float64_t *omega_meas, float64_t dt);

#endif /* __QMEKF_H__ */