#ifndef __EKF_IMPL_H__
#define __EKF_IMPL_H__

#include "dsp/arm_math.h"
#include "gnc/ekf/ekf.h"

struct ground_station_instance {
  float64_t lat;
  float64_t lon;
  float64_t alt;
  float64_t r_ecef[3];
};

struct flight_hist_data_instance {
  float32_t max_alt;
  uint32_t num_updates;
  uint32_t num_predicts;
};

void flight_ekf_init();

void flight_ekf_set_gps_covariance(struct ekf_instance *ekf,
                                   float32_t *gps_cov);

void flight_ekf_update(float32_t *r_ecef_gps, float32_t *gps_cov,
                       float64_t baro);

void flight_ekf_predict(float64_t *accel,
                        arm_matrix_instance_f64 *attitude_matrix, float64_t dt);

#endif /* __EKF_IMPL_H__ */