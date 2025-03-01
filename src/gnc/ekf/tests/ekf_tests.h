#ifndef __EKF_TESTS_H__
#define __EKF_TESTS_H__

#include <stdint.h>

uint8_t test_ekf_get_stm();
uint8_t test_ekf_get_sens();
uint8_t test_ekf_get_residual();
// uint8_t test_ekf_set_gps_covariance();
uint8_t test_ekf_update();
uint8_t test_ekf_predict();

#endif /* __EKF_TESTS_H__ */