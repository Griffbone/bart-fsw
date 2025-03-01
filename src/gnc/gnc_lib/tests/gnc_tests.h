#ifndef __GNC_TESTS_H__
#define __GNC_TESTS_H__

#include <stdint.h>

uint8_t test_quaternion_multiplication();
uint8_t test_quaternion_propagate_delta_angle();
uint8_t test_quaternion_propagate_omega();
uint8_t test_quaternion_to_rotation();
uint8_t test_rotation_1();
uint8_t test_rotation_2();
uint8_t test_rotation_3();
uint8_t test_skew_symmetric();
uint8_t test_apply_rotation();
uint8_t test_vec_subtract();
uint8_t test_vec_add();
uint8_t test_ecef_to_ned();

#endif /* __GNC_TESTS_H__ */