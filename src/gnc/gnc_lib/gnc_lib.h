#ifndef __GNC_LIB__
#define __GNC_LIB__

#include "dsp/arm_math.h"

#define PI_F 3.1415926535897932384626433832795

enum gnc_err {
  GNC_ERR_OK,
};

/* Quaternion Functions */
enum gnc_err quaternion_multiplication(float64_t *qa, float64_t *qb,
                                       float64_t *qc);

enum gnc_err quaternion_norm(float64_t *qa, float64_t *qb);

enum gnc_err quaternion_propagate_delta_angle();
enum gnc_err quaternion_propagate_omega(float64_t *q0, float64_t *omega,
                                        float64_t dt, float64_t *qf);
enum gnc_err quaternion_to_rotation(float64_t *quaternion,
                                    arm_matrix_instance_f64 *rotation_matrix);

/* Rotation matrix functions */
enum gnc_err rotation_1(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix);
enum gnc_err rotation_2(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix);
enum gnc_err rotation_3(float64_t theta,
                        arm_matrix_instance_f64 *rotation_matrix);

enum gnc_err skew_symmetric(float64_t *vector, float64_t *vector_skew);

enum gnc_err apply_rotation(arm_matrix_instance_f64 *rotation_matrix,
                            float64_t *vector, float64_t *vector_transformed);

enum gnc_err vec_subtract(float64_t *vector_1, float64_t *vector_2, uint8_t len,
                          float64_t *vector_3);

enum gnc_err vec_add(float64_t *vector_1, float64_t *vector_2, uint8_t len,
                     float64_t *vector_3);

enum gnc_err gnc_ecef_to_ned(float64_t lat_ref, float64_t lon_ref,
                             float64_t *r_ref_ecef, float64_t *r_ecef,
                             float64_t *r_ned);

#endif /* __GNC_LIB__ */