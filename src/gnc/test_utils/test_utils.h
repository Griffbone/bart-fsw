#ifndef __TEST_UTILS_H__
#define __TEST_UTILS_H__

#include "dsp/arm_math.h"
#include <stdint.h>

void print_matrix(float64_t *matrix, uint16_t rows, uint16_t cols);
uint8_t compare(float64_t *buf_1, float64_t *buf_2, uint16_t num_el,
                float64_t tol);

#endif /* __TEST_UTILS_H__ */