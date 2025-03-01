#ifndef __QMEKF_TESTS_H__
#define __QMEKF_TESTS_H__

#include "dsp/arm_math.h"
#include "gnc/test_utils/test_utils.h"
#include <stdint.h>

void print_matrix(float64_t *matrix, uint16_t rows, uint16_t cols);

uint8_t test_qmekf_get_stm();
uint8_t test_qmekf_get_sens();
uint8_t test_qmekf_get_resisdual();
uint8_t test_qmekf_update();
uint8_t test_qmekf_predict();

#endif /* __QMEKF_TESTS_H__ */