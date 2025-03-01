#ifndef __QMEKF_IMPL_H__
#define __QMEKF_IMPL_H__

#include "dsp/arm_math.h"
#include "gnc/qmekf/qmekf.h"

struct qmekf_hist_instance {
  uint32_t num_updates;
  uint32_t num_predicts;
};

void flight_qmekf_init();

void flight_qmekf_update(float64_t *mag, float64_t *accel);

void flight_qmekf_predict(float64_t *omega, float64_t dt);

#endif /* __QMEKF_IMPL_H__ */