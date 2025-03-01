#ifndef __ALTIMETRY_H__
#define __ALTIMETRY_H__

#include "dsp/arm_math.h"
#include <stdint.h>

uint8_t isa_get_layer(float64_t alt);
float64_t isa_pressure_from_alt(uint8_t layer, float64_t alt);
float64_t isa_sensitivity_from_alt(uint8_t layer, float64_t alt);

#endif /* __ALTIMETRY_H__ */