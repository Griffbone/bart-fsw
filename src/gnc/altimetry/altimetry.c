/**
 * @file altimetry.c
 * @brief Functions used in dealing with barometric pressure and altitude data.
 *
 * This file contains source code for converting between pressure and altitude,
 * and determining the pressure-altitude sensitivity for Kalman filtering.
 *
 * @author Griffin Jourda
 */

#include "altimetry.h"
#include <math.h>

/// ISA temperature lapse rates (K/m)
static const float64_t ISA_LAPSE_RATES[8] = {-0.0065, 0,       0.001,  0.0028,
                                             0,       -0.0028, -0.002, 0};

/// ISA atmosphere layefloat64_tr base altitudes (m)
static const float64_t ISA_BASE_ALTS[8] = {0,     11000, 20000, 32000,
                                           47000, 51000, 71000, 84852};

/// ISA atmosphere layer base temperatures (K)
static const float64_t ISA_BASE_TEMPS[8] = {288.1500, 216.6500, 216.6500,
                                            228.6500, 270.6500, 270.6500,
                                            214.6500, 186.9460};

/// ISA atmosphere layer base pressures (Pa)
static const float64_t ISA_BASE_PRESS[8] = {
    101325,         22632.0405969347, 5474.8776606600, 868.0158377494,
    110.9057845539, 66.9385353730,    3.9563927546,    0.3733803710};

/// ISA air specific gas constant
static const float64_t ISA_R = 287.05287;

/// ISA gravitational constant (m/s^2)
static const float64_t ISA_G = 9.80665;

/**
 * @brief Get information on atmospheric layer given altitude
 *
 * This function determines which ISA atospheric layer a given altitude is in.
 * This gives the index to retrieve base temperature, pressure, etc. from the
 * lookup tables defind in this file.
 * @param alt altitude (m)
 * @return atmospheric layer index
 * @note Altitudes below 0 km are taken to be in the first (0 to 11 km) layer
 * and altitudes above 84.852 km are taken to be in the eighth (84852 km +
 * layer).
 * @warning No error checking is performed
 */
uint8_t isa_get_layer(float64_t alt) {
  // Determine which layer of the ISA we are in
  uint8_t layer = 0;

  if (alt < 0) {
    layer = 0;
  } else if (alt > ISA_BASE_ALTS[7]) {
    layer = 7;
  } else {
    for (uint8_t i = 0; i < 7; i++) {
      if ((ISA_BASE_ALTS[i] <= alt) & (ISA_BASE_ALTS[i + 1] >= alt)) {
        layer = i;
        break;
      }
    }
  }

  return layer;
}

/**
 * @brief Get atmospheric pressure from altitude
 *
 * This function is used to determine the static pressure (in Pascals) at a
 * given altitude above sea level (in meters).
 * @param layer atmospheric layer number for lookup tables
 * @param alt altitude (m)
 * @return barometric (Pa)
 * @note This function is accurate for altitudes between 0 and 84.852 km. Lower
 * than zero km and above 84.852 km yields computations based on data for the
 * highest or lowest atmospheric layer and may not be entirely accurate.
 * @warning No error checking is perfomrmed
 */
float64_t isa_pressure_from_alt(uint8_t layer, float64_t alt) {
  // Get base conditions and lapse rate
  float64_t h_base = ISA_BASE_ALTS[layer];
  float64_t t_base = ISA_BASE_TEMPS[layer];
  float64_t p_base = ISA_BASE_PRESS[layer];
  float64_t lapse = ISA_LAPSE_RATES[layer];

  // Calculate temperature and pressure
  float64_t temp;
  float64_t press;

  if (lapse == 0) {
    temp = t_base;
    press = p_base * exp((-ISA_G * (alt - h_base)) / (ISA_R * temp));
  } else {
    temp = t_base + lapse * (alt - h_base);
    press = p_base * pow(temp / t_base, -ISA_G / (lapse * ISA_R));
  }

  return press;
}

/**
 * @brief Get pressure sensitivity from altitude
 *
 * This function is used to determine the pressure sensitivity (Pa/m) at a given
 * altitue above sea level. This is used for the measurement sensitivity for
 * barometric pressure observation model.
 * @param layer atmospheric layer number for lookup tables
 * @param alt altitude (m)
 * @return pressure altitude sensitivity (Pa/m)
 * @note This function is accurate for altitudes between 0 and 84.852 km. Lower
 * than zero km and above 84.852 km yields computations based on data for the
 * highest or lowest atmospheric layer and may not be entirely accurate.
 * @warning No error checking is perfomrmed
 */
float64_t isa_sensitivity_from_alt(uint8_t layer, float64_t alt) {
  // Get base conditions and lapse rate
  float64_t h_base = ISA_BASE_ALTS[layer];
  float64_t t_base = ISA_BASE_TEMPS[layer];
  float64_t p_base = ISA_BASE_PRESS[layer];
  float64_t lapse = ISA_LAPSE_RATES[layer];

  // Compute pressure/altitude sensitivity
  float64_t H = 0;

  if (lapse == 0) {
    H = -(ISA_G * p_base * exp(-(ISA_G * (alt - h_base)) / (ISA_R * t_base))) /
        (ISA_R * t_base);
  } else {
    H = -(ISA_G * p_base) / (ISA_R * t_base *
                             pow(((t_base + lapse * (alt - h_base)) / t_base),
                                 (ISA_G / (ISA_R * lapse) + 1.0)));
  }
  /*
  if (lapse == 0) {
          H = (ISA_G*p_base*exp(-(ISA_G*(alt -
  h_base))/(ISA_R*t_base))/(ISA_R*t_base)); } else { H =
  -(ISA_G*p_base)/(ISA_R*t_base*pow(((t_base + lapse*(alt - h_base))/t_base),
  (ISA_G/(ISA_R*lapse) + 1.0)));
  }
  */

  return H;
}