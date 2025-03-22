/* Driver for pyro circuits
  Driver for pyro circuitry on the BART control board 
*/

#ifndef __PYRO__H__
#define __PYRO__H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

enum pyro_err {
  PYRO_ERR_OK, 
  PYRO_ERR_HAL,
  PYRO_ERR_INPUT,
  PYRO_ERR_GENERAL,
};

enum pyro_err pyro_init();
enum pyro_err pyro_test(struct pyro_device *device);
enum pyro_err pyro_fire_channel(uint8_t channel, uint16_t delay);
enum pyro_err pyro_sense_channel(uint8_t channel);

#endif /* __PYRO_H__ */