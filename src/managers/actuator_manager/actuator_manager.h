#ifndef __ACTUATOR_MANAGER_H__
#define __ACTUATOR_MANAGER_H__

#include "state_machine_impl/state_machine_impl.h"
#include <stdint.h>

enum actuator_manager_err {
  ACTUATOR_MANAGER_ERR_OK,
};

struct actuator_manager_instance {
  uint8_t continuity;

  // Pyro board I2C setup
  void *hi2c;
  uint8_t pyro_i2c_addr;
  uint32_t i2c_timeout_ms;
};

enum actuator_manager_err
actuator_manager_init(struct actuator_manager_instance *manager);
enum actuator_manager_err
actuator_manager_update(struct actuator_manager_instance *manager,
                        struct state_machine *state_machine);
enum actuator_manager_err
actuator_manager_fire_apogee(struct actuator_manager_instance *manager);

#endif /* __ACTUATOR_MANAGER_H__ */