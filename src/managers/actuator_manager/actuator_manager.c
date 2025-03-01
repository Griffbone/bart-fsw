#include "actuator_manager.h"
#include "drivers/pyro_board/pyro_board.h"
#include "stm32h7xx_hal.h"

#include "cli/cli.h"
extern struct cli_handle cli;

struct pyro_board_device pyro_board;

/**
 * @brief initialize pyro board.
 *
 * Initializes the actuator manager for Strange Magic. This is a
 * hardware-specific implementation, since it uses the i2c address of the
 * support board MCU.
 * @param manager pointer to actuator manager struct
 * @return enum containing the status of the function.
 */
enum actuator_manager_err
actuator_manager_init(struct actuator_manager_instance *manager) {
  // Initialize pyro board structure
  pyro_board.hi2c = manager->hi2c;
  pyro_board.i2c_address = manager->pyro_i2c_addr;
  pyro_board.i2c_timeout_ms = manager->i2c_timeout_ms;

  manager->continuity = 0xFF;

  return ACTUATOR_MANAGER_ERR_OK;
}

/**
 * @brief update step for the actuator manager
 *
 * Reads continuity from the pyro board and updates the actuator_manager struct.
 * If in apogee state, fire pyros.
 * @param manager pointer to actuator manager struct
 * @return enum containing the status of the function.
 * @warning This function will fire pyros under certain conditions!
 */
enum actuator_manager_err
actuator_manager_update(struct actuator_manager_instance *manager,
                        struct state_machine *state_machine) {
  uint8_t ret;
  ret = pyro_board_read_continuity(&pyro_board, &(manager->continuity));
  /* ***** */
  cli_transmit(&cli, "pyro continuity: %02x\r\n", manager->continuity);
  cli_transmit(&cli, "continuity ret: %d\r\n", ret);

  if (ret == PYRO_BOARD_ERR_READ) {
    manager->continuity = 0xFF;
  }

  if (state_machine->state == STATE_MACHINE_STATE_APOGEE) {
    actuator_manager_fire_apogee(manager);
    state_machine->signal = STATE_MACHINE_SIGNAL_DESCENT_DETECTED;
  }
  
  return ACTUATOR_MANAGER_ERR_OK;
}

/**
 * @brief fire apogee charges for Strange Magic
 *
 * Fires all e-matches (1, 2, 3, and 4) for apogee.
 * @param manager pointer to actuator manager struct
 * @return enum containing the status of the function
 * @warning this is a harness-specific implementation for Strange Magic only!
 */
enum actuator_manager_err
actuator_manager_fire_apogee(struct actuator_manager_instance *manager) {
  for (uint8_t i = 1; i < 5; i++) {
    pyro_board_fire_channel(&pyro_board, i);
  }

  return ACTUATOR_MANAGER_ERR_OK;
}
