#include "pyro_board.h"

enum pyro_board_err pyro_board_fire_channel(struct pyro_board_device *device,
                                            uint8_t channel) {
  pyro_board_write_byte(device, channel);
  return PYRO_BOARD_ERR_OK;
}

enum pyro_board_err pyro_board_read_continuity(struct pyro_board_device *device,
                                               uint8_t *continuity) {
  uint8_t ret = PYRO_BOARD_ERR_OK;
  ret = pyro_board_read_byte(device, continuity);
  return ret;
}

__attribute__((weak)) enum pyro_board_err
pyro_board_write_byte(struct pyro_board_device *device, uint8_t byte) {
  return PYRO_BOARD_ERR_OK;
}

__attribute__((weak)) enum pyro_board_err
pyro_board_read_byte(struct pyro_board_device *device, uint8_t *byte) {
  return PYRO_BOARD_ERR_OK;
}
