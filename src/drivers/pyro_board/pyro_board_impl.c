#include "pyro_board.h"
#include "stm32h7xx_hal.h"

enum pyro_board_err pyro_board_write_byte(struct pyro_board_device *device,
                                          uint8_t byte) {
  HAL_I2C_Master_Transmit(device->hi2c, device->i2c_address << 1, &byte, 1,
                          HAL_MAX_DELAY);

  return PYRO_BOARD_ERR_OK;
}

enum pyro_board_err pyro_board_read_byte(struct pyro_board_device *device,
                                         uint8_t *byte) {
  uint8_t rx;
  uint8_t ret;
  ret = HAL_I2C_Master_Receive(device->hi2c, device->i2c_address << 1, &rx, 1,
                         HAL_MAX_DELAY);

  if (ret != HAL_OK) {
    return PYRO_BOARD_ERR_READ;
  }

  *byte = rx;

  return PYRO_BOARD_ERR_OK;
}
