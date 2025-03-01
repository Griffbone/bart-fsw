#ifndef __PYRO_BOARD_H__
#define __PYRO_BOARD_H__

#include <stdint.h>

enum pyro_board_err {
  PYRO_BOARD_ERR_OK,
  PYRO_BOARD_ERR_WRITE,
  PYRO_BOARD_ERR_READ,
};

struct pyro_board_device {
  void *hi2c;
  uint8_t i2c_address;
  uint32_t i2c_timeout_ms;
};

enum pyro_board_err pyro_board_fire_channel(struct pyro_board_device *device,
                                            uint8_t channel);
enum pyro_board_err pyro_board_read_continuity(struct pyro_board_device *device,
                                               uint8_t *continuity);

enum pyro_board_err pyro_board_write_byte(struct pyro_board_device *device,
                                          uint8_t bvte);
enum pyro_board_err pyro_board_read_byte(struct pyro_board_device *device,
                                         uint8_t *byte);

#endif /* __PYRO_BOARD_DRIVER_H__ */
