#include "stm32h7xx_hal.h"
#include "ublox_gnss.h"

enum ublox_gnss_err ublox_gnss_send_msg(struct ublox_gnss_device *device,
                                        const uint8_t *buffer, uint16_t size) {
  switch (device->transport_type) {
  case UBLOX_GNSS_TRANSPORT_UART:
    HAL_UART_Transmit(device->transport_handle.uart, buffer, size,
                      HAL_MAX_DELAY);
    break;
  default:
    break;
  }

  return UBLOX_GNSS_ERR_OK;
}
