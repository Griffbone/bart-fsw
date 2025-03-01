#include "iam20680ht.h"
#include "stm32h7xx_hal.h"

enum iam20680ht_err iam20680ht_delay(uint16_t delay_ms) {
  HAL_Delay(delay_ms);

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_write_byte(struct iam20680ht_device *device,
                                          uint8_t addr, uint8_t byte) {
  uint8_t tx[2];

  tx[0] = addr & 0x7F;
  tx[1] = byte;

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(device->hspi, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_read_byte(struct iam20680ht_device *device,
                                         uint8_t addr, uint8_t *byte) {
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = (1 << 7) | (addr & 0x7F);
  tx[1] = 0x00;

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(device->hspi, tx, rx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  *byte = rx[1];

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_write_block(struct iam20680ht_device *device, uint8_t start_reg,
                       uint8_t bytes, uint8_t *data) {
  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_read_block(struct iam20680ht_device *device, uint8_t start_reg,
                      uint8_t bytes, uint8_t *data) {
  uint8_t rx[bytes + 1];
  uint8_t tx[bytes + 1];

  for (int i = 1; i <= bytes; i++) {
    tx[i] = 0x00;
  }

  tx[0] = start_reg;

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(device->hspi, tx, rx, bytes + 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  for (int i = 0; i < bytes; i++) {
    data[i] = rx[i + 1];
  }

  return IAM20680HT_ERR_OK;
}