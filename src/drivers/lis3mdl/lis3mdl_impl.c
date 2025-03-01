#include "lis3mdl.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Write to a single LIS3MDL register
 *
 * This function performs a single register write on a LIS2MDL device though the
 * STM32 SPI HAL.
 * @param device pointer to a lis3mdl_device structure
 * @param reg register to write to
 * @param data data to write to register
 */
enum lis3mdl_err lis3mdl_write_byte(struct lis3mdl_device *device, uint8_t reg,
                                    uint8_t data) {
  uint8_t tx[2] = {reg, data};

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(device->hspi, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  return LIS3MDL_ERR_OK;
}

/**
 * @brief Read a single LIS3MDL register
 *
 * This function performs a single register read on a LIS3MDL device through the
 * STM32 SPI HAL.
 * @param device pointer to a lis3mdl_device structure
 * @param reg register to read
 * @param data pointer to buffer to store read byte
 */
enum lis3mdl_err lis3mdl_read_byte(struct lis3mdl_device *device, uint8_t reg,
                                   uint8_t *data) {
  uint8_t tx[2] = {0x80 | reg, 0x00};
  uint8_t rx[2];

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(device->hspi, tx, rx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  *data = rx[1];
  return LIS3MDL_ERR_OK;
}

/**
 * @brief Write to multile LIS3MDL registers
 *
 * This function perfomrs a multiple-write on an LIS3MDL device through the
 *STM32 SPI HAL. The function takes in the first register and uses the
 *auto-incrementation function of the LIS3MDL to write to consecutive registers
 * @param device pointer to a lis3mdl_device structure
 * @param start_reg first register to write to
 * @param bytes number of bytes to write
 * @param data pointer to buffer with data to write
 * @warning no error checking is performed. Make sure to allocate appropriate
 *buffer sizes for all inputs.
 **/
enum lis3mdl_err lis3mdl_write_block(struct lis3mdl_device *device,
                                     uint8_t start_reg, uint8_t bytes,
                                     uint8_t *data) {
  uint8_t tx[bytes + 1];

  for (int i = 1; i <= bytes; i++) {
    tx[i] = data[i - 1];
  }

  tx[0] = 0x40 | start_reg;

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(device->hspi, tx, bytes + 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  return LIS3MDL_ERR_OK;
}

/**
 * @brief Read multiple LIS3MDL registers
 *
 * This function performs a mulitple-read on an LIS3MDL device through the STM32
 * SPI HAL. The function takes in the first register and uses the
 * auto-incrementation function of the LIS3MDL to read consecutive registers.
 * @param device pointer to a lis3mdl_device structure
 * @param start_reg first register to read
 * @param bytes number of consecutive registers to read
 * @param data pointer to buffer to store read bytes
 * @warning no error checking is performed. Make sure to allocate appropriate
 * buffer sizes for all inputs.
 */
enum lis3mdl_err lis3mdl_read_block(struct lis3mdl_device *device,
                                    uint8_t start_reg, uint8_t bytes,
                                    uint8_t *data) {
  uint8_t tx[bytes + 1];
  uint8_t rx[bytes + 1];

  for (int i = 1; i <= bytes; i++) {
    tx[i] = 0x00;
  }

  tx[0] = 0xC0 | start_reg;

  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(device->hspi, tx, rx, bytes + 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  for (int i = 0; i < bytes; i++) {
    data[i] = rx[i + 1];
  }

  return LIS3MDL_ERR_OK;
}