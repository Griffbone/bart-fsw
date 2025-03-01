#include "icm45686.h"
#include "stm32h7xx_hal.h"

enum icm45686_err icm45686_soft_reset(struct icm45686_device *device) {
  enum icm45686_err status;

  // Trigger soft reset
  status = icm45686_write_byte(device, ICM45686_REG_MISC2, ICM45686_SOFT_RESET);
  if (status != ICM45686_ERR_OK) {
    return status;
  }

  // Wait 1ms 
  status = icm45686_delay_ms(1);
  
  return status;
}

enum icm45686_err icm45686_read_whoami(struct icm45686_device *device, uint8_t *whoami) {
  enum icm45686_err status = ICM45686_ERR_OK;

  // Read whoami register
  status = icm45686_read_byte(device, ICM45686_REG_WHO_AM_I, whoami);

  return status;
}

enum icm45686_err icm45686_init(struct icm45686_device *device) {
  enum icm45686_err status = ICM45686_ERR_OK;
  
  // Perform soft reset 
  status = icm45686_soft_reset(device);

  if (status != ICM45686_ERR_OK) {
    return status;
  }

  // Read whoami register 
  uint8_t whoami;
  status = icm45686_read_byte(device, ICM45686_REG_WHO_AM_I, &whoami);

  if (status != ICM45686_ERR_OK) {
    return status;
  }

  if (whoami != ICM45686_ERR_OK) {
    return ICM45686_ERR_VALUE;
  }

  // Set power modes
  uint8_t pwr_mgmt0 = device->gyro_pwr_mode | device->accel_pwr_mode;
  status = icm45686_write_byte(device, ICM45686_REG_PWR_MGMT0, pwr_mgmt0);

  if (status != ICM45686_ERR_OK) {
    return status;
  }

  // Set gyro config
  uint8_t gyro_config0 = device->gyro_fs | device->gyro_odr;
  status = icm45686_write_byte(device, ICM45686_REG_GYRO_CONFIG0, gyro_config0);

  if (status != ICM45686_ERR_OK) {
    return status;
  }

  // Set accelerometer config
  uint8_t accel_config0 = device->accel_fs | device->accel_odr;
  status = icm45686_write_byte(device, ICM45686_REG_ACCEL_CONFIG0, accel_config0);

  return status;
}

enum icm45686_err icm45686_read_accel(struct icm45686_device *device, double *accel_reading);

enum icm45686_err icm45686_read_gyro(struct icm45686_device *device, double *gyro_reading);

enum icm45686_err icm45686_read_temp(struct icm45686_device *device, double *temp);

enum icm45686_err icm45686_read_byte(struct icm45686_device *device, uint8_t addr, uint8_t *byte) {
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = (1 << 7) | (addr & 0x7F);
  tx[1] = 0x00;

  // TODO: add error handling
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(device->hspi, tx, rx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  *byte = rx[1];

  return ICM45686_ERR_OK;
}

enum icm45686_err icm45686_write_byte(struct icm45686_device *device, uint8_t addr, uint8_t byte) {
  uint8_t tx[2];

  tx[0] = addr & 0x7F;
  tx[1] = byte;

  // TODO: add error handling
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(device->hspi, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

  return ICM45686_ERR_OK;
}

enum icm45686_err icm45686_delay_ms(uint16_t delay) {
  // TODO: add error handling
  HAL_Delay(delay);

  return ICM45686_ERR_OK;
}

// enum iam20680ht_err iam20680ht_write_block(struct iam20680ht_device *device, uint8_t start_reg,
//                        uint8_t bytes, uint8_t *data) {
//   return IAM20680HT_ERR_OK;
// }

// enum iam20680ht_err iam20680ht_read_block(struct iam20680ht_device *device, uint8_t start_reg,
//                       uint8_t bytes, uint8_t *data) {
//   uint8_t rx[bytes + 1];
//   uint8_t tx[bytes + 1];

//   for (int i = 1; i <= bytes; i++) {
//     tx[i] = 0x00;
//   }

//   tx[0] = start_reg;

//   HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
//   HAL_SPI_TransmitReceive(device->hspi, tx, rx, bytes + 1, HAL_MAX_DELAY);
//   HAL_GPIO_WritePin(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

//   for (int i = 0; i < bytes; i++) {
//     data[i] = rx[i + 1];
//   }

//   return IAM20680HT_ERR_OK;
// }