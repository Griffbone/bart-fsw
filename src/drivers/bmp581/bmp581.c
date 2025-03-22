#include "bmp581.h"
#include "stm32h7xx_hal.h"

enum bmp581_err bmp581_init(struct bmp581_device *device);

enum bmp581_err bmp581_write_byte(struct bmp581_device *device, uint8_t reg, uint8_t data) {
    uint8_t ret;
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = data;

    ret = HAL_I2C_Master_Transmit(device->hi2c, device->i2c_addr << 1, &tx, 2, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
        return BMP581_ERR_HAL;
    }

    return BMP581_ERR_OK;
}

enum bmp581_err bmp581_read_byte(struct bmp581_device *device, uint8_t reg, uint8_t *data) {
    uint8_t rx;
    uint8_t ret;

    ret = HAL_I2C_Master_Transmit(device->hi2c, device->i2c_addr << 1, &reg, 1, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
      return BMP581_ERR_HAL;
    }

    ret = HAL_I2C_Master_Receive(device->hi2c, device->i2c_addr << 1, &rx, 1, HAL_MAX_DELAY);
  
    if (ret != HAL_OK) {
        return BMP581_ERR_HAL;
    }

    *data = rx;

    return BMP581_ERR_OK;
}