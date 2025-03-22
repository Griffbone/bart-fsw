#include "bmp581.h"
#include "stm32h7xx_hal.h"

enum bmp581_err bmp581_init(struct bmp581_device *device) {
    // TODO: error handling

    // Recommended power-up procedure from page 18 of the datasheet
    uint8_t buf;

    // Wait t_powerup
    bmp581_delay(3);

    // Soft reset
    bmp581_soft_reset(device);

    // Check chip ID is not zero
    bmp581_read_byte(device, BMP581_REG_CHIP_ID, &buf);

    if (buf == 0) {
        return BMP581_ERR_GENERAL;
    }
    
    // Check status register
    bmp581_read_byte(device, BMP581_REG_CHIP_STATUS, &buf);
    uint8_t status_nvm_ready = (buf >> 1) && 0x01;
    uint8_t status_nvm_err = (buf >> 2) && 0x01;
    
    if (status_nvm_ready == 0 || status_nvm_err == 1) {
        return BMP581_ERR_GENERAL;
    }

    // Read interrupt status register
    bmp581_read_byte(device, BMP581_REG_INT_STATUS, &buf);
    uint8_t int_status_por = (buf >> 4) && 0x01;

    if (int_status_por != 1) {
        return buf;
    }

    // Enter standby mode 
    bmp581_write_byte(device, BMP581_REG_ODR_CONFIG, BMP581_PWR_STANDBY);
    bmp581_delay(3);

    // Set measurement configuration
    // Enable pressure measurment
    bmp581_write_byte(device, BMP581_REG_OSR_CONFIG, 1 << 6);
    // TODO: SET ODR CONFIGURATION HERE

    // Enter normal mode 
    bmp581_write_byte(device, BMP581_REG_ODR_CONFIG, BMP581_PWR_NORMAL);

    return BMP581_ERR_OK;    
}

enum bmp581_err bmp581_soft_reset(struct bmp581_device *device) {
    // TODO: add error handling
    bmp581_write_byte(device, BMP581_REG_CMD, BMP581_COMMAND_SOFT_REST);
    bmp581_delay(2);

    return BMP581_ERR_OK;
}

enum bmp581_err bmp581_read_temp(struct bmp581_device *device, float *temp) {
    uint8_t buf[3]; 
    int32_t temp_raw; 

    bmp581_read_block(device, BMP581_REG_TEMP_XLSB, 3, buf);

    temp_raw = ((buf[2] << 16) | (buf[1] << 8) | buf[0]);
    *temp = temp_raw/65536.0f;
    
    return BMP581_ERR_OK;
}

enum bmp581_err bmp581_read_press(struct bmp581_device *device, float *press) {
    uint8_t buf[3];
    int32_t press_raw;

    bmp581_read_block(device, BMP581_REG_PRESS_XLSB, 3, buf);

    press_raw = ((buf[2] << 16) | (buf[1] << 8) | buf[0]);
    *press = press_raw/64.0f;

    return BMP581_ERR_OK;
}

enum bmp581_err bmp581_write_byte(struct bmp581_device *device, uint8_t reg, uint8_t data) {
    uint8_t ret;
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = data;

    ret = HAL_I2C_Master_Transmit(device->hi2c, device->i2c_addr << 1, tx, 2, HAL_MAX_DELAY);

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

enum bmp581_err bmp581_read_block(struct bmp581_device *device, uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    uint8_t ret;

    ret = HAL_I2C_Mem_Read(device->hi2c, device->i2c_addr << 1, start_reg, 1, data, bytes, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
        return BMP581_ERR_HAL;
    }

    return BMP581_ERR_OK;
}

enum bmp581_err bmp581_delay(uint32_t ms) {
    HAL_Delay(ms);

    return BMP581_ERR_OK;
}