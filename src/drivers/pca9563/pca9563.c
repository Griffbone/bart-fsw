#include "pca9563.h"
#include "stm32h7xx_hal.h"

pca9563_err_t pca9563_set_pin_mode(pca9563_device_t *device, uint8_t pin, uint8_t mode) {
    pca9563_err_t ret;

    // Read config register 
    uint8_t config_val; 
    ret = pca9563_read_byte(device, PCA9563_REG_CONFIG, &config_val);
    if (ret != PCA9563_ERR_OK) {
        return ret;
    }

    // Determine new config register
    if (mode == PCA9563_PIN_OUTPUT) {
        config_val &= ~pin;
    } 
    else if (mode == PCA9563_PIN_INPUT) {
        config_val |= pin;
    }
    else {
        return PCA9563_ERR_UNKNOWN_MODE;
    }

    // Write new config register 
    ret = pca9563_write_byte(device, PCA9563_REG_CONFIG, config_val);
    if (ret != PCA9563_ERR_OK) {
        return ret;
    }
    
    return PCA9563_ERR_OK;
}

pca9563_err_t pca9563_set_pin(pca9563_device_t *device, uint8_t pin, uint8_t value) {
    pca9563_err_t ret;

    // Read output port register
    uint8_t output_val; 
    ret = pca9563_read_byte(device, PCA9563_REG_OUTPUT_PORT, &output_val);
    if (ret != PCA9563_ERR_OK) {
        return ret;
    }

    // Determine new output port register
    if (value == PCA9563_PIN_HIGH) {
        output_val |= pin;
    } 
    else if (value == PCA9563_PIN_LOW) {
        output_val &= ~pin;
    }
    else {
        return PCA9563_ERR_UNKNOWN_VALUE;
    }

    // Write new output port value
    ret = pca9563_write_byte(device, PCA9563_REG_OUTPUT_PORT, output_val);
    if (ret != PCA9563_ERR_OK) {
        return ret;
    }
    
    return PCA9563_ERR_OK;
}

// // TODO: expand this functionality
// pca9563_err_t pca9563_read_pin(pca9563_device_t *device, uint8_t pin, uint8_t *value) {
//     return PCA9563_ERR_OK;
// }

pca9563_err_t pca9563_read_byte(pca9563_device_t *device, uint8_t reg, uint8_t *byte) {
    uint8_t ret;

    // Write-address device and send command byte (address to read)
    ret = HAL_I2C_Master_Transmit(device->hi2c, PCA9563_I2C_ADDR << 1, &reg, 1, device->timeout);
    if (ret != HAL_OK) {
        return PCA9563_ERR_HAL;
    }

    // Read-address device and read a byte
    ret = HAL_I2C_Master_Receive(device->hi2c, PCA9563_I2C_ADDR << 1, byte, 1, device->timeout);
    if (ret != HAL_OK) {
        return PCA9563_ERR_HAL;
    }

    return PCA9563_ERR_OK;
}

pca9563_err_t pca9563_write_byte(pca9563_device_t *device, uint8_t reg, uint8_t byte) {
    uint8_t ret;

    uint8_t tx[2]; 
    tx[0] = reg;
    tx[1] = byte;

    ret = HAL_I2C_Master_Transmit(device->hi2c, PCA9563_I2C_ADDR << 1, tx, 2, device->timeout);
    if (ret != HAL_OK) {
        return PCA9563_ERR_HAL;
    }

    return PCA9563_ERR_OK;
}