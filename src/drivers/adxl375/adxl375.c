#include "adxl375.h"
#include "stm32h7xx_hal.h"

enum adxl375_err adxl375_read_byte(struct adxl375_device *device, uint8_t reg, uint8_t *data) {
    uint8_t ret;
    uint8_t tx = 0x80 | reg;

    HAL_GPIO_SET(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
    
    ret = HAL_SPI_Transmit(device->hspi, &tx, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ADXL375_ERR_HAL;
    }

    ret = HAL_SPI_Receive(device->hspi, data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ADXL375_ERR_HAL;
    }

    HAL_GPIO_SET(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);

    return ADXL375_ERR_OK;
}

enum adxl375_err adxl375_write_byte(struct adxl375_device *device, uint8_t reg, uint8_t data) {
    uint8_t ret;
    uint8_t tx[2] = {reg, data};

    HAL_GPIO_SET(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_RESET);
    
    ret = HAL_SPI_Transmit(device->hspi, tx, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ADXL375_ERR_HAL;
    }

    ret = HAL_GPIO_SET(device->cs_gpio_port, device->cs_gpio_pin, GPIO_PIN_SET);
    if (ret != HAL_OK) {
        return ADXL375_ERR_HAL;
    }

    return ADXL375_ERR_OK;
}