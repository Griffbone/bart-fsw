#include "pyro.h"
#include "gpio.h"

// Hardware configuration 
static const uint16_t FIRE_PINS[6] = {
    FIRE_1_Pin,
    FIRE_2_Pin, 
    FIRE_3_Pin, 
    FIRE_4_Pin, 
    FIRE_5_Pin,
    FIRE_6_Pin
};

GPIO_TypeDef *FIRE_PORTS[6] = {
    FIRE_1_GPIO_Port,
    FIRE_2_GPIO_Port,
    FIRE_3_GPIO_Port, 
    FIRE_4_GPIO_Port,
    FIRE_5_GPIO_Port,
    FIRE_6_GPIO_Port
};

static const uint16_t SENSE_PINS[6] = {
    SENSE_1_Pin, 
    SENSE2_Pin,         // TODO: fix this in cubemx
    SENSE_3_Pin, 
    SENSE_4_Pin,
    SENSE_5_Pin,
    SENSE_6_Pin
};

GPIO_TypeDef *SENSE_PORTS[6] = {
    SENSE_1_GPIO_Port,
    SENSE2_GPIO_Port,   // TODO: fix this in cubemx
    SENSE_3_GPIO_Port,
    SENSE_4_GPIO_Port,
    SENSE_5_GPIO_Port,
    SENSE_6_GPIO_Port
};

enum pyro_err pyro_init() {
    for (uint8_t i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(FIRE_PORTS[i], FIRE_PINS[i], GPIO_PIN_RESET);
    }

    return PYRO_ERR_OK;
}

enum pyro_err pyro_test() {
    uint8_t ret;

    for (uint8_t channel = 1; channel < 7; channel++) {
        ret = pyro_fire_channel(channel, 250);
        if (ret != PYRO_ERR_OK) {
            return PYRO_ERR_GENERAL;
        }
    }

    return PYRO_ERR_OK;
}

enum pyro_err pyro_fire_channel(uint8_t channel, uint16_t delay) {
    // Check for valid channel
    if ((channel == 0) || (channel > 6)) {
        return PYRO_ERR_INPUT;
    }

    // Fire channel
    HAL_GPIO_WritePin(FIRE_PORTS[channel - 1], FIRE_PINS[channel - 1], GPIO_PIN_SET);
    HAL_Delay(delay);
    HAL_GPIO_WritePin(FIRE_PORTS[channel - 1], FIRE_PINS[channel - 1], GPIO_PIN_RESET);

    return PYRO_ERR_OK;
}

enum pyro_err pyro_sense_channel(uint8_t channel) {
    // Check for valid channel
    if ((channel == 0) || (channel > 6)) {
        return PYRO_ERR_INPUT;
    }

    // Sense channel
    HAL_GPIO_ReadPin(SENSE_PORTS[channel - 1], SENSE_PINS[channel - 1]);

    return PYRO_ERR_OK;
}