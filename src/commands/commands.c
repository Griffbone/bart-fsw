#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "system/gpio.h"

#include "drivers/pca9563/pca9563.h"
#include "drivers/pyro/pyro.h"

extern pca9563_device_t io_expander;

uint8_t command_init(struct cli_handle *cli) {
  cli_add_command(cli, "help", "", command_help);
  cli_add_command(cli, "clear", "", command_clear);
  cli_add_command(cli, "led", "", command_led_toggle);
  cli_add_command(cli, "fire", "", command_pyro_fire);
  cli_add_command(cli, "sense", "", command_pyro_sense);

  return 0;
}

uint8_t command_help(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  for (uint16_t i = 0; i < CLI_MAXIMUM_NUMBER_OF_COMMANDS; i++) {
    if (strcmp(cli->commands[i].command, "") == 0) {
      break;
    }

    cli_transmit(cli, "%s\r\n", cli->commands[i].command);
  }

  return 0;
}

uint8_t command_clear(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  cli_transmit(cli, "\033[2J");
  cli_transmit(cli, "\033[1;1H");

  return 0;
}

uint8_t command_led_toggle(struct cli_handle *cli, uint16_t argc,
                           char *argv[]) {
  if (argc != 3) {
    return 1;
  }

  switch (atoi(argv[1])) {
    case 1:
      if (strcmp(argv[2], "on") == 0) {
        HAL_GPIO_WritePin(USR_LED_1_GPIO_Port, USR_LED_1_Pin, GPIO_PIN_SET);
      }
      else if (strcmp(argv[2], "off") == 0) {
        HAL_GPIO_WritePin(USR_LED_1_GPIO_Port, USR_LED_1_Pin, GPIO_PIN_RESET);
      }
      else {
        return 1;
      }
      break; 
    case 2:
      if (strcmp(argv[2], "on") == 0) {
        HAL_GPIO_WritePin(USR_LED_2_GPIO_Port, USR_LED_2_Pin, GPIO_PIN_SET);
      }
      else if (strcmp(argv[2], "off") == 0) {
        HAL_GPIO_WritePin(USR_LED_2_GPIO_Port, USR_LED_2_Pin, GPIO_PIN_RESET);
      }
      else {
        return 1;
      }
      break; 
    case 3: 
      if (strcmp(argv[2], "on") == 0) {
        pca9563_set_pin(&io_expander, PCA9563_PIN_P3, PCA9563_PIN_HIGH);
      }
      else if (strcmp(argv[2], "off") == 0) {
        pca9563_set_pin(&io_expander, PCA9563_PIN_P3, PCA9563_PIN_LOW);
      }
      else {
        return 1;
      }
      break;
    default:
      return 1;
  }

  return 0;
}

uint8_t command_pyro_fire(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  pyro_fire_channel(atoi(argv[1]), 10);

  return 0;
}

uint8_t command_pyro_sense(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  uint8_t continuity;
  pyro_sense_channel(atoi(argv[1]), &continuity);
  cli_transmit(cli, "Channel %d continuity: %d\r\n", atoi(argv[0]), continuity);

  return 0;
}