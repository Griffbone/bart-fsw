#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include <stdint.h>

#include "cli/cli.h"

uint8_t command_init(struct cli_handle *cli);

uint8_t command_help(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_clear(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_led_toggle(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_pyro_fire(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_pyro_sense(struct cli_handle *cli, uint16_t argc, char *argv[]);

#endif /* __COMMANDS_H__ */
