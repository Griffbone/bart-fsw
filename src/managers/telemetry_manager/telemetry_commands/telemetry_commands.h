#ifndef __TELEMETRY_COMMANDS_H__
#define __TELEMETRY_COMMANDS_H__

#include <stdint.h>

#include "cli/cli.h"

uint8_t telemetry_commands_init(struct cli_handle *cli);
uint8_t telemetry_command_arm(struct cli_handle *cli, uint16_t argc,
                              char *argv[]);
uint8_t telemetry_command_abort(struct cli_handle *cli, uint16_t argc,
                                char *argv[]);
uint8_t telemetry_command_ping(struct cli_handle *cli, uint16_t argc,
                               char *argv[]);

#endif /* __TELEMETRY_COMMANDS_H__ */
