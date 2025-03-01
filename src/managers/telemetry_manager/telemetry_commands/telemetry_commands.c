#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry_commands.h"
#include "state_machine_impl/state_machine_impl.h"
#include "system/init.h"
#include "system/usart.h"
#include "managers/telemetry_manager/telemetry_manager.h"

extern struct telemetry_manager_instance telemetry_manager;
extern struct state_machine state_machine;

uint8_t telemetry_commands_init(struct cli_handle *cli) {
  cli_add_command(cli, "arm", "", telemetry_command_arm);
  cli_add_command(cli, "abort", "", telemetry_command_abort);
  cli_add_command(cli, "ping", "", telemetry_command_ping);

  return 0;
}

uint8_t telemetry_command_arm(struct cli_handle *cli, uint16_t argc,
                              char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  state_machine.signal = STATE_MACHINE_SIGNAL_ARM;
  telemetry_manager.recently_received_command = TELEMETRY_MANAGER_COMMAND_ARM;

  return 0;
}

uint8_t telemetry_command_abort(struct cli_handle *cli, uint16_t argc,
                                char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  state_machine.signal = STATE_MACHINE_SIGNAL_ABORT;
  telemetry_manager.recently_received_command =
      TELEMETRY_MANAGER_COMMAND_ABORT;

  return 0;
}

uint8_t telemetry_command_ping(struct cli_handle *cli, uint16_t argc,
                               char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  telemetry_manager.recently_received_command =
      TELEMETRY_MANAGER_COMMAND_PING;
}
