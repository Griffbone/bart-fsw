#ifndef __CLI_H__
#define __CLI_H__

#include <stddef.h>
#include <stdint.h>

#define CLI_MAXIMUM_NUMBER_OF_COMMANDS  128
#define CLI_MAXIMUM_COMMAND_LENGTH      512
#define CLI_MAXIMUM_NUMBER_OF_ARGUMENTS 16
#define CLI_MAXIMUM_TRANSMIT_SIZE       256
#define CLI_MAXIMUM_RECEIVE_SIZE        256
#define CLI_SHELL_NAME                  "osama"

struct cli_command;
struct cli_handle;

enum cli_err {
  CLI_ERR_OK = 0,
};

struct cli_command {
  const char *command;
  const char *manual;
  uint8_t (*execute)(struct cli_handle *cli, uint16_t argc, char *argv[]);
};

struct cli_handle {
  void *uart_handle;
  struct cli_command commands[CLI_MAXIMUM_NUMBER_OF_COMMANDS];
};

enum cli_err cli_init(struct cli_handle *cli, void *huart);

enum cli_err cli_add_command(struct cli_handle *cli, const char *command,
                             const char *manual,
                             uint8_t (*execute)(struct cli_handle *cli,
                                                uint16_t argc, char *argv[]));
enum cli_err cli_transmit(struct cli_handle *cli, const char *format, ...);
enum cli_err cli_receive(struct cli_handle *cli, const char *receive,
                         size_t size);
enum cli_err cli_echo(struct cli_handle *cli, const char *chars);
enum cli_err cli_handle_command(struct cli_handle *cli);
enum cli_err cli_greet(struct cli_handle *cli);

#endif /*__CLI_H__ */
