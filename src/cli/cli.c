#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#include "cli.h"

char rx_buffer[CLI_MAXIMUM_RECEIVE_SIZE];
uint16_t rx_buffer_end_ptr = 0;

static void strip(char *dest, const char *src, const char *delim);

enum cli_err cli_init(struct cli_handle *cli, void *huart) {
  cli->uart_handle = huart;

  for (uint16_t i = 0; i < CLI_MAXIMUM_NUMBER_OF_COMMANDS; i++) {
    cli->commands[i].command = "";
  }

  cli_greet(cli);

  return CLI_ERR_OK;
}

enum cli_err cli_add_command(struct cli_handle *cli, const char *command,
                             const char *manual,
                             uint8_t (*execute)(struct cli_handle *cli,
                                                uint16_t argc, char *argv[])) {
  for (uint16_t i = 0; i < CLI_MAXIMUM_NUMBER_OF_COMMANDS; i++) {
    if (strcmp(cli->commands[i].command, "") == 0) {
      cli->commands[i].command = command;
      cli->commands[i].manual = manual;
      cli->commands[i].execute = execute;

      return CLI_ERR_OK;
    }
  }

  return CLI_ERR_OK;
}

enum cli_err cli_transmit(struct cli_handle *cli, const char *fmt, ...) {
  uint8_t buffer[CLI_MAXIMUM_TRANSMIT_SIZE];
  va_list args;

  va_start(args, fmt);

  vsnprintf((char *)buffer, sizeof(buffer), fmt, args);

  va_end(args);

  HAL_UART_Transmit(cli->uart_handle, (uint8_t *)buffer, strlen((char *)buffer),
                    HAL_MAX_DELAY);

  return CLI_ERR_OK;
}

enum cli_err cli_receive(struct cli_handle *cli, const char *receive,
                         size_t size) {
  for (uint16_t i = 0; i < size; i++) {
    if (receive[i] == '\b') {
      rx_buffer_end_ptr--;
    }

    else {
      rx_buffer[rx_buffer_end_ptr++] = receive[i];
    }
  }

  return CLI_ERR_OK;
}

enum cli_err cli_echo(struct cli_handle *cli, const char *chars) {
  static uint8_t buffer[256];
  int b_ind = 0;
  memset(buffer, 0, 256);

  for (int i = 0; i < strlen(chars); i++) {
    // Carriage return
    if (chars[i] == '\r') {
      buffer[b_ind++] = '\r';
      buffer[b_ind++] = '\n';
    }

    // New line
    else if (chars[i] == '\n')
      continue;

    // Backspace
    else if (chars[i] == 127) {
      buffer[b_ind++] = '\b';
      buffer[b_ind++] = ' ';
      buffer[b_ind++] = '\b';
    }

    // Characters
    else
      buffer[b_ind++] = chars[i];
  }

  HAL_UART_Transmit(cli->uart_handle, (uint8_t *)buffer, b_ind, HAL_MAX_DELAY);

  return CLI_ERR_OK;
}

enum cli_err cli_handle_command(struct cli_handle *cli) {
  static char rx_buffer_stripped[CLI_MAXIMUM_COMMAND_LENGTH];
  static char *delim = " ";

  memset(rx_buffer_stripped, '\0', CLI_MAXIMUM_COMMAND_LENGTH);

  if (rx_buffer_end_ptr == CLI_MAXIMUM_RECEIVE_SIZE) {
    cli_transmit(cli, "Buffer overflow! Don't type giberish!\r\n");
  }

  uint8_t is_return_present = 0;
  for (uint16_t i = 0; i < rx_buffer_end_ptr; i++) {
    if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r') {
      is_return_present = 1;
    }
  }

  if (!is_return_present) {
    return CLI_ERR_OK;
  }

  strip(rx_buffer_stripped, rx_buffer, "\r\n");
  memset(rx_buffer, 0, sizeof(rx_buffer));
  rx_buffer_end_ptr = 0;

  if (strcmp(rx_buffer_stripped, "") == 0) {
    cli_transmit(cli, CLI_SHELL_NAME "$ ");

    return CLI_ERR_OK;
  }

  char *command = strtok(rx_buffer_stripped, delim);

  uint8_t is_command_valid = 0;

  uint16_t i;
  for (i = 0; i < CLI_MAXIMUM_NUMBER_OF_COMMANDS; i++) {
    if (strcmp(cli->commands[i].command, command) == 0) {
      is_command_valid = 1;
      break;
    }
  }

  if (!is_command_valid) {
    cli_transmit(cli, "%s: command not found\r\n", command);
    cli_transmit(cli, CLI_SHELL_NAME "$ ");

    return CLI_ERR_OK;
  }

  uint16_t argc;
  char *argv[CLI_MAXIMUM_NUMBER_OF_ARGUMENTS];
  char *token;

  argc = 1;
  argv[0] = command;

  while ((token = strtok(NULL, delim))) {
    argv[argc] = token;
    argc++;
  }

  cli->commands[i].execute(cli, argc, argv);

  cli_transmit(cli, CLI_SHELL_NAME "$ ");

  return CLI_ERR_OK;
}

enum cli_err cli_greet(struct cli_handle *cli) {
  cli_transmit(cli, CLI_SHELL_NAME "$ ");

  return CLI_ERR_OK;
}

enum cli_err __cli_uart_transmit(struct cli_handle *cli, char *buf,
                                 uint16_t size) {

  return CLI_ERR_OK;
}

static void strip(char *dest, const char *src, const char *delim) {
  uint16_t length = strlen(src);
  uint16_t counter = 0;
  for (uint16_t i = 0; i < length; i++) {
    if (strchr(delim, src[i]) != NULL) {
      continue;
    }

    dest[counter] = src[i];
    counter++;
  }

  dest[counter] = '\0';
}
