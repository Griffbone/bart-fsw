// Standard libraries
#include <stdio.h>
#include <string.h>

// Interface code
#include "cli/cli.h"
#include "commands/commands.h"

// System drivers
#include "system/init.h"
#include "system/spi.h"
#include "system/usart.h"
#include "system/i2c.h"
#include "system/i2c.h"

// Device drivers
#include "drivers/pca9563/pca9563.h"
#include "drivers/bmp581/bmp581.h"
#include "drivers/pyro/pyro.h"

// Setup CLI
volatile uint8_t cli_uart_rx_data[256];
volatile uint8_t is_cli_uart_rx_data_available;
struct cli_handle cli;
// struct cli_handle telemetry_cli;

// Setup telemetry interface 
volatile uint8_t telem_uart_rx_data[256];
volatile uint8_t is_telem_rx_data_available;

// Device structures 
struct bmp581_device baro;
pca9563_device_t io_expander;

int main(void) {
  init(); 

  // Initialize IO expander
  io_expander.hi2c = &hi2c1;
  io_expander.timeout = 100;
  pca9563_set_pin_mode(&io_expander, PCA9563_PIN_P0, PCA9563_PIN_OUTPUT);
  pca9563_set_pin_mode(&io_expander, PCA9563_PIN_P1, PCA9563_PIN_OUTPUT);
  pca9563_set_pin_mode(&io_expander, PCA9563_PIN_P2, PCA9563_PIN_INPUT);
  pca9563_set_pin_mode(&io_expander, PCA9563_PIN_P3, PCA9563_PIN_OUTPUT);
  
  // Initialize barometer 
  baro.i2c_addr = 0x47;
  baro.hi2c = &hi2c2;
  uint8_t status = bmp581_init(&baro);

  // Initialize pyro hardware
  pyro_init();

  // Initialize radio
  pca9563_set_pin(&io_expander, PCA9563_PIN_P1, PCA9563_PIN_LOW);

  // Initialize debug CLI
  cli_init(&cli, &huart1);
  command_init(&cli);
  memset((void *)cli_uart_rx_data, 0, sizeof(cli_uart_rx_data));

  // Initialize CLI UART interrupt
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)cli_uart_rx_data, sizeof(cli_uart_rx_data));

  // Other random shit
  char at_command[] = "AT\r\n";

  while(1) {
    // Radio debugging
    HAL_GPIO_TogglePin(USR_LED_1_GPIO_Port, USR_LED_1_Pin);
    HAL_UART_Transmit(&huart5, (uint8_t *)at_command, sizeof(at_command)-1, HAL_MAX_DELAY);
    HAL_Delay(250);

    /* ========== SENSORS ========== */
    // bmp581_read_temp(&baro, &temp);
    // bmp581_read_press(&baro, &press);
    // cli_transmit(&cli, "%.2f %.2f \r\n", temp, press);

    /* ========== CLI HANDLING ========== */
    if (is_cli_uart_rx_data_available != 0) {
    cli_receive(&cli, (char *)cli_uart_rx_data,
              strlen((char *)cli_uart_rx_data));
      cli_handle_command(&cli);

      memset((void *)cli_uart_rx_data, 0, sizeof(cli_uart_rx_data));
      is_cli_uart_rx_data_available = 0;
    }

    /* ========== TELEMETRY HANDLING ========== */
    if (is_telem_rx_data_available) {
      cli_transmit(&cli, "s\r\n", telem_uart_rx_data);
    }
  }

  return 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // CLI callback
  if (huart->Instance == USART1) {
    is_cli_uart_rx_data_available = 1;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)cli_uart_rx_data,
                                sizeof(cli_uart_rx_data));
  }

  if (huart->Instance == UART5) {
    is_telem_rx_data_available = 1;
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, (uint8_t *)telem_uart_rx_data,
                                sizeof(telem_uart_rx_data));
  }
}