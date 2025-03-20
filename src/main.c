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

// Device drivers
#include "drivers/icm45686/icm45686.h"

// Setup CLI
extern DMA_HandleTypeDef hdma_usart1_rx; // TODO: idk if this should be usart our uart
volatile uint8_t cli_uart_rx_data[256];
volatile uint8_t is_cli_uart_rx_data_available;
struct cli_handle cli;
struct cli_handle telemetry_cli;

int main(void) {
  init(); 

  // Initialize debug CLI
  cli_init(&cli, &huart1);
  command_init(&cli);
  memset((void *)cli_uart_rx_data, 0, sizeof(cli_uart_rx_data));

  while(1) {
    HAL_GPIO_WritePin(USR_LED_1_GPIO_Port, USR_LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(USR_LED_1_GPIO_Port, USR_LED_1_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);

    // cli_transmit(&cli, "hello world\r\n");  
  
    // CLI Handling
    if (is_cli_uart_rx_data_available != 0) {
    cli_receive(&cli, (char *)cli_uart_rx_data,
              strlen((char *)cli_uart_rx_data));
      cli_handle_command(&cli);

      memset((void *)cli_uart_rx_data, 0, sizeof(cli_uart_rx_data));
      is_cli_uart_rx_data_available = 0;
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
}


// #include "ring_buffer/ring_buffer.h"

// #include "system/i2c.h"
// #include "system/init.h"
// #include "system/rtc.h"
// #include "system/spi.h"
// #include "system/tim.h"

// volatile uint8_t telemetry_uart_rx_data[256];
// volatile uint8_t is_telemetry_uart_rx_data_ready;

// volatile uint8_t is_gps_rx_data_available;
// uint8_t gps_char;

// extern DMA_HandleTypeDef hdma_uart7_rx;
// __attribute__((section(".buffer"))) uint8_t uart7_rx_dma_buffer[1024];
// uint16_t uart7_rx_dma_buffer_size;

// struct ring_buffer uart7_rx_rb;

// // Instantiate sensor structures
// struct sensor_manager_instance sensor_manager;
// struct telemetry_manager_instance telemetry_manager;
// struct state_machine state_machine;
// struct actuator_manager_instance actuator_manager;

// uint8_t fdcan_rx_data[8];

// int main(void) {
//   init();

//   // Initialize debug CLI
//   cli_init(&cli, &huart6);
//   command_init(&cli);
//   memset((void *)cli_uart_rx_data, 0, sizeof(cli_uart_rx_data));

//   // Initialize telemetry CLI
//   cli_init(&telemetry_cli, &huart2);
//   telemetry_commands_init(&telemetry_cli);
//   memset((void *)telemetry_uart_rx_data, 0, sizeof(telemetry_uart_rx_data));
  
//   HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)telemetry_uart_rx_data,
//                               sizeof(telemetry_uart_rx_data));

//   // Initialize CLI UART Interrupt
//   HAL_UARTEx_ReceiveToIdle_IT(&huart6, (uint8_t *)cli_uart_rx_data,
//                               sizeof(cli_uart_rx_data));

//   // Initialize GPS UART DMA
//   HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_dma_buffer,
//                                sizeof(uart7_rx_dma_buffer));
//   __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

//   RTC_TimeTypeDef time;
//   RTC_DateTypeDef date;
  
//   // System time timer
//   HAL_TIM_Base_Start(&htim24);
//   __HAL_TIM_SET_COUNTER(&htim24, 0);

//   // Initialize manager
//   sensor_manager.htim = &htim24;
//   sensor_manager.hrtc = &hrtc;
//   sensor_manager_init(&sensor_manager);

//   telemetry_manager.htim = &htim24;
//   telemetry_manager.huart = &huart2; // TODO: use 2 for flight, 6 for debug
//   telemetry_manager_init(&telemetry_manager);
  
//   actuator_manager.hi2c = &hi2c1; 
//   actuator_manager.pyro_i2c_addr = 0x69;
//   actuator_manager.i2c_timeout_ms = 10;
//   actuator_manager_init(&actuator_manager);

//   state_machine_init(&state_machine);
  
//   uint8_t log[64];

//   // Infinite loop
//   while (1) {
//     state_machine_update(&state_machine);
//     sensor_manager_update(&sensor_manager);
//     actuator_manager_update(&actuator_manager, &state_machine);
//     telemetry_manager_update(&telemetry_manager, &sensor_manager, &actuator_manager, &state_machine);

//     // cli_transmit(&cli, "%.6lf,%.6lf,%.6lf\r\n", sensor_manager.high_g_accel[0], sensor_manager.high_g_accel[1], sensor_manager.high_g_accel[2]);
//     cli_transmit(&cli, "state: %d\r\n", state_machine.state);    
//     cli_transmit(&cli, "pin: %d\r\n", HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3));
//     cli_transmit(&cli, "boost time: %.3f\r\n", (double) __HAL_TIM_GET_COUNTER(&htim23) / (1e6));
//     cli_transmit(&cli, "\r\n");
//     HAL_Delay(500);

//     if (is_telemetry_uart_rx_data_ready != 0) {
//       cli_receive(&telemetry_cli, (char *)telemetry_uart_rx_data,
//                   strlen((char *)telemetry_uart_rx_data));
//       cli_handle_command(&telemetry_cli);

//       memset((void *)telemetry_uart_rx_data, 0, sizeof(telemetry_uart_rx_data));

//       is_telemetry_uart_rx_data_ready = 0;
//     }
//   }

//   return 0;
// }

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//   // CLI callback
//   if (huart->Instance == USART6) {
//     is_cli_uart_rx_data_available = 1;
//     HAL_UARTEx_ReceiveToIdle_IT(&huart6, (uint8_t *)cli_uart_rx_data,
//                                 sizeof(cli_uart_rx_data));
//   }

//   if (huart->Instance == USART2) {
//     is_telemetry_uart_rx_data_ready = 1;
//     HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)telemetry_uart_rx_data,
//                                 sizeof(telemetry_uart_rx_data));
//   }

//   // GPS callback
//   if (huart->Instance == UART7) {
//     ring_buffer_write(&uart7_rx_rb, uart7_rx_dma_buffer, Size);
//     uart7_rx_dma_buffer_size = Size;

//     HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_dma_buffer,
//                                  sizeof(uart7_rx_dma_buffer));
//     __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
//   }
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//   if (GPIO_Pin == GPIO_PIN_3) {
//     if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3) == GPIO_PIN_SET) {
//       state_machine.signal = STATE_MACHINE_SIGNAL_UMBILICAL_DISCONNECT;
//     }
//   }
// }
