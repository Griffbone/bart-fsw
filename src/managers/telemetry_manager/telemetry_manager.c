#include "telemetry_manager.h"
#include "fatfs/app/fatfs.h"

#include "cli/cli.h"
#include "system/rtc.h"
#include "system/tim.h"
#include <stdio.h>
#include <string.h>

extern struct cli_handle cli;

// FATFS
FATFS fatfs;
FIL log_file;
UINT sd_bytes_wrote;

// System time of last transmit
uint32_t time_of_last_transmit;

__attribute__((section(".buffer")))
uint8_t uart2_tx_dma_buffer[TELEMETRY_MANAGER_TELEM_PACKET_LENGTH];

/**
 * @brief initialize the telemetry manager.
 *
 * Initializes the SD card by creating a file, writing the header to it, and
 * initializing values for the manager.
 * @param manager pointer to a telemetry manager structure.
 * @return telemetry manager manager error enum.
 */
enum telemetry_manager_err
telemetry_manager_init(struct telemetry_manager_instance *manager) {
  // Initialize SD card
  telemetry_manager_sd_card_init(manager);

  // Set time of last transmit to 0
  time_of_last_transmit = 0;

  // Set recently-receieved command to none
  manager->recently_received_command = TELEMETRY_MANAGER_COMMAND_NONE;

  return TELEMETRY_MANAGER_ERR_OK;
};

/**
 * @brief update the telemetry manager
 *
 * Saves information to SD unless in landed state, then checks if it has been a
 * certain amount of time since the last time it transmitted. If so, transmits
 * telemetry. Then, if recently_recieved_command has been edited by some other
 * manager, transmits an acknowledge packet.
 * @param manager pointer to a telemetry manager structure
 * @param sensor_manager pointer to a sensor manager structure
 * @param actuator_manager pointer to an actuator manager structure (for
 * continuity)
 * @param state_machine pointer to a state machine structure
 * @return telemetry manager manager error enum
 * @warning an acknowledge packet is not necissarily transmitted for every
 * received command, since the flight computer could concievably receive two
 * commands before the telemetry manager gets to it.
 */
enum telemetry_manager_err
telemetry_manager_update(struct telemetry_manager_instance *manager,
                         struct sensor_manager_instance *sensor_manager,
                         struct actuator_manager_instance *actuator_manager,
                         struct state_machine *state_machine) {

  if (state_machine->state != STATE_MACHINE_STATE_LANDED &&
      state_machine->state != STATE_MACHINE_STATE_PAD_SAFE) {
    telemetry_manager_sd_card_write_row(manager, sensor_manager,
                                        actuator_manager, state_machine);
  }

  uint32_t system_time =
      __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)manager->htim);

  if (system_time - time_of_last_transmit >=
      TELEMETRY_MANAGER_TX_INTERVAL_MS * 1000) {
    telemetry_manager_transmit_telemetry(manager, sensor_manager,
                                         actuator_manager, state_machine);
    time_of_last_transmit = system_time;
  }

  if (manager->recently_received_command != TELEMETRY_MANAGER_COMMAND_NONE) {
    telemetry_manager_transmit_ack(manager, sensor_manager, actuator_manager,
                                   state_machine);
    manager->recently_received_command = TELEMETRY_MANAGER_COMMAND_NONE;
  }

  return TELEMETRY_MANAGER_ERR_OK;
}

/**
 * @brief initialize the SD card.
 *
 * Initializes the FATFS file system, creates a file, and writes the CSV header
 * to it.
 * @param manager pointer to a telemetry manager structure
 * @return telemetry manager manager error enum
 */
enum telemetry_manager_err
telemetry_manager_sd_card_init(struct telemetry_manager_instance *manager) {
  // Initialize FATFS file system
  f_mount(&fatfs, "", 1);

  // The file name is yyyy-mm-dd-hh-mm-ss.csv
  // Use the RTC to get the file name. This code is modified from
  // https://controllerstech.com/internal-rtc-in-stm32/.
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

  // Get the RTC current Time
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  // Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  char filename[13];
  sprintf(filename, "%02d-%02d-%02d.csv", gTime.Hours % 100,
          gTime.Minutes % 100, gTime.Seconds % 100);

  f_open(&log_file, filename,
         FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_OPEN_APPEND);

  // Write header to file
  char csv_header[] =
      // System time
      "system_time,"

      // Current vehicle state
      "current_state,"

      // Sensor readings
      // "imu_accel_x,imu_accel_y,imu_accel_z,"
      // "imu_gyro_x,imu_gyro_y,imu_gyro_z,"
      "high_g_accel_x,high_g_accel_y,high_g_accel_z,"
      "mag_reading_x,mag_reading_y,mag_reading_z,"
      // "temp,"
      // "press,"

      // GPS data
      "lat,lon,height,"
      "vel_ned_x,vel_ned_y,vel_ned_z,"
      "year,month,day,hour,min,sec,"
      "num_svs,"
      "gps_cov_ned_1,gps_cov_ned_2,gps_cov_ned_3,gps_cov_ned_4,gps_cov_ned_5,"
      "gps_cov_ned_6,"

      // Umbilical connected
      "umb_connected,"

      // Time of boost detection
      "boost_time,"

      // Pyro board continuity
      "pyro_board_continuity"
      "\n";

  f_write(&log_file, csv_header, strlen(csv_header), &sd_bytes_wrote);
  f_sync(&log_file);

  return TELEMETRY_MANAGER_ERR_OK;
}

/**
 * @brief write a row to the CSV file on the sd card
 *
 * Generates a row by continually snprintf-ing to a character array. The length
 * of the row must be less than TELEMETRY_MANAGER_LAST_ROW_LENGTH, but if it
 * isn't, I think since I used snprintf it'll just write down weird stuff, it
 * won't seg-fault.
 * @param manager pointer to a telemetry manager structure
 * @param sensor_manager pointer to a sensor manager structure
 * @param actuator_manager pointer to an actuator manager structure
 * @param state_machine pointer to a state machine structure
 * @return telemetry manager manager error enum
 */
enum telemetry_manager_err telemetry_manager_sd_card_write_row(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine) {
  // The row string is continually concatenated, then written to the file.
  char row[TELEMETRY_MANAGER_MAX_ROW_LENGTH];

  int row_length = 0;

  // system time
  uint32_t system_time =
      __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)manager->htim);

  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%ld,", system_time);

  // current vehicle state
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%d,", state_machine->state);

  // high-g accel reading
  row_length += snprintf(
      row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
      "%.6f,%.6f,%.6f,", sensor_manager->high_g_accel[0],
      sensor_manager->high_g_accel[1], sensor_manager->high_g_accel[2]);

  // mag_reading
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%.6f,%.6f,%.6f,", sensor_manager->mag_reading[0],
               sensor_manager->mag_reading[1], sensor_manager->mag_reading[2]);

  // GPS lat, lon, height
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%ld,%ld,%ld,", sensor_manager->lat, sensor_manager->lon,
               sensor_manager->height);

  // vel_ned
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%ld,%ld,%ld,", sensor_manager->vel_ned[0],
               sensor_manager->vel_ned[1], sensor_manager->vel_ned[2]);

  // year,month,day,hour,min,sec
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%d,%d,%d,%d,%d,%d,", sensor_manager->year,
               sensor_manager->month, sensor_manager->day, sensor_manager->hour,
               sensor_manager->min, sensor_manager->sec);

  // num_svs
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%d,", sensor_manager->num_svs);

  // gps_cov
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,", sensor_manager->gps_cov_ned[0],
               sensor_manager->gps_cov_ned[1], sensor_manager->gps_cov_ned[2],
               sensor_manager->gps_cov_ned[3], sensor_manager->gps_cov_ned[4],
               sensor_manager->gps_cov_ned[5]);

  // boost time
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%d,", state_machine->umbilical_continuity);
  
  // boost time
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%ld,", state_machine->boost_time);

  // pyro board continuity
  row_length +=
      snprintf(row + row_length, TELEMETRY_MANAGER_MAX_ROW_LENGTH - row_length,
               "%d\r\n", actuator_manager->continuity);

  // Write the populated row string to the file
  f_write(&log_file, row, strlen(row), &sd_bytes_wrote);
  f_sync(&log_file);

  return TELEMETRY_MANAGER_ERR_OK;
}

/**
 * @brief transmit data over the xbee radio.
 *
 * Formats information from the sensor, actuator, and state machine into the
 * telemetry packet structure at
 * https://gtxr.getoutline.com/doc/strange-magic-packets-and-file-formats-gFttmh6yJf,
 * then transmits it over the UART channel specified by radio_uart_handle.
 * @param manager pointer to a telemetry manager structure
 * @param sensor_manager pointer to a sensor manager structure
 * @param actuator_manager pointer to an actuator manager structure (for
 * continuity)
 * @param state_machine pointer to a state state_machine structure
 * @return telemetry manager manager error enum
 */
enum telemetry_manager_err telemetry_manager_transmit_telemetry(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine) {

  char packet[TELEMETRY_MANAGER_TELEM_PACKET_LENGTH];

  // Preamble
  packet[0] = TELEMETRY_MANAGER_PACKET_PREAMBLE_1;
  packet[1] = TELEMETRY_MANAGER_PACKET_PREAMBLE_2;

  // Header
  packet[2] = TELEMETRY_MANAGER_PACKET_ID_TELEMETRY;

  uint32_t system_time =
      __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)manager->htim);
  // System time
  memcpy(&packet[3], &system_time, 4);

  // Current vehicle state
  packet[7] = state_machine->state;

  // Pyro continuity
  packet[8] = actuator_manager->continuity;

  // GPS number of satellites
  packet[9] = sensor_manager->num_svs;

  // GPS lat, lon, height
  memcpy(&packet[10], &(sensor_manager->lat), 4);
  memcpy(&packet[14], &(sensor_manager->lon), 4);
  memcpy(&packet[18], &(sensor_manager->height), 4);

  // GPS velocity, north-east-down
  memcpy(&packet[22], &(sensor_manager->vel_ned[0]), 4);
  memcpy(&packet[26], &(sensor_manager->vel_ned[1]), 4);
  memcpy(&packet[30], &(sensor_manager->vel_ned[2]), 4);

  // Baro pressure
  memcpy(&packet[34], &(sensor_manager->press), 4);

  // Lowest barometer pressure seen during flight
  memcpy(&packet[38], &(sensor_manager->lowest_press), 4);

  // Highest GPS altitude seen during flight
  memcpy(&packet[42], &(sensor_manager->highest_gps_alt), 4);

  // System time of boost detection
  memcpy(&packet[46], &(state_machine->boost_time), 4);

  // Umbilical continuity
  packet[50] = state_machine->umbilical_continuity;

  // Calculate checksum
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (uint8_t i = 2; i < TELEMETRY_MANAGER_TELEM_PACKET_LENGTH - 2; i++) {
    ck_a += packet[i];
    ck_b += ck_a;
  }

  packet[51] = ck_a;
  packet[52] = ck_b;

  memcpy(uart2_tx_dma_buffer, packet, sizeof(packet));

  // TODO: change to DMA UART
  // HAL_UART_Transmit((UART_HandleTypeDef *)manager->huart, (uint8_t *)packet,
  //                   sizeof(packet), HAL_MAX_DELAY);
  HAL_UART_Transmit_IT((UART_HandleTypeDef *) manager->huart,
      uart2_tx_dma_buffer, sizeof(packet));
  return TELEMETRY_MANAGER_ERR_OK;
}

enum telemetry_manager_err telemetry_manager_transmit_ack(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine) {

  char packet[TELEMETRY_MANAGER_ACK_PACKET_LENGTH];

  // Preamble
  packet[0] = TELEMETRY_MANAGER_PACKET_PREAMBLE_1;
  packet[1] = TELEMETRY_MANAGER_PACKET_PREAMBLE_2;

  // Header
  packet[2] = TELEMETRY_MANAGER_PACKET_ID_ACK;

  // Command to acknowledge
  packet[3] = manager->recently_received_command;

  // Calculate checksum
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (uint8_t i = 2; i < TELEMETRY_MANAGER_ACK_PACKET_LENGTH - 2; i++) {
    ck_a += packet[i];
    ck_b += ck_a;
  }

  packet[4] = ck_a;
  packet[5] = ck_b;

  memcpy(uart2_tx_dma_buffer, packet, sizeof(packet));

  // TODO: change to DMA UART
  // HAL_UART_Transmit((UART_HandleTypeDef *)manager->huart, (uint8_t *)packet,
  //                   sizeof(packet), HAL_MAX_DELAY);
  HAL_UART_Transmit_IT((UART_HandleTypeDef *) manager->huart,
      uart2_tx_dma_buffer, sizeof(packet));

  return TELEMETRY_MANAGER_ERR_OK;
}
