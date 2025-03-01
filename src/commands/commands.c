#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "drivers/ms5611/ms5611.h"
#include "drivers/pyro_board/pyro_board.h"
#include "fatfs/app/fatfs.h"
#include "gnc/ekf/ekf.h"
#include "gnc/ekf/tests/ekf_tests.h"
#include "gnc/gnc_lib/gnc_lib.h"
#include "gnc/gnc_lib/tests/gnc_tests.h"
#include "gnc/qmekf/qmekf.h"
#include "gnc/qmekf/tests/qmekf_tests.h"
#include "drivers/lis3mdl/lis3mdl.h"
#include "drivers/ublox_gnss/ublox_gnss.h"
#include "state_machine_impl/state_machine_impl.h"
#include "system/init.h"
#include "system/usart.h"

extern struct ms5611_device baro;
extern struct pyro_board_device pyro_board;
extern struct qmekf_instance qmekf;
extern struct ekf_instance ekf;
extern FATFS fatfs;
extern FIL log_file;
extern struct lis3mdl_device mag;
extern struct ublox_gnss_device gps;
// extern struct state_machine state_machine;
extern struct gps_data gps_data;

uint8_t command_init(struct cli_handle *cli) {
  cli_add_command(cli, "led", "", command_led_toggle);
  cli_add_command(cli, "nmea", "", command_gps_nmea_toggle);
  cli_add_command(cli, "baro-test", "", command_baro_test);
  // cli_add_command(cli, "pyro-fire", "fire pyro channel: pyro-fire <channel>",
  //                 command_pyro_fire);
  // cli_add_command(cli, "pyro-continuity", "read all pyro channel continuity",
  //                 command_pyro_read_continuity);
  cli_add_command(cli, "clear", "", command_clear);
  cli_add_command(cli, "test-gnc", "", command_test_gnc);
  cli_add_command(cli, "test-qmekf", "", command_test_qmekf);
  cli_add_command(cli, "test-ekf", "", command_test_ekf);
  // cli_add_command(cli, "rx-sensor-sim", "", command_rx_sensor_sim);
  // cli_add_command(cli, "sd-close-log", "", command_sd_close_log);
  // cli_add_command(cli, "rx-ekf-sim", "", command_rx_ekf_sim);
  cli_add_command(cli, "mag-read", "", command_mag_read);
  cli_add_command(cli, "mag-burst-read", "", command_mag_burst_read);
  cli_add_command(cli, "hgimu-toggle", "", command_hgimu_toggle);
  cli_add_command(cli, "gps-assist", "", command_gps_assist);
  cli_add_command(cli, "arm", "", command_arm);

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

uint8_t command_led_toggle(struct cli_handle *cli, uint16_t argc,
                           char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  if (strcmp(argv[1], "on") == 0) {
    HAL_GPIO_WritePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
  }

  else if (strcmp(argv[1], "off") == 0) {
    HAL_GPIO_WritePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
  }

  else {
    return 1;
  }

  return 0;
}

uint8_t command_gps_nmea_toggle(struct cli_handle *cli, uint16_t argc,
                                char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  uint8_t dat1[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x01, 0x01, 0x00,
                    0x00, 0x02, 0x00, 0x74, 0x10, 0x00, 0x21, 0xc0};

  if (strcmp(argv[1], "on") == 0) {
    dat1[14] = 0x01;
    dat1[15] = 0x22;
    dat1[16] = 0xc1;
    HAL_UART_Transmit(&huart7, dat1, 17, HAL_MAX_DELAY);
  }

  else if (strcmp(argv[1], "off") == 0) {
    dat1[14] = 0x00;
    dat1[15] = 0x21;
    dat1[16] = 0xc0;
    HAL_UART_Transmit(&huart7, dat1, 17, HAL_MAX_DELAY);
  }

  else {
    return 1;
  }

  return 0;
}

uint8_t command_baro_test(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  float temperature;
  float pressure;

  ms5611_read_eeprom(&baro);
  ms5611_read_temperature_and_pressure(&baro, &temperature, &pressure);
  cli_transmit(cli, "temperature: %f\r\n", temperature);
  cli_transmit(cli, "pressure: %f\r\n", pressure);

  return 0;
}

// uint8_t command_pyro_fire(struct cli_handle *cli, uint16_t argc, char *argv[]) {
//   // If there are additional arguments or channel is > 1 character then error
//   if ((argc != 2) | (strlen(argv[1]) > 1)) {
//     cli_transmit(cli, "Invalid argument\r\n");
//     return 1;
//   }

//   uint8_t channel = atoi(argv[1]);
//   if ((channel >= 1) && (channel <= 6)) {
//     // cli_transmit(cli, "\tAre you sure you want to fire channel %i?
//     // (Y/N)\r\n", channel);
//     pyro_board_fire_channel(&pyro_board, channel);
//   } else {
//     cli_transmit(cli, "Invalid channel \"%i\"\r\n", channel);
//     return 1;
//   }

//   return 0;
// }

// uint8_t command_pyro_read_continuity(struct cli_handle *cli, uint16_t argc,
//                                      char *argv[]) {
//   if (argc != 1) {
//     return 1;
//   }

//   uint8_t continuity;
//   pyro_board_read_continuity(&pyro_board, &continuity);

//   uint8_t c1 = (continuity >> 5) & 1;
//   uint8_t c2 = (continuity >> 4) & 1;
//   uint8_t c3 = (continuity >> 3) & 1;
//   uint8_t c4 = (continuity >> 2) & 1;
//   uint8_t c5 = (continuity >> 1) & 1;
//   uint8_t c6 = continuity & 1;

//   cli_transmit(cli,
//                "\tChannel 1: %i\r\n \tChannel 2: %i\r\n \tChannel 3: %i\r\n "
//                "\tChannel 4: %i\r\n \tChannel 5: %i\r\n \tChannel 6: %i\r\n",
//                c1, c2, c3, c4, c5, c6);
//   return 0;
// }

uint8_t command_clear(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  cli_transmit(cli, "\033[2J");
  cli_transmit(cli, "\033[1;1H");

  return 0;
}

uint8_t command_test_gnc(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  if (!test_quaternion_multiplication()) {
    cli_transmit(cli,
                 "\e[0;31m\tquaternion multiplication test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tquaternion multiplication test passed\e[0;37m\r\n");
  }

  if (!test_quaternion_propagate_delta_angle()) {
    cli_transmit(
        cli,
        "\e[0;31m\tdelta-angle quaternion propagation test failed\e[0;37m\r\n");
  } else {
    cli_transmit(
        cli,
        "\e[0;32m\tdelta-angle quaternion propagation test passed\e[0;37m\r\n");
  }

  if (!test_quaternion_propagate_omega()) {
    cli_transmit(
        cli, "\e[0;31m\tomega quaternion propagation test failed\e[0;37m\r\n");
  } else {
    cli_transmit(
        cli, "\e[0;32m\tomega quaternion propagation test passed\e[0;37m\r\n");
  }

  if (!test_quaternion_to_rotation()) {
    cli_transmit(cli,
                 "\e[0;31m\tquaternion to rotation test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tquaternion to rotation test passed\e[0;37m\r\n");
  }

  if (!test_rotation_1()) {
    cli_transmit(cli, "\e[0;31m\trotation_1 test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\trotation_1 test passed\e[0;37m\r\n");
  }

  if (!test_rotation_2()) {
    cli_transmit(cli, "\e[0;31m\trotation_2 test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\trotation_2 test passed\e[0;37m\r\n");
  }

  if (!test_rotation_3()) {
    cli_transmit(cli, "\e[0;31m\trotation_3 test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\trotation_3 test passed\e[0;37m\r\n");
  }

  if (!test_skew_symmetric()) {
    cli_transmit(cli,
                 "\e[0;31m\tskew-symmetric matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tskew-symmetric matrix test passed\e[0;37m\r\n");
  }

  if (!test_apply_rotation()) {
    cli_transmit(cli,
                 "\e[0;31m\tapply rotation matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tapply rotation matrix test passed\e[0;37m\r\n");
  }

  if (!test_vec_subtract()) {
    cli_transmit(cli, "\e[0;31m\tvector subtraction test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tvector subtraction test passed\e[0;37m\r\n");
  }

  if (!test_vec_add()) {
    cli_transmit(cli, "\e[0;31m\tvector addition test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tvector addition test passed\e[0;37m\r\n");
  }

  if (!test_ecef_to_ned()) {
    cli_transmit(cli, "\e[0;31m\tECEF to NED test failed\e[0;37m\r\n");
  } else {
      cli_transmit(cli, "\e[0;32m\tECEF to NED test passed\e[0;37m\r\n");
  }

  return 0;
}

uint8_t command_test_qmekf(struct cli_handle *cli, uint16_t argc,
                           char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  if (!test_qmekf_get_stm()) {
    cli_transmit(cli,
                 "\e[0;31m\tQMEKF transition matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tQMEKF transition matrix test passed\e[0;37m\r\n");
  }

  if (!test_qmekf_get_sens()) {
    cli_transmit(cli,
                 "\e[0;31m\tQMEKF sensitiviy matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tQMEKF sensitiviy matrix test passed\e[0;37m\r\n");
  }

  if (!test_qmekf_get_resisdual()) {
    cli_transmit(cli, "\e[0;31m\tQMEKF residual test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tQMEKF residual test passed\e[0;37m\r\n");
  }

  if (!test_qmekf_update()) {
    cli_transmit(cli, "\e[0;31m\tQMEKF update test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tQMEKF update test passed\e[0;37m\r\n");
  }

  if (!test_qmekf_predict()) {
    cli_transmit(cli, "\e[0;31m\tQMEKF predict test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tQMEKF predict test passed\e[0;37m\r\n");
  }

  return 0;
}

uint8_t command_test_ekf(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (!test_ekf_get_sens()) {
    cli_transmit(cli,
                 "\e[0;31m\tEKF sensitiviy matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tEKF sensitiviy matrix test passed\e[0;37m\r\n");
  }

  if (!test_ekf_get_stm()) {
    cli_transmit(cli,
                 "\e[0;31m\tEKF transition matrix test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli,
                 "\e[0;32m\tEKF transition matrix test passed\e[0;37m\r\n");
  }

  if (!test_ekf_get_residual()) {
    cli_transmit(cli, "\e[0;31m\tEKF residual test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tEKF residual test passed\e[0;37m\r\n");
  }

  /*
  if (!test_ekf_set_gps_covariance()) {
    cli_transmit(cli, "\e[0;31m\tEKF set gps covariance test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tEKF set gps covariance test passed\e[0;37m\r\n");
  }
  */

  if (!test_ekf_predict()) {
    cli_transmit(cli, "\e[0;31m\tEKF predict test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tEKF predict test passed\e[0;37m\r\n");
  }

  if (!test_ekf_update()) {
    cli_transmit(cli, "\e[0;31m\tEKF udpate test failed\e[0;37m\r\n");
  } else {
    cli_transmit(cli, "\e[0;32m\tEKF udpate test passed\e[0;37m\r\n");
  }

  return 0;
}

uint8_t command_mag_read(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  double reading[3];
  lis3mdl_read_mag(&mag, reading);
  reading[0] = reading[0]*100000;
  reading[1] = reading[1]*100000;
  reading[2] = reading[2]*100000;
  cli_transmit(cli, "%.2f,%.2f,%.2f\r\n", reading[0], reading[1], reading[2]);

  return 0;
}

// uint8_t command_rx_sensor_sim(struct cli_handle *cli, uint16_t argc,
//                               char *argv[]) {
//   if (argc != 11) {
//     return 1;
//   }

//   float64_t omega[3] = {atof(argv[1]), atof(argv[2]), atof(argv[3])};
//   float64_t accel[3] = {atof(argv[4]), atof(argv[5]), atof(argv[6])};
//   float64_t mag[3] = {atof(argv[7]), atof(argv[8]), atof(argv[9])};
//   float64_t time = atof(argv[10]);

//   qmekf_predict(&qmekf, omega, time - qmekf.time_prev);
//   qmekf_update(&qmekf, mag, accel);
//   qmekf.time_prev = time;

//   static uint8_t on = 0;

//   on ^= 1;
//   if (on) {
//     HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
//   } else {
//     HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
//   }

//   // Write to SD card
//   char log[128];
//   sprintf(log, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n", time,
//           qmekf.quaternion[0], qmekf.quaternion[1], qmekf.quaternion[2],
//           qmekf.quaternion[3], qmekf.gyro_bias[0], qmekf.gyro_bias[1],
//           qmekf.gyro_bias[2]);
//   UINT bytes_wrote;
//   f_write(&log_file, log, strlen(log), &bytes_wrote);

//   return 0;
// }

// uint8_t command_rx_ekf_sim(struct cli_handle *cli, uint16_t argc,
//                            char *argv[]) {

//   if (argc != 14) {
//     HAL_GPIO_TogglePin(USER_LED_2_GPIO_Port, USER_LED_2_Pin);
//     return 1;
//   }

//   float64_t time = atof(argv[1]);
//   uint8_t update_flag = atoi(argv[2]);
//   float64_t accel[3] = {atof(argv[3]), atof(argv[4]), atof(argv[5])};
//   float64_t quaternion[4] = {atof(argv[6]), atof(argv[7]), atof(argv[8]),
//                               atof(argv[9])};
//   float64_t gps[3] = {atof(argv[10]), atof(argv[11]), atof(argv[12])};
//   float64_t baro = atof(argv[13]);

//   // Compute rotation matrix from Quaternion 
//   quaternion_to_rotation(quaternion, qmekf.attitude_matrix);
  
//   // Run prediction step
//   ekf_predict(&ekf, accel, qmekf.attitude_matrix, time - ekf.time_prev);
//   ekf.time_prev = time;

//   // Run update step
//   if (update_flag) {
//     ekf_update(&ekf, gps, baro);
//   }

//   // Write to SD card
//   char log[128];
//   sprintf(log, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n", time, ekf.r_enu[0],
//   ekf.r_enu[1], ekf.r_enu[2], ekf.v_enu[0], ekf.v_enu[1], ekf.v_enu[2]); 
//   UINT bytes_wrote; 
//   f_write(&log_file, log, strlen(log), &bytes_wrote);

//   // Flash LEDs
//   static uint8_t on = 0;

//   on ^= 1;
//   if (on) {
//     HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
//   } else {
//     HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
//   }

//   return 0;
// }

// uint8_t command_sd_close_log(struct cli_handle *cli, uint16_t argc,
//                              char *argv[]) {
//   if (argc != 1) {
//     return 1;
//   }

//   f_close(&log_file);
//   f_mount(NULL, "", 0);

//   return 0;
// }

uint8_t command_mag_burst_read(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 3) {
    return 1; 
  }

  double reading[3];
  uint32_t duration = atoi(argv[1]);
  uint32_t delay = atoi(argv[2]);

  cli_transmit(cli, "reading for: %.3f seconds \r\n", (double)duration/1000);

  for (uint32_t i = 0; i < duration; i += delay) {
    lis3mdl_read_mag(&mag, reading);
    reading[0] = reading[0]*100000;
    reading[1] = reading[1]*100000;
    reading[2] = reading[2]*100000;
    cli_transmit(cli, "%.2f,%.2f,%.2f\r\n", reading[0], reading[1], reading[2]);
    HAL_Delay(delay);
  }

  return 0;
}

uint8_t command_hgimu_toggle(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  if (strcmp(argv[1], "on") == 0) {
    HAL_GPIO_WritePin(HGUIDE_NRST_GPIO_Port, HGUIDE_NRST_Pin, GPIO_PIN_SET);
  } else if (strcmp(argv[1], "off") == 0) {
    HAL_GPIO_WritePin(HGUIDE_NRST_GPIO_Port, HGUIDE_NRST_Pin, GPIO_PIN_RESET);
  }

  return 0;
}

uint8_t command_gps_assist(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 1) {
    return 1;
  }

  uint8_t buf[8];
  ublox_protocol_encode(0x13, 0x80, NULL, 0, buf);
  for (uint16_t i = 0; i < 8; i++) {
    cli_transmit(cli, "0x%02x ", buf[i]);
  }
  ublox_gnss_send_msg(&gps, buf, 8);

  return 0;
}

uint8_t command_arm(struct cli_handle *cli, uint16_t argc, char *argv[]) {
  if (argc != 2) {
    return 1;
  }

  // state_machine_handle_event(&state_machine, &(struct event) {EVENT_COMMAND,
  //     (void *)(uintptr_t)(atoi(argv[1]))});

  return 0;
}
