#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include <stdint.h>

#include "cli/cli.h"

uint8_t command_init(struct cli_handle *cli);

uint8_t command_set_dac_output(struct cli_handle *cli, uint16_t argc,
                               char *argv[]);
uint8_t command_help(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_led_toggle(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_gps_nmea_toggle(struct cli_handle *cli, uint16_t argc,
                                char *argv[]);
uint8_t command_baro_test(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_pyro_fire(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_pyro_read_continuity(struct cli_handle *cli, uint16_t argc,
                                     char *argv[]);
uint8_t command_clear(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_mag_read(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_mag_burst_read(struct cli_handle *cli, uint16_t argc, char *argv[]);

uint8_t command_test_gnc(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_test_qmekf(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_test_ekf(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_rx_sensor_sim(struct cli_handle *cli, uint16_t argc,
                              char *argv[]);
uint8_t command_rx_ekf_sim(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_sd_close_log(struct cli_handle *cli, uint16_t argc,
                             char *argv[]);
uint8_t command_hgimu_toggle(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_gps_assist(struct cli_handle *cli, uint16_t argc, char *argv[]);

uint8_t command_arm(struct cli_handle *cli, uint16_t argc, char *argv[]);
uint8_t command_gps_get_numsv(struct cli_handle *cli, uint16_t argc, char *argv[]);

#endif /* __COMMANDS_H__ */
