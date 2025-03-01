#ifndef __TELEMETRY_MANAGER_H__
#define __TELEMETRY_MANAGER_H__

#include "managers/actuator_manager/actuator_manager.h"
#include "managers/sensor_manager/sensor_manager.h"
#include "state_machine_impl/state_machine_impl.h"
#include <stdint.h>

#define TELEMETRY_MANAGER_TX_INTERVAL_MS      1000
#define TELEMETRY_MANAGER_MAX_ROW_LENGTH      512
#define TELEMETRY_MANAGER_TELEM_PACKET_LENGTH 53
#define TELEMETRY_MANAGER_PACKET_PREAMBLE_1   0xB5
#define TELEMETRY_MANAGER_PACKET_PREAMBLE_2   0x62
#define TELEMETRY_MANAGER_PACKET_ID_TELEMETRY 0x00
#define TELEMETRY_MANAGER_PACKET_ID_ACK       0xFF
#define TELEMETRY_MANAGER_ACK_PACKET_LENGTH   6

enum telemetry_manager_err {
  TELEMETRY_MANAGER_ERR_OK,
};

enum telemetry_manager_command {
  TELEMETRY_MANAGER_COMMAND_NONE,
  TELEMETRY_MANAGER_COMMAND_ARM,
  TELEMETRY_MANAGER_COMMAND_ABORT,
  TELEMETRY_MANAGER_COMMAND_PING
};

struct telemetry_manager_instance {
  enum telemetry_manager_command recently_received_command;
  void *htim;
  void *huart;
};

enum telemetry_manager_err
telemetry_manager_init(struct telemetry_manager_instance *manager);

enum telemetry_manager_err
telemetry_manager_update(struct telemetry_manager_instance *manager,
                         struct sensor_manager_instance *sensor_manager,
                         struct actuator_manager_instance *actuator_manager,
                         struct state_machine *state_machine);

enum telemetry_manager_err
telemetry_manager_sd_card_init(struct telemetry_manager_instance *manager);

enum telemetry_manager_err telemetry_manager_sd_card_write_row(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine);

enum telemetry_manager_err telemetry_manager_transmit_telemetry(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine);

enum telemetry_manager_err telemetry_manager_transmit_ack(
    struct telemetry_manager_instance *manager,
    struct sensor_manager_instance *sensor_manager,
    struct actuator_manager_instance *actuator_manager,
    struct state_machine *state_machine);
#endif /* __TELEMETRY_MANAGER_H__ */
