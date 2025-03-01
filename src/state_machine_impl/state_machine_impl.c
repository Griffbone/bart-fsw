#include "state_machine_impl.h"
#include "system/tim.h"

const uint32_t STATE_MACHINE_TIME_AT_APOGEE = 145*100000;
const uint32_t STATE_MACHINE_TIME_AT_LANDED = 304*1000000;
// const uint32_t STATE_MACHINE_TIME_AT_LANDED = 200*100000;

enum state_machine_err state_machine_init(struct state_machine *machine) {
  machine->state = STATE_MACHINE_STATE_PAD_SAFE;
  machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
  
  __HAL_TIM_SET_COUNTER(&htim23, 0);

  return STATE_MACHINE_ERR_OK;
};

enum state_machine_err state_machine_update(struct state_machine *machine) {
  switch (machine->state) {
    case STATE_MACHINE_STATE_PAD_SAFE:
      HAL_TIM_Base_Stop(&htim23);
      __HAL_TIM_SET_COUNTER(&htim23, 0);

      if (machine->signal == STATE_MACHINE_SIGNAL_ARM) {
        machine->state = STATE_MACHINE_STATE_PAD_ARMED;
      } 
      break;
    case STATE_MACHINE_STATE_PAD_ARMED:
        if (machine->signal == STATE_MACHINE_SIGNAL_UMBILICAL_DISCONNECT) {
          machine->state = STATE_MACHINE_STATE_BOOST;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
          
          HAL_TIM_Base_Start(&htim23);
          __HAL_TIM_SET_COUNTER(&htim23, 0);
        }

        if (machine->signal == STATE_MACHINE_SIGNAL_ABORT) {
          machine->state = STATE_MACHINE_STATE_PAD_SAFE;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
        }
      break;
    case STATE_MACHINE_STATE_BOOST:
        if (__HAL_TIM_GET_COUNTER(&htim23) > STATE_MACHINE_TIME_AT_APOGEE) {
          machine->signal = STATE_MACHINE_SIGNAL_APOGEE_DETECTED;
        }

        if (machine->signal == STATE_MACHINE_SIGNAL_APOGEE_DETECTED) {
          machine->state = STATE_MACHINE_STATE_APOGEE;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
        }

        if (machine->signal == STATE_MACHINE_SIGNAL_ABORT) {
          machine->state = STATE_MACHINE_STATE_PAD_SAFE;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
        }
      break;
    case STATE_MACHINE_STATE_APOGEE:
        if (machine->signal == STATE_MACHINE_SIGNAL_DESCENT_DETECTED) {
          machine->state = STATE_MACHINE_STATE_DESCENT;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
        }
        
        if (machine->signal == STATE_MACHINE_SIGNAL_ABORT) {
          machine->state = STATE_MACHINE_STATE_PAD_SAFE;
          machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
        }
      break;
    case STATE_MACHINE_STATE_DESCENT:
      if (__HAL_TIM_GET_COUNTER(&htim23) > STATE_MACHINE_TIME_AT_LANDED) {
        machine->signal = STATE_MACHINE_SIGNAL_LANDING_DETECTED;
      }

      if (machine->signal == STATE_MACHINE_SIGNAL_LANDING_DETECTED) {
        machine->state = STATE_MACHINE_STATE_LANDED;
        machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
      }

      if (machine->signal == STATE_MACHINE_SIGNAL_ABORT) {
        machine->state = STATE_MACHINE_STATE_PAD_SAFE;
        machine->signal = STATE_MACHINE_SIGNAL_NO_SIGNAL;
      }

      break;
    case STATE_MACHINE_STATE_LANDED:

      break;
  }

  return STATE_MACHINE_ERR_OK;
}
