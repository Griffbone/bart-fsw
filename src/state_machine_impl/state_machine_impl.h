#ifndef __STATE_MACHINE_IMPL_H__
#define __STATE_MACHINE_IMPL_H__

#include <stdint.h>

enum state_machine_err {
  STATE_MACHINE_ERR_OK,
};

enum state_machine_state {
  STATE_MACHINE_STATE_PAD_SAFE,
  STATE_MACHINE_STATE_PAD_ARMED,
  STATE_MACHINE_STATE_BOOST,
  STATE_MACHINE_STATE_APOGEE,
  STATE_MACHINE_STATE_DESCENT,
  STATE_MACHINE_STATE_LANDED,
};

enum state_machine_signal {
  STATE_MACHINE_SIGNAL_NO_SIGNAL,
  STATE_MACHINE_SIGNAL_ARM,
  STATE_MACHINE_SIGNAL_UMBILICAL_DISCONNECT,
  STATE_MACHINE_SIGNAL_APOGEE_DETECTED,
  STATE_MACHINE_SIGNAL_DESCENT_DETECTED,
  STATE_MACHINE_SIGNAL_LANDING_DETECTED,
  STATE_MACHINE_SIGNAL_ABORT,
};

struct state_machine {
  enum state_machine_state state;
  enum state_machine_signal signal;
  uint32_t boost_time;
};

enum state_machine_err state_machine_init(struct state_machine *machine);
enum state_machine_err state_machine_update(struct state_machine *machine);


#endif /* __STATE_MACHINE_IMPL_H__ */
