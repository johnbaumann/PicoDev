#include "picodev.h"

#include <hardware/pio.h>

void deinitStateMachine(const StateMachine *stateMachine) {
    pio_sm_set_enabled(stateMachine->pio, stateMachine->sm, false);
    pio_sm_clear_fifos(stateMachine->pio, stateMachine->sm);
    pio_remove_program(stateMachine->pio, stateMachine->program, stateMachine->offset);
}

void initStateMachine(StateMachine *stateMachine) {
    stateMachine->offset = pio_add_program(stateMachine->pio, stateMachine->program);
    if (stateMachine->init) {
        stateMachine->init(stateMachine->pio, stateMachine->sm, stateMachine->offset);
    }
}