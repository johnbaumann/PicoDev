#include "picodev.h"

#include <hardware/pio.h>

void deinitStateMachine(const StateMachine* stateMachine) {
    pio_sm_set_enabled(stateMachine->pio, stateMachine->sm, false);
    pio_sm_clear_fifos(stateMachine->pio, stateMachine->sm);
    pio_remove_program(stateMachine->pio, stateMachine->program, stateMachine->offset);
}

void initStateMachine(StateMachine* stateMachine) {
    if (stateMachine->linkedSM) {
        StateMachine* sm = (StateMachine*)stateMachine->linkedSM;
        stateMachine->offset = sm->offset;
    } else {
        stateMachine->offset = pio_add_program(stateMachine->pio, stateMachine->program);
    }
    if (stateMachine->init) {
        stateMachine->init(stateMachine->pio, stateMachine->sm, stateMachine->offset);
    }
    pio_sm_exec_wait_blocking(stateMachine->pio, stateMachine->sm, pio_encode_set(pio_x, stateMachine->xReg));
    if (!stateMachine->disabled) {
        pio_sm_set_enabled(stateMachine->pio, stateMachine->sm, true);
    }
}