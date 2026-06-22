#pragma once

#include <hardware/pio.h>
#include <stdbool.h>

enum Pin {
    PIN_RST = 0u,
    PIN_CS = 1u,
    PIN_D0 = 3u,
    PIN_D1 = 4u,
    PIN_D2 = 5u,
    PIN_D3 = 6u,
    PIN_D4 = 7u,
    PIN_D5 = 8u,
    PIN_D6 = 9u,
    PIN_D7 = 10u,
    PIN_RD = 12u,
    PIN_WR = 13u,
    PIN_A0 = 14u,
    // The pins below are not physical connections
    // These are used for internal purposes only
    STATUS_D0 = 17u,
    STATUS_D1 = 18u,
    STATUS_D2 = 19u,
    STATUS_D3 = 20u,
    STATUS_D4 = 21u,
    STATUS_D5 = 22u,
    STATUS_D6 = 23u,
    STATUS_D7 = 24u,
};

typedef struct {
    void (*const init)(const PIO, const unsigned int, const unsigned int);  // pio, sm, offset
    unsigned int offset;
    const unsigned int sm;
    const PIO pio;
    const struct pio_program *const program;
} StateMachine;

void deinitStateMachine(const StateMachine *stateMachine);
void initStateMachine(StateMachine *stateMachine);

// To-do: Move this to the main file later
extern volatile bool g_resetPending;
