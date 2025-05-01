#pragma once

#include <stdbool.h>
#include <stdint.h>

enum Pin {
    // UART0_TX = 0,
    // UART0_RX = 1,
    // UNUSED = 2,
    PIN_D0 = 3,
    PIN_D1 = 4,
    PIN_D2 = 5,
    PIN_D3 = 6,
    PIN_D4 = 7,
    PIN_D5 = 8,
    PIN_D6 = 9,
    PIN_D7 = 10,
    PIN_A0 = 11,
    PIN_RD = 12,
    PIN_WR = 13,
    PIN_CS = 14,
    PIN_RST = 15,
};

// To-do: Move this to the main file later
extern bool resetPending;
extern uint32_t lastLowEvent;

void resetCallback(unsigned int gpio, uint32_t events);