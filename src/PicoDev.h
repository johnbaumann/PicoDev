#pragma once

#include <stdbool.h>

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
    PIN_RST = 11,
    PIN_RD = 12,
    PIN_WR = 13,
    PIN_CS = 14,
    PIN_A0 = 15,
    STATUS_D0 = 16,
    STATUS_D1 = 17,
    STATUS_D2 = 18,
    STATUS_D3 = 19,
    STATUS_D4 = 20,
    STATUS_D5 = 21,
    STATUS_D6 = 22,
    STATUS_D7 = 23,
};

// To-do: Move this to the main file later
extern volatile bool g_resetPending;
