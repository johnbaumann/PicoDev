#pragma once

#include <stdbool.h>

enum Pin {
    PIN_RST = 0,
    PIN_CS = 1,
    PIN_D0 = 3,
    PIN_D1 = 4,
    PIN_D2 = 5,
    PIN_D3 = 6,
    PIN_D4 = 7,
    PIN_D5 = 8,
    PIN_D6 = 9,
    PIN_D7 = 10,
    PIN_RD = 12,
    PIN_WR = 13,
    PIN_A0 = 14,
    // The pins below are not physical connections
    // These are used for internal purposes only
    STATUS_D0 = 15,
    STATUS_D1 = 16,
    STATUS_D2 = 17,
    STATUS_D3 = 18,
    STATUS_D4 = 19,
    STATUS_D5 = 20,
    STATUS_D6 = 21,
    STATUS_D7 = 22,
};

// To-do: Move this to the main file later
extern volatile bool g_resetPending;
