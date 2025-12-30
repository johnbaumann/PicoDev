#pragma once

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
};

// To-do: Move this to the main file later
extern volatile bool g_resetPending;
