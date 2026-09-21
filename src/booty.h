#pragma once

#include <stdbool.h>

extern volatile bool g_bootyTransferComplete;

void BOOTY_arm(void);
void BOOTY_deinit(void);