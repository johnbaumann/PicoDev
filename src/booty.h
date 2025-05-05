#pragma once

#include <stdbool.h>

extern volatile bool BOOTY_transferComplete;

void BOOTY_arm(void);
void BOOTY_deinit(void);