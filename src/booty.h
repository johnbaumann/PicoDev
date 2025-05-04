#pragma once

#include <stdbool.h>
#include <stdint.h>

extern volatile bool BOOTY_transferComplete;

void BOOTY_deinit();
int BOOTY_arm();