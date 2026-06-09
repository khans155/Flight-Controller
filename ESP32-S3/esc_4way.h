#pragma once
#include <Arduino.h>

// Runs a direct BLHeli 4-way serial interface over USB Serial.
// This is intended to be called only when ESC_PASSTHROUGH_MODE == 1.
void esc4wayRun();
