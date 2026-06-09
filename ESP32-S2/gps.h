#pragma once
#include <Arduino.h>


extern HardwareSerial gpsSerial;

extern volatile float longitude;
extern volatile float latitude;
extern volatile bool fixGPS;

void getLocationGPS();