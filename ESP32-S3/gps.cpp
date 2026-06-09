#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "config.h"

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

volatile float longitude = 0;
volatile float latitude = 0;
volatile bool fixGPS = false;

void getLocationGPS(){
  while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }
  if (gps.location.isValid() && gps.location.isUpdated()) {
    fixGPS = true;
    longitude = gps.location.lat();
    latitude = gps.location.lng();
  } else {
    fixGPS = false;
  }
}