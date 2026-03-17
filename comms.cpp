#include "comms.h"
#include "config.h"
#include "motors.h"
#include <stdarg.h>

WiFiServer telnetServer(23);
WiFiClient telnetClient;

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  telnetServer.begin();
  telnetServer.setNoDelay(true);
}

void remotePrint(const char* format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(buffer);
  }
}

void handleRemoteCommands() {
  if (!telnetClient || !telnetClient.available()) return;
  String cmd = telnetClient.readStringUntil('\n');
  cmd.trim();

  if      (cmd.startsWith("RP "))  { rollRatePID.kp  = pitchRatePID.kp  = cmd.substring(3).toFloat(); remotePrint("Rate KP=%.4f\r\n",  rollRatePID.kp); }
  else if (cmd.startsWith("RPa ")) { rollAnglePID.kp = pitchAnglePID.kp = cmd.substring(4).toFloat(); remotePrint("Angle KP=%.4f\r\n", rollAnglePID.kp); }
  else if (cmd.startsWith("RI "))  { rollRatePID.ki  = pitchRatePID.ki  = cmd.substring(3).toFloat(); remotePrint("Rate KI=%.4f\r\n",  rollRatePID.ki); }
  else if (cmd.startsWith("RIa ")) { rollAnglePID.ki = pitchAnglePID.ki = cmd.substring(4).toFloat(); remotePrint("Angle KI=%.4f\r\n", rollAnglePID.ki); }
  else if (cmd.startsWith("RD "))  { rollRatePID.kd  = pitchRatePID.kd  = cmd.substring(3).toFloat(); remotePrint("Rate KD=%.4f\r\n",  rollRatePID.kd); }
  else if (cmd.startsWith("RDa ")) { rollAnglePID.kd = pitchAnglePID.kd = cmd.substring(4).toFloat(); remotePrint("Angle KD=%.4f\r\n", rollAnglePID.kd); }
  else if (cmd.startsWith("YP "))  { yawRatePID.kp   = cmd.substring(3).toFloat(); remotePrint("Yaw Rate KP=%.4f\r\n", yawRatePID.kp); }
  else if (cmd.startsWith("YI "))  { yawRatePID.ki   = cmd.substring(3).toFloat(); remotePrint("Yaw Rate KI=%.4f\r\n", yawRatePID.ki); }
  else if (cmd.startsWith("YD "))  { yawRatePID.kd   = cmd.substring(3).toFloat(); remotePrint("Yaw Rate KD=%.4f\r\n", yawRatePID.kd); }
  else if (cmd.startsWith("VP "))  { velXPID.kp      = velYPID.kp       = cmd.substring(3).toFloat(); remotePrint("Vel KP=%.4f\r\n",      velXPID.kp); }
  else if (cmd.startsWith("VI "))  { velXPID.ki      = velYPID.ki       = cmd.substring(3).toFloat(); remotePrint("Vel KI=%.4f\r\n",      velXPID.ki); }
  else if (cmd.startsWith("VD "))  { velXPID.kd      = velYPID.kd       = cmd.substring(3).toFloat(); remotePrint("Vel KD=%.4f\r\n",      velXPID.kd); }
  else if (cmd.startsWith("AP "))  { altPID.kp = cmd.substring(3).toFloat(); remotePrint("Alt KP=%.4f\r\n", altPID.kp); }
  else if (cmd.startsWith("AI "))  { altPID.ki = cmd.substring(3).toFloat(); remotePrint("Alt KI=%.4f\r\n", altPID.ki); }
  else if (cmd.startsWith("AD "))  { altPID.kd = cmd.substring(3).toFloat(); remotePrint("Alt KD=%.4f\r\n", altPID.kd); }
  else if (cmd.startsWith("M1 "))  { m1_offset = cmd.substring(3).toFloat(); remotePrint("M1 offset=%.3f\r\n", m1_offset); }
  else if (cmd.startsWith("M2 "))  { m2_offset = cmd.substring(3).toFloat(); remotePrint("M2 offset=%.3f\r\n", m2_offset); }
  else if (cmd.startsWith("M3 "))  { m3_offset = cmd.substring(3).toFloat(); remotePrint("M3 offset=%.3f\r\n", m3_offset); }
  else if (cmd.startsWith("M4 "))  { m4_offset = cmd.substring(3).toFloat(); remotePrint("M4 offset=%.3f\r\n", m4_offset); }
  else if (cmd.startsWith("PO "))  { roll_offset = cmd.substring(3).toFloat(); remotePrint("Pitch offset=%.3f\r\n", roll_offset); }
  else if (cmd.startsWith("RO "))  { pitch_offset  = cmd.substring(3).toFloat(); remotePrint("Roll offset=%.3f\r\n",  pitch_offset);  }
}