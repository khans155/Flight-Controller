#pragma once
#include <Arduino.h>
#include <WiFi.h>

extern WiFiServer telnetServer;
extern WiFiClient telnetClient;

void initWiFi();
void remotePrint(const char* format, ...);
void handleRemoteCommands();