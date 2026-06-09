#pragma once
#include <Arduino.h>
#include <WiFi.h>

extern WiFiServer telnetServer;
extern WiFiClient telnetClient;


#define RPRINT_BUF_SIZE 4096

// Ring buffer — written by any task, drained only by CommsTask
typedef struct {
    char     buf[RPRINT_BUF_SIZE];
    volatile uint32_t head;  // written by producers
    volatile uint32_t tail;  // read by CommsTask
} RPrintRing;

extern RPrintRing rpRing;

void initWiFi();
void remotePrint(const char* format, ...);
void handleRemoteCommands();
