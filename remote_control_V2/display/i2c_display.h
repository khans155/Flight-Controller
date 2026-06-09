#pragma once
#include <Arduino.h>

#define CYD_I2C_ADDR 0x42 // Arbitrary I2C slave address

enum DisplayPacketType {
    PACKET_TELEM = 0,
    PACKET_TEXT  = 1
};

// Pack the struct to prevent memory alignment mismatches between ESP32-S3 and ESP32
#pragma pack(push, 1)
struct DisplayPacket {
    uint8_t type;
    union {
        struct {
            float roll;
            float pitch;
            float alt;
            float throttle;
            float batt_tx;
            float batt_rx;
            int8_t   state;
        } telem;
        struct {
            char line1[17];
            char line2[17];
            char line3[17];
            char line4[17];
        } text;
    } data;
};
#pragma pack(pop)