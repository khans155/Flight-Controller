#pragma once
#include <Arduino.h>
#include "pid.h"


struct __attribute__((packed)) ControlPacket {
    int8_t   leftX;         //  0
    int8_t   leftY;         //  1
    int8_t   rightX;        //  2
    int8_t   rightY;        //  3
    uint8_t  l2;            //  4
    uint8_t  r2;            //  5
    uint16_t buttons;       //  6-7
    uint8_t  flags;         //  8  — FLAG_TELEM_REQUEST | FLAG_TUNE_CMD
    uint8_t  tuneParamId;   //  9  — TUNE_* constant (valid when FLAG_TUNE_CMD)
    float    tuneValue;     // 10-13
    uint8_t  crc;           // 14  — XOR of bytes 0..13
};  // 15 bytes — matches sizeof(TelemetryPacket) = RADIO_PAYLOAD_SIZE 
 
#define FLAG_TELEM_REQUEST  (1 << 0)
#define FLAG_TUNE_CMD       (1 << 1)

struct __attribute__((packed)) TelemetryPacket {
    int16_t  roll;        // degrees * 100  (from roll_ag)
    int16_t  pitch;       // degrees * 100  (from pitch_ag)
    int16_t  altitude;    // metres  * 100  (from height_filtered)
    int16_t  velocityX;   // m/s     * 1    (from vx_est)
    int16_t  velocityY;   // m/s     * 1    (from vy_est)
    int16_t  velocityZ;   // m/s     * 100  (from vz_f)
    uint16_t battMv;      // millivolts (0 = not implemented yet)
    uint8_t  crc;
};


#define RADIO_PAYLOAD_SIZE  sizeof(TelemetryPacket)   // 15 bytes

// PID controllers (accessible by comms.cpp for live tuning via python GUI)
extern PID velXPID, velYPID;
extern PID rollAnglePID, pitchAnglePID;
extern PID rollRatePID, pitchRatePID;
extern PID yawRatePID, altPID ;

// Motor offsets 
extern float m1_offset, m2_offset, m3_offset, m4_offset;
extern float pitch_offset, roll_offset;

// Motor outputs 
extern uint16_t m1, m2, m3, m4;
extern uint16_t m1_corr, m2_corr, m3_corr, m4_corr;

// RC inputs 
extern uint16_t throttle;
extern int rollOffset, pitchOffset, yawOffset;

extern float real_height;

// Throttle-lock mode state (activated/deactivated with Circle button)
extern bool throttleLockActive;

// Utility
float floatMap(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t usToDuty(uint16_t us);
float expoCurve(float x, float expo);

void initMotors();
void mixMotor();
void readPS5();
void sendTelemetry();
void applyTuneCommand(uint8_t paramId, float value);
void IRAM_ATTR emergencyMotorStop();

