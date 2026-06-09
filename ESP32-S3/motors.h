#pragma once
#include <Arduino.h>
#include "pid.h"
#include "config.h"



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
    int8_t   state;
    uint16_t throttle;
    uint16_t battMv;      // millivolts (0 = not implemented yet)
    uint8_t  crc;
};


#define RADIO_PAYLOAD_SIZE  sizeof(TelemetryPacket)   // 15 bytes

#define NOP1 __asm__ __volatile__("nop")
#define NOP4 NOP1;NOP1;NOP1;NOP1
#define NOP16 NOP4;NOP4;NOP4;NOP4
#define NOP32 NOP16;NOP16

#define T1H_NOPS  NOP32;NOP32;NOP32;NOP32;NOP32;NOP32     // ~192 nops
#define T0H_NOPS  NOP32;NOP32;NOP32                        // ~96 nops
#define T1L_NOPS  NOP32;NOP32;NOP32;NOP4                   // ~108 nops
#define T0L_NOPS  NOP32;NOP32;NOP32;NOP32;NOP32;NOP32;NOP16;NOP4 

// PID controllers (accessible by comms.cpp for live tuning)
extern PID DRAM_ATTR velXPID, velYPID;
extern PID DRAM_ATTR rollAnglePID, pitchAnglePID;
extern PID DRAM_ATTR rollRatePID, pitchRatePID;
extern PID DRAM_ATTR yawRatePID, altPID ;

// Motor offsets 
extern float DRAM_ATTR m1_offset, m2_offset, m3_offset, m4_offset;
extern float DRAM_ATTR pitch_offset, roll_offset;

// Motor outputs 
extern uint16_t DRAM_ATTR m1, m2, m3, m4;
extern uint16_t DRAM_ATTR m1_corr, m2_corr, m3_corr, m4_corr;

// RC inputs 
extern uint16_t DRAM_ATTR throttle;
extern int DRAM_ATTR rollOffset, pitchOffset, yawOffset;

// State variables
extern bool DRAM_ATTR armed;
extern bool DRAM_ATTR altHoldActive;

extern float DRAM_ATTR real_height;

// Throttle-lock mode state (activated/deactivated with Circle button)
extern bool DRAM_ATTR throttleLockActive;

//Batt
extern uint16_t batt_rx;

// Utility
float IRAM_ATTR floatMap(float x, float in_min, float in_max, float out_min, float out_max);
float IRAM_ATTR expoCurve(float x, float expo);

void IRAM_ATTR argb_set_color(uint8_t r, uint8_t g, uint8_t b);

void initMotors();
uint8_t IRAM_ATTR dshotCRC(uint16_t packet);
void IRAM_ATTR sendDshot(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void IRAM_ATTR mixMotor(uint64_t now);
void readPS5();
void sendTelemetry();
void applyTuneCommand(uint8_t paramId, float value);

void IRAM_ATTR emergencyMotorStop();

