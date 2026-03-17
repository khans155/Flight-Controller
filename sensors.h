#pragma once
#include "kalman.h"
#include <Arduino.h>

struct BiquadState {
    float x1, x2, y1, y2; 
};

struct BiquadCoeffs {
    float b0, b1, b2, a1, a2;
};


// Filtered sensor outputs (read by motor mixing)
extern float gx_f, gy_f, gz_f;
extern float ax_f, ay_f, az_f;
extern float mx_f, my_f, mz_f;
extern float roll_ag, pitch_ag, yaw_ag;
extern float yaw_mag;
extern float ax_world, ay_world;
extern float vx_est, vy_est;

extern volatile float vx, vy, vz, vx_f, vy_f, vz_f;
extern volatile float height_lidar;
extern volatile bool newLidar;
extern volatile uint8_t  flow_quality;
extern Kalman1D kx;
extern Kalman1D ky;


// Raw sensor values (needed by calibrateMPU)
extern int16_t gyroX, gyroY, gyroZ;
extern int16_t accX, accY, accZ;
extern int16_t magX, magY, magZ;

extern SemaphoreHandle_t magMutex;

void initSensors();        // Wire setup + MPU/mag register writes
void initFilter();   
void calibrateMPU();
void readMPU();
void readMag();
void applyMPUCalibration(int16_t &gx, int16_t &gy, int16_t &gz,
                         int16_t &ax, int16_t &ay, int16_t &az);
void applyMagCalibration(float &x, float &y, float &z);
void updateFilter();