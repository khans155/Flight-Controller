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
extern volatile float height_baro;
extern volatile bool newLidar;
extern volatile bool baroFlag;
extern volatile uint8_t  flow_quality;
extern volatile float height_filtered;
extern volatile float gyro_lpf;
extern volatile float gyro_notch;

extern Kalman1D kx;
extern Kalman1D ky;

extern int16_t gyroX, gyroY, gyroZ, gyro2X, gyro2Y, gyro2Z;
extern int16_t accX, accY, accZ, acc2X, acc2Y, acc2Z;
extern int16_t magX, magY, magZ;

void i2c_init();
void writeRegister(in_port_t port, uint8_t addr, uint8_t reg, uint8_t val);
uint8_t readRegister(in_port_t port, uint8_t addr, uint8_t reg);
void initSensors();        
void initFilter();   
void calibrateMPU();
void readMPU();
void readMPU2();
void readMag();
void applyMPUCalibration1(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az);

void applyMPUCalibration2(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az);
void applyMagCalibration(float &x, float &y, float &z);
void updateFilter();