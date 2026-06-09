#pragma once
#include "kalman.h"
#include <Arduino.h>
#include <SPI.h>
#include "driver/i2c.h"

struct BiquadState {
    float x1, x2, y1, y2; 
};

struct BiquadCoeffs {
    float b0, b1, b2, a1, a2;
};


// Filtered sensor outputs (read by motor mixing)
extern float DRAM_ATTR gx_f, gy_f, gz_f;
extern float DRAM_ATTR ax_f, ay_f, az_f;
extern float DRAM_ATTR mx_f, my_f, mz_f;
extern float DRAM_ATTR roll_ag, pitch_ag, yaw_ag;
extern float DRAM_ATTR yaw_mag;
extern float DRAM_ATTR ax_world, ay_world;
extern float DRAM_ATTR vx_est, vy_est;

extern volatile float DRAM_ATTR vx, vy, vz, vx_f, vy_f, vz_f;
extern volatile float DRAM_ATTR height_lidar;
extern volatile float DRAM_ATTR height_baro;
extern volatile bool DRAM_ATTR newLidar;
extern volatile bool DRAM_ATTR baroFlag;
extern volatile uint8_t DRAM_ATTR  flow_quality;
extern volatile float DRAM_ATTR height_filtered;
extern volatile float DRAM_ATTR gyro_lpf;
extern volatile float DRAM_ATTR gyro_notch;

extern Kalman1D DRAM_ATTR kx;
extern Kalman1D DRAM_ATTR ky;

extern int16_t DRAM_ATTR gyroX, gyroY, gyroZ;
extern int16_t DRAM_ATTR accX, accY, accZ;
extern int16_t magX, magY, magZ;

void i2c_ext_init();
void writeRegister(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val);
uint8_t readRegister(i2c_port_t port, uint8_t addr, uint8_t reg);

void IRAM_ATTR spiWrite(uint8_t reg, uint8_t val);
void IRAM_ATTR spiRead(uint8_t reg, uint8_t* buf, uint8_t len);

void initSensors();        
void initFilter();   
void calibrateMPU();
void IRAM_ATTR readMPU();
void readMag();
void IRAM_ATTR applyMPUCalibration(int16_t &gx, int16_t &gy, int16_t &gz,
                         int16_t &ax, int16_t &ay, int16_t &az);
void applyMagCalibration(float &x, float &y, float &z);
void IRAM_ATTR updateFilter(uint64_t now);