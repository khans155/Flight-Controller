#pragma once
#include <Arduino.h>

struct Kalman1D {
    float x;   // velocity estimate
    float p;   // error covariance
    float q;   // process noise  — tune: ~10-50 cm/s² * dt²
    float r;   // measurement noise — tune: ~5-15 cm/s variance
};



void kalman_predict(Kalman1D *k, float accel, float dt);
void kalman_correct(Kalman1D *k, float measurement, uint8_t quality);
