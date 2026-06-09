#pragma once
#include <Arduino.h>


void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt);

void quaternionToEuler(float &roll, float &pitch, float &yaw);

void updatequaternion(float &new_q0, float &new_q1, float &new_q2, float &new_q3);
void getWorldAcceleration(float ax, float ay, float az, float &worldX, float &worldY);
void getWorldVelocity(float vx, float vy, float &worldX, float &worldY);
void getBodyVelocity(float vx_world, float vy_world, float &bodyX, float &bodyY);
void getLinearAcceleration(float ax, float ay, float az, float &linAx, float &linAy);
