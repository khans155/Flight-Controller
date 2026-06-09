#pragma once
#include <Arduino.h>

// Call once per loop with filtered sensor values
// gx/gy/gz in deg/s, ax/ay/az in g (unnormalized), dt in seconds

void IRAM_ATTR mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt);

// Extracts roll and pitch (degrees) from internal quaternion
void IRAM_ATTR quaternionToEuler(float &roll, float &pitch, float &yaw);

void IRAM_ATTR updatequaternion(float &new_q0, float &new_q1, float &new_q2, float &new_q3);
void IRAM_ATTR getWorldAcceleration(float ax, float ay, float az, float &worldX, float &worldY);
void getWorldVelocity(float vx, float vy, float &worldX, float &worldY);
void getBodyVelocity(float vx_world, float vy_world, float &bodyX, float &bodyY);
void IRAM_ATTR getLinearAcceleration(float ax, float ay, float az, float &linAx, float &linAy);
