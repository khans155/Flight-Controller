#pragma once
#include <Arduino.h>

// Call once per loop with filtered sensor values
// gx/gy/gz in deg/s, ax/ay/az in g (unnormalized), dt in seconds

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt);

// Extracts roll and pitch (degrees) from internal quaternion
void quaternionToEuler(float &roll, float &pitch, float &yaw);

void updatequaternion(float &new_q0, float &new_q1, float &new_q2, float &new_q3);
void getWorldAcceleration(float ax, float ay, float az, float &worldX, float &worldY);