#include "mahony.h"
#include "config.h"
#include <math.h>

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float gxBias = 0.0f, gyBias = 0.0f, gzBias = 0.0f;

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt)
{
  float accNorm = sqrtf(ax*ax + ay*ay + az*az);
  if (accNorm < 0.01f) return;

  float accelDeviation = fabsf(accNorm - 1.0f);
  float accelWeight = 0.0f;
  if (accelDeviation < 0.2f) {  
    float t = accelDeviation / 0.2f;
    accelWeight = 0.5f * (1.0f + cosf(t * M_PI));
  }
  ax /= accNorm; ay /= accNorm; az /= accNorm;

  float vx = 2.0f * (q1*q3 - q0*q2);
  float vy = 2.0f * (q0*q1 + q2*q3);
  float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);

  float ex = (ay*vz - az*vy) * accelWeight;
  float ey = (az*vx - ax*vz) * accelWeight;
  float ez = (ax*vy - ay*vx) * accelWeight;

  gxBias += ex * MAHONY_KI * dt;
  gyBias += ey * MAHONY_KI * dt;
  gzBias += ez * MAHONY_KI * dt;

  float gxC = gx * DEG_TO_RAD + (MAHONY_KP * ex) + gxBias;
  float gyC = gy * DEG_TO_RAD + (MAHONY_KP * ey) + gyBias;
  float gzC = gz * DEG_TO_RAD + (MAHONY_KP * ez) + gzBias;

  float q0_old = q0, q1_old = q1, q2_old = q2, q3_old = q3;
  q0 += (-q1_old*gxC - q2_old*gyC - q3_old*gzC) * 0.5f * dt;
  q1 += ( q0_old*gxC + q2_old*gzC - q3_old*gyC) * 0.5f * dt;
  q2 += ( q0_old*gyC - q1_old*gzC + q3_old*gxC) * 0.5f * dt;
  q3 += ( q0_old*gzC + q1_old*gyC - q2_old*gxC) * 0.5f * dt;

  float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;
  
}

void quaternionToEuler(float &roll, float &pitch, float &yaw) {
  roll  =  atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
  pitch = -asinf(2.0f*(q0*q2 - q3*q1)) * RAD_TO_DEG;
  yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
  if (yaw < 0.0f) yaw += 360.0f;

}
void updatequaternion(float &new_q0, float &new_q1, float &new_q2, float &new_q3){
  q0 = new_q0;
  q1 = new_q1;
  q2 = new_q2;
  q3 = new_q3;
}

void getWorldAcceleration(float ax, float ay, float az, float &worldX, float &worldY) {
    // 1. Rotate the body-frame acceleration vector into the world frame
    // This uses the standard quaternion rotation: v' = q * v * q_conj
    
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    worldX = ax * (q0q0 + q1q1 - q2q2 - q3q3) + ay * 2.0f * (q1q2 - q0q3) + az * 2.0f * (q1q3 + q0q2);
    worldY = ax * 2.0f * (q1q2 + q0q3) + ay * (q0q0 - q1q1 + q2q2 - q3q3) + az * 2.0f * (q2q3 - q0q1);

    worldX *= 981.0f; 
    worldY *= 981.0f;
    // worldZ *= 981.0f;
}