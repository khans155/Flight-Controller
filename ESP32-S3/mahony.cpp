#include "mahony.h"
#include "config.h"
#include <math.h>

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float gxBias = 0.0f, gyBias = 0.0f, gzBias = 0.0f;


void IRAM_ATTR mahonyUpdate(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt)
{
    if (dt <= 0.0f || dt > 0.02f) return;

    float accNorm = sqrtf(ax*ax + ay*ay + az*az);

    float accelWeight = 0.0f;

    if (accNorm > 0.01f) {
        float accelDeviation = fabsf(accNorm - 1.0f);

        if (accelDeviation < 0.08f) {
            accelWeight = 1.0f;
        } else if (accelDeviation < 0.20f) {
            accelWeight = 1.0f - (accelDeviation - 0.08f) / 0.12f;
        } else {
            accelWeight = 0.0f;
        }

        ax /= accNorm;
        ay /= accNorm;
        az /= accNorm;
    }

    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);

    // Reduce accelerometer correction during fast rotation
    if (gyroMag > 100.0f && gyroMag < 250.0f) {
        accelWeight *= 1.0f - (gyroMag - 100.0f) / 150.0f;
    } else if (gyroMag >= 250.0f) {
        accelWeight = 0.0f;
    }

    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    if (accelWeight > 0.0f) {
        // Estimated gravity direction in body frame
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);

        // Error between measured gravity and estimated gravity
        ex = (ay*vz - az*vy) * accelWeight;
        ey = (az*vx - ax*vz) * accelWeight;
        ez = (ax*vy - ay*vx) * accelWeight;

        // No magnetometer active: yaw is not observable from accel
        ez = 0.0f;
    }

    // Bias learning only when accel is trusted and rotation is not too fast
    if (accelWeight > 0.0f && gyroMag < HIGH_GYRO_THRESH_DPS) {
        gxBias += ex * MAHONY_KI * dt;
        gyBias += ey * MAHONY_KI * dt;

        gxBias = constrain(gxBias, -MAHONY_BIAS_LIMIT_RAD, MAHONY_BIAS_LIMIT_RAD);
        gyBias = constrain(gyBias, -MAHONY_BIAS_LIMIT_RAD, MAHONY_BIAS_LIMIT_RAD);
    }

    // No mag: do not estimate yaw bias from accel
    gzBias = 0.0f;

    float gxC = gx * DEG_TO_RAD + (MAHONY_KP * ex) + gxBias;
    float gyC = gy * DEG_TO_RAD + (MAHONY_KP * ey) + gyBias;
    float gzC = gz * DEG_TO_RAD;

    float q0_old = q0;
    float q1_old = q1;
    float q2_old = q2;
    float q3_old = q3;

    q0 += (-q1_old*gxC - q2_old*gyC - q3_old*gzC) * 0.5f * dt;
    q1 += ( q0_old*gxC + q2_old*gzC - q3_old*gyC) * 0.5f * dt;
    q2 += ( q0_old*gyC - q1_old*gzC + q3_old*gxC) * 0.5f * dt;
    q3 += ( q0_old*gzC + q1_old*gyC - q2_old*gxC) * 0.5f * dt;

    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (qNorm > 0.0001f && isfinite(qNorm)) {
        q0 /= qNorm;
        q1 /= qNorm;
        q2 /= qNorm;
        q3 /= qNorm;
    } else {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        gxBias = gyBias = gzBias = 0.0f;
    }
}

void IRAM_ATTR quaternionToEuler(float &roll, float &pitch, float &yaw) {
    roll = atan2f(
        2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)
    ) * RAD_TO_DEG;

    float sinPitch = 2.0f * (q0*q2 - q3*q1);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    pitch = -asinf(sinPitch) * RAD_TO_DEG;

    yaw = atan2f(
        2.0f * (q0*q3 + q1*q2),
        1.0f - 2.0f * (q2*q2 + q3*q3)
    ) * RAD_TO_DEG;

    if (yaw < 0.0f) yaw += 360.0f;
}

void IRAM_ATTR updatequaternion(float &new_q0, float &new_q1, float &new_q2, float &new_q3){
  q0 = new_q0;
  q1 = new_q1;
  q2 = new_q2;
  q3 = new_q3;
}

void IRAM_ATTR getWorldAcceleration(float ax, float ay, float az, float &worldX, float &worldY) {
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

void getWorldVelocity(float vx, float vy, float &worldX, float &worldY) {
    // Same quaternion rotation as getWorldAcceleration
    // but no gravity removal and no unit scaling — velocity is already in cm/s

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;

    // Only need the X and Y rows of the rotation matrix
    // vz from flow sensor is 0 so those terms drop out
    worldX = vx * (q0q0 + q1q1 - q2q2 - q3q3) + vy * 2.0f * (q1q2 - q0q3);
    worldY = vx * 2.0f * (q1q2 + q0q3)         + vy * (q0q0 - q1q1 + q2q2 - q3q3);
}

void getBodyVelocity(float vx_world, float vy_world, float &bodyX, float &bodyY) {
    // Inverse (transpose) of getWorldVelocity rotation matrix
    // Swaps the sign on the q0q3 cross terms in the off-diagonal elements

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;

    bodyX = vx_world * (q0q0 + q1q1 - q2q2 - q3q3) + vy_world * 2.0f * (q1q2 + q0q3);
    bodyY = vx_world * 2.0f * (q1q2 - q0q3)         + vy_world * (q0q0 - q1q1 + q2q2 - q3q3);
}

void IRAM_ATTR getLinearAcceleration(float ax, float ay, float az, float &linAx, float &linAy) {
    // Calculate the Direction of Gravity (G) in the Body Frame based on the Quaternion
    float gx = 2.0f * (q1 * q3 - q0 * q2);
    float gy = 2.0f * (q0 * q1 + q2 * q3);
    float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    linAx = ax - gx;
    linAy = ay - gy;
    linAx *= 981.0f; 
    linAy *= 981.0f;
}