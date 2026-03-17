#include "motors.h"
#include "config.h"
#include "sensors.h"
#include <ps5Controller.h>

// -------------------- PID Controllers --------------------
PID velXPID = {VEL_KP, VEL_KI, VEL_KD, 0, 0, 0, PID_D_ALPHA_VEL, VEL_ANGLE_LIM};
PID velYPID = {VEL_KP, VEL_KI, VEL_KD, 0, 0, 0, PID_D_ALPHA_VEL, VEL_ANGLE_LIM};

PID rollAnglePID  = {ROLL_ANGLE_KP,  ROLL_ANGLE_KI,  ROLL_ANGLE_KD,  0,0,0, PID_D_ALPHA_ANGLE, ROLL_ANGLE_LIM};
PID pitchAnglePID = {PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD, 0,0,0, PID_D_ALPHA_ANGLE, PITCH_ANGLE_LIM};

PID rollRatePID   = {ROLL_RATE_KP,   ROLL_RATE_KI,   ROLL_RATE_KD,   0,0,0, PID_D_ALPHA_RATE,  ROLL_RATE_LIM};
PID pitchRatePID  = {PITCH_RATE_KP,  PITCH_RATE_KI,  PITCH_RATE_KD,  0,0,0, PID_D_ALPHA_RATE,  PITCH_RATE_LIM};
PID yawRatePID    = {YAW_RATE_KP,    YAW_RATE_KI,    YAW_RATE_KD,    0,0,0, PID_D_ALPHA_RATE,  YAW_RATE_LIM};

PID altPID = {ALT_KP, ALT_KI, ALT_KD, 0, 0, 0, PID_D_ALPHA_ALT, ALT_RATE_LIMIT};

// -------------------- Motor state --------------------
float m1_offset = M1_OFFSET_DEFAULT;
float m2_offset = M2_OFFSET_DEFAULT;
float m3_offset = M3_OFFSET_DEFAULT;
float m4_offset = M4_OFFSET_DEFAULT;
float pitch_offset = PITCH_OFFSET, roll_offset =  ROLL_OFFSET;

uint16_t m1, m2, m3, m4;
uint16_t m1_corr, m2_corr, m3_corr, m4_corr;

uint16_t throttle = 1000;
int rollOffset = 0, pitchOffset = 0, yawOffset = 0;

static uint32_t lastMicros = 0;
bool armed = false;

bool altHoldActive = false;
float targetHeight = 0;
uint16_t baseThrottle = 1221; 
float lastValidHeight = 0;
float real_height = 0;

// -------------------- Utilities --------------------
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t usToDuty(uint16_t us) {
  return (uint32_t)((float)us / PWM_PERIOD_US * PWM_MAX);
}

// -------------------- Init --------------------
void initMotors(const char* ps5Mac) {
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PIN, 0);
  ledcAttachPin(MOTOR2_PIN, 1);
  ledcAttachPin(MOTOR3_PIN, 2);
  ledcAttachPin(MOTOR4_PIN, 3);
  ps5.begin(ps5Mac);
}

// -------------------- PS5 Input --------------------
void readPS5() {
if (!ps5.isConnected()) {
    throttle = 1000;
    rollOffset = pitchOffset = yawOffset = 0;
    return;
  }
  static bool xPressedLast = false;
  bool xPressedNow = ps5.Cross();
  real_height =  fabsf(height_lidar* cos(roll_ag* DEG_TO_RAD) * cos(pitch_ag* DEG_TO_RAD));
  if (armed){
    if (xPressedNow && !xPressedLast) {
      if (!altHoldActive) {
        if (height_lidar > 0 && height_lidar <= MAX_ALTITUDE) {
          altHoldActive = true;
          targetHeight = real_height;
          baseThrottle = throttle; 
          altPID.integral = 0;       
        }
      } else {
        altHoldActive = false;
      }
    }
    xPressedLast = xPressedNow;
  }

  // 1. Use signed integers for raw inputs
  int16_t roll_raw  = constrain(ps5.LStickX(), -127, 127);
  int16_t pitch_raw = constrain(ps5.LStickY(), -127, 127);
  int16_t yaw_raw   = constrain(ps5.RStickX(), -127, 127);
  
  int deadzone = 20;
  if (roll_raw  < deadzone && roll_raw  > -deadzone) roll_raw = 0;
  if (pitch_raw < deadzone && pitch_raw > -deadzone) pitch_raw = 0;
  if (yaw_raw   < deadzone && yaw_raw   > -deadzone) yaw_raw = 0;


  uint16_t roll  = floatMap(roll_raw,  -127, 127, 1000, 2000);
  uint16_t pitch = floatMap(-pitch_raw, -127, 127, 1000, 2000);
  uint16_t yaw   = floatMap(yaw_raw,   -127, 127, 1000, 2000);
  
 
// 2. R1 arms motors to idle — only allowed from disarmed (1000)
  if (ps5.R1() && throttle == 1000) {
    throttle = 1221;
    armed = true;
  }


  if (!altHoldActive) {
    // 3. Throttle step control via R2 (increase) and L2 (decrease)
    const int triggerDeadzone = 10;

    uint8_t r2 = ps5.R2Value();
    uint8_t l2 = ps5.L2Value();

    if (r2 <= triggerDeadzone) r2 = 0;
    if (l2 <= triggerDeadzone) l2 = 0;

    static unsigned long lastR2Step = 0;
    static unsigned long lastL2Step = 0;
    unsigned long now = millis();

    // R2 only increases throttle once armed (throttle >= 1221)
    if (r2 > 0 && throttle >= 1221) {
      float rate = floatMap(r2, triggerDeadzone, 255, 1.0f, 100.0f);
      unsigned long interval = (unsigned long)(1000.0f / rate);
      if (now - lastR2Step >= interval) {
        throttle = constrain(throttle + 1, 1221, 2000);
        lastR2Step = now;
      }
    }

    // L2 only decreases throttle once armed, floor is idle (1221)
    if (l2 > 0 && throttle >= 1221) {
      float rate = floatMap(l2, triggerDeadzone, 255, 1.0f, 150.0f);
      unsigned long interval = (unsigned long)(1000.0f / rate);
      if (now - lastL2Step >= interval) {
        throttle = constrain(throttle - 1, 1221, 2000);
        lastL2Step = now;
      }
    }
  }
  // 4. L1 disarms — resets throttle to minimum (1000)
  if (ps5.L1()) {
    throttle = 1000;
    armed = false;
    altHoldActive = false;
  }

  // 5. Final Constraints
  roll     = constrain(roll,     1000, 2000);
  pitch    = constrain(pitch,    1000, 2000);
  throttle = constrain(throttle, 1000, 2000);
  yaw      = constrain(yaw,      1000, 2000);

  // 6. Calculate Offsets
  rollOffset  = roll  - 1500;
  pitchOffset = pitch - 1500;
  yawOffset   = yaw   - 1500;
}

// -------------------- Motor Mixing --------------------
float linearizeMotor(float us)
{
  const float k = 0.7f;  // 0 = no correction, ~0.5–0.7 typical

  // normalize 1000–2000 → 0–1
  float t = (us - 1000.0f) / 1000.0f;
  t = constrain(t, 0.0f, 1.0f);

  // thrust linearization curve
  t = t * (1.0f - k) + (t * t) * k;

  // back to PWM range
  return 1000.0f + t * 1000.0f;
}

void mixMotor() {
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.05f) return;

  bool lowThrottle = (throttle < LOW_THROTTLE_THRESHOLD);
  float desiredRollAngle = 0, desiredPitchAngle = 0;

  if (abs(pitchOffset) > 50) {
      desiredPitchAngle = pitchOffset * ANGLE_SCALE;
      velXPID.integral = 0; 
    } else {
      desiredPitchAngle = computePID(velYPID, vy_est, dt, lowThrottle);
    }
  if (abs(rollOffset) > 50) {
      desiredRollAngle = rollOffset * ANGLE_SCALE;
      velXPID.integral = 0; 
    } else {
      desiredRollAngle = computePID(velXPID, vx_est, dt, lowThrottle);
    }

  float desiredYawRate  = yawOffset   * YAW_RATE_SCALE;

  float dRollRate  = computePID(rollAnglePID,  -desiredRollAngle  - pitch_ag - pitch_offset, dt, lowThrottle);
  float dPitchRate = computePID(pitchAnglePID, desiredPitchAngle - roll_ag - roll_offset,  dt, lowThrottle);

  float rCorr = computePID(rollRatePID,  dRollRate  + gy_f, dt, lowThrottle);
  float pCorr = computePID(pitchRatePID, dPitchRate - gx_f, dt, lowThrottle);
  float yCorr = computePID(yawRatePID,   desiredYawRate + gz_f, dt, lowThrottle);


  // Altitude hold
  float finalThrottle = throttle;
  if (altHoldActive) {
    // 1. Data Validation Gate
    bool heightIsSanityChecked = true;
    
    if (height_lidar <= 0 || height_lidar > MAX_ALTITUDE) {
        heightIsSanityChecked = false; // Out of range
    } else if (abs(real_height - lastValidHeight) > HEIGHT_SPIKE_LIMIT && lastValidHeight != 0) {
        heightIsSanityChecked = false; // Glitch/Spike
    }

    if (heightIsSanityChecked) {
      // 2. Compute PID correction
      float altCorrection = computePID(altPID, targetHeight - real_height, dt, lowThrottle);
      
      // 3. Apply correction to the base throttle
      finalThrottle = baseThrottle + altCorrection;
      
      // Update the manual throttle variable so when we switch off, we stay at this level
      throttle = constrain((uint16_t)finalThrottle, 1221, 2000); 
      lastValidHeight = real_height;
    } else {
      // SAFETY: If sensor fails, just hover at the last known good throttle
      finalThrottle = throttle; 
    }
  }


  m1_offset = constrain(m1_offset, 0, 1.5f);
  m2_offset = constrain(m2_offset, 0, 1.5f);
  m3_offset = constrain(m3_offset, 0, 1.5f);
  m4_offset = constrain(m4_offset, 0, 1.5f);

  m1_corr = (uint16_t)(finalThrottle * m1_offset - pCorr + rCorr - yCorr);
  m2_corr = (uint16_t)(finalThrottle * m2_offset - pCorr - rCorr + yCorr);
  m3_corr = (uint16_t)(finalThrottle * m3_offset + pCorr + rCorr + yCorr);
  m4_corr = (uint16_t)(finalThrottle * m4_offset + pCorr - rCorr - yCorr);

  if (lowThrottle) {
    m1 = m2 = m3 = m4 = 1000;
  } else {
    m1 = linearizeMotor(constrain(m1_corr, 1000, 2000));
    m2 = linearizeMotor(constrain(m2_corr, 1000, 2000));
    m3 = linearizeMotor(constrain(m3_corr, 1000, 2000));
    m4 = linearizeMotor(constrain(m4_corr, 1000, 2000));
  }

  ledcWrite(0, usToDuty(m1));
  ledcWrite(1, usToDuty(m2));
  ledcWrite(2, usToDuty(m3));
  ledcWrite(3, usToDuty(m4));
}