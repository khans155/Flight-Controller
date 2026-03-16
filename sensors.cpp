#include "sensors.h"
#include "config.h"
#include "mahony.h"
#include "kalman.h"
#include "optical_flow.h"
#include "comms.h"
#include <Wire.h>
#include <math.h>

// -------------------- Shared sensor state --------------------
float gx_f = 0, gy_f = 0, gz_f = 0;
float ax_f = 0, ay_f = 0, az_f = 0;
float mx_f = 0, my_f = 0, mz_f = 0;
float roll_ag = 0, pitch_ag = 0, yaw_ag = 0;
float roll_kal = 0, pitch_kal = 0;
float yaw_mag = 0;
float yaw_error = 0;
float ax_world = 0, ay_world = 0;


volatile float vx = 0, vy = 0, vz = 0, vx_f = 0, vy_f = 0, vz_f = 0;
volatile float height_lidar = 0;
volatile bool newLidar = false;
volatile uint8_t  flow_quality = 0;

Kalman1D kx = {0, 100.0f, 50.0f, 15.0f};
Kalman1D ky = {0, 100.0f, 50.0f, 15.0f};


int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

BiquadState stGX, stGY, stGZ, stAX, stAY, stAZ;
BiquadState stMX, stMY, stMZ, stVX, stVY, stVZ;

SemaphoreHandle_t magMutex = xSemaphoreCreateMutex();

// -------------------- Calibration values (from config.h) --------------------
float gyroOffset[3] = {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z};
static const float accOffset[3] = {ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z};
static const float accScale[3]  = {ACC_SCALE_X,  ACC_SCALE_Y,  ACC_SCALE_Z};
static const float magOffset[3] = {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z};
static const float softIron[3][3] = SOFT_IRON;


// -------------------- Helpers --------------------
static void writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}


// -------------------- Init --------------------
void initSensors() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(1000000);
  delay(500);

  // MPU-6050
  writeRegister(MPU_ADDR, 0x6B, 0x00); delay(50);
  writeRegister(MPU_ADDR, 0x1B, 0x10);
  writeRegister(MPU_ADDR, 0x1C, 0x00);
  writeRegister(MPU_ADDR, 0x19, 0x00);
  writeRegister(MPU_ADDR, 0x1A, 0x04); delay(50);

  // Magnetometer
  readRegister(MAG_ADDR, 0x00); // read ID (discard)
  writeRegister(MAG_ADDR, 0x0B, 0x80); delay(50);
  uint8_t ctrl2 = (QMC5883P_RANGE_2G << 2) | QMC5883P_SETRESET_ON;
  //writeRegister(MAG_ADDR, 0x29, 0x06);
  writeRegister(MAG_ADDR, 0x0B, ctrl2);
  uint8_t ctrl1 = (QMC5883P_OSR_1 << 4) | (QMC5883P_DSR_8 << 6) | (QMC5883P_ODR_200HZ << 2) | QMC5883P_MODE_CONTINUOUS;
  writeRegister(MAG_ADDR, 0x0A, ctrl1);


  //calibrateMPU();
}

void initFilter(){
  float roll, pitch, yaw, yaw_init;
  readMPU();
  readMag();
  applyMPUCalibration(gyroX, gyroY, gyroZ, accX, accY, accZ);
  float mx = magX, my = magY, mz = magZ;
  applyMagCalibration(mx, my, mz);
  float ax = accX / 16384.0f;
  float ay = accY / 16384.0f;
  float az = accZ / 16384.0f;
  roll  = atan2(ay, az);
  pitch = -atan2(ax, sqrt(ay*ay + az*az));

  float Xh = mx_f * cos(roll*DEG_TO_RAD) - mz_f * sin(roll*DEG_TO_RAD);
  float Yh = mx_f * sin(pitch*DEG_TO_RAD) * sin(roll*DEG_TO_RAD) + my_f * cos(pitch*DEG_TO_RAD) - mz_f * sin(pitch*DEG_TO_RAD) * cos(roll*DEG_TO_RAD);
  yaw_init = atan2(-Yh, Xh) * (180.0 / PI);
  if (yaw_ag < 0) yaw_ag += 360.0;

  yaw = yaw_init;
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);

  float q0 = cr*cp*cy + sr*sp*sy;
  float q1 = sr*cp*cy - cr*sp*sy;
  float q2 = cr*sp*cy + sr*cp*sy;
  float q3 = cr*cp*sy - sr*sp*cy;

  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  updatequaternion(q0, q1, q2, q3);
}   

// -------------------- Read --------------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accX  = Wire.read() << 8 | Wire.read();
  accY  = Wire.read() << 8 | Wire.read();
  accZ  = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void readMag() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x09);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, (uint8_t)1);
  byte status = Wire.read();

  if (status & 0x01) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(MAG_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      uint8_t xl = Wire.read(), xh = Wire.read();
      uint8_t yl = Wire.read(), yh = Wire.read();
      uint8_t zl = Wire.read(), zh = Wire.read();
      int16_t nx = (int16_t)((xh << 8) | xl);
      int16_t ny = (int16_t)((yh << 8) | yl);
      int16_t nz = (int16_t)((zh << 8) | zl);
      xSemaphoreTake(magMutex, portMAX_DELAY);
      magX = nx; magY = ny; magZ = nz;
      xSemaphoreGive(magMutex);
    }
  }
}


// -------------------- Calibration --------------------
void calibrateMPU() {
  const int samples = 5000;
  float gxSum = 0, gySum = 0, gzSum = 0;
  Serial.println("Calibrating gyro... Keep still.");
  for (int i = 0; i < samples; i++) {
    readMPU();
    gxSum += gyroX; gySum += gyroY; gzSum += gyroZ;
    delay(1);
  }
  gyroOffset[0] = gxSum / samples;
  gyroOffset[1] = gySum / samples;
  gyroOffset[2] = gzSum / samples;
}

void applyMPUCalibration(int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ,
                         int16_t &accX, int16_t &accY, int16_t &accZ) {
  gyroX -= (int16_t)gyroOffset[0];
  gyroY -= (int16_t)gyroOffset[1];
  gyroZ -= (int16_t)gyroOffset[2];
  accX = (int16_t)((accX - accOffset[0]) * accScale[0]);
  accY = (int16_t)((accY - accOffset[1]) * accScale[1]);
  accZ = (int16_t)((accZ - accOffset[2]) * accScale[2]);
}

void applyMagCalibration(float &x, float &y, float &z) {
  x -= magOffset[0];
  y -= magOffset[1];
  z -= magOffset[2];
  float cx = softIron[0][0]*x + softIron[0][1]*y + softIron[0][2]*z;
  float cy = softIron[1][0]*x + softIron[1][1]*y + softIron[1][2]*z;
  float cz = softIron[2][0]*x + softIron[2][1]*y + softIron[2][2]*z;
  x = cx; y = cy; z = cz;
}


// -------------------- Filter --------------------
void computeCoeffs(BiquadCoeffs &c, float cutoff, float dt) {
    // ff = cutoff / fs -> ff = cutoff * dt
    float tanVal = tanf(M_PI * cutoff * dt);
    float ita = 1.0f / tanVal;
    float q = sqrtf(2.0f); // Butterworth damping

    c.b0 = 1.0f / (1.0f + q * ita + ita * ita);
    c.b1 = 2.0f * c.b0;
    c.b2 = c.b0;
    c.a1 = 2.0f * c.b0 * (1.0f - ita * ita);
    c.a2 = c.b0 * (1.0f - q * ita + ita * ita);
}

float applyFilter(BiquadState &s, BiquadCoeffs &c, float input) {
    float output = c.b0 * input + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
    
    s.x2 = s.x1;
    s.x1 = input;
    s.y2 = s.y1;
    s.y1 = output;
    
    return output;
}

void updateFilter() {
  static uint64_t filterLastMicros = esp_timer_get_time();
  static uint64_t lastMagUpdate    = esp_timer_get_time();
  static uint64_t lastLidarFlowUpdate    = esp_timer_get_time();


  readMPU();
  applyMPUCalibration(gyroX, gyroY, gyroZ, accX, accY, accZ);

  uint64_t now = esp_timer_get_time();
  float dt = (float)(now - filterLastMicros) * 1e-6f;
  filterLastMicros = now;
  if (dt <= 0.0001f || dt > 0.05f) return;

  BiquadCoeffs gCoeffs, aCoeffs, mCoeffs, vCoeffs;
  computeCoeffs(gCoeffs, GYRO_LPF_CUTOFF, dt);
  computeCoeffs(aCoeffs, ACC_LPF_CUTOFF, dt);

  // Gyro
  float gx = gyroX / 32.8f;
  float gy = gyroY / 32.8f;
  float gz = gyroZ / 32.8f;

  gx = applyFilter(stGX, gCoeffs, gx);
  gy = applyFilter(stGY, gCoeffs, gy);
  gz = applyFilter(stGZ, gCoeffs, gz);

  if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
  if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;
  if (fabsf(gz) < GYRO_DEADBAND_Z) gz = 0.0f;

  gx_f = gx;
  gy_f = gy;
  gz_f = gz;

  // Accel
  float ax = accX / 16384.0f;
  float ay = accY / 16384.0f;
  float az = accZ / 16384.0f;

  ax_f = applyFilter(stAX, aCoeffs, ax);
  ay_f = applyFilter(stAY, aCoeffs, ay);
  az_f = applyFilter(stAZ, aCoeffs, az);

  //-----------MAHONY UPDATE-----------------
  mahonyUpdate(gx_f, gy_f, gz_f, ax_f, ay_f, az_f, dt);
  quaternionToEuler(roll_ag, pitch_ag, yaw_ag);

  //----------Magnetometer-------------------
  float dt_mag = (esp_timer_get_time() - lastMagUpdate) * 1e-6f;
  if (dt_mag >= (1.0f/MAG_SAMPLE_RATE) ){
    computeCoeffs(mCoeffs, MAG_LPF_CUTOFF, dt_mag);
    lastMagUpdate = esp_timer_get_time();

    readMag();
    float mx = magX, my = magY, mz = magZ;
    applyMagCalibration(mx, my, mz);

    mx_f = applyFilter(stMX, mCoeffs, mx);
    my_f = applyFilter(stMY, mCoeffs, my);
    mz_f = applyFilter(stMZ, mCoeffs, mz);

    float Xh = mx_f * cosf(roll_ag*DEG_TO_RAD) - mz_f * sinf(roll_ag*DEG_TO_RAD);
    float Yh = mx_f * sinf(pitch_ag*DEG_TO_RAD) * sinf(roll_ag*DEG_TO_RAD) + my_f * cosf(pitch_ag*DEG_TO_RAD) - mz_f * sinf(pitch_ag*DEG_TO_RAD) * cosf(roll_ag*DEG_TO_RAD);
    yaw_mag = atan2f(-Yh, Xh) * (180.0f / PI);
    if (yaw_mag < 0.0f) yaw_mag += 360.0f;
  }
  
  if (newLidar){
    uint64_t now = esp_timer_get_time();
    float dt_lidar = (now - lastLidarFlowUpdate) * 1e-6f;
    lastLidarFlowUpdate = now;
    float qualityScale = constrain(flow_quality / 200.0f, 0.1f, 1.0f);
    float dynamicLPF = FLOW_LPF_CUTOFF * qualityScale;
    computeCoeffs(vCoeffs, dynamicLPF, dt_lidar);
    vx -= (gyroY / 32.8f) * DEG_TO_RAD * height_lidar * 100.0f;
    vy -= (gyroX / 32.8f) * DEG_TO_RAD * height_lidar * 100.0f;

    vx -= -(gyroZ / 32.8f) * DEG_TO_RAD * FLOW_OFFSET_Y * 100.0f;
    vy -=  (gyroZ / 32.8f) * DEG_TO_RAD * FLOW_OFFSET_X * 100.0f;

    vx = vx * fabsf(cosf(pitch_ag*DEG_TO_RAD));
    vy = vy * fabsf(cosf(roll_ag*DEG_TO_RAD));

    vx = applyFilter(stVX, vCoeffs, vx);
    vy = applyFilter(stVY, vCoeffs, vy);

    vx_f += constrain((vx - vx_f), -VEL_SPIKE_THRESHOLD, VEL_SPIKE_THRESHOLD);
    vy_f += constrain((vy - vy_f), -VEL_SPIKE_THRESHOLD, VEL_SPIKE_THRESHOLD);




    if (fabsf(vx_f) < 0.2f/qualityScale) {vx_f = 0;}
    if (fabsf(vy_f) < 0.2f/qualityScale) {vy_f = 0;}

    newLidar = false;
  }
  

}