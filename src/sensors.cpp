#include "sensors.h"
#include "config.h"
#include "mahony.h"
#include "kalman.h"
#include "optical_flow.h"
#include "comms.h"
#include "driver/i2c.h"
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
float vx_est = 0, vy_est = 0;
// Lever arm: offset of IMU from drone CoM (meters)
const float rx = -0.005f, ry = 0.04f, rz = 0.01f;

volatile float vx = 0, vy = 0, vz = 0, vx_f = 0, vy_f = 0, vz_f = 0;
volatile float height_lidar = 0;
volatile bool newLidar = false;
volatile uint8_t  flow_quality = 0;
volatile float height_filtered = 0;

Kalman1D kx = {0, 100.0f, 50.0f, 15.0f};
Kalman1D ky = {0, 100.0f, 50.0f, 15.0f};



int16_t gyroX, gyroY, gyroZ, gyro2X, gyro2Y, gyro2Z;
int16_t accX, accY, accZ, acc2X, acc2Y, acc2Z;
int16_t magX, magY, magZ;

BiquadState stGX, stGY, stGZ, stAX, stAY, stAZ;
BiquadState stMX, stMY, stMZ, stVX, stVY, stVZ, stH;

SemaphoreHandle_t magMutex = xSemaphoreCreateMutex();

// -------------------- Calibration values (from config.h) --------------------
float gyroOffset[3] = {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z};
static const float accOffset[3] = {ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z};
static const float accScale[3]  = {ACC_SCALE_X,  ACC_SCALE_Y,  ACC_SCALE_Z};
float gyro2Offset[3] = {GYRO2_OFFSET_X, GYRO2_OFFSET_Y, GYRO2_OFFSET_Z};
static const float acc2Offset[3] = {ACC2_OFFSET_X, ACC2_OFFSET_Y, ACC2_OFFSET_Z};
static const float acc2Scale[3]  = {ACC2_SCALE_X,  ACC2_SCALE_Y,  ACC2_SCALE_Z};
static const float magOffset[3] = {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z};
static const float softIron[3][3] = SOFT_IRON;


// -------------------- Helpers --------------------
void i2c_init() {
  i2c_config_t conf;
  conf.mode             = I2C_MODE_MASTER;
  conf.sda_io_num       = I2C_SDA;
  conf.scl_io_num       = I2C_SCL;
  conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_FREQ;
  conf.clk_flags        = 0;
  i2c_param_config(I2C_PORT, &conf);
  i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}


static void writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };
  i2c_master_write_to_device(I2C_PORT, addr, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static uint8_t readRegister(uint8_t addr, uint8_t reg) {
  uint8_t val = 0;
  i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
  return val;
}


// -------------------- Init --------------------
void initSensors() {
  i2c_init();
  delay(500);

  // MPU-6050
  writeRegister(MPU_ADDR, 0x6B, 0x00); delay(50);
  writeRegister(MPU_ADDR, 0x1B, 0x10);
  writeRegister(MPU_ADDR, 0x1C, 0x00);
  writeRegister(MPU_ADDR, 0x19, 0x00);
  writeRegister(MPU_ADDR, 0x1A, 0x03); delay(50);
  // Second MPU-6050
  writeRegister(MPU2_ADDR, 0x6B, 0x00); delay(50);
  writeRegister(MPU2_ADDR, 0x1B, 0x10);
  writeRegister(MPU2_ADDR, 0x1C, 0x00);
  writeRegister(MPU2_ADDR, 0x19, 0x00);
  writeRegister(MPU2_ADDR, 0x1A, 0x03); delay(50);
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
  readMPU2();
  readMag();
  applyMPUCalibration1(gyroX, gyroY, gyroZ, accX, accY, accZ);
  applyMPUCalibration2(gyro2X, gyro2Y, gyro2Z, acc2X, acc2Y, acc2Z);
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
    uint8_t reg = 0x3B;
    uint8_t buf[14];

    i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, buf, 14, pdMS_TO_TICKS(5));

    accX  = (int16_t)(buf[0]  << 8 | buf[1]);
    accY  = (int16_t)(buf[2]  << 8 | buf[3]);
    accZ  = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6], buf[7] = temp (skip)
    gyroX = (int16_t)(buf[8]  << 8 | buf[9]);
    gyroY = (int16_t)(buf[10] << 8 | buf[11]);
    gyroZ = (int16_t)(buf[12] << 8 | buf[13]);
}


void readMPU2() {
    uint8_t reg = 0x3B;
    uint8_t buf[14];

    i2c_master_write_read_device(I2C_PORT, MPU2_ADDR, &reg, 1, buf, 14, pdMS_TO_TICKS(5));

    acc2X  = (int16_t)(buf[0]  << 8 | buf[1]);
    acc2Y  = (int16_t)(buf[2]  << 8 | buf[3]);
    acc2Z  = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6], buf[7] = temp (skip)
    gyro2X = (int16_t)(buf[8]  << 8 | buf[9]);
    gyro2Y = (int16_t)(buf[10] << 8 | buf[11]);
    gyro2Z = (int16_t)(buf[12] << 8 | buf[13]);
}

void readMag() {
  uint8_t status = readRegister(MAG_ADDR, 0x09);
  if (!(status & 0x01)) return;

  uint8_t reg = 0x01, buf[6];
  i2c_master_write_read_device(I2C_PORT, MAG_ADDR, &reg, 1, buf, 6, I2C_TIMEOUT_MS);

  int16_t nx = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t ny = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t nz = (int16_t)(buf[5] << 8 | buf[4]);

  xSemaphoreTake(magMutex, portMAX_DELAY);
  magX = nx; magY = ny; magZ = nz;
  xSemaphoreGive(magMutex);
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

void applyMPUCalibration1(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az) {
  gx -= (int16_t)gyroOffset[0];
  gy -= (int16_t)gyroOffset[1];
  gz -= (int16_t)gyroOffset[2];
  ax  = (int16_t)((ax - accOffset[0]) * accScale[0]);
  ay  = (int16_t)((ay - accOffset[1]) * accScale[1]);
  az  = (int16_t)((az - accOffset[2]) * accScale[2]);
}


void applyMPUCalibration2(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az) {
  gx -= (int16_t)gyro2Offset[0];
  gy -= (int16_t)gyro2Offset[1];
  gz -= (int16_t)gyro2Offset[2];
  ax  = (int16_t)((ax - acc2Offset[0]) * acc2Scale[0]);
  ay  = (int16_t)((ay - acc2Offset[1]) * acc2Scale[1]);
  az  = (int16_t)((az - acc2Offset[2]) * acc2Scale[2]);
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
  static uint64_t lastLidarFlowUpdate = esp_timer_get_time();
  static float gyroAccumX = 0.0f, gyroAccumY = 0.0f, gyroAccumZ = 0.0f;
  static int   gyroAccumCount = 0;
  static bool  readFirstIMU = true;
  static int16_t lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
  static int16_t lastAccX  = 0, lastAccY  = 0, lastAccZ  = 0;

  if (readFirstIMU) {
    readMPU();
    applyMPUCalibration1(gyroX, gyroY, gyroZ, accX, accY, accZ);
    // Average fresh MPU1 with last MPU2 reading
    // (MPU2 data is 500µs stale — negligible)
  } else {
    readMPU2();
    applyMPUCalibration2(gyro2X, gyro2Y, gyro2Z, acc2X, acc2Y, acc2Z);
    // Mirror into gyroX/accX namespace for uniform processing below
    gyroX = gyro2X; gyroY = gyro2Y; gyroZ = gyro2Z;
    accX  = acc2X;  accY  = acc2Y;  accZ  = acc2Z;
  }
  readFirstIMU = !readFirstIMU;

  uint64_t now = esp_timer_get_time();
  float dt = (float)(now - filterLastMicros) * 1e-6f;
  filterLastMicros = now;
  if (dt <= 0.00005f || dt > 0.05f) return;

  BiquadCoeffs gCoeffs, aCoeffs, mCoeffs, vCoeffs, hCoeffs;
  computeCoeffs(gCoeffs, GYRO_LPF_CUTOFF, dt);
  computeCoeffs(aCoeffs, ACC_LPF_CUTOFF, dt);

  // Average current fresh read with last cycle's other IMU
  float gx = (gyroX / 32.8f + lastGyroX / 32.8f) / 2.0f;
  float gy = (gyroY / 32.8f + lastGyroY / 32.8f) / 2.0f;
  float gz = (gyroZ / 32.8f + lastGyroZ / 32.8f) / 2.0f;

  float ax = (accX / 16384.0f + lastAccX / 16384.0f) / 2.0f;
  float ay = (accY / 16384.0f + lastAccY / 16384.0f) / 2.0f;
  float az = (accZ / 16384.0f + lastAccZ / 16384.0f) / 2.0f;

  // Store this cycle's reading for next cycle's average
  lastGyroX = gyroX; lastGyroY = gyroY; lastGyroZ = gyroZ;
  lastAccX  = accX;  lastAccY  = accY;  lastAccZ  = accZ;

  gx = applyFilter(stGX, gCoeffs, gx);
  gy = applyFilter(stGY, gCoeffs, gy);
  gz = applyFilter(stGZ, gCoeffs, gz);

  if (fabsf(gx) < GYRO_DEADBAND)   gx = 0.0f;
  if (fabsf(gy) < GYRO_DEADBAND)   gy = 0.0f;
  if (fabsf(gz) < GYRO_DEADBAND_Z) gz = 0.0f;

  gx_f = gx; gy_f = gy; gz_f = gz;

  ax_f = applyFilter(stAX, aCoeffs, ax);
  ay_f = applyFilter(stAY, aCoeffs, ay);
  az_f = applyFilter(stAZ, aCoeffs, az);

  //-----------MAHONY UPDATE-----------------
  mahonyUpdate(gx_f, gy_f, gz_f, ax_f, ay_f, az_f, dt);
  quaternionToEuler(roll_ag, pitch_ag, yaw_ag);

  getLinearAcceleration(ax_f, ay_f, az_f, ax_world, ay_world);
  if (fabsf(ax_world ) < 10.0f) {ax_world = 0;}
  if (fabsf(ay_world ) < 10.0f) {ay_world = 0;}
  vx_est += -ax_world * dt; vy_est += ay_world * dt;

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
  
  // Average gyro after 10 loops for velocity compensation
  gyroAccumX += gx_f;
  gyroAccumY += gy_f;
  gyroAccumZ += gz_f;
  gyroAccumCount++;


  if (newLidar){
    uint64_t now = esp_timer_get_time();
    float dt_lidar = (now - lastLidarFlowUpdate) * 1e-6f;
    lastLidarFlowUpdate = now;

    float qualityScale = constrain(flow_quality / 500.0f, 0.001f, 1.0f);
    float dynamicLPF = FLOW_LPF_CUTOFF * qualityScale;

    computeCoeffs(vCoeffs, dynamicLPF, dt_lidar);
    computeCoeffs(hCoeffs, HEIGHT_LPF, dt_lidar);
    height_lidar = applyFilter(stH, hCoeffs, height_lidar);
    height_filtered += constrain((height_lidar - height_filtered), -0.005f, 0.005f);
    vx = applyFilter(stVX, vCoeffs, vx);
    vy = applyFilter(stVY, vCoeffs, vy);
    float avgGX = (gyroAccumCount > 0) ? gyroAccumX / gyroAccumCount : gx_f;
    float avgGY = (gyroAccumCount > 0) ? gyroAccumY / gyroAccumCount : gy_f;
    float avgGZ = (gyroAccumCount > 0) ? gyroAccumZ / gyroAccumCount : gz_f;

    gyroAccumX = gyroAccumY = gyroAccumZ = 0.0f;
    gyroAccumCount = 0;
    vx -= (avgGY) * DEG_TO_RAD * height_lidar * 100.0f;
    vy -= (avgGX) * DEG_TO_RAD * height_lidar * 100.0f;
    //vx -= -(avgGZ) * DEG_TO_RAD * FLOW_OFFSET_Y * 100.0f;
    //vy -=  (avgGZ) * DEG_TO_RAD * FLOW_OFFSET_X * 100.0f;
    float cp = fabsf(cosf(pitch_ag * DEG_TO_RAD));
    float cr = fabsf(cosf(roll_ag  * DEG_TO_RAD));
    if (cp > 0.3f) vx /= cp;
    if (cr > 0.3f) vy /= cr;
    if (fabsf(vx) < 0.1f/qualityScale) {vx = 0;}
    if (fabsf(vy) < 0.1f/qualityScale) {vy = 0;}
    vx_f += constrain((vx - vx_f), -VEL_SPIKE_THRESHOLD, VEL_SPIKE_THRESHOLD);
    vy_f += constrain((vy - vy_f), -VEL_SPIKE_THRESHOLD, VEL_SPIKE_THRESHOLD);

    const float TAU_FLOW = 0.05f;
    float fuse_factor = constrain(dt_lidar / (dt_lidar + TAU_FLOW), 0.01f, 0.8f);

    vx_est += fuse_factor * (vx_f - vx_est);
    vy_est += fuse_factor * (vy_f - vy_est);

    newLidar = false;
  }

}