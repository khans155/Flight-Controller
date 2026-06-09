#include "sensors.h"
#include "config.h"
#include "mahony.h"
#include "kalman.h"
#include "optical_flow.h"
#include "baro.h"
#include "comms.h"
#include "driver/i2c.h"
#include <SPI.h>
#include <math.h>


// -------------------- Shared sensor state --------------------
float DRAM_ATTR gx_f = 0, gy_f = 0, gz_f = 0;
float DRAM_ATTR ax_f = 0, ay_f = 0, az_f = 0;
float DRAM_ATTR mx_f = 0, my_f = 0, mz_f = 0;
float DRAM_ATTR roll_ag = 0, pitch_ag = 0, yaw_ag = 0;
float DRAM_ATTR roll_kal = 0, pitch_kal = 0;
float DRAM_ATTR yaw_mag = 0;
float DRAM_ATTR yaw_error = 0;
float DRAM_ATTR ax_world = 0, ay_world = 0;
float DRAM_ATTR vx_est = 0, vy_est = 0;
// Lever arm: offset of IMU from drone CoM (meters)
const float rx = -0.005f, ry = 0.04f, rz = 0.01f;

volatile float DRAM_ATTR vx = 0, vy = 0, vz = 0, vx_f = 0, vy_f = 0, vz_f = 0;
volatile float DRAM_ATTR height_lidar = 0;
volatile float DRAM_ATTR height_baro = 0.0f;
volatile bool DRAM_ATTR newLidar = false;
volatile bool DRAM_ATTR baroFlag = false;
volatile uint8_t DRAM_ATTR  flow_quality = 0;
volatile float DRAM_ATTR height_filtered = 0;
volatile float DRAM_ATTR gyro_lpf = GYRO_LPF_CUTOFF;
volatile float DRAM_ATTR gyro_notch = GYRO_NOTCH;

Kalman1D DRAM_ATTR kx = {0, 100.0f, 50.0f, 15.0f};
Kalman1D DRAM_ATTR ky = {0, 100.0f, 50.0f, 15.0f};



int16_t DRAM_ATTR gyroX, gyroY, gyroZ;
int16_t DRAM_ATTR accX, accY, accZ;
int16_t DRAM_ATTR magX, magY, magZ;


//LPF 2nd Order Butterworth
BiquadState DRAM_ATTR stGX, stGY, stGZ, stAX, stAY, stAZ;
BiquadState DRAM_ATTR stMX, stMY, stMZ, stVX, stVY, stVZ, stH;
BiquadCoeffs DRAM_ATTR gCoeffs, aCoeffs, mCoeffs, vCoeffs, hCoeffs;

//Notch Filter
BiquadState DRAM_ATTR notchStGX, notchStGY, notchStGZ;
BiquadCoeffs DRAM_ATTR notchCoeffs;

// -------------------- Calibration values (from config.h) --------------------
float DRAM_ATTR gyroOffset[3] = {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z};
static const float DRAM_ATTR accOffset[3]  = {ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z};
static const float DRAM_ATTR accScale[3]   = {ACC_SCALE_X,  ACC_SCALE_Y,  ACC_SCALE_Z};
static const float DRAM_ATTR magOffset[3]  = {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z};
static const float DRAM_ATTR softIron[3][3] = SOFT_IRON;


// -------------------- Helpers --------------------
void i2c_ext_init() {
  i2c_config_t conf1;
  conf1.mode             = I2C_MODE_MASTER;
  conf1.sda_io_num       = I2C_EXT_SDA; 
  conf1.scl_io_num       = I2C_EXT_SCL;
  conf1.sda_pullup_en    = GPIO_PULLUP_ENABLE;
  conf1.scl_pullup_en    = GPIO_PULLUP_ENABLE;
  conf1.master.clk_speed = I2C_EXT_FREQ;
  conf1.clk_flags        = 0;
  i2c_param_config(I2C_EXT_BUS, &conf1);
  i2c_driver_install(I2C_EXT_BUS, I2C_MODE_MASTER, 0, 0, 0);
}

void writeRegister(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val){
  uint8_t buf[2] = { reg, val };
  i2c_master_write_to_device(port, addr, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

uint8_t readRegister(i2c_port_t port, uint8_t addr, uint8_t reg){
  uint8_t val = 0;
  i2c_master_write_read_device(port, addr, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
  return val;
}

void IRAM_ATTR spiWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CSN_MPU, LOW);

  SPI.transfer(reg & 0x7F); // MSB = 0 for write
  SPI.transfer(val);
  digitalWrite(PIN_CSN_MPU, HIGH);
  SPI.endTransaction();
}

void IRAM_ATTR spiRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  // Can read safely at 10MHz
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CSN_MPU, LOW);

  SPI.transfer(reg | 0x80); // MSB = 1 for read
  for(uint8_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CSN_MPU, HIGH);
  SPI.endTransaction();
}

// --------------------Biquad/Notch Filters--------------------

void IRAM_ATTR computeCoeffs(BiquadCoeffs &c, float cutoff, float dt) {
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

void IRAM_ATTR computeNotchCoeffs(BiquadCoeffs &c, float notchFreq, float Q, float dt) {
    float w0    = 2.0f * M_PI * notchFreq * dt;   
    float cosW0 = cosf(w0);
    float alpha = sinf(w0) / (2.0f * Q);           
    float norm  = 1.0f / (1.0f + alpha);          

    c.b0 =  1.0f * norm;
    c.b1 = -2.0f * cosW0 * norm;
    c.b2 =  1.0f * norm;
    c.a1 = -2.0f * cosW0 * norm;                  
    c.a2 =  (1.0f - alpha) * norm;
}

float IRAM_ATTR applyFilter(BiquadState &s, BiquadCoeffs &c, float input) {
    float output = c.b0 * input + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
    
    s.x2 = s.x1;
    s.x1 = input;
    s.y2 = s.y1;
    s.y1 = output;
    
    return output;
}

// -------------------- Init --------------------
void initSensors() {
  i2c_ext_init();

  // SPI Setup
  pinMode(PIN_CSN_MPU, OUTPUT);
  digitalWrite(PIN_CSN_MPU, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  SPI.setFrequency(20000000);
  delay(50);

  spiWrite(MPU_REG_PWR_MGMT_1, 0x80); // Reset
  delay(70);
  spiWrite(MPU_REG_PWR_MGMT_1, 0x01); // Auto select best clock
  delay(20);
  spiWrite(MPU_REG_USER_CTRL, 0x10);  // Disable I2C interface 
  delay(20);
  uint8_t whoami;
  spiRead(0x75, &whoami, 1);
  Serial.printf("WHO_AM_I = 0x%02X\n", whoami);
  
  //spiWrite(0x19, 0x03);
  spiWrite(MPU_REG_GYRO_CONFIG, 0x10); // Gyro: ±1000 dps
  spiWrite(MPU_REG_ACCEL_CONFIG, 0x00); // Accel: ±2g

  spiWrite(MPU_REG_CONFIG, 0x02);       // Gyro DLPF 
  spiWrite(MPU_REG_ACCEL_CONFIG2, 0x03);// Accel DLPF 
  spiWrite(0x19, 0x00); 
  delay(20);
  
  // Magnetometer
  readRegister(I2C_EXT_BUS, MAG_ADDR, 0x00); // read ID (discard)
  writeRegister(I2C_EXT_BUS, MAG_ADDR, 0x0B, 0x80); delay(50);
  uint8_t ctrl2 = (QMC5883P_RANGE_2G << 2) | QMC5883P_SETRESET_ON;
  //writeRegister(MAG_ADDR, 0x29, 0x06);
  writeRegister(I2C_EXT_BUS, MAG_ADDR, 0x0B, ctrl2);
  uint8_t ctrl1 = (QMC5883P_OSR_1 << 4) | (QMC5883P_DSR_8 << 6) | (QMC5883P_ODR_200HZ << 2) | QMC5883P_MODE_CONTINUOUS;
  writeRegister(I2C_EXT_BUS, MAG_ADDR, 0x0A, ctrl1);

  initBMP280();
  calibrateBMP280Ground();

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
  roll  = atan2f(ay, az);
  pitch = -atan2f(ax, sqrt(ay*ay + az*az));

  float Xh = mx_f * cosf(roll) - mz_f * sinf(roll);
  float Yh = mx_f * sinf(pitch) * sinf(roll) + my_f * cosf(pitch) - mz_f * sinf(pitch) * cosf(roll);
  yaw_init = atan2f(-Yh, Xh) * (180.0 / PI);
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

  computeCoeffs(gCoeffs, gyro_lpf, IMU_PERIOD/1e6f);
  computeCoeffs(aCoeffs, ACC_LPF_CUTOFF, IMU_PERIOD/1e6f);
  computeNotchCoeffs(notchCoeffs, gyro_notch, 5.0f, IMU_PERIOD/1e6f);
}  

// -------------------- Read --------------------
void IRAM_ATTR readMPU() {
    uint8_t buf[14];
    spiRead(MPU_REG_ACCEL_XOUT_H, buf, 14);

    accX  = (int16_t)(buf[0]  << 8 | buf[1]);
    accY  = (int16_t)(buf[2]  << 8 | buf[3]);
    accZ  = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6], buf[7] = temp (skip)
    gyroX = (int16_t)(buf[8]  << 8 | buf[9]);
    gyroY = (int16_t)(buf[10] << 8 | buf[11]);
    gyroZ = (int16_t)(buf[12] << 8 | buf[13]);
}

void readMag() {
  uint8_t status = readRegister(I2C_EXT_BUS, MAG_ADDR, 0x09);
  if (!(status & 0x01)) return;

  uint8_t reg = 0x01, buf[6];
  i2c_master_write_read_device(I2C_EXT_BUS, MAG_ADDR, &reg, 1, buf, 6, I2C_TIMEOUT_MS);

  int16_t nx = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t ny = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t nz = (int16_t)(buf[5] << 8 | buf[4]);

  magX = nx; magY = ny; magZ = nz;
}

// -------------------- Calibration --------------------
void calibrateMPU() {
  const int samples = 2000;
  float gxSum = 0, gySum = 0, gzSum = 0;

  Serial.println("Calibrating gyro... Keep still.");
  for (int i = 0; i < samples; i++) {
    readMPU();
    gxSum += gyroX; gySum += gyroY; gzSum += gyroZ;
    delay(2);
  }
  gyroOffset[0] = gxSum / samples;
  gyroOffset[1] = gySum / samples;
  gyroOffset[2] = gzSum / samples;
}

void IRAM_ATTR applyMPUCalibration(int16_t &gx, int16_t &gy, int16_t &gz,
                         int16_t &ax, int16_t &ay, int16_t &az) {
  gx -= (int16_t)gyroOffset[0];
  gy -= (int16_t)gyroOffset[1];
  gz -= (int16_t)gyroOffset[2];
  ax  = (int16_t)((ax - accOffset[0]) * accScale[0]);
  ay  = (int16_t)((ay - accOffset[1]) * accScale[1]);
  az  = (int16_t)((az - accOffset[2]) * accScale[2]);
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

void IRAM_ATTR updateFilter(uint64_t now) {
  static uint64_t DRAM_ATTR filterLastMicros = 0;
  static uint64_t DRAM_ATTR lastMagUpdate    = 0;
  static uint64_t DRAM_ATTR lastLidarFlowUpdate = 0;
  static float DRAM_ATTR gyroAccumX = 0.0f, gyroAccumY = 0.0f, gyroAccumZ = 0.0f;
  static int DRAM_ATTR   gyroAccumCount = 0;

  readMPU(); 
  applyMPUCalibration(gyroX, gyroY, gyroZ, accX, accY, accZ);
  //remotePrint("gyroX: %d, gyro2X: %d, gyroY: %d, gyro2Y: %d, gyroZ: %d, gyro2Z: %d \r\n", gyroX, gyro2X, gyroY, gyro2Y, gyroZ, gyro2Z);

  float dt = (float)(now - filterLastMicros) * 1e-6f;
  if (dt <= 0.00005f || dt > 0.05f) {
    filterLastMicros = now;  
    return;
  }
  filterLastMicros = now;

  static float DRAM_ATTR last_dt = 0.0f; // Recompute coeffs if loop period changes significantly i.g > 100us
  if (fabsf(dt - last_dt) > 0.0001f) {
    computeCoeffs(gCoeffs, gyro_lpf, dt);
    computeCoeffs(aCoeffs, ACC_LPF_CUTOFF, dt);
    computeNotchCoeffs(notchCoeffs, gyro_notch, 5.0f, dt);
    last_dt = dt; 
  }

  float gx = gyroX / 32.8f;
  float gy = gyroY / 32.8f;
  float gz = gyroZ / 32.8f;

  float ax = accX / 16384.0f;
  float ay = accY / 16384.0f;
  float az = accZ / 16384.0f;

  gx = applyFilter(stGX, gCoeffs, gx); 
  gx = applyFilter(notchStGX, notchCoeffs, gx);
  gy = applyFilter(stGY, gCoeffs, gy); 
  gy = applyFilter(notchStGY, notchCoeffs, gy);
  gz = applyFilter(stGZ, gCoeffs, gz); 
  gz = applyFilter(notchStGZ, notchCoeffs, gz);

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
  if (fabsf(ax_world ) < 3.0f) {ax_world = 0;}
  if (fabsf(ay_world ) < 3.0f) {ay_world = 0;}
  vx_est += -ax_world * dt; vy_est += ay_world * dt;

  //----------Magnetometer------------------- MOVE TO IO TASK!!!
  //float dt_mag = (now - lastMagUpdate) * 1e-6f;
  //if (dt_mag >= (1.0f/MAG_SAMPLE_RATE) ){
  //  computeCoeffs(mCoeffs, MAG_LPF_CUTOFF, dt_mag);
  //  lastMagUpdate = esp_timer_get_time();

    //readMag();
  //  float mx = magX, my = magY, mz = magZ;
  //  applyMagCalibration(mx, my, mz);

  //  mx_f = applyFilter(stMX, mCoeffs, mx);
  // my_f = applyFilter(stMY, mCoeffs, my);
  //  mz_f = applyFilter(stMZ, mCoeffs, mz);

  //  float Xh = mx_f * cosf(roll_ag*DEG_TO_RAD) - mz_f * sinf(roll_ag*DEG_TO_RAD);
  //  float Yh = mx_f * sinf(pitch_ag*DEG_TO_RAD) * sinf(roll_ag*DEG_TO_RAD) + my_f * cosf(pitch_ag*DEG_TO_RAD) - mz_f * sinf(pitch_ag*DEG_TO_RAD) * cosf(roll_ag*DEG_TO_RAD);
  //  yaw_mag = atan2f(-Yh, Xh) * (180.0f / PI);
  //  if (yaw_mag < 0.0f) yaw_mag += 360.0f;
  //}
  
  // Average gyro after 10 loops for velocity compensation
  gyroAccumX += gx_f;
  gyroAccumY += gy_f;
  gyroAccumZ += gz_f;
  gyroAccumCount++;


  if (newLidar){
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

    if (baroFlag) {
      baroFlag = false;
      //readBMP280();
    }

    newLidar = false;
  }

}