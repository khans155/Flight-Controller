#pragma once

// -------------------- WiFi --------------------
#define WIFI_SSID     ".."
#define WIFI_PASSWORD ".."

// -------------------- Pins --------------------
// I2C
#define I2C_IMU_SDA  21
#define I2C_IMU_SCL  22
#define I2C_EXT_SDA 4
#define I2C_EXT_SCL 0
// PWM
#define MOTOR1_PIN 25
#define MOTOR2_PIN 26
#define MOTOR3_PIN 27
#define MOTOR4_PIN 14
// UART
#define RX_MTF 16
#define TX_MTF 17
#define RX_GPS 32
#define TX_GPS 33
// SPI
#define PIN_CE    2
#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19

// -------------------- PWM --------------------
#define PWM_FREQ       400
#define PWM_RESOLUTION 16
#define PWM_MAX        65535
#define PWM_PERIOD_US  2500

// -------------------- I2C --------------------
#define I2C_IMU_BUS I2C_NUM_0 
#define I2C_EXT_BUS I2C_NUM_1
#define I2C_IMU_FREQ 1000000
#define I2C_EXT_FREQ 100000
#define I2C_TIMEOUT_MS pdMS_TO_TICKS(5)

#define MPU_ADDR 0x68
#define MPU2_ADDR 0x69
#define MAG_ADDR 0x2C
#define BMP280_ADDR 0x76

// -------------------- MAG CONFIG --------------------
#define  QMC5883P_MODE_NORMAL  0x01    ///< Normal mode
#define  QMC5883P_MODE_CONTINUOUS  0x03 ///< Continuous mode
#define  QMC5883P_ODR_10HZ  0x00  ///< 10 Hz output data rate
#define  QMC5883P_ODR_50HZ  0x01  ///< 50 Hz output data rate
#define  QMC5883P_ODR_100HZ  0x02 ///< 100 Hz output data rate
#define  QMC5883P_ODR_200HZ  0x03  ///< 200 Hz output data rate
#define  QMC5883P_OSR_8  0x00 ///< Over sample ratio = 8
#define  QMC5883P_OSR_4  0x01 ///< Over sample ratio = 4
#define  QMC5883P_OSR_2  0x02 ///< Over sample ratio = 2
#define  QMC5883P_OSR_1  0x03  ///< Over sample ratio = 1
#define  QMC5883P_DSR_1  0x00 ///< Downsample ratio = 1
#define  QMC5883P_DSR_2  0x01 ///< Downsample ratio = 2
#define  QMC5883P_DSR_4  0x02 ///< Downsample ratio = 4
#define  QMC5883P_DSR_8  0x03  ///< Downsample ratio = 8
#define  QMC5883P_RANGE_30G  0x00 ///< ±30 Gauss range
#define  QMC5883P_RANGE_12G  0x01 ///< ±12 Gauss range
#define  QMC5883P_RANGE_8G  0x02  ///< ±8 Gauss range
#define  QMC5883P_RANGE_2G  0x03   ///< ±2 Gauss range
#define  QMC5883P_SETRESET_ON  0x00      ///< Set and reset on
#define  QMC5883P_SETRESET_SETONLY  0x01 ///< Set only on
#define  QMC5883P_SETRESET_OFF  0x02      ///< Set and reset off

// -------------------- Controller Buttons -------------------
#define BTN_R1        (1 << 15)
#define BTN_L1        (1 << 14)
#define BTN_CROSS     (1 << 13)
#define BTN_CIRCLE    (1 << 12)
#define BTN_TRIANGLE  (1 << 11)
#define BTN_SQUARE    (1 << 10)
#define BTN_L3        (1 <<  9)
#define BTN_R3        (1 <<  8)
#define BTN_OPTIONS   (1 <<  7)
#define BTN_SHARE     (1 <<  6)
#define BTN_PS        (1 <<  5)
#define BTN_TOUCHPAD  (1 <<  4)
#define BTN_LEFT      (1 <<  3)
#define BTN_RIGHT     (1 <<  2)
#define BTN_UP        (1 <<  1)
#define BTN_DOWN      (1 <<  0)

// -------------------- Remote Tune Bitmap -------------------
#define TUNE_RATE_KP    0  
#define TUNE_RATE_KI    1
#define TUNE_RATE_KD    2
#define TUNE_YAW_KP     3   
#define TUNE_YAW_KI     4
#define TUNE_YAW_KD     5
#define TUNE_ANGLE_KP   6   
#define TUNE_ANGLE_KI   7
#define TUNE_ANGLE_KD   8
#define TUNE_VEL_KP     9   
#define TUNE_VEL_KI    10
#define TUNE_VEL_KD    11
#define TUNE_ALT_KP    12   
#define TUNE_ALT_KI    13
#define TUNE_ALT_KD    14
#define TUNE_PARAM_COUNT 15


// -------------------- Sensor Stuff --------------------
#define ACC_LPF_CUTOFF  5.0f
#define GYRO_LPF_CUTOFF 30.0f
#define GYRO_NOTCH 100.0f
#define MAG_LPF_CUTOFF   0.4f
#define FLOW_LPF_CUTOFF   20.0f
#define HEIGHT_LPF 15.0f
#define GYRO_DEADBAND  0.5f
#define GYRO_DEADBAND_Z 0.50f
#define VEL_SPIKE_THRESHOLD 0.3f
#define FLOW_OFFSET_X  0.0f   // +forward
#define FLOW_OFFSET_Y  0.10f 
#define FLOW_QUALITY_MIN 30

#define MAG_SAMPLE_RATE 20.0f  //Hz
#define LIDAR_FLOW_RATE 100.0f //Hz

// -------------------- Motor Offsets --------------------
// Scale factors applied to throttle per motor (1.0 = no correction)
#define M1_OFFSET_DEFAULT 1.0f
#define M2_OFFSET_DEFAULT 1.0f
#define M3_OFFSET_DEFAULT 1.0f
#define M4_OFFSET_DEFAULT 1.0f

// -------------------- Calibration --------------------
#define ACC_OFFSET_X   1031.00f
#define ACC_OFFSET_Y  -865.50f
#define ACC_OFFSET_Z  -511.00f

#define ACC2_OFFSET_X   1081.50f
#define ACC2_OFFSET_Y  -409.50f
#define ACC2_OFFSET_Z  -1073.50f

#define GYRO_OFFSET_X  -35.1912f
#define GYRO_OFFSET_Y   44.4739f
#define GYRO_OFFSET_Z  -29.1541f

#define GYRO2_OFFSET_X  -37.7105f
#define GYRO2_OFFSET_Y   2.7794f
#define GYRO2_OFFSET_Z  -28.6941f

#define ACC_SCALE_X  0.9960f
#define ACC_SCALE_Y  0.9934f
#define ACC_SCALE_Z  0.9786f

#define ACC2_SCALE_X  0.9929f
#define ACC2_SCALE_Y  0.9967f
#define ACC2_SCALE_Z  0.9861f

#define MAG_OFFSET_X  -260.10f
#define MAG_OFFSET_Y   -470.07f
#define MAG_OFFSET_Z   100.90f

#define ROLL_OFFSET 1.6f
#define PITCH_OFFSET 0.0f

#define SOFT_IRON { \
  { 0.939f, 0.006f,  0.038f }, \
  { 0.006f, 0.967f,  0.004f }, \
  { 0.038f, 0.004f,  1.103f }  \
}

// -------------------- PID Gains --------------------
// Velocity loop
#define VEL_KP   0.07f
#define VEL_KI   0.001f
#define VEL_KD   0.0005f
#define VEL_ANGLE_LIM  1.0f


// Angle loop
#define ROLL_ANGLE_KP   8.0f
#define ROLL_ANGLE_KI   0.001f
#define ROLL_ANGLE_KD   0.000f
#define ROLL_ANGLE_LIM  170.0f

#define PITCH_ANGLE_KP  8.0f
#define PITCH_ANGLE_KI  0.001f
#define PITCH_ANGLE_KD  0.000f
#define PITCH_ANGLE_LIM 170.0f

// Rate loop
#define ROLL_RATE_KP   0.55f
#define ROLL_RATE_KI   0.05f
#define ROLL_RATE_KD   0.02f
#define ROLL_RATE_LIM  210.0f

#define PITCH_RATE_KP  0.55f
#define PITCH_RATE_KI  0.05f
#define PITCH_RATE_KD  0.02f
#define PITCH_RATE_LIM 210.0f

#define YAW_RATE_KP    1.10f
#define YAW_RATE_KI    0.30f
#define YAW_RATE_KD    0.02f
#define YAW_RATE_LIM   250.0f

// Altitude loop
#define ALT_KP  5.5f
#define ALT_KI   1.05f
#define ALT_KD   0.1f
#define ALT_RATE_LIMIT  350.0f

// Derivative filter alpha (higher = less filtering)
#define PID_D_ALPHA_VEL 0.05f
#define PID_D_ALPHA_ANGLE 0.05f
#define PID_D_ALPHA_RATE  0.1f
#define PID_D_ALPHA_ALT 0.02f

// -------------------- Mahony Filter --------------------
#define MAHONY_KP  1.5f
#define MAHONY_KI  0.001f

// -------------------- OTHER --------------------
#define ANGLE_SCALE     0.035f   // rollOffset -> desired angle (deg)
#define YAW_RATE_SCALE  0.25f    // yawOffset  -> desired yaw rate (deg/s)
#define LOW_THROTTLE_THRESHOLD 1090
#define SPEED_SOUND 0.000343f
#define MAX_ALTITUDE 450.0f
#define HEIGHT_SPIKE_LIMIT 5.0f
#define THROTTLE_LOCK_SCALE  0.5f
#define HIGH_GYRO_THRESH_DPS 100.0f
