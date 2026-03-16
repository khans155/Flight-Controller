#pragma once

// -------------------- WiFi --------------------
#define WIFI_SSID     "ssid"
#define WIFI_PASSWORD "password"

// -------------------- Pins --------------------
// I2C
#define MPU_SDA 21
#define MPU_SCL 22
// PWM
#define MOTOR1_PIN 25
#define MOTOR2_PIN 26
#define MOTOR3_PIN 27
#define MOTOR4_PIN 14
// UART
#define RX_MTF 16
#define TX_MTF 17

// -------------------- PWM --------------------
#define PWM_FREQ       400
#define PWM_RESOLUTION 16
#define PWM_MAX        65535
#define PWM_PERIOD_US  2500

// -------------------- I2C Addresses --------------------
#define MPU_ADDR 0x68
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

// -------------------- Sensor Stuff --------------------
#define ACC_LPF_CUTOFF  8.0f
#define GYRO_LPF_CUTOFF 40.0f
#define MAG_LPF_CUTOFF   0.4f
#define FLOW_LPF_CUTOFF   5.0f
#define GYRO_DEADBAND  0.30f
#define GYRO_DEADBAND_Z 0.30f
#define VEL_SPIKE_THRESHOLD 100.0f
#define FLOW_OFFSET_X  0.0f   // +forward
#define FLOW_OFFSET_Y  0.09f 
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
#define ACC_OFFSET_X   1016.50f
#define ACC_OFFSET_Y  -544.50f
#define ACC_OFFSET_Z  -1236.50f

#define GYRO_OFFSET_X  -145.08905f
#define GYRO_OFFSET_Y   33.3236f
#define GYRO_OFFSET_Z  -78.5578f

#define ACC_SCALE_X  0.9981f
#define ACC_SCALE_Y  0.9954f
#define ACC_SCALE_Z  0.9767f

#define MAG_OFFSET_X  -260.10f
#define MAG_OFFSET_Y   -470.07f
#define MAG_OFFSET_Z   100.90f

#define ROLL_OFFSET 0.0f
#define PITCH_OFFSET 0.0f

#define SOFT_IRON { \
  { 0.939f, 0.006f,  0.038f }, \
  { 0.006f, 0.967f,  0.004f }, \
  { 0.038f, 0.004f,  1.103f }  \
}

// -------------------- PID Gains --------------------
// Angle (outer) loop
#define ROLL_ANGLE_KP   5.0f
#define ROLL_ANGLE_KI   0.0f
#define ROLL_ANGLE_KD   0.0f
#define ROLL_ANGLE_LIM  150.0f

#define PITCH_ANGLE_KP  5.0f
#define PITCH_ANGLE_KI  0.0f
#define PITCH_ANGLE_KD  0.0f
#define PITCH_ANGLE_LIM 150.0f

// Rate (inner) loop
#define ROLL_RATE_KP   0.40f
#define ROLL_RATE_KI   0.0005f
#define ROLL_RATE_KD   0.00f
#define ROLL_RATE_LIM  200.0f

#define PITCH_RATE_KP  0.40f
#define PITCH_RATE_KI  0.0005f
#define PITCH_RATE_KD  0.00f
#define PITCH_RATE_LIM 200.0f

#define YAW_RATE_KP    0.40f
#define YAW_RATE_KI    0.02f
#define YAW_RATE_KD    0.0001f
#define YAW_RATE_LIM   200.0f

// Derivative filter alpha (higher = less filtering)
#define PID_D_ALPHA_ANGLE 0.02f
#define PID_D_ALPHA_RATE  0.05f

// -------------------- Mahony Filter --------------------
#define MAHONY_KP  1.0f
#define MAHONY_KI  0.005f

// -------------------- OTHER --------------------
#define ANGLE_SCALE     0.02f   // rollOffset -> desired angle (deg)
#define YAW_RATE_SCALE  0.20f    // yawOffset  -> desired yaw rate (deg/s)
#define LOW_THROTTLE_THRESHOLD 1090
#define SPEED_SOUND 0.000343f
