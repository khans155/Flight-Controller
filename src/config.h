#pragma once

// -------------------- WiFi --------------------
#define WIFI_SSID     "-----"
#define WIFI_PASSWORD "-----"

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

// -------------------- I2C --------------------
#define I2C_PORT I2C_NUM_0
#define I2C_SDA  21
#define I2C_SCL  22
#define I2C_FREQ 1000000
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

// -------------------- Sensor Stuff --------------------
#define ACC_LPF_CUTOFF  8.0f
#define GYRO_LPF_CUTOFF 40.0f
#define MAG_LPF_CUTOFF   0.4f
#define FLOW_LPF_CUTOFF   10.0f
#define HEIGHT_LPF 2.0f
#define GYRO_DEADBAND  0.25f
#define GYRO_DEADBAND_Z 0.30f
#define VEL_SPIKE_THRESHOLD 0.5f
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

#define GYRO_OFFSET_X  -31.4412f
#define GYRO_OFFSET_Y   47.4639f
#define GYRO_OFFSET_Z  -5.3521f

#define GYRO2_OFFSET_X  -34.6470f
#define GYRO2_OFFSET_Y   7.1321f
#define GYRO2_OFFSET_Z  -29.9797f

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
#define VEL_KP   0.15f
#define VEL_KI   0.01f
#define VEL_KD   0.0015f
#define VEL_ANGLE_LIM  8.0f


// Angle loop
#define ROLL_ANGLE_KP   9.0f
#define ROLL_ANGLE_KI   0.0001f
#define ROLL_ANGLE_KD   0.0f
#define ROLL_ANGLE_LIM  150.0f

#define PITCH_ANGLE_KP  9.0f
#define PITCH_ANGLE_KI  0.0001f
#define PITCH_ANGLE_KD  0.0f
#define PITCH_ANGLE_LIM 150.0f

// Rate loop
#define ROLL_RATE_KP   0.35f
#define ROLL_RATE_KI   0.00001f
#define ROLL_RATE_KD   0.00001f
#define ROLL_RATE_LIM  200.0f

#define PITCH_RATE_KP  0.35f
#define PITCH_RATE_KI  0.00001f
#define PITCH_RATE_KD  0.00001f
#define PITCH_RATE_LIM 200.0f

#define YAW_RATE_KP    0.75f
#define YAW_RATE_KI    0.3f
#define YAW_RATE_KD    0.01f
#define YAW_RATE_LIM   200.0f

// Altitude loop
#define ALT_KP  5.5f
#define ALT_KI   0.05f
#define ALT_KD   0.01f
#define ALT_RATE_LIMIT  300.0f

// Derivative filter alpha (higher = less filtering)
#define PID_D_ALPHA_VEL 0.02f
#define PID_D_ALPHA_ANGLE 0.02f
#define PID_D_ALPHA_RATE  0.05f
#define PID_D_ALPHA_ALT 0.02f

// -------------------- Mahony Filter --------------------
#define MAHONY_KP  1.0f
#define MAHONY_KI  0.005f

// -------------------- OTHER --------------------
#define ANGLE_SCALE     0.03f   // rollOffset -> desired angle (deg)
#define YAW_RATE_SCALE  0.30f    // yawOffset  -> desired yaw rate (deg/s)
#define LOW_THROTTLE_THRESHOLD 1090
#define SPEED_SOUND 0.000343f
#define MAX_ALTITUDE 450.0f
#define HEIGHT_SPIKE_LIMIT 5.0f
