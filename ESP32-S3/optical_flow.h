#pragma once
#include <Arduino.h>

const uint8_t  MSG_HEAD          = 0xEF;
const uint8_t  MSG_ID_RANGE      = 0x51;
const uint8_t  MAX_PAYLOAD_LEN   = 64;

// ── Parser state machine ─────────────────────────────────────────────────────
struct MicolinkMsg {
  uint8_t  head;
  uint8_t  dev_id;
  uint8_t  sys_id;
  uint8_t  msg_id;
  uint8_t  seq;
  uint8_t  len;
  uint8_t  payload[MAX_PAYLOAD_LEN];
  uint8_t  checksum;
  uint8_t  status;       // parser state (0–7)
  uint8_t  payload_cnt;  // bytes received so far
};

// ── Parsed sensor data ───────────────────────────────────────────────────────
struct SensorData {
  uint32_t time_ms;
  uint32_t distance_mm;
  uint8_t  tof_strength;
  uint8_t  tof_precision;
  uint8_t  tof_status;
  int16_t  flow_vel_x;     // raw: cm/s @ 1m
  int16_t  flow_vel_y;     // raw: cm/s @ 1m
  uint8_t  flow_quality;
  uint8_t  flow_status;
};

extern HardwareSerial sensorSerial;
extern MicolinkMsg msg;


bool verifyChecksum(MicolinkMsg* m); 
bool parseChar(MicolinkMsg* m, uint8_t data);
void extractPayload(MicolinkMsg* m, SensorData& d);
void dataLidarFlow(float &height_lidar, float &vx, float &vy, uint8_t &flow_quality);