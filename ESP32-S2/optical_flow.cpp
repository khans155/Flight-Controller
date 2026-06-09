#include "optical_flow.h"
#include <HardwareSerial.h>
#include "config.h"

HardwareSerial sensorSerial(2);
MicolinkMsg msg;

// ── Checksum: sum of all bytes from head through end of payload ──────────────
bool verifyChecksum(MicolinkMsg* m) {
  uint8_t sum = 0;
  uint8_t headerBytes[6] = { m->head, m->dev_id, m->sys_id,
                              m->msg_id, m->seq, m->len };
  for (int i = 0; i < 6; i++)       sum += headerBytes[i];
  for (int i = 0; i < m->len; i++)  sum += m->payload[i];
  return (sum == m->checksum);
}

// ── Feed one byte through the state machine ──────────────────────────────────
bool parseChar(MicolinkMsg* m, uint8_t data) {
  switch (m->status) {
    case 0:
      if (data == MSG_HEAD) { m->head = data; m->status++; }
      break;
    case 1: m->dev_id = data; m->status++; break;
    case 2: m->sys_id = data; m->status++; break;
    case 3: m->msg_id = data; m->status++; break;
    case 4: m->seq    = data; m->status++; break;
    case 5:
      m->len = data;
      m->payload_cnt = 0;
      if      (m->len == 0)              m->status += 2; // skip payload stage
      else if (m->len > MAX_PAYLOAD_LEN) m->status = 0;  // reject oversized
      else                               m->status++;
      break;
    case 6:
      m->payload[m->payload_cnt++] = data;
      if (m->payload_cnt == m->len) m->status++;
      break;
    case 7:
      m->checksum = data;
      m->status = 0;
      m->payload_cnt = 0;
      return verifyChecksum(m);
    default:
      m->status = 0;
      m->payload_cnt = 0;
      break;
  }
  return false;
}

// ── Copy payload bytes into SensorData struct ────────────────────────────────
void extractPayload(MicolinkMsg* m, SensorData& d) {
  memcpy(&d.time_ms,       m->payload + 0,  4);
  memcpy(&d.distance_mm,   m->payload + 4,  4);
  d.tof_strength  =        m->payload[8];
  d.tof_precision =        m->payload[9];
  d.tof_status    =        m->payload[10];
  // payload[11] = reserved1
  memcpy(&d.flow_vel_x,    m->payload + 12, 2);
  memcpy(&d.flow_vel_y,    m->payload + 14, 2);
  d.flow_quality  =        m->payload[16];
  d.flow_status   =        m->payload[17];
  // payload[18–19] = reserved2
}

void dataLidarFlow(float &height_lidar, float &vx, float &vy, uint8_t &quality) {
  while (sensorSerial.available()){
    uint8_t b = sensorSerial.read();
    if (!parseChar(&msg, b)) continue;
    if (msg.msg_id != MSG_ID_RANGE) continue;

    SensorData d;
    extractPayload(&msg, d);

    height_lidar   = d.distance_mm * 0.001f;
    if (height_lidar < 0.01f){height_lidar = 0.01f;}
    vx  = d.flow_vel_x  * height_lidar;
    vy  = d.flow_vel_y  * height_lidar;
    quality = d.flow_quality;

  }
}