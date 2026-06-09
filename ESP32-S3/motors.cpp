#include "motors.h"
#include "config.h"
#include "sensors.h"
#include "comms.h"
#include <SPI.h>
#include <RF24.h>
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"


// -------------------- PID Controllers --------------------
PID DRAM_ATTR velXPID = {VEL_KP, VEL_KI, VEL_KD, 0, 0, 0, PID_D_ALPHA_VEL, VEL_ANGLE_LIM};
PID DRAM_ATTR velYPID = {VEL_KP, VEL_KI, VEL_KD, 0, 0, 0, PID_D_ALPHA_VEL, VEL_ANGLE_LIM};

PID DRAM_ATTR rollAnglePID  = {ROLL_ANGLE_KP,  ROLL_ANGLE_KI,  ROLL_ANGLE_KD,  0,0,0, PID_D_ALPHA_ANGLE, ROLL_ANGLE_LIM};
PID DRAM_ATTR pitchAnglePID = {PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD, 0,0,0, PID_D_ALPHA_ANGLE, PITCH_ANGLE_LIM};

PID DRAM_ATTR rollRatePID   = {ROLL_RATE_KP,   ROLL_RATE_KI,   ROLL_RATE_KD,   0,0,0, PID_D_ALPHA_RATE,  ROLL_RATE_LIM};
PID DRAM_ATTR pitchRatePID  = {PITCH_RATE_KP,  PITCH_RATE_KI,  PITCH_RATE_KD,  0,0,0, PID_D_ALPHA_RATE,  PITCH_RATE_LIM};
PID DRAM_ATTR yawRatePID    = {YAW_RATE_KP,    YAW_RATE_KI,    YAW_RATE_KD,    0,0,0, PID_D_ALPHA_RATE,  YAW_RATE_LIM};

PID DRAM_ATTR altPID = {ALT_KP, ALT_KI, ALT_KD, 0, 0, 0, PID_D_ALPHA_ALT, ALT_RATE_LIMIT};

// -------------------- Motor state --------------------
float DRAM_ATTR m1_offset = M1_OFFSET_DEFAULT;
float DRAM_ATTR m2_offset = M2_OFFSET_DEFAULT;
float DRAM_ATTR m3_offset = M3_OFFSET_DEFAULT;
float DRAM_ATTR m4_offset = M4_OFFSET_DEFAULT;
float DRAM_ATTR pitch_offset = PITCH_OFFSET, roll_offset =  ROLL_OFFSET;

uint16_t DRAM_ATTR m1, m2, m3, m4;
uint16_t DRAM_ATTR m1_corr, m2_corr, m3_corr, m4_corr;

uint16_t DRAM_ATTR throttle = 1000;
int DRAM_ATTR rollOffset = 0, pitchOffset = 0, yawOffset = 0;


bool DRAM_ATTR armed = false;

bool DRAM_ATTR altHoldActive = false;
float DRAM_ATTR targetHeight = 0;
uint16_t DRAM_ATTR baseThrottle = 1221; 
float DRAM_ATTR lastValidHeight = 0;
float DRAM_ATTR real_height = 0;

bool DRAM_ATTR     throttleLockActive = false;
static uint16_t DRAM_ATTR throttleLockBase   = 1221;

static uint8_t *led_buf = nullptr;

uint16_t batt_rx = 1200;

// -------------------- DSHOT --------------------

const int DRAM_ATTR numEscs = 4;

const gpio_num_t DRAM_ATTR escPins[] = {
    (gpio_num_t)MOTOR1_PIN,
    (gpio_num_t)MOTOR2_PIN,
    (gpio_num_t)MOTOR3_PIN,
    (gpio_num_t)MOTOR4_PIN
};

const rmt_channel_t DRAM_ATTR rmtChannels[] = {
    RMT_CHANNEL_0,
    RMT_CHANNEL_1,
    RMT_CHANNEL_2,
    RMT_CHANNEL_3
};

// DShot300 timings @ clk_div=8
const uint32_t DRAM_ATTR T0H = 12;
const uint32_t DRAM_ATTR T0L = 21;
const uint32_t DRAM_ATTR T1H = 25;
const uint32_t DRAM_ATTR T1L = 8;

// ----- RF24 setup -----
SPIClass spi_nrf(HSPI);
RF24 radio(PIN_CE_NRF, PIN_CSN_NRF);
const byte CONTROL_ADDR[6] = "CTRL1";
const byte TELEM_ADDR[6]   = "TELM1";

// -------------------- Utilities --------------------
float IRAM_ATTR floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float IRAM_ATTR expoCurve(float x, float expo) {
  return x * (1.0f - expo) + x * x * x * expo;
}


static uint8_t calcControlCRC(const ControlPacket& pkt) {
    const uint8_t* b = (const uint8_t*)&pkt;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(ControlPacket) - 1; i++) crc ^= b[i];
    return crc;
}
 
static uint8_t calcTelemCRC(const TelemetryPacket& pkt) {
    const uint8_t* b = (const uint8_t*)&pkt;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) crc ^= b[i];
    return crc;
}

uint8_t IRAM_ATTR dshotCRC(uint16_t packet) {
    uint8_t crc = 0;
    uint16_t csum_data = packet;

    for (int i = 0; i < 3; i++) {
        crc ^= csum_data;
        csum_data >>= 4;
    }

    crc &= 0x0F;
    return crc;
}

void IRAM_ATTR sendDshot(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    uint16_t values[4] = {m1, m2, m3, m4};
    rmt_item32_t items[4][16];

    for (int ch = 0; ch < 4; ch++) {
        uint16_t throttle = values[ch];

        uint16_t dshotValue;
        if (throttle <= 1000) {
            dshotValue = 0;
        } else {
            dshotValue = map(throttle, 1001, 2000, 48, 2047);
            dshotValue = constrain(dshotValue, 48, 2047);
        }

        uint16_t packet = dshotValue << 1;
        packet = (packet << 4) | dshotCRC(packet);

        for (int i = 0; i < 16; i++) {
            bool bit = (packet >> (15 - i)) & 1;

            if (bit) {
                items[ch][i] = {{{ T1H, 1, T1L, 0 }}};
            } else {
                items[ch][i] = {{{ T0H, 1, T0L, 0 }}};
            }
        }
    }

    // Start all channels close together
    for (int ch = 0; ch < 4; ch++) {
        rmt_write_items(rmtChannels[ch], items[ch], 16, false);
    }

    // Optional. Usually not needed at 1 kHz because the next frame is far away.
    // for (int ch = 0; ch < 4; ch++) {
    //     rmt_wait_tx_done(rmtChannels[ch], pdMS_TO_TICKS(1));
    // }
}

static inline void IRAM_ATTR send_bit_1() {
    GPIO.out_w1ts = (1UL << LED_PIN);  // direct register write, faster than digitalWrite
    T1H_NOPS;
    GPIO.out_w1tc = (1UL << LED_PIN);
    T1L_NOPS;
}

static inline void IRAM_ATTR send_bit_0() {
    GPIO.out_w1ts = (1UL << LED_PIN);
    T0H_NOPS;
    GPIO.out_w1tc = (1UL << LED_PIN);
    T0L_NOPS;
}

static void IRAM_ATTR send_byte(uint8_t val) {
    for (int i = 7; i >= 0; i--) {
        if (val & (1 << i)) send_bit_1();
        else                 send_bit_0();
    }
}

void argb_init() {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    GPIO.out_w1tc = (1UL << LED_PIN);
}

void IRAM_ATTR argb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    portDISABLE_INTERRUPTS();
    send_byte(g);
    send_byte(r);
    send_byte(b);
    portENABLE_INTERRUPTS();

    // Reset pulse — pin stays low after last bit anyway
    // but ets_delay_us is safe here since interrupts are back on
    ets_delay_us(60);
}

// -------------------- Init Motors/Controller --------------------
void initMotors() {
  // ---------------- DSHOT INIT ----------------

  for (int i = 0; i < numEscs; i++) {

      rmt_config_t config;

      config.rmt_mode = RMT_MODE_TX;
      config.channel = rmtChannels[i];
      config.gpio_num = escPins[i];
      config.mem_block_num = 1;

      config.clk_div = 8;

      config.tx_config.loop_en = false;
      config.tx_config.carrier_en = false;
      config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
      config.tx_config.idle_output_en = true;

      rmt_config(&config);
      rmt_driver_install(rmtChannels[i], 0, 0);
  }

  delay(100);

  argb_init();

  // Send idle packets so ESC initializes properly
  for (int i = 0; i < 1000; i++) {
      sendDshot(1000, 1000, 1000, 1000);
      delayMicroseconds(100);
  }

  // ---------------- NRF INIT ----------------

  spi_nrf.begin(NRF_SCK, NRF_MISO, NRF_MOSI, PIN_CSN_NRF);
  spi_nrf.setFrequency(80000000);

  while (!radio.begin(&spi_nrf)) {
    Serial.println("[RX] ERROR: NRF24L01 not detected. Check wiring!");
    delay(100);
  }

  radio.setPALevel(RF24_PA_MAX);       
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setDataRate(RF24_250KBPS);     
  radio.setChannel(76);               
  radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
  radio.openReadingPipe(1, CONTROL_ADDR);
  radio.openWritingPipe(TELEM_ADDR);
  radio.startListening(); 
  radio.powerUp();            
}

void sendTelemetry() {
    TelemetryPacket t;
 
    // Pull live values from sensor fusion outputs (sensors.h externs)
    t.roll      = (int16_t)(roll_ag           * 100.0f);
    t.pitch     = (int16_t)(pitch_ag          * 100.0f);
    t.altitude  = (int16_t)(height_lidar      * 100.0f);
    t.velocityX = (int16_t)(vx_est            * 1.0f);
    t.velocityY = (int16_t)(vy_est            * 1.0f);
    t.velocityZ = (int16_t)(vz_f              * 1.0f);
    if (!armed)
      t.state = (int8_t)0;
    else {
      if (throttleLockActive)
        t.state = (int8_t)3;
      else if (altHoldActive)
        t.state = (int8_t)2;
      else
        t.state = (int8_t)1;
    }

    t.throttle  = (int16_t)(throttle          * 1.0f);
    t.battMv    = (int16_t)(batt_rx           * 1.0f);
    t.crc       = calcTelemCRC(t);
 
    radio.stopListening();                        // flip to TX
    //delayMicroseconds(150);
    //radio.write(&t, RADIO_PAYLOAD_SIZE);
    bool ok = radio.writeFast(&t, sizeof(t));
    radio.txStandBy(2);
    radio.startListening();                       // flip back to RX
    

}

void applyTuneCommand(uint8_t paramId, float value) {
    switch (paramId) {
        // ── Rate (roll & pitch linked) ──────────────────────
        case TUNE_RATE_KP:
            rollRatePID.kp  = pitchRatePID.kp  = value;
            remotePrint("[TUNE] Rate  KP = %.4f\r\n", value);
            break;
        case TUNE_RATE_KI:
            rollRatePID.ki  = pitchRatePID.ki  = value;
            remotePrint("[TUNE] Rate  KI = %.4f\r\n", value);
            break;
        case TUNE_RATE_KD:
            rollRatePID.kd  = pitchRatePID.kd  = value;
            remotePrint("[TUNE] Rate  KD = %.4f\r\n", value);
            break;
 
        // ── Yaw rate ────────────────────────────────────────
        case TUNE_YAW_KP:
            yawRatePID.kp   = value;
            remotePrint("[TUNE] Yaw   KP = %.4f\r\n", value);
            break;
        case TUNE_YAW_KI:
            yawRatePID.ki   = value;
            remotePrint("[TUNE] Yaw   KI = %.4f\r\n", value);
            break;
        case TUNE_YAW_KD:
            yawRatePID.kd   = value;
            remotePrint("[TUNE] Yaw   KD = %.4f\r\n", value);
            break;
 
        // ── Angle (roll & pitch linked) ─────────────────────
        case TUNE_ANGLE_KP:
            rollAnglePID.kp = pitchAnglePID.kp = value;
            remotePrint("[TUNE] Angle KP = %.4f\r\n", value);
            break;
        case TUNE_ANGLE_KI:
            rollAnglePID.ki = pitchAnglePID.ki = value;
            remotePrint("[TUNE] Angle KI = %.4f\r\n", value);
            break;
        case TUNE_ANGLE_KD:
            rollAnglePID.kd = pitchAnglePID.kd = value;
            remotePrint("[TUNE] Angle KD = %.4f\r\n", value);
            break;
 
        // ── Velocity (X & Y linked) ─────────────────────────
        case TUNE_VEL_KP:
            velXPID.kp      = velYPID.kp       = value;
            remotePrint("[TUNE] Vel   KP = %.4f\r\n", value);
            break;
        case TUNE_VEL_KI:
            velXPID.ki      = velYPID.ki       = value;
            remotePrint("[TUNE] Vel   KI = %.4f\r\n", value);
            break;
        case TUNE_VEL_KD:
            velXPID.kd      = velYPID.kd       = value;
            remotePrint("[TUNE] Vel   KD = %.4f\r\n", value);
            break;
 
        // ── Altitude ────────────────────────────────────────
        case TUNE_ALT_KP:
            altPID.kp       = value;
            remotePrint("[TUNE] Alt   KP = %.4f\r\n", value);
            break;
        case TUNE_ALT_KI:
            altPID.ki       = value;
            remotePrint("[TUNE] Alt   KI = %.4f\r\n", value);
            break;
        case TUNE_ALT_KD:
            altPID.kd       = value;
            remotePrint("[TUNE] Alt   KD = %.4f\r\n", value);
            break;
 
        default:
            remotePrint("[TUNE] Unknown paramId %d — ignored\r\n", paramId);
            break;
    }
}

// -------------------- PS5 Input --------------------
void readPS5() {
  if (!radio.available()) return; 
  ControlPacket pkt;
  radio.read(&pkt, sizeof(pkt));
  
  // ── Validate checksum ─────────────────────────────────────
  if (calcControlCRC(pkt) != pkt.crc) {
      remotePrint("[RX] CRC mismatch — packet discarded");
      return;
  }
  // ── Tune command ─────────────────────────────────────────
  if (pkt.flags & FLAG_TUNE_CMD) {
      applyTuneCommand(pkt.tuneParamId, pkt.tuneValue);
      return;
  }
  // ── Telemetry request ────────────────────────────────────
  if (pkt.flags & FLAG_TELEM_REQUEST) {
      sendTelemetry();
  }

  // ── Alt-hold toggle (X button, armed only) ──────────────
  static bool xPressedLast = false;
  static bool squarePressedLast = false;
  static bool circlePressedLast = false;

  bool squarePressedNow = (pkt.buttons & BTN_SQUARE);
  bool xPressedNow = (pkt.buttons & BTN_CROSS);
  bool circlePressedNow = (pkt.buttons & BTN_CIRCLE);

  real_height =  fabsf(height_filtered
               * cos(roll_ag* DEG_TO_RAD) 
               * cos(pitch_ag* DEG_TO_RAD )) * 100.0f; // tilt compensation and unit conversion to cm
  
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

    // ── Throttle-lock toggle (Circle button, armed only) ────
    if (circlePressedNow && !circlePressedLast) {
      if (!throttleLockActive) {
        throttleLockBase   = throttle;
        throttleLockActive = true;
      } else {
        throttleLockActive = false;
        throttle           = throttleLockBase;  
      }
    }
    circlePressedLast = circlePressedNow;
  }

  // Calibrate MPU on square press — only allowed while disarmed
  if (!armed && squarePressedNow && !squarePressedLast) {
    calibrateMPU();
  }
  squarePressedLast = squarePressedNow;

  // ── Stick processing ──────────────────────────────────────
  int16_t roll_raw  = constrain((pkt.leftX), -127, 127);
  int16_t pitch_raw = constrain(-1*(pkt.leftY), -127, 127);
  int16_t yaw_raw   = constrain(pkt.rightX, -127, 127);
  
  int deadzone = 25;
  if (roll_raw  < deadzone && roll_raw  > -deadzone) roll_raw = 0;
  if (pitch_raw < deadzone && pitch_raw > -deadzone) pitch_raw = 0;
  if (yaw_raw   < deadzone && yaw_raw   > -deadzone) yaw_raw = 0;


  
  const float expo = 0.75f;   

  // normalize stick inputs to -1 → 1
  float rollNorm  = roll_raw  / 127.0f;
  float pitchNorm = -pitch_raw / 127.0f;
  float yawNorm   = yaw_raw   / 127.0f;

  // apply expo
  rollNorm  = expoCurve(rollNorm, expo);
  pitchNorm = expoCurve(pitchNorm, expo);
  yawNorm   = expoCurve(yawNorm, expo);

  // convert to 1000–2000 RC range
  uint16_t roll  = 1500 + (rollNorm  * 500.0f);
  uint16_t pitch = 1500 + (pitchNorm * 500.0f);
  uint16_t yaw   = 1500 + (yawNorm   * 500.0f);


  // ── Arm / disarm ──────────────────────────────────────────
  if ((pkt.buttons & BTN_R1) && throttle == 1000) {
    throttle = 1221;
    armed = true;
  }


   if (!altHoldActive) {
    const int triggerDeadzone = 10;
 
    uint8_t r2 = pkt.r2;
    uint8_t l2 = pkt.l2;
 
    if (r2 <= triggerDeadzone) r2 = 0;
    if (l2 <= triggerDeadzone) l2 = 0;
 
    if (throttleLockActive) {
      int16_t triggerNet = (int16_t)(r2*1.5f) - (int16_t)(l2*1.05f);
      throttle = (uint16_t)constrain(
          (int32_t)throttleLockBase + (int32_t)(triggerNet * THROTTLE_LOCK_SCALE),
          1221, 2000);
    } else {
      // ── Normal mode: step-increment / step-decrement ──────────────────
      static unsigned long lastR2Step = 0;
      static unsigned long lastL2Step = 0;
      unsigned long now = millis();
 
      // R2 only increases throttle once armed (throttle >= 1221)
      if (r2 > 0 && throttle >= 1221) {
        float rate = floatMap(r2, triggerDeadzone, 255, 1.0f, 250.0f);
        unsigned long interval = (unsigned long)(1000.0f / rate);
        if (now - lastR2Step >= interval) {
          throttle = constrain(throttle + 1, 1221, 2000);
          lastR2Step = now;
        }
      }
 
      // L2 only decreases throttle once armed, floor is idle (1221)
      if (l2 > 0 && throttle >= 1221) {
        float rate = floatMap(l2, triggerDeadzone, 255, 1.0f, 250.0f);
        unsigned long interval = (unsigned long)(1000.0f / rate);
        if (now - lastL2Step >= interval) {
          throttle = constrain(throttle - 1, 1221, 2000);
          lastL2Step = now;
        }
      }
    }
  }
  // 4. L1 disarms — resets throttle to minimum (1000)
  if (pkt.buttons & BTN_L1) {
    throttle           = 1000;
    armed              = false;
    altHoldActive      = false;
    throttleLockActive = false;
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

float IRAM_ATTR linearizeMotor(float us)
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

void IRAM_ATTR mixMotor(uint64_t now) {
  static uint64_t DRAM_ATTR lastMicros = 0;

  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.05f){
    lastMicros = now;
    return;
  }
  lastMicros = now;

  bool lowThrottle = (throttle < LOW_THROTTLE_THRESHOLD);
  float desiredRollAngle = 0, desiredPitchAngle = 0;

  if (abs(pitchOffset) > 0) {
      desiredPitchAngle = pitchOffset * ANGLE_SCALE;
      velYPID.integral = 0; 
    } else {
      desiredPitchAngle = computePID(velYPID, vy_est, dt, lowThrottle);
    }
  if (abs(rollOffset) > 0) {
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

  sendDshot(m1, m2, m3, m4);
}



void IRAM_ATTR emergencyMotorStop() {

  for (int i = 0; i < 200; i++) {
      sendDshot(1000, 1000, 1000, 1000);
  }

  volatile uint32_t x = 0;
  while (x++ < 80000);
}