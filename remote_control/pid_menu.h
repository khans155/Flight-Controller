#pragma once
// ============================================================
//  pid_menu.h  —  PID tuning menu for the ESP32-CAM transmitter
//
//  SETUP CHECKLIST
//  ---------------
//  1. #include "pid_menu.h" near the top of your .ino (it brings
//     in ControlPacket, TelemetryPacket, BTN_*, FLAG_*, TUNE_*).
//
//  2. DELETE these from your .ino — they are now defined here:
//       - ControlPacket  struct
//       - TelemetryPacket struct
//       - RADIO_PAYLOAD_SIZE
//       - All BTN_* defines
//       - All FLAG_* defines
//       - calcControlCRC()
//       - calcTelemCRC()
//
//  3. Call initTuneDefaults() once in setup() with your config values.
//
//  4. At the top of loop(), replace your LCD block:
//       bool inMenu = handleMenuInput();
//       if (!inMenu && millis() - lastLCD >= 100) {
//           lastLCD = millis();
//           telemValid ? lcdUpdateTelemetry(lastTelem) : lcdUpdateControls(pkt);
//       }
//
//  CONTROLS  (DPad + face buttons — all unused by flight)
//  -------------------------------------------------------
//  OPTIONS         Enter / exit the menu
//  DPad UP / DOWN  Scroll params  /  adjust value in adjust mode
//  CROSS           Enter adjust mode  /  confirm & transmit
//  CIRCLE          Cancel adjust      /  exit menu
//
//  Hold UP/DOWN in adjust mode: fine step on first press,
//  x10 step after 500 ms at 80 ms intervals.
// ============================================================

#include <Arduino.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

// ──────────────────────────────────────────────────────────────
//  Packet structs
// ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) ControlPacket {
    int8_t   leftX;
    int8_t   leftY;
    int8_t   rightX;
    int8_t   rightY;
    uint8_t  l2;
    uint8_t  r2;
    uint16_t buttons;
    uint8_t  flags;         // FLAG_TELEM_REQUEST | FLAG_TUNE_CMD
    uint8_t  tuneParamId;   // valid when FLAG_TUNE_CMD set
    float    tuneValue;     // 4 bytes
    uint8_t  crc;           // XOR of bytes 0..13
};  // 15 bytes

struct __attribute__((packed)) TelemetryPacket {
    int16_t  roll;
    int16_t  pitch;
    int16_t  altitude;
    int16_t  velocityX;
    int16_t  velocityY;
    int16_t  velocityZ;
    uint16_t battMv;
    uint8_t  crc;
};  // 15 bytes

#define RADIO_PAYLOAD_SIZE  sizeof(TelemetryPacket)

// ──────────────────────────────────────────────────────────────
//  Flags
// ──────────────────────────────────────────────────────────────
#define FLAG_TELEM_REQUEST  (1 << 0)
#define FLAG_TUNE_CMD       (1 << 1)

// ──────────────────────────────────────────────────────────────
//  Button bitmasks
// ──────────────────────────────────────────────────────────────
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

// ──────────────────────────────────────────────────────────────
//  Tune parameter IDs
// ──────────────────────────────────────────────────────────────
#define TUNE_RATE_KP     0
#define TUNE_RATE_KI     1
#define TUNE_RATE_KD     2
#define TUNE_YAW_KP      3
#define TUNE_YAW_KI      4
#define TUNE_YAW_KD      5
#define TUNE_ANGLE_KP    6
#define TUNE_ANGLE_KI    7
#define TUNE_ANGLE_KD    8
#define TUNE_VEL_KP      9
#define TUNE_VEL_KI     10
#define TUNE_VEL_KD     11
#define TUNE_ALT_KP     12
#define TUNE_ALT_KI     13
#define TUNE_ALT_KD     14
#define TUNE_PARAM_COUNT 15

// ──────────────────────────────────────────────────────────────
//  CRC helpers  (moved here — DELETE originals from your .ino)
// ──────────────────────────────────────────────────────────────
static uint8_t calcControlCRC(const ControlPacket& p) {
    const uint8_t* b = (const uint8_t*)&p;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(ControlPacket) - 1; i++) crc ^= b[i];
    return crc;
}

static uint8_t calcTelemCRC(const TelemetryPacket& p) {
    const uint8_t* b = (const uint8_t*)&p;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) crc ^= b[i];
    return crc;
}

// ──────────────────────────────────────────────────────────────
//  Forward declarations of globals defined in your .ino
//  (no changes needed in the .ino for these)
// ──────────────────────────────────────────────────────────────
extern RF24              radio;
extern LiquidCrystal_I2C lcd;
extern ControlPacket     pkt;

// ──────────────────────────────────────────────────────────────
//  Parameter display names and step sizes
// ──────────────────────────────────────────────────────────────
static const char* TUNE_NAMES[TUNE_PARAM_COUNT] = {
    "Rate  KP", "Rate  KI", "Rate  KD",
    "Yaw   KP", "Yaw   KI", "Yaw   KD",
    "Angle KP", "Angle KI", "Angle KD",
    "Vel   KP", "Vel   KI", "Vel   KD",
    "Alt   KP", "Alt   KI", "Alt   KD"
};

// Fine step per tap; x10 applied automatically on hold
static const float TUNE_STEPS[TUNE_PARAM_COUNT] = {
    0.005f,  0.0005f, 0.0005f,   // Rate  KP / KI / KD
    0.005f,  0.0005f, 0.0005f,   // Yaw   KP / KI / KD
    0.005f,  0.0005f, 0.0005f,   // Angle KP / KI / KD
    0.005f,  0.0005f, 0.0005f,   // Vel   KP / KI / KD
    0.5f,    0.05f,   0.05f      // Alt   KP / KI / KD  (coarser)
};

static float tuneValues[TUNE_PARAM_COUNT] = { 0 };

// ──────────────────────────────────────────────────────────────
//  Call once in setup() to seed the menu with your boot gains
// ──────────────────────────────────────────────────────────────
inline void initTuneDefaults(
    float rateKP,  float rateKI,  float rateKD,
    float yawKP,   float yawKI,   float yawKD,
    float angleKP, float angleKI, float angleKD,
    float velKP,   float velKI,   float velKD,
    float altKP,   float altKI,   float altKD)
{
    tuneValues[TUNE_RATE_KP]  = rateKP;
    tuneValues[TUNE_RATE_KI]  = rateKI;
    tuneValues[TUNE_RATE_KD]  = rateKD;
    tuneValues[TUNE_YAW_KP]   = yawKP;
    tuneValues[TUNE_YAW_KI]   = yawKI;
    tuneValues[TUNE_YAW_KD]   = yawKD;
    tuneValues[TUNE_ANGLE_KP] = angleKP;
    tuneValues[TUNE_ANGLE_KI] = angleKI;
    tuneValues[TUNE_ANGLE_KD] = angleKD;
    tuneValues[TUNE_VEL_KP]   = velKP;
    tuneValues[TUNE_VEL_KI]   = velKI;
    tuneValues[TUNE_VEL_KD]   = velKD;
    tuneValues[TUNE_ALT_KP]   = altKP;
    tuneValues[TUNE_ALT_KI]   = altKI;
    tuneValues[TUNE_ALT_KD]   = altKD;
}

// ──────────────────────────────────────────────────────────────
//  Menu state
// ──────────────────────────────────────────────────────────────
enum MenuState { MENU_OFF, MENU_BROWSE, MENU_ADJUST };
static MenuState menuState = MENU_OFF;
static int       menuIndex = 0;

// ──────────────────────────────────────────────────────────────
//  LCD rendering
// ──────────────────────────────────────────────────────────────
static void lcdUpdateMenu() {
    char row0[17], row1[17], buf[17];

    if (menuState == MENU_BROWSE) {
        snprintf(row0, sizeof(row0), "%-10s %c%c",
            TUNE_NAMES[menuIndex],
            menuIndex > 0                  ? '^' : ' ',
            menuIndex < TUNE_PARAM_COUNT-1 ? 'v' : ' ');
        snprintf(row1, sizeof(row1), "%8.4f  [X]edit",
            tuneValues[menuIndex]);
    } else {
        snprintf(row0, sizeof(row0), "%-8s ^/v  adj",
            TUNE_NAMES[menuIndex]);
        snprintf(row1, sizeof(row1), "%8.4f [X]send",
            tuneValues[menuIndex]);
    }

    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%-16s", row0); lcd.print(buf);
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%-16s", row1); lcd.print(buf);
}

// ──────────────────────────────────────────────────────────────
//  Transmit a one-shot tune packet
// ──────────────────────────────────────────────────────────────
static void sendTuneCommand(uint8_t paramId, float value) {
    ControlPacket tunePkt  = pkt;
    tunePkt.flags          = FLAG_TUNE_CMD;
    tunePkt.tuneParamId    = paramId;
    tunePkt.tuneValue      = value;
    tunePkt.crc            = calcControlCRC(tunePkt);

    radio.stopListening();
    radio.write(&tunePkt, sizeof(tunePkt));
    delayMicroseconds(200);

    Serial.printf("[MENU] Sent TUNE param=%d val=%.4f\n", paramId, value);
}

// ──────────────────────────────────────────────────────────────
//  Main entry point — call every loop() iteration.
//  Returns true while the menu is on screen.
// ──────────────────────────────────────────────────────────────
inline bool handleMenuInput() {
    static bool optPrev   = false;
    static bool upPrev    = false, downPrev  = false;
    static bool crossPrev = false, circPrev  = false;

    static uint32_t holdStart  = 0;
    static uint32_t lastRepeat = 0;
    static bool     holding    = false;

    bool optNow   = (pkt.buttons & BTN_OPTIONS);
    bool upNow    = (pkt.buttons & BTN_UP);
    bool downNow  = (pkt.buttons & BTN_DOWN);
    bool crossNow = (pkt.buttons & BTN_CROSS);
    bool circNow  = (pkt.buttons & BTN_CIRCLE);

    if (optNow && !optPrev) {
        menuState = (menuState == MENU_OFF) ? MENU_BROWSE : MENU_OFF;
        if (menuState == MENU_BROWSE) lcdUpdateMenu();
    }
    optPrev = optNow;

    if (menuState == MENU_OFF) return false;

    if (menuState == MENU_BROWSE) {
        if (upNow && !upPrev && menuIndex > 0) {
            menuIndex--;
            lcdUpdateMenu();
        }
        if (downNow && !downPrev && menuIndex < TUNE_PARAM_COUNT - 1) {
            menuIndex++;
            lcdUpdateMenu();
        }
        if (crossNow && !crossPrev) {
            menuState = MENU_ADJUST;
            holding = false;
            lcdUpdateMenu();
        }
        if (circNow && !circPrev) {
            menuState = MENU_OFF;
        }

    } else {  // MENU_ADJUST
        if (crossNow && !crossPrev) {
            sendTuneCommand(menuIndex, tuneValues[menuIndex]);
            menuState = MENU_BROWSE;
            lcdUpdateMenu();
        }
        if (circNow && !circPrev) {
            menuState = MENU_BROWSE;
            lcdUpdateMenu();
        }

        if (upNow || downNow) {
            float step = TUNE_STEPS[menuIndex];
            float dir  = upNow ? 1.0f : -1.0f;
            uint32_t now = millis();

            if (!holding) {
                if ((upNow && !upPrev) || (downNow && !downPrev)) {
                    tuneValues[menuIndex] = constrain(
                        tuneValues[menuIndex] + dir * step, 0.0f, 500.0f);
                    lcdUpdateMenu();
                    holdStart = lastRepeat = now;
                    holding = true;
                }
            } else if (now - holdStart > 500 && now - lastRepeat >= 80) {
                tuneValues[menuIndex] = constrain(
                    tuneValues[menuIndex] + dir * step * 10.0f, 0.0f, 500.0f);
                lcdUpdateMenu();
                lastRepeat = now;
            }
        } else {
            holding = false;
        }
    }

    upPrev    = upNow;
    downPrev  = downNow;
    crossPrev = crossNow;
    circPrev  = circNow;

    return true;
}