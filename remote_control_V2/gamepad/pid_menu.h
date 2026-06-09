#pragma once
#include <Arduino.h>
#include <RF24.h>

#define WIFI_SSID     "WuTangLan"
#define WIFI_PASSWORD "W0rkpl@y$le3p"

//  Packet structs
struct __attribute__((packed)) ControlPacket {
    int8_t   leftX;
    int8_t   leftY;
    int8_t   rightX;
    int8_t   rightY;
    uint8_t  l2;
    uint8_t  r2;
    uint16_t buttons;
    uint8_t  flags;         
    uint8_t  tuneParamId;  
    float    tuneValue;     
    uint8_t  crc;           
};  // 15 bytes

struct __attribute__((packed)) TelemetryPacket {
    int16_t  roll;
    int16_t  pitch;
    int16_t  altitude;
    int16_t  velocityX;
    int16_t  velocityY;
    int16_t  velocityZ;
    int8_t   state;
    uint16_t throttle;
    uint16_t battMv;
    uint8_t  crc;
};  // 15 bytes

enum DisplayPacketType {
    PACKET_TELEM = 0,
    PACKET_TEXT  = 1
};

#pragma pack(push, 1)
struct DisplayPacket {
    uint8_t type;
    union {
        struct {
            float roll;
            float pitch;
            float alt;
            float throttle;
            float batt_tx;
            float batt_rx;
            int8_t   state;
        } telem;
        struct {
            char line1[17];
            char line2[17];
            char line3[17];
            char line4[17];
        } text;
    } data;
};
#pragma pack(pop)

extern void sendToDisplay(const DisplayPacket& pkt);

#define RADIO_PAYLOAD_SIZE  sizeof(TelemetryPacket)
#define CYD_I2C_ADDR 0x42

//  Flags
#define FLAG_TELEM_REQUEST  (1 << 0)
#define FLAG_TUNE_CMD       (1 << 1)


//  Button bitmasks
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

//  Tune bitmasks
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

extern RF24              DRAM_ATTR radio;
extern ControlPacket     DRAM_ATTR pkt;

//  Parameter display names and step sizes
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
    0.5f,    0.05f,   0.05f      // Alt   KP / KI / KD 
};

static float tuneValues[TUNE_PARAM_COUNT] = { 0 };
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

//  Menu state
enum MenuState {
    MENU_OFF,
    MENU_ROOT,
    MENU_SUB,
    MENU_EDIT
};

enum RootMenu {
    ROOT_PID,
    ROOT_LPF,
    ROOT_CAL,
    ROOT_MOTOR,
    ROOT_SPARE1,
    ROOT_SPARE2,
    ROOT_SPARE3,
    ROOT_COUNT
};


const char* ROOT_NAMES[ROOT_COUNT] = {
    "PID",
    "LPF",
    "Calibration",
    "Motor Scale",
    "Spare 1",
    "Spare 2",
    "Spare 3"
};

MenuState menuState = MENU_OFF;
int rootIndex = 0;
int subIndex  = 0;

// Motor scale
float motorScale[4] = {1.0,1.0,1.0,1.0};

// LPF placeholders
float gyroLPF  = 30;
float accelLPF = 5;
float magLPF   = 1;
float flowLPF  = 5;
float lidarLPF = 10;

// LCD helper

void lcdStatus(const char *row1, const char *row2, const char *row3, const char *row4) {
    DisplayPacket pkt;
    pkt.type = PACKET_TEXT;
    
    // Copy and ensure null-termination
    strncpy(pkt.data.text.line1, row1, 16);
    pkt.data.text.line1[16] = '\0'; 
    strncpy(pkt.data.text.line2, row2, 16);
    pkt.data.text.line2[16] = '\0';
    strncpy(pkt.data.text.line3, row3, 16);
    pkt.data.text.line3[16] = '\0';
    strncpy(pkt.data.text.line4, row4, 16);
    pkt.data.text.line4[16] = '\0';
    
    sendToDisplay(pkt);
}

void lcdUpdateTelemetry(const TelemetryPacket &t) {
    DisplayPacket pkt;
    pkt.type = PACKET_TELEM;
    pkt.data.telem.roll       = t.pitch / -100.0f;
    pkt.data.telem.pitch      = t.roll / -100.0f;
    pkt.data.telem.alt        = t.altitude / 100.0f;
    pkt.data.telem.throttle   = t.throttle / 100.0f;
    pkt.data.telem.batt_tx    = 8.1 / 100.0f;
    pkt.data.telem.batt_rx    = t.battMv / 100.0f;
    pkt.data.telem.state      = t.state;
    
    sendToDisplay(pkt);
}

void lcdPrint2(const char* r1, const char* r2, const char* r3, const char* r4) {
    DisplayPacket pkt;
    pkt.type = PACKET_TEXT;
    
    strncpy(pkt.data.text.line1, r1, 16);
    pkt.data.text.line1[16] = '\0';
    strncpy(pkt.data.text.line2, r2, 16);
    pkt.data.text.line2[16] = '\0';
    strncpy(pkt.data.text.line3, r3, 16);
    pkt.data.text.line3[16] = '\0';
    strncpy(pkt.data.text.line4, r4, 16);
    pkt.data.text.line4[16] = '\0';
    
    sendToDisplay(pkt);
}

void drawRootMenu()
{
    char row1[17] = "";
    char row2[17] = "";
    char row3[17] = "";

    snprintf(row1, 17, ">%s", ROOT_NAMES[rootIndex]);
    if (rootIndex + 1 < ROOT_COUNT)
        snprintf(row2, 17, "%s", ROOT_NAMES[rootIndex + 1]);
    else
        snprintf(row2, 17, " ");
    if (rootIndex + 2 < ROOT_COUNT)
        snprintf(row3, 17, "%s", ROOT_NAMES[rootIndex + 2]);
    else
        snprintf(row3, 17, " ");
    lcdPrint2("Options", row1, row2, row3);
}

void drawPIDMenu()
{
    char row1[17] = "";
    char row2[17] = "";
    char row3[17] = "";

    snprintf(row1, 17, ">%s", TUNE_NAMES[subIndex]);
    if (subIndex + 1 < TUNE_PARAM_COUNT)
        snprintf(row2, 17, "%s", TUNE_NAMES[subIndex + 1]);
    else
        snprintf(row2, 17, " ");
    if (subIndex + 2 < TUNE_PARAM_COUNT)
        snprintf(row3, 17, "%s", TUNE_NAMES[subIndex + 2]);
    else
        snprintf(row3, 17, " ");
    lcdPrint2("PID", row1, row2, row3);
}

// Draw edit screen
void drawEdit(const char* name,float val)
{
    char row1[17];
    snprintf(row1,17,"%6.4f  ^/v",val);
    lcdPrint2(name,row1, " ", " ");
}

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
bool handleMenuInput()
{
    static bool optPrev=false;
    static bool upPrev=false;
    static bool downPrev=false;
    static bool crossPrev=false;
    static bool circPrev=false;

    bool optNow   = pkt.buttons & BTN_OPTIONS;
    bool upNow    = pkt.buttons & BTN_UP;
    bool downNow  = pkt.buttons & BTN_DOWN;
    bool crossNow = pkt.buttons & BTN_CROSS;
    bool circNow  = pkt.buttons & BTN_CIRCLE;

    // OPTIONS toggles menu
    if(optNow && !optPrev)
    {
        if(menuState==MENU_OFF)
        {
            menuState=MENU_ROOT;
            drawRootMenu();
        }
        else{
            menuState=MENU_OFF;
            rootIndex = 0;
            subIndex = 0;
            lcdStatus("Gamepad ready", " ", " ", " ");
        }
    }

    optPrev=optNow;

    if(menuState==MENU_OFF){
        lcdStatus("Gamepad ready", " ", " ", " ");
        rootIndex = 0;
        subIndex = 0;
        return false;
    }
    // ROOT MENU
    if(menuState==MENU_ROOT)
    {
        if(upNow && !upPrev && rootIndex>0)
        {
            rootIndex--;
            drawRootMenu();
        }

        if(downNow && !downPrev && rootIndex < ROOT_COUNT - 1)
        {
            rootIndex++;
            drawRootMenu();
        }

        if(crossNow && !crossPrev)
        {
            subIndex=0;
            menuState=MENU_SUB;

            if(rootIndex==ROOT_PID)
                drawPIDMenu();
        }
        if(circNow && !circPrev)
        {
            menuState=MENU_OFF;
            lcdStatus("Gamepad ready", " ", " ", " ");
        }
    }

    // PID SUBMENU
    else if(menuState==MENU_SUB && rootIndex==ROOT_PID)
    {
        if(upNow && !upPrev && subIndex>0)
        {
            subIndex--;
            drawPIDMenu();
        }

        if(downNow && !downPrev && subIndex<TUNE_PARAM_COUNT-1)
        {
            subIndex++;
            drawPIDMenu();
        }

        if(crossNow && !crossPrev)
        {
            menuState=MENU_EDIT;
            drawEdit(TUNE_NAMES[subIndex],tuneValues[subIndex]);
        }

        if(circNow && !circPrev)
        {
            menuState=MENU_ROOT;
            drawRootMenu();
        }
    }

    // EDIT VALUE
    else if(menuState==MENU_EDIT)
    {
        if(upNow && !upPrev)
        {
            tuneValues[subIndex]+=TUNE_STEPS[subIndex];
            drawEdit(TUNE_NAMES[subIndex],tuneValues[subIndex]);
        }

        if(downNow && !downPrev)
        {
            tuneValues[subIndex]-=TUNE_STEPS[subIndex];
            drawEdit(TUNE_NAMES[subIndex],tuneValues[subIndex]);
        }

        if(crossNow && !crossPrev)
        {
            sendTuneCommand(subIndex,tuneValues[subIndex]);
            menuState=MENU_SUB;
            drawPIDMenu();
        }

        if(circNow && !circPrev)
        {
            menuState=MENU_SUB;
            drawPIDMenu();
        }
    }

    upPrev=upNow;
    downPrev=downNow;
    crossPrev=crossNow;
    circPrev=circNow;

    return true;
}