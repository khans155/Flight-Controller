#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <ps5Controller.h>
#include <LiquidCrystal_I2C.h>
#include "esp_bt.h"
#include "pid_menu.h"

#define PIN_CE    2
#define PIN_CSN   15
#define PIN_SCK   14
#define PIN_MOSI  13
#define PIN_MISO  12

#define I2C_SDA   3
#define I2C_SCL   0

#define TELEM_INTERVAL_MS  50
#define TELEM_WINDOW_US    3500

RF24 radio(PIN_CE, PIN_CSN);
const byte ADDRESS[6] = "Node1";

LiquidCrystal_I2C lcd(0x3F, 16, 2);



ControlPacket pkt;
volatile bool newInput = false;

// Last received telemetry — updated in the RX window
TelemetryPacket lastTelem = {};
bool telemValid = false;

// ============================================================
//  LCD helpers
// ============================================================
void lcdStatus(const char* row0, const char* row1 = "") {
    char buf[17];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%-16s", row0);
    lcd.print(buf);
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "%-16s", row1);
    lcd.print(buf);
}

void lcdUpdateControls(const ControlPacket& p) {
    char buf[17];
    snprintf(buf, sizeof(buf), "LX:%-4d LY:%-4d", p.leftX, p.leftY);
    lcd.setCursor(0, 0);
    lcd.print(buf);
    snprintf(buf, sizeof(buf), "L2:%-3d  R2:%-3d", p.l2, p.r2);
    lcd.setCursor(0, 1);
    lcd.print(buf);
}

void lcdUpdateTelemetry(const TelemetryPacket& t) {
    char buf[17];
    // Row 0: roll and pitch
    snprintf(buf, sizeof(buf), "R:%-5.1f P:%-5.1f",
             t.pitch / 100.0f, t.roll / 100.0f);
    lcd.setCursor(0, 0);
    lcd.print(buf);
    // Row 1: altitude in metres
    snprintf(buf, sizeof(buf), "Alt: %-6.2fm    ",
             t.altitude / 100.0f);
    lcd.setCursor(0, 1);
    lcd.print(buf);
}


void notify() {
    pkt.leftX  = ps5.LStickX();
    pkt.leftY  = ps5.LStickY();
    pkt.rightX = ps5.RStickX();
    pkt.rightY = ps5.RStickY();
    pkt.l2     = ps5.L2Value();
    pkt.r2     = ps5.R2Value();

    pkt.buttons = 0;
    if (ps5.R1())       pkt.buttons |= BTN_R1;
    if (ps5.L1())       pkt.buttons |= BTN_L1;
    if (ps5.Cross())    pkt.buttons |= BTN_CROSS;
    if (ps5.Circle())   pkt.buttons |= BTN_CIRCLE;
    if (ps5.Triangle()) pkt.buttons |= BTN_TRIANGLE;
    if (ps5.Square())   pkt.buttons |= BTN_SQUARE;
    if (ps5.L3())       pkt.buttons |= BTN_L3;
    if (ps5.R3())       pkt.buttons |= BTN_R3;
    if (ps5.Options())  pkt.buttons |= BTN_OPTIONS;
    if (ps5.Share())    pkt.buttons |= BTN_SHARE;
    if (ps5.PSButton()) pkt.buttons |= BTN_PS;
    if (ps5.Touchpad()) pkt.buttons |= BTN_TOUCHPAD;
    if (ps5.Left())     pkt.buttons |= BTN_LEFT;
    if (ps5.Right())    pkt.buttons |= BTN_RIGHT;
    if (ps5.Up())       pkt.buttons |= BTN_UP;
    if (ps5.Down())     pkt.buttons |= BTN_DOWN;

    newInput = true;
}

void onConnect() {
    Serial.println("PS5 connected");
}

void onDisconnect() {
    Serial.println("PS5 disconnected");
}


void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    delay(500);

    initTuneDefaults(
    0.55, 0.05,  0.02,
    1.10, 0.05,  0.02,
    8.00, 0.001, 0.00,
    0.07, 0.001, 0.0005,
    5.50, 1.05,  0.02);



    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.init();
    lcd.backlight();
    lcdStatus("ESP32-CAM TX", "Initialising...");

    Serial.println("\n[TX] ESP32-CAM PS5 → NRF24 Transmitter");
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,     ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN,    ESP_PWR_LVL_P9);
    ps5.begin("4c:b9:9b:0c:2b:63");
    //("14:3a:9a:39:3b:b7");

    ps5.attach(notify);
    ps5.attachOnConnect(onConnect);
    ps5.attachOnDisconnect(onDisconnect);

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
    SPI.setFrequency(10000000);

    if (!radio.begin(&SPI)) {
        Serial.println("[TX] ERROR: NRF24L01 not detected. Check wiring!");
        lcdStatus("NRF24 ERROR", "Check wiring!");
        while (true) {}
    }

    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setAutoAck(false);
    radio.setRetries(0, 0);
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.openWritingPipe(ADDRESS);
    radio.openReadingPipe(1, ADDRESS);   // needed for the telem RX window
    radio.stopListening();               // start in TX mode
    radio.powerUp();



    Serial.printf("[RF] Control pkt:   %d bytes\n", sizeof(ControlPacket));
    Serial.printf("[RF] Telemetry pkt: %d bytes\n", sizeof(TelemetryPacket));
    Serial.printf("[RF] Radio payload: %d bytes\n", RADIO_PAYLOAD_SIZE);
    Serial.println("[RF] NRF24 ready.");

    char rfBuf[17];
    snprintf(rfBuf, sizeof(rfBuf), "RF ready %dB", sizeof(ControlPacket));
    lcdStatus(rfBuf, "Waiting for PS5");

}

// ============================================================
void loop() {
  static uint32_t lastReconnectAttempt = 0;
  if (!ps5.isConnected()) {
    delay(10);

    static uint32_t lastBlink = 0;
    static bool blink = false;
    if (millis() - lastBlink >= 500) {
        lastBlink = millis();
        blink = !blink;
        lcdStatus("Waiting for PS5", blink ? "Hold CREATE+PS" : "              ");
    }
    Serial.println("[BT] Waiting for PS5 controller - hold CREATE + PS to pair...");

    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();

      Serial.println("Restarting PS5 interface...");
      ps5.end();
      delay(200);
      ps5.begin("4c:b9:9b:0c:2b:63");
    }
    return;
  }
  if (!newInput) {
    taskYIELD();
    return;
  }

  newInput = false;

  // ── Decide whether to request telemetry this cycle ────────
  static uint32_t lastTelemReq = 0;
  bool reqTelem = (millis() - lastTelemReq >= TELEM_INTERVAL_MS);

  // ── Finalise packet: set flags then (re)calculate CRC ─────
  pkt.flags = reqTelem ? FLAG_TELEM_REQUEST : 0;
  pkt.crc   = calcControlCRC(pkt);


  // ── Transmit ──────────────────────────────────────────────
  radio.stopListening();   // ensure TX mode

    if (reqTelem) {

        radio.flush_tx();
        radio.flush_rx();
        radio.write(&pkt, sizeof(pkt));
        delayMicroseconds(200);

        lastTelemReq = millis();

        // ── Telemetry receive window ───────────────────────────
        // WROOM detects the flag, flips to TX, fires telem back.
        // We listen for TELEM_WINDOW_US then return to TX mode.
        radio.flush_tx();
        radio.flush_rx();
        radio.startListening();
        delayMicroseconds(200);

        uint32_t windowStart = micros();
        bool     gotTelem    = false;
        while ((micros() - windowStart) < TELEM_WINDOW_US) {
            while (radio.available()) {

                TelemetryPacket t;
                radio.read(&t, sizeof(t));

                if (calcTelemCRC(t) == t.crc) {
                    lastTelem  = t;
                    telemValid = true;
                    gotTelem   = true;

                   // Serial.printf(
                     //   "[TELEM] Roll:%6.2f° Pitch:%6.2f° Alt:%6.2fm "
                    //    "Vx:%5.2f Vy:%5.2f Vz:%5.2f m/s\n",
                    //    t.roll      / 100.0f,
                    //    t.pitch     / 100.0f,
                    //    t.altitude  / 100.0f,
                    //    t.velocityX / 100.0f,
                    //    t.velocityY / 100.0f,
                    //    t.velocityZ / 100.0f);
                } else {
                    Serial.println("[TELEM] CRC mismatch — discarded");
                }
                if (gotTelem) break;
            }
        }

        if (!gotTelem) {
            static uint32_t missCount = 0;
            Serial.printf("[TELEM] Missed window #%lu\n", ++missCount);
        }

        radio.stopListening();  // back to TX
        delayMicroseconds(200);
        radio.flush_rx();
        radio.flush_tx();
    } else {
        // Normal cycle — fire and forget, non-blocking
        radio.write(&pkt, sizeof(pkt));
        delayMicroseconds(200);
    }

    handleMenuInput();
    // ── LCD update (throttled) ────────────────────────────────
    static uint32_t lastLCD = 0;
    if (millis() - lastLCD >= 100) {
        lastLCD = millis();
        if (telemValid & (menuState == MENU_OFF)) {

            lcdUpdateTelemetry(lastTelem);
        }
    }

  taskYIELD();   
}
