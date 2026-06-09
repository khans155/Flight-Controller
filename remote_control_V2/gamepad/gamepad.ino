
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "pid_menu.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
// GPIO 19 & 20 are reserved for USB-OTG — every other assignment goes here.
// Adjust to match your specific ESP32-S3 breakout board.

#define PIN_CE    4
#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MOSI  17
#define PIN_MISO  16

#define I2C_SDA   8
#define I2C_SCL   9

// ── Timing ────────────────────────────────────────────────────────────────────
#define TELEM_INTERVAL_MS  50
#define TELEM_WINDOW_US    2500

// ── NRF24 ─────────────────────────────────────────────────────────────────────
RF24 DRAM_ATTR radio(PIN_CE, PIN_CSN);
const byte CONTROL_ADDR[6] = "CTRL1";
const byte TELEM_ADDR[6]   = "TELM1";



// ── Shared control state ──────────────────────────────────────────────────────
ControlPacket   DRAM_ATTR pkt;
volatile bool   DRAM_ATTR newInput  = false;
TelemetryPacket DRAM_ATTR lastTelem = {};
bool            DRAM_ATTR telemValid = false;

// ── USB Host state ────────────────────────────────────────────────────────────
#define HID_REPORT_MAX      64
#define USB_HOST_PRIORITY   5
#define USB_CLIENT_PRIORITY 4

static usb_host_client_handle_t s_client_hdl   = NULL;
static usb_device_handle_t      s_dev_hdl      = NULL;
static uint8_t                  s_dev_addr     = 0;
static uint8_t                  s_ep_in_addr   = 0;
static uint16_t                 s_ep_in_mps    = 0;
static uint8_t                  s_intf_num     = 0;
static usb_transfer_t          *s_in_xfer      = NULL;
static SemaphoreHandle_t        s_device_ready = NULL;

// ── Forward declarations ──────────────────────────────────────────────────────
static void usb_host_task(void *arg);
static void usb_client_task(void *arg);
static void claim_hid_interface(void);
static void submit_in_transfer(void);
static void on_transfer_done(usb_transfer_t *xfer);
static void parse_gamepad_report(const uint8_t *data, size_t len);


// =============================================================================
//  LCD helpers
// =============================================================================



void sendToDisplay(const DisplayPacket& pkt) {
    Wire.beginTransmission(CYD_I2C_ADDR);
    Wire.write((const uint8_t*)&pkt, sizeof(DisplayPacket));
    Wire.endTransmission();
}




// =============================================================================
//  USB Host daemon task  (drives the ESP-IDF USB Host library event loop)
// =============================================================================

static void usb_host_task(void *arg) {
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            Serial.println("[USB] No clients — freeing devices");
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            Serial.println("[USB] All devices freed");
        }
    }
}


// =============================================================================
//  USB Client task  (handles device connect / disconnect events)
// =============================================================================

static void usb_client_task(void *arg) {
    usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 10,
        .async = {
            .client_event_callback = [](const usb_host_client_event_msg_t *msg, void *) {
                switch (msg->event) {

                    case USB_HOST_CLIENT_EVENT_NEW_DEV:
                        s_dev_addr = msg->new_dev.address;
                        Serial.printf("[USB] Device connected — addr %d\n", s_dev_addr);
                        xSemaphoreGive(s_device_ready);
                        break;

                    case USB_HOST_CLIENT_EVENT_DEV_GONE:
                        Serial.println("[USB] Device disconnected");
                        s_dev_hdl    = NULL;
                        s_ep_in_addr = 0;
                        if (s_in_xfer) {
                            usb_host_transfer_free(s_in_xfer);
                            s_in_xfer = NULL;
                        }
                        lcdStatus("Ctrl lost", "Replug gamepad", " ", " ");
                        break;

                    default:
                        break;
                }
            },
            .callback_arg = NULL,
        },
    };

    ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &s_client_hdl));
    Serial.println("[USB] Client registered — waiting for gamepad...");

    while (true) {
        if (xSemaphoreTake(s_device_ready, 0) == pdTRUE) {
            if (usb_host_device_open(s_client_hdl, s_dev_addr, &s_dev_hdl) == ESP_OK) {
                Serial.println("[USB] Device opened");
                claim_hid_interface();
            } else {
                Serial.println("[USB] Failed to open device");
            }
        }
        usb_host_client_handle_events(s_client_hdl, pdMS_TO_TICKS(10));
    }
}


// =============================================================================
//  Enumerate descriptors, find the first HID Interrupt IN endpoint, claim it
// =============================================================================

static void claim_hid_interface() {
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(s_dev_hdl, &config_desc));

    int                    offset   = 0;
    const usb_intf_desc_t *hid_intf = NULL;
    const usb_ep_desc_t   *hid_ep   = NULL;

    while (offset < config_desc->wTotalLength) {
        const usb_standard_desc_t *desc =
            (const usb_standard_desc_t *)((uint8_t *)config_desc + offset);
        if (desc->bLength == 0) break;

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc;
            if (intf->bInterfaceClass == 0x03) {    // 0x03 = HID
                hid_intf = intf;
                Serial.printf("[USB] HID interface %d (sub=%02X proto=%02X)\n",
                              intf->bInterfaceNumber,
                              intf->bInterfaceSubClass,
                              intf->bInterfaceProtocol);
            }
        }

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && hid_intf) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)desc;
            bool is_in   = (ep->bEndpointAddress & 0x80) != 0;
            bool is_intr = (ep->bmAttributes & 0x03) == 0x03;
            if (is_in && is_intr && s_ep_in_addr == 0) {
                hid_ep       = ep;
                s_ep_in_addr = ep->bEndpointAddress;
                s_ep_in_mps  = ep->wMaxPacketSize;
                s_intf_num   = hid_intf->bInterfaceNumber;
                Serial.printf("[USB] Interrupt IN EP 0x%02X — MPS=%d bytes\n",
                              s_ep_in_addr, s_ep_in_mps);
            }
        }
        offset += desc->bLength;
    }

    if (!hid_ep) {
        Serial.println("[USB] No HID Interrupt IN endpoint found — is this a HID gamepad?");
        lcdStatus("No HID endpt", "Not a gamepad?", " ", " ");
        return;
    }

    esp_err_t err = usb_host_interface_claim(s_client_hdl, s_dev_hdl, s_intf_num, 0);
    if (err != ESP_OK) {
        Serial.printf("[USB] Interface claim failed: %s\n", esp_err_to_name(err));
        return;
    }
    Serial.printf("[USB] Interface %d claimed — streaming reports\n", s_intf_num);
    lcdStatus("Gamepad ready", " ", " ", " ");

    // Allocate transfer buffer, rounded up to a whole number of MPS packets
    size_t buf_size = ((HID_REPORT_MAX + s_ep_in_mps - 1) / s_ep_in_mps) * s_ep_in_mps;
    ESP_ERROR_CHECK(usb_host_transfer_alloc(buf_size, 0, &s_in_xfer));
    s_in_xfer->device_handle    = s_dev_hdl;
    s_in_xfer->bEndpointAddress = s_ep_in_addr;
    s_in_xfer->callback         = on_transfer_done;
    s_in_xfer->context          = NULL;
    s_in_xfer->num_bytes        = buf_size;
    s_in_xfer->timeout_ms       = 0;   // non-blocking poll

    submit_in_transfer();
}

static void submit_in_transfer() {
    if (!s_in_xfer || !s_dev_hdl) return;
    esp_err_t err = usb_host_transfer_submit(s_in_xfer);
    if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
        Serial.printf("[USB] Transfer submit failed: %s\n", esp_err_to_name(err));
    }
}

static void on_transfer_done(usb_transfer_t *xfer) {
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
        parse_gamepad_report(xfer->data_buffer, xfer->actual_num_bytes);
    }
    submit_in_transfer();   // re-arm for the next report
}


// =============================================================================
//  Map generic HID gamepad report bytes → ControlPacket
//
//  Layout verified against the USB.ino report parser:
//
//    [0]   LX  — int8_t, 0x00 = center
//    [1]   LY  — int8_t, 0x00 = center
//    [2]   RX  — int8_t, 0x00 = center
//    [3]   RY  — int8_t, 0x00 = center
//    [4]   D-pad  (nibble, step 0x10):
//              0x00=U  0x10=UR  0x20=R  0x30=DR
//              0x40=D  0x50=DL  0x60=L  0x70=UL  0x80=neutral
//    [6]   Face / shoulder button bitmask:
//              bit0 = B1  = Cross    → BTN_CROSS
//              bit1 = B2  = Circle   → BTN_CIRCLE
//              bit2 = B3  = R2 dig.  → (mapped to BTN_R1 pair — see below)
//              bit3 = B4  = Square   → BTN_SQUARE
//              bit4 = B5  = Triangle → BTN_TRIANGLE
//              bit5 = B6  = R1       → BTN_R1
//              bit6 = B7  = L2 dig.  → (analog in [10]; digital not forwarded)
//              bit7 = B8  = L1       → BTN_L1
//    [7]   Extended button bitmask:
//              bit0 = LT-BTN  (L2 digital, duplicate of data[6] bit6)
//              bit1 = RT-BTN  (R2 digital, duplicate of data[6] bit2)
//              bit2 = OPTIONS → BTN_OPTIONS
//              bit3 = SHARE   → BTN_SHARE
//              bit4 = HOME    → BTN_PS
//              bit5 = LS      → BTN_L3
//              bit6 = RS      → BTN_R3
//    [9]   RT analog (0x00–0xFF)  → pkt.r2
//   [10]   LT analog (0x00–0xFF)  → pkt.l2
//
//  NOTE: BTN_TOUCHPAD has no equivalent on a generic gamepad and is left clear.
//  If your controller layout differs, update the byte offsets and bitmasks here.
// =============================================================================

static uint8_t s_last_report[HID_REPORT_MAX] = {0};

static void parse_gamepad_report(const uint8_t *data, size_t len) {
    if (len < 11) return;   // need at least through byte 10

    // Skip identical reports to avoid unnecessary radio traffic
    size_t cmp_len = min(len, (size_t)HID_REPORT_MAX);
    //if (memcmp(data, s_last_report, cmp_len) == 0) return;
    memcpy(s_last_report, data, cmp_len);

    pkt.leftX  = (int8_t)data[0];
    pkt.leftY  = (int8_t)data[1];
    pkt.rightX = (int8_t)data[2];
    pkt.rightY = (int8_t)data[3];

    // Triggers (Analog) [cite: 271]
    pkt.r2 = data[9];
    pkt.l2 = data[10];

    // Reset buttons and map bitmasks [cite: 263]
    pkt.buttons = 0;
    uint8_t faceBtns = data[6];
    uint8_t extBtns  = data[7];
    uint8_t dpad     = data[4];

    // Face/Shoulder (Based on USB.ino mapping)
    if (faceBtns & 0x01) pkt.buttons |= BTN_CROSS;    // B1
    if (faceBtns & 0x02) pkt.buttons |= BTN_CIRCLE;   // B2
    if (faceBtns & 0x08) pkt.buttons |= BTN_SQUARE;   // B4
    if (faceBtns & 0x10) pkt.buttons |= BTN_TRIANGLE; // B5
    if (faceBtns & 0x80) pkt.buttons |= BTN_R1;       // B8
    if (faceBtns & 0x40) pkt.buttons |= BTN_L1;       // B8
    
    // Extensions [cite: 268, 269]
    if (extBtns & 0x20) pkt.buttons |= BTN_L3;      // LS
    if (extBtns & 0x40) pkt.buttons |= BTN_R3;      // RS
    if (extBtns & 0x04) pkt.buttons |= BTN_OPTIONS; // OPTIONS
    if (extBtns & 0x08) pkt.buttons |= BTN_SHARE;   // SHARE
    if (extBtns & 0x10) pkt.buttons |= BTN_PS;

    // D-Pad [cite: 259, 260, 261]
    if (dpad == 0x00) pkt.buttons |= BTN_UP;
    if (dpad == 0x40) pkt.buttons |= BTN_DOWN;
    if (dpad == 0x60) pkt.buttons |= BTN_LEFT;
    if (dpad == 0x20) pkt.buttons |= BTN_RIGHT;

    newInput = true;
}

void OTATask(void *pvParameters) {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ArduinoOTA.setHostname("rem");  
  ArduinoOTA.setPassword("0110"); 

  ArduinoOTA.onStart([]() {
    //vTaskPrioritySet(hOTA, configMAX_PRIORITIES - 1);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
    esp_restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    // 
  });

  ArduinoOTA.begin();

  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void LCD(void *pvParameters) {
  for (;;) {
    lcdUpdateTelemetry(lastTelem);
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}


// =============================================================================
//  setup()
// =============================================================================
void setup() {
    Serial.begin(115200);
    // Note: Serial goes over UART0 (GPIO43/44), NOT over the USB-OTG port.
    // Use a USB-UART adapter on those pins if you want debug output.
    while (!Serial) {}
    delay(500);

    initTuneDefaults(
        0.55, 0.05,  0.02,    // Rate  KP / KI / KD
        1.10, 0.05,  0.02,    // Yaw   KP / KI / KD
        8.00, 0.001, 0.00,    // Angle KP / KI / KD
        0.07, 0.001, 0.0005,  // Vel   KP / KI / KD
        5.50, 1.05,  0.02);   // Alt   KP / KI / KD

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(1000000);
    lcdStatus("ESP32-S3 TX", "Initialising...", " ", " ");

    Serial.println("\n[TX] ESP32-S3 USB Gamepad → NRF24 Transmitter");

    // ── USB Host ─────────────────────────────────────────────────────────────
    s_device_ready = xSemaphoreCreateBinary();

    usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    Serial.println("[USB] Host installed");
    xTaskCreatePinnedToCore(OTATask, "OTATask", 8384, NULL, 2, NULL, 0);
    // Core 0 → USB daemon; Core 1 → USB client (same core as Arduino loop)
    xTaskCreatePinnedToCore(usb_host_task,   "usb_host",   4096, NULL,
                            USB_HOST_PRIORITY,   NULL, 0);
    xTaskCreatePinnedToCore(usb_client_task, "usb_client", 4096, NULL,
                            USB_CLIENT_PRIORITY, NULL, 0);

    // ── NRF24 ────────────────────────────────────────────────────────────────
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
    SPI.setFrequency(10000000);
    while (!radio.begin(&SPI)) {
        Serial.println("[RX] ERROR: NRF24L01 not detected. Check wiring!");
        delay(100);
        lcdStatus("NRF24 ERROR", "Check wiring!", " ", " ");
    }


    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setAutoAck(false);
    radio.setRetries(0, 0);
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.openWritingPipe(CONTROL_ADDR);
    radio.openReadingPipe(1, TELEM_ADDR);
    radio.stopListening();
    radio.powerUp();

    Serial.printf("[RF] Control pkt:   %d bytes\n", sizeof(ControlPacket));
    Serial.printf("[RF] Telemetry pkt: %d bytes\n", sizeof(TelemetryPacket));
    Serial.printf("[RF] Radio payload: %d bytes\n", RADIO_PAYLOAD_SIZE);
    Serial.println("[RF] NRF24 ready — plug in gamepad");

    lcdStatus("NRF24 ready", "Plug in gamepad", " ", " ");
    //xTaskCreatePinnedToCore(LCD, "LCD", 8384, NULL, 10, NULL, 0);
}


// =============================================================================
//  loop()
// =============================================================================
void loop() {
    // ── No gamepad connected ─────────────────────────────────────────────────
    if (s_dev_hdl == NULL) {
        static uint32_t lastBlink = 0;
        static bool     blink     = false;
        if (millis() - lastBlink >= 500) {
            lastBlink = millis();
            blink     = !blink;
            lcdStatus("No USB gamepad", blink ? "Plug in ctrl" : "            ", " ", "");
        }
        taskYIELD();
        return;
    }

    static uint32_t lastTelemReq = 0;
    bool reqTelem = (millis() - lastTelemReq >= TELEM_INTERVAL_MS);

    // ── Finalise packet ──────────────────────────────────────────────────────

    // ── Wait for a fresh HID report from parse_gamepad_report() ──────────────
  //  if (!newInput) {
   //     radio.write(&pkt, sizeof(pkt));
    //    delayMicroseconds(150);
    //    taskYIELD();
    //    return;
    //}
   //newInput = false;

    // ── Telemetry request scheduling ─────────────────────────────────────────

    pkt.flags = reqTelem ? FLAG_TELEM_REQUEST : 0;
    pkt.crc   = calcControlCRC(pkt);
    // ── Transmit ─────────────────────────────────────────────────────────────
    //radio.stopListening();

    if (reqTelem) {
    //if (false) {
        radio.flush_tx();
        radio.flush_rx();
        radio.write(&pkt, sizeof(pkt));
        //delayMicroseconds(10);

        lastTelemReq = millis();

        // Open a short RX window: the WROOM detects FLAG_TELEM_REQUEST,
        // flips to TX, and fires a TelemetryPacket back.
        radio.flush_tx();
        radio.flush_rx();
        radio.startListening();
        //delayMicroseconds(10);

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
                } else {
                    Serial.println("[TELEM] CRC mismatch — discarded");
                }
                if (gotTelem) break;
            }
            if (gotTelem) break;
        }

        if (!gotTelem) {
            static uint32_t missCount = 0;
            Serial.printf("[TELEM] Missed window #%lu\n", ++missCount);
        }

        radio.stopListening();
        delayMicroseconds(10);
        radio.flush_rx();
        radio.flush_tx();

    } else {
        // Normal cycle — fire and forget
        radio.write(&pkt, sizeof(pkt));
        //delayMicroseconds(10);
    }

    // ── PID menu (DPad + buttons on LCD) ─────────────────────────────────────
    if (newInput) {
        handleMenuInput();
        newInput = false;
    }
     //── LCD telemetry update ────────────────────────────
    static uint32_t lastLCD = 0;
    if (millis() - lastLCD >= 50) {
        lastLCD = millis();
        lcdUpdateTelemetry(lastTelem);
    }
    
    radio.flush_rx();
    radio.flush_tx();
    //taskYIELD();
    vTaskDelay(1);
    //delayMicroseconds(1000); 
}

