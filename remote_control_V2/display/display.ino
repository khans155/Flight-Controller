#include <Arduino.h>
#include <WiFi.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <ArduinoOTA.h>

#include "i2c_display.h"
#include "ui.h"
#include "screens.h"

#define WIFI_SSID     "WuTangLan"
#define WIFI_PASSWORD "W0rkpl@y$le3p"

#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320

#define CYD_I2C_ADDR 0x42

#define SLAVE_SDA 32
#define SLAVE_SCL 25

TFT_eSPI tft = TFT_eSPI();

volatile bool newTelemData = false;
volatile bool newTextData = false;
DisplayPacket latestTelem;
DisplayPacket latestText;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 20];

static int clampInt(int value, int minValue, int maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

// Converts a centered angle value into the 0-100 slider range used by the new UI.
// The UI sliders are centered at 50. Change angleLimit if your roll/pitch range is different.
static int angleToSliderPercent(float angle, float angleLimit = 45.0f)
{
    float pct = ((angle + angleLimit) * 100.0f) / (angleLimit * 2.0f);
    return clampInt((int)lroundf(pct), 0, 100);
}

// Accepts either 1000-2000 style throttle, 0-100 percent, or 0.0-1.0 normalized throttle.
static int throttleToPercent(float throttle)
{
    float pct;

    if (throttle > 100.0f) {
        pct = ((throttle - 1000.0f) / 1000.0f) * 100.0f;
    } else if (throttle <= 1.0f) {
        pct = throttle * 100.0f;
    } else {
        pct = throttle;
    }

    return clampInt((int)lroundf(pct), 0, 100);
}

static void setLabel(lv_obj_t *label, const char *text)
{
    if (label != nullptr) {
        lv_label_set_text(label, text);
    }
}

static void setBar(lv_obj_t *bar, int value)
{
    if (bar != nullptr) {
        lv_bar_set_value(bar, clampInt(value, 0, 100), LV_ANIM_OFF);
    }
}

static void setSlider(lv_obj_t *slider, int value)
{
    if (slider != nullptr) {
        lv_slider_set_value(slider, clampInt(value, 0, 100), LV_ANIM_OFF);
    }
}

void OTATask(void *pvParameters)
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ArduinoOTA.setHostname("remdis");
    ArduinoOTA.setPassword("0110");

    ArduinoOTA.onStart([]() {
        // vTaskPrioritySet(hOTA, configMAX_PRIORITIES - 1);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
        esp_restart();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]\n", error);
    });

    ArduinoOTA.begin();

    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void my_disp_flush(lv_disp_drv_t *disp,
                   const lv_area_t *area,
                   lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

void receiveEvent(int howMany)
{
    if (howMany == sizeof(DisplayPacket)) {
        DisplayPacket pkt;
        Wire.readBytes((uint8_t *)&pkt, sizeof(DisplayPacket));

        if (pkt.type == PACKET_TELEM) {
            latestTelem = pkt;
            newTelemData = true;
        } else if (pkt.type == PACKET_TEXT) {
            latestText = pkt;
            newTextData = true;
        }
    } else {
        Serial.println("packet size mismatch");
        while (Wire.available()) Wire.read();
    }
}


static void normalizeStickSlider(lv_obj_t *slider, int x, int y, int w, int h)
{
    if (slider == nullptr) return;

    // Do not use PicoPixel's rotation transform for these indicators on the ESP32.
    // LVGL's transform keeps the original object box, so rotated sliders can look clipped
    // or shifted on real hardware.
    lv_obj_set_pos(slider, x, y);
    lv_obj_set_size(slider, w, h);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);

    lv_obj_set_style_transform_angle(slider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_pivot_x(slider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_pivot_y(slider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_angle(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_pivot_x(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_pivot_y(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(slider, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(slider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(slider, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(slider, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(slider, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(slider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Hide the filled section so the white knob acts like the stick-position marker.
    lv_obj_set_style_bg_opa(slider, LV_OPA_TRANSP, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(slider, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(slider, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(slider, lv_color_hex(0xffffff), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_width(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_height(slider, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
}

static void fixStickSliders()
{
    // Roll is a normal horizontal indicator at the bottom-left.
    normalizeStickSlider(objects.roll, 17, 262, 202, 21);

    // Pitch should be a real vertical LVGL slider, not a rotated horizontal slider.
    // LVGL chooses vertical orientation automatically when height > width.
    normalizeStickSlider(objects.pitch, 230, 35, 21, 202);

    // Yaw uses the same rotated style in the generated UI, so normalize it too.
    normalizeStickSlider(objects.yaw, 263, 262, 202, 21);
}

void configure_generated_gui()
{
    // The new PicoPixel UI creates all objects inside ui_init().
    // These are only runtime defaults/cleanup after the generated screen loads.
    setLabel(objects.label_menu1, "Awaiting connection...");
    setLabel(objects.label_1, "");
    setLabel(objects.label_2, "");
    setLabel(objects.label_3, "");

    setLabel(objects.roll_1, "ROLL: 0.0");
    setLabel(objects.pitch_1, "PITCH: 0.0");
    setLabel(objects.alt, "ALT: 0.00m");
    setLabel(objects.throttle, "THR: 0%");

    setSlider(objects.roll, 50);
    setSlider(objects.pitch, 50);
    setSlider(objects.yaw, 50);

    fixStickSliders();

    setBar(objects.rx_bar, 50);
    setBar(objects.tx_bar, 50);

    if (objects.spinbox_1 != nullptr) {
        lv_spinbox_set_value(objects.spinbox_1, 0);
    }
}

void update_gui()
{
    if (newTelemData) {
        newTelemData = false;

        char text[40];

        float roll = latestTelem.data.telem.roll;
        float pitch = latestTelem.data.telem.pitch;
        float altitude = latestTelem.data.telem.alt;
        int throttlePct = (((latestTelem.data.telem.throttle*100.0f - 1000.0f)/1000.0f)*100.0f);
        int rxBatteryPercent = constrain(map((int)(latestTelem.data.telem.batt_rx * 100.0f), 960, 1260, 0, 100),0, 100);
        int txBatteryPercent = constrain(map((int)(latestTelem.data.telem.batt_tx * 10000.0f), 640, 840, 0, 100),0, 100);

        snprintf(text, sizeof(text), "ROLL: %.1f", roll); setLabel(objects.roll_1, text);
        snprintf(text, sizeof(text), "PITCH: %.1f", pitch); setLabel(objects.pitch_1, text);
        snprintf(text, sizeof(text), "ALT: %.2fm", altitude); setLabel(objects.alt, text);
        snprintf(text, sizeof(text), "THR: %d%%", throttlePct); setLabel(objects.throttle, text);

        setSlider(objects.roll, angleToSliderPercent(roll));
        setSlider(objects.pitch, angleToSliderPercent(pitch));

        snprintf(text, sizeof(text), "RX %d%%  %d mV", rxBatteryPercent, (int)(latestTelem.data.telem.batt_rx * 100.0f)); setLabel(objects.rx, text);
        snprintf(text, sizeof(text), "TX %d%%", txBatteryPercent); setLabel(objects.tx, text);
        lv_bar_set_value(objects.rx_bar, rxBatteryPercent, LV_ANIM_OFF);
        lv_bar_set_value(objects.tx_bar, txBatteryPercent, LV_ANIM_OFF);

        if (objects.spinbox_1 != nullptr) {
            lv_spinbox_set_value(objects.spinbox_1, throttlePct * 10);
        }

        if (WiFi.status() != WL_CONNECTED){
            lv_obj_set_style_img_recolor_opa(objects.image_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_img_recolor(objects.image_2, lv_color_hex(0), LV_PART_MAIN | LV_STATE_DEFAULT);
            setLabel(objects.dbm, "");
        } else {
            lv_obj_set_style_img_recolor_opa(objects.image_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_img_recolor(objects.image_2, lv_color_hex(0x00f207), LV_PART_MAIN | LV_STATE_DEFAULT);
            snprintf(text, sizeof(text), "%d dBm", WiFi.RSSI()); setLabel(objects.dbm, text);
        }

        if (latestTelem.data.telem.state == 0){ //unarmed
            lv_obj_set_style_img_recolor_opa(objects.image_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_img_recolor(objects.image_1, lv_color_hex(0xff00ff), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        else {
            if (latestTelem.data.telem.state == 3){ //throttle lock active
                lv_obj_set_style_img_recolor_opa(objects.image_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_img_recolor(objects.image_1, lv_color_hex(0xff6400), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (latestTelem.data.telem.state == 2){ //alt hold active
                lv_obj_set_style_img_recolor_opa(objects.image_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_img_recolor(objects.image_1, lv_color_hex(0x0064ff), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else{ //armed
                lv_obj_set_style_img_recolor_opa(objects.image_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_img_recolor(objects.image_1, lv_color_hex(0x00ff00), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
        }

    }

    if (newTextData) {
        newTextData = false;

        // Old GUI had two menu lines. The new terminal box has four text lines.
        setLabel(objects.label_menu1, latestText.data.text.line1);
        setLabel(objects.label_1, latestText.data.text.line2);
        setLabel(objects.label_2, latestText.data.text.line3);
        setLabel(objects.label_3, latestText.data.text.line4);
    }
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(CYD_I2C_ADDR, SLAVE_SDA, SLAVE_SCL, 1000000);
    Wire.onReceive(receiveEvent);

    lv_init();

    tft.begin();
    tft.setRotation(1);

    lv_disp_draw_buf_init(&draw_buf,
                          buf,
                          NULL,
                          SCREEN_WIDTH * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);

    // Replaces the old hand-built create_gui() with the new PicoPixel UI.
    ui_init();
    configure_generated_gui();

    xTaskCreatePinnedToCore(OTATask, "OTATask", 8384, NULL, 2, NULL, 0);
}

void loop()
{
    lv_timer_handler();
    ui_tick();

    update_gui();

    vTaskDelay(1);
}
