#include <ArduinoOTA.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_debug_helpers.h"
#include "esp_attr.h"
#include "esc_4way.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "comms.h"
#include "optical_flow.h"
#include "gps.h"

TaskHandle_t hSensor, hIO, hComms, hOTA;
volatile uint32_t g_execTimeUS = 0;

//============================ Tasks =====================================

// ─── Core 1: HIGHEST priority - Sensors, Filter, Mix  ───────────────────

void IRAM_ATTR SensorTask(void *pvParameters) {
    const uint64_t periodUs = IMU_PERIOD; // 
    uint64_t nextWake = esp_timer_get_time() + periodUs;

    for (;;) {
        int64_t waitUs = (int64_t)(nextWake - esp_timer_get_time());

        if (waitUs > 0) {
            delayMicroseconds((uint32_t)waitUs);
        }

        uint64_t start = esp_timer_get_time();

        // If badly late, don't run catch-up loops
        if ((int64_t)(start - nextWake) > (int64_t)periodUs) {
            nextWake = start + periodUs;
        } else {
            nextWake += periodUs;
        }

        updateFilter(start);
        mixMotor(start);

        g_execTimeUS = esp_timer_get_time() - start;
    }
}

// ─── Core 0: HIGH priority - Inputs, Slow Sensors ──────────────────────────────

void IOTask(void *pvParameters) {

    static float battValue = 0;
    static int battCount = 0;

    uint32_t lastLidarUs = micros();
    uint32_t lastBaroUs  = micros();
    uint32_t lastGpsUs   = micros();
    uint32_t lastLedUs = micros();

    const uint32_t lidarPeriod = 10000;   // 100 Hz
    const uint32_t baroPeriod  = 40000;   // 25 Hz
    const uint32_t gpsPeriod   = 200000;  // 5 Hz
    const uint32_t ledPeriod = 50000; // 20Hz

    batt_rx = analogReadMilliVolts(BAT_ADC_PIN) * 1.25587f;


    for (;;) {
        uint32_t now = micros();
        // ───── LIDAR + FLOW @ 100Hz ─────
        if ((uint32_t)(now - lastLidarUs) >= lidarPeriod) {
            lastLidarUs += lidarPeriod;
            if (!newLidar) {
                float h, x, y;
                uint8_t q;

                dataLidarFlow(h, x, y, q);
                height_lidar = h;
                vx = x;
                vy = y;
                flow_quality = q;
                //newLidar = true;
            }
        }
        // ───── BARO @ 25Hz ─────
        if ((uint32_t)(now - lastBaroUs) >= baroPeriod) {
            lastBaroUs += baroPeriod;
            baroFlag = true;
            // 
            // readBarometer();
        }
        // ───── GPS @ 5Hz ─────
        if ((uint32_t)(now - lastGpsUs) >= gpsPeriod) {
            lastGpsUs += gpsPeriod;
           // getLocationGPS();
        }
        if ((uint32_t)(now - lastLedUs) >= ledPeriod) {
            lastLedUs += ledPeriod;
            battValue += analogReadMilliVolts(BAT_ADC_PIN) * 1.25587f;
            battCount += 1;

            if (battCount >= 250){
              batt_rx = (int)(battValue/battCount);
              battCount = 0;
              battValue = 0;
            }

            if (armed) {
                if(throttleLockActive & !altHoldActive){
                  argb_set_color(255, 100, 0);
                }
                else if(!throttleLockActive & altHoldActive){
                  argb_set_color(0, 100, 255);
                }
                else if(throttleLockActive & altHoldActive){
                  argb_set_color(255, 100, 255);
                }
                else {
                  argb_set_color(0, 255, 0);
                }
            } else {
                argb_set_color(255, 0, 255);
            }
        }

        // ───── RADIO / CONTROLS ─────
        readPS5();
        vTaskDelay(1);
        //taskYIELD();
    }
}

// ─── Core 0: LOW priority - Comms, Telemetry, OTA ────────────────────────────────────────

void CommsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 20Hz debug telemetry

   // remotePrint("Stack remaining (words):"); remotePrint("  Sensor:     %d", uxTaskGetStackHighWaterMark(hSensor));
    //remotePrint("  IO:      %d", uxTaskGetStackHighWaterMark(hIO)); remotePrint("  Comms:       %d\n", uxTaskGetStackHighWaterMark(hComms));
    //remotePrint("vx_est: %.2f, vy_est: %.2f, vx:%.6f, vy:%.6f, height: %0.6f \r\n", vx_est, vy_est, vx_f, vy_f, height_filtered* cosf(roll_ag* DEG_TO_RAD) * cosf(pitch_ag* DEG_TO_RAD)*100.0f);
    remotePrint("Pitch: %.2f, Roll: %.2f, height_lidar: %0.6f, batt_rx: %d ", pitch_ag, roll_ag, height_lidar, batt_rx);

    //if (fixGPS){
      //remotePrint("Long: %.6f, Lat: %.6f, exec: %llu us\r\n", longitude, latitude, g_execTimeUS);
    //} else {
      //remotePrint("No GPS Fix, exec: %llu us\r\n", g_execTimeUS);
    //}
    remotePrint("   M1:%d M2:%d M3:%d M4:%d  off:%d %d %d exec: %lu\r\n", m1_corr, m2_corr, m3_corr, m4_corr, m1, m2, m3, (unsigned long)g_execTimeUS);
    //remotePrint("   X: %d  Y: %d  Z: %d", magX, magY, magZ); remotePrint("   X: %.4f  Y: %.4f  Z: %.4f", mx_f, my_f, mz_f);
    //remotePrint("   X: %d  Y: %d  Z: %d", accX, accY, accZ); remotePrint("   X: %.4f  Y: %.4f  Z: %.4f \r\n", ax_f, ay_f, az_f);

    if (telnetServer.hasClient()) {
      if (!telnetClient || !telnetClient.connected()) {
        if (telnetClient) telnetClient.stop();
        telnetClient = telnetServer.available();
        remotePrint("Computer Connected!\r\n");
      } else {
        telnetServer.available().stop();
      }
    }
    handleRemoteCommands();
  }
}

void PrintTask(void *pvParameters) {
  for (;;) {
    Serial.printf("exec: %llu us\n", g_execTimeUS);
    vTaskDelay(pdMS_TO_TICKS(100));  
  }
}

void OTATask(void *pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ArduinoOTA.setHostname("quad");  
  ArduinoOTA.setPassword("0110"); 

  ArduinoOTA.onStart([]() {
    //vTaskPrioritySet(hOTA, configMAX_PRIORITIES - 1);
    emergencyMotorStop(); 
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

// ─── Safety Hooks ────────────────────────────────────────
void motorShutdownCallback() {
  emergencyMotorStop();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  emergencyMotorStop();
  esp_restart(); 
}

void esp_task_wdt_isr_user_handler(void) {
  emergencyMotorStop();
}

void safeRestart(const char* reason = "") {
  emergencyMotorStop();
  esp_restart();
}

//======================== Setup =====================================


void setup() {
  esp_register_shutdown_handler(motorShutdownCallback);
  //disableCore1WDT();
  //disableCore0WDT();

  Serial.begin(115200);
  vTaskDelay(100);

  #if ESC_PASSTHROUGH_MODE
    // Binary 4-way interface mode. Do not start WiFi, sensors, RMT/DShot,
    // radio, OTA, or any flight tasks while the ESC tool is connected.
    esc4wayRun();
    return;
  #endif

  initWiFi();
  vTaskDelay(700);
  
  initSensors();
  
  gpsSerial.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  sensorSerial.begin(115200, SERIAL_8N1, RX_MTF, TX_MTF);
  memset(&msg, 0, sizeof(msg));
  initMotors();
  
  initFilter();
  pinMode(BAT_ADC_PIN, INPUT);
  
  xTaskCreatePinnedToCore(CommsTask, "CommsTask", 8096, NULL, 1, &hComms,  0);     
  xTaskCreatePinnedToCore(SensorTask, "SensorTask",  30384, NULL, configMAX_PRIORITIES - 1, &hSensor, 1);
  xTaskCreatePinnedToCore(IOTask,     "IOTask",      10384,  NULL, 10,                        &hIO,     0);
  xTaskCreatePinnedToCore(OTATask, "OTATask", 8384, NULL, 2, &hOTA, 0);
  //xTaskCreatePinnedToCore(PrintTask,"PrintTask", 4096, NULL, 8, NULL, 0);

}

void loop() {
  vTaskDelete(NULL);
}