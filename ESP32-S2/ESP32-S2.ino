#include <ArduinoOTA.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_debug_helpers.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "comms.h"
#include "optical_flow.h"
#include "gps.h"

TaskHandle_t hSensor, hIO, hComms, hOTA;


hw_timer_t *lidarTimer      = NULL;
hw_timer_t *controllerTimer = NULL;

volatile bool lidarFlag = false;
volatile bool controllerFlag = false;
volatile uint64_t g_execTimeUS = 0;

void IRAM_ATTR onLidarTimer(){ lidarFlag = true; }
void IRAM_ATTR onControllerTimer(){ controllerFlag = true; }

//============================ Tasks =====================================

// ─── Core 1: HIGHEST priority - Sensors, Filter, Motor Mix  ───────────────────

void SensorTask(void *pvParameters) {
const uint64_t xFrequencyUS = 1000; 
    uint64_t xLastWakeTime = esp_timer_get_time();

    for (;;) {
        while ((esp_timer_get_time() - xLastWakeTime) < xFrequencyUS) {
            delayMicroseconds(1); 
        }
        xLastWakeTime += xFrequencyUS;

        uint64_t start = esp_timer_get_time();
        updateFilter();
        mixMotor();
        g_execTimeUS = esp_timer_get_time() - start; 
    }
}

// ─── Core 0: HIGH priority - Inputs, Slow Sensors ──────────────────────────────

void IOTask(void *pvParameters) {
  static uint8_t baroDiv = 0;
  static uint8_t gpsDiv = 0;

  for (;;) {
    if (lidarFlag && !newLidar) { //Lidar and flow sensor sampled at 100 Hz
      lidarFlag = false;
      float h, x, y; uint8_t q;
      dataLidarFlow(h, x, y, q);
      height_lidar = h; vx = x; vy = y; flow_quality = q;
      newLidar = true;

      if (++baroDiv >= 4) { //Baro sampled at 25 Hz
        baroDiv = 0;
        baroFlag = true;
      }
      if (++gpsDiv >= 20) { //GPS sampled at 5 Hz
        gpsDiv = 0;
        getLocationGPS();
      }

    }
    readPS5(); // Remote control read at 1 kHz
    vTaskDelay(1); 
  }
}

// ─── Core 0: LOW priority - Comms, Telemetry, OTA ────────────────────────────────────────

void CommsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  initWiFi();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 20Hz debug telemetry
    remotePrint("Pitch: %.2f, Roll: %.2f, height_lidar: %0.6f, height_baro: %0.6f ", pitch_ag, roll_ag, height_lidar, height_baro);
    remotePrint("   M1:%d M2:%d M3:%d M4:%d  off:%d %d %d exec: %llu\r\n", m1_corr, m2_corr, m3_corr, m4_corr, m1, m2, m3, g_execTimeUS);
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

void OTATask(void *pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  ArduinoOTA.setHostname("quad");  
  ArduinoOTA.setPassword("0110"); 

  ArduinoOTA.onStart([]() {
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
  disableCore1WDT();
  disableCore0WDT();

  xTaskCreatePinnedToCore(CommsTask, "CommsTask", 10096, NULL, 3, &hComms,  0);     
  vTaskDelay(1000);
  Serial.begin(115200);
  while (!Serial) { vTaskDelay(100); }

  initSensors();
  gpsSerial.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  sensorSerial.begin(115200, SERIAL_8N1, RX_MTF, TX_MTF);
  memset(&msg, 0, sizeof(msg));

  initMotors();
  initFilter();


  hw_timer_t *t1 = timerBegin(1, 80, true);
  timerAttachInterrupt(t1, &onLidarTimer, true);
  timerAlarmWrite(t1, 10000, true);  
  timerAlarmEnable(t1);

  hw_timer_t *t2 = timerBegin(2, 80, true);
  timerAttachInterrupt(t2, &onControllerTimer, true);
  timerAlarmWrite(t2, 10000, true);   
  timerAlarmEnable(t2);

  xTaskCreatePinnedToCore(SensorTask, "SensorTask",  20000, NULL, configMAX_PRIORITIES - 1, &hSensor, 1);
  xTaskCreatePinnedToCore(IOTask,     "IOTask",      10096,  NULL, 6,                        &hIO,     0);
  xTaskCreatePinnedToCore(OTATask, "OTATask", 8192, NULL, 4, &hOTA, 0);

}

void loop() {
  vTaskDelete(NULL);
}