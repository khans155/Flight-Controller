#include "src/config.h"
#include "src/sensors.h"
#include "src/motors.h"
#include "src/comms.h"
#include "src/optical_flow.h"

TaskHandle_t hSensor, hIO, hComms;

//hw_timer_t *sensorTimer     = NULL;
hw_timer_t *lidarTimer      = NULL;
hw_timer_t *controllerTimer = NULL;

//volatile bool sensorFlag = false;
volatile bool lidarFlag = false;
volatile bool controllerFlag = false;
volatile uint64_t g_execTimeUS = 0;

//void IRAM_ATTR onSensorTimer(){ sensorFlag = true; }
void IRAM_ATTR onLidarTimer(){ lidarFlag = true; }
void IRAM_ATTR onControllerTimer(){ controllerFlag = true; }

//============================ Tasks =====================================

// ─── Core 1: HIGHEST priority - Sensors, Filter, Mix  ───────────────────

void SensorTask(void *pvParameters) {
const uint64_t xFrequencyUS = 500; 
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
  for (;;) {
    if (lidarFlag && !newLidar) {
      lidarFlag = false;
      float h, x, y; uint8_t q;
      dataLidarFlow(h, x, y, q);
      height_lidar = h; vx = x; vy = y; flow_quality = q;
      newLidar = true;
    }

    if (controllerFlag) {
      controllerFlag = false;
      readPS5();
    }

    vTaskDelay(1); 
  }
}

// ─── Core 0: LOW priority - Comms, Telemetry ────────────────────────────────────────

void CommsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  initWiFi();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 20Hz telemetry

   // remotePrint("Stack remaining (words):"); remotePrint("  Sensor:     %d", uxTaskGetStackHighWaterMark(hSensor));
    //remotePrint("  IO:      %d", uxTaskGetStackHighWaterMark(hIO)); remotePrint("  Comms:       %d\n", uxTaskGetStackHighWaterMark(hComms));
    //remotePrint("vx_est: %.2f, vy_est: %.2f, vx:%.6f, vy:%.6f, height: %0.6f \r\n", vx_est, vy_est, vx_f, vy_f, height_filtered* cosf(roll_ag* DEG_TO_RAD) * cosf(pitch_ag* DEG_TO_RAD)*100.0f);
    remotePrint("Pitch: %.2f, Roll: %.2f, gx_f:%.6f, gy_f:%.6f, gz_f: %0.6f ", pitch_ag, roll_ag, gx_f, gy_f, gz_f);
    remotePrint("gyroX: %d, gyro2X: %d, gz_f:%.6f, accY:%d, acc2Y: %d exec: %llu us\r\n", gyroX, gyro2X, gz_f, accY, acc2Y, g_execTimeUS);
    //remotePrint("   M1:%d M2:%d M3:%d M4:%d  off:%d %d %d %d\r\n", m1_corr, m2_corr, m3_corr, m4_corr, m1, m2, m3, throttle);
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
    vTaskDelay(pdMS_TO_TICKS(100));  // print at 2Hz — adjust as needed
  }
}

//======================== Setup =====================================

void setup() {
  xTaskCreatePinnedToCore(CommsTask, "CommsTask", 10096, NULL, 3, &hComms,  0);     
  vTaskDelay(1000);

  Serial.begin(115200);
  while (!Serial) { vTaskDelay(100); }

  initSensors();
  sensorSerial.begin(115200, SERIAL_8N1, RX_MTF, TX_MTF);
  memset(&msg, 0, sizeof(msg));
  initFilter();
  initMotors("4c:b9:9b:0c:2b:63");
  //initMotors("14:3a:9a:39:3b:b7");

  hw_timer_t *t1 = timerBegin(1, 80, true);
  timerAttachInterrupt(t1, &onLidarTimer, true);
  timerAlarmWrite(t1, 10000, true);   // 10ms
  timerAlarmEnable(t1);

  hw_timer_t *t2 = timerBegin(2, 80, true);
  timerAttachInterrupt(t2, &onControllerTimer, true);
  timerAlarmWrite(t2, 20000, true);   // 20ms
  timerAlarmEnable(t2);




  disableCore1WDT();
  disableCore0WDT();
  xTaskCreatePinnedToCore(SensorTask, "SensorTask",  10000, NULL, configMAX_PRIORITIES - 1, &hSensor, 1);
  xTaskCreatePinnedToCore(IOTask,     "IOTask",      4096,  NULL, 6,                        &hIO,     0);
  //xTaskCreatePinnedToCore(PrintTask,"PrintTask", 4096, NULL, 8, NULL, 0);

}

void loop() {
  vTaskDelete(NULL);
}