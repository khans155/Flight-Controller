#include <Arduino.h>
#include "config.h"
#include "esc_4way.h"
#include "esp_task_wdt.h"

#ifndef ESC_PASSTHROUGH_MODE
#define ESC_PASSTHROUGH_MODE 0
#endif

#if ESC_PASSTHROUGH_MODE

// -----------------------------------------------------------------------------
// Direct BLHeli / 4-way interface for ESP32-S3
// -----------------------------------------------------------------------------
// This implements the Arduino-style 4-way interface directly on USB Serial.
// In BLHeliSuite, use the 4way-if interface; classic BLHeliSuite normally fixes the host baud at 38400.
//
// Notes:
// - This is intentionally only compiled in passthrough mode.
// - The ESC signal line is used as a 19200 baud half-duplex 1-wire serial line.
// - Remove props and power the ESC from a current-limited supply while testing.
// -----------------------------------------------------------------------------

// -------------------- 4-way protocol constants --------------------
static constexpr uint8_t cmd_Remote_Escape        = 0x2E; // '.' response
static constexpr uint8_t cmd_Local_Escape         = 0x2F; // '/' request

static constexpr uint8_t cmd_InterfaceTestAlive   = 0x30;
static constexpr uint8_t cmd_ProtocolGetVersion   = 0x31;
static constexpr uint8_t cmd_InterfaceGetName     = 0x32;
static constexpr uint8_t cmd_InterfaceGetVersion  = 0x33;
static constexpr uint8_t cmd_InterfaceExit        = 0x34;
static constexpr uint8_t cmd_DeviceReset          = 0x35;
static constexpr uint8_t cmd_DeviceInitFlash      = 0x37;
static constexpr uint8_t cmd_DevicePageErase      = 0x39;
static constexpr uint8_t cmd_DeviceRead           = 0x3A;
static constexpr uint8_t cmd_DeviceWrite          = 0x3B;
static constexpr uint8_t cmd_DeviceReadEEprom     = 0x3D;
static constexpr uint8_t cmd_DeviceWriteEEprom    = 0x3E;
static constexpr uint8_t cmd_InterfaceSetMode     = 0x3F;
static constexpr uint8_t cmd_DeviceVerify         = 0x40;

static constexpr uint8_t ACK_OK                   = 0x00;
static constexpr uint8_t ACK_I_UNKNOWN_ERROR      = 0x01;
static constexpr uint8_t ACK_I_INVALID_CMD        = 0x02;
static constexpr uint8_t ACK_I_INVALID_CRC        = 0x03;
static constexpr uint8_t ACK_I_INVALID_CHANNEL    = 0x08;
static constexpr uint8_t ACK_I_INVALID_PARAM      = 0x09;
static constexpr uint8_t ACK_D_VERIFY_ERROR       = 0x04;
static constexpr uint8_t ACK_D_GENERAL_ERROR      = 0x0F;

// Interface modes used by BLHeliSuite / ESC configurators.
static constexpr uint8_t imC2       = 0;
static constexpr uint8_t imSIL_BLB  = 1; // BLHeli_S / SiLabs bootloader
static constexpr uint8_t imATM_BLB  = 2;
static constexpr uint8_t imSK       = 3;
static constexpr uint8_t imARM_BLB  = 4; // BLHeli_32 / ARM bootloader

// Version 20.0.05 is accepted by newer BLHeli tools.
static constexpr uint8_t SERIAL_4WAY_PROTOCOL_VER = 107;
static constexpr uint8_t SERIAL_4WAY_VERSION_HI   = 200; // 20.0.05 -> 200, 5
static constexpr uint8_t SERIAL_4WAY_VERSION_LO   = 13;

// BLHeli bootloader commands/result codes.
static constexpr uint8_t RestartBootloader        = 0x00;
static constexpr uint8_t ExitBootloader           = 0x01;
static constexpr uint8_t CMD_PROG_FLASH           = 0x01;
static constexpr uint8_t CMD_ERASE_FLASH          = 0x02;
static constexpr uint8_t CMD_READ_FLASH_SIL       = 0x03;
static constexpr uint8_t CMD_READ_EEPROM          = 0x04;
static constexpr uint8_t CMD_PROG_EEPROM          = 0x05;
static constexpr uint8_t CMD_KEEP_ALIVE           = 0xFD;
static constexpr uint8_t CMD_SET_BUFFER           = 0xFE;
static constexpr uint8_t CMD_SET_ADDRESS          = 0xFF;
static constexpr uint8_t brSUCCESS                = 0x30;
static constexpr uint8_t brERRORVERIFY            = 0xC0;
static constexpr uint8_t brERRORCOMMAND           = 0xC1;
static constexpr uint8_t brERRORCRC               = 0xC2;
static constexpr uint8_t brNONE                   = 0xFF;

// -------------------- MSP v1 constants used by ESC/Bluejay Configurator --------------------
static constexpr uint8_t MSP_API_VERSION          = 1;
static constexpr uint8_t MSP_FC_VARIANT           = 2;
static constexpr uint8_t MSP_FC_VERSION           = 3;
static constexpr uint8_t MSP_BOARD_INFO           = 4;
static constexpr uint8_t MSP_BUILD_INFO           = 5;
static constexpr uint8_t MSP_NAME                 = 10;
static constexpr uint8_t MSP_FEATURE_CONFIG       = 36;
static constexpr uint8_t MSP_REBOOT               = 68;
static constexpr uint8_t MSP_ADVANCED_CONFIG      = 90;
static constexpr uint8_t MSP_STATUS               = 101;
static constexpr uint8_t MSP_MOTOR                = 104;
static constexpr uint8_t MSP_MOTOR_CONFIG         = 131;
static constexpr uint8_t MSP_UID                  = 160;
static constexpr uint8_t MSP_SET_4WAY_IF          = 245;

// Forward declarations. Arduino generates prototypes for .ino files,
// but not reliably for normal .cpp files, so helpers must be declared
// before any earlier function calls them.
static uint8_t mapBootAckTo4WayAck(int bootAck);

#ifndef ESC_4WAY_DEFAULT_MODE
#define ESC_4WAY_DEFAULT_MODE imSIL_BLB
#endif

#ifndef ESC_PASSTHROUGH_DIRECT_4WAY
#define ESC_PASSTHROUGH_DIRECT_4WAY 1
#endif

#ifndef ESC_PASSTHROUGH_MSP_4WAY
#define ESC_PASSTHROUGH_MSP_4WAY 2
#endif

#ifndef ESC_PASSTHROUGH_PROTOCOL
#define ESC_PASSTHROUGH_PROTOCOL ESC_PASSTHROUGH_DIRECT_4WAY
#endif

#ifndef ESC_4WAY_HOST_BAUD
#define ESC_4WAY_HOST_BAUD 38400
#endif

#ifndef ESC_MSP_HOST_BAUD
#define ESC_MSP_HOST_BAUD 115200
#endif

#ifndef ESC_4WAY_ESC_BAUD
#define ESC_4WAY_ESC_BAUD 19200
#endif

#ifndef ESC_4WAY_DEBUG
#define ESC_4WAY_DEBUG 0
#endif

// Betaflight/Cleanflight drives the ESC line push-pull while transmitting,
// then switches back to input pull-up for receive. Open-drain can work with
// an external pull-up, but on ESP32 the rise time can be too slow/noisy.
#ifndef ESC_4WAY_TX_OPEN_DRAIN
#define ESC_4WAY_TX_OPEN_DRAIN 0
#endif

// Betaflight/BLHeli 4-way uses CMD_KEEP_ALIVE as a 2-byte bootloader command
// {0xFD, 0x00} plus CRC and expects brERRORCOMMAND. Keeping all ESCs alive is safer with BLHeliSuite because it can leave
// several ESC bootloaders open while the COM port is idle between operations.
// Some BLHeliSuite versions close the interface unless keepalive is very frequent.
// v14 avoids write corruption by servicing at most one ESC per keepalive slice,
// never while host bytes are waiting, and suppressing keepalive around flash writes.
#ifndef ESC_4WAY_KEEPALIVE_MS
#define ESC_4WAY_KEEPALIVE_MS 1
#endif

#ifndef ESC_4WAY_KEEPALIVE_ALL_ESC
#define ESC_4WAY_KEEPALIVE_ALL_ESC 1
#endif

#ifndef ESC_4WAY_FLASH_QUIET_MS
#define ESC_4WAY_FLASH_QUIET_MS 80
#endif

static const uint8_t escPins4Way[4] = {
  MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN
};

static int activeEsc = 0;
static int activePin = MOTOR1_PIN;
static uint8_t activeMode = ESC_4WAY_DEFAULT_MODE;
static bool escBootloaderActive[4] = {false, false, false, false};
static uint32_t lastEscKeepAliveMs = 0;
static uint32_t suppressKeepAliveUntilMs = 0;
static uint8_t keepAliveRoundRobinEsc = 0;

static constexpr uint32_t ESC_BIT_US = 1000000UL / ESC_4WAY_ESC_BAUD;

// The ESC side is a bit-banged 19200-baud software UART. A short USB/FreeRTOS
// interrupt in the middle of one byte can corrupt writes. Keep interrupts off
// only while shifting/sampling a single byte (~520 us), not during whole blocks.
static portMUX_TYPE escUartMux = portMUX_INITIALIZER_UNLOCKED;

// -------------------- CRC helpers --------------------
static uint16_t crcXmodemUpdate(uint16_t crc, uint8_t data) {
  crc ^= ((uint16_t)data << 8);
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else              crc <<= 1;
  }
  return crc;
}

static uint16_t escByteCrc(uint8_t data, uint16_t crc) {
  for (uint8_t i = 0; i < 8; i++) {
    if (((data & 0x01) ^ (crc & 0x0001)) != 0) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc >>= 1;
    }
    data >>= 1;
  }
  return crc;
}

// -------------------- 1-wire ESC serial --------------------
static inline void escReleaseLine() {
  pinMode(activePin, INPUT_PULLUP);
}

static inline void escTxMode() {
#if ESC_4WAY_TX_OPEN_DRAIN
  pinMode(activePin, OUTPUT_OPEN_DRAIN);
#else
  pinMode(activePin, OUTPUT);
#endif
}

static inline void escDriveLow() {
  digitalWrite(activePin, LOW);
  escTxMode();
}

static inline void escDriveHigh() {
  digitalWrite(activePin, HIGH);
  escTxMode();
}

static inline bool escReadLine() {
  return digitalRead(activePin) != LOW;
}

static void escSelect(uint8_t esc) {
  if (esc > 3) esc = 0;
  activeEsc = esc;
  activePin = escPins4Way[esc];
  escReleaseLine();
}

static void escWriteByte(uint8_t b) {
  // Match Betaflight/Cleanflight's software UART timing: one idle-high bit,
  // one start bit, 8 data bits LSB-first, then stop-high.
  // Keep interrupts off for this one byte so flash-write blocks do not get
  // occasional CRC errors from ISR jitter.
  uint16_t bitmask = ((uint16_t)b << 2) | 1U | (1U << 10);
  uint32_t btime = micros();

  portENTER_CRITICAL(&escUartMux);
  while (bitmask != 0) {
    if (bitmask & 1U) escDriveHigh();
    else              escDriveLow();

    btime += ESC_BIT_US;
    bitmask >>= 1;
    if (bitmask == 0) break;
    while ((int32_t)(micros() - btime) < 0) { }
  }
  portEXIT_CRITICAL(&escUartMux);

  escReleaseLine();
}


static int escReadByte(uint32_t timeoutMs) {
  escReleaseLine();
  const uint32_t timeoutUs = timeoutMs * 1000UL;
  const uint32_t startWait = micros();

  // Wait for start bit low.
  while (escReadLine()) {
    if ((uint32_t)(micros() - startWait) >= timeoutUs) return -1;
  }

  // Betaflight samples the start bit at 3/4 of a bit time, then samples all
  // 8 data bits and the stop bit at full-bit intervals. This is a bit more
  // tolerant of software timing jitter than sampling only the data bits.
  const uint32_t START_SAMPLE_US = ESC_BIT_US - (ESC_BIT_US / 4); // ~39us @19200
  uint32_t btime = micros() + START_SAMPLE_US;
  uint16_t bitmask = 0;

  portENTER_CRITICAL(&escUartMux);
  for (uint8_t bit = 0; bit < 10; bit++) {
    while ((int32_t)(micros() - btime) < 0) { }
    if (escReadLine()) bitmask |= (1U << bit);
    btime += ESC_BIT_US;
  }
  portEXIT_CRITICAL(&escUartMux);

  // bit0 must be the low start bit, bit9 must be the high stop bit.
  if ((bitmask & 1U) || !(bitmask & (1U << 9))) return -1;
  return (uint8_t)(bitmask >> 1);
}


static uint16_t escReadBytes(uint8_t *rx, uint16_t maxLen, uint16_t firstByteTimeoutMs) {
  uint16_t count = 0;

  int b = escReadByte(firstByteTimeoutMs);
  if (b < 0) return 0;
  rx[count++] = (uint8_t)b;

  // After the first byte, use a shorter inter-byte timeout.
  while (count < maxLen) {
    b = escReadByte(3);
    if (b < 0) break;
    rx[count++] = (uint8_t)b;
  }
  return count;
}

static uint16_t escReadBytesExact(uint8_t *rx,
                                  uint16_t expectedLen,
                                  uint16_t firstByteTimeoutMs,
                                  uint16_t interByteTimeoutMs) {
  uint16_t count = 0;

  while (count < expectedLen) {
    const uint16_t timeout = (count == 0) ? firstByteTimeoutMs : interByteTimeoutMs;
    const int b = escReadByte(timeout);
    if (b < 0) break;
    rx[count++] = (uint8_t)b;
  }

  return count;
}

static uint16_t escSend(const uint8_t *tx, uint16_t len, bool appendCrc = true) {
  if (len == 0) len = 256;

  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    escWriteByte(tx[i]);
    crc = escByteCrc(tx[i], crc);
  }

  if (appendCrc) {
    // BLHeli bootloader CRC is little-endian on the wire.
    escWriteByte(crc & 0xFF);
    escWriteByte((crc >> 8) & 0xFF);
  }

  escReleaseLine();
  return len + (appendCrc ? 2 : 0);
}

static bool escCommandAck(const uint8_t *tx, uint16_t len, uint16_t timeoutMs, uint8_t *rxBuf = nullptr, uint16_t *rxLen = nullptr) {
  uint8_t localRx[300] = {0};
  uint8_t *rx = rxBuf ? rxBuf : localRx;
  uint16_t maxLen = rxBuf ? *rxLen : sizeof(localRx);

  escSend(tx, len, true);

  // Important: this is software/bit-banged 1-wire serial. There is no RX buffer.
  // Start listening immediately after releasing the line or the ESC reply can be missed.
  uint16_t n = escReadBytes(rx, maxLen, timeoutMs);
  if (rxLen) *rxLen = n;
  if (n == 0) return false;
  return rx[n - 1] == brSUCCESS || rx[0] == brSUCCESS;
}

static bool escKeepAlive(uint8_t esc) {
  if (esc > 3 || !escBootloaderActive[esc]) return false;
  const int savedEsc = activeEsc;

  escSelect(esc);

  // Match Betaflight/Cleanflight: CMD_KEEP_ALIVE is sent as two command bytes
  // {0xFD, 0x00}, then CRC. The expected bootloader reply is brERRORCOMMAND.
  const uint8_t keepAliveCmd[] = { CMD_KEEP_ALIVE, 0x00 };
  escSend(keepAliveCmd, sizeof(keepAliveCmd), true);

  int ack = escReadByte(4);

  escSelect(savedEsc);
  return ack == brERRORCOMMAND;
}

static void serviceEscKeepAlive() {
  if (ESC_4WAY_KEEPALIVE_MS == 0) return;

  // Never start a bit-banged keepalive if BLHeliSuite has already placed host
  // bytes in the USB serial buffer. Read the host frame first. This is critical
  // during flashing, where a page erase is followed by DeviceWrite very quickly.
  if (Serial.available() > 0) return;

  const uint32_t now = millis();
  if ((int32_t)(now - suppressKeepAliveUntilMs) < 0) return;
  if ((uint32_t)(now - lastEscKeepAliveMs) < ESC_4WAY_KEEPALIVE_MS) return;
  lastEscKeepAliveMs = now;

#if ESC_4WAY_KEEPALIVE_ALL_ESC
  // Do not keep all four ESCs alive in one blocking burst. At 19200 baud even
  // a tiny bootloader command takes milliseconds, and doing four in a row can
  // make the adapter miss/late-process the next 4-way host frame. Round-robin
  // one initialized ESC per service call instead.
  for (uint8_t n = 0; n < 4; n++) {
    const uint8_t esc = keepAliveRoundRobinEsc & 0x03;
    keepAliveRoundRobinEsc = (uint8_t)((keepAliveRoundRobinEsc + 1) & 0x03);
    if (escBootloaderActive[esc]) {
      if (Serial.available() == 0) (void)escKeepAlive(esc);
      break;
    }
  }
#else
  if (activeEsc >= 0 && activeEsc < 4 && escBootloaderActive[activeEsc] && Serial.available() == 0) {
    (void)escKeepAlive((uint8_t)activeEsc);
  }
#endif
}

static bool escEnterBootloader(uint8_t *rx, uint16_t *rxLen) {
  // Same boot-init used by Betaflight serial_4way_avrootloader.c for BLHeli
  // bootloader mode. It is sent without an appended CRC because the MCU is not
  // connected yet; the final F4 7D are part of the boot-init pattern itself.
  const uint8_t bootInit[] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0x0D, 'B', 'L', 'H', 'e', 'l', 'i',
    0xF4, 0x7D
  };

  // Expected no-CRC boot reply: "471x" + signature_hi + signature_lo +
  // boot_version + boot_pages + ACK. The ACK is usually brSUCCESS (0x30).
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    escReleaseLine();
    delay(5);

    escSend(bootInit, sizeof(bootInit), false);

    // Listen immediately. Waiting here misses the bootloader's software-UART
    // reply because there is no hardware RX FIFO on this bit-banged line.
    uint16_t maxLen = *rxLen;
    uint16_t n = 0;
    for (; n < 9 && n < maxLen; n++) {
      int b = escReadByte(n == 0 ? 20 : 3);
      if (b < 0) break;
      rx[n] = (uint8_t)b;
    }
    *rxLen = n;

    if (n >= 9 &&
        rx[0] == '4' && rx[1] == '7' && rx[2] == '1' &&
        rx[8] == brSUCCESS) {
      return true;
    }

    delay(20);
  }

  return false;
}


// -------------------- Host 4-way serial helpers --------------------
static bool serialReadExact(uint8_t *dst, uint16_t len, uint32_t timeoutMs, bool serviceKeepAliveWhileWaiting = true) {
  const uint32_t start = millis();
  uint16_t got = 0;
  while (got < len) {
    while (Serial.available() && got < len) {
      dst[got++] = (uint8_t)Serial.read();
    }

    // Only run ESC keep-alive while waiting for a new host frame. Once a 4-way
    // frame has started, don't bit-bang ESC lines until the frame is fully read.
    // This prevents long multi-byte host frames from being interrupted.
    if (serviceKeepAliveWhileWaiting && got == 0) serviceEscKeepAlive();

    if ((uint32_t)(millis() - start) > timeoutMs) return false;
    delay(1);
  }
  return true;
}

static void send4WayResponse(uint8_t cmd, uint8_t addrH, uint8_t addrL,
                             const uint8_t *params, uint16_t paramLen, uint8_t ack) {
  uint8_t out[270] = {0};
  uint16_t idx = 0;

  out[idx++] = cmd_Remote_Escape;
  out[idx++] = cmd;
  out[idx++] = addrH;
  out[idx++] = addrL;
  out[idx++] = (paramLen == 256) ? 0 : (uint8_t)paramLen;

  for (uint16_t i = 0; i < paramLen; i++) out[idx++] = params ? params[i] : 0;
  out[idx++] = ack;

  uint16_t crc = 0;
  for (uint16_t i = 0; i < idx; i++) crc = crcXmodemUpdate(crc, out[i]);
  out[idx++] = (crc >> 8) & 0xFF;
  out[idx++] = crc & 0xFF;

  Serial.write(out, idx);
  Serial.flush();
}

static void sendAck(uint8_t cmd, uint8_t addrH, uint8_t addrL, uint8_t ack) {
  // Betaflight/Cleanflight 4-way keeps O_PARAM_LEN = 1 for simple
  // ACK-only commands, with one dummy 0x00 parameter followed by ACK.
  // BLHeliSuite can reject zero-length ACK replies as "unexpected parameter length".
  const uint8_t dummy = 0x00;
  send4WayResponse(cmd, addrH, addrL, &dummy, 1, ack);
}

static uint16_t inputParamBytes(uint8_t cmd, uint8_t lenByte) {
  // In 4-way, len byte 0 can mean 256 bytes for flash buffer commands.
  // For interface commands it means no input parameters.
  if (lenByte == 0 && (cmd == cmd_DeviceWrite || cmd == cmd_DeviceVerify)) return 256;
  return lenByte;
}

static bool read4WayFrameFromFirst(uint8_t firstByte, uint8_t *frame, uint16_t *frameLen) {
  frame[0] = firstByte;
  if (!serialReadExact(&frame[1], 4, 1000, false)) return false;

  const uint8_t cmd = frame[1];
  const uint16_t paramBytes = inputParamBytes(cmd, frame[4]);
  if (paramBytes > 256) return false;

  if (!serialReadExact(&frame[5], paramBytes + 2, 1500, false)) return false;
  *frameLen = 5 + paramBytes + 2;
  return true;
}

static bool read4WayFrame(uint8_t *frame, uint16_t *frameLen) {
  // Find local escape byte. Keep ESC bootloaders alive while the host is idle,
  // but do not service ESC lines in the middle of a host frame.
  uint8_t b = 0;
  do {
    if (!serialReadExact(&b, 1, 1000, true)) return false;
  } while (b != cmd_Local_Escape && b != cmd_Remote_Escape);

  return read4WayFrameFromFirst(b, frame, frameLen);
}

static bool verifyHostCrc(const uint8_t *frame, uint16_t frameLen, uint16_t paramBytes) {
  uint16_t crc = 0;
  const uint16_t crcOffset = 5 + paramBytes;
  for (uint16_t i = 0; i < crcOffset; i++) crc = crcXmodemUpdate(crc, frame[i]);
  const uint16_t rxCrc = ((uint16_t)frame[crcOffset] << 8) | frame[crcOffset + 1];
  return crc == rxCrc;
}

// -------------------- MSP v1 serial helpers --------------------
static uint8_t mspChecksum(uint8_t size, uint8_t cmd, const uint8_t *payload) {
  uint8_t crc = size ^ cmd;
  for (uint8_t i = 0; i < size; i++) crc ^= payload[i];
  return crc;
}

static void sendMspFrame(char direction, uint8_t cmd, const uint8_t *payload, uint8_t size) {
  uint8_t hdr[5] = { '$', 'M', (uint8_t)direction, size, cmd };
  Serial.write(hdr, sizeof(hdr));
  if (size && payload) Serial.write(payload, size);
  const uint8_t crc = mspChecksum(size, cmd, payload ? payload : (const uint8_t *)"");
  Serial.write(crc);
  Serial.flush();
}

static void sendMspResponse(uint8_t cmd, const uint8_t *payload, uint8_t size) {
  sendMspFrame('>', cmd, payload, size);
}

static void sendMspError(uint8_t cmd) {
  sendMspFrame('!', cmd, nullptr, 0);
}

static void mspStart4WayInterface() {
  // ESC Configurator / Bluejay Configurator first talks MSP, then switches the
  // same serial port to raw 4-way frames after MSP_SET_4WAY_IF. Prepare the ESC
  // pins but do not start normal flight/RMT motor output.
  activeMode = imSIL_BLB;
  for (uint8_t i = 0; i < 4; i++) {
    escBootloaderActive[i] = false;
    pinMode(escPins4Way[i], INPUT_PULLUP);
  }
  escSelect(0);
  lastEscKeepAliveMs = millis();
}

static bool handleMspCommand(uint8_t cmd, const uint8_t *payload, uint8_t size) {
  (void)payload;
  (void)size;

  uint8_t out[96] = {0};
  uint8_t outLen = 0;

  switch (cmd) {
    case MSP_API_VERSION:
      out[0] = 0;    // MSP protocol version
      out[1] = 1;    // API major
      out[2] = 42;   // API minor, Betaflight 4.x style
      outLen = 3;
      break;

    case MSP_FC_VARIANT:
      out[0] = 'B'; out[1] = 'T'; out[2] = 'F'; out[3] = 'L';
      outLen = 4;
      break;

    case MSP_FC_VERSION:
      out[0] = 4; out[1] = 1; out[2] = 0;
      outLen = 3;
      break;

    case MSP_BOARD_INFO: {
      // Enough Betaflight-style board info for web configurators to accept this
      // as an FC that can enter 4-way ESC passthrough.
      const char name[] = "ESP32S3 ESC Bridge";
      out[0] = 'E'; out[1] = 'S'; out[2] = 'P'; out[3] = '3'; // board id
      out[4] = 0x00; out[5] = 0x00; // hardware rev
      out[6] = 0x00;                // no MAX7456
      out[7] = 0x00;                // target capabilities
      out[8] = sizeof(name) - 1;
      memcpy(&out[9], name, sizeof(name) - 1);
      outLen = 9 + sizeof(name) - 1;
      break;
    }

    case MSP_BUILD_INFO:
      // 11-byte date + 8-byte time + 7-byte git rev = 26 bytes.
      memcpy(out, "Jan 01 2026", 11);
      memcpy(out + 11, "00:00:00", 8);
      memcpy(out + 19, "esp4way", 7);
      outLen = 26;
      break;

    case MSP_NAME: {
      const char name[] = "ESP32-ESC";
      memcpy(out, name, sizeof(name) - 1);
      outLen = sizeof(name) - 1;
      break;
    }

    case MSP_FEATURE_CONFIG:
      out[0] = out[1] = out[2] = out[3] = 0x00;
      outLen = 4;
      break;

    case MSP_STATUS:
      // 22-byte Betaflight status payload; all-zero/no-sensors is fine here.
      outLen = 22;
      break;

    case MSP_MOTOR:
      // 8 motors, little-endian. First four at 1000, rest 0.
      for (uint8_t i = 0; i < 4; i++) {
        out[i * 2] = 0xE8;
        out[i * 2 + 1] = 0x03;
      }
      outLen = 16;
      break;

    case MSP_MOTOR_CONFIG:
      // minthrottle=1070, maxthrottle=2000, mincommand=1000, motor count=4.
      out[0] = 0x2E; out[1] = 0x04;
      out[2] = 0xD0; out[3] = 0x07;
      out[4] = 0xE8; out[5] = 0x03;
      out[6] = 4;
      out[7] = 14; // motor pole count placeholder
      out[8] = 0;  // DShot telemetry disabled
      out[9] = 0;  // ESC sensor feature disabled
      outLen = 10;
      break;

    case MSP_ADVANCED_CONFIG:
      // 20-byte payload. Index 8=6 means DShot300 in Betaflight's enum; this is
      // only descriptive for the configurator, not used by the passthrough code.
      outLen = 20;
      out[8] = 0x06;
      out[9] = 0xE0; out[10] = 0x01; // 480 Hz nominal
      break;

    case MSP_UID:
      outLen = 12;
      break;

    case MSP_SET_4WAY_IF:
      mspStart4WayInterface();
      out[0] = 4; // number of ESC outputs available: M1..M4
      outLen = 1;
      break;

    case MSP_REBOOT:
      sendMspResponse(cmd, nullptr, 0);
      delay(20);
      return true;

    default:
      sendMspError(cmd);
      return true;
  }

  sendMspResponse(cmd, out, outLen);
  return true;
}

static bool readAndHandleMspFrame() {
  // The '$' byte has already been consumed. This supports MSPv1 frames: $M< size cmd payload crc.
  uint8_t hdr[4] = {0};
  if (!serialReadExact(hdr, sizeof(hdr), 1000, false)) return false;
  if (hdr[0] != 'M' || hdr[1] != '<') {
    return false;
  }

  const uint8_t size = hdr[2];
  const uint8_t cmd  = hdr[3];
  uint8_t payload[255] = {0};
  if (size && !serialReadExact(payload, size, 1000, false)) return false;

  uint8_t rxCrc = 0;
  if (!serialReadExact(&rxCrc, 1, 1000, false)) return false;

  const uint8_t calc = mspChecksum(size, cmd, payload);
  if (rxCrc != calc) {
    sendMspError(cmd);
    return true;
  }

  return handleMspCommand(cmd, payload, size);
}

static uint8_t runEscReadCommand(uint8_t bootCmd, uint8_t addrH, uint8_t addrL, uint8_t readLen, uint8_t *out, uint16_t *outLen) {
  // In the BLHeli 4-way protocol the host passes the requested byte count as
  // one parameter. A value of 0 means 256 bytes. BLHeliSuite's verify step uses
  // 256-byte reads, so returning a zero-length frame here causes "COM Read Error"
  // / "Flash verify failed" even though programming succeeded.
  const uint16_t expectedDataLen = (readLen == 0) ? 256 : readLen;
  const uint16_t expectedTotalLen = expectedDataLen + 3; // data + CRC low + CRC high + ACK

  uint8_t rx[300] = {0};

  // Read-back verify is long enough that a single software-UART sample glitch can
  // lose a byte. Retry the same read a few times before reporting failure to the PC.
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;

    uint16_t rxLen = sizeof(rx);
    const uint8_t setAddr[] = { CMD_SET_ADDRESS, 0x00, addrH, addrL };
    if (!escCommandAck(setAddr, sizeof(setAddr), 200, rx, &rxLen)) {
      delay(2);
      continue;
    }

    const uint8_t readCmd[] = { bootCmd, readLen };
    escSend(readCmd, sizeof(readCmd), true);

    // Read exactly the number of bytes BLHeliSuite requested, rather than reading
    // until a short timeout. The old read-until-timeout logic could stop early on
    // a 256-byte verify block and then return ACK_D_GENERAL_ERROR.
    memset(rx, 0, sizeof(rx));
    rxLen = escReadBytesExact(rx, expectedTotalLen, 700, 12);
    if (rxLen != expectedTotalLen) {
      delay(2);
      continue;
    }

    const int bootAck = rx[expectedTotalLen - 1];
    if (bootAck != brSUCCESS) {
      *outLen = 0;
      return mapBootAckTo4WayAck(bootAck);
    }

    uint16_t crc = 0;
    for (uint16_t i = 0; i < expectedDataLen + 2; i++) crc = escByteCrc(rx[i], crc);
    if (crc != 0) {
      delay(2);
      continue;
    }

    for (uint16_t i = 0; i < expectedDataLen; i++) out[i] = rx[i];
    *outLen = expectedDataLen;
    suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
    return ACK_OK;
  }

  *outLen = 0;
  suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
  return ACK_D_GENERAL_ERROR;
}

static uint8_t runEscRead(uint8_t addrH, uint8_t addrL, uint8_t readLen, uint8_t *out, uint16_t *outLen) {
  return runEscReadCommand(CMD_READ_FLASH_SIL, addrH, addrL, readLen, out, outLen);
}

static uint8_t runEscReadEeprom(uint8_t addrH, uint8_t addrL, uint8_t readLen, uint8_t *out, uint16_t *outLen) {
  return runEscReadCommand(CMD_READ_EEPROM, addrH, addrL, readLen, out, outLen);
}

static uint8_t mapBootAckTo4WayAck(int bootAck) {
  if (bootAck == brSUCCESS || bootAck == brNONE) return ACK_OK;
  if (bootAck == brERRORCRC) return ACK_I_INVALID_CRC;
  if (bootAck == brERRORCOMMAND) return ACK_I_INVALID_CMD;
  if (bootAck == brERRORVERIFY) return 0x04; // ACK_I_VERIFY_ERROR in 4-way
  return ACK_D_GENERAL_ERROR;
}

static int escReadAckOrNone(uint32_t timeoutMs) {
  // Betaflight's BL_GetACK() initializes LastACK to brNONE and returns brNONE
  // when no byte arrives before the timeout. Our escReadByte() returns -1 on
  // timeout, so convert that timeout into brNONE for stages that expect it.
  const int ack = escReadByte(timeoutMs);
  return (ack < 0) ? brNONE : ack;
}

static uint8_t runEscWriteCommand(uint8_t bootCmd, uint8_t addrH, uint8_t addrL, const uint8_t *data, uint16_t dataLen, uint16_t progTimeoutMs) {
  suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
  // Match Betaflight's BL_WriteA / BL_SendCMDSetBuffer sequence:
  //   SET_ADDRESS -> wait for brSUCCESS
  //   SET_BUFFER  -> expect brNONE, meaning no byte was returned before timeout
  //   DATA+CRC    -> wait for brSUCCESS
  //   PROG_*      -> wait for brSUCCESS
  // The important v15 fix: brNONE is a timeout/no-byte condition, not a literal
  // 0xFF byte from the ESC. Earlier patches treated timeout as failure here, so
  // DeviceWrite returned silBLB:General Error before the data bytes were sent.

  const uint8_t setAddr[] = { CMD_SET_ADDRESS, 0x00, addrH, addrL };
  escSend(setAddr, sizeof(setAddr), true);
  int ack = escReadByte(50);
  if (ack != brSUCCESS) return mapBootAckTo4WayAck(ack);

  uint8_t setBuffer[4] = { CMD_SET_BUFFER, 0x00, 0x00, (uint8_t)dataLen };
  if (dataLen == 256) {
    setBuffer[2] = 0x01;
    setBuffer[3] = 0x00;
  }

  escSend(setBuffer, sizeof(setBuffer), true);
  ack = escReadAckOrNone(5);
  if (ack != brNONE) return mapBootAckTo4WayAck(ack);

  escSend(data, dataLen, true);
  ack = escReadByte(250);
  if (ack != brSUCCESS) return mapBootAckTo4WayAck(ack);

  const uint8_t prog[] = { bootCmd, 0x01 };
  escSend(prog, sizeof(prog), true);
  ack = escReadByte(progTimeoutMs);
  if (ack != brSUCCESS) return mapBootAckTo4WayAck(ack);

  suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
  return ACK_OK;
}

static uint8_t runEscWrite(uint8_t addrH, uint8_t addrL, const uint8_t *data, uint16_t dataLen) {
  // Betaflight uses 500ms for flash programming ACK timeout.
  return runEscWriteCommand(CMD_PROG_FLASH, addrH, addrL, data, dataLen, 500);
}

static uint8_t runEscWriteEeprom(uint8_t addrH, uint8_t addrL, const uint8_t *data, uint16_t dataLen) {
  // Betaflight gives EEPROM programming a longer timeout. Some SiLabs ESCs take
  // noticeably longer to commit settings than to answer normal flash reads.
  return runEscWriteCommand(CMD_PROG_EEPROM, addrH, addrL, data, dataLen, 3000);
}

static uint8_t runEscPageErase(uint8_t page) {
  suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
  uint8_t rx[300] = {0};
  uint16_t rxLen = sizeof(rx);

  // SiLabs pages are 512 bytes (page << 1). ARM BLHeli_32 pages are 1024 bytes (page << 2).
  const uint8_t addrH = (activeMode == imARM_BLB) ? (page << 2) : (page << 1);
  const uint8_t addrL = 0x00;
  const uint8_t setAddr[] = { CMD_SET_ADDRESS, 0x00, addrH, addrL };
  if (!escCommandAck(setAddr, sizeof(setAddr), 200, rx, &rxLen)) return ACK_D_GENERAL_ERROR;

  const uint8_t eraseCmd[] = { CMD_ERASE_FLASH, 0x01 };
  escSend(eraseCmd, sizeof(eraseCmd), true);

  // Do NOT delay before listening. The BLHeli bootloader reply is bit-banged
  // 1-wire serial with no RX FIFO on the ESP32 side. Betaflight sends the
  // erase command and immediately waits for one ACK, with a long timeout.
  const int ack = escReadByte(3000);
  if (ack != brSUCCESS) return mapBootAckTo4WayAck(ack);

  // Some SiLabs bootloaders ACK the page erase before the flash array is fully
  // ready for the following program command. Give it a tiny settle time before
  // BLHeliSuite immediately sends the first DeviceWrite.
  delay(10);
  suppressKeepAliveUntilMs = millis() + ESC_4WAY_FLASH_QUIET_MS;
  return ACK_OK;
}

static void handle4WayFrame(uint8_t *frame, uint16_t frameLen) {
  const uint8_t cmd = frame[1];
  const uint8_t addrH = frame[2];
  const uint8_t addrL = frame[3];
  const uint8_t lenByte = frame[4];
  const uint16_t paramBytes = inputParamBytes(cmd, lenByte);
  uint8_t *param = &frame[5];

  if (!verifyHostCrc(frame, frameLen, paramBytes)) {
    sendAck(cmd, addrH, addrL, ACK_I_INVALID_CRC);
    return;
  }

  uint8_t out[256] = {0};
  uint16_t outLen = 0;
  uint8_t ack = ACK_OK;

#if ESC_4WAY_DEBUG
  Serial.printf("4WAY cmd=0x%02X addr=%02X%02X len=%u esc=%d pin=%d\n",
                cmd, addrH, addrL, paramBytes, activeEsc, activePin);
#endif

  switch (cmd) {
    case cmd_InterfaceTestAlive:
      // Reply immediately. Do not bit-bang ESC keep-alive before this ACK, because
      // BLHeliSuite uses this to decide whether the interface itself is still alive.
      sendAck(cmd, addrH, addrL, ACK_OK);
      return;

    case cmd_ProtocolGetVersion:
      out[0] = SERIAL_4WAY_PROTOCOL_VER;
      send4WayResponse(cmd, addrH, addrL, out, 1, ACK_OK);
      return;

    case cmd_InterfaceGetName: {
      const char name[] = "m4wESP32"; // 8 chars is okay; tools do not require 9.
      memcpy(out, name, sizeof(name) - 1);
      send4WayResponse(cmd, addrH, addrL, out, sizeof(name) - 1, ACK_OK);
      return;
    }

    case cmd_InterfaceGetVersion:
      out[0] = SERIAL_4WAY_VERSION_HI;
      out[1] = SERIAL_4WAY_VERSION_LO;
      send4WayResponse(cmd, addrH, addrL, out, 2, ACK_OK);
      return;

    case cmd_InterfaceSetMode:
      if (paramBytes < 1) ack = ACK_I_INVALID_PARAM;
      else if (param[0] == imSIL_BLB || param[0] == imARM_BLB || param[0] == imATM_BLB) {
        activeMode = param[0];
        ack = ACK_OK;
      } else {
        ack = ACK_I_INVALID_PARAM;
      }
      sendAck(cmd, addrH, addrL, ack);
      return;

    case cmd_InterfaceExit:
      for (uint8_t esc = 0; esc < 4; esc++) escBootloaderActive[esc] = false;
      escReleaseLine();
      sendAck(cmd, addrH, addrL, ACK_OK);
      return;

    case cmd_DeviceInitFlash: {
      const uint8_t esc = (paramBytes >= 1) ? param[0] : 0;
      if (esc > 3) {
        sendAck(cmd, addrH, addrL, ACK_I_INVALID_CHANNEL);
        return;
      }

      escSelect(esc);
      uint8_t rx[32] = {0};
      uint16_t rxLen = sizeof(rx);
      if (escEnterBootloader(rx, &rxLen)) {
        escBootloaderActive[esc] = true;
        lastEscKeepAliveMs = millis();
        // Return 4-byte DeviceInfo, matching Betaflight serial_4way.
        // DeviceInfo[3] is the interface mode. The first two bytes are the MCU ID.
        out[0] = (rxLen > 5) ? rx[5] : 0x30;
        out[1] = (rxLen > 4) ? rx[4] : 0xF3;
        out[2] = (rxLen > 3) ? rx[3] : 0x00;
        out[3] = activeMode;
        send4WayResponse(cmd, addrH, addrL, out, 4, ACK_OK);
      } else {
        // DeviceInitFlash must still return a 4-byte DeviceInfo field even when
        // the device connect fails. Betaflight clears DeviceInfo to zero and
        // returns ACK_D_GENERAL_ERROR; BLHeliSuite expects exactly that length.
        out[0] = out[1] = out[2] = out[3] = 0x00;
        send4WayResponse(cmd, addrH, addrL, out, 4, ACK_D_GENERAL_ERROR);
      }
      return;
    }

    case cmd_DeviceReset: {
      if (paramBytes >= 1 && param[0] <= 3) escSelect(param[0]);

      // Leave the BLHeli bootloader and let the ESC application run.
      // Earlier patches sent RestartBootloader (0x00) and then held the line low,
      // which could leave the ESC in bootloader/programming state after a flash.
      // BLHeliSuite normally expects the ESC to exit bootloader after flashing so
      // it can read the newly-flashed application identity.
      const uint8_t exitCmd[] = { ExitBootloader, 0x00 };
      escSend(exitCmd, sizeof(exitCmd), true);
      (void)escReadAckOrNone(50);

      if (activeEsc >= 0 && activeEsc < 4) escBootloaderActive[activeEsc] = false;
      escReleaseLine();
      delay(250);
      sendAck(cmd, addrH, addrL, ACK_OK);
      return;
    }

    case cmd_DeviceRead:
      if (paramBytes < 1) {
        sendAck(cmd, addrH, addrL, ACK_I_INVALID_PARAM);
        return;
      }
      ack = runEscRead(addrH, addrL, param[0], out, &outLen);
      send4WayResponse(cmd, addrH, addrL, out, outLen, ack);
      return;

    case cmd_DeviceWrite:
      ack = runEscWrite(addrH, addrL, param, paramBytes == 0 ? 256 : paramBytes);
      sendAck(cmd, addrH, addrL, ack);
      return;

    case cmd_DeviceReadEEprom:
      if (paramBytes < 1) {
        sendAck(cmd, addrH, addrL, ACK_I_INVALID_PARAM);
        return;
      }
      ack = runEscReadEeprom(addrH, addrL, param[0], out, &outLen);
      send4WayResponse(cmd, addrH, addrL, out, outLen, ack);
      return;

    case cmd_DeviceWriteEEprom:
      ack = runEscWriteEeprom(addrH, addrL, param, paramBytes == 0 ? 256 : paramBytes);
      sendAck(cmd, addrH, addrL, ack);
      return;

    case cmd_DevicePageErase:
      if (paramBytes < 1) {
        sendAck(cmd, addrH, addrL, ACK_I_INVALID_PARAM);
        return;
      }
      ack = runEscPageErase(param[0]);
      sendAck(cmd, addrH, addrL, ack);
      return;

    case cmd_DeviceVerify: {
      // Do a real compare instead of blindly ACKing OK. Some BLHeliSuite paths
      // use cmd_DeviceVerify rather than a full read-back verify, and returning
      // OK here can make a bad/incomplete flash look successful.
      const uint16_t verifyLen = (paramBytes == 0) ? 256 : paramBytes;
      if (verifyLen > 256) {
        sendAck(cmd, addrH, addrL, ACK_I_INVALID_PARAM);
        return;
      }

      uint16_t readBackLen = 0;
      ack = runEscRead(addrH, addrL, (verifyLen == 256) ? 0 : (uint8_t)verifyLen, out, &readBackLen);
      if (ack == ACK_OK) {
        if (readBackLen != verifyLen || memcmp(out, param, verifyLen) != 0) {
          ack = ACK_D_VERIFY_ERROR;
        }
      }
      sendAck(cmd, addrH, addrL, ack);
      return;
    }

    default:
      sendAck(cmd, addrH, addrL, ACK_I_INVALID_CMD);
      return;
  }
}


static bool readAndHandleHostFrame(uint8_t *frame, uint16_t *frameLen) {
  // Accept both raw 4-way frames ('/') for classic BLHeliSuite/direct tools and
  // MSP frames ('$M<...') for ESC Configurator / Bluejay Configurator.
  uint8_t b = 0;
  do {
    if (!serialReadExact(&b, 1, 1000, true)) return false;
  } while (b != cmd_Local_Escape && b != cmd_Remote_Escape && b != '$');

  if (b == '$') {
    return readAndHandleMspFrame();
  }

  if (!read4WayFrameFromFirst(b, frame, frameLen)) return false;
  handle4WayFrame(frame, *frameLen);
  return true;
}


void esc4wayRun() {
  // Passthrough mode is a single tight binary protocol loop. Avoid WDT resets
  // during long ESC reads/writes.
  esp_task_wdt_deinit();

  Serial.end();
  delay(100);
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
#if ESC_PASSTHROUGH_PROTOCOL == ESC_PASSTHROUGH_MSP_4WAY
  Serial.begin(ESC_MSP_HOST_BAUD);
#else
  Serial.begin(ESC_4WAY_HOST_BAUD);
#endif
  delay(300);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(escPins4Way[i], INPUT_PULLUP);
  }
  escSelect(0);

  // Do not print human-readable text here; BLHeliSuite expects binary protocol.
  uint8_t frame[270] = {0};
  uint16_t frameLen = 0;

  while (true) {
    if (!readAndHandleHostFrame(frame, &frameLen)) {
      serviceEscKeepAlive();
      delay(1);
    }
  }
}

#else

void esc4wayRun() {
  // Not used in normal flight-controller mode.
}

#endif
