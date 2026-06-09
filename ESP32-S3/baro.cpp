#include "sensors.h"
#include "config.h"
#include "driver/i2c.h"

static float bmp280_p0 = 1013.25f;

// Bosch factory-trim coefficients (loaded once in initBMP280)
static uint16_t bmp_dig_T1;
static int16_t  bmp_dig_T2, bmp_dig_T3;
static uint16_t bmp_dig_P1;
static int16_t  bmp_dig_P2, bmp_dig_P3, bmp_dig_P4,
                bmp_dig_P5, bmp_dig_P6, bmp_dig_P7,
                bmp_dig_P8, bmp_dig_P9;
static int32_t  bmp_t_fine; 

void initBMP280() {
    // Soft-reset
    writeRegister(I2C_EXT_BUS, BMP280_ADDR, 0xE0, 0xB6);
    delay(10);

    // Verify chip ID (should be 0x58 for BMP280)
    uint8_t id = readRegister(I2C_EXT_BUS, BMP280_ADDR, 0xD0);
    if (id != 0x58) {
        Serial.printf("[BMP280] unexpected chip ID: 0x%02X\n", id);
    }

    // Read 24 bytes of calibration data starting at 0x88
    uint8_t reg = 0x88;
    uint8_t cal[24];
    i2c_master_write_read_device(I2C_EXT_BUS, BMP280_ADDR,
                                 &reg, 1, cal, sizeof(cal), I2C_TIMEOUT_MS);

    bmp_dig_T1 = (uint16_t)(cal[1]  << 8 | cal[0]);
    bmp_dig_T2 = (int16_t) (cal[3]  << 8 | cal[2]);
    bmp_dig_T3 = (int16_t) (cal[5]  << 8 | cal[4]);
    bmp_dig_P1 = (uint16_t)(cal[7]  << 8 | cal[6]);
    bmp_dig_P2 = (int16_t) (cal[9]  << 8 | cal[8]);
    bmp_dig_P3 = (int16_t) (cal[11] << 8 | cal[10]);
    bmp_dig_P4 = (int16_t) (cal[13] << 8 | cal[12]);
    bmp_dig_P5 = (int16_t) (cal[15] << 8 | cal[14]);
    bmp_dig_P6 = (int16_t) (cal[17] << 8 | cal[16]);
    bmp_dig_P7 = (int16_t) (cal[19] << 8 | cal[18]);
    bmp_dig_P8 = (int16_t) (cal[21] << 8 | cal[20]);
    bmp_dig_P9 = (int16_t) (cal[23] << 8 | cal[22]);

    // ctrl_meas: osrs_t = x2 (010), osrs_p = x16 (101), mode = normal (11)
    // config:    t_sb = 0.5ms (000), filter = x16 (100), spi3w_en = 0
    writeRegister(I2C_EXT_BUS, BMP280_ADDR, 0xF5, 0b00010000); // config — set filter first
    writeRegister(I2C_EXT_BUS, BMP280_ADDR, 0xF4, 0b01010111); // ctrl_meas — then wake
    delay(50); // let the first measurement settle
}
void readBMP280() {
    // Burst-read 6 bytes: 0xF7=press_msb … 0xFC=temp_xlsb
    uint8_t reg = 0xF7;
    uint8_t buf[6];
    i2c_master_write_read_device(I2C_EXT_BUS, BMP280_ADDR,
                                 &reg, 1, buf, sizeof(buf), I2C_TIMEOUT_MS);

    // Reconstruct 20-bit raw values (same bit pattern Bosch specifies)
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    // --- Bosch temperature compensation (produces t_fine for pressure) ---
    int32_t var1 = (((adc_T >> 3) - ((int32_t)bmp_dig_T1 << 1))
                    * (int32_t)bmp_dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)bmp_dig_T1)
                    * ((adc_T >> 4) - (int32_t)bmp_dig_T1)) >> 12)
                    * (int32_t)bmp_dig_T3) >> 14;
    bmp_t_fine = var1 + var2;
    // float temp_C = (bmp_t_fine * 5 + 128) / 256.0f / 100.0f; // available if needed

    // --- Bosch pressure compensation (integer path, result in Pa * 256) ---
    int64_t v1 = (int64_t)bmp_t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)bmp_dig_P6;
    v2 += (v1 * (int64_t)bmp_dig_P5) << 17;
    v2 += ((int64_t)bmp_dig_P4) << 35;
    v1  = ((v1 * v1 * (int64_t)bmp_dig_P3) >> 8)
        + ((v1 * (int64_t)bmp_dig_P2) << 12);
    v1  = (((int64_t)1 << 47) + v1) * (int64_t)bmp_dig_P1 >> 33;

    if (v1 == 0) return; // guard against divide-by-zero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)bmp_dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)bmp_dig_P8 * p) >> 19;
    p  = ((p + v1 + v2) >> 8) + ((int64_t)bmp_dig_P7 << 4);

    float pressure_hPa = (float)p / 25600.0f; // Pa*256 → hPa

    // --- Hypsometric altitude (metres above the reference set at boot) ---
    height_baro = 44330.0f * (1.0f - powf(pressure_hPa / bmp280_p0, 0.1903f));
}

void calibrateBMP280Ground() {
    // Average several readings to get a stable ground-level pressure
    float sum = 0;
    const int n = 20;
    for (int i = 0; i < n; i++) {
        readBMP280();
        uint8_t reg = 0xF7;
        uint8_t buf[6];
        i2c_master_write_read_device(I2C_EXT_BUS, BMP280_ADDR,
                                     &reg, 1, buf, sizeof(buf), I2C_TIMEOUT_MS);
        int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
        // reuse last t_fine from the readBMP280() call above
        int64_t v1 = (int64_t)bmp_t_fine - 128000;
        int64_t v2 = v1 * v1 * (int64_t)bmp_dig_P6;
        v2 += (v1 * (int64_t)bmp_dig_P5) << 17;
        v2 += ((int64_t)bmp_dig_P4) << 35;
        v1  = ((v1 * v1 * (int64_t)bmp_dig_P3) >> 8) + ((v1 * (int64_t)bmp_dig_P2) << 12);
        v1  = (((int64_t)1 << 47) + v1) * (int64_t)bmp_dig_P1 >> 33;
        if (v1 == 0) continue;
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = ((int64_t)bmp_dig_P9 * (p >> 13) * (p >> 13)) >> 25;
        v2 = ((int64_t)bmp_dig_P8 * p) >> 19;
        p  = ((p + v1 + v2) >> 8) + ((int64_t)bmp_dig_P7 << 4);
        sum += (float)p / 25600.0f;
        delay(25);
    }
    bmp280_p0 = sum / n;
    Serial.printf("[BMP280] ground reference: %.2f hPa\n", bmp280_p0);
}
