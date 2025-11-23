/*
 * File: ms5837.c
 * Description: High-level driver for MS5837-30BA
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef FOSC
#define FOSC    8000000UL      // 8 MHz Internal Oscillator
#endif

#ifndef FCY
#define FCY     (FOSC/2)       // 4 MHz Instruction Cycle
#endif

#include <libpic30.h>
#include "i2c_master.h"
#include "ms5837.h"

// Sensor Constants
#define MS5837_ADDR         0x76
#define MS5837_CMD_RESET    0x1E
#define MS5837_CMD_ADC_READ 0x00
#define MS5837_CMD_PROM     0xA0
#define MS5837_CMD_CONV_P   0x4A // OSR 8192 Pressure
#define MS5837_CMD_CONV_T   0x5A // OSR 8192 Temperature

MS5837_t ms5837;

static uint8_t crc4(const uint16_t prom[8]) {
    uint16_t n_rem = 0;
    uint16_t crc_read = prom[7] & 0xF; // Stored CRC
    uint16_t prom_local[8];

    for (int i = 0; i < 8; i++) {
        prom_local[i] = prom[i];
    }
    prom_local[7] &= 0xFFF0; // Clear CRC nibble

    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) {
            n_rem ^= (uint16_t)(prom_local[cnt >> 1] & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(prom_local[cnt >> 1] >> 8);
        }

        for (int n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = (n_rem >> 12) & 0x000F;
    return (uint8_t)(n_rem ^ crc_read);
}

/*
 * Function: MS5837_Init
 * Description: Resets sensor and reads calibration coefficients.
 */
bool MS5837_Init(void) {
    I2C_Reset_Bus(); // Ensure clean slate
    I2C_Init();

    // 1. Reset Sensor
    I2C_Start();
    if (!I2C_Write_Byte((MS5837_ADDR << 1) | 0)) { return false; }
    if (!I2C_Write_Byte(MS5837_CMD_RESET)) { return false; }
    I2C_Stop();
    __delay_ms(10); // Wait for internal reload

    // 2. Read PROM coefficients (C0..C7)
    for (int i = 0; i < 8; i++) {
        I2C_Start();
        if (!I2C_Write_Byte((MS5837_ADDR << 1) | 0)) { return false; }
        if (!I2C_Write_Byte(MS5837_CMD_PROM + (i * 2))) { return false; }
        I2C_Stop();

        I2C_Start();
        if (!I2C_Write_Byte((MS5837_ADDR << 1) | 1)) { return false; }
        uint8_t msb = I2C_Read_Byte(true);
        uint8_t lsb = I2C_Read_Byte(false); // NACK on last byte
        I2C_Stop();

        ms5837.C[i] = (uint16_t)((msb << 8) | lsb);
    }

    // 3. Validate CRC (CRC4 should be zero when OK)
    return (crc4(ms5837.C) == 0);
}

/*
 * Function: MS5837_Read_Data
 * Description: Performs conversion and reads D1 and D2.
 */
void MS5837_Read_Data(void) {
    // --- Get Pressure (D1) ---
    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 0);
    I2C_Write_Byte(MS5837_CMD_CONV_P);
    I2C_Stop();

    __delay_ms(20); // Wait for OSR 8192 conversion (~17ms)

    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 0);
    I2C_Write_Byte(MS5837_CMD_ADC_READ);
    I2C_Stop();

    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 1);
    uint32_t b2 = I2C_Read_Byte(true);
    uint32_t b1 = I2C_Read_Byte(true);
    uint32_t b0 = I2C_Read_Byte(false);
    I2C_Stop();
    ms5837.D1 = (b2 << 16) | (b1 << 8) | b0;

    // --- Get Temperature (D2) ---
    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 0);
    I2C_Write_Byte(MS5837_CMD_CONV_T);
    I2C_Stop();

    __delay_ms(20);

    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 0);
    I2C_Write_Byte(MS5837_CMD_ADC_READ);
    I2C_Stop();

    I2C_Start();
    I2C_Write_Byte((MS5837_ADDR << 1) | 1);
    b2 = I2C_Read_Byte(true);
    b1 = I2C_Read_Byte(true);
    b0 = I2C_Read_Byte(false);
    I2C_Stop();
    ms5837.D2 = (b2 << 16) | (b1 << 8) | b0;
}

/*
 * Function: MS5837_Calculate
 * Description: Applies 1st and 2nd order compensation.
 */
void MS5837_Calculate(void) {
    int64_t dT, TEMP, OFF, SENS, P;
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    dT = (int64_t)ms5837.D2 - ((int64_t)ms5837.C[5] * 256);
    TEMP = 2000 + ((dT * (int64_t)ms5837.C[6]) / 8388608);

    OFF = ((int64_t)ms5837.C[2] * 65536) + (((int64_t)ms5837.C[4] * dT) / 128);
    SENS = ((int64_t)ms5837.C[1] * 32768) + (((int64_t)ms5837.C[3] * dT) / 256);

    if (TEMP < 2000) {
        T2 = (3 * dT * dT) / 8589934592LL;
        OFF2 = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;

        if (TEMP < -1500) {
            OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    } else {
        T2 = (2 * dT * dT) / 137438953472LL;
        OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENS2 = 0;
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    P = (((ms5837.D1 * SENS) / 2097152) - OFF) / 8192;

    ms5837.TEMP = (int32_t)TEMP;
    ms5837.P = (int32_t)P;
}
