/*
 * File: main.c
 * Description: Application entry point with UART Redirection
 */

#include <xc.h>
#include <stdio.h>
#include "i2c_master.h"
#include "ms5837.h"

// Configuration Bits (Example for Internal FRC with PLL)
#pragma config FNOSC = FRCPLL   // Internal Fast RC with PLL
#pragma config FWDTEN = OFF     // Watchdog Timer Disabled

// Redirect printf to UART1
// This function is called by printf for every character.
int write(int handle, void *buffer, unsigned int len) {
    int i;
    char *pch = buffer;
    for (i = 0; i < len; i++) {
        while (U1STAbits.UTXBF); // Wait if TX Buffer is Full
        U1TXREG = *pch++;        // Send Character
    }
    return len;
}

int main(void) {
    // 1. Initialize System
    // (User's existing UART init code goes here)
    // UART_Init(9600); 
    
    printf("\r\n\r\n--- System Boot ---\r\n");
    printf("Initializing I2C Master...\r\n");
    
    // 2. Initialize Sensor
    if (MS5837_Init()) {
        printf("MS5837 Detected. PROM CRC Valid.\r\n");
        printf("Calibration C1: %u, C2: %u\r\n", ms5837.C, ms5837.C);
    } else {
        printf("ERROR: Sensor Init Failed.\r\n");
        printf("Check: 1. Pull-up Resistors (4.7k)\r\n");
        printf("       2. Wiring (SDA=RB9, SCL=RB8)\r\n");
        while(1); // Halt
    }

    // 3. Main Measurement Loop
    while(1) {
        MS5837_Read_Data();
        MS5837_Calculate();
        
        // Display Data
        // Pressure is in 0.1 mbar steps. 10130 = 1013.0 mbar.
        // Temperature is in 0.01 C steps. 2000 = 20.00 C.
        float press_mbar = ms5837.P / 10.0f;
        float temp_c = ms5837.TEMP / 100.0f;
        
        // Simple freshwater depth approx: (P - 1013) / 100
        // 1 bar (1000 mbar) ~= 10m water
        float depth_m = (press_mbar - 1013.0f) / 100.0f;
        
        printf("P: %.1f mbar | T: %.2f C | Depth: %.2f m\r\n", 
               (double)press_mbar, (double)temp_c, (double)depth_m);
        
        __delay_ms(1000); // 1 Hz Update Rate
    }
    
    return 0;
}