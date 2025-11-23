/*
 * File: i2c_master.c
 * Description: Low-level I2C driver for PIC24FJ64GA702 with Bus Recovery
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// System Frequency Definitions
// Adjust these to match your oscillator configuration (Configuration Bits)
#define FOSC    8000000UL      // 8 MHz Internal Oscillator
#define FCY     (FOSC/2)       // 4 MHz Instruction Cycle
#define FSCL    100000UL       // 100 kHz I2C Bus Speed

#include <libpic30.h>
#include "i2c_master.h"

/*
 * Function: I2C_Init
 * Description: Configures I2C1 module, Baud Rate, and Slew Rate
 */
void I2C_Init(void) {
    // 1. Disable I2C Module during configuration
    I2C1CONL = 0x0000;

    // 2. Calculate Baud Rate Generator (BRG)
    // Formula: ((FCY/FSCL) - (FCY/10000000)) - 1
    // For 4MHz FCY and 100kHz FSCL: ((40) - (0.4)) - 1 = ~39
    I2C1BRG = 39;

    // 3. Configure Slew Rate
    // DISSLW = 1: Slew rate control disabled (Standard Mode 100kHz)
    // DISSLW = 0: Slew rate control enabled (Fast Mode 400kHz)
    I2C1CONLbits.DISSLW = 1;

    // 4. Enable I2C Module
    I2C1CONLbits.I2CEN = 1;
}

/*
 * Function: I2C_Reset_Bus
 * Description: Manually toggles SCL to release a stuck SDA line.
 *              Implements the "9-clock" recovery sequence.
 */
void I2C_Reset_Bus(void) {
    // 1. Disable I2C to relinquish control of pins to GPIO module
    I2C1CONLbits.I2CEN = 0;

    // 2. Configure SCL1 (RB8) and SDA1 (RB9) as Inputs first
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;

    // 3. Check if Bus is actually stuck (SDA Low while SCL High)
    if (PORTBbits.RB9 == 0) {
        // 4. Configure SCL as Output to drive clock
        TRISBbits.TRISB8 = 0;

        // 5. Toggle SCL 9 times
        for (int i = 0; i < 9; i++) {
            LATBbits.LATB8 = 0; // Drive Low
            __delay_us(10);     // Half period (50kHz equiv)
            LATBbits.LATB8 = 1; // Drive High
            __delay_us(10);
        }

        // 6. Generate a STOP condition sequence manually
        TRISBbits.TRISB9 = 0;   // SDA Output
        LATBbits.LATB9 = 0;     // SDA Low
        __delay_us(10);
        LATBbits.LATB8 = 1;     // SCL High
        __delay_us(10);
        LATBbits.LATB9 = 1;     // SDA High (STOP)
        __delay_us(10);
    }

    // 7. Re-Initialize the I2C Module
    I2C_Init();
}

/*
 * Function: I2C_Start
 * Description: Generates Start Condition and waits for completion.
 */
void I2C_Start(void) {
    I2C1CONLbits.SEN = 1;       // Initiate Start
    while (I2C1CONLbits.SEN);   // Blocking wait for hardware clear
}

/*
 * Function: I2C_Stop
 * Description: Generates Stop Condition.
 */
void I2C_Stop(void) {
    I2C1CONLbits.PEN = 1;       // Initiate Stop
    while (I2C1CONLbits.PEN);   // Blocking wait
}

/*
 * Function: I2C_Write_Byte
 * Description: Sends 8 bits and returns ACK/NACK status.
 * Returns: true if ACK received, false if NACK.
 */
bool I2C_Write_Byte(uint8_t data) {
    I2C1TRN = data;             // Load buffer
    while (I2C1STATbits.TRSTAT);// Wait for transmission to complete
    return (I2C1STATbits.ACKSTAT == 0); // 0 = ACK, 1 = NACK
}

/*
 * Function: I2C_Read_Byte
 * Description: Reads 8 bits and sends ACK or NACK.
 * Params: ack - true to send ACK (request more data), false for NACK (end of stream)
 */
uint8_t I2C_Read_Byte(bool ack) {
    I2C1CONLbits.RCEN = 1;      // Enable Receive Mode
    while (I2C1CONLbits.RCEN);  // Wait for 8 bits

    uint8_t data = I2C1RCV;     // Read buffer

    // Acknowledge Sequence
    I2C1CONLbits.ACKDT = ack ? 0 : 1; // 0=ACK, 1=NACK
    I2C1CONLbits.ACKEN = 1;     // Start ACK sequence
    while (I2C1CONLbits.ACKEN); // Wait for ACK completion

    return data;
}
