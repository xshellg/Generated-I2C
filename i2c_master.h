#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <stdint.h>
#include <stdbool.h>

void I2C_Init(void);
void I2C_Reset_Bus(void);
void I2C_Start(void);
void I2C_Stop(void);
bool I2C_Write_Byte(uint8_t data);
uint8_t I2C_Read_Byte(bool ack);

#endif // I2C_MASTER_H
