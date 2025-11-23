#ifndef MS5837_H
#define MS5837_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t C[8];
    uint32_t D1;
    uint32_t D2;
    int32_t TEMP;
    int32_t P;
} MS5837_t;

extern MS5837_t ms5837;

bool MS5837_Init(void);
void MS5837_Read_Data(void);
void MS5837_Calculate(void);

#endif // MS5837_H
