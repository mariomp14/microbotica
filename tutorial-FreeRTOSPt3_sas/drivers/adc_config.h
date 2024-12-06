#ifndef ADCCONFIG_H_
#define ADCCONFIG_H_

#include<stdint.h>

typedef struct
{
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t temp;
    uint16_t chan5;
    uint16_t chan6;
} MuestrasADC;

typedef struct
{
    uint32_t chan1;
    uint32_t chan2;
    uint32_t chan3;
    uint32_t chan4;
    uint32_t temp;
    uint32_t chan5;
    uint32_t chan6;
} MuestrasLeidasADC;


void configADC_ISR(void);
void configADC_LeeADC(MuestrasADC *datos);
void configADC_IniciaADC(void);


#endif
