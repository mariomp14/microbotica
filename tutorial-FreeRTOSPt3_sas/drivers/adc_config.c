#include <drivers/adc_config.h>
#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t cola_adc;

void configADC_IniciaADC(void)
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
                   // Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
                   //CONFIGURAR SECUENCIADOR 0(YA QUE TIENE 8 posiciones de la fifo)8muestras
    ADCSequenceDisable(ADC0_BASE,0);

                   //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);//OJO:EL reloj de conversión e el mismo para los 2 ADC (ADC0 y ADC1).
                                                                        //Esta función no la puedo llamar con el argumento ADC1 siempre con el ADC0.
                   //ojo tengo que configurar con la secuencia 0
    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_TIMER,0);
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE,0,4,ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC0_BASE,0,5,ADC_CTL_CH5);
    ADCSequenceStepConfigure(ADC0_BASE,0,6,ADC_CTL_TS);//termometro
    ADCSequenceStepConfigure(ADC0_BASE,0,7,ADC_CTL_CH6|ADC_CTL_IE |ADC_CTL_END );    //La ultima muestra provoca la interrupcion, ojo este canal no lo utilizamos pero tenemos que ponerlo para rellenar las 8 posiciones
    ADCSequenceEnable(ADC0_BASE,0); //ACTIVO LA SECUENCIA

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE,0);//el 0 es el secuenciador
    ADCIntEnable(ADC0_BASE,0);
    //IntPrioritySet(INT_ADC0SS1,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_ADC0SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    //IntEnable(INT_ADC0SS1);
    IntEnable(INT_ADC0SS0);//interrupcion de secuenciador 0

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ui32Period = 0.5*SysCtlClockGet();
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
    TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
    TimerEnable(TIMER2_BASE, TIMER_A);
    //Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
    cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
    if (cola_adc==NULL)
    {
        while(1);
    }
}
void configADC_LeeADC(MuestrasADC *datos)
{
    xQueueReceive(cola_adc,datos,portMAX_DELAY);

}
void configADC_ISR(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    MuestrasLeidasADC leidas;
    MuestrasADC finales;
    ADCIntClear(ADC0_BASE,0);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDataGet(ADC0_BASE,0,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
    finales.chan1=leidas.chan1;
    finales.chan2=leidas.chan2;
    finales.chan3=leidas.chan3;
    finales.chan4=leidas.chan4;
    finales.temp=leidas.temp;
    finales.chan5=leidas.chan5;
    finales.chan6=leidas.chan6;

    //Guardamos en la cola
    xQueueSendFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

