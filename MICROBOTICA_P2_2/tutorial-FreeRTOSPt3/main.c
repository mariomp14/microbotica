//*****************************************************************************
//
// TestSHARPLED: Testea el funcionamiento de los sensores IR SHARP
// Válido para cualquier tipo de sensor IR analógico
// La entrada de datos del sensor debe ir conectada al pin PE5
// Las otras dos entradas a tierra y Vcc (5V o 3.3V según indique el fabricante)
//
// Al acercar la mano/u otro objeto reflectante al sensor, el LED rojo debe brillar mas.
// Ten en cuenta que algunos sensores ofrecen un cierto voltaje incluso cuando no hay
// obstáculo, por lo que en ese caso el LED brillará un poco también.
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "drivers/rgb.h"
#include "driverlib/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/uart.h"

//Este es el divisor de 625khz(nuestra frecuencia de reloj) para que nos de un periodo de pwm de 50hz (20ms).
#define PERIOD_PWM 12500    // TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
//Lo mismo que el de arriba pero para 1 ms(frecuencia para velocidad maxima para una rueda, las dos ruedas van en sentido contrario)
#define COUNT_1MS 625   // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 962  // TODO: Ciclos para amplitud de pulso de parada (1.52ms) // El valor del motor izquierdo es 962 para que este neutro y 966 para el derecho.
#define STOPCOUNT2 966
#define COUNT_2MS 1250   // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms (a elegir)
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion
#define TASKPRIO 1           // Prioridad para la tarea LED1TASK
#define TASKSTACKSIZE 128    // TamaÃ±o de pila para la tarea LED1TASK

static QueueHandle_t cola_adc;

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




//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//#if 0
void vApplicationIdleHook( void )
{
        SysCtlSleep();
        //SysCtlDeepSleep();
}
//#endif
static portTASK_FUNCTION(ADCTask,pvParameters)
{
    MuestrasADC muestras;
//    MuestrasLeidasADC m;
    const uint32_t rojo[3]={0xFF00,0x0,0x0};
    const uint32_t verde[3]={0x0,0xFF00,0x0};
    int anterior = 0;
    int cont = -1;

   while(1)
   {
       xQueueReceive(cola_adc,&muestras,portMAX_DELAY);

      // m.chan1 = muestras.chan1;
       //HACER CON UN UMBRAL
       //EN BLANCO DA 200 Y NEGRO UNOS 800 LA MITAD SERIA 500

       if(abs(muestras.chan1 - anterior) >= 300 )
       {
           anterior = muestras.chan1;
           cont ++;
       }
       if(cont == 17)
       {
           UARTprintf("VUELTA");
           cont = 0;
       }


       UARTprintf("VALOR ADC : %d\n", muestras.chan1);
       UARTprintf("contador : %d\n", cont);
      // anterior = muestras.chan1;

       //APARTADO 1 PRACTICA 2.2
       /*  xQueueReceive(cola_adc,&muestras,portMAX_DELAY);
       if(muestras.chan1 <= 800) //detecta negro
       {
           RGBSet(verde,1.0);
       }
       else
       {
           RGBSet(rojo,1.0);
       }
       UARTprintf("VALOR ADC : %d\n", muestras.chan1);*/
      //RGBSet(verde,1.0);*/
   }
}


int main(void){

    uint32_t ui32Period;
    //Reloj del sistema a 40MHz procedente del PLL
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);    // Establece reloj del sistema (40MHz/64=625KHz)

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);    // PF2 como salida PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);    // PF2 como salida PWM
    GPIOPinConfigure(GPIO_PF3_M1PWM7);    // del módulo PWM1 (ver tabla 10.2 Data-Sheet, columnas 4 y 5)


    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM); //Aunque ponga 32 bits, lo maximo que permite son 16 bits.

    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);Esto es para que empiece el programa con el motor izquierdo parado
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS+350);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2); Esto es para que empiece el programa con el motor derecho parado
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_1MS+350);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);// Habilita la salida de la señal

    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Esta funcion habilita la interrupcion de la UART y le da la prioridad adecuada si esta activado el soporte para FreeRTOS
    UARTStdioConfig(0,115200,SysCtlClockGet());
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

                   //HABILITAMOS EL GPIOE
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
    ui32Period = 0.005*SysCtlClockGet();
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
    TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
    TimerEnable(TIMER2_BASE, TIMER_A);


    cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
    if (cola_adc==NULL)
    {
        while(1);
    }


    if((xTaskCreate(ADCTask, "ADC", TASKSTACKSIZE, NULL,tskIDLE_PRIORITY + TASKPRIO, NULL) != pdTRUE))
        {
            while(1)
            {
            }
        }


    vTaskStartScheduler();

    while(1);
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
