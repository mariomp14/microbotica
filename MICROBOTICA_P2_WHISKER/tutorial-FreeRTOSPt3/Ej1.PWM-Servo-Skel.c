/* ************************************************************************** */
/* UNIVERSIDAD DE MALAGA               DEPARTAMENTO DE TECNOLOGIA ELECTRONICA */
/* http://www.uma.es                                    http://www.dte.uma.es */
/* ========================================================================== */
/* PROGRAMA :  PWM-Servo                                                      */
/* VERSION  : 1.0                                                             */
/* TARGET   : Kit  TIVA Launchpad IDE CCSv                                    */
/*
   DESCRIPCIÓN:
    Genera dos ondas PWM a 50Hz (20ms) por los pines PF2 y PF3.
    La PWM por PF3 se pondrá (de momento) con un ciclo de trabajo fijo de 
	1.5ms (parado)
    La PWM por PF2 se inicia con un ciclo de trabajo de 1.5ms (STOPCOUNT, 
	servo parado o prácticamente parado)
    Al pulsar los botones el ciclo aumenta/disminuye en CYCLE_ INCREMENTS 
    Los márgenes teóricos del servo están entre 1ms-2m (COUNT_1MS-COUNT_2MS)

    Si nada mas empezar el servo esta prácticamente parado (o se para con 
	2-3 pulsaciones de uno u otro botón), se
    considera calibrado. Si no, se tendrá que abrir y calibrar mediante el 
	potenciómetro interno.

  **************************************************************************   */
#include <stdint.h>
#include <stdbool.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h" 
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h" // Libreria GPIO
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "inc/hw_ints.h" // Definiciones de interrupciones
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"
#include "event_groups.h"//necesario para utilizar flags de eventos
// Incluir librerias de periférico y otros que se vaya a usar para control PWM y gestión
// de botones  (TODO)

//Este es el divisor de 625khz(nuestra frecuencia de reloj) para que nos de un periodo de pwm de 50hz (20ms).
#define PERIOD_PWM 12500	// TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
//Lo mismo que el de arriba pero para 1 ms(frecuencia para velocidad maxima para una rueda, las dos ruedas van en sentido contrario)
#define COUNT_1MS 625   // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 962  // TODO: Ciclos para amplitud de pulso de parada (1.52ms) // El valor del motor izquierdo es 962 para que este neutro y 966 para el derecho.
#define STOPCOUNT2 966
#define COUNT_2MS 1250   // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms (a elegir)
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion

#define TASKPRIO 1           // Prioridad para la tarea LED1TASK
#define TASKSTACKSIZE 128    // TamaÃ±o de pila para la tarea LED1TASK

static EventGroupHandle_t FlagsEventos;//flag de eventos
#define MOTOR_DER 0x01  //(1<<0) // 0x0001




uint32_t ui32DutyCycle = STOPCOUNT; //motor izquierdo para pararse
uint32_t   ui32DutyCycle2 = STOPCOUNT2; //motor derecho para pararse


//3.b (mitad de velocidad)

uint32_t DutyCycle_mitadvelocidad_izq = (COUNT_2MS+STOPCOUNT)/2; //
uint32_t DutyCycle_mitadvelocidad_der = (COUNT_1MS+STOPCOUNT2)/2; //

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
 static portTASK_FUNCTION(MOTORTask,pvParameters)
 {
     EventBits_t eventosMOTOR;
     while(1)
     {
         eventosMOTOR = xEventGroupWaitBits(FlagsEventos,MOTOR_DER,pdFALSE,pdFALSE,portMAX_DELAY);
         xEventGroupClearBits(FlagsEventos,MOTOR_DER);
         if (eventosMOTOR & MOTOR_DER)
         {
             xEventGroupClearBits(FlagsEventos,MOTOR_DER);



            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT-20);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS);//Esto es para que gire sobre

            vTaskDelay(1000);

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, DutyCycle_mitadvelocidad_der);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, DutyCycle_mitadvelocidad_izq);
         }

         }


    }


int main(void){

    // Variables para el numero de ciclos de reloj para el periodo(20ms) y el ciclo de trabajo(entre  1y2 ms)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);   //40mhz  // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable (cantidades menores de 16bits)
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
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, DutyCycle_mitadvelocidad_izq);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2); Esto es para que empiece el programa con el motor derecho parado
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, DutyCycle_mitadvelocidad_der);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);// Habilita la salida de la señal

    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM
            // Configura pulsadores placa TIVA (int. por flanco de bajada)

    ButtonsInit();
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);
  // Configuracion  ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1520us para salida por PF2, y COUNT_STOP para salida por PF3(o puedes poner otro valor si quieres que se mueva)
  	// Opcion 1: Usar un Timer en modo PWM (ojo! Los timers PWM solo soportan cuentas 
    //  de 16 bits, a menos que uséis un prescaler/timer extension)
  	// Opcion 2: Usar un módulo PWM(no dado en Sist. Empotrados pero mas sencillo)
    if((xTaskCreate(MOTORTask, "MOTOR", TASKSTACKSIZE, NULL,tskIDLE_PRIORITY + TASKPRIO, NULL) != pdTRUE))
        {
            while(1)
            {
            }
        }

    FlagsEventos = xEventGroupCreate();
     if( FlagsEventos == NULL )
     {
         while(1);
     }
    vTaskStartScheduler();

    while(1);

  // Codigo principal, (poner en bucle infinito o bajo consumo)
}

// Rutinas de interrupción de pulsadores
// Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
// Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS
void GPIOFIntHandler(void)
{

    BaseType_t higherPriorityTaskWoken=pdFALSE;
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    // Boton Izquierdo: reduce ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MINCOUNT
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){

        xEventGroupSetBitsFromISR(FlagsEventos, MOTOR_DER,higherPriorityTaskWoken);
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}
