
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
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
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/i2c.h"
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "inc/hw_ints.h" // Definiciones de interrupciones
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"
#include "event_groups.h"//necesario para utilizar flags de eventos
#include "semphr.h"              // FreeRTOS: definiciones para uso de semaforos

#include "drivers/adc_config.h"
#include "drivers/pwm_config.h"

#include "drivers/i2c.h"
#include "drivers/ssd1306.h"

#define RADIO_MITAD 1.4 //radio de las ruedas
#define RADIO 2.8 //radio de las ruedas
#define DIST_RUEDAS 9.5


#define TASKPRIO_ADC 1
#define TASKPRIO_PRINC 1
#define TASKPRIO_LINEA 1

#define TASKSTACKSIZE 128

static EventGroupHandle_t FlagsEventos;//flag de eventos
#define WHISKER 0x01  //(1<<0) // 0x0001
#define INT_ENCODER_DER 0x02  //(1<<0) // 0x0001
#define INT_ENCODER_IZQ 0x04  //(1<<0) // 0x0001
#define INT_SENSOR_LINEA_DELANTE 0x08  //(1<<0) // 0x0001
#define INT_SENSOR_LINEA_ATRAS 0x10
#define MOTOR_30KG_ENCENDER 0x20
#define MOTOR_30KG_APAGAR 0x40
#define INT_ENCODER_DER_MOV 0x80
#define INT_ENCODER_IZQ_MOV 0x100
#define INT_SENSOR_LINEA_ATRAS2 0x200
#define ESQUIVAR 0x400
//estado para la planificacion
#define ESTADO_CENTRO 1
#define ESTADO_BORDE 2
#define ESTADO_ATAQUE 3
#define ESTADO_BUSCAR_CENTRO 4
#define ESTADO_ESQUIVO 5
#define ESTADO_WHISKER_ATAQUE 6

int estado = ESTADO_BUSCAR_CENTRO; //Se empieza en buscar centro
int estado_anterior_LCD = -1;
//variables para controlar la distancia recorrida por el robot y el angulo de giro
volatile float angulo_der = 0.0;
volatile float angulo_izq = 0.0;
volatile float angulo_der_mov = 0.0;
volatile float angulo_izq_mov = 0.0;
volatile float distancia_recorrida = 0.0;
volatile float angulo_recorrido = 0.0;
// Variables globales de localización
float rc1 = 0;
float theta = 0;



//PUERTOS USADOS, PARA SABERLO PARA FUTURAS IMPLEMENTACIONES
//USADOS : PF2, PF3, PC4 --> PWM
//PF4 --> WHISKER
//PE2, PE5 --> ENCODERS
//PE3 --> ADC
//PA0, PA1 --> UART
//PE4, PE1 --> SENSOR DE LINEA
//PD0, PD1, PB6, PB7 --> I2C LCD


/*Tarea planificadora : Le llegan interrupciones, de whisker, sensor de linea y odometria(pero no los encoders porque estan dentro de esa funcion)
 * Estados --> Choca Sensor de linea de alante, Chocasolo derecho atras, solo izquierdo de atras y chocan los dos de atras.
 *         -->Choque contra el whisker
 *         --> Sensor de distancia detecta al otro(Activa el motor30kg y va a por el, hay que combinarlo con la odometria)
 *Tarea Motores: Le llega lo que decida la planificadora, para mover los motores como mas convenga. Aqui no llegan interrupciones,
 *               llegan otros flags, que estan por encima de los flags de interrrupciones.
 *  Flags motores --> Recto, "sobre si mismo"(para esquivar), Giro para mirar al centro.
 *
 *Para que detecte los dos de linea traseros, lo suyo es cuando detecte uno, mover el robot dependiendo de cual toco y
 * no limpiar el flag, hasta que se active el flag INT_SENSORES_LINEA_TRASEROS, para que cuando pille un solo sensor de linea trasero
 *  se quede activo ese flag hasta que toque tambien el otro de linea y asi se activa el flag INT_SENSORES_LINEA_TRASEROS, que es una OR
 *  de los dos, por lo tanto solo se activa cuando los bits de los dos sensores separados estan activos.
 *
 */
//para look up table es para buscar mas rapido la distancia que esta leyendo el sensor en relacion al valor del ADC
unsigned short B[8] = {40,35,30,25,20,15,10,5};
unsigned short A[8] = {320,368,460,536,712,1073,1667,3457};

void    ft_putchar_LCD(unsigned char data, int X, int Y)
{
    //write(1, &c, 1);
    OLED_sendCharXY( data,  X,  Y);
}

void    ft_putnbr(int nb,int X, int *Y)
{

    if (nb < 0)
    {
        ft_putchar_LCD('-', X,(*Y)++);

        if (nb == -2147483648)
        {
            ft_putchar_LCD('2',X,(*Y)++);

            nb = 147483648;
        }
        else
        {
            nb = -nb;
        }
    }
    if (nb >= 10)
    {
        ft_putnbr(nb / 10, X, Y);
    }
    ft_putchar_LCD ((nb % 10) + '0',X,(*Y)++);

}
void actualizar_odometria(float rc0)
{
    EventBits_t eventosMOTOR;
   //le pasamos como argumento la distancia inicial al centro (50cm)
    float angulo_izq_anterior = 0.0;
    float angulo_der_anterior = 0.0;
    float d_izq;
    float d_der;
    float d;
//    float delta_theta;
//    float drc;
    // Cambios en los ángulos
    float delta_angulo_izq;
    float delta_angulo_der;



    eventosMOTOR = xEventGroupWaitBits(FlagsEventos, INT_ENCODER_DER_MOV | INT_ENCODER_IZQ_MOV, pdFALSE, pdFALSE, portMAX_DELAY);

    if (eventosMOTOR & INT_ENCODER_DER_MOV) {
         xEventGroupClearBits(FlagsEventos, INT_ENCODER_DER_MOV);

         UARTprintf("INTERRUPCION ODOMETRIA DER\n");
         angulo_der_mov = angulo_der_mov + 60.0 * (M_PI / 180.0); // Convertir 60 grados a radiane

     }

     // Actualización del ángulo izquierdo
     if (eventosMOTOR & INT_ENCODER_IZQ_MOV) {
         xEventGroupClearBits(FlagsEventos, INT_ENCODER_IZQ_MOV);

         UARTprintf("INTERRUPCION ODOMETRIA IZQ\n");
         angulo_izq_mov = angulo_izq_mov + 60.0 * (M_PI / 180.0); // Convertir 60 grados a radianes


     }

     vTaskDelay(300);
    delta_angulo_izq = angulo_izq_mov - angulo_izq_anterior;
    delta_angulo_der = angulo_der_mov - angulo_der_anterior;
    // Guardar los valores actuales como anteriores
    angulo_izq_anterior = angulo_izq_mov;
    angulo_der_anterior = angulo_der_mov;

    // Calcular desplazamientos lineales
    d_izq = -RADIO * delta_angulo_izq;
    d_der = -RADIO * delta_angulo_der;

    // Calcular desplazamiento y cambio en orientación
    d = (d_izq + d_der) / 2.0; //Velocidad lineal promedio
    //delta_theta = (d_der - d_izq) / DIST_RUEDAS;//Velocidad angular del robot


    // Cambio en la distancia radial
    //drc = d * cos(theta) ;
    rc1 = rc0 + d; // Actualización de la distancia radial
    if(rc1 <= 0)
    {
        estado = ESTADO_CENTRO;
        rc1 = 0;
        angulo_der_mov=0;
        angulo_izq_mov=0;
    }
    //theta += delta_theta;//orientacion
    // Guardar el nuevo rc como estado inicial para la próxima iteración
     rc0 = rc1;
}


void mover_robot(float c) {
    EventBits_t eventosMOTOR;
//Controlamos motor mediante pwm, segund el tiempo del periodo que este ON o OFF
    // Configurar dirección de las ruedas en función de `c`
    if (c > 0) {  // Mover hacia adelante
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT - 25);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2 + 25);
    } else {  // Mover hacia atrás
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT + 10);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2 - 10);
    }

        while(abs(c) >= distancia_recorrida)
        {

            eventosMOTOR = xEventGroupWaitBits(FlagsEventos, INT_ENCODER_DER | INT_ENCODER_IZQ, pdFALSE, pdFALSE, portMAX_DELAY);

            // Actualización del ángulo derecho
            if (eventosMOTOR & INT_ENCODER_DER) {
                xEventGroupClearBits(FlagsEventos, INT_ENCODER_DER);

                UARTprintf("INTERRUPCION DER\n");
                angulo_der = angulo_der + 60.0 * (M_PI / 180.0); // Convertir 20 grados a radiane

            }

            // Actualización del ángulo izquierdo
            if (eventosMOTOR & INT_ENCODER_IZQ) {
                xEventGroupClearBits(FlagsEventos, INT_ENCODER_IZQ);

                UARTprintf("INTERRUPCION IZQ\n");
                angulo_izq = angulo_izq + 60.0 * (M_PI / 180.0); // Convertir 20 grados a radianes


            }

            // Calcular la distancia recorrida
            distancia_recorrida = (RADIO_MITAD * (angulo_der + angulo_izq));
        }


            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2);
            angulo_der=0.0;
            angulo_izq=0.0;
            distancia_recorrida = 0.0;
            vTaskDelay(500);


}

void girar_robot(float g)
{
    EventBits_t eventosMOTOR;

    if (g > 0) {  // Giro horario
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+10);
    } else {  // Giro antihorario
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2-10);
    }

    while(abs(g) >= abs(angulo_recorrido*(180.0 /M_PI )))
    {
            eventosMOTOR = xEventGroupWaitBits(FlagsEventos, INT_ENCODER_DER | INT_ENCODER_IZQ, pdFALSE, pdFALSE, portMAX_DELAY);

    // Actualización del ángulo derecho
    if (eventosMOTOR & INT_ENCODER_DER) {
        xEventGroupClearBits(FlagsEventos, INT_ENCODER_DER);

        UARTprintf("INTERRUPCION DER\n");
        angulo_der = angulo_der + 60.0 * (M_PI / 180.0); // Convertir 20 grados a radiane

    }

    // Actualización del ángulo izquierdo
    if (eventosMOTOR & INT_ENCODER_IZQ) {
        xEventGroupClearBits(FlagsEventos, INT_ENCODER_IZQ);

        UARTprintf("INTERRUPCION IZQ\n");
        angulo_izq = angulo_izq + 60.0 * (M_PI / 180.0); // Convertir 20 grados a radianes

    }

    angulo_recorrido = ((RADIO/DIST_RUEDAS) * (angulo_izq-angulo_der));
    }


        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2);
        angulo_der=0.0;
        angulo_izq=0.0;
        angulo_recorrido = 0.0;
        vTaskDelay(500);

}
unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax)
{
  unsigned int imid;

  while (imin < imax)
    {
      imid= (imin+imax)>>1;

      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
    }
    return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}



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
     unsigned short index;


     while(1)
     {

         configADC_LeeADC(&muestras); //Espera y lee muestras del ADC (BLOQUEANTE)

         UARTprintf("ADC : %d\n",muestras.chan1);

         //OLED_clearDisplay();
         //ft_putnbr(muestras.chan1,X,&Y);

         index = binary_lookup(A,muestras.chan1,0,7);

         UARTprintf("INDICE : %d\n", index);

         UARTprintf("sas con sus : %d\n", B[index]);
         if(B[index] <= B[5])
           {
             xEventGroupSetBits(FlagsEventos, MOTOR_30KG_ENCENDER);
           }
         else
         {
             xEventGroupSetBits(FlagsEventos, MOTOR_30KG_APAGAR);
         }
         if(B[index] < B[6])
           {
             xEventGroupSetBits(FlagsEventos, ESQUIVAR);
           }
    }


 }
 static portTASK_FUNCTION(PRINCIPALTask, pvParameters) {
     EventBits_t eventosMOTOR;



     while (1) {
         // Esperar por cualquier evento relevante
         eventosMOTOR = xEventGroupWaitBits(FlagsEventos,ESQUIVAR|WHISKER|MOTOR_30KG_ENCENDER|MOTOR_30KG_APAGAR,pdFALSE,pdFALSE,portMAX_DELAY);

         xEventGroupClearBits(FlagsEventos,ESQUIVAR|MOTOR_30KG_ENCENDER|MOTOR_30KG_APAGAR|WHISKER);


         // Evaluar los eventos recibidos usando switch-case
         switch (estado) {
             case ESTADO_CENTRO:

                 if (estado_anterior_LCD != ESTADO_CENTRO) {
                     OLED_clearDisplay();
                     OLED_sendStrXY("CENTRO CHILL", 1, 0);
                     estado_anterior_LCD = ESTADO_CENTRO; // Actualiza el indicador

                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT+15);
                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+15);
                 }

                 if(eventosMOTOR & MOTOR_30KG_ENCENDER)
                 {
                     estado = ESTADO_ATAQUE;
                 }

                 if(eventosMOTOR & WHISKER)
                 {
                     estado = ESTADO_WHISKER_ATAQUE;
                 }

                 break;

             case ESTADO_BORDE:                 //Este estado es para que cuando toque borde, este aqui , este estado como tal no hace nada, es solo para cuando la otra tarea
                                               //detecte que este en el borde, se quede aqui sin hacer nada y la otra tarea sea la prioridad.Hasta que los dos sensores traseros
                                                  //toquen la linea blanca, no se cambia de estado.
                if (estado_anterior_LCD != ESTADO_BORDE) {
                     OLED_clearDisplay();
                     OLED_sendStrXY("BORDE", 1, 0);
                     estado_anterior_LCD = ESTADO_BORDE;
                 }
                 break;

             case ESTADO_ATAQUE:

                 if (estado_anterior_LCD != ESTADO_ATAQUE) {
                     OLED_clearDisplay();
                     OLED_sendStrXY("ATACO", 1, 0);
                     estado_anterior_LCD = ESTADO_ATAQUE;

                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT-80);
                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+80);
                 }

                 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, COUNT_1MS-400);
                 vTaskDelay(1000);
                 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, COUNT_2MS+400);
                 vTaskDelay(1000);

                 if(eventosMOTOR & MOTOR_30KG_APAGAR)
                 {
                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT-30);
                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+30);
                     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, STOPCOUNT);
                 }

                 break;

             case ESTADO_BUSCAR_CENTRO:

                 if (estado_anterior_LCD != ESTADO_BUSCAR_CENTRO) {
                     OLED_clearDisplay();
                     OLED_sendStrXY("BUSCO CENTRO", 1, 0);
                     estado_anterior_LCD = ESTADO_BUSCAR_CENTRO;

                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT - 20);
                     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2 + 20);
                 }

                 actualizar_odometria(15);

                 if(eventosMOTOR & ESQUIVAR)
                 {
                  estado = ESTADO_ESQUIVO;
                 }
                 if(eventosMOTOR & WHISKER)
                 {
                     estado = ESTADO_WHISKER_ATAQUE;
                 }

                 break;
             case ESTADO_ESQUIVO:

                 if (estado_anterior_LCD != ESTADO_ESQUIVO) {
                      OLED_clearDisplay();
                      OLED_sendStrXY("ESQUIVO", 1, 0);
                      estado_anterior_LCD = ESTADO_ESQUIVO;

                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT+20); //Giro sobre si mismo 1 segundo y luego me voy a buscar borde para volver al centro
                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+20);

                      vTaskDelay(2000);

                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT - 30);
                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2 + 30);
                  }


                 break;
             case ESTADO_WHISKER_ATAQUE:    //Giro sobre si mismo 2 segundo mientras pego con el motor de 30kg y me voy recto

                 if (estado_anterior_LCD != ESTADO_WHISKER_ATAQUE) {
                      OLED_clearDisplay();
                      OLED_sendStrXY("WHISKER ATACO", 1, 0);
                      estado_anterior_LCD = ESTADO_WHISKER_ATAQUE;

                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT+20);
                      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+20);
                  }


                 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, COUNT_1MS-400);
                 vTaskDelay(1000);
                 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, COUNT_2MS+400);
                 vTaskDelay(1000);

                 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT - 30);
                 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2 + 30);

                 break;

             default:
                 OLED_clearDisplay();
                 OLED_sendStrXY("No detecto estados", 1, 0);
                 break;
         }

         vTaskDelay(100);
     }
 }
 static portTASK_FUNCTION(MOTORESTask,pvParameters)
  {
     EventBits_t eventosMOTOR;
      while(1)
      {

          eventosMOTOR = xEventGroupWaitBits(FlagsEventos,INT_SENSOR_LINEA_DELANTE|INT_SENSOR_LINEA_ATRAS|INT_SENSOR_LINEA_ATRAS2,pdFALSE,pdFALSE,portMAX_DELAY);

          xEventGroupClearBits(FlagsEventos,INT_SENSOR_LINEA_DELANTE);

          if(eventosMOTOR & INT_SENSOR_LINEA_DELANTE)
          {
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT+15);
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+15);
              estado = ESTADO_BORDE;
          }
          if((eventosMOTOR & INT_SENSOR_LINEA_ATRAS) && (eventosMOTOR & INT_SENSOR_LINEA_ATRAS2))
          {
              xEventGroupClearBits(FlagsEventos,INT_SENSOR_LINEA_ATRAS|INT_SENSOR_LINEA_ATRAS2);
              UARTprintf("TOCAN LOS DOS DE ATRAS\n");
              estado = ESTADO_BUSCAR_CENTRO;
          }
          if((eventosMOTOR & INT_SENSOR_LINEA_ATRAS) && !(eventosMOTOR & INT_SENSOR_LINEA_ATRAS2))
          {
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2-10);
              estado = ESTADO_BORDE;


          }
          if((eventosMOTOR & INT_SENSOR_LINEA_ATRAS2) && !(eventosMOTOR & INT_SENSOR_LINEA_ATRAS))
          {
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT+10);
              PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2);
              estado = ESTADO_BORDE;

          }

     }


  }



 int main(void){


    // Variables para el numero de ciclos de reloj para el periodo(20ms) y el ciclo de trabajo(entre  1y2 ms)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);   //40mhz  // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable (cantidades menores de 16bits)
    config_pwm_motors();

    I2C_3_config();

    OLED_Init();
    OLED_clearDisplay();

            // Configura pulsadores placa TIVA (int. por flanco de bajada)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);



    //Esta funcion habilita la interrupcion de la UART y le da la prioridad adecuada si esta activado el soporte para FreeRTOS
    UARTStdioConfig(0,115200,SysCtlClockGet());
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    configADC_IniciaADC();

    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0, GPIO_STRENGTH_2MA,  GPIO_PIN_TYPE_STD); //desactivo pullup y pulldown
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_2, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0);
    IntEnable(INT_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);

    ButtonsInit();
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);




    if((xTaskCreate(ADCTask, "ADC", TASKSTACKSIZE, NULL,tskIDLE_PRIORITY + TASKPRIO_ADC, NULL) != pdTRUE))
        {
            while(1)
            {
            }
        }
    if((xTaskCreate(PRINCIPALTask, "PRINCIPAL", TASKSTACKSIZE, NULL,tskIDLE_PRIORITY + TASKPRIO_PRINC, NULL) != pdTRUE))
        {
            while(1)
            {
            }
        }
    if((xTaskCreate(MOTORESTask, "MOTORES", TASKSTACKSIZE, NULL,tskIDLE_PRIORITY + TASKPRIO_LINEA, NULL) != pdTRUE))
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


}


void GPIOFIntHandler(void)
{

    BaseType_t higherPriorityTaskWoken=pdFALSE;
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    int32_t i32Status2 = GPIOIntStatus(GPIO_PORTE_BASE,GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0);


    static TickType_t last_time_left=0;
    TickType_t current_time_left;
    static TickType_t last_time_right=0;
    TickType_t current_time_right;
    static TickType_t last_time_linea=0;
    TickType_t current_time_linea;
    static TickType_t last_time_lineatras=0;
    TickType_t current_time_lineatras;
    static TickType_t last_time_lineatras2=0;
    TickType_t current_time_lineatras2;
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){//Whisker

        xEventGroupSetBitsFromISR(FlagsEventos, WHISKER,&higherPriorityTaskWoken);
    }

    if(((i32Status2 & GPIO_PIN_5) == GPIO_PIN_5))//encoder derecho
    {
        current_time_left= xTaskGetTickCountFromISR();

        if((current_time_left-last_time_left)>=(0.35*configTICK_RATE_HZ))//anitrrebotes?
        {
            xEventGroupSetBitsFromISR(FlagsEventos, INT_ENCODER_DER,&higherPriorityTaskWoken);
            xEventGroupSetBitsFromISR(FlagsEventos, INT_ENCODER_DER_MOV,&higherPriorityTaskWoken);
            /*
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_DELANTE);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_ATRAS);
            */
        }
        last_time_left=current_time_left;


    }

    if(((i32Status2 & GPIO_PIN_2) == GPIO_PIN_2))
    {
        current_time_right= xTaskGetTickCountFromISR();
        if((current_time_right-last_time_right)>=(0.35*configTICK_RATE_HZ)){

            xEventGroupSetBitsFromISR(FlagsEventos, INT_ENCODER_IZQ,&higherPriorityTaskWoken);
            xEventGroupSetBitsFromISR(FlagsEventos, INT_ENCODER_IZQ_MOV,&higherPriorityTaskWoken);
            /*
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_DELANTE);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_ATRAS);
*/
        }
        last_time_right=current_time_right;

    }

    if(((i32Status2 & GPIO_PIN_4) == GPIO_PIN_4))
    {
        current_time_linea= xTaskGetTickCountFromISR();
        if((current_time_linea-last_time_linea)>=(0.35*configTICK_RATE_HZ)){

            xEventGroupSetBitsFromISR(FlagsEventos, INT_SENSOR_LINEA_DELANTE,&higherPriorityTaskWoken);
            /*
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ);

*/

        }
        last_time_linea=current_time_linea;

    }
    if(((i32Status2 & GPIO_PIN_1) == GPIO_PIN_1))
    {
        current_time_lineatras= xTaskGetTickCountFromISR();
        if((current_time_lineatras-last_time_lineatras)>=(0.35*configTICK_RATE_HZ)){

            xEventGroupSetBitsFromISR(FlagsEventos, INT_SENSOR_LINEA_ATRAS,&higherPriorityTaskWoken);
            /*
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_DELANTE);
*/

        }
        last_time_lineatras=current_time_lineatras;

    }
    if(((i32Status2 & GPIO_PIN_0) == GPIO_PIN_0))
    {
        current_time_lineatras2= xTaskGetTickCountFromISR();
        if((current_time_lineatras2-last_time_lineatras2)>=(0.35*configTICK_RATE_HZ)){

            xEventGroupSetBitsFromISR(FlagsEventos, INT_SENSOR_LINEA_ATRAS2,&higherPriorityTaskWoken);
            /*
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_DER_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_ENCODER_IZQ_MOV);
            xEventGroupClearBitsFromISR(FlagsEventos,INT_SENSOR_LINEA_DELANTE);
*/


        }
        last_time_lineatras2=current_time_lineatras2;

    }


    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

