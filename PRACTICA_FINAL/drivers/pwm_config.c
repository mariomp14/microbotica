/*
 * pwm_config.c
 *
 *  Created on: 22 nov. 2024
 *      Author: usuario
 */

#include <stdint.h>
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
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "inc/hw_ints.h" // Definiciones de interrupciones
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"
#include "event_groups.h"//necesario para utilizar flags de eventos

#include "drivers/pwm_config.h"

void config_pwm_motors(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);    // Establece reloj del sistema (40MHz/64=625KHz)

        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);    // PF2 como salida PWM
        GPIOPinConfigure(GPIO_PF2_M1PWM6);

        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);    // PF2 como salida PWM
        GPIOPinConfigure(GPIO_PF3_M1PWM7);    // del módulo PWM1 (ver tabla 10.2 Data-Sheet, columnas 4 y 5)

        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);    // PB6 como salida PWM
        GPIOPinConfigure(GPIO_PB6_M0PWM0);     // del módulo PWM1 (ver tabla 10.2 Data-Sheet, columnas 4 y 5)

        PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM); //Aunque ponga 32 bits, lo maximo que permite son 16 bits.

        PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PERIOD_PWM); //Aunque ponga 32 bits, lo maximo que permite son 16 bits.

        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);Esto es para que empiece el programa con el motor izquierdo parado
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT-10);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
        PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2); Esto es para que empiece el programa con el motor derecho parado
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT2+10);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);// Habilita la salida de la señal

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, STOPCOUNT);  // Establece el ciclo de trabajo (en este caso, un porcentaje del valor máximo, el periodo)
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);// Habilita la salida de la señal


        PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM
        PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}


