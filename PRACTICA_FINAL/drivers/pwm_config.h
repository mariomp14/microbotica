/*
 * pwm.h
 *
 *  Created on: 22 nov. 2024
 *      Author: usuario
 */

#ifndef DRIVERS_PWM_H_
#define DRIVERS_PWM_H_

#include<stdint.h>

//Este es el divisor de 625khz(nuestra frecuencia de reloj) para que nos de un periodo de pwm de 50hz (20ms).
#define PERIOD_PWM 12500    // TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
//Lo mismo que el de arriba pero para 1 ms(frecuencia para velocidad maxima para una rueda, las dos ruedas van en sentido contrario)
#define COUNT_1MS 625   // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 964  // TODO: Ciclos para amplitud de pulso de parada (1.52ms) // El valor del motor izquierdo es 962 para que este neutro y 966 para el derecho.
#define STOPCOUNT2 966
#define COUNT_2MS 1250   // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)


void config_pwm_motors(void);



#endif /* DRIVERS_PWM_H_ */
