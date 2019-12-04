/*
 * main.h
 *
 *  Created on: 5 de nov de 2019
 *      Author: Emanuel Huber
 */

#ifndef MAIN_H_
#define MAIN_H_


/* Private define */
#define ADC1_DR    ((uint32_t)0x4001244C)
#define ARRAYSIZE 4
#define LED	GPIO_Pin_13
#define REVERSE_ACCELERATION_PIN GPIO_Pin_15
#define AUTONOMOUS_MODE_PIN GPIO_Pin_14
#define AUTONOMOUS_STEERING_PIN GPIO_Pin_2
#define AUTONOMOUS_ACCELERATION_PIN GPIO_Pin_3
#define AUTONOMOUS_STEERING_IDX 0
#define AUTONOMOUS_ACCELERATION_IDX 1
#define ENABLE_LEFT_STEERING_PIN GPIO_Pin_12
#define ENABLE_RIGHT_STEERING_PIN GPIO_Pin_13
#define PWM_TIMER_PERIOD 665
#define CONTROLLER_TIMEOUT (configTICK_RATE_HZ / 5) // 200 ms




#endif /* MAIN_H_ */
