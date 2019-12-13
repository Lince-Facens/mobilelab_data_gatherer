#include "actions.h"

#ifndef TEST

#include "peripherals.h"
#include "stm32f10x.h"

extern __IO uint16_t ADC_values[ADC_ARRAYSIZE];

void setSteeringRight(float value) {
	TIM_SetCompare1(TIM3, value * PWM_TIMER_PERIOD);
}
void resetSteeringRight() {
	TIM_SetCompare1(TIM3, 0);
}

void setSteeringLeft(float value) {
	TIM_SetCompare2(TIM3, value * PWM_TIMER_PERIOD);
}
void resetSteeringLeft() {
	TIM_SetCompare2(TIM3, 0);
}

void setAcceleration(float value) {
	TIM_SetCompare3(TIM3, value * PWM_TIMER_PERIOD);
}
void resetAcceleration() {
	TIM_SetCompare3(TIM3, 0);
}

void setAccelerationForward() {
	GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);
}
void setAccelerationBackward() {
	GPIO_SetBits(GPIOB, REVERSE_ACCELERATION_PIN);
}

void setControlEnabled() {
	GPIO_SetBits(GPIOA, CONTROL_ENABLED_PIN);
}
void setControlDisabled() {
	GPIO_ResetBits(GPIOA, CONTROL_ENABLED_PIN);
}

void setLedEnabled() {
	GPIO_ResetBits(GPIOC, LED);
}
void setLedDisabled() {
	GPIO_SetBits(GPIOC, LED);
}

uint8_t isAutonomousMode() {
	return GPIO_ReadInputDataBit(GPIOB, AUTONOMOUS_MODE_PIN);
}

uint8_t isSteeringLeftEnabled() {
	return GPIO_ReadInputDataBit(GPIOB, ENABLE_LEFT_STEERING_PIN);
}

uint8_t isSteeringRightEnabled() {
	return GPIO_ReadInputDataBit(GPIOB, ENABLE_RIGHT_STEERING_PIN);
}

uint16_t getAutonomousSteering() {
	return ADC_values[ADC_AUTONOMOUS_STEERING];
}

uint16_t getAutonomousAcceleration() {
	return ADC_values[ADC_AUTONOMOUS_ACCELERATION];
}

#else

float steeringRight = 0, steeringLeft = 0, acceleration = 0;
uint8_t accelerationReverse = 0, controlEnabled = 0;

void setSteeringRight(float value) {
	steeringRight = value;
}
void resetSteeringRight() {
	steeringRight = 0;
}

void setSteeringLeft(float value) {
	steeringLeft = value;
}
void resetSteeringLeft() {
	steeringLeft = 0;
}

void setAcceleration(float value) {
	acceleration = value;
}
void resetAcceleration() {
	acceleration = 0;
}

void setAccelerationForward() {
	accelerationReverse = 0;
}
void setAccelerationBackward() {
	accelerationReverse = 1;
}

void setControlEnabled() {
	controlEnabled = 1;
}
void setControlDisabled() {
	controlEnabled = 0;
}

void setLedEnabled() {
	// NOOP
}
void setLedDisabled() {
	// NOOP
}

uint8_t isAutonomousMode() {
	return 0;
}

uint8_t isSteeringLeftEnabled() {
	return 1;
}

uint8_t isSteeringRightEnabled() {
	return 1;
}

uint16_t getAutonomousSteering() {
	return 0;
}

uint16_t getAutonomousAcceleration() {
	return 0;
}

void displayOutput() {
	printf("Enabled\tRight\tLeft\tAcceleration\tReversed\n%i\t%.4f\t%.4f\t%.4f\t%i\n",
			controlEnabled, steeringRight, steeringLeft, acceleration, accelerationReverse);
}

#endif
