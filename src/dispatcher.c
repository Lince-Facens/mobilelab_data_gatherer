#include <stdlib.h>
#include <math.h>

#include "stm32f10x_it.h"
#include "peripherals.h"
#include "dispatcher.h"

/* CONSTANTS ------------------------------------------------------------*/

#define MAX_ACCELERATION_PPM (ACCELERATION_TOTAL_FORWARD + ACCELERATION_MAX_ERROR)
#define MIN_ACCELERATION_PPM (ACCELERATION_CENTER + (ACCELERATION_TOTAL_FORWARD - ACCELERATION_CENTER) * ACCELERATION_MIN_THRESHOLD)
#define ACCELERATION_RANGE (ACCELERATION_TOTAL_FORWARD - ACCELERATION_MAX_ERROR - MIN_ACCELERATION_PPM)

#define MAX_REVERSE_PPM (ACCELERATION_CENTER + (ACCELERATION_TOTAL_BACKWARD - ACCELERATION_CENTER) * ACCELERATION_MIN_THRESHOLD)
#define MIN_REVERSE_PPM (ACCELERATION_TOTAL_BACKWARD - ACCELERATION_MAX_ERROR)
#define REVERSE_RANGE (ACCELERATION_TOTAL_BACKWARD + ACCELERATION_MAX_ERROR - MAX_REVERSE_PPM)

#define MAX_STEERING_LEFT_PPM (STEERING_TOTAL_LEFT + STEERING_MAX_ERROR)
#define MIN_STEERING_LEFT_PPM (STEERING_CENTER + (STEERING_TOTAL_LEFT - STEERING_CENTER) * STEERING_MIN_THRESHOLD)
#define LEFT_STEERING_RANGE (STEERING_TOTAL_LEFT - STEERING_MAX_ERROR - MIN_STEERING_LEFT_PPM)

#define MAX_STEERING_RIGHT_PPM (STEERING_CENTER + (STEERING_TOTAL_RIGHT - STEERING_CENTER) * STEERING_MIN_THRESHOLD)
#define MIN_STEERING_RIGHT_PPM (STEERING_TOTAL_RIGHT - STEERING_MAX_ERROR)
#define RIGHT_STEERING_RANGE (STEERING_TOTAL_RIGHT + STEERING_MAX_ERROR - MAX_STEERING_RIGHT_PPM)

/* FUNCTIONS ------------------------------------------------------------*/

#if STEERING_CALIBRATE

int steeringCount = 0;

void updateSteering(double diff, uint8_t enableL, uint8_t enableR)
{
	if (++steeringCount > 10) {
		printf("Steering:\t%d us\n", (int) diff);
		steeringCount = 0;
	}
}

#else // STEERING_CALIBRATE

void updateSteering(double diff, uint8_t enableL, uint8_t enableR)
{
	if (enableR && diff > MIN_STEERING_RIGHT_PPM && diff < MAX_STEERING_RIGHT_PPM) {
		// Turn right

		// Sets and limits the right steering
		TIM_SetCompare1(TIM3, PWM_TIMER_PERIOD * fmin((diff - MAX_STEERING_RIGHT_PPM) / RIGHT_STEERING_RANGE, 1));
		TIM_SetCompare2(TIM3, 0);

	} else if (enableL && diff > MIN_STEERING_LEFT_PPM && diff < MAX_STEERING_LEFT_PPM) {
		// Turn left

		// Sets and limits the left steering
		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, PWM_TIMER_PERIOD * fmin((diff - MIN_STEERING_LEFT_PPM) / LEFT_STEERING_RANGE, 1));

	} else if (diff <= MIN_STEERING_LEFT_PPM && diff >= MAX_STEERING_RIGHT_PPM) {
		// No steering - We'll set it back to zero

		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, 0);

	}

	// In case the steering goes over the limits, we'll leave it as is, keeping the previous value
}

#endif // STEERING_CALIBRATE

#if ACCELERATION_CALIBRATE

int accelerationCount = 0;

void updateAcceleration(double diff)
{
	if (++accelerationCount > 10) {
		printf("Acceleration:\t%d us\n", (int) diff);
		accelerationCount = 0;
	}
}

#else // ACCELERATION_CALIBRATE

void updateAcceleration(double diff)
{
	// Checks whether it's in acceleration ranges

	if (diff < MAX_ACCELERATION_PPM && diff > MIN_ACCELERATION_PPM) {
		// Accelerate Forward

		// Updates the TIM peripheral with the acceleration percentage
		TIM_SetCompare3(TIM3, PWM_TIMER_PERIOD * fmin((diff - MIN_ACCELERATION_PPM) / ACCELERATION_RANGE, 1));
		// Resets the pin that indicates that this acceleration is backwards
		GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);

	} else if (diff > MIN_REVERSE_PPM && diff < MAX_REVERSE_PPM) {
		// Accelerate Backward

		// Updates the TIM peripheral with the acceleration percentage
		TIM_SetCompare3(TIM3, PWM_TIMER_PERIOD * fmin((diff - MAX_REVERSE_PPM) / REVERSE_RANGE, 1));
		// Sets the pin that indicates that this acceleration is backwards
		GPIO_SetBits(GPIOB, REVERSE_ACCELERATION_PIN);

	} else if (diff <= MIN_ACCELERATION_PPM && diff >= MAX_REVERSE_PPM) {
		// No acceleration - We'll set it back to zero

		TIM_SetCompare3(TIM3, 0);
		GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);

	}

	// In case the acceleration goes over the limits, we'll leave it as is, keeping the previous value
}

#endif // ACCELERATION_CALIBRATE

void handleControllerTimeout()
{
	// Resets the acceleration and steering
	TIM_SetCompare1(TIM3, 0);
	TIM_SetCompare2(TIM3, 0);
	TIM_SetCompare3(TIM3, 0);
}
