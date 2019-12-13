#include <stdlib.h>
#include <math.h>

#include "actions.h"
#include "dispatcher.h"

/* CONSTANTS ------------------------------------------------------------*/

#define MAX_ACCELERATION_PPM (CONTROLLER_ACCELERATION_TOTAL_FORWARD + CONTROLLER_ACCELERATION_MAX_ERROR)
#define MIN_ACCELERATION_PPM (CONTROLLER_ACCELERATION_CENTER + (CONTROLLER_ACCELERATION_TOTAL_FORWARD - CONTROLLER_ACCELERATION_CENTER) * CONTROLLER_ACCELERATION_MIN_THRESHOLD)
#define ACCELERATION_RANGE (CONTROLLER_ACCELERATION_TOTAL_FORWARD - CONTROLLER_ACCELERATION_MAX_ERROR - MIN_ACCELERATION_PPM)

#define MAX_REVERSE_PPM (CONTROLLER_ACCELERATION_CENTER + (CONTROLLER_ACCELERATION_TOTAL_BACKWARD - CONTROLLER_ACCELERATION_CENTER) * CONTROLLER_ACCELERATION_MIN_THRESHOLD)
#define MIN_REVERSE_PPM (CONTROLLER_ACCELERATION_TOTAL_BACKWARD - CONTROLLER_ACCELERATION_MAX_ERROR)
#define REVERSE_RANGE (CONTROLLER_ACCELERATION_TOTAL_BACKWARD + CONTROLLER_ACCELERATION_MAX_ERROR - MAX_REVERSE_PPM)

#define MAX_STEERING_LEFT_PPM (CONTROLLER_STEERING_TOTAL_LEFT + CONTROLLER_STEERING_MAX_ERROR)
#define MIN_STEERING_LEFT_PPM (CONTROLLER_STEERING_CENTER + (CONTROLLER_STEERING_TOTAL_LEFT - CONTROLLER_STEERING_CENTER) * CONTROLLER_STEERING_MIN_THRESHOLD)
#define LEFT_STEERING_RANGE (CONTROLLER_STEERING_TOTAL_LEFT - CONTROLLER_STEERING_MAX_ERROR - MIN_STEERING_LEFT_PPM)

#define MAX_STEERING_RIGHT_PPM (CONTROLLER_STEERING_CENTER + (CONTROLLER_STEERING_TOTAL_RIGHT - CONTROLLER_STEERING_CENTER) * CONTROLLER_STEERING_MIN_THRESHOLD)
#define MIN_STEERING_RIGHT_PPM (CONTROLLER_STEERING_TOTAL_RIGHT - CONTROLLER_STEERING_MAX_ERROR)
#define RIGHT_STEERING_RANGE (CONTROLLER_STEERING_TOTAL_RIGHT + CONTROLLER_STEERING_MAX_ERROR - MAX_STEERING_RIGHT_PPM)

/* EXTERNAL VARIABLES ---------------------------------------------------*/

extern uint8_t enableL, enableR, autonomous_mode;

/* FUNCTIONS ------------------------------------------------------------*/

#if CONTROLLER_STEERING_CALIBRATE

int steeringCount = 0;

void updateControllerSteering(double diff, uint8_t enableL, uint8_t enableR)
{
	if (++steeringCount > 10) {
		printf("Steering:\t%d us\n", (int) diff);
		steeringCount = 0;
	}
}

#else // CONTROLLER_STEERING_CALIBRATE

void updateControllerSteering(double diff, uint8_t enableL, uint8_t enableR)
{
	if (enableR && diff > MIN_STEERING_RIGHT_PPM && diff < MAX_STEERING_RIGHT_PPM) {
		// Turn right

		// Sets and limits the right steering
		setSteeringRight(fmin((diff - MAX_STEERING_RIGHT_PPM) / RIGHT_STEERING_RANGE, 1));
		resetSteeringLeft();

	} else if (enableL && diff > MIN_STEERING_LEFT_PPM && diff < MAX_STEERING_LEFT_PPM) {
		// Turn left

		// Sets and limits the left steering
		resetSteeringRight();
		setSteeringLeft(fmin((diff - MIN_STEERING_LEFT_PPM) / LEFT_STEERING_RANGE, 1));

	} else if (diff <= MIN_STEERING_LEFT_PPM && diff >= MAX_STEERING_RIGHT_PPM) {
		// No steering - We'll set it back to zero

		resetSteeringRight();
		resetSteeringLeft();

	}

	setControlEnabled();

	// In case the steering goes over the limits, we'll leave it as is, keeping the previous value
}

#endif // CONTROLLER_STEERING_CALIBRATE

#if CONTROLLER_ACCELERATION_CALIBRATE

int accelerationCount = 0;

void updateControllerAcceleration(double diff)
{
	if (++accelerationCount > 10) {
		printf("Acceleration:\t%d us\n", (int) diff);
		accelerationCount = 0;
	}
}

#else // CONTROLLER_ACCELERATION_CALIBRATE

void updateControllerAcceleration(double diff)
{
	// Checks whether it's in acceleration ranges

	if (diff < MAX_ACCELERATION_PPM && diff > MIN_ACCELERATION_PPM) {
		// Accelerate Forward

		// Updates the TIM peripheral with the acceleration percentage
		setAcceleration(fmin((diff - MIN_ACCELERATION_PPM) / ACCELERATION_RANGE, 1));
		// Resets the pin that indicates that this acceleration is backwards
		setAccelerationForward();

	} else if (diff > MIN_REVERSE_PPM && diff < MAX_REVERSE_PPM) {
		// Accelerate Backward

		// Updates the TIM peripheral with the acceleration percentage
		setAcceleration(fmin((diff - MAX_REVERSE_PPM) / REVERSE_RANGE, 1));
		// Sets the pin that indicates that this acceleration is backwards
		setAccelerationBackward();

	} else if (diff <= MIN_ACCELERATION_PPM && diff >= MAX_REVERSE_PPM) {
		// No acceleration - We'll set it back to zero

		resetAcceleration();
		setAccelerationForward();

	}

	setControlEnabled();

	// In case the acceleration goes over the limits, we'll leave it as is, keeping the previous value
}

#endif // CONTROLLER_ACCELERATION_CALIBRATE


int8_t autonomous_mode_count = 1;

void updateAutonomousMode()
{
	autonomous_mode = AUTONOMOUS_MODE_TEST || isAutonomousMode();
	//enableL = isSteeringLeftEnabled();
	//enableR = isSteeringRightEnabled();

	if (autonomous_mode) {
		autonomous_mode_count = fmin(autonomous_mode_count + 1, AUTONOMOUS_MODE_WINDOW_SIZE * 2);
	} else {
		autonomous_mode_count = fmax(autonomous_mode_count - 1, 0);
	}

	autonomous_mode = autonomous_mode_count >= AUTONOMOUS_MODE_WINDOW_SIZE;

	if (!enableL && !enableR) {

		resetSteeringRight();
		resetSteeringLeft();
		resetAcceleration();
		setAccelerationForward();
		setControlDisabled();

	} else if (autonomous_mode) {

		double steering_value = getAutonomousSteering();
		double acceleration_value = getAutonomousAcceleration();

		if (steering_value > 2068) {
			resetSteeringRight();
			setSteeringLeft(steering_value / 4095.0);
		} else if (steering_value < 2028) {
			setSteeringRight(-(steering_value - 2048)/2048.0);
			resetSteeringLeft();
		} else if (steering_value >= 2028 && steering_value <= 2068) {
			resetSteeringRight();
			resetSteeringLeft();
		}

		if (acceleration_value > 2090) {
			setAcceleration(acceleration_value / 4095.0);
			setAccelerationForward();
		} else if (acceleration_value < 2000) {
			setAcceleration(-(acceleration_value - 2048)/2048.0);
			setAccelerationBackward();
		} else if (acceleration_value >= 2000 && acceleration_value <= 2090) {
			resetAcceleration();
			setAccelerationForward();
		}

		setControlEnabled();
	}

}

void handleControllerTimeout()
{
	// Resets the acceleration and steering and disables the control pin
	setSteeringRight(0);
	setSteeringLeft(0);
	setAcceleration(0);
	setAccelerationForward();
	setControlDisabled();
}
