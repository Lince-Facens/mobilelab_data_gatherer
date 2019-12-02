/**
 * Dispatches the data from the PPM interruptions to the PWM peripheral
 */

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

/* CALIBRATION -----------------------------------------------------------*/

#define ACCELERATION_CALIBRATE 0 // Whether it'll print out periods to calibrate the values below
#define STEERING_CALIBRATE 0 // Whether it'll print out periods to calibrate the values below

#define ACCELERATION_MAX_ERROR 10 // The error that a full acceleration may have
#define ACCELERATION_MIN_THRESHOLD 0.3 // Threshold (%) from stationary to accelerating

#define ACCELERATION_CENTER 810 // Not accelerating
#define ACCELERATION_TOTAL_FORWARD 1133 // Fully accelerating forward
#define ACCELERATION_TOTAL_BACKWARD 488 // Fully accelerating backward

#define STEERING_MAX_ERROR 10 // The error that a full steerage may have
#define STEERING_MIN_THRESHOLD 0.3 // Threshold (%) from being centered to steering to a direction

#define STEERING_CENTER 790 // Centered
#define STEERING_TOTAL_LEFT 1140 // Fully steering to the left
#define STEERING_TOTAL_RIGHT 493 // Fully steering to the right

/* FUNCTIONS -------------------------------------------------------------*/

/**
 * Updates the steering from the period (in microseconds) between each PPM interruption
 */
void updateSteering(double diff, uint8_t enableL, uint8_t enableR);

/**
 * Updates the acceleration from the period (in microseconds) between each PPM interruption
 */
void updateAcceleration(double diff);

#endif /* DISPATCHER_H_ */
