/**
 * Dispatches the data from the PPM interruptions to the PWM peripheral
 */

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

/* AUTONOMOUS MODE -------------------------------------------------------*/

#define AUTONOMOUS_MODE_WINDOW_SIZE 10 // The window size to switch to autonomous mode
#define AUTONOMOUS_MODE_TEST 0 // Whether we'll force the autonomous mode for testing purposes

/* CONTROLLER CALIBRATION ------------------------------------------------*/

#define CONTROLLER_ACCELERATION_CALIBRATE 0 // Whether it'll print out periods to calibrate the values below
#define CONTROLLER_STEERING_CALIBRATE 0 // Whether it'll print out periods to calibrate the values below

#define CONTROLLER_ACCELERATION_MAX_ERROR 10 // The error that a full acceleration may have
#define CONTROLLER_ACCELERATION_MIN_THRESHOLD 0.1 // Threshold (%) from stationary to accelerating

#define CONTROLLER_ACCELERATION_CENTER 1490 // Not accelerating
#define CONTROLLER_ACCELERATION_TOTAL_FORWARD 2090 // Fully accelerating forward
#define CONTROLLER_ACCELERATION_TOTAL_BACKWARD 900 // Fully accelerating backward

#define CONTROLLER_STEERING_MAX_ERROR 20 // The error that a full steerage may have
#define CONTROLLER_STEERING_MIN_THRESHOLD 0.3 // Threshold (%) from being centered to steering to a direction

#define CONTROLLER_STEERING_CENTER 1510 // Centered
#define CONTROLLER_STEERING_TOTAL_LEFT 2090 // Fully steering to the left
#define CONTROLLER_STEERING_TOTAL_RIGHT 910 // Fully steering to the right

#define CONTROLLER_TIMEOUT (configTICK_RATE_HZ / 5) // 200 ms
#define CONTROLLER_WARMUP (configTICK_RATE_HZ) // 1000 ms

/* FUNCTIONS -------------------------------------------------------------*/

/**
 * Updates the steering from the period (in microseconds) between each PPM interruption
 */
void updateControllerSteering(double diff, uint8_t enableL, uint8_t enableR);

/**
 * Updates the acceleration from the period (in microseconds) between each PPM interruption
 */
void updateControllerAcceleration(double diff);

/**
 * Updates the autonomous mode, enable left and enable right pins and forwards data from the AI system to the control system
 */
void updateAutonomousMode();

/**
 * Handles a timeout
 */
void handleControllerTimeout();

#endif /* DISPATCHER_H_ */
