/*
 * Main configuration constants
 */

#ifndef MAIN_H_
#define MAIN_H_


/* Private define */
#define AUTONOMOUS_STEERING_IDX 0
#define AUTONOMOUS_ACCELERATION_IDX 1
#define CONTROLLER_TIMEOUT (configTICK_RATE_HZ / 5) // 200 ms
#define CONTROLLER_WARMUP (configTICK_RATE_HZ) // 1000 ms

#define AUTONOMOUS_MODE_WINDOW_SIZE 10 // The window size to switch to autonomous mode
#define AUTONOMOUS_MODE_TEST 0 // Whether we'll force the autonomous mode for testing purposes

#endif /* MAIN_H_ */
