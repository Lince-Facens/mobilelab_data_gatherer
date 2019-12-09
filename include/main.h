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




#endif /* MAIN_H_ */
