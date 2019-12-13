/**
 ******************************************************************************
 * @file    mobilelab_data_gathering/main.cpp
 * @author  Emanuel Huber da Silva
 * @version V1.0
 * @date    06-August-2019
 * @brief   Main program body
 ******************************************************************************
*/

 /* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

#include "peripherals.h"
#include "dispatcher.h"
#include "actions.h"

#include <math.h>

/* Private variables ---------------------------------------------------------*/
uint8_t enableL = 1, enableR = 1;
uint8_t autonomous_mode = 0;
uint32_t ppmChannel0 = 0, ppmChannel1 = 0, ppmLastTimeout = 0;

/* Private function prototypes -----------------------------------------------*/
static void prvPPM2PWM(void *pvParameters);
static void prvControllerTimeout(void *pvParameters);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief Converts the PPM signal to PWM
 */
static void prvPPM2PWM(void *pvParameters)
{
	while (1) {
		updateAutonomousMode();
	}
}

/**
 * @brief Checks whether the controller has been disconnected by timing out 200 ms after the last interruption
 */
static void prvControllerTimeout(void *pvParameters)
{
	TickType_t xNextWakeTime = xTaskGetTickCount();

	while (1) {

		if (autonomous_mode) {
			vTaskDelayUntil(&xNextWakeTime, CONTROLLER_TIMEOUT);
			continue;
		}

		TickType_t ticks = xTaskGetTickCount();

		if (ticks < ppmChannel0 && ticks < ppmChannel1 && ticks >= CONTROLLER_TIMEOUT) {
			// A tick overflow must have happened and it went back to zero
			handleControllerTimeout();
			ppmLastTimeout = ticks;

		} else {

			ticks -= CONTROLLER_TIMEOUT;

			if (ppmChannel0 < ticks && ppmChannel1 < ticks) {
				// Both tick counters have been halted for more than the timeout period
				handleControllerTimeout();
				ppmLastTimeout = ticks;
			}
		}

		vTaskDelayUntil(&xNextWakeTime, CONTROLLER_TIMEOUT);
	}
}

int main(void)
{
	// Initializes hardware
	setupPeripherals();

	setLedEnabled();
	setAccelerationForward();

	xTaskCreate(prvPPM2PWM, "PPM-to-PWM-Converter", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	xTaskCreate(prvControllerTimeout, "Controller-Timeout", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	while (1);

	return 0;
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif
