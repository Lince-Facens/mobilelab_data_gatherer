#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "main.h"
#include "dispatcher.h"

uint32_t ppmPrevDiff0 = 0, ppmPrevDiff1 = 0;

extern uint32_t ppmChannel0;
extern uint32_t ppmChannel1;
extern uint32_t ppmLastTimeout;
extern uint8_t enableL, enableR;
extern uint8_t autonomous_mode;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 *
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) == RESET || autonomous_mode) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		return;
	}

	TickType_t timeElapsed = xTaskGetTickCount();

	// Avoids calculating the difference on the first run
	if (ppmChannel0 > 0 && timeElapsed - CONTROLLER_WARMUP >= ppmLastTimeout) {

		int diff = timeElapsed - ppmChannel0;

		// Avoids using the longest part of the wave
		if (diff > 0 && ppmPrevDiff0 > diff) {
			updateSteering(diff * (1000000.0 / configTICK_RATE_HZ), enableL, enableR);
		}

		ppmPrevDiff0 = diff;
	}

	ppmChannel0 = timeElapsed;

	// Clears the EXTI line 0 pending bit
	EXTI_ClearITPendingBit(EXTI_Line0);
}

/*
* @brief  This function handles External line 1 interrupt request.
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) == RESET || autonomous_mode) {
		EXTI_ClearITPendingBit(EXTI_Line1);
		return;
	}

	TickType_t timeElapsed = xTaskGetTickCount();

	// Avoids calculating the difference on the first run
	if (ppmChannel1 > 0 && timeElapsed - CONTROLLER_WARMUP >= ppmLastTimeout) {

		int diff = timeElapsed - ppmChannel1;

		// Avoids using the longest part of the wave
		if (diff > 0 && ppmPrevDiff1 > diff) {
			updateAcceleration(diff * (1000000.0 / configTICK_RATE_HZ));
		}

		ppmPrevDiff1 = diff;

	}

	ppmChannel1 = timeElapsed;

	// Clears the EXTI line 1 pending bit
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
 {
 }*/

