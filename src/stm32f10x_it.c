/**
 ******************************************************************************
 * @file    USART/HyperTerminal_HwFlowControl/stm32f10x_it.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "main.h"
#include <math.h>

uint32_t ppmChannel0 = 0, ppmChannel1 = 0;
uint32_t ppmPrevDiff0 = 0, ppmPrevDiff1 = 0;
uint32_t left, right;

extern uint16_t Timer3Period;
extern uint8_t enableL, enableR;
extern uint8_t autonomous_mode;
extern uint32_t adc_init_status;

/** @addtogroup STM32F10x_StdPeriph_Examples
 * @{
 */

/** @addtogroup USART_HyperTerminal_HwFlowControl
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define ACCELERATION_MAX_ERROR 10
#define ACCELERATION_MIN_THRESHOLD 0.3

#define ACCELERATION_CENTER 810
#define ACCELERATION_TOTAL_FORWARD 1133
#define ACCELERATION_TOTAL_BACKWARD 488

#define STEERING_MAX_ERROR 10
#define STEERING_MIN_THRESHOLD 0.3

#define STEERING_CENTER 790
#define STEERING_TOTAL_LEFT 1140
#define STEERING_TOTAL_RIGHT 493


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

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

void updateSteering(double diff)
{
	if (enableR && diff > MIN_STEERING_RIGHT_PPM && diff < MAX_STEERING_RIGHT_PPM) {
		// Turn right

		// Sets and limits the right steering
		TIM_SetCompare1(TIM3, Timer3Period * fmin((diff - MAX_STEERING_RIGHT_PPM) / RIGHT_STEERING_RANGE, 1));
		TIM_SetCompare2(TIM3, 0);

	} else if (enableL && diff > MIN_STEERING_LEFT_PPM && diff < MAX_STEERING_LEFT_PPM) {
		// Turn left

		// Sets and limits the left steering
		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, Timer3Period * fmin((diff - MIN_STEERING_LEFT_PPM) / LEFT_STEERING_RANGE, 1));

	} else if (diff <= MIN_STEERING_LEFT_PPM && diff >= MAX_STEERING_RIGHT_PPM) {
		// No steering - We'll set it back to zero

		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, 0);

	}

	// In case the steering goes over the limits, we'll leave it as is, keeping the previous value
}

void updateAcceleration(double diff) {
	// Checks whether it's in acceleration ranges

	if (diff < MAX_ACCELERATION_PPM && diff > MIN_ACCELERATION_PPM) {
		// Accelerate Forward

		// Updates the TIM peripheral with the acceleration percentage
		TIM_SetCompare3(TIM3, Timer3Period * fmin((diff - MIN_ACCELERATION_PPM) / ACCELERATION_RANGE, 1));
		// Resets the pin that indicates that this acceleration is backwards
		GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);

	} else if (diff > MIN_REVERSE_PPM && diff < MAX_REVERSE_PPM) {
		// Accelerate Backward

		// Updates the TIM peripheral with the acceleration percentage
		TIM_SetCompare3(TIM3, Timer3Period * fmin((diff - MAX_REVERSE_PPM) / REVERSE_RANGE, 1));
		// Sets the pin that indicates that this acceleration is backwards
		GPIO_SetBits(GPIOB, REVERSE_ACCELERATION_PIN);

	} else if (diff <= MIN_ACCELERATION_PPM && diff >= MAX_REVERSE_PPM) {
		// No acceleration - We'll set it back to zero

		TIM_SetCompare3(TIM3, 0);

	}

	// In case the acceleration goes over the limits, we'll leave it as is, keeping the previous value
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) == RESET || autonomous_mode) return;

	int timeElapsed = xTaskGetTickCount();

	// Avoids calculating the difference on the first run
	if (ppmChannel0 > 0) {

		int diff = timeElapsed - ppmChannel0;

		// Avoids using the longest part of the wave
		if (diff > 0 && ppmPrevDiff0 > diff) {
			updateSteering(diff * (1000000.0 / configTICK_RATE_HZ));
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
	if (EXTI_GetITStatus(EXTI_Line1) == RESET || autonomous_mode) return;

	int timeElapsed = xTaskGetTickCount();

	// Avoids calculating the difference on the first run
	if (ppmChannel1 > 0) {

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

/*
* @brief  This function handles DMA1 interrupt.
* @param  None
* @retval None
*/
void DMA1_Channel1_IRQHandler(void)
{
	//Test on DMA1 Channel1 Transfer Complete interrupt
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		adc_init_status = 1;
		//Clear DMA1 interrupt pending bits
		DMA_ClearITPendingBit(DMA1_IT_GL1);
	}
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

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
