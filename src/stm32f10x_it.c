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
#include "main.h"
#include <math.h>

uint64_t ppmChannel0 = 0, ppmChannel1 = 0;
uint32_t left, right;

extern uint64_t timer;
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
#define MAX_ACCELERATION_PPM 2100
#define MIN_ACCELERATION_PPM 1550
#define ACCELERATION_RANGE fabs(MAX_ACCELERATION_PPM - MIN_ACCELERATION_PPM)
#define ACCELERATION_MIN_THRESHOLD_PERC 0.1
#define ACCELERATION_MIN_THRESHOLD ACCELERATION_RANGE * ACCELERATION_MIN_THRESHOLD_PERC
#define MAX_REVERSE_PPM 900
#define MIN_REVERSE_PPM 1450
#define REVERSE_RANGE fabs(MIN_REVERSE_PPM - MAX_REVERSE_PPM)
#define REVERSE_MIN_THRESHOLD_PERC 0.1
#define REVERSE_MIN_THRESHOLD REVERSE_RANGE * REVERSE_MIN_THRESHOLD_PERC
#define MAX_STEERING_LEFT_PPM 2089
#define MIN_STEERING_LEFT_PPM 1450
#define LEFT_STEERING_RANGE_PPM fabs(MIN_STEERING_LEFT_PPM - MAX_STEERING_LEFT_PPM)
#define LEFT_STEERING_MIN_THRESHOLD_PERC 0.1
#define LEFT_STEERING_MIN_THRESHOLD LEFT_STEERING_RANGE_PPM * LEFT_STEERING_MIN_THRESHOLD_PERC
#define MAX_STEERING_RIGHT_PPM 900
#define MIN_STEERING_RIGHT_PPM 1450
#define RIGHT_STEERING_RANGE_PPM fabs(MAX_STEERING_RIGHT_PPM - MIN_STEERING_RIGHT_PPM)
#define RIGHT_STEERING_MIN_THRESHOLD_PERC 0.1
#define RIGHT_STEERING_MIN_THRESHOLD RIGHT_STEERING_RANGE_PPM * RIGHT_STEERING_MIN_THRESHOLD_PERC
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

/**0
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
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None#define REVERSE_ACCELERATION_PIN 14
 *
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	timer++;
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	int max_value = Timer3Period - 60;
	if (EXTI_GetITStatus(EXTI_Line0) != RESET && !autonomous_mode)
	{
		if (ppmChannel0 == 0)
		{
			ppmChannel0 = timer;
		}
		else
		{
			int actual = timer;
			volatile int diff = actual - ppmChannel0;

			if (diff < MAX_STEERING_LEFT_PPM) {
				// Turn right
				if (diff < MIN_STEERING_RIGHT_PPM && diff > MAX_STEERING_RIGHT_PPM && enableR && ( (MIN_STEERING_RIGHT_PPM - diff) > RIGHT_STEERING_MIN_THRESHOLD ))
				{
					volatile double valor = Timer3Period * ((MIN_STEERING_RIGHT_PPM - diff) / RIGHT_STEERING_RANGE_PPM);
					TIM_SetCompare1(TIM3, (valor >= Timer3Period) ? (Timer3Period - 1) :  valor);
					TIM_SetCompare2(TIM3, 0);
				}
				// Turn left
				else if (diff > MIN_STEERING_LEFT_PPM && diff < MAX_STEERING_LEFT_PPM && enableL &&  ((diff - MIN_STEERING_LEFT_PPM) > LEFT_STEERING_MIN_THRESHOLD ) )
				{
					volatile double valor = Timer3Period * ((diff - MIN_STEERING_LEFT_PPM) / LEFT_STEERING_RANGE_PPM);
					TIM_SetCompare2(TIM3, (valor >= Timer3Period) ? (Timer3Period-1) : valor);
					TIM_SetCompare1(TIM3, 0);
				}
				else {
					TIM_SetCompare1(TIM3, 0);
					TIM_SetCompare2(TIM3, 0);
				}
			}

			ppmChannel0 = actual;
		}

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/*
* @brief  This function handles External line 0 interrupt request.
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET && !autonomous_mode)
	{
		if (ppmChannel1 == 0)
		{
			ppmChannel1 = timer;
		}
		else
		{
			int actual = timer;
			volatile int diff = actual - ppmChannel1;

            if (diff < MAX_ACCELERATION_PPM && diff > MIN_ACCELERATION_PPM) {
				TIM_SetCompare3(TIM3, Timer3Period * ((diff - MIN_ACCELERATION_PPM) / ACCELERATION_RANGE));
				GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);
			}
			else if (diff < MIN_REVERSE_PPM && diff > MAX_REVERSE_PPM) {
				TIM_SetCompare3(TIM3, Timer3Period * ((diff - MAX_REVERSE_PPM) / REVERSE_RANGE));
				GPIO_SetBits(GPIOB, REVERSE_ACCELERATION_PIN);
			}

			ppmChannel1 = actual;
		}

		/* Clear the  EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
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
