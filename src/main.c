/**
 ******************************************************************************
 * @file    mobilelab_data_gathering/main.cpp
 * @author  Emanuel Huber da Silva
 * @version V1.0
 * @date    06-August-2019
 * @brief   Main program body
 ******************************************************************************

 /* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR    ((uint32_t)0x4001244C)
#define ARRAYSIZE 3

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
uint32_t status;
__IO uint16_t ADC_values[ARRAYSIZE];
uint64_t timer = 0;
uint16_t Timer3Period = (uint16_t) 665;
uint8_t enableL = 1, enableR = 1;

/* Private function prototypes -----------------------------------------------*/
//void RCC_Configuration(void);
//void GPIO_Configuration(void);
//void NVIC_Configuration(void);
//void DMA_Configuration(void);
//void ADC_Configurationperiod(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
	SysTick_Config(SystemCoreClock / 1000000);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure PA.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PWM outputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

//	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	int PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = Timer3Period ;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

	while (1)
	{
//		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
//			GPIO_SetBits(GPIOA, GPIO_Pin_7);
//		else
//			GPIO_ResetBits(GPIOA, GPIO_Pin_7);
		enableL = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
		enableR = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);

		if (!enableL || !enableR) {
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 0);
		}

	}

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
