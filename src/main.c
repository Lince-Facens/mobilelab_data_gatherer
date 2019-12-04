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

#include "stm32f10x.h"
#include "dispatcher.h"
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC_values[ARRAYSIZE];
uint8_t enableL = 0, enableR = 0;
uint8_t autonomous_mode = 0;
uint32_t ppmChannel0 = 0, ppmChannel1 = 0;

/* Private function prototypes -----------------------------------------------*/
void hardwareSetup(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void EXTI_Configuration(void);
void TIM_Configuration(void);
static void prvPPM2PWM(void *pvParameters);
static void prvControllerTimeout(void *pvParameters);
/* Private functions ---------------------------------------------------------*/

/* 
 * @brief  Configures TIM1 for PWM outputs
 * @param  None
 * @retval None
 */
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* Compute the prescaler value */
	int PrescalerValue = (uint16_t) (SystemCoreClock / (50000)) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = PWM_TIMER_PERIOD;
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

	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

/*
 * @brief  Configures ADC readings
 * @param  None
 * @retval None
 */
void ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ARRAYSIZE;

	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1));

	//Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*
 * @brief  Configures NVIC hardware interrupts
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * @brief  Configures EXTI hardware for interruptions 
 * @param  None
 * retval  None
 */
void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Connect EXTI0 Line to PA.01 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI1 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

}

/*
 * @brief  Configures DMA hardware
 * @param  None
 * @retval None
 */
void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_values;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 channel1 */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	//Enable DMA1 Channel transfer
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/*
 * @brief  Configures GPIO pins
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PA.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | AUTONOMOUS_STEERING_PIN | AUTONOMOUS_ACCELERATION_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PWM outputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PWM outputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ENABLE_LEFT_STEERING_PIN | ENABLE_RIGHT_STEERING_PIN | AUTONOMOUS_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = REVERSE_ACCELERATION_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*
 * @brief  Configures hardware and peripherals clock
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
	//SysTick_Config(SystemCoreClock / 1000000);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	/* Enable DMA1 clocks */
	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief  hardware setup function
 * @param  None
 * @retval None
 */
void hardwareSetup(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    DMA_Configuration();
    EXTI_Configuration();
    NVIC_Configuration();
    ADC_Configuration();
    TIM_Configuration();
}

/**
 * @brief Converts the PPM signal to PWM
 */
static void prvPPM2PWM(void *pvParameters)
{
	while (1)
	{
		autonomous_mode = GPIO_ReadInputDataBit(GPIOB, AUTONOMOUS_MODE_PIN);
		enableL = GPIO_ReadInputDataBit(GPIOB, ENABLE_LEFT_STEERING_PIN);
		enableR = GPIO_ReadInputDataBit(GPIOB, ENABLE_RIGHT_STEERING_PIN);

		if (!autonomous_mode && !enableL && !enableR) {
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 0);
			GPIO_ResetBits(GPIOC, LED);
		}
		else {
			GPIO_SetBits(GPIOC, LED);

		    if (autonomous_mode) {
			    double steering_value     = ADC_values[AUTONOMOUS_STEERING_IDX];
			    double acceleration_value = ADC_values[AUTONOMOUS_ACCELERATION_IDX];

			    if (steering_value > 2068) {
			    	TIM_SetCompare1(TIM3, PWM_TIMER_PERIOD * steering_value / 4095.0);
			    	TIM_SetCompare2(TIM3, 0);
			    }
			    else if (steering_value < 2028){
			    	TIM_SetCompare2(TIM3, PWM_TIMER_PERIOD * -(steering_value - 2048)/2048.0);
				    TIM_SetCompare1(TIM3, 0);
		    	}

			    if (acceleration_value > 400) {
			    	TIM_SetCompare3(TIM3, PWM_TIMER_PERIOD * acceleration_value/4095.0);
			    }
		    }
		}

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

		} else {

			ticks -= CONTROLLER_TIMEOUT;

			if (ppmChannel0 < ticks && ppmChannel1 < ticks) {
				handleControllerTimeout();
			}
		}

		vTaskDelayUntil(&xNextWakeTime, CONTROLLER_TIMEOUT);
	}
}

int main(void)
{
	// Initializes hardware
	hardwareSetup();

	GPIO_ResetBits(GPIOC, LED);
	GPIO_ResetBits(GPIOB, REVERSE_ACCELERATION_PIN);

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
