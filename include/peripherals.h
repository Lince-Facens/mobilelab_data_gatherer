/*
 * Peripherals configurations definitions
 */

#include "stm32f10x.h"

#define LED	GPIO_Pin_13 // C13 (Output)
#define AUTONOMOUS_STEERING_PIN GPIO_Pin_2 // A2 (Input)
#define AUTONOMOUS_ACCELERATION_PIN GPIO_Pin_3 // A3 (Input)

#define ENABLE_LEFT_STEERING_PIN GPIO_Pin_12 // B12 (Input)
#define ENABLE_RIGHT_STEERING_PIN GPIO_Pin_13 // B13 (Input)
#define AUTONOMOUS_MODE_PIN GPIO_Pin_14 // B14 (Input)
#define REVERSE_ACCELERATION_PIN GPIO_Pin_15 // B15 (Output)
#define CONTROL_ENABLED_PIN GPIO_Pin_8 // A8 (Output)

#define ARRAYSIZE 2
#define ADC1_DR ((uint32_t)0x4001244C)

#define PWM_TIMER_PERIOD 665

__IO uint16_t ADC_values[ARRAYSIZE];


/* Public functions */
void setupPeripherals(void);

/* Private functions */
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void EXTI_Configuration(void);
void TIM_Configuration(void);
