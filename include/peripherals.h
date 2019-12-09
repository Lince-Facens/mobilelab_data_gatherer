/*
 * Peripherals configurations definitions
 */

#define LED	GPIO_Pin_13 // C13
#define AUTONOMOUS_STEERING_PIN GPIO_Pin_2
#define AUTONOMOUS_ACCELERATION_PIN GPIO_Pin_3
#define AUTONOMOUS_MODE_PIN GPIO_Pin_14
#define ENABLE_LEFT_STEERING_PIN GPIO_Pin_12
#define ENABLE_RIGHT_STEERING_PIN GPIO_Pin_13
#define REVERSE_ACCELERATION_PIN GPIO_Pin_15 // B15

#define ARRAYSIZE 2
#define ADC1_DR ((uint32_t)0x4001244C)

#define PWM_TIMER_PERIOD 665

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
