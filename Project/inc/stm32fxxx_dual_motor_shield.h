/**
  ******************************************************************************
  * @file    stm32fxxx_dual_motor_shield.h
  * @author  MikrocontrollerProjekte
  * 				 YouTube:	https://www.youtube.com/c/MikrocontrollerProjekte
  * 				 GitHub:	https://github.com/MikrocontollerProjekte/
  * @brief   Dual G2 High-Power Motor Driver 24v14 Shield
  ******************************************************************************
  * @attention: - remove Jumper ARDVIN from the motor shield for STM32F746G Disco!!!
  *  						- add external Pull Down Resistor (10k) between Enable Pins MxSLP and GND for programming and reset
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32FXXX_DUAL_MOTOR_SHIELD_H_
#define STM32FXXX_DUAL_MOTOR_SHIELD_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal.h"

/* Exported define -----------------------------------------------------------*/
// settings for 20 kHz
#define TIM_M1_PERIOD					(uint32_t)5399
#define TIM_M2_PERIOD					(uint32_t)10799

/* Definition for TIMx clock resources */
#define TIM_M1          			TIM2
#define TIM_M2               	TIM1
#define TIM_M1_IRQ          	TIM2_IRQn
#define TIM_M2_IRQ            TIM1_CC_IRQn
#define TIM_M1_ISR          	TIM2_IRQHandler
#define TIM_M2_ISR            TIM1_CC_IRQHandler
#define TIM_M1_CLK_ENABLE()		__HAL_RCC_TIM2_CLK_ENABLE()
#define TIM_M2_CLK_ENABLE()		__HAL_RCC_TIM1_CLK_ENABLE()
#define TIM_M1_AF							GPIO_AF1_TIM2
#define TIM_M2_AF							GPIO_AF1_TIM1
#define TIM_M1_CH							TIM_CHANNEL_1
#define TIM_M2_CH							TIM_CHANNEL_1

/* Definition for TIMx Channel Pins */
#define TIM_M1_CHANNEL_GPIO_PORT()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define TIM_M2_CHANNEL_GPIO_PORT()	__HAL_RCC_GPIOA_CLK_ENABLE()

// M1PWM - PWM Motor 1
#define M1PWM_Pin GPIO_PIN_15
#define M1PWM_GPIO_Port GPIOA

// M1DIR - Direction Motor 1
#define M1DIR_Pin GPIO_PIN_3
#define M1DIR_GPIO_Port GPIOI

// M2DIR - Direction Motor 2
#define M2DIR_Pin GPIO_PIN_2
#define M2DIR_GPIO_Port GPIOI

// M2PWM - PWM Motor 2
#define M2PWM_Pin GPIO_PIN_8
#define M2PWM_GPIO_Port GPIOA

// M2SLP - Enable Motor 2
#define M2SLP_Pin GPIO_PIN_7
#define M2SLP_GPIO_Port GPIOG

// M1SLP - Enable Motor 1
#define M1SLP_Pin GPIO_PIN_6
#define M1SLP_GPIO_Port GPIOG

// M2CS - Current Sense Outputs Motor 2
#define M2CS_Pin GPIO_PIN_10
#define M2CS_GPIO_Port GPIOF

// M1CS - Current Sense Outputs Motor 1
#define M1CS_Pin GPIO_PIN_0
#define M1CS_GPIO_Port GPIOA

// M1FLT - Fault indicators Motor 1
#define M1FLT_Pin GPIO_PIN_6
#define M1FLT_GPIO_Port GPIOH

// M2FLT - Fault indicators Motor 2
#define M2FLT_Pin GPIO_PIN_14
#define M2FLT_GPIO_Port GPIOB


/* Definition for ADCx clock resources */
#define ADC_MOTOR                          			ADC3
#define ADC_MOTOR_CLK_ENABLE()             			__HAL_RCC_ADC3_CLK_ENABLE()
#define DMA_MOTOR_CLK_ENABLE()             			__HAL_RCC_DMA2_CLK_ENABLE()
#define ADC_MOTOR_CHANNEL1_GPIO_CLOCK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define ADC_MOTOR_CHANNEL2_GPIO_CLOCK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define DMA_MOTOR_CHANNEL												DMA_CHANNEL_2
#define DMA_MOTOR_INSTANCE											DMA2_Stream0

/* Definition for ADCx's NVIC */
#define ADC_MOTOR_IRQn                         	ADC_IRQn
#define DMA_MOTOR_IRQ														DMA2_Stream0_IRQn

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	MOTOR_UNKNOWN = 0,
	MOTOR_1,
	MOTOR_2
}motor_t;

typedef enum
{
	CLOCKWISE = 0,
	COUNTERCLOCKWISE
}direction_t;

typedef enum
{
	DISABLE_MOTOR = 0,
  ENABLE_MOTOR
}status_t;

typedef enum
{
	FAULT_INACTIVE = 0,
	FAULT_ACTIVE
}fault_status_t;

typedef enum
{
  APP_DUAL_MOTOR_START,
	APP_DUAL_MOTOR_RAMP_UP,
  APP_DUAL_MOTOR_RUN,
	APP_DUAL_MOTOR_RAMP_DOWN,
	APP_DUAL_MOTOR_STOP,
	APP_DUAL_MOTOR_END
}app_dual_motor_states_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void init_dual_motor_shield(void);
void app_dual_motor_shield(void);
void Error_Handler_Motor(void);
void set_enable_disable_status(motor_t motor, status_t status);
void set_motor_dutycycle(motor_t motor, uint8_t dutycycle_percent);
void set_motor_direction(motor_t motor, direction_t direction);
fault_status_t get_motor_fault_status(motor_t motor);
uint32_t get_motor_current(motor_t motor);
uint8_t get_motor_dutycycle(motor_t motor);
status_t get_enable_disable_status(motor_t motor);
direction_t get_motor_direction(motor_t motor);
app_dual_motor_states_t get_motor_statemachine_state(void);

#endif /* STM32FXXX_DUAL_MOTOR_SHIELD_H_ */
