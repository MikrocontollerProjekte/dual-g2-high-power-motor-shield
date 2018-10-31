/**
  ******************************************************************************
  * @file    stm32fxxx_dual_motor_shield.c
  * @author  MikrocontrollerProjekte
  * 				 YouTube:	https://www.youtube.com/c/MikrocontrollerProjekte
  * 				 GitHub:	https://github.com/MikrocontollerProjekte/
  * @brief   Dual G2 High-Power Motor Driver 24v14 Shield
  ******************************************************************************
  * @attention: - remove Jumper ARDVIN from the motor shield for STM32F746G Disco!!!
  *  						- add external Pull Down Resistor (10k) between Enable Pins MxSLP and GND for programming and reset
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32fxxx_dual_motor_shield.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim_m1;
TIM_HandleTypeDef htim_m2;
TIM_OC_InitTypeDef sConfig;

ADC_HandleTypeDef hadc_motor;
DMA_HandleTypeDef hdma_adc_motor;
volatile uint32_t dma_buffer[2] = {0,0};
volatile uint32_t adc_buffer[2] = {0,0};
volatile uint32_t adc_value[2] = {0,0};

app_dual_motor_states_t app_dual_motor_state = APP_DUAL_MOTOR_START;
direction_t direction_m1 = CLOCKWISE;
direction_t direction_m2 = CLOCKWISE;
status_t status_m1 = DISABLE_MOTOR;
status_t status_m2 = DISABLE_MOTOR;
uint8_t dutycycle_m1 = 0;
uint8_t dutycycle_m2 = 0;
volatile uint32_t current_calibration_m1 = 0;
volatile uint32_t current_calibration_m2 = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void init_motor_gpio_pins(void);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void MX_TIM_M1_Init(void);
void MX_TIM_M2_Init(void);
void MX_ADC_motor_Init(void);
void Error_Handler_Motor(void);

/* Exported functions --------------------------------------------------------*/


/**
  * @brief  Initializes the Dual G2 High-Power Motor Driver 24v14 Shield
  * @param  None
  * @retval None
  */
void init_dual_motor_shield(void)
{
	// init GPIO pins
	init_motor_gpio_pins();

	// Disable Drivers
	set_enable_disable_status(MOTOR_1, DISABLE_MOTOR);
	set_enable_disable_status(MOTOR_2, DISABLE_MOTOR);

	// set directions
	set_motor_direction(MOTOR_1, CLOCKWISE);
	set_motor_direction(MOTOR_2, CLOCKWISE);

	// init Timer Clocks and Timer GPIOs
	HAL_TIM_PWM_MspInit(&htim_m1);
	HAL_TIM_PWM_MspInit(&htim_m2);

	// init ADC, ADC Clocks and ADC GPIOs
	hadc_motor.Instance = ADC_MOTOR;
	HAL_ADC_MspInit(&hadc_motor);
	MX_ADC_motor_Init();

	// init Timer
	MX_TIM_M1_Init();
	MX_TIM_M2_Init();

	// set dutycycle to zero
	set_motor_dutycycle(MOTOR_1, 0);
	set_motor_dutycycle(MOTOR_2, 0);

	// check motor faults
	if(get_motor_fault_status(MOTOR_1) == FAULT_ACTIVE)
	{
		//Error_Handler_Motor();
	}
	if(get_motor_fault_status(MOTOR_2) == FAULT_ACTIVE)
	{
		//Error_Handler_Motor();
	}
}


/**
  * @brief  Run application for Dual G2 High-Power Motor Driver 24v14 Shield
  * @param  None
  * @retval None
  */
void app_dual_motor_shield(void)
{
	static uint8_t duty_ramp = 8;
	static uint32_t time_count = 0;

	switch(app_dual_motor_state)
	{
		case APP_DUAL_MOTOR_START:
			// set Dutycycle
			set_motor_dutycycle(MOTOR_1, 0);
			set_motor_dutycycle(MOTOR_2, 0);

			// set directions
			set_motor_direction(MOTOR_1, CLOCKWISE);
			set_motor_direction(MOTOR_2, CLOCKWISE);

			// Enable Motor
			set_enable_disable_status(MOTOR_1, ENABLE_MOTOR);
			set_enable_disable_status(MOTOR_2, ENABLE_MOTOR);

			app_dual_motor_state = APP_DUAL_MOTOR_RAMP_UP;
			break;
		case APP_DUAL_MOTOR_RAMP_UP:
			// set Dutycycle
			set_motor_dutycycle(MOTOR_1, duty_ramp);
			set_motor_dutycycle(MOTOR_2, duty_ramp);

			duty_ramp = duty_ramp + 1;

			if(duty_ramp == 4)
			{
				// measurement of calibration values on startup with active driver
				current_calibration_m1 = get_motor_current(MOTOR_1);
				current_calibration_m2 = get_motor_current(MOTOR_2);
			}

			if(duty_ramp > 40)
			{
				app_dual_motor_state = APP_DUAL_MOTOR_RUN;
			}
			break;
		case APP_DUAL_MOTOR_RUN:
			time_count = time_count + 1;

			if(time_count > 100)
			{
				app_dual_motor_state = APP_DUAL_MOTOR_RAMP_DOWN;
			}
			break;
		case APP_DUAL_MOTOR_RAMP_DOWN:
			// set Dutycycle
			set_motor_dutycycle(MOTOR_1, duty_ramp);
			set_motor_dutycycle(MOTOR_2, duty_ramp);

			duty_ramp = duty_ramp - 1;

			if(duty_ramp < 8)
			{
				app_dual_motor_state = APP_DUAL_MOTOR_STOP;
			}
			break;
		case APP_DUAL_MOTOR_STOP:
			// set dutycycle to zero
			set_motor_dutycycle(MOTOR_1, 0);
			set_motor_dutycycle(MOTOR_2, 0);

			// Disable Drivers
			set_enable_disable_status(MOTOR_1, DISABLE_MOTOR);
			set_enable_disable_status(MOTOR_2, DISABLE_MOTOR);

			app_dual_motor_state = APP_DUAL_MOTOR_END;
			break;
		case APP_DUAL_MOTOR_END:
			break;
		default: // should never come here
		 break;
	}
}


/**
  * @brief  set motor stage enable or disable
  * @param  motor_t motor 1 or 2, status_t status enable or disable
  * @retval None
  */
void set_enable_disable_status(motor_t motor, status_t status)
{
	if(motor == MOTOR_1)
	{
		status_m1 = status;

		if(status == DISABLE_MOTOR)
		{
			HAL_GPIO_WritePin(M1SLP_GPIO_Port, M1SLP_Pin, GPIO_PIN_RESET);
		}
		else if(status == ENABLE_MOTOR)
		{
			HAL_GPIO_WritePin(M1SLP_GPIO_Port, M1SLP_Pin, GPIO_PIN_SET);
		}
	}
	else if(motor == MOTOR_2)
	{
		status_m2 = status;

		if(status == DISABLE_MOTOR)
		{
			HAL_GPIO_WritePin(M2SLP_GPIO_Port, M2SLP_Pin, GPIO_PIN_RESET);
		}
		else if(status == ENABLE_MOTOR)
		{
			HAL_GPIO_WritePin(M2SLP_GPIO_Port, M2SLP_Pin, GPIO_PIN_SET);
		}
	}
}


/**
  * @brief  set motor dutycycle
  * @param  motor_t motor 1 or 2, uint8_t dutycycle_percent
  * @retval None
  */
void set_motor_dutycycle(motor_t motor, uint8_t dutycycle_percent)
{
	uint8_t duty = 0;

	if(dutycycle_percent > 100)
	{
		duty = (uint8_t)100;
	}
	else
	{
		duty = dutycycle_percent;
	}

	if(motor == MOTOR_1)
	{
		dutycycle_m1 = duty;

	  /* Set the pulse value for TIM_M1_CH */
	  sConfig.Pulse = (uint32_t)(TIM_M1_PERIOD * duty / 100);
	  if(HAL_TIM_PWM_ConfigChannel(&htim_m1, &sConfig, TIM_M1_CH) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler_Motor();
	  }
	  if(HAL_TIM_PWM_Start(&htim_m1, TIM_M1_CH) != HAL_OK)
	  {
	    /* PWM Generation Error */
	    Error_Handler_Motor();
	  }
	}
	else if(motor == MOTOR_2)
	{
		dutycycle_m2 = duty;

	  /* Set the pulse value for TIM_M2_CH */
	  sConfig.Pulse = (uint32_t)(TIM_M2_PERIOD * duty / 100);
	  if(HAL_TIM_PWM_ConfigChannel(&htim_m2, &sConfig, TIM_M2_CH) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler_Motor();
	  }
	  if(HAL_TIM_PWM_Start(&htim_m2, TIM_M2_CH) != HAL_OK)
	  {
	    /* PWM Generation Error */
	    Error_Handler_Motor();
	  }
	}
}


/**
  * @brief  set motor direction
  * @param  motor_t motor 1 or 2, direction_t direction CLOCKWISE or COUNTERCLOCKWISE
  * @retval None
  */
void set_motor_direction(motor_t motor, direction_t direction)
{
	if(motor == MOTOR_1)
	{
		direction_m1 = direction;

		if(direction == CLOCKWISE)
		{
			HAL_GPIO_WritePin(M1DIR_GPIO_Port, M1DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction == COUNTERCLOCKWISE)
		{
			HAL_GPIO_WritePin(M1DIR_GPIO_Port, M1DIR_Pin, GPIO_PIN_SET);
		}
	}
	else if(motor == MOTOR_2)
	{
		direction_m2 = direction;

		if(direction == CLOCKWISE)
		{
			HAL_GPIO_WritePin(M2DIR_GPIO_Port, M2DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction == COUNTERCLOCKWISE)
		{
			HAL_GPIO_WritePin(M2DIR_GPIO_Port, M2DIR_Pin, GPIO_PIN_SET);
		}
	}
}


/**
  * @brief  Initializes the GPIO peripheral.
  * @param  None
  * @retval None
  */
void init_motor_gpio_pins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2DIR_GPIO_Port, M1DIR_Pin|M2DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2SLP_GPIO_Port, M2SLP_Pin|M1SLP_Pin, GPIO_PIN_RESET);

  // M1DIR - Direction Motor 1
  // M2DIR - Direction Motor 2
  /*Configure GPIO pins : M1DIR_Pin M2DIR_Pin */
  GPIO_InitStruct.Pin = M1DIR_Pin|M2DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(M2DIR_GPIO_Port, &GPIO_InitStruct);

  // M1SLP - Enable Motor 1
  // M2SLP - Enable Motor 2
  /*Configure GPIO pins : M2SLP_Pin M1SLP_Pin */
  GPIO_InitStruct.Pin = M2SLP_Pin|M1SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(M2SLP_GPIO_Port, &GPIO_InitStruct);

  // M1FLT - Fault indicators Motor 1
  /*Configure GPIO pin : M1FLT_Pin */
  GPIO_InitStruct.Pin = M1FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(M1FLT_GPIO_Port, &GPIO_InitStruct);

  // M2FLT - Fault indicators Motor 2
  /*Configure GPIO pin : M2FLT_Pin */
  GPIO_InitStruct.Pin = M2FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(M2FLT_GPIO_Port, &GPIO_InitStruct);
}


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  if(htim->Instance == TIM_M2)
  {
		/* TIMx Peripheral clock enable */
		TIM_M2_CLK_ENABLE();

		/* Enable GPIO Channels Clock */
		TIM_M2_CHANNEL_GPIO_PORT();

    /* TIM1 GPIO Configuration */
		GPIO_InitStruct.Pin = M2PWM_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = TIM_M2_AF;
		HAL_GPIO_Init(M2PWM_GPIO_Port, &GPIO_InitStruct);
  }
  else if(htim->Instance == TIM_M1)
  {
		/* TIMx Peripheral clock enable */
		TIM_M1_CLK_ENABLE();

		/* Enable GPIO Channels Clock */
		TIM_M1_CHANNEL_GPIO_PORT();

		/* TIM2 GPIO Configuration */
		GPIO_InitStruct.Pin = M1PWM_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = TIM_M1_AF;
		HAL_GPIO_Init(M1PWM_GPIO_Port, &GPIO_InitStruct);
  }
}


/**
  * @brief  Initializes the Timer of Motor 1
  * @param  None
  * @retval None
  */
void MX_TIM_M1_Init(void)
{
  htim_m1.Instance = TIM_M1;
  htim_m1.Init.Prescaler         = 0;
  htim_m1.Init.Period            = TIM_M1_PERIOD;
  htim_m1.Init.ClockDivision     = 0;
  htim_m1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_m1.Init.RepetitionCounter = 0;
  htim_m1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_PWM_Init(&htim_m1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler_Motor();
  }

  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for TIM_M1_CH */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&htim_m1, &sConfig, TIM_M1_CH) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler_Motor();
  }

  // TIM PWM Interrupt
  HAL_NVIC_SetPriority(TIM_M1_IRQ, 1, 0);
  HAL_NVIC_EnableIRQ(TIM_M1_IRQ);
  HAL_TIM_PWM_Start_IT(&htim_m1, TIM_M1_CH);
}


/**
  * @brief  Initializes the Timer of Motor 2
  * @param  None
  * @retval None
  */
void MX_TIM_M2_Init(void)
{
  htim_m2.Instance = TIM_M2;
  htim_m2.Init.Prescaler         = 0;
  htim_m2.Init.Period            = TIM_M2_PERIOD;
  htim_m2.Init.ClockDivision     = 0;
  htim_m2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_m2.Init.RepetitionCounter = 0;
  htim_m2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_PWM_Init(&htim_m2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler_Motor();
  }

  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for TIM_M2_CH */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&htim_m2, &sConfig, TIM_M2_CH) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler_Motor();
  }

  // TIM PWM Interrupt
  HAL_NVIC_SetPriority(TIM_M2_IRQ, 1, 0);
  HAL_NVIC_EnableIRQ(TIM_M2_IRQ);
  HAL_TIM_PWM_Start_IT(&htim_m2, TIM_M2_CH);
}


/**
  * @brief  get motor fault status
  * @param  motor_t motor 1 or 2, direction_t direction CLOCKWISE or COUNTERCLOCKWISE
  * @retval fault_status_t FAULT_ACTIVE or FAULT_INACTIVE
  */
fault_status_t get_motor_fault_status(motor_t motor)
{
	if(motor == MOTOR_1)
	{
		if(HAL_GPIO_ReadPin(M1FLT_GPIO_Port, M1FLT_Pin) == GPIO_PIN_RESET)
		{
			return FAULT_ACTIVE;
		}
		else
		{
			return FAULT_INACTIVE;
		}
	}
	else if(motor == MOTOR_2)
	{
		if(HAL_GPIO_ReadPin(M2FLT_GPIO_Port, M2FLT_Pin) == GPIO_PIN_RESET)
		{
			return FAULT_ACTIVE;
		}
		else
		{
			return FAULT_INACTIVE;
		}
	}
	else
	{
		Error_Handler_Motor();
		return 0;
	}
}


/**
  * @brief  get motor current in mA (20 mV/A)
  * @param  motor_t motor 1 or 2
  * @retval uint32_t motor current in mA
  */
uint32_t get_motor_current(motor_t motor)
{
	uint32_t temp_current = 0;

	if(motor == MOTOR_1)
	{
		temp_current = (uint32_t)(((adc_value[1] * 3300) / 4096) * 50);
		if(	 (temp_current > current_calibration_m1)
			 &&(get_enable_disable_status(MOTOR_1) == ENABLE_MOTOR)
		)
		{
			// The current is measured as peak current in the high state of PWM.
			// With the PWM dutycycle value an average current can be calculated.
			// This is only an approximation.
			// For exact current measurement, the RMS value must be integrated.
			temp_current = (temp_current - current_calibration_m1) / 100 * (uint32_t)get_motor_dutycycle(MOTOR_1);
			return temp_current;
		}
		else
		{
			return 0;
		}
	}
	else if(motor == MOTOR_2)
	{
		temp_current = (uint32_t)(((adc_value[0] * 3300) / 4096) * 50);
		if(  (temp_current > current_calibration_m2)
			 &&(get_enable_disable_status(MOTOR_2) == ENABLE_MOTOR)
		)
		{
			// The current is measured as peak current in the high state of PWM.
			// With the PWM dutycycle value an average current can be calculated.
			// This is only an approximation.
			// For exact current measurement, the RMS value must be integrated.
			temp_current = (temp_current - current_calibration_m2) / 100 * (uint32_t)get_motor_dutycycle(MOTOR_2);
			return temp_current;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		Error_Handler_Motor();
		return 0;
	}
}


/**
  * @brief  get motor dutycycle in percent
  * @param  motor_t motor 1 or 2
  * @retval uint8_t dutycycle in percent
  */
uint8_t get_motor_dutycycle(motor_t motor)
{
	if(motor == MOTOR_1)
	{
		return dutycycle_m1;
	}
	else if(motor == MOTOR_2)
	{
		return dutycycle_m2;
	}
	else
	{
		Error_Handler_Motor();
		return 0;
	}
}


/**
  * @brief  get enable / disable status
  * @param  motor_t motor 1 or 2
  * @retval status_t ENABLE / DISABLE
  */
status_t get_enable_disable_status(motor_t motor)
{
	if(motor == MOTOR_1)
	{
		return status_m1;
	}
	else if(motor == MOTOR_2)
	{
		return status_m2;
	}
	else
	{
		Error_Handler_Motor();
		return 0;
	}
}


/**
  * @brief  get motor direction
  * @param  motor_t motor 1 or 2
  * @retval direction_t CW / CCW
  */
direction_t get_motor_direction(motor_t motor)
{
	if(motor == MOTOR_1)
	{
		return direction_m1;
	}
	else if(motor == MOTOR_2)
	{
		return direction_m2;
	}
	else
	{
		Error_Handler_Motor();
		return 0;
	}
}


/**
  * @brief  get actual state of application statemachine
  * @param  None
  * @retval app_dual_motor_states_t 	actual state
  */
app_dual_motor_states_t get_motor_statemachine_state(void)
{
	return app_dual_motor_state;
}


/**
  * @brief  ADC_MOTOR init function
  * @param  None
  * @retval None
  */
void MX_ADC_motor_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)*/
  hadc_motor.Instance = ADC_MOTOR;
  hadc_motor.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc_motor.Init.Resolution = ADC_RESOLUTION_12B;
  hadc_motor.Init.ScanConvMode = ENABLE;
  hadc_motor.Init.ContinuousConvMode = ENABLE;
  hadc_motor.Init.DiscontinuousConvMode = DISABLE;
  hadc_motor.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_motor.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc_motor.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_motor.Init.NbrOfConversion = 2;
  hadc_motor.Init.DMAContinuousRequests = ENABLE;
  hadc_motor.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if(HAL_ADC_Init(&hadc_motor) != HAL_OK)
  {
    Error_Handler_Motor();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.*/
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if(HAL_ADC_ConfigChannel(&hadc_motor, &sConfig) != HAL_OK)
  {
    Error_Handler_Motor();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.*/
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if(HAL_ADC_ConfigChannel(&hadc_motor, &sConfig) != HAL_OK)
  {
    Error_Handler_Motor();
  }

  HAL_ADC_Start_DMA(&hadc_motor, (uint32_t*)dma_buffer, 2);
}


/**
  * @brief  ADC_MOTOR GPIO and Clock init function
  * @param  ADC_HandleTypeDef for ADChandle
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(hadc->Instance == ADC_MOTOR)
	{
		/* Peripheral clock enable */
		ADC_MOTOR_CLK_ENABLE();

	  /* DMA controller clock enable */
		DMA_MOTOR_CLK_ENABLE();
		ADC_MOTOR_CHANNEL1_GPIO_CLOCK_ENABLE();
		ADC_MOTOR_CHANNEL2_GPIO_CLOCK_ENABLE();

		// ADC GPIOs
		GPIO_InitStruct.Pin = M2CS_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(M2CS_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = M1CS_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(M1CS_GPIO_Port, &GPIO_InitStruct);

    /* ADC_MOTOR DMA Init */
    hdma_adc_motor.Instance = DMA_MOTOR_INSTANCE;
    hdma_adc_motor.Init.Channel = DMA_MOTOR_CHANNEL;
    hdma_adc_motor.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc_motor.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc_motor.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc_motor.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc_motor.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc_motor.Init.Mode = DMA_CIRCULAR;
    hdma_adc_motor.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc_motor.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc_motor) != HAL_OK)
    {
      Error_Handler_Motor();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc_motor);

    /* ADC_MOTOR interrupt Init */
    HAL_NVIC_SetPriority(ADC_MOTOR_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_MOTOR_IRQn);

	  /* DMA interrupt init */
	  /* DMA_MOTOR_INSTANCE_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA_MOTOR_IRQ, 1, 0);
	  HAL_NVIC_EnableIRQ(DMA_MOTOR_IRQ);
	}
}


/**
  * @brief  This function handles ADC1, ADC2 and ADC_MOTOR global interrupts.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&hadc_motor);
}


/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(uint8_t i = 0; i < 2; i++)
	{
		adc_buffer[i] = dma_buffer[i];
	}
}


/**
  * @brief  This function handles ADC_MOTOR DMA2 stream0 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc_motor);
}


/**
  * @brief  ERROR Handler disables power stage
  * @param  None
  * @retval None
  */
void Error_Handler_Motor(void)
{
	// Disable Drivers
	set_enable_disable_status(MOTOR_1, DISABLE_MOTOR);
	set_enable_disable_status(MOTOR_2, DISABLE_MOTOR);

	// set dutycycle to zero
	set_motor_dutycycle(MOTOR_1, 0);
	set_motor_dutycycle(MOTOR_2, 0);

	// add some notifications and a failure detection
	while(1);
}


/**
  * @brief  This ISR is called on falling PWM edge and copy the latest valid current value
  * 				current measurement is only accurate if PWM pin state is active high
  * @param  TIM_HandleTypeDef
  * @retval None
  */
void TIM_M1_ISR(TIM_HandleTypeDef* htim)
{
	// current measurement values are only valid while PWM pin state is active high
	adc_value[1] = adc_buffer[1];

//	if(get_enable_disable_status(MOTOR_2) == DISABLE_MOTOR)
//		set_enable_disable_status(MOTOR_2, ENABLE_MOTOR);
//	else
//		set_enable_disable_status(MOTOR_2, DISABLE_MOTOR);

  HAL_TIM_IRQHandler(&htim_m1);
}


/**
  * @brief  This ISR is called on falling PWM edge and copy the latest valid current value
  * 				current measurement is only accurate if PWM pin state is high
  * @param  TIM_HandleTypeDef
  * @retval None
  */
void TIM_M2_ISR(TIM_HandleTypeDef* htim)
{
	// current measurement values are only valid while PWM pin state is active high
	adc_value[0] = adc_buffer[0];

//	if(get_enable_disable_status(MOTOR_2) == DISABLE_MOTOR)
//		set_enable_disable_status(MOTOR_2, ENABLE_MOTOR);
//	else
//		set_enable_disable_status(MOTOR_2, DISABLE_MOTOR);

  HAL_TIM_IRQHandler(&htim_m2);
}
