#include "main.h"
#include "stm32f0xx_hal.h"

// PORT A (IN)
//      PWM_OUT       0x008 // GPI0A_3
#define LDR_NORMAL		0x010	// GPIOA_4
#define STROBE_PWR		0x020	// GPIOA_5
#define LDR_LOTS			0x080	// GPIOA_7
#define RESET_BUTTON	0x100	// GPIOA_8
#define DE_ACTIVATE		0x200	// GPIOA_9


// PORT B (OUT)
#define LASER_NORMAL	0x01	// GPIOB_0
#define LASER_LOTS		0x02	// GPIOB_1
#define RED_GRN_LIGHT	0x04	// GPIOB_2
#define READY					0x08	// GPIOB_3
#define LOW_POWER			0x10	// GPIOB_4

TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc;
uint8_t last_pwr_state = 0;
                               
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static void Error_Handler(void) { while(1); }

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

    /**Initializes the CPU, AHB and APB busses clocks*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		Error_Handler();
	
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);			/**Configure the Systick interrupt time */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);/**Configure the Systick */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);						/* SysTick_IRQn interrupt configuration */
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2000; //0x1F3F;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();

  HAL_TIM_MspPostInit(&htim2);
}

/* ADC init function */
static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
    Error_Handler();

  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    Error_Handler();
}

/** Pinout Configuration*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// Outputs
	GPIO_InitStruct.Pin = LASER_NORMAL | LASER_LOTS | RED_GRN_LIGHT;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = READY | LOW_POWER;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// Inputs
	GPIO_InitStruct.Pin = RESET_BUTTON | DE_ACTIVATE;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = STROBE_PWR;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = LDR_NORMAL | LDR_LOTS;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/*
	1. ADC value is from 0 - FF mapped to 0 - 800Hz
  2. Equation for Hz vs period graph to be linear with ADC is: period = ~3398 / Hz
    Working out:
		New_freq = Clk Freq [8Mhz] / (PRE [2000] + 1)
		New_period = 1 / New_freq [~3998Hz]
		PWM_freq = 1 / (New_period [~250us] x Period)
		Period = ~3998 / PWM_freq
 */
static uint32_t CalcPeriod(uint32_t adc_value)
{
	uint32_t freq = adc_value * 100 / 255;	// 1.
	if (freq == 0)
		freq = 1;
	return 3998 / freq;											// 2.
}

static uint8_t CheckStrobeState(void)
{
	if (last_pwr_state != (GPIOA->IDR & STROBE_PWR))
	{
		last_pwr_state = (GPIOA->IDR & STROBE_PWR);
		return 1;
	}
	else
		return 0;
}	

int main(void)
{
	uint8_t laser_loop = 1;
	uint8_t strobe_power = 0;
  HAL_Init();/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  SystemClock_Config();/* Configure the system clock */
  uint16_t period = 3998;
  uint16_t pulse = 1999;
	

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
	
  /*
  LDR_NORMAL		- when laser is on, port is active (3v)
  STROBE_PWR		- is active high (when on, then 3V is pumping in)
  LDR_LOTS			- when laser is on, port is active (3v)
  RESET_BUTTON	- Is active low (When idle, its at 3v, when pressed it goes to 0v)
  DE_ACTIVATE		- Is active low (When idle, its at 3v, when pressed it goes to 0v)

  LASER_NORMAL	- Active High
  LASER_LOTS		- Active High
  RED_GRN_LIGHT	- normally says lazer on, when pushed high the light says lazer green
  READY					- Active low (when high, the light is off, when low the light is on)
  LOW_POWER			- Active low (when high, the light is off, when low the light is on)
  */
  
	GPIOB->ODR |= (LASER_NORMAL | 					// turn laser_normal on
								READY);										// turn ready LED off

  while (1)
  {
		while(laser_loop)
		{
			if((GPIOA->IDR & LDR_NORMAL) == 0) 	// if normal_laser beam broken?
			{
				GPIOB->ODR |= LASER_LOTS;					// turn laser_lots on
			}
			
			if(!(GPIOA->IDR & RESET_BUTTON) &&	// has reset_button been pushed?
				(GPIOA->IDR & LDR_LOTS) &&				// AND laser_lots beam active?
				(GPIOA->IDR & LDR_NORMAL))				// AND laser_normal beam active?
			{
				GPIOB->ODR &= ~LASER_LOTS;				// turn laser_lots off
			}
			
			if(!(GPIOA->IDR & DE_ACTIVATE) &&		// has de_activate been pushed?
				((GPIOB->ODR & LASER_LOTS) == 0)&&// AND laser_lots beam off?
				(GPIOA->IDR & LDR_NORMAL))				// AND laser_normal beam active?
			{
				GPIOB->ODR &= ~(LASER_NORMAL | READY);    // turn laser_normal off; turn Ready light on
				GPIOB->ODR |= RED_GRN_LIGHT | LOW_POWER;  // change light to green; Turn Low pwr Light off
				laser_loop = 0;
			}
		}
		
		if(CheckStrobeState())
		{
			if (last_pwr_state)
			{
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
				HAL_ADC_Start(&hadc);
			}
			else
			{
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				HAL_ADC_Stop(&hadc);
			}
		}
		
		if (last_pwr_state)
		{
			period = CalcPeriod(HAL_ADC_GetValue(&hadc));
			pulse = period * 99 / 100;
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse);
			__HAL_TIM_SetAutoreload(&htim2, period);
		}
		//HAL_Delay(500);
  }
}
