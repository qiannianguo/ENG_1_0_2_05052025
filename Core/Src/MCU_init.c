/**
******************************************************************************
* @file			: MCU_init.c
* @short        : MCU peripheral initialization
******************************************************************************
*
* @details
* This file consist the MCU peripheral resets and initializations,
*
*
* @note
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "MCU_init.h"
#include <stm32l0xx_hal.h>
#include "stm32l0xx_hal_iwdg.h"  // 必须包含IWDG头文件
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


ADC_HandleTypeDef hadc;
//DMA_HandleTypeDef hdma_adc;    //DMA not used. Use polling ADC

DAC_HandleTypeDef hdac;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim22;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim21;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;  // 定义看门狗句柄
/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);   //DMA not used. Use polling ADC
static void MX_DAC_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM22_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t);

void delay_us (uint16_t us)
{
__HAL_TIM_SET_COUNTER(&htim3,0);  				// set the counter value a 0
while (__HAL_TIM_GET_COUNTER(&htim3) < us);  	// wait for the counter to reach the us input in the parameter
}


/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
void MCU_peripheral_init(void)
  {
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
#if !DEBUG_MODE
  IWDG_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  // 3. 喂狗（必须在超时前调用 HAL_IWDG_Refresh）
  HAL_IWDG_Refresh(&hiwdg);
#endif
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();			//use polling ADC
  MX_DAC_Init();
  MX_ADC_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM22_Init();
  MX_TIM3_Init();
  MX_TIM21_Init();

  /* USER CODE BEGIN 2 */

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);     	//Reset MCU power off output
  POWER_ON();
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);      	//amber led off
  LED_AMB_OFF();

  //HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buffer, 18);		//DMA not used. Use polling ADC
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim22);
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

/** Configure the main internal regulator output voltage
*/
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

/** Initializes the RCC Oscillators according to the specified parameters
* in the RCC_OscInitTypeDef structure.
*/
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
{
Error_Handler();
}

/** Initializes the CPU, AHB and APB buses clocks
*/
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
{
Error_Handler();
}
PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
{
Error_Handler();
}
}
/*1. 当前确定的硬件配置
根据 SystemClock_Config()：

HSE = 8MHz → 经过PLL（×6/3）→ SYSCLK = 16MHz

APB1 = APB2 = 16MHz（无分频）

TIM21（挂载APB2）和 TIM22（挂载APB1）的时钟均为16MHz

2. 定时器参数
定时器	Prescaler	Period	计算式
TIM21	160	500	(160+1)×(500+1)/16,000,000
TIM22	160	498	(160+1)×(498+1)/16,000,000
 *
 */
/**
* @brief ADC Initialization Function
* @param None
* @retval None
*/
static void MX_ADC_Init(void)
{

/* USER CODE BEGIN ADC_Init 0 */

/* USER CODE END ADC_Init 0 */

ADC_ChannelConfTypeDef sConfig = {0};
(void)sConfig;
/* USER CODE BEGIN ADC_Init 1 */

/* USER CODE END ADC_Init 1 */

/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
*/
hadc.Instance = ADC1;
hadc.Init.OversamplingMode = DISABLE;
hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
hadc.Init.Resolution = ADC_RESOLUTION_12B;
hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc.Init.ContinuousConvMode = ENABLE;
hadc.Init.DiscontinuousConvMode = DISABLE;
hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
hadc.Init.DMAContinuousRequests = DISABLE;
hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
hadc.Init.LowPowerAutoWait = DISABLE;
hadc.Init.LowPowerFrequencyMode = DISABLE;
hadc.Init.LowPowerAutoPowerOff = DISABLE;
if (HAL_ADC_Init(&hadc) != HAL_OK)
{
Error_Handler();
}

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_1;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_5;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_6;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_7;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/

/** Configure for the selected ADC regular channel to be converted.

sConfig.Channel = ADC_CHANNEL_14;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}*/
/* USER CODE BEGIN ADC_Init 2 */

/* USER CODE END ADC_Init 2 */

}

/**
* @brief DAC Initialization Function
* @param None
* @retval None
*/
static void MX_DAC_Init(void)
{

/* USER CODE BEGIN DAC_Init 0 */

/* USER CODE END DAC_Init 0 */

DAC_ChannelConfTypeDef sConfig = {0};

/* USER CODE BEGIN DAC_Init 1 */

/* USER CODE END DAC_Init 1 */

/** DAC Initialization
*/
hdac.Instance = DAC;
if (HAL_DAC_Init(&hdac) != HAL_OK)
{
  Error_Handler();
}

/** DAC channel OUT1 config
*/
sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN DAC_Init 2 */

/* USER CODE END DAC_Init 2 */

}

/**
* @brief I2C3 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C3_Init(void)
{

/* USER CODE BEGIN I2C3_Init 0 */

/* USER CODE END I2C3_Init 0 */

/* USER CODE BEGIN I2C3_Init 1 */

/* USER CODE END I2C3_Init 1 */
hi2c3.Instance = I2C3;
hi2c3.Init.Timing =  0x00303D5B;  //0x40003eff;
hi2c3.Init.OwnAddress1 = 0;
hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hi2c3.Init.OwnAddress2 = 0;
hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
if (HAL_I2C_Init(&hi2c3) != HAL_OK)
{
Error_Handler();
}

/** Configure Analogue filter
*/
if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
{
Error_Handler();
}

/** Configure Digital filter
*/
if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN I2C3_Init 2 */

/* USER CODE END I2C3_Init 2 */

}

/**
* @brief SPI1 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI1_Init(void)
{

/* USER CODE BEGIN SPI1_Init 0 */

/* USER CODE END SPI1_Init 0 */

/* USER CODE BEGIN SPI1_Init 1 */

/* USER CODE END SPI1_Init 1 */
/* SPI1 parameter configuration*/
hspi1.Instance = SPI1;
hspi1.Init.Mode = SPI_MODE_MASTER;
hspi1.Init.Direction = SPI_DIRECTION_2LINES;
hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
hspi1.Init.NSS = SPI_NSS_SOFT;
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
hspi1.Init.CRCPolynomial = 7;
if (HAL_SPI_Init(&hspi1) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN SPI1_Init 2 */

/* USER CODE END SPI1_Init 2 */

}

/**
* @brief TIM22 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM22_Init(void)
{

/* USER CODE BEGIN TIM22_Init 0 */

/* USER CODE END TIM22_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};

/* USER CODE BEGIN TIM22_Init 1 */

/* USER CODE END TIM22_Init 1 */
htim22.Instance = TIM22;
htim22.Init.Prescaler = 160;
htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
htim22.Init.Period = 498;            //498 5ms
htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM22_Init 2 */

/* USER CODE END TIM22_Init 2 */

}

/**
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM3_Init(void)
{

/* USER CODE BEGIN TIM3_Init 0 */

/* USER CODE END TIM3_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};

/* USER CODE BEGIN TIM3_Init 1 */

/* USER CODE END TIM3_Init 1 */
htim3.Instance = TIM3;
htim3.Init.Prescaler = 16-1;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 65535;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

/* USER CODE END TIM3_Init 2 */

}

/**
* @brief TIM21 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM21_Init(void)
{

/* USER CODE BEGIN TIM21_Init 0 */

/* USER CODE END TIM21_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};

/* USER CODE BEGIN TIM21_Init 1 */

/* USER CODE END TIM21_Init 1 */
htim21.Instance = TIM21;
htim21.Init.Prescaler = 160;
htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
htim21.Init.Period = 500;
htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM21_Init 2 */

/* USER CODE END TIM21_Init 2 */

}
/*2.1 时钟计算
预分频器(Prescaler)：160

实际分频系数 = Prescaler + 1 = 161

定时器时钟 = 主时钟 / 161 ≈ 198.76kHz (若主频32MHz)

2.2 定时周期计算
自动重装载值(Period)：500

实际计数值 = Period + 1 = 501

定时周期 = 计数次数 / 定时器时钟 ≈ 501/198.76kHz ≈ 2.52ms*/

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

/* USER CODE BEGIN USART2_Init 0 */

/* USER CODE END USART2_Init 0 */

/* USER CODE BEGIN USART2_Init 1 */

/* USER CODE END USART2_Init 1 */
huart2.Instance = USART2;
huart2.Init.BaudRate = 57600; //115200;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
if (HAL_UART_Init(&huart2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN USART2_Init 2 */

/* USER CODE END USART2_Init 2 */

}

/*
* Enable DMA controller clock

static void MX_DMA_Init(void)
{

// DMA controller clock enable
//__HAL_RCC_DMA1_CLK_ENABLE();

// DMA interrupt init
// DMA1_Channel1_IRQn interrupt configuration
//HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}*/

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOC, DO_Pin|PUMP_STEP_A_Pin|PUMP_STEP_B_Pin|PUMP_STEP_C_Pin
		  |PUMP_STEP_D_Pin|PD_Pin|USONIC_PWR_PIN, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB, DOB11_Pin|TPA_STEP_A_Pin|TPA_STEP_B_Pin|TPA_STEP_C_Pin
		  |TPA_STEP_D_Pin|DOB8_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, POWER_OFF_Pin|LED_AMB_Pin|AUDIO_Pin
		  |VOLUME_Pin|DOA15_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

/*Configure GPIO pins : NC_Pin NCC14_Pin NCC15_Pin
		   NCC3_Pin */
/*GPIO_InitStruct.Pin = NC_Pin|NCC14_Pin|NCC15_Pin|NCC3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/

/*Configure GPIO pins : DO_Pin PUMP_STEP_A_Pin PUMP_STEP_B_Pin PUMP_STEP_C_Pin
		   PUMP_STEP_D_Pin PD_Pin */
GPIO_InitStruct.Pin = DO_Pin|PUMP_STEP_A_Pin|PUMP_STEP_B_Pin|PUMP_STEP_C_Pin
		  |PUMP_STEP_D_Pin|PD_Pin|USONIC_PWR_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : DI_Pin DIB1_Pin DIB2_Pin NCB9_Pin */
GPIO_InitStruct.Pin = DI_Pin|DIB1_Pin|DIB2_Pin|NCB9_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*Configure GPIO pin : DIB10_Pin */
GPIO_InitStruct.Pin = DIB10_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(DIB10_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : DOB11_Pin TPA_STEP_A_Pin TPA_STEP_B_Pin TPA_STEP_C_Pin
		   TPA_STEP_D_Pin DOB8_Pin */
GPIO_InitStruct.Pin = DOB11_Pin|TPA_STEP_A_Pin|TPA_STEP_B_Pin|TPA_STEP_C_Pin
		  |TPA_STEP_D_Pin|DOB8_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*Configure GPIO pins : POWER_OFF_Pin LED_GRN_Pin LED_AMB_Pin AUDIO_Pin
		   VOLUME_Pin DOA15_Pin */
GPIO_InitStruct.Pin = POWER_OFF_Pin|LED_AMB_Pin|AUDIO_Pin
		  |VOLUME_Pin|DOA15_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pin : DIC10_Pin */
GPIO_InitStruct.Pin = DIC10_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(DIC10_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pin : INT_Pin */
GPIO_InitStruct.Pin = INT_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pin : CS_Pin */
GPIO_InitStruct.Pin = CS_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : USART1_TX_Pin USART1_RX_Pin */
GPIO_InitStruct.Pin = USART1_TX_Pin|USART1_RX_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void IWDG_Init(void) {
	/* USER CODE BEGIN IWDG_Init 0 */

	  /* USER CODE END IWDG_Init 0 */

	  /* USER CODE BEGIN IWDG_Init 1 */

	  /* USER CODE END IWDG_Init 1 */
	  hiwdg.Instance = IWDG;
	  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	  hiwdg.Init.Window = 4095;
	  hiwdg.Init.Reload = 4095;
	  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN IWDG_Init 2 */

	  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
__disable_irq();
while (1)
{
}
/* USER CODE END Error_Handler_Debug */
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
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
