/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lz_uart.h"
#include "lz_serial.h"
#include "lz_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LZ_VREFINT 19
#define LZ_VSIGNAL 0

//#define LZ_CHECK_DMA_SYNC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

COMP_HandleTypeDef hcomp1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
volatile uint16_t gBuffer[1024];
static_assert((sizeof(gBuffer) & (sizeof(gBuffer) - 1)) == 0, "Queue size must be a power of 2");

volatile uint32_t still_to_transmit = 0;

volatile uint8_t is_trigger_enabled = 0;
volatile uint8_t is_trigger_ready = 0;
volatile uint8_t is_busy = 0;
volatile uint8_t is_triggered = 0;
volatile uint8_t is_tx_transmitting_dma = 0;

volatile uint8_t events_count = 0;
volatile uint32_t missed_triggers = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_COMP1_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct lz_settings {
	uint32_t arr;
	uint16_t total_points;
	int16_t trigger_offset;
	uint16_t num_of_events;
	uint16_t threshold_l;
	uint16_t threshold_h;
	uint16_t num_of_acquisitions;
} gSettings;

enum {
	LZ_IDLE, LZ_SEND_SETTINGS, LZ_UPDATE_SETTINGS, LZ_RUN
} gState = LZ_IDLE;

volatile uint8_t gRxReadingData = 0;
uint8_t __lz_serial_new_character_read(char c) {
	if (!gRxReadingData) {
		switch (c) {
		case 'u':
			gState = LZ_UPDATE_SETTINGS;
			return 1;
		case 's':
			gState = LZ_SEND_SETTINGS;
			return 1;
		case 'r':
			gState = LZ_RUN;
			return 1;
		default:
			break;
		}
	}

	return 0;
}

void __lz_serial_error_tx_buffer_full(void) {
	GPIOE->BSRR = GPIO_BSRR_BS1;
}

void __lz_serial_error_rx_buffer_full(void) {
	Error_Handler();
}

void applySettings(const struct lz_settings *settings) {
	assert(!(ADC3->CR & ADC_CR_ADSTART)); // check ADC is not measuring

	// configure sampling frequency
	TIM2->ARR = settings->arr;
	TIM2->CNT = 0;

	// configure pretrigger / trigger delay
	TIM3->ARR = settings->total_points + settings->trigger_offset;

	// configure DMA lost conversion check
	TIM4->ARR = sizeof(gBuffer) / sizeof(uint16_t) - 1;

	// configure trigger
	ADC3->LTR2 = settings->threshold_l;
	ADC3->HTR2 = settings->threshold_h;
}

void startAdc(void) {
	ADC3->CR |= ADC_CR_ADSTART;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void startCircularBuffer(void) {
	// Setup ADC circular buffer
	DMA2_Stream0->PAR = (uint32_t) &ADC3->DR;
	DMA2_Stream0->M0AR = (uint32_t) &gBuffer;
	DMA2_Stream0->NDTR = sizeof(gBuffer) / sizeof(uint16_t);
	DMA2_Stream0->CR |= DMA_SxCR_CIRC;

	// Clear flags
	DMA2->LIFCR = DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CHTIF0
			| DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0;

	// Enable check and circular buffer
	TIM2->CR1 &= ~TIM_CR1_CEN;

#ifdef LZ_CHECK_DMA_SYNC
	TIM4->CR1 |= TIM_CR1_CEN;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	TIM4->CNT = sizeof(gBuffer) / sizeof(uint16_t) - DMA2_Stream0->NDTR;
#else
	DMA2_Stream0->CR |= DMA_SxCR_EN;
#endif

	TIM2->CR1 |= TIM_CR1_CEN;
}

void stopAll(void) {
	TIM2->CR1 &= ~TIM_CR1_CEN; // sample clock
	TIM3->CR1 &= ~TIM_CR1_CEN; // pretrigger clock
	TIM4->CR1 &= ~TIM_CR1_CEN; // DMA circular buffer check

	ADC3->CR |= ADC_CR_ADSTP;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;

	// Wait for ADC to stop, ADC DMA to stop and USART DMA transmission to stop (wait for end of tx)
	while ((ADC3->CR & ADC_CR_ADSTART) || (DMA2_Stream0->CR & DMA_SxCR_EN)
			|| is_tx_transmitting_dma) {
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_COMP1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
	COMP1->CFGR |= COMP_CFGRx_EN;

	DAC1->CR|=DAC_CR_EN1;
	DAC1->DHR12R1 = 1240; //IN 12 BIT CON FONDO SCALA 3.3
	DAC1->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

	lz_adc_prepare_channel(LZ_VSIGNAL, LZ_ST_1_5_CLOCK);

	lz_adc_calibration();
	lz_adc_enable();
	LZ_ADC_SEQUENCE(LZ_VSIGNAL);

	// Configure circular mode DMA
	ADC3->CFGR |= 3 << ADC_CFGR_DMNGT_Pos;

	// Overrun mode (debug purpose only)
	//ADC3->CFGR |= ADC_CFGR_OVRMOD;

	// Enable TIM3 interrupt (end of post-trigger)
	TIM3->CR1 |= TIM_CR1_OPM;

	// Enable USART
	lz_uart_setup();

	USART3->CR3 |= USART_CR3_DMAT;

	gSettings.arr = 120;
	gSettings.num_of_events = 10;
	gSettings.threshold_h = 21845;
	gSettings.threshold_l = 14563;
	gSettings.total_points = 256;
	gSettings.trigger_offset = 0;
	gSettings.num_of_acquisitions = 1000;

	applySettings(&gSettings);

	startCircularBuffer();
	startAdc();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch (gState) {
				case LZ_SEND_SETTINGS:
					stopAll();
					lz_uart_tx_enable_interrupt();

					lz_serial_send_binary(&gSettings.arr, sizeof(gSettings.arr));
					lz_serial_send_binary(&gSettings.total_points,
							sizeof(gSettings.total_points));
					lz_serial_send_binary(&gSettings.trigger_offset,
							sizeof(gSettings.trigger_offset));
					lz_serial_send_binary(&gSettings.num_of_events,
							sizeof(gSettings.num_of_events));
					lz_serial_send_binary(&gSettings.threshold_l,
							sizeof(gSettings.threshold_l));
					lz_serial_send_binary(&gSettings.threshold_h,
							sizeof(gSettings.threshold_h));
					lz_serial_send_binary(&gSettings.num_of_acquisitions, sizeof(gSettings.num_of_acquisitions));

					lz_uart_tx_disable_interrupt();

					startAdc();
					startCircularBuffer();

					gState = LZ_IDLE;
					break;

				case LZ_UPDATE_SETTINGS:
					stopAll();

					gRxReadingData = 1;
					lz_serial_read(&gSettings.arr, sizeof(gSettings.arr));
					lz_serial_read(&gSettings.total_points,
							sizeof(gSettings.total_points));
					lz_serial_read(&gSettings.trigger_offset,
							sizeof(gSettings.trigger_offset));
					lz_serial_read(&gSettings.num_of_events,
							sizeof(gSettings.num_of_events));
					lz_serial_read(&gSettings.threshold_l, sizeof(gSettings.threshold_l));
					lz_serial_read(&gSettings.threshold_h, sizeof(gSettings.threshold_h));
					lz_serial_read(&gSettings.num_of_acquisitions, sizeof(gSettings.num_of_acquisitions));

					gRxReadingData = 0;

					applySettings(&gSettings);
					startAdc();
					startCircularBuffer();

					gState = LZ_IDLE;
					break;

				case LZ_RUN:
					for (uint32_t i = 0; i < gSettings.num_of_acquisitions; i++) {
						TIM3->SR &= ~TIM_SR_UIF;

						while (COMP12->SR & COMP_SR_C1VAL);
						while (!(COMP12->SR & COMP_SR_C1VAL));

						TIM3->CNT = 1;
						TIM3->CR1 |= TIM_CR1_CEN;

						while (!(TIM3->SR & TIM_SR_UIF));

						// stop DMA ADC and TIM4
						TIM2->CR1 &= ~TIM_CR1_CEN;

#ifdef LZ_CHECK_DMA_SYNC
						TIM4->CR1 &= ~TIM_CR1_CEN;
#endif
						DMA2_Stream0->CR &= ~DMA_SxCR_EN;
						while (DMA2_Stream0->CR & DMA_SxCR_EN);

						// check if DMA and TIM4 are in sync
						uint32_t succ_idx = sizeof(gBuffer) / sizeof(uint16_t) - DMA2_Stream0->NDTR;
#ifdef LZ_CHECK_DMA_SYNC
						uint32_t timer_idx = TIM4->CNT;
						if (timer_idx != succ_idx)
						{
							;
						}
#endif

						uint16_t init_idx = (succ_idx - gSettings.total_points)
								& (sizeof(gBuffer) / sizeof(uint16_t) - 1);
						uint16_t till_end = sizeof(gBuffer) / sizeof(uint16_t) - init_idx;

						uint16_t num_to_read = MIN(gSettings.total_points, till_end);
						still_to_transmit = MAX(0,
								(int32_t )gSettings.total_points - (int32_t )num_to_read);

						// enable USART3 DMA
						DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;
						DMA1_Stream3->M0AR = (uint32_t) (gBuffer + init_idx);
						DMA1_Stream3->NDTR = num_to_read * sizeof(uint16_t);
						USART3->CR3 |= USART_CR3_DMAT;

						DMA1->LIFCR = DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 | DMA_LIFCR_CHTIF3
								| DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3;

						DMA1_Stream3->CR |= DMA_SxCR_EN;

						while (DMA1_Stream3->CR & DMA_SxCR_EN);

						if (still_to_transmit > 0) {
							// Transmit remaining measurements
							DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;
							DMA1_Stream3->M0AR = (uint32_t) &gBuffer;
							DMA1_Stream3->NDTR = still_to_transmit * sizeof(uint16_t);
							USART3->CR3 |= USART_CR3_DMAT;

							DMA1->LIFCR = DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 | DMA_LIFCR_CHTIF3
									| DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3;

							DMA1_Stream3->CR |= DMA_SxCR_EN;

							while (DMA1_Stream3->CR & DMA_SxCR_EN);
						}


						// Reharm only if ADC is running
						if (ADC3->CR & ADC_CR_ADSTART) {
							GPIOB->BSRR = GPIO_BSRR_BR0;
							startCircularBuffer();
							TIM2->CR1 |= TIM_CR1_CEN;
						}
					}
					gState = LZ_IDLE;
					break;

				case LZ_IDLE:
					break;
				}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
