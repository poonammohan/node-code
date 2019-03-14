/**
  ******************************************************************************
  * File Name          : system.c
  * Description        : System configuration, monitoring functions
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "dc_dc_ld.h"
#include "battery.h"
#include "panel.h"
#include "led.h"
#include "connectivity.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc1Pr;
DMA_HandleTypeDef hdma_adc1;
HRTIM_HandleTypeDef hhrtim1;
static TIM_HandleTypeDef ThreeSlSccTim;
TIM_HandleTypeDef ThreeSlLedEnTim;
TIM_HandleTypeDef ThreeSlLedAnaDimTim;
TIM_HandleTypeDef ThreeSlDcDcLdLoopTim;
UART_HandleTypeDef Sp1mlUart;
WWDG_HandleTypeDef WwdgHandle;

uint16_t ThreeSlSccPwmDuty = THREESL_SCC_PWM_DUTY_MAX;
uint16_t ThreeSlSccTimAcutalDuty;

uint32_t AdcRegV[NO_OF_REG_CHANNELS];

uint8_t* TS_CAL1_H = (uint8_t*)0x1FFFF7B9;
uint8_t* TS_CAL1_L = (uint8_t*)0x1FFFF7B8;
uint8_t* TS_CAL2_H = (uint8_t*)0x1FFFF7C3;
uint8_t* TS_CAL2_L = (uint8_t*)0x1FFFF7C2;
uint8_t TempCal1L,TempCal1H,TempCal2L,TempCal2H;
float TempSlope,TempOffset;
int32_t Temperature;


uint32_t DcDcBoostPeriod;
int32_t DcDcLdDutyChange = 0;
int32_t DcDcLdDutyPeriod;
uint32_t DcDcLdMaxDutyPeriod = (uint32_t)(DC_DC_LD_DUTY_MAX*DC_DC_BOOST_PEIROD);
uint32_t DcDcLdMinDutyPeriod = (uint32_t)(DC_DC_LD_DUTY_MIN*DC_DC_BOOST_PEIROD);
uint32_t AcDcCcrPeriod=0;
uint32_t LedI[NO_OF_INJ_SAMPLES];
int32_t LedCurrentErrorP = 0;
int32_t LedCurrentErrorI = 0;
int32_t LedCurrentGainP = 0;
int32_t LedCurrentGainI = 0;
int32_t Kp = 3;
int32_t Ki = 1;
int32_t LedCurrentGain = 0;
uint32_t LedI_CurrTh = 0;
uint32_t LedCurrent,LedCurrentMax;
bool LightOn = false;
bool LightDim = false;

float BattCurrK;
uint8_t BattDischgCurrent;

bool IgnoreLedCurrent = false;

uint8_t UartRxData[UART_RX_DATABUFF_SIZE];
uint8_t UartTxData[UART_TX_DATABUFF_SIZE];
uint16_t UartRxDataStartPntr = 0;
uint16_t UartRxDataEndNxtPntr = 0;
uint16_t UartRxDataLength = 0;
uint32_t Uart_Wait_Tick = 0;
uint32_t Uart_ResetCount = 0;
bool ReadDataPkt = false;

bool Flag_AcDcLdEnable = true;

uint16_t Tick_10 = 1;
uint32_t Tick_1000s = 1;
uint32_t Tick_1h = 1;
bool Batt_Tick_10 = false;
bool Batt_Tick_1000s = false;

/* Private function prototypes -----------------------------------------------*/
static void system_GPIO_Init(void);
static void system_ADC1_Init(void);
static void system_HRTIM1_Init(void);
static void system_DMA_Init(void);
static void system_TIM3_Init(void);
static void system_USART3_UART_Init(void);
static void system_TIM2_Init(void);
static void system_TIM17_Init(void);
static void system_TIM15_Init(void);
static void system_TempSensorInit(void);
static void system_WWDG_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);
                
/* Exported functions --------------------------------------------------------*/

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
* @brief This function initialize system peripherals and set initialization
         parameters
*/
void system_Init(void)
{
  system_GPIO_Init();
  system_DMA_Init();
  system_ADC1_Init();
  system_HRTIM1_Init();
  system_TIM2_Init();
  system_TIM17_Init();
  system_TIM3_Init();
  system_TIM15_Init();
  system_USART3_UART_Init();
  
  /* Temperature sensor init */
  system_TempSensorInit();
  
  /* Window watchdog init */
  system_WWDG_Init();  
  
  /* Initialize LED driver */
  system_DcDcLdSet();
  /* this is also required for measuring led current */
  system_DcDcLdEnable(); 
  
  /* Reset SP1ML */
  system_ResetModuleConn();
  
  /* Monitor battery and panel voltage */
  system_Monitor();   
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
    PB6 and PB7 are enable pins of DC DC boost LED driver gate driver
    PA10 - PIR sensor input
    PB14 SP1ML reset pin
*/
void system_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);//Added by Sunil GPIO_PIN_5
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);//AC DC Enable
  
  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;//Added by Sunil GPIO_PIN_5
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);       
  
  /* SP1ML reset pin configuration, PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
  
  /* HVLED on-off (AC-DC-ON-OFF) pin configuration, PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);      
}


/* ADC1 init function
   Channel 8 is used for sensing LED current in injected configuration
   Other channels are regular group channels in continuous conversion mode */
void system_ADC1_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = NO_OF_REG_CHANNELS;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  
    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);  
  
  /* Start ADC1 calibration */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  
  /* Start injected channel */
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  /* Start regular ADC channels with DMA */
  HAL_ADC_Start_DMA(&hadc1, AdcRegV, NO_OF_REG_CHANNELS);
}

/**
* @brief This function initialize DMA1 to be used with ADC1
*/
void system_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 1);
  /* Uncomment to enable DMA Channel 1 Interrupt Request */
//  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}


/**
* @brief This function initialize HRTIM
         HRTIM is used for driving DC DC boost LED driver
         Two output channels are used for driving two channels with 180 phase
         shift to drive 2-phase converter
         HRTIM_MASTER, HRTIM_A and HRTIM_B are configured
*/
void system_HRTIM1_Init(void)
{

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg;
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
  HRTIM_TimerCfgTypeDef pTimerCfg;
  HRTIM_CompareCfgTypeDef pCompareCfg;
  HRTIM_OutputCfgTypeDef pOutputCfg;

  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  HAL_HRTIM_Init(&hhrtim1);

  HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14);

  HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10);

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_MASTER_CMP2;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg);

  pTimeBaseCfg.Period = DC_DC_BOOST_PEIROD;
  pTimeBaseCfg.RepetitionCounter = 0;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_ENABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg);

  pCompareCfg.CompareValue = 0xF000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  pCompareCfg.CompareValue = 0xF000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg);

  pTimeBaseCfg.Period = 20000;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg);

  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg);

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg);

  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMEV_1;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg);

  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg);

  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/* TIM2 init function
  Timer2 generates PWM for solar charge controller */
void system_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ThreeSlSccTim.Instance = THREE_SL_SCC_TIM;
  ThreeSlSccTim.Init.Prescaler = 0;
  ThreeSlSccTim.Init.CounterMode = TIM_COUNTERMODE_UP;
  ThreeSlSccTim.Init.Period = THREESL_SCC_PWM_DUTY_MAX;
  ThreeSlSccTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&ThreeSlSccTim);

  ThreeSlSccTim.Instance->CR1|=(TIM_CR1_ARPE);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ThreeSlSccTim, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = THREESL_SCC_PWM_DUTY_MAX+1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&ThreeSlSccTim, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&ThreeSlSccTim);

}

/* Timer3 for output DC-DC led driver and AC-DC led driver output on-off */
void system_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ThreeSlLedEnTim.Instance = THREE_SL_LED_EN_TIM;
  ThreeSlLedEnTim.Init.Prescaler = 0;
  ThreeSlLedEnTim.Init.CounterMode = TIM_COUNTERMODE_UP;
  ThreeSlLedEnTim.Init.Period = THREE_SL_LED_EN_TIM_PERIOD;
  ThreeSlLedEnTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&ThreeSlLedEnTim);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&ThreeSlLedEnTim, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&ThreeSlLedEnTim);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ThreeSlLedEnTim, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = THREE_SL_LED_EN_TIM_OUT_OFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&ThreeSlLedEnTim, &sConfigOC, 
                            THREE_SL_LED_EN_TIM_AC_DC_OUT_CH);

  HAL_TIM_PWM_ConfigChannel(&ThreeSlLedEnTim, &sConfigOC,
                            THREE_SL_LED_EN_TIM_DC_DC_OUT_CH);

  HAL_TIM_MspPostInit(&ThreeSlLedEnTim); 
  
  HAL_TIM_PWM_Start(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_AC_DC_OUT_CH);
  HAL_TIM_PWM_Start(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_DC_DC_OUT_CH);  
}


/* TIM17 init function 
  Timer17 for timebase which regulated DC-DC boost LED driver output current */
void system_TIM17_Init(void)
{
  ThreeSlDcDcLdLoopTim.Instance = THREE_SL_DC_DC_LD_LOOP_TIM;
  ThreeSlDcDcLdLoopTim.Init.Prescaler = 0;
  ThreeSlDcDcLdLoopTim.Init.CounterMode = TIM_COUNTERMODE_UP;
  ThreeSlDcDcLdLoopTim.Init.Period = 12799;
  ThreeSlDcDcLdLoopTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  ThreeSlDcDcLdLoopTim.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&ThreeSlDcDcLdLoopTim);
}

/* TIM15 init function 
  Timer15 for analog dimming of AC-DC LED driver section */
void system_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ThreeSlLedAnaDimTim.Instance = THREE_SL_LED_ANA_DIM_TIM;
  ThreeSlLedAnaDimTim.Init.Prescaler = THREE_SL_LED_ANA_TIM_PRESC;
  ThreeSlLedAnaDimTim.Init.CounterMode = TIM_COUNTERMODE_UP;
  ThreeSlLedAnaDimTim.Init.Period = THREE_SL_LED_ANA_TIM_PERIOD;
  ThreeSlLedAnaDimTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&ThreeSlLedAnaDimTim);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&ThreeSlLedAnaDimTim, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&ThreeSlLedAnaDimTim);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ThreeSlLedAnaDimTim, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;//THREE_SL_LED_ANA_TIM_PERIOD/2 + 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;  
  HAL_TIM_PWM_ConfigChannel(&ThreeSlLedAnaDimTim, &sConfigOC, 
                            THREE_SL_LED_ANA_TIM_OUT_CH);

  HAL_TIM_MspPostInit(&ThreeSlLedAnaDimTim); 
  
  HAL_TIMEx_PWMN_Start(&ThreeSlLedAnaDimTim, THREE_SL_LED_ANA_TIM_OUT_CH);
}

/**
* @brief This function initialize temperature sensor in STM32
*/
void system_TempSensorInit(void)
{
  TempCal1L = *TS_CAL1_L;
  TempCal1H = *TS_CAL1_H;
  TempCal2L = *TS_CAL2_L;
  TempCal2H = *TS_CAL2_H;

  TempSlope = 80/((float)(256*TempCal2H+TempCal2L)-(float)(256*TempCal1H+TempCal1L));
  TempOffset = 30 - (TempSlope*(float)(256*TempCal1H+TempCal1L));  
}

/* USART3 init function 
   USART3 is used for communication with SP1ML module */
void system_USART3_UART_Init(void)
{
  Sp1mlUart.Instance = THREE_SL_CONN_SP1ML_UART;
  Sp1mlUart.Init.BaudRate = 9600;
  Sp1mlUart.Init.WordLength = UART_WORDLENGTH_8B;
  Sp1mlUart.Init.StopBits = UART_STOPBITS_1;
  Sp1mlUart.Init.Parity = UART_PARITY_NONE;
  Sp1mlUart.Init.Mode = UART_MODE_TX_RX;
  Sp1mlUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Sp1mlUart.Init.OverSampling = UART_OVERSAMPLING_16;
  Sp1mlUart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  Sp1mlUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&Sp1mlUart);

  __HAL_UART_ENABLE_IT(&Sp1mlUart,UART_IT_RXNE);
}

/**
* @brief This function initialize window watchdog with EWI
*/
void system_WWDG_Init(void)
{
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_1;
  WwdgHandle.Init.Window    = 0x7F;
  WwdgHandle.Init.Counter   = 0x7F;
  HAL_WWDG_Init(&WwdgHandle);

  HAL_WWDG_Start(&WwdgHandle);
  
  __HAL_WWDG_ENABLE_IT(&WwdgHandle, WWDG_IT_EWI);
}

/**
* @brief This function monitor panel and battery status
*/
void system_Monitor(void)
{
  /* if battery voltage is between battery minimum voltage (-~1.5V -> 10V) and 
      battery maximum voltage (+~0.5V -> 15V), battery is ok */
  if ((battery_GetVoltage() < (battery_GetMaxVoltage()+200)) && 
      (battery_GetVoltage() > (battery_GetMinVoltage() - 150)))
  {
    battery_SetFlag(BATTERY_MASK_OK);
  }
  else
  {
    battery_ResetFlag(BATTERY_MASK_OK);
    battery_SetFlag(BATTERY_MASK_DEEP_DSCHRG);
  }
  
  if (panel_GetVoltage() > (panel_GetMinVoltage() - 150))
  {
    panel_SetFlag(PANEL_MASK_OK);
  }
  else if (panel_GetVoltage() < (panel_GetMinVoltage() - 250))
  {
    panel_ResetFlag(PANEL_MASK_OK);
  }
}

/**
* @brief This function acquire, average and update regular ADC channels value
         and refresh window watchdog
*/
void system_UpdateSysParams(void)
{
  static uint32_t battVolt = 0, countBattVolt = 0;
  static uint32_t battChgCurr = 0, countBattChgCurr = 0;
  static uint32_t panelVolt = 0, countPanelVolt = 0;
  static uint32_t ldOutVolt = 0, countLdOutVolt = 0;
  static uint32_t tempSensor = 0, countTempSensor = 0;
  
  tempSensor += *(AdcRegV + ADC_TEMPERATURE_SENSOR_VAL_INDEX);
  countTempSensor++;
  if (countTempSensor == AVG_TEMP_SENSOR)
  {
    tempSensor = tempSensor/countTempSensor;
    /* Empirically subtracting 10 from temperature for compensating
       dissipation in MCU */
    Temperature = (int32_t)(TempSlope*((float)(tempSensor)) + TempOffset - 15);
    countTempSensor = 0;
    tempSensor = 0;
  }  
  
  battVolt += *(AdcRegV + ADC_BATT_VOLTAGE_INDEX);
  countBattVolt++;
  if (countBattVolt == AVG_BATT_VOLT_SAMPLES)
  {
    battVolt = battVolt/countBattVolt;
    battery_UpdateVoltage(&battVolt,&Temperature);
    countBattVolt = 0;
    battVolt = 0;
  }
  
  battChgCurr += *(AdcRegV + ADC_BATT_CHARGING_CURRENT_INDEX);
  countBattChgCurr++;
  if (countBattChgCurr == AVG_BATT_CHG_CURR_SAMPLES)
  {
    battChgCurr = battChgCurr/countBattChgCurr;
    battery_UpdateChgCurrent(&battChgCurr);
    countBattChgCurr = 0;
    battChgCurr = 0;
  }
  
  panelVolt += *(AdcRegV + ADC_PANEL_VOLTAGE_INDEX);
  countPanelVolt++;
  if (countPanelVolt == AVG_PANEL_VOLT_SAMPLES)
  {
    panelVolt = panelVolt/countPanelVolt;
    panel_UpdateVoltage(&panelVolt);
    countPanelVolt = 0;
    panelVolt = 0;
  }
  
  ldOutVolt += *(AdcRegV + ADC_DC_DC_LD_OUTPUT_VOLTAGE_INDEX);
  countLdOutVolt++;
  if (countLdOutVolt == AVG_LD_OUT_VOLT_SAMPLES)
  {
    ldOutVolt = ldOutVolt/countLdOutVolt;
    dc_dc_ld_UpdateOutVoltage(&ldOutVolt);
    countLdOutVolt = 0;
    ldOutVolt = 0;
  }
  
    
  HAL_WWDG_Refresh(&WwdgHandle,0x7F);
}


/* This function configure PD2 panel on/off pin
   Configure panel reverse ADC channel 9 of ADC1 in regular conversion mode
   Set panel reverse flag accordingly and deinitialize ADC
   */
void system_CheckPanelReverse(void)
{
  static uint8_t noOfReversePanelChSamples = 10;
  static uint32_t reversePanelChVal = 0;
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  ADC_ChannelConfTypeDef sConfig; 
  
    __GPIOD_CLK_ENABLE();  
    
  /* Configure GPIO pin : PD2, Panel off pin configuration */
  system_PanelDisable();
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
  
  /* Delay for stabilizing startup */
  HAL_Delay(50);
  
    /**Common config 
    */
  hadc1Pr.Instance = ADC1;  
  hadc1Pr.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
  hadc1Pr.Init.Resolution = ADC_RESOLUTION12b;
  hadc1Pr.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1Pr.Init.ContinuousConvMode = ENABLE;
  hadc1Pr.Init.DiscontinuousConvMode = DISABLE;
  hadc1Pr.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1Pr.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1Pr.Init.NbrOfConversion = 1;
  hadc1Pr.Init.DMAContinuousRequests = DISABLE;
  hadc1Pr.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1Pr.Init.LowPowerAutoWait = DISABLE;
  hadc1Pr.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1Pr);

  
  /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1Pr, &sConfig);  
  HAL_ADCEx_Calibration_Start(&hadc1Pr, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1Pr);  
  
  HAL_Delay(1);
  
  for (uint8_t count=0; count<noOfReversePanelChSamples; count++)
  {
    while (!(__HAL_ADC_GET_FLAG(&hadc1Pr, ADC_FLAG_EOC)));
    reversePanelChVal += hadc1Pr.Instance->DR;
  }
  
  reversePanelChVal = reversePanelChVal/noOfReversePanelChSamples;
  
  if (reversePanelChVal > REVERSE_PANEL_CH_TH)
  {
    panel_SetFlag(PANEL_MASK_REVERSE);
  }
  else
  {
    panel_ResetFlag(PANEL_MASK_REVERSE);
  }
  
  reversePanelChVal = 0;
  HAL_ADC_Stop(&hadc1Pr);
  HAL_ADC_DeInit(&hadc1Pr); 
  HAL_DMA_DeInit(&hdma_adc1);
}

/**
* @brief This function calculate constant multiplier to be used to further
         calculate battery discharge current from LED output current
*/
void system_CalcBattCurrK(void)
{
  BattCurrK = (MCU_VDD*(DC_DC_SENSE_R1+DC_DC_SENSE_R2)*BATT_SENSE_R2)/
    (DC_DC_LED_DRIVER_EFF*ADC_RESOLUTION*LED_CURRENT_SENSE_R*DC_DC_SENSE_R2*
     (BATT_SENSE_R1+BATT_SENSE_R2));
}


/* This function return BattCurrent*10
    Discharge current value is a function of converter efficiency, resistor
    divider ratios, adc resolution, MCU voltage and actual output current, 
    output voltage and input voltage */
uint8_t system_CalcBattDischgCurrent(void)
{
  if (LedCurrent <= LED_CURRENT_ZERO_OFFSET)
  {
    BattDischgCurrent = 0;
  }
  else
  {
    BattDischgCurrent = (uint8_t)(((float)((dc_dc_ld_GetOutVoltage())*
                                  (LedCurrent - LED_CURRENT_ZERO_OFFSET)/
                                    (battery_GetVoltage())))*BattCurrK*10);
  }
    
  return BattDischgCurrent;
}

/**
* @brief This function turn-off panel
*/
void system_PanelDisable(void)
{
  /* Turn off panel */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);  
}

/**
* @brief This function turn-on panel
*/
void system_PanelEnable(void)
{
  /* Enable panel if it is not connected in reverse */
  if (!(panel_GetFlag(PANEL_MASK_REVERSE)))
  {
    /* Turn on panel */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  }  
}

/**
* @brief This function turn-off battery charging
*/
void system_DisableCharging(void)
{
  HAL_TIM_PWM_Stop(&ThreeSlSccTim, THREE_SL_SCC_TIM_CHANNEL);
}

/**
* @brief This function turn-on battery charging
*/
void system_EnableCharging(void)        
{
  HAL_TIM_PWM_Start(&ThreeSlSccTim, THREE_SL_SCC_TIM_CHANNEL);  
}

/**
* @brief This function return charge controller PWM value
*/
uint16_t system_GetSccPwm(void)
{
  return (THREESL_SCC_PWM_DUTY_MAX - 
          __HAL_TIM_GET_COMPARE(&ThreeSlSccTim,THREE_SL_SCC_TIM_CHANNEL));
}

/**
* @brief This function set charge controller PWM value
*/
void system_SetSccPwm(uint16_t duty)
{
  ThreeSlSccTimAcutalDuty = THREESL_SCC_PWM_DUTY_MAX - duty;
  
  if (ThreeSlSccTimAcutalDuty > THREESL_SCC_PWM_DUTY_MAX)
  {
    ThreeSlSccTimAcutalDuty = THREESL_SCC_PWM_DUTY_MAX;
  }
  else if (ThreeSlSccTimAcutalDuty < THREESL_SCC_PWM_DUTY_MIN)
  {
    ThreeSlSccTimAcutalDuty = THREESL_SCC_PWM_DUTY_MIN;
  }
  
  __HAL_TIM_SET_COMPARE(&ThreeSlSccTim, THREE_SL_SCC_TIM_CHANNEL, ThreeSlSccTimAcutalDuty);
}

/**
* @brief This function return temperature
*/
int16_t system_GetTemperature(void)
{
  return Temperature;
}

/**
* @brief This function set flag to disable LED current monitoring
*/
bool system_GetIgnoreLedCurrent(void)
{
  return IgnoreLedCurrent;
}

/**
* @brief This function set flag to enable LED current monitoring
*/
void system_SetIgnoreLedCurrent(bool set)
{
  IgnoreLedCurrent = set;
}

/**
* @brief This function initialize various parameters and registers
         for DC DC boost LED driver
*/
void system_DcDcLdSet(void)
{
  DcDcBoostPeriod = DC_DC_BOOST_PEIROD;
  
  /* Set duty for ADC trigger setting, Timer A and Timer B*/
  DcDcLdDutyPeriod = (int32_t)(DcDcBoostPeriod*DC_DC_LD_DUTY_MAX);
  
  /* Set DC DC LED driver period (frequency) */
  __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, DcDcBoostPeriod);
  /* Set compare unit 2 of master period for trigerring ADC */
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2,
                         DcDcLdDutyPeriod/2);
  /* Set period of Timer A according to master timer period */
  __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, DcDcBoostPeriod+1);
  /* Set period of Timer B according to master timer period */
  __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, DcDcBoostPeriod+1);
  
  DcDcLdDutyPeriod = (int32_t)(DcDcBoostPeriod*DC_DC_LD_DUTY_MIN);
  
  /* Set compare (duty) of Timer A */
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1
                         , DcDcLdDutyPeriod);
  /* Set compare (duty) of Timer B */
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1
                         , DcDcLdDutyPeriod);  
}

/**
* @brief This function disable DC DC boost LED driver gate driver
*/
void system_DcDcLdDisable(void)
{
  /* Disable gate driver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
* @brief This function enable DC DC boost LED driver gate driver
*/
void system_DcDcLdEnable(void)
{
  /* Enable HRTIM */
  
  /* Enable gate driver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  
  /* Enable Counter */
  HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER|
                                 HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_B);  
  
  /* setting duty cycle to minimum (soft start) */
  system_DcDcLdSetDuty(DcDcLdMinDutyPeriod);  

  hhrtim1.Instance->sCommonRegs.CR2 |= HRTIM_CR2_MSWU;
  hhrtim1.Instance->sCommonRegs.CR2 |= HRTIM_CR2_TASWU;
  hhrtim1.Instance->sCommonRegs.CR2 |= HRTIM_CR2_TBSWU;  
}

/**
* @brief This function disables output of DC DC boost LED driver
*/
void system_DcDcLdOutDisable(void)
{
  DcDcLdDutyPeriod = DcDcLdMinDutyPeriod;
  
  /* Disable gate driver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
  
  /* Disable current regulation loop */
  HAL_TIM_Base_Stop_IT(&ThreeSlDcDcLdLoopTim);
  
  /* Disable Output */
  __HAL_TIM_SET_COMPARE(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_DC_DC_OUT_CH,
                        THREE_SL_LED_EN_TIM_OUT_OFF);  
}

/**
* @brief This function enables output of DC DC boost LED driver
*/
void system_DcDcLdOutEnable(void)
{  
  /* Enable Output */
  __HAL_TIM_SET_COMPARE(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_DC_DC_OUT_CH,
                        THREE_SL_LED_EN_TIM_OUT_ON);
  
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 
                         HRTIM_COMPAREUNIT_1, DcDcLdMinDutyPeriod);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 
                         HRTIM_COMPAREUNIT_1, DcDcLdMinDutyPeriod);
}

/**
* @brief This function start current regulation loop for DC DC boost LED driver
*/
void system_DcDcStartRegulate(void)
{  
  /* Enable gate driver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  
  /* Enable PWM Outputs */
  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);      
      
  /* Enable current regulation loop */
  HAL_TIM_Base_Start_IT(&ThreeSlDcDcLdLoopTim);
}

/**
* @brief This function return duty cycle register value of DC DC boost LED driver
*/
uint32_t system_DcDcLdGetDuty(void)
{
  return (__HAL_HRTIM_GETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 
                         HRTIM_COMPAREUNIT_1));
}

/**
* @brief This function set duty cycle register value of DC DC boost LED driver
*/
void system_DcDcLdSetDuty(uint32_t duty)
{
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1
                         , duty);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1
                         , duty);   
}


/**
* @brief This function disable AC DC LED driver
*/
void system_AcDcLdDisable(void)
{
  /* Disable AC DC */
 GPIOC->BSRR = (uint32_t)GPIO_PIN_10;//------>originally was there commented by me
//  GPIOC->BRR = (uint32_t)GPIO_PIN_10;// Added by Sunil
}

/**
* @brief This function enable AC DC LED driver
*/
void system_AcDcLdEnable(void)
{
  /* Enable AC DC */
  GPIOC->BRR = (uint32_t)GPIO_PIN_10;//------>originally was there commented by me
//  GPIOC->BSRR = (uint32_t)GPIO_PIN_10;// Added by Sunil
}

/**
* @brief This function disable output of AC DC LED driver
*/
void system_AcDcLdOutDisable(void)
{
  /* Disable Output */
  __HAL_TIM_SET_COMPARE(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_AC_DC_OUT_CH,
                        THREE_SL_LED_EN_TIM_OUT_OFF); 
 // AcDcCcrPeriod = __HAL_TIM_GET_COMPARE(&ThreeSlLedEnTim,THREE_SL_LED_EN_TIM_AC_DC_OUT_CH);
}

/**
* @brief This function enable output of AC DC LED driver
*/
void system_AcDcLdOutEnable(void)
{
  /* Enable AC-DC Led driver */
  system_AcDcLdEnable();
  
  /* Enable Output */
  __HAL_TIM_SET_COMPARE(&ThreeSlLedEnTim, THREE_SL_LED_EN_TIM_AC_DC_OUT_CH,
                        THREE_SL_LED_EN_TIM_OUT_ON);  
 // AcDcCcrPeriod = __HAL_TIM_GET_COMPARE(&ThreeSlLedEnTim,THREE_SL_LED_EN_TIM_AC_DC_OUT_CH);
  
  system_AcDcLdEnable();  
}
/*********************Added by Sunil***********************************/  
bool system_GetTim3ACCCr_val(void)
{
  AcDcCcrPeriod = __HAL_TIM_GET_COMPARE(&ThreeSlLedEnTim,THREE_SL_LED_EN_TIM_AC_DC_OUT_CH);
  if((AcDcCcrPeriod > 0) )//&& (dc_dc_ld_GetOutVoltage()> DC_DC_LD_OUT_VOLT_25) )
  {
   if(dc_dc_ld_GetOutVoltage()> DC_DC_LD_OUT_VOLT_25)
   {  
    return true;
   } 
   else
   {  
     return false;
   } 
  }
  else
 {
   return true;
 }
}
/*********************Added by Sunil***********************************/  

void system_AcDcSetDim(uint8_t dim)
{
  static uint32_t timCrrTemp;
  
  timCrrTemp = (__HAL_TIM_GET_AUTORELOAD(&ThreeSlLedAnaDimTim)*dim)/100;
  __HAL_TIM_SET_COMPARE(&ThreeSlLedAnaDimTim, THREE_SL_LED_ANA_TIM_OUT_CH,
                        timCrrTemp);
}

/**
* @brief This function return status of Light On/off command sent from server
*/
bool system_GetServerLightCommand(void)
{
  return LightOn;
}

/**
* @brief This function set status of light On/off command sent from server
*/
void system_SetServerLightCommand(bool set)
{
  LightOn = set;
}

/**
* @brief This function return status of light dim command sent from server
*/
bool system_GetServerLightDimCommand(void)
{
  return LightDim;
}

/**
* @brief This function set status of light dim command sent from server
*/
void system_SetServerLightDimCommand(bool set)
{
  LightDim = set;
}

/**
* @brief This function return tick (in ms) corresponding to time passed since
         last valid packet received over UART from SP1ML
*/
uint32_t system_GetUartWaitTick(void)
{
  return Uart_Wait_Tick;
}

/**
* @brief This function resets UART wait tick
*/
void system_ResetUartTick(void)
{
  Uart_Wait_Tick = 0;
}

/**
* @brief This function checks the uart trasnmitter emmpty flag
          and transmits one byte over uart.
*/
bool system_TransmitUartData(uint8_t data)
{
  if (__HAL_UART_GET_FLAG(&Sp1mlUart, UART_FLAG_TXE))
  {
    Sp1mlUart.Instance->TDR = data;
    return true;
  }
  else
  {
    return false;
  }
}

/**
* @brief This function deinitialize and reinitialize UART (SP1ML) interface
         If UART reset occurs for 20 times, SP1ML module is reset
         If SP1ML modeule is reset 3 (UART reset occurs for 60 times) times
         system resets.
*/
void system_ResetConn(void)
{
  HAL_UART_DeInit(&Sp1mlUart);
  conn_ResetConn();
  
  system_USART3_UART_Init();
  Uart_Wait_Tick = 0;
  Uart_ResetCount++;
  
  if ((Uart_ResetCount%6) == 0)
  {
    system_ResetModuleConn();
    
    if (Uart_ResetCount >= 12)
    {
      NVIC_SystemReset();
    }
  }
}

/**
* @brief This function return status of Data Read flag of packet received from
         SP1ML
*/
void system_ResetModuleConn(void)
{
  GPIOB->BRR = (uint32_t)GPIO_PIN_14;
  HAL_Delay(1);
  GPIOB->BSRR = (uint32_t)GPIO_PIN_14;    
}

/**
* @brief This function return status of Data Read flag of packet received from
         SP1ML
*/
bool system_GetReadDataPktFlag(void)
{
  return ReadDataPkt;
}

/**
* @brief This function set status of Data Read flag of packet sent from SP1ML
*/
void system_SetReadDataPktFlag(bool set)
{
  ReadDataPkt = set;
}

/**
* @brief This function get flag of 10ms counter
*/
bool system_GetBattTick_10(void)
{
  return Batt_Tick_10;
}

/**
* @brief This function set flag of 10ms counter
*/
void system_SetBattTick_10(bool set)
{
  Batt_Tick_10 = set;
}

/**
* @brief This function get flag of 1000s counter
*/
bool system_GetBattTick_1000s(void)
{
  return Batt_Tick_1000s;
}

/**
* @brief This function set flag of 1000s counter
*/
void system_SetBattTick_1000s(bool set)
{
  Batt_Tick_1000s = set;
}

/**
* @brief Callback of ADC injected channel, output LED current
         This function take samples of output current and compute the median
         value of LED output current.
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  static uint32_t countLedI = 0;
  static uint32_t count1 = 0;
  static uint32_t count2 = 0;
  static uint32_t tmpVal = 0;
  
  LedI[countLedI] = hadc->Instance->JDR1;
  
  if (countLedI == NO_OF_INJ_SAMPLES-1)
  {
    /* Sort LedI */    
    for (count1 = 0 ; count1 < NO_OF_INJ_SAMPLES-1; count1++)
    {
      for (count2 = 0 ; count2 < NO_OF_INJ_SAMPLES-count1-1; count2++)
      {
        if (LedI[count2] > LedI[count2+1]) /* For decreasing order use < */
        {
          tmpVal = LedI[count2];
          LedI[count2] = LedI[count2+1];
          LedI[count2+1] = tmpVal;
        }
      }
    }
    
    /* Take median */
    LedCurrent = LedI[NO_OF_INJ_SAMPLES/2];
  }    
  
  countLedI +=1;
  if (countLedI == NO_OF_INJ_SAMPLES)
    countLedI = 0;
  
  if (LedCurrent > LED_I_OC)
  {
    if (!(system_GetIgnoreLedCurrent()))
    {
      dc_dc_ld_SetFlag(DC_DC_LD_MASK_OVER_CURRENT);
      system_DcDcLdOutDisable();
      system_AcDcLdOutDisable();
    }
  }
}

/**
* @brief Callback of Timer17 for regulating output current of DC DC boost LED
         driver
         This function calculate difference between current output current and 
         threshold output current and accordingly adjust duty cycle of HRTIM
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Led current loop regulation */
  LedCurrentErrorP = LedI_CurrTh - LedCurrent;
  if (LedCurrentErrorP < -100)
    LedCurrentErrorP = -100;
  if (LedCurrentErrorP > 100)
    LedCurrentErrorP = 100;

  LedCurrentErrorI = (LedCurrentErrorI*15 + LedCurrentErrorP)/16;
  
  if (LedCurrentErrorI < -100)
    LedCurrentErrorI = -100;
  if (LedCurrentErrorI > 100)
    LedCurrentErrorI = 100;  

  LedCurrentGainP = LedCurrentErrorP*Kp;
  LedCurrentGainI = LedCurrentErrorI*Ki;
  
  LedCurrentGain = LedCurrentGainP + LedCurrentGainI;
  
  if (LedCurrent < LedI_CurrTh/2)
  {
    DcDcLdDutyChange = LedCurrentGain>>6;//LedCurrentGain>>2;
  }
  else
  {
    DcDcLdDutyChange = LedCurrentGain>>6;//LedCurrentGain>>4;
  }
  
  DcDcLdDutyPeriod += DcDcLdDutyChange;

  if (DcDcLdDutyPeriod < DcDcLdMinDutyPeriod)     
    DcDcLdDutyPeriod = DcDcLdMinDutyPeriod; 
  if (DcDcLdDutyPeriod > DcDcLdMaxDutyPeriod)     
    DcDcLdDutyPeriod = DcDcLdMaxDutyPeriod;    
  
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 
                         HRTIM_COMPAREUNIT_1, DcDcLdDutyPeriod);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 
                         HRTIM_COMPAREUNIT_1, DcDcLdDutyPeriod); 

  LedI_CurrTh = led_GetThreshI();
}

/**
* @brief Callback of UART
         Callback when packet is received over UART from SP1ML
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{ 
  conn_ReadData(UartHandle->Instance->RDR);
}

/**
* @brief Callback of Systick
*/
void HAL_SYSTICK_Callback(void)
{
  Tick_10++;
  if (Tick_10 > 10)
  {
    Batt_Tick_10 = true;
    Tick_10 = 0;
  }
  
  Tick_1000s++;
  if (Tick_1000s > 1000000)//reset after ~17 mins
  {
    Batt_Tick_1000s = true;
    Tick_1000s = 0;    
  }
  
  Tick_1h++;
  if (Tick_1h > 3600000)//reset after 60 mins
  {
//    NVIC_SystemReset();
  }
  
  Uart_Wait_Tick++;
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
