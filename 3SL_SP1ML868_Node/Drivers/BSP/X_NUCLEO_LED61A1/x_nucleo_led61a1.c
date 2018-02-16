/**
 ******************************************************************************
 * @file    x_nucleo_led61a1.c
 * @author  CL
 * @version V1.0.0
 * @date    30-September-2015
 * @brief   This file provides X_NUCLEO_LED61A1 LED6001 shield board specific functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "x_nucleo_led61a1.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup X_NUCLEO_LED61A1
 * @{
 */

/** @defgroup X_NUCLEO_LED61A1_Private_Defines X_NUCLEO_LED61A1_Private_Defines
 * @{
 */
#ifndef NULL
  #define NULL      (void *) 0
#endif
/**
 * @}
 */

/** @defgroup X_NUCLEO_LED61A1_Private_Variables X_NUCLEO_LED61A1_Private_Variables
 * @{
 */

static LED_DRIVER_DrvTypeDef *LedDriverDrv = NULL;
static uint8_t LedDriverInitialized = 0;

static TIM_HandleTypeDef        TIM_EXPBD_ADIM_Handle;
static TIM_HandleTypeDef        TIM_EXPBD_PDIM_Handle;
static ADC_HandleTypeDef        ADC_EXPBD_Handle;
/**
 * @}
 */

/* Link function for LED Driver */
LED_DRIVER_StatusTypeDef LED6001_IO_Init(void);
void LED6001_IO_DeInit(void);
void LED6001_IO_ITConfig(void);
void LED6001_SetPdimTimer(uint8_t PwmVal);
void LED6001_SetAdimTimer(uint8_t PwmVal);


static LED_DRIVER_StatusTypeDef LED_DRIVER_IO_Init(void);
static void LED_DRIVER_IO_DeInit(void);
static void Photo_Sensor_IO_Init(void);
static void TIM_EXPBD_ADIM_MspInit(void);
static void TIM_EXPBD_PDIM_MspInit(void);
static void ADC_EXPBD_MspInit(void);
static HAL_StatusTypeDef TIM_EXPBD_Init(void);
static void ADC_EXPBD_Init(void);

/** @defgroup X_NUCLEO_LED61A1_Exported_Functions X_NUCLEO_LED61A1_Exported_Functions
 * @{
 */
/**
 * @brief  Get value of analog dimming level
 * @param  None
 * @retval Analog dimming level
 */
uint8_t BSP_LED_DRIVER_GetAdim()
{
  return (uint8_t)(((__HAL_TIM_GetCompare(&TIM_EXPBD_ADIM_Handle, \
                  X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CHANNEL))*100)  \
                   /X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PERIOD);  
}


/**
 * @brief  Get value of pwm dimming level
 * @param  None
 * @retval Pwm dimming level
 */
uint8_t BSP_LED_DRIVER_GetPdim()
{
  return (uint8_t)(((__HAL_TIM_GetCompare(&TIM_EXPBD_PDIM_Handle, \
                  X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CHANNEL))*100)  \
                   /X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PERIOD);
}


/**
 * @brief  Configures LED driver interface
 * @param  None
 * @retval LED_DRIVER_OK in case of success
 */
LED_DRIVER_StatusTypeDef BSP_LED_DRIVER_Init(void)
{
  LED_DRIVER_InitTypeDef InitStructure;
  
  if (!LedDriverInitialized)
  {
    LedDriverDrv = &LED6001Drv;
    InitStructure.PdimVal = LED6001_PDIM_FULL;
    InitStructure.AdimVal = LED6001_ADIM_FULL;
    
    if ( LedDriverDrv->Init == NULL)
    {
      LedDriverDrv = NULL;
      return LED_DRIVER_ERROR;
    }
    
    if (LedDriverDrv->Init(&InitStructure) != LED_DRIVER_OK)
    {
      LedDriverDrv = NULL;
      return LED_DRIVER_ERROR;
    }
  }
  LedDriverInitialized = 1;
  return LED_DRIVER_OK;  
}

void BSP_LED_DRIVER_DeInit(void)
{
  LedDriverDrv->SetPwmDim(0);
//  LedDriverDrv = &LED6001Drv;
//  LedDriverDrv->DeInit();
//  LedDriverInitialized = 0;
}

/**
 * @brief  Get status of LED driver initialization
 * @param  None
 * @retval LED driver initialization status
 */
uint8_t BSP_LED_DRIVER_isInitialized(void)
{
  return LedDriverInitialized;
}


/**
 * @brief  Get photo sensor output level
 * @param  None
 * @retval Return value of photo sensor
 */
uint32_t BSP_Photo_Sensor_GetVal(void)
{
  return(HAL_ADC_GetValue(&ADC_EXPBD_Handle));
}


/**
 * @brief  Configures photo sensor interface
 * @param  None
 * @retval None
 */
void BSP_PS_Init(void)
{
  Photo_Sensor_IO_Init();  
}


/**
 * @brief  Set analog dimming level
 * @param  Dimming level
 * @retval None
 */
void BSP_SetAnaDim(uint8_t AnaDim)
{
  static uint8_t prevAnaDim = 0;
  
  if (prevAnaDim != AnaDim)
  {
    LedDriverDrv->SetAnaDim(AnaDim);
  }
  prevAnaDim = AnaDim;
}   


/**
 * @brief  Set pwm dimming level
 * @param  Dimming level
 * @retval None
 */
void BSP_SetPwmDim(uint8_t PwmDim)
{
  static uint8_t prevPwmDim = 0;
  
  if (prevPwmDim != PwmDim)
  {
    LedDriverDrv->SetPwmDim(PwmDim);
  }
  prevPwmDim = PwmDim;
}
   

/********************************* LINK LED DRIVER *****************************/
/**
 * @brief  Configures LED6001 
 * @param  None
 * @retval LED_DRIVER_OK in case of success, an error code otherwise
 */
LED_DRIVER_StatusTypeDef LED6001_IO_Init(void)
{   
    return LED_DRIVER_IO_Init();
}


/**
 * @brief  Reset LED6001 
 * @param  None
 * @retval None
 */
void LED6001_IO_DeInit(void)
{
  LED_DRIVER_IO_DeInit();
}
/**
 * @brief  Configures LED6001 interrupt lines for NUCLEO boards
 * @param  None
 * @retval None
 */
void LED6001_IO_ITConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  
  /* Enable XFAULT GPIO clock */
  LED_DRIVER_FAULT_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = LED_DRIVER_FAULT_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FAST;

  GPIO_InitStructureInt1.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(LED_DRIVER_FAULT_GPIO_PORT, &GPIO_InitStructureInt1);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LED_DRIVER_FAULT_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(LED_DRIVER_FAULT_EXTI_IRQn);
  
}


/**
 * @brief  Set analog dimming timer level
 * @param  Dimming level
 * @retval None
 */
void LED6001_SetAdimTimer(uint8_t PwmVal)
{
  __HAL_TIM_SetCompare(&TIM_EXPBD_ADIM_Handle,\
                        X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CHANNEL,\
                  ((uint32_t)PwmVal*X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PERIOD)/100);  
}


/**
 * @brief  Set pwm dimming timer level
 * @param  Dimming level
 * @retval None
 */
void LED6001_SetPdimTimer(uint8_t PwmVal)
{
  __HAL_TIM_SetCompare(&TIM_EXPBD_PDIM_Handle,\
                        X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CHANNEL,\
                  ((uint32_t)PwmVal*X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PERIOD)/100);
}


/** @defgroup X_NUCLEO_LED61A1_Private_Functions X_NUCLEO_LED61A1_Private_Functions
 * @{
 */
/**
 * @brief  Configures Timer and EXTI for LED_DRIVER
 * @param  None
 * @retval LED_OK in case of success, an error code otherwise
 */
static LED_DRIVER_StatusTypeDef LED_DRIVER_IO_Init(void)
{
    if(TIM_EXPBD_Init() != HAL_OK)
    {
      return LED_DRIVER_ERROR;
    }
    
    return LED_DRIVER_OK;
}


/**
 * @brief  Deinitialize LED driver
 * @param  None
 * @retval None
 */
static void LED_DRIVER_IO_DeInit(void)
{
 /* To be implemented */
}


/**
 * @brief  Configure ADC for photo sensor 
 * @param  None
 * @retval None
 */
void Photo_Sensor_IO_Init(void)
{
  ADC_EXPBD_Init();
}


/**
 * @brief  Configures ADC interface
 * @param  None
 * @retval None
 */
void ADC_EXPBD_Init(void)
{
  ADC_ChannelConfTypeDef ADC_EXPBD_Ch_Config;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  ADC_EXPBD_Handle.Instance = ADC1;
#ifdef STM32L053xx
  ADC_EXPBD_Handle.Init.OversamplingMode = DISABLE;
  ADC_EXPBD_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
  ADC_EXPBD_Handle.Init.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  ADC_EXPBD_Handle.Init.Overrun = OVR_DATA_PRESERVED;
  ADC_EXPBD_Handle.Init.LowPowerAutoWait = DISABLE;
  ADC_EXPBD_Handle.Init.LowPowerFrequencyMode = DISABLE;
  ADC_EXPBD_Handle.Init.LowPowerAutoOff = DISABLE; 
  ADC_EXPBD_Handle.Init.ScanDirection = ADC_SCAN_DIRECTION_UPWARD;
#endif
#ifdef STM32F401xE
  ADC_EXPBD_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  ADC_EXPBD_Handle.Init.ScanConvMode = DISABLE;
  ADC_EXPBD_Handle.Init.NbrOfConversion = 1;
#endif  
  ADC_EXPBD_Handle.Init.Resolution = ADC_RESOLUTION12b;
  ADC_EXPBD_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC_EXPBD_Handle.Init.ContinuousConvMode = ENABLE     ;
  ADC_EXPBD_Handle.Init.DiscontinuousConvMode = DISABLE;
  ADC_EXPBD_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
  ADC_EXPBD_Handle.Init.DMAContinuousRequests = DISABLE;
  ADC_EXPBD_Handle.Init.EOCSelection = EOC_SEQ_CONV;


    /**Configure for the selected ADC regular channel to be converted. 
    */
  ADC_EXPBD_Ch_Config.Channel = ADC_CHANNEL_8;
#ifdef STM32F401xE  
  ADC_EXPBD_Ch_Config.Rank = 1;
  ADC_EXPBD_Ch_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;   
#endif 
  
  ADC_EXPBD_MspInit();
  HAL_ADC_Init(&ADC_EXPBD_Handle);
  HAL_ADC_ConfigChannel(&ADC_EXPBD_Handle, &ADC_EXPBD_Ch_Config);  
  HAL_ADC_Start(&ADC_EXPBD_Handle);
}


/**
 * @brief  ADC MSP Initialization
 * @param  None 
 * @retval None
 */
void ADC_EXPBD_MspInit(void)
{    
  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(ADC_EXPBD_Handle.Instance == ADC1)
  {

    /* Peripheral clock enable */
    __ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PB0     ------> ADC_IN8 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }  
}


/**
 * @brief  Configures TIM interface
 * @param  None
 * @retval HAL_OK if success
 */
static HAL_StatusTypeDef TIM_EXPBD_Init(void)
{
  HAL_StatusTypeDef ret_val = HAL_OK;  
#ifdef STM32L053xx  
  TIM_ClockConfigTypeDef TIM_EXPBD_PDIM_CLK_SRC_Config;
#endif
  TIM_ClockConfigTypeDef TIM_EXPBD_ADIM_CLK_SRC_Config;
  TIM_MasterConfigTypeDef TIM_EXPBD_PDIM_MASTER_Config;
  TIM_MasterConfigTypeDef TIM_EXPBD_ADIM_MASTER_Config;
  TIM_OC_InitTypeDef TIM_EXPBD_PDIM_OC_Init;
  TIM_OC_InitTypeDef TIM_EXPBD_ADIM_OC_Init;    
  
  if (HAL_TIM_PWM_GetState(&TIM_EXPBD_PDIM_Handle) == HAL_TIM_STATE_RESET)
  {
    /* TIM_EXPBD_PDIM peripheral configuration */
    TIM_EXPBD_PDIM_Handle.Instance = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM;
    TIM_EXPBD_PDIM_Handle.Init.Prescaler = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PRESCALER;
    TIM_EXPBD_PDIM_Handle.Init.CounterMode = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_COUNTERMODE;
    TIM_EXPBD_PDIM_Handle.Init.Period = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PERIOD;
    TIM_EXPBD_PDIM_Handle.Init.ClockDivision = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CLOCKDIVISION;
#ifdef STM32L053xx   
    TIM_EXPBD_PDIM_CLK_SRC_Config.ClockSource = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CLOCKSOURCE;    
#endif    
    TIM_EXPBD_PDIM_MASTER_Config.MasterOutputTrigger = TIM_TRGO_RESET;
    TIM_EXPBD_PDIM_MASTER_Config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    
    TIM_EXPBD_PDIM_OC_Init.OCMode = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCMODE;
    TIM_EXPBD_PDIM_OC_Init.Pulse = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PULSE;
    TIM_EXPBD_PDIM_OC_Init.OCPolarity = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCPOLARITY;
    TIM_EXPBD_PDIM_OC_Init.OCFastMode = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCFASTMODE;
    
    /* Init the PDIM Timer */
    TIM_EXPBD_PDIM_MspInit();
#ifdef STM32L053xx    
    ret_val = HAL_TIM_Base_Init(&TIM_EXPBD_PDIM_Handle);
    if (ret_val != HAL_OK)      return ret_val;
    ret_val = HAL_TIM_ConfigClockSource(&TIM_EXPBD_PDIM_Handle, &TIM_EXPBD_PDIM_CLK_SRC_Config);    
    if (ret_val != HAL_OK)      return ret_val;    
#endif    
    ret_val = HAL_TIM_PWM_Init(&TIM_EXPBD_PDIM_Handle);
    if (ret_val != HAL_OK)      return ret_val;    
    ret_val = HAL_TIMEx_MasterConfigSynchronization(&TIM_EXPBD_PDIM_Handle, &TIM_EXPBD_PDIM_MASTER_Config);        
    if (ret_val != HAL_OK)      return ret_val;    
    ret_val = HAL_TIM_PWM_ConfigChannel(&TIM_EXPBD_PDIM_Handle, &TIM_EXPBD_PDIM_OC_Init, X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CHANNEL);    
    if (ret_val != HAL_OK)      return ret_val;
    ret_val = HAL_TIM_PWM_Start(&TIM_EXPBD_PDIM_Handle,X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CHANNEL);
    if (ret_val != HAL_OK)      return ret_val;  
  }
    
  if (HAL_TIM_PWM_GetState(&TIM_EXPBD_ADIM_Handle) == HAL_TIM_STATE_RESET)
  {
    /* TIM_EXPBD_ADIM peripheral configuration */
    TIM_EXPBD_ADIM_Handle.Instance = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM;
    TIM_EXPBD_ADIM_Handle.Init.Prescaler = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PRESCALER;
    TIM_EXPBD_ADIM_Handle.Init.Period = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PERIOD;
    TIM_EXPBD_ADIM_Handle.Init.ClockDivision = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CLOCKDIVISION;
    
    TIM_EXPBD_ADIM_CLK_SRC_Config.ClockSource = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CLOCKSOURCE;    
    
    TIM_EXPBD_ADIM_MASTER_Config.MasterOutputTrigger = TIM_TRGO_RESET;
    TIM_EXPBD_ADIM_MASTER_Config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    
    TIM_EXPBD_ADIM_OC_Init.OCMode = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCMODE;
    TIM_EXPBD_ADIM_OC_Init.Pulse = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PULSE;
    TIM_EXPBD_ADIM_OC_Init.OCPolarity = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCPOLARITY;
    TIM_EXPBD_ADIM_OC_Init.OCFastMode = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCFASTMODE;

    /* Init the ADIM Timer */
    TIM_EXPBD_ADIM_MspInit();
    ret_val = HAL_TIM_Base_Init(&TIM_EXPBD_ADIM_Handle);
    if (ret_val != HAL_OK)      return ret_val;
    ret_val = HAL_TIM_ConfigClockSource(&TIM_EXPBD_ADIM_Handle,\
                                        &TIM_EXPBD_ADIM_CLK_SRC_Config);    
    if (ret_val != HAL_OK)      return ret_val;    
    ret_val = HAL_TIM_PWM_Init(&TIM_EXPBD_ADIM_Handle);
    if (ret_val != HAL_OK)      return ret_val;    
    ret_val = HAL_TIMEx_MasterConfigSynchronization(&TIM_EXPBD_ADIM_Handle,\
                                                &TIM_EXPBD_ADIM_MASTER_Config);        
    if (ret_val != HAL_OK)      return ret_val;    
    ret_val = HAL_TIM_PWM_ConfigChannel(&TIM_EXPBD_ADIM_Handle,\
                                        &TIM_EXPBD_ADIM_OC_Init,\
                                        X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CHANNEL);    
    if (ret_val != HAL_OK)      return ret_val;
    ret_val = HAL_TIM_PWM_Start(&TIM_EXPBD_ADIM_Handle,\
                                X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CHANNEL);
    if (ret_val != HAL_OK)      return ret_val;
  }
  return ret_val;
}


/**
 * @brief  ADIM Timer MSP Initialization
 * @param  None
 * @retval None
 */
static void TIM_EXPBD_ADIM_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  X_NUCLEO_TIM_EXPBD_ADIM_PORT_CLK_ENABLE();
  X_NUCLEO_TIM_EXPBD_ADIM_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PIN;
  GPIO_InitStruct.Mode = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_MODE;
  GPIO_InitStruct.Pull = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PULL;
  GPIO_InitStruct.Speed = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_SPEED;
  GPIO_InitStruct.Alternate = X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_ALTERNATE;
  HAL_GPIO_Init(X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PORT, &GPIO_InitStruct);  
}


/**
 * @brief  PDIM Timer MSP Initialization
 * @param  None
 * @retval None
 */
static void TIM_EXPBD_PDIM_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Peripheral clock enable */
  X_NUCLEO_TIM_EXPBD_PDIM_PORT_CLK_ENABLE();
  X_NUCLEO_TIM_EXPBD_PDIM_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PIN;
  GPIO_InitStruct.Mode = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_MODE;
  GPIO_InitStruct.Pull = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PULL;
  GPIO_InitStruct.Speed = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_SPEED;
  GPIO_InitStruct.Alternate = X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_ALTERNATE;
  HAL_GPIO_Init(X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PORT, &GPIO_InitStruct);
}


/**
 * @brief  ADIM Timer duty set
 * @param  Duty value
 * @retval None
 */
void Set_TIM_EXPBD_ADIM_Duty(uint8_t level)
{
  /* To be implemented */
}


/**
 * @brief  PDIM Timer duty set
 * @param  Duty value
 * @retval None
 */
void Set_TIM_EXPBD_PDIM_Duty(uint8_t level)
{
  /* To be implemented */
}


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
