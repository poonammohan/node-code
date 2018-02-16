/**tttttttttt
  ******************************************************************************
  * File Name          : stm32f3xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
#include "stm32f3xx_hal.h"

extern DMA_HandleTypeDef hdma_adc1;


/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  __SYSCFG_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/**
  * @brief ADC MSP Initialization
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock enable */
    __ADC12_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9  
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }

}

/**
  * @brief ADC MSP Deinitialization
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock disable */
    __ADC12_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);

  }

}

/**
  * @brief WWDG MSP Initialization
  */
void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hhrtim)
{

  if(hhrtim->Instance==HRTIM1)
  {
    /* Peripheral clock enable */
    __HRTIM1_CLK_ENABLE();
  }

}

/**
  * @brief HHRTIM MSP Initialization
  */
void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hhrtim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hhrtim->Instance==HRTIM1)
  {
  
    /**HRTIM1 GPIO Configuration    
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }

}

/**
  * @brief HHRTIM MSP Deinitialization
  */
void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hhrtim)
{

  if(hhrtim->Instance==HRTIM1)
  {
    /* Peripheral clock disable */
    __HRTIM1_CLK_DISABLE();
  }

}

/**
  * @brief Timer2 MSP Initialization
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();
  }

}

/**
  * @brief Timer3, Timer6, Timer16, Timer17 MSP Initialization
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  
  if(htim_base->Instance==TIM6)
  {
    /* Peripheral clock enable */
    __TIM6_CLK_ENABLE();
  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM6_DAC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
  }  
  
  if(htim_base->Instance==TIM16)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();
  }
  
  if(htim_base->Instance==TIM17)
  {
    /* Peripheral clock enable */
    __TIM17_CLK_ENABLE();
  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
  }
  
  else if(htim_base->Instance==TIM15)
  {
    /* USER CODE END TIM15_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
  }  
  
}

/**
  * @brief Timer2, Timer3, Timer16 MSP Initialization
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2)
  {
  
    /**TIM2 GPIO Configuration    
    PA5     ------> TIM2_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }
  
  if(htim->Instance==TIM16)
  {
  
    /**TIM16 GPIO Configuration    
    PB8     ------> TIM16_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }  

  if(htim->Instance==TIM3)
  {
    
    /**TIM3 GPIO Configuration    
    PA4     ------> TIM3_CH2
    PA6     ------> TIM3_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  }
  
  if(htim->Instance==TIM15)
  {
    /**TIM15 GPIO Configuration    
    PA1     ------> TIM15_CH1N 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }  
  
}

/**
  * @brief Timer2 MSP Initialization
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();
  }

}

/**
  * @brief Timer 17 MSP Deinitialization
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM17)
  {
    /* Peripheral clock disable */
    __TIM17_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  }

}

/**
  * @brief UART MSP Initialization
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART3)
  {
    /* Peripheral clock enable */
    __USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }

}


/**
  * @brief WWDG MSP Initialization
  */
void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg)
{
  /* WWDG Peripheral clock enable */
  __HAL_RCC_WWDG_CLK_ENABLE();
  
/* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);  
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
