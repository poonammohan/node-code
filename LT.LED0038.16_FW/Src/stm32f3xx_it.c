/**
******************************************************************************
* @file    stm32f3xx_it.c
* @brief   Interrupt Service Routines.
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
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "led.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef ThreeSlDcDcLdLoopTim;
extern TIM_HandleTypeDef htm16; //Added by Madhava
extern UART_HandleTypeDef Sp1mlUart;
extern WWDG_HandleTypeDef WwdgHandle;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
  {

    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();

  }

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
  {

  }

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
  {
    /* Set ADC state */
    SET_BIT(hadc1.State, HAL_ADC_STATE_INJ_EOC);

    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);

    /* Clear injected group conversion flag */
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  }

/**
* @brief This function handles TIM1 trigger and commutation and TIM17 interrupts.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&ThreeSlDcDcLdLoopTim, TIM_IT_UPDATE) !=RESET)
      {
        __HAL_TIM_CLEAR_IT(&ThreeSlDcDcLdLoopTim, TIM_IT_UPDATE);
        HAL_TIM_PeriodElapsedCallback(&ThreeSlDcDcLdLoopTim);
      }
  }


/**
* @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXT line 28.
*/
void USART3_IRQHandler(void)
  {
    if (__HAL_UART_GET_IT(&Sp1mlUart, UART_IT_RXNE))
      {
        HAL_UART_RxCpltCallback(&Sp1mlUart);
      }
  }

/**
* @brief This function handles WWDG global interrupt.
*/
void WWDG_IRQHandler (void)
  {
    __HAL_WWDG_CLEAR_FLAG(&WwdgHandle, WWDG_FLAG_EWIF);
  }

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)    //Added by Madhava
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);

  }
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //Added by Madhava
  if (GPIO_Pin == GPIO_PIN_10) {

#if 0
    if(led_GetFlag(LED_MASK_DIM_ENABLE))        // Added By Chinna
      {
        led_SetFlag(LED_MASK_ENABLE);
      }
#endif
    //  if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != SET)
    //{
    //led_ResetFlag(LED_MASK_ENABLE);
    led_ResetFlag(LED_MASK_DIM_ENABLE);
    //}
#if 0
    if(dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE) || ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
      {
        led_ResetFlag(LED_MASK_DIM_ENABLE);

      }
#endif


#if 0   //disabled by Chinna
    //HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htm16,TIM_CHANNEL_1);
    while( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10));
    HAL_TIM_PWM_Stop(&htm16,TIM_CHANNEL_1);
    //HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
#endif
  }else {
    //DO NOTHING
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
