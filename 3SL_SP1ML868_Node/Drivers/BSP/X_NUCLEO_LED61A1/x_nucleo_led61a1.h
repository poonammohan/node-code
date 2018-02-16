/**
  ******************************************************************************
  * @file    x_nucleo_led61a1.h
  * @author  CL
  * @version V1.0.0
  * @date    30-September-2015
  * @brief   This file contains definitions for the x_nucleo_led61a1.c 
  *          board specific functions.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_LED61A1_H
#define __X_NUCLEO_LED61A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#endif
#ifdef STM32L053xx
#include "stm32l0xx_hal.h"
#endif

#include "led_driver.h"
#include "led6001.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_LED61A1
  * @{
  */

/** @defgroup X_NUCLEO_LED61A1_Exported_Types X_NUCLEO_LED61A1_Exported_Types
  * @{
  */
typedef enum
{
  PWM_DIMMING_STEPS,
  ANALOG_DIMMING_STEPS,
  PWM_DIMMING_VARY,
  ANALOG_DIMMING_VARY,
  ANALOG_DIMMING_PHOTO
}LED_DRIVER_Demo;
/**
  * @}
  */

/** @defgroup X_NUCLEO_LED61A1_Exported_Defines X_NUCLEO_LED61A1_Exported_Defines
  * @{
  */


/* TIM peripheral for PWM dimming configuration defines */
#define X_NUCLEO_TIM_EXPBD_PDIM_PORT_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#define X_NUCLEO_TIM_EXPBD_PDIM_CLK_ENABLE()            __TIM2_CLK_ENABLE()   
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PIN             GPIO_PIN_10   
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PORT            GPIOB   
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_MODE            GPIO_MODE_AF_PP 
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PULL            GPIO_NOPULL
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_SPEED           GPIO_SPEED_LOW
#ifdef STM32L053xx
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_ALTERNATE       GPIO_AF2_TIM2
#endif
#ifdef STM32F401xE
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_ALTERNATE       GPIO_AF1_TIM2
#endif    
    

#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM                 TIM2
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PRESCALER       4
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_COUNTERMODE     TIM_COUNTERMODE_UP
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PERIOD          64000
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CLOCKDIVISION   TIM_CLOCKDIVISION_DIV1
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CLOCKSOURCE     TIM_CLOCKSOURCE_INTERNAL
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCMODE          TIM_OCMODE_PWM1
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_PULSE           5000//65000
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCPOLARITY      TIM_OCPOLARITY_HIGH
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_OCFASTMODE      TIM_OCFAST_DISABLE
#define X_NUCLEO_LED61A1_TIM_EXPBD_PDIM_CHANNEL         TIM_CHANNEL_3

/* TIM peripheral for analog dimming configuration defines */
#define X_NUCLEO_TIM_EXPBD_ADIM_PORT_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#ifdef STM32L053xx 
#define X_NUCLEO_TIM_EXPBD_ADIM_CLK_ENABLE()            __TIM22_CLK_ENABLE() 
#endif
#ifdef STM32F401xE
#define X_NUCLEO_TIM_EXPBD_ADIM_CLK_ENABLE()            __TIM3_CLK_ENABLE() 
#endif    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PIN             GPIO_PIN_4    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PORT            GPIOB   
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_MODE            GPIO_MODE_AF_PP 
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PULL            GPIO_NOPULL
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_SPEED           GPIO_SPEED_LOW
#ifdef STM32L053xx     
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_ALTERNATE       GPIO_AF4_TIM22   
#endif
#ifdef STM32F401xE
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_ALTERNATE       GPIO_AF2_TIM3
#endif    
    
#ifdef STM32L053xx    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM                 TIM22    
#endif
#ifdef STM32F401xE    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM                 TIM3 
#endif    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PRESCALER       4
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PERIOD          64000
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CLOCKDIVISION   TIM_CLOCKDIVISION_DIV1
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CLOCKSOURCE     TIM_CLOCKSOURCE_INTERNAL
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCMODE          TIM_OCMODE_PWM1
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_PULSE           32000//65000
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCPOLARITY      TIM_OCPOLARITY_HIGH
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_OCFASTMODE      TIM_OCFAST_DISABLE    
#define X_NUCLEO_LED61A1_TIM_EXPBD_ADIM_CHANNEL         TIM_CHANNEL_1

#define ZERO                                            0
#define MIN_BRIGHT_PDIM                                 10
#define MIN_BRIGHT_ADIM                                 10
#define MAX_BRIGHT_PDIM                                 100    
#define MAX_BRIGHT_ADIM                                 40


/* Definition for interrupt Pins */
#define LED_DRIVER_FAULT_GPIO_PORT              GPIOB
#define LED_DRIVER_FAULT_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define LED_DRIVER_FAULT_GPIO_CLK_DISABLE()     __GPIOB_CLK_DISABLE()
#define LED_DRIVER_FAULT_PIN                    GPIO_PIN_5     
     
#if (defined (USE_STM32F4XX_NUCLEO))
#define LED_DRIVER_FAULT_EXTI_IRQn           EXTI9_5_IRQn
#endif

#if (defined (USE_STM32L0XX_NUCLEO))
#define LED_DRIVER_FAULT_EXTI_IRQn           EXTI4_15_IRQn
#endif     
     
   
/** @defgroup X_NUCLEO_LED61A1_Exported_Functions X_NUCLEO_LED61A1_Exported_Functions
 * @{
 */
/* LED driver configuration functions */    
uint8_t BSP_LED_DRIVER_isInitialized(void);
LED_DRIVER_StatusTypeDef BSP_LED_DRIVER_Init(void);
void BSP_LED_DRIVER_DeInit(void);
uint8_t BSP_LED_DRIVER_GetPdim(void);
uint8_t BSP_LED_DRIVER_GetAdim(void);
void BSP_SetPwmDim(uint8_t PwmDim);
void BSP_SetAnaDim(uint8_t AnaDim);
void BSP_PS_Init(void);
uint32_t BSP_Photo_Sensor_GetVal(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* __X_NUCLEO_IKS01A1_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
