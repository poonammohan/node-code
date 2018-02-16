/**
 ******************************************************************************
 * @file    led6001.c
 * @author  CL
 * @version V1.0.0
 * @date    30-Sep-2015
 * @brief   This file provides a set of functions needed to manage led6001.
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
#include "led6001.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup LED6001
 * @{
 */
static LED_DRIVER_StatusTypeDef LED6001_Init(LED_DRIVER_InitTypeDef *LED6001_Init);
static void LED6001_DeInit(void);
static void LED6001_SetPwmDim(uint8_t PwmVal);
static void LED6001_SetAnaDim(uint8_t PwmVal);
static LED_DRIVER_StatusTypeDef LED6001_FaultStatus(void);


/** @defgroup LED6001_Private_Variables LED6001_Private_Variables
 * @{
 */

LED_DRIVER_DrvTypeDef LED6001Drv =
{
  LED6001_Init,
  LED6001_DeInit,
  LED6001_SetPwmDim,
  LED6001_SetAnaDim,
  LED6001_FaultStatus,
  NULL
};


/**
 * @}
 */

/** @defgroup LED6001_Private_Functions LED6001_Private_Functions
 * @{
 */

/**
 * @brief  Set LED6001 Initialization
 * @param  LED6001_Init the configuration setting for the LED6001
 * @retval LED_DRIVER_OK in case of success, an error code otherwise
 */
static LED_DRIVER_StatusTypeDef LED6001_Init(LED_DRIVER_InitTypeDef *LED6001_Init)
{
  /* Configure the low level interface ---------------------------------------*/
  if(LED6001_IO_Init() != LED_DRIVER_OK)
  {
    return LED_DRIVER_ERROR;
  }
  
  LED6001_IO_ITConfig();

  return LED_DRIVER_OK;
}


/**
 * @brief  Deinitialize LED6001 interface
 * @param  None
 * @retval None
 */
static void LED6001_DeInit(void)
{
  LED6001_IO_DeInit();
}


/**
 * @brief  Set pwm dim value
 * @param  Dim value
 * @retval None
 */
static void LED6001_SetPwmDim(uint8_t PwmVal)
{
  LED6001_SetPdimTimer(PwmVal);
}


/**
 * @brief  Set analog dim value
 * @param  Dim value
 * @retval None
 */
static void LED6001_SetAnaDim(uint8_t PwmVal)
{
  LED6001_SetAdimTimer(PwmVal);
}


/**
 * @brief  Return fault status of LED 6001
 * @param  None
 * @retval Fault status
 */
static LED_DRIVER_StatusTypeDef LED6001_FaultStatus(void)
{
  return LED_DRIVER_OK;
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

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
