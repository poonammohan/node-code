/**
  ******************************************************************************
  * @file    state.h
  * @brief   Header file for state.c
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
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STATE_H
#define __STATE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"


/* Exported types ------------------------------------------------------------*/
   
typedef enum 
{  
  SCC_STATE_FAC_SETTING,
  SCC_STATE_OFF,
  SCC_STATE_START_UP, 
  SCC_STATE_BATT_CHG,
  SCC_STATE_CRITICAL
} SCC_StateTypeDef;
    
typedef enum 
{  
  LED_STATE_FAC_SETTING,
  LED_STATE_OFF,
  LED_STATE_START_UP, 
  LED_STATE_ON,
  LED_STATE_CRITICAL
} LED_StateTypeDef;

typedef enum 
{  
  Conn_STATE_START_UP, 
  Conn_STATE_ON
} Conn_StateTypeDef;

typedef struct
{
  LED_StateTypeDef state_LED;
  SCC_StateTypeDef state_SCC;
  Conn_StateTypeDef state_Conn;
} ThreeSL_StateTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void SM_ThreeSl(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
