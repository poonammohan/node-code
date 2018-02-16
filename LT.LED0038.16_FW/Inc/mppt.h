/**
  ******************************************************************************
  * @file    mppt.h
  * @brief   Header file for mppt.c
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
#ifndef __MPPT_H
#define __MPPT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common.h"
   
#define MPPT_PWM_DUTY_MAX       1000//same as timer period ARR value
#define MPPT_PWM_DUTY_UB        0.87*MPPT_PWM_DUTY_MAX//duty cycle can never cross 900 (on 1000 scale)
#define MPPT_PWM_DUTY_RESET     0.55*MPPT_PWM_DUTY_MAX
   
#define MPPT_POSITIVE                1
#define MPPT_NEGATIVE                0   
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MPPT_DUTY_INC,
  MPPT_DUTY_DEC,
  MPPT_DUTY_NO_CHANGE
} MpptDutyChangeTypeDef;
   
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


uint16_t mppt_Routine(uint16_t,uint16_t,uint16_t,uint16_t,uint16_t);
void mppt_Reinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPPT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
