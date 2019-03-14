/**
  ******************************************************************************
  * @file    dc_dc_ld.h
  * @brief   Header file for dc_dc_ld.c
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
#ifndef __DC_DC_LD_H
#define __DC_DC_LD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common.h"

#define DC_DC_LD_MASK_ENABLE             0x01
#define DC_DC_LD_MASK_OVER_VOLTAGE       0x02
#define DC_DC_LD_MASK_OVER_CURRENT       0x04

#define DC_DC_LD_DUTY_MAX                0.70
#define DC_DC_LD_DUTY_MIN                0.03   

#define DC_DC_LD_OUT_VOLT_MAX            3800//FOr OVP ~ 49V
/***************************Added by Sunil*************************************/
#define DC_DC_LD_OUT_VOLT_30             2327//FOr 30v ~ 30V
#define DC_DC_LD_OUT_VOLT_25             1940//FOr 25v ~ 25V
#define DC_DC_LD_OUT_VOLT_20             1551//FOr 20v ~ 20V
#define DC_DC_LD_OUT_VOLT_15             1164//FOr 15v ~ 15V
/***************************Added by Sunil*************************************/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void dc_dc_ld_UpdateOutVoltage(uint32_t*);
uint16_t dc_dc_ld_GetOutVoltage(void);
void dc_dc_ld_SetFlag(uint16_t flag);
void dc_dc_ld_ResetFlag(uint16_t flag);
bool dc_dc_ld_GetFlag(uint16_t flag);

#ifdef __cplusplus
}
#endif

#endif /* __DC_DC_LD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
