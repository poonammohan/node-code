/**
  ******************************************************************************
  * @file    panel.h
  * @brief   Header file for panel.c
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
#ifndef PANEL_H
#define PANEL_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common.h"
   
#define PANEL_ALL_FLAGS         0xFF   
#define PANEL_MASK_ENABLE       0x01
#define PANEL_MASK_PRESENT      0x02   
#define PANEL_MASK_OK           0x04   
#define PANEL_MASK_UNDER_V      0x08 
#define PANEL_MASK_OVER_V       0x10 
#define PANEL_MASK_REVERSE      0x20

#define PANEL_MINIMUM_VOLTAGE   2450//13.9 V
#define PANEL_MAXIMUM_VOLTAGE   4096
#define PANEL_MPPT_VOLTAGE      3075//17.5 V
#define PANEL_MIN_VOLTAGE       2550//14.5 V   
#define PANEL_MAX_VOLTAGE       4096   
#define PANEL_CURRENT_CONSTANT  0.0475
#define PANEL_VOLTAGE_CONSTANT  0.057
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint16_t panel_GetMinVoltage(void);
uint16_t panel_GetVoltage(void);
uint16_t panel_GetMpptVoltage(void);
void panel_UpdateVoltage(uint32_t*);
void panel_SetFlag(uint16_t);
void panel_ResetFlag(uint16_t);
bool panel_GetFlag(uint16_t);

#ifdef __cplusplus
}
#endif

#endif /* __PANEL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
