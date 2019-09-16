/**
  ******************************************************************************
  * @file    battery.h
  * @brief   Header file for battery.c
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
#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common.h"

#define BATTERY_ALL_FLAGS                       0xFF
#define BATTERY_MASK_ENABLE                     0x01
#define BATTERY_MASK_PRESENT                    0x02
#define BATTERY_MASK_CHRGD                      0x04
#define BATTERY_MASK_DSCHRG                     0x08   
#define BATTERY_MASK_DEEP_DSCHRG                0x10     
#define BATTERY_MASK_OK                         0x20   
#define BATTERY_MASK_SHRT_CKT                   0x40 //used for over current   
   
#define BATTERY_DISCHRG_VOLT_OFFSET_PA          40
   
#define BATTERY_MINIMUM_VOLTAGE                 1550//8 V
#define BATTERY_MIN_DISCHRG_VOLT                1650//2350//Sunil//2350//12.2V at 0A discharge current 2200--->11.44,2080---->10.816   
#define BATTERY_FLOAT_VOLTAGE                   2630//2653//(2630-->13.7 V)(2653-->13.8 V)
#define BATTERY_MAXIMUM_VOLTAGE                 3000//(2825-->14.7 V)(2999-->15.6 V)
#define BATTERY_AC_DC_SWITCH_HIGH_CUT_OFF       3500
#define BATTERY_AC_DC_SWITCH_LOW_CUT_OFF        1000   
#define BATTERY_AC_DC_SWITCH_MIN_DISCHRG_VOLT   2800
#define BATTERY_FLOAT_CURRENT_TH                200//~ 1A
#define BATTERY_MAXIMUM_CHARGING_CURRENT        3000//3000//3000//2000//1538//(3000//12 A)----Changed by sunil as 1538-->6.25amp-->75 Ah battery//246i-->ADC value 
#define BATTERY_FLOAT_CURRENT_OFFSET            100   
#define BATTERY_DEEP_DISCHRG_THRESHOLD          1540//8V   
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BATTERY_CHARGING_MODE_BULK,
  BATTERY_CHARGING_MODE_TOPPING,
  BATTERY_CHARGING_MODE_FLOAT,
  BATTERY_CHARGING_MODE_OFF
} BatteryChargingModeTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint16_t battery_GetVoltage(void);
void battery_UpdateVoltage(uint32_t*,int32_t*);
uint16_t battery_GetMinVoltage(void);
uint16_t battery_GetMinDischrgVoltage(void);
uint16_t battery_GetFltVoltage(void);
uint16_t battery_GetMaxVoltage(void);
uint16_t battery_GetChargngCurrent(void);
uint16_t battery_GetDischargngCurrent(void);
uint16_t battery_GetMaxCurrent(void);
void battery_UpdateChgCurrent(uint32_t*);
uint16_t battery_GetFltCurrent(void);
void battery_SetFlag(uint16_t);
void battery_ResetFlag(uint16_t);
bool battery_GetFlag(uint16_t flag);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
