/**
  ******************************************************************************
  * @file    system.h
  * @brief   Header file for system.c
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
#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "common.h"
   
#define DC_DC_BOOST_PEIROD                      16000   
#define NO_OF_INJ_SAMPLES                       4
#define NO_OF_REG_SAMPLES                       4
#define NO_OF_REG_CHANNELS                      6   
   
#define THREE_SL_SCC_TIM                        TIM2   
#define THREE_SL_SCC_TIM_CHANNEL                TIM_CHANNEL_1
#define THREESL_SCC_PWM_DUTY_MAX                1000  
#define THREESL_SCC_PWM_DUTY_MIN                0.1*THREESL_SCC_PWM_DUTY_MAX
#define PWM_DUTY_CHANGE_STEP                    1
   
#define THREE_SL_DC_DC_LD_LOOP_TIM              TIM17  
   
#define THREE_SL_LED_EN_TIM                     TIM3 
#define THREE_SL_LED_EN_TIM_DC_DC_OUT_CH        TIM_CHANNEL_2
#define THREE_SL_LED_EN_TIM_AC_DC_OUT_CH        TIM_CHANNEL_1
#define THREE_SL_LED_EN_TIM_PERIOD              64
#define THREE_SL_LED_EN_TIM_OUT_ON              65   
#define THREE_SL_LED_EN_TIM_OUT_OFF             0   

#define THREE_SL_LED_ANA_DIM_TIM                TIM15   
#define THREE_SL_LED_ANA_TIM_OUT_CH             TIM_CHANNEL_1   
#define THREE_SL_LED_ANA_TIM_PRESC              1   
#define THREE_SL_LED_ANA_TIM_PERIOD             533//53332
   
#define ADC_BATT_VOLTAGE_INDEX                  1
#define ADC_BATT_CHARGING_CURRENT_INDEX         3
#define ADC_PANEL_VOLTAGE_INDEX                 0
#define ADC_DC_DC_LD_OUTPUT_VOLTAGE_INDEX       4
#define ADC_TEMPERATURE_SENSOR_VAL_INDEX        5
#define ADC_PHOTO_SENSOR_VAL_INDEX              2
   
#define AVG_BATT_VOLT_SAMPLES                   128
#define AVG_BATT_CHG_CURR_SAMPLES               128
#define AVG_BATT_DISCHG_CURR_SAMPLES            128   
#define AVG_PANEL_VOLT_SAMPLES                  128
#define AVG_LD_OUT_VOLT_SAMPLES                 8
#define AVG_TEMP_SENSOR                         128

#define CONN_LED_ON                             0x02
#define CONN_LED_INC_BRIGHT                     0x01
#define CONN_LED_OFF                            0x0C
#define CONN_LED_DEC_BRIGHT                     0x0B
      
#define THREE_SL_CONN_SP1ML_UART                USART3      
#define UART_RX_DATABUFF_SIZE                   1000
#define UART_TX_DATABUFF_SIZE                   1000  
   
#define DC_DC_LED_DRIVER_EFF                    0.9
#define MCU_VDD                                 3.3
#define ADC_RESOLUTION                          4096
#define LED_CURRENT_ZERO_OFFSET                 60
#define LED_CURRENT_SENSE_R                     0.075//in ohms
#define DC_DC_SENSE_R1                          15//in kohms
#define DC_DC_SENSE_R2                          1//in kohms
#define BATT_SENSE_R1                           18//in kohms
#define BATT_SENSE_R2                           3.3//in kohms

#define OVER_TEMPERATURE_LIMIT                  80   
   
#define REVERSE_PANEL_CH_TH                     1900// ~ 7V
   
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SystemClock_Config(void);
void system_Init(void);
void system_Monitor(void);
void system_UpdateSysParams(void);
void system_CheckPanelReverse(void);
void system_CalcBattCurrK(void);
uint8_t system_CalcBattDischgCurrent(void);
void system_PanelDisable(void);
void system_PanelEnable(void);
void system_DisableCharging(void);
void system_EnableCharging(void);
uint16_t system_GetSccPwm(void);
void system_SetSccPwm(uint16_t);
int16_t system_GetTemperature(void);
bool system_GetIgnoreLedCurrent(void);
void system_SetIgnoreLedCurrent(bool);
void system_DcDcLdSet(void);
void system_DcDcLdDisable(void);
void system_DcDcLdEnable(void);
void system_DcDcLdOutDisable(void);
void system_DcDcLdOutEnable(void);
void system_DcDcStartRegulate(void);
uint32_t system_DcDcLdGetDuty(void);
void system_DcDcLdSetDuty(uint32_t);
void system_AcDcLdDisable(void);
void system_AcDcLdEnable(void);
void system_AcDcLdOutDisable(void);
void system_AcDcLdOutEnable(void);
void system_AcDcSetDim(uint8_t);
bool system_GetServerLightCommand(void);
void system_SetServerLightCommand(bool);
bool system_GetServerLightDimCommand(void);
void system_SetServerLightDimCommand(bool);
void system_ResetConn(void);
void system_ResetModuleConn(void);
uint32_t system_GetUartWaitTick(void);
void system_ResetUartTick(void);
bool system_TransmitUartData(uint8_t);
bool system_GetReadDataPktFlag(void);
void system_SetReadDataPktFlag(bool);
bool system_GetBattTick_10(void);
void system_SetBattTick_10(bool);
bool system_GetBattTick_1000s(void);
void system_SetBattTick_1000s(bool);
bool system_GetTim3ACCCr_val(void);//Added by sunil

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
