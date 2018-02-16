/**
  ******************************************************************************
  * File Name          : battery.c
  * Description        : Functions related to battery handling
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
#include "battery.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  BATTERY_ENABLE,
  BATTERY_DISABLE
} BatteryEnabeTypeDef;

typedef enum
{
  BATTERY_PRESENT,
  BATTERY_NOT_PRESENT
} BatteryPresentTypeDef;

typedef enum
{
  BATTERY_OK,
  NATTERY_NOT_OK
} BatteryStatusTypeDef;

typedef enum
{
  BATTERY_CHARGED,
  BATTERY_DISCHARGED,
  BATTERY_DEEP_DISCHARGED
} BatteryChargedTypeDef;

typedef enum
{
  BATTERY_CHARGING,
  BATTERY_DISCHARGING,
  BATTERY_IDLE
} BatteryChargingTypeDef;


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t StateFlags_Battery = 0x01;

uint16_t BattMinVoltage = BATTERY_MINIMUM_VOLTAGE;
uint16_t BattMinDischrgVoltage = BATTERY_MIN_DISCHRG_VOLT;
uint16_t BattFltVoltage = BATTERY_FLOAT_VOLTAGE;
uint16_t BattFltVoltage_TC;//Temperature compensated float voltage
uint16_t BattMaxVoltage = BATTERY_MAXIMUM_VOLTAGE;
uint16_t BattMaxVoltage_TC;//Temperature compensated maximum voltage
uint16_t BattFltCurrent = BATTERY_FLOAT_CURRENT_TH;
uint16_t BattMaxChargingCurrent = BATTERY_MAXIMUM_CHARGING_CURRENT;
uint16_t BattDeepDischrgTh = BATTERY_DEEP_DISCHRG_THRESHOLD;

uint16_t BattChargingCurrent = 0;
uint16_t BattDischargingCurrent = 0;
uint16_t BattVoltage = 0;


/* Private function prototypes -----------------------------------------------*/
void battery_Control(BatteryEnabeTypeDef);
BatteryPresentTypeDef battery_Is_Present(void);
BatteryStatusTypeDef battery_Is_Ok(void);
BatteryChargedTypeDef battery_Is_Charged(void);
BatteryChargingTypeDef battery_Is_Charging(void);

/* Exported functions --------------------------------------------------------*/


/**
* @brief This function return battery voltage
*/
uint16_t battery_GetVoltage(void)
{
  return BattVoltage;
}

/**
* @brief This function updates battery voltage
                       calculates temperature compensated battery float voltage
                       threshold and temperature compensated battery maximum
                       voltage threshold levels
         Input parameters are battery voltage and ambient temperature
*/
void battery_UpdateVoltage(uint32_t* volt,int32_t* tempMcu)
{  
  BattVoltage = (uint16_t)*volt;
  
  /* Applying temperature compensation for maximum charging battery voltage
     and maximum float battery voltage
     Compensation of -30mV/degC/12V for battery maximum voltage with reference
     to 25 degC. It implies -5.769/degC for battery maximum voltage
     Compensation of -18mV/degC/12V for battery float voltage with reference 
     to 25 degC. It implies -3.461/degC for battery float votlage */
  BattMaxVoltage_TC = (uint16_t)((int32_t)BattMaxVoltage + 
                                 ((-5.769)*(*tempMcu - 25)));
    
  BattFltVoltage_TC = (uint16_t)((int32_t)BattFltVoltage + 
                                 ((-3.461)*(*tempMcu - 25)));
  
}

/**
* @brief This function returns battery minimum voltage threshold
*/
uint16_t battery_GetMinVoltage(void)
{
  return BattMinVoltage;
}

/**
* @brief This function return temeprature compensated maximum voltage threshold
*/
uint16_t battery_GetMaxVoltage(void)
{
  return BattMaxVoltage_TC;
}

/**
* @brief This function return battery minimum disharge voltage threshold
*/
uint16_t battery_GetMinDischrgVoltage(void)
{
  return BattMinDischrgVoltage;
}

/**
* @brief This function return temperature compensated float voltage threshold
*/
uint16_t battery_GetFltVoltage(void)
{
  return BattFltVoltage_TC;
}

/**
* @brief This function return battery maximum current threshold
*/
uint16_t battery_GetMaxCurrent(void)
{
  return BattMaxChargingCurrent;
}

/**
* @brief This function return battery charging current
*/
uint16_t battery_GetChargngCurrent(void)
{
  return BattChargingCurrent;
}

/**
* @brief This function return battery discharging current
*/
uint16_t battery_GetDischargngCurrent(void)
{
  return BattDischargingCurrent;
}

/**
* @brief This function update battery charging current
*/
void battery_UpdateChgCurrent(uint32_t* current)
{
  BattChargingCurrent = (uint16_t)*current;
}

/**
* @brief This function return battery float current threshold
*/
uint16_t battery_GetFltCurrent(void)
{
  return BattFltCurrent;
}

/**
* @brief This function set various battery flags depending on input flag
*/
void battery_SetFlag(uint16_t flag)
{
  StateFlags_Battery = StateFlags_Battery | flag;
}

/**
* @brief This function reset various battery flags depending on input flag
*/
void battery_ResetFlag(uint16_t flag)
{
  StateFlags_Battery = StateFlags_Battery & (~flag);
}

/**
* @brief This function return various battery flags depending on input flag
*/
bool battery_GetFlag(uint16_t flag)
{
  if (StateFlags_Battery & flag)
    return true;
  else
    return false;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
