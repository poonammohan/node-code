/**
  ******************************************************************************
  * File Name          : mppt.c
  * Description        : Functions related to mppt routine
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
#include "mppt.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  MPPT_INIT,
  MPPT_SCAN,
  MPPT_TRACK,
  MPPT_PRTB_N_OBS,
  TRACK_VOLTAGE
} MpptStageTypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MpptStageTypeDef MpptStage = MPPT_INIT;
MpptDutyChangeTypeDef MpptDutyChange = MPPT_DUTY_NO_CHANGE;
uint16_t MpptBattMaxCurr;
uint16_t MpptBattMaxVolt;
uint16_t MpptPwmDutyMax;
uint16_t MpptPwmDutyCurr;
uint16_t MpptPwmDutyRef;
uint16_t MpptBattCurrArray[1000];
uint8_t MpptSign = MPPT_POSITIVE;
uint16_t VoltageRef;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
* @brief This function runs mppt routine while charging the battery
         It takes target battery voltage, target battery current, existing
         battery voltage, existing battery current and existing PWM of DC DC
         converter used for charging
         It returns the new PWM value of DC DC converter being used for charging
*/
uint16_t mppt_Routine(uint16_t targetVoltage,uint16_t targetCurrent,
                               uint16_t currentVoltage,uint16_t currentCurrent,
                               uint16_t currentPwm)
{
  switch (MpptStage)
  {
    /* Initialize mppt routine */
  case MPPT_INIT:
    MpptBattMaxCurr = 0;
    MpptBattMaxVolt = 0;
    MpptPwmDutyMax = 0;
    MpptPwmDutyCurr = 0;
    MpptStage = MPPT_SCAN;
    break;
    
    /* Scan the whole range for searching peak power point */
  case MPPT_SCAN:
    if (MpptBattMaxCurr < currentCurrent)
    {
      MpptBattMaxCurr = currentCurrent;
      MpptBattMaxVolt = currentVoltage;
      /* Duty cycle corresponding to maximum battery current */
      MpptPwmDutyMax = currentPwm;
    }
    
    /* If duty cycle upper bound is reached, go to mppt track */
    if (MpptPwmDutyCurr <= MPPT_PWM_DUTY_UB)
    {
      MpptPwmDutyCurr++;
    }
    else
    {
      MpptStage = MPPT_TRACK;
    }
    break;
    
    /* Set the converter at maximum battery current duty cycle */
  case MPPT_TRACK:
    if (MpptPwmDutyMax < MpptPwmDutyCurr)
    {
      MpptPwmDutyCurr--;
    }
    else
    {
      MpptPwmDutyRef = MpptPwmDutyMax;
      MpptStage = MPPT_PRTB_N_OBS;
    }
    break;
    
    /* Perturb and observe routine for tracking peak power point
       Depending on the change in battery current in the vicinity of existing
       peak power point, it follows the shifted peak power point */
  case MPPT_PRTB_N_OBS:
      MpptBattCurrArray[MpptPwmDutyCurr] = currentCurrent;
      
      if ((MpptPwmDutyCurr - MpptPwmDutyRef) >= 2)
      {
        if (MpptBattCurrArray[MpptPwmDutyCurr] > MpptBattCurrArray[MpptPwmDutyRef])
        {
          MpptSign = MPPT_POSITIVE;
          MpptPwmDutyRef += 1;
        }
        else
        {
          MpptSign = MPPT_NEGATIVE;
        }
      }
      
      if ((MpptPwmDutyRef - MpptPwmDutyCurr) >= 2)
      {
        if (MpptBattCurrArray[MpptPwmDutyCurr] > MpptBattCurrArray[MpptPwmDutyRef])
        {
          MpptSign = MPPT_NEGATIVE;
          MpptPwmDutyRef -= 1;
        }
        else
        {
          MpptSign = MPPT_POSITIVE;
        }
      }
      
      if (MpptSign == MPPT_POSITIVE)
      {
        if (MpptPwmDutyCurr < MPPT_PWM_DUTY_UB)
        {
          MpptPwmDutyCurr += 1;
        }
        else
        {
          MpptSign = MPPT_NEGATIVE;
          if (MpptPwmDutyCurr > 0)
          {
            MpptPwmDutyCurr -= 1;
          }
        }
      }
      else
      {
        if (MpptPwmDutyCurr > 0)
        {
          MpptPwmDutyCurr -= 1;
        }
        else
        {
          MpptSign = MPPT_POSITIVE;
          if (MpptPwmDutyCurr < MPPT_PWM_DUTY_UB)
          {
            MpptPwmDutyCurr += 1; 
          }
        }
      }
      
      /* if current battery voltage reach target battery voltage, go to track
         voltage case. Perturb and observe is not required in this case */
    if (targetVoltage <= currentVoltage)
    {
      VoltageRef = currentVoltage;
      MpptStage = TRACK_VOLTAGE;
    }
    break;
        
  case TRACK_VOLTAGE:
    /* Tracks battery voltage and adjust converter PWM */
    if ((currentVoltage - targetVoltage) >= 10)
    {
      if (MpptPwmDutyCurr > 0)
        MpptPwmDutyCurr--;
    }
    
    if ((targetVoltage - currentVoltage) >= 10)      
    {
      if (MpptPwmDutyCurr < MPPT_PWM_DUTY_UB)
      {
        MpptPwmDutyCurr++;
      }
    }
    
    /* If target battery voltage is greater than current voltage (plus 
       hysteresis), it implies MPP is required. Initialize mppt routine */
    if (targetVoltage >= (currentVoltage + 300))
    {
      MpptStage = MPPT_INIT;
    }    
    break;
  }
  
  return MpptPwmDutyCurr;
}

/**
* @brief This function reinitialize mppt routine
*/
void mppt_Reinit(void)
{
  MpptStage = MPPT_INIT;
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
