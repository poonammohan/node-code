/**
  ******************************************************************************
  * File Name          : dc_dc_ld.c
  * Description        : Functions related to DC-DC LED Driver
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
#include "dc_dc_ld.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t StateFlags_DcDcLd = 0x00;
uint16_t DcDcLdOutVoltage;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
* @brief This function update DC DC LED driver output voltage
*/
void dc_dc_ld_UpdateOutVoltage(uint32_t* voltage)
{
  DcDcLdOutVoltage = (uint16_t)*voltage;
  
  if (DcDcLdOutVoltage > DC_DC_LD_OUT_VOLT_MAX)
  {
    dc_dc_ld_SetFlag(DC_DC_LD_MASK_OVER_VOLTAGE);
  }
}

/**
* @brief This function return DC DC LED driver output voltage
*/
uint16_t dc_dc_ld_GetOutVoltage(void)
{
  return DcDcLdOutVoltage;
}

/**
* @brief This function set various flags of DC DC LED driver
*/
void dc_dc_ld_SetFlag(uint16_t flag)
{
  StateFlags_DcDcLd = StateFlags_DcDcLd | flag;
}

/**
* @brief This function reset various flags of DC DC LED driver
*/
void dc_dc_ld_ResetFlag(uint16_t flag)
{
  StateFlags_DcDcLd = StateFlags_DcDcLd & (~flag);
}

/**
* @brief This function return status of various flags of DC DC LED driver
*/
bool dc_dc_ld_GetFlag(uint16_t flag)
{
  if (StateFlags_DcDcLd & flag)
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
